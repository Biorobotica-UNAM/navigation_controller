#!/usr/bin/env python3

import os
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node

import numpy as np

if not hasattr(np, "maximum_sctype"):
    def maximum_sctype(t):
        return np.float64
    np.maximum_sctype = maximum_sctype

if not hasattr(np, "float"):
    np.float = float

import tf_transformations as tf
import tf2_ros

from geometry_msgs.msg import TransformStamped, Pose

from known_locations_interfaces.srv import LocationsServer, LocationsGet, SavePoseLocation

# LocationsServer.srv:
# string location_name
# ---
# bool success
#
# LocationsGet.srv:
# string location_name
# ---
# bool success
# string message
# geometry_msgs/Pose pose


# ================= Helpers =================

def _repo_root_from_this_file(
    markers=('package.xml', 'setup.py', 'CMakeLists.txt')
) -> Path:
    """
    Sube desde este archivo hasta encontrar un archivo típico de raíz del paquete.
    """
    p = Path(__file__).resolve()
    for parent in (p.parent,) + tuple(p.parents):
        for m in markers:
            if (parent / m).exists():
                return parent
    return p.parent


def _pkg_yaml_abs_path(rel_path: str) -> str:
    """
    Devuelve ruta ABSOLUTA dentro del repo del paquete (no install).
    """
    root = _repo_root_from_this_file()
    path = (root / rel_path.lstrip('/')).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    return str(path)


# ================= Node =================

class LocationsManagerServer(Node):

    def __init__(self):
        super().__init__('locations_manager_server')

        self.declare_parameter('known_locations_file', 'known_locations.yaml')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.known_locations_file_name = (
            self.get_parameter('known_locations_file')
            .get_parameter_value().string_value
        )
        self.map_frame = (
            self.get_parameter('map_frame')
            .get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter('base_frame')
            .get_parameter_value().string_value
        )

        self.known_locations = {}
        self.load_known_locations(self.known_locations_file_name)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        self.ready_to_publish_tf = False
        self.wait_to_publish(timeout_sec=2.0)
        
        self.publish_all_known_locations()

        # Services
        self.srv_add_known_loc = self.create_service(
            LocationsServer,
            'locations_manager/add',
            self.handler_add_known_loc
        )

        self.srv_get_known_loc = self.create_service(
            LocationsGet,
            'locations_manager/get',
            self.handler_get_known_loc
        )


        self.srv_save_new_loc = self.create_service(
            SavePoseLocation,
            'locations_manager/save',
            self.handler_save_new_loc
        )


        self.get_logger().info("Known locations server ready.")


    # ================= YAML =================

    def load_known_locations(self, file_name):
        yaml_path = _pkg_yaml_abs_path(f'config/{file_name}')

        if not os.path.exists(yaml_path):
            self.initialize_yaml(yaml_path)

        with open(yaml_path, 'r') as f:
            self.known_locations = yaml.safe_load(f) or {}

        self.yaml_path = yaml_path


    def initialize_yaml(self, file_path):
        default_locations = {
            'origin': {
                'x': 0.0,
                'y': 0.0,
                'theta': 0.0,
                'qx': 0.0,
                'qy': 0.0,
                'qz': 0.0,
                'qw': 1.0
            }
        }

        with open(file_path, 'w') as f:
            yaml.dump(default_locations, f)

        self.get_logger().info(
            f"Initialized default known locations at {file_path}"
        )


    def append_known_loc_to_yaml(self):
        with open(self.yaml_path, 'w') as f:
            yaml.dump(self.known_locations, f)


    # ================= TF =================

    def wait_to_publish(self, timeout_sec=2.0):
        start = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=timeout_sec)

        while self.get_clock().now() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.ready_to_publish_tf = True


    def publish_all_known_locations(self):
        for name, pose_dict in self.known_locations.items():
            self.publish_single_known_location(name, pose_dict)


    def publish_single_known_location(self, name, pose_dict):
        if not self.ready_to_publish_tf:
            self.get_logger().warn(f"TF not ready yet for location '{name}'")
            return

        quat = tf.quaternion_from_euler(
                0.0, 0.0, pose_dict['theta']
            )

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.map_frame
        ts.child_frame_id = name

        ts.transform.translation.x = pose_dict['x']
        ts.transform.translation.y = pose_dict['y']
        ts.transform.translation.z = 0.0

        ts.transform.rotation.x = quat[0]
        ts.transform.rotation.y = quat[1]
        ts.transform.rotation.z = quat[2]
        ts.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(ts)

    def get_robot_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            return trans.transform

        except Exception as e:
            self.get_logger().error(
                f"Failed to get robot pose: {e}"
            )
            return None


    # ================= Services =================

    def handler_add_known_loc(self, req, _resp=None):
        resp = LocationsServer.Response()
        resp.success = False

        name = req.location_name

        if not name:
            self.get_logger().warn("Location name is empty.")
            return resp
        
        self.get_logger().info(f"Calling save: {name}")
        if self.save_location_from_buffer(name):
            resp.success = True
            self.get_logger().info(f"Added location from buffer: {name}")
            return resp

        pose = self.get_robot_current_pose()
        if pose is None:
            return resp
    
        theta = tf.euler_from_quaternion([
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z,
            pose.rotation.w
        ])[2]

        self.known_locations[name] = {
            'x': pose.translation.x,
            'y': pose.translation.y,
            'theta': theta,
            'qx': pose.rotation.x,
            'qy': pose.rotation.y,
            'qz': pose.rotation.z,
            'qw': pose.rotation.w
        }

        self.append_known_loc_to_yaml()
        self.publish_single_known_location(name, self.known_locations[name])

        resp.success = True
        self.get_logger().info(f"Added location: {name}")
        return resp


    def handler_get_known_loc(self, req, _resp=None):
        resp = LocationsGet.Response()
        resp.success = False

        name = req.location_name

        if name not in self.known_locations:
            resp.message = f"Location '{name}' not found."
            self.get_logger().warn(resp.message)
            return resp

        loc = self.known_locations[name]

        resp.pose = Pose()
        resp.pose.position.x = loc['x']
        resp.pose.position.y = loc['y']
        resp.pose.position.z = loc['theta']

        resp.pose.orientation.x = loc['qx']
        resp.pose.orientation.y = loc['qy']
        resp.pose.orientation.z = loc['qz']
        resp.pose.orientation.w = loc['qw']

        resp.success = True
        resp.message = "OK"

        self.get_logger().info(f"Returned location: {name}")
        return resp


    def handler_save_new_loc(self, req, _resp=None):
        resp = SavePoseLocation.Response()
        resp.success = False

        name = req.location_name
        pose = req.pose

        if not name:
            self.get_logger().warn("Location name is empty.")
            return resp

        theta = tf.euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])[2]

        self.known_locations[name] = {
            'x': pose.position.x,
            'y': pose.position.y,
            'theta': theta,
            'qx': pose.orientation.x,
            'qy': pose.orientation.y,
            'qz': pose.orientation.z,
            'qw': pose.orientation.w
        }

        self.append_known_loc_to_yaml()
        self.publish_single_known_location(name, self.known_locations[name])

        resp.success = True
        self.get_logger().info(f"Saved new location: {name}")
        return resp
    
    def save_location_from_buffer(self, name):
            """
            Busca una transformación (TF) en el buffer usando el nombre proporcionado,
            extrae su posición/orientación respecto al marco del mapa y la guarda en el YAML.
            """
            if not name:
                self.get_logger().warn("save_location_from_buffer: El nombre recibido está vacío.")
                return False

            try:
                # Buscamos la transformación del frame 'name' (ej. "customer") respecto a 'map'
                # rclpy.time.Time() nos da la transformación más reciente disponible
                trans = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    name,
                    rclpy.time.Time()
                )
                
                # Extraemos de forma cómoda la traslación y rotación obtenidas
                translation = trans.transform.translation
                rotation = trans.transform.rotation

                # Calculamos theta (Yaw) a partir del cuaternión para mantener tu estructura de datos
                theta = tf.euler_from_quaternion([
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w
                ])[2]

                # Guardamos la localización en el diccionario usando el mismo formato del YAML
                self.known_locations[name] = {
                    'x': translation.x,
                    'y': translation.y,
                    'theta': theta,
                    'qx': rotation.x,
                    'qy': rotation.y,
                    'qz': rotation.z,
                    'qw': rotation.w
                }

                # Sincronizamos los cambios escribiendo en el archivo físico .yaml
                self.append_known_loc_to_yaml()

                # Volvemos a publicar la localización como estática (actualiza si ya existía)
                self.publish_single_known_location(name, self.known_locations[name])

                self.get_logger().info(f"Localización '{name}' extraída de TF y guardada con éxito en el YAML.")
                return True

            except tf2_ros.LookupException:
                self.get_logger().error(f"No se pudo guardar '{name}': El frame no existe en el árbol de TF.")
            except tf2_ros.ConnectivityException:
                self.get_logger().error(f"No se pudo guardar '{name}': Error de conectividad en el árbol de TF.")
            except tf2_ros.ExtrapolationException:
                self.get_logger().error(f"No se pudo guardar '{name}': Error de extrapolación temporal en el buffer.")
            except Exception as e:
                self.get_logger().error(f"Error inesperado al intentar guardar desde el buffer: {e}")
                
            return False



# ================= Main =================

def main(args=None):
    rclpy.init(args=args)

    node = LocationsManagerServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
