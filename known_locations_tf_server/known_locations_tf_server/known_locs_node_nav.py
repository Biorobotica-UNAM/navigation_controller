#!/usr/bin/env python3
import os
import math
import tempfile
import threading
from copy import deepcopy
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from actionlib_msgs.msg import GoalStatus
import tf_transformations as tf
import tf2_ros
import yaml
import numpy as np

from known_locations_interfaces.srv import LocationsServer, LocationsGet
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

# ----------------- NUEVO: SIEMPRE usar YAML del repo (no ~/.ros, no install) -----------------
def _repo_root_from_this_file(markers=('package.xml', 'setup.py', 'CMakeLists.txt')) -> Path:
    """
    Sube desde este archivo hasta encontrar un archivo marcador típico de raíz de paquete.
    Si no encuentra nada, usa la carpeta del archivo como fallback.
    """
    p = Path(__file__).resolve()
    for parent in (p.parent,) + tuple(p.parents):
        for m in markers:
            if (parent / m).exists():
                return parent
    return p.parent

def _pkg_yaml_abs_path(rel_path: str) -> str:
    """
    Devuelve una ruta ABSOLUTA dentro del repositorio del paquete (no en install/).
    Crea carpetas si hacen falta.
    """
    root = _repo_root_from_this_file()
    path = (root / rel_path.lstrip('/')).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    return str(path)
# ----------------------------------------------------------------------------------------------


class KnownLocationsServer(Node):
    def __init__(self):
        super().__init__('locs_server')

        # Parámetros (mantén tu estructura relativa dentro del repo)
        self.declare_parameter("file_name", "config/known_locations.yaml")
        # 'package_name' ya no se usa para resolver la ruta del YAML, pero lo dejo por compatibilidad
        self.declare_parameter("package_name", "known_locations_tf_server")

        self.file_name = self.get_parameter("file_name").get_parameter_value().string_value
        self.package_name = self.get_parameter("package_name").get_parameter_value().string_value

        # RUTA SIEMPRE EN EL REPO:
        self.yaml_path = _pkg_yaml_abs_path(self.file_name)

        # Si no existe el YAML, inicializa vacío
        if not os.path.exists(self.yaml_path):
            try:
                with open(self.yaml_path, 'w') as f:
                    yaml.safe_dump({}, f, default_flow_style=False)
                self.get_logger().info(f"Inicializado YAML vacío en {self.yaml_path}")
            except Exception as e:
                self.get_logger().warn(f"No se pudo crear YAML: {e}")

        self._cb_group = ReentrantCallbackGroup()

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publisher para goals de navegación
        self.goal_pub = self.create_publisher(PoseStamped, '/nav_control/goal', 10)
        self.sub = self.create_subscription(GoalStatus, '/navigation/status', self.callback_status_nav,10, callback_group=self._cb_group)

        # Servicios
        self.srv_add = self.create_service(LocationsServer, 'known_location_add', self.callback_add)
        self.srv_update = self.create_service(LocationsServer, 'update_yaml_interactive', self.callback_update)  # placeholder
        self.srv_get = self.create_service(LocationsGet, 'known_location_get', self.callback_get)
        self.srv_goal = self.create_service(LocationsServer, 'known_location_goal', self.callback_goal, callback_group=self._cb_group)

        self._cv = threading.Condition()
        self._goal_finished = False
        self._goal_succeeded = False
        self._last_status = 0
        self._last_text = ""

        # Al iniciar: leer YAML, republicar TF estáticas
        self.df = self.yaml_to_df()
        if not self.df.empty:
            for i in range(len(self.df)):
                x, y = float(self.df['x'].iloc[i]), float(self.df['y'].iloc[i])
                qx, qy, qz, qw = [float(v) for v in self.df[['qx','qy','qz','qw']].iloc[i].values]
                child = str(self.df['child_id_frame'].iloc[i])

                t = self.write_tf([x, y, 0.0], [qx, qy, qz, qw], child)
                self.tf_static_broadcaster.sendTransform(t)
                rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(f"Known locations server listo. YAML en: {self.yaml_path}")

    # ---------- Utilidades TF ----------
    def write_tf(self, pose_xyz, quat_xyzw, child_frame, parent_frame='map'):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(pose_xyz[0])
        t.transform.translation.y = float(pose_xyz[1])
        t.transform.translation.z = float(pose_xyz[2])
        t.transform.rotation.x = float(quat_xyzw[0])
        t.transform.rotation.y = float(quat_xyzw[1])
        t.transform.rotation.z = float(quat_xyzw[2])
        t.transform.rotation.w = float(quat_xyzw[3])
        return t

    def read_tf(self, t: TransformStamped):
        pose = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z], dtype=float)
        quat = np.array([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w], dtype=float)
        return pose, quat

    # ---------- YAML ----------
    def read_yaml(self):
        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                return data or {}
        except FileNotFoundError:
            return {}
        except Exception as e:
            self.get_logger().error(f"YAML read error: {e}")
            return {}

    def write_yaml_locs(self, trans: TransformStamped, name: str):
        pose, quat = self.read_tf(trans)

        con = self.read_yaml()
        if con is None:
            con = {}

        con[name] = {
            'x': round(float(pose[0]), 3),
            'y': round(float(pose[1]), 3),
            'theta': round(float(tf.euler_from_quaternion(quat)[2]), 3),
            'qx': round(float(quat[0]), 3),
            'qy': round(float(quat[1]), 3),
            'qz': round(float(quat[2]), 3),
            'qw': round(float(quat[3]), 3)
        }

        # Escritura atómica
        os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(self.yaml_path), prefix='.tmp_yaml_')
        try:
            with os.fdopen(fd, 'w') as f:
                yaml.safe_dump(con, f, default_flow_style=False)
            os.replace(tmp, self.yaml_path)
        finally:
            if os.path.exists(tmp):
                try:
                    os.remove(tmp)
                except OSError:
                    pass
        return True

    def yaml_to_dict(self):
        con = self.read_yaml()
        if not con:
            return {}

        # Devuelve el diccionario tal como está en el YAML
        return con
    
    def callback_status_nav(self, msg: GoalStatus):
        self.get_logger().info(f"[STATUS] code={msg.status} text='{msg.text}'")
        with self._cv:
            self._last_status = msg.status
            self._last_text = msg.text

            if msg.status == 3:
                self._goal_finished = True
                self._goal_succeeded = True
                self._cv.notify_all()
            elif msg.status == 4:
                self._goal_finished = True
                self._goal_succeeded = False
                self._cv.notify_all()
        

    # ---------- Servicios ----------
    def callback_add(self, req, _resp=None):
        resp = LocationsServer.Response()
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            child_name = req.location_name.strip()
            if not child_name:
                raise RuntimeError("Nombre de locación vacío")

            t_pub = deepcopy(trans)
            t_pub.child_frame_id = child_name
            self.tf_static_broadcaster.sendTransform(t_pub)

            succ = self.write_yaml_locs(t_pub, child_name)
            resp.success = bool(succ)
            self.get_logger().info(f"Locación '{child_name}' guardada en YAML del paquete y TF estática publicada.")
        except Exception as e:
            self.get_logger().error(f"No TF map<-base_link: {e}")
            resp.success = False
        return resp

    def callback_update(self, req, _resp=None):
        resp = LocationsServer.Response()
        resp.success = True
        return resp

    def _lookup_pose_by_name(self, name: str):
        yaml_data = self.yaml_to_dict()
        if name not in yaml_data:
            return None, False

        loc = yaml_data[name]
        pose = Pose()
        pose.position.x = loc['x']
        pose.position.y = loc['y']
        pose.position.z = 0.0
        pose.orientation.x = loc['qx']
        pose.orientation.y = loc['qy']
        pose.orientation.z = loc['qz']
        pose.orientation.w = loc['qw']
        return pose, True


    def callback_get(self, req, _resp=None):
        resp = LocationsGet.Response()
        try:
            pose, ok = self._lookup_pose_by_name(req.location_name)
            if not ok:
                resp.success = False
                resp.message = f"Location '{req.location_name}' not found"
                return resp
            resp.pose = pose
            resp.success = True
            resp.message = "OK"
        except Exception as e:
            self.get_logger().error(f"Error leyendo YAML: {e}")
            resp.success = False
            resp.message = "Error reading YAML"
        return resp

    def callback_goal(self, req, _resp=None):
        resp = LocationsServer.Response()
        try:
            pose, ok = self._lookup_pose_by_name(req.location_name)
            if not ok:
                resp.success = False
                return resp

            with self._cv:
                self._goal_finished = False
                self._goal_succeeded = False
                self._last_status = 0
                self._last_text = ""

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose = pose

            self.goal_pub.publish(msg)
            self.get_logger().info(f"Goal publicado a '{req.location_name}'")

            timeout_sec = 120.0  # por ejemplo 2 minutos
            start_time = self.get_clock().now()

            with self._cv:
                while not self._goal_finished:
                    now = self.get_clock().now()
                    elapsed = (now - start_time).nanoseconds / 1e9
                    remaining = timeout_sec - elapsed

                    if remaining <= 0.0:
                        # Timeout: nadie marcó _goal_finished a True
                        self.get_logger().warn("Timeout esperando GoalStatus de navegación")
                        resp.success = False
                        return resp
                    
                    self.get_logger().info("Before wait (cv)")
                    # Espera (libera el lock mientras tanto)
                    self._cv.wait(timeout=remaining)

                # Si salimos del while es porque _goal_finished == True
                if self._goal_succeeded:
                    resp.success = True
                    self.get_logger().info("Navegación completada correctamente")
                else:
                    resp.success = False
                    self.get_logger().warn(
                        f"Navegación falló. status={self._last_status}, text='{self._last_text}'"
                    )

        except Exception as e:
            self.get_logger().error(f"No se pudo publicar goal: {e}")
            resp.success = False
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = KnownLocationsServer()

    # === USAR MULTITHREADED EXECUTOR ===
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()   # <-- NO usar rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
