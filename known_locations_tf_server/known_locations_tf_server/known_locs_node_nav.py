#!/usr/bin/env python3
import os
import math
import tempfile
from copy import deepcopy

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
import tf_transformations as tf
import tf2_ros
import yaml
import numpy as np
import pandas as pd

# Servicios (tú ya los tienes en tu paquete de interfaces)
from known_locations_interfaces.srv import LocationsServer, LocationsGet
# LocationsServer.srv (asumido):
# string location_name
# ---
# bool success
#
# LocationsGet.srv (asumido):
# string location_name
# ---
# bool success
# string message
# geometry_msgs/Pose pose


def _resolve_repo_share(pkg_name: str) -> str:
    """
    Devuelve la carpeta share real (resuelta) del paquete.
    Si se construyó con --symlink-install, esto apunta al árbol del repo (no a una copia).
    """
    share = get_package_share_directory(pkg_name)
    try:
        return os.path.realpath(share)
    except Exception:
        return share


def _desired_yaml_path(pkg_name: str, rel_path: str) -> str:
    """
    Si el paquete está symlinked (desarrollo), escribe en el repo.
    Si el paquete es una instalación real (no symlink), escribe en ~/.ros/<pkg>/<basename>.
    """
    share_real = _resolve_repo_share(pkg_name)
    rel_norm = rel_path.lstrip('/')  # asegurar relativo
    candidate = os.path.join(share_real, rel_norm)

    # Heurística simple: si la ruta real del share NO contiene 'install', asumimos repo editable
    parts = os.path.normpath(share_real).split(os.sep)
    if 'install' not in parts:
        os.makedirs(os.path.dirname(candidate), exist_ok=True)
        return candidate

    # Instalación real -> no escribir dentro de install; usar ~/.ros
    home_yaml = os.path.join(
        os.path.expanduser('~/.ros'),
        pkg_name,
        os.path.basename(rel_norm),
    )
    os.makedirs(os.path.dirname(home_yaml), exist_ok=True)
    return home_yaml


class KnownLocationsServer(Node):
    def __init__(self):
        super().__init__('locs_server')

        # Parámetros
        self.declare_parameter("file_name", "config/known_locations.yaml")   # relativo; conserva tu estructura
        self.declare_parameter("package_name", "known_locations_tf_server")  # cámbialo si tu paquete se llama distinto

        self.file_name = self.get_parameter("file_name").get_parameter_value().string_value
        self.package_name = self.get_parameter("package_name").get_parameter_value().string_value

        # Rutas (repo si hay symlink, ~/.ros si es instalación real)
        self.yaml_path = _desired_yaml_path(self.package_name, self.file_name)
        self.pkg_share_for_read = _resolve_repo_share(self.package_name)

        # Si no existe YAML destino pero sí existe el de plantilla en el paquete, inicializa con él
        default_yaml = os.path.join(self.pkg_share_for_read, self.file_name.lstrip('/'))
        if (not os.path.exists(self.yaml_path)) and os.path.exists(default_yaml):
            try:
                os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)
                with open(default_yaml, 'rb') as src, open(self.yaml_path, 'wb') as dst:
                    dst.write(src.read())
            except Exception as e:
                self.get_logger().warn(f"No se pudo inicializar YAML desde plantilla: {e}")

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publisher para goals de navegación
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        # Servicios
        self.srv_add = self.create_service(LocationsServer, 'known_location_add', self.callback_add)
        self.srv_update = self.create_service(LocationsServer, 'update_yaml_interactive', self.callback_update)  # placeholder
        self.srv_get = self.create_service(LocationsGet, 'known_location_get', self.callback_get)
        self.srv_goal = self.create_service(LocationsServer, 'known_location_goal', self.callback_goal)

        # Al iniciar: leer YAML, republicar TF estáticas (no IMs)
        self.df = self.yaml_to_df()
        if not self.df.empty:
            for i in range(len(self.df)):
                # trans = [x, y, z(=0)]
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

        # Mantener tu estructura: lista de 7 diccionarios por clave
        data = [
            dict(x=round(float(pose[0]), 3)),
            dict(y=round(float(pose[1]), 3)),
            dict(theta=round(float(tf.euler_from_quaternion(quat)[2]), 3)),
            dict(qx=round(float(quat[0]), 3)),
            dict(qy=round(float(quat[1]), 3)),
            dict(qz=round(float(quat[2]), 3)),
            dict(qw=round(float(quat[3]), 3)),
        ]
        con[name] = data

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

    def yaml_to_df(self):
        con = self.read_yaml()
        if not con:
            return pd.DataFrame()

        values = []
        locations = []
        for c in con:
            locations.append(c)
            # c -> lista de 7 dicts
            for i in range(len(con[c])):
                # Tomar el único valor de cada dict
                values.append(list(con[c][i].values())[0])

        if len(values) == 0:
            return pd.DataFrame()

        data = np.array(values, dtype=float).reshape((int(len(values) / 7), 7))
        df = pd.DataFrame(data, columns=['x', 'y', 'th', 'qx', 'qy', 'qz', 'qw'])
        df['child_id_frame'] = locations
        return df

    # ---------- Servicios ----------
    def callback_add(self, req, _resp=None):
        """ known_location_add: agrega locación actual (map<-base_link) con el nombre dado. """
        resp = LocationsServer.Response()
        try:
            # TF en ahora: map <- base_link
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            # republicar como estática con el nombre dado
            child_name = req.location_name.strip()
            if not child_name:
                raise RuntimeError("Nombre de locación vacío")

            t_pub = deepcopy(trans)
            t_pub.child_frame_id = child_name
            self.tf_static_broadcaster.sendTransform(t_pub)

            succ = self.write_yaml_locs(t_pub, child_name)
            resp.success = bool(succ)
            self.get_logger().info(f"Locación '{child_name}' guardada en YAML y TF estática publicada.")
        except Exception as e:
            self.get_logger().error(f"No TF map<-base_link: {e}")
            resp.success = False
        return resp

    def callback_update(self, req, _resp=None):
        """ update_yaml_interactive: placeholder (compatibilidad). """
        resp = LocationsServer.Response()
        resp.success = True
        return resp

    def _lookup_pose_by_name(self, name: str):
        """ Devuelve (Pose, True/False) desde YAML por nombre. """
        df = self.yaml_to_df()
        if df.empty:
            return None, False
        loc = df[df['child_id_frame'] == name]
        if loc.empty:
            return None, False

        pose = Pose()
        pose.position.x = float(loc['x'].iloc[0])
        pose.position.y = float(loc['y'].iloc[0])
        pose.position.z = 0.0
        pose.orientation.x = float(loc['qx'].iloc[0])
        pose.orientation.y = float(loc['qy'].iloc[0])
        pose.orientation.z = float(loc['qz'].iloc[0])
        pose.orientation.w = float(loc['qw'].iloc[0])
        return pose, True

    def callback_get(self, req, _resp=None):
        """ known_location_get: regresa la Pose de la locación. """
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
        """ known_location_goal: publica PoseStamped en /move_base_simple/goal para la locación. """
        resp = LocationsServer.Response()
        try:
            pose, ok = self._lookup_pose_by_name(req.location_name)
            if not ok:
                resp.success = False
                return resp

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose = pose

            self.goal_pub.publish(msg)
            resp.success = True
            self.get_logger().info(f"Goal publicado a '{req.location_name}' en /move_base_simple/goal")
        except Exception as e:
            self.get_logger().error(f"No se pudo publicar goal: {e}")
            resp.success = False
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = KnownLocationsServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import os
import math
import tempfile
from copy import deepcopy

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
import tf_transformations as tf
import tf2_ros
import yaml
import numpy as np
import pandas as pd

# Servicios (tú ya los tienes en tu paquete de interfaces)
from known_locations_interfaces.srv import LocationsServer, LocationsGet
# LocationsServer.srv (asumido):
# string location_name
# ---
# bool success
#
# LocationsGet.srv (asumido):
# string location_name
# ---
# bool success
# string message
# geometry_msgs/Pose pose


def _resolve_repo_share(pkg_name: str) -> str:
    """
    Devuelve la carpeta share real (resuelta) del paquete.
    Si se construyó con --symlink-install, esto apunta al árbol del repo (no a una copia).
    """
    share = get_package_share_directory(pkg_name)
    try:
        return os.path.realpath(share)
    except Exception:
        return share


def _desired_yaml_path(pkg_name: str, rel_path: str) -> str:
    """
    Si el paquete está symlinked (desarrollo), escribe en el repo.
    Si el paquete es una instalación real (no symlink), escribe en ~/.ros/<pkg>/<basename>.
    """
    share_real = _resolve_repo_share(pkg_name)
    rel_norm = rel_path.lstrip('/')  # asegurar relativo
    candidate = os.path.join(share_real, rel_norm)

    # Heurística simple: si la ruta real del share NO contiene 'install', asumimos repo editable
    parts = os.path.normpath(share_real).split(os.sep)
    if 'install' not in parts:
        os.makedirs(os.path.dirname(candidate), exist_ok=True)
        return candidate

    # Instalación real -> no escribir dentro de install; usar ~/.ros
    home_yaml = os.path.join(
        os.path.expanduser('~/.ros'),
        pkg_name,
        os.path.basename(rel_norm),
    )
    os.makedirs(os.path.dirname(home_yaml), exist_ok=True)
    return home_yaml


class KnownLocationsServer(Node):
    def __init__(self):
        super().__init__('locs_server')

        # Parámetros
        self.declare_parameter("file_name", "config/known_locations.yaml")   # relativo; conserva tu estructura
        self.declare_parameter("package_name", "known_locations_tf_server")  # cámbialo si tu paquete se llama distinto

        self.file_name = self.get_parameter("file_name").get_parameter_value().string_value
        self.package_name = self.get_parameter("package_name").get_parameter_value().string_value

        # Rutas (repo si hay symlink, ~/.ros si es instalación real)
        self.yaml_path = _desired_yaml_path(self.package_name, self.file_name)
        self.pkg_share_for_read = _resolve_repo_share(self.package_name)

        # Si no existe YAML destino pero sí existe el de plantilla en el paquete, inicializa con él
        default_yaml = os.path.join(self.pkg_share_for_read, self.file_name.lstrip('/'))
        if (not os.path.exists(self.yaml_path)) and os.path.exists(default_yaml):
            try:
                os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)
                with open(default_yaml, 'rb') as src, open(self.yaml_path, 'wb') as dst:
                    dst.write(src.read())
            except Exception as e:
                self.get_logger().warn(f"No se pudo inicializar YAML desde plantilla: {e}")

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publisher para goals de navegación
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        # Servicios
        self.srv_add = self.create_service(LocationsServer, 'known_location_add', self.callback_add)
        self.srv_update = self.create_service(LocationsServer, 'update_yaml_interactive', self.callback_update)  # placeholder
        self.srv_get = self.create_service(LocationsGet, 'known_location_get', self.callback_get)
        self.srv_goal = self.create_service(LocationsServer, 'known_location_goal', self.callback_goal)

        # Al iniciar: leer YAML, republicar TF estáticas (no IMs)
        self.df = self.yaml_to_df()
        if not self.df.empty:
            for i in range(len(self.df)):
                # trans = [x, y, z(=0)]
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

        # Mantener tu estructura: lista de 7 diccionarios por clave
        data = [
            dict(x=round(float(pose[0]), 3)),
            dict(y=round(float(pose[1]), 3)),
            dict(theta=round(float(tf.euler_from_quaternion(quat)[2]), 3)),
            dict(qx=round(float(quat[0]), 3)),
            dict(qy=round(float(quat[1]), 3)),
            dict(qz=round(float(quat[2]), 3)),
            dict(qw=round(float(quat[3]), 3)),
        ]
        con[name] = data

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

    def yaml_to_df(self):
        con = self.read_yaml()
        if not con:
            return pd.DataFrame()

        values = []
        locations = []
        for c in con:
            locations.append(c)
            # c -> lista de 7 dicts
            for i in range(len(con[c])):
                # Tomar el único valor de cada dict
                values.append(list(con[c][i].values())[0])

        if len(values) == 0:
            return pd.DataFrame()

        data = np.array(values, dtype=float).reshape((int(len(values) / 7), 7))
        df = pd.DataFrame(data, columns=['x', 'y', 'th', 'qx', 'qy', 'qz', 'qw'])
        df['child_id_frame'] = locations
        return df

    # ---------- Servicios ----------
    def callback_add(self, req, _resp=None):
        """ known_location_add: agrega locación actual (map<-base_link) con el nombre dado. """
        resp = LocationsServer.Response()
        try:
            # TF en ahora: map <- base_link
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            # republicar como estática con el nombre dado
            child_name = req.location_name.strip()
            if not child_name:
                raise RuntimeError("Nombre de locación vacío")

            t_pub = deepcopy(trans)
            t_pub.child_frame_id = child_name
            self.tf_static_broadcaster.sendTransform(t_pub)

            succ = self.write_yaml_locs(t_pub, child_name)
            resp.success = bool(succ)
            self.get_logger().info(f"Locación '{child_name}' guardada en YAML y TF estática publicada.")
        except Exception as e:
            self.get_logger().error(f"No TF map<-base_link: {e}")
            resp.success = False
        return resp

    def callback_update(self, req, _resp=None):
        """ update_yaml_interactive: placeholder (compatibilidad). """
        resp = LocationsServer.Response()
        resp.success = True
        return resp

    def _lookup_pose_by_name(self, name: str):
        """ Devuelve (Pose, True/False) desde YAML por nombre. """
        df = self.yaml_to_df()
        if df.empty:
            return None, False
        loc = df[df['child_id_frame'] == name]
        if loc.empty:
            return None, False

        pose = Pose()
        pose.position.x = float(loc['x'].iloc[0])
        pose.position.y = float(loc['y'].iloc[0])
        pose.position.z = 0.0
        pose.orientation.x = float(loc['qx'].iloc[0])
        pose.orientation.y = float(loc['qy'].iloc[0])
        pose.orientation.z = float(loc['qz'].iloc[0])
        pose.orientation.w = float(loc['qw'].iloc[0])
        return pose, True

    def callback_get(self, req, _resp=None):
        """ known_location_get: regresa la Pose de la locación. """
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
        """ known_location_goal: publica PoseStamped en /move_base_simple/goal para la locación. """
        resp = LocationsServer.Response()
        try:
            pose, ok = self._lookup_pose_by_name(req.location_name)
            if not ok:
                resp.success = False
                return resp

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose = pose

            self.goal_pub.publish(msg)
            resp.success = True
            self.get_logger().info(f"Goal publicado a '{req.location_name}' en /move_base_simple/goal")
        except Exception as e:
            self.get_logger().error(f"No se pudo publicar goal: {e}")
            resp.success = False
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = KnownLocationsServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
