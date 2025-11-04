#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from geometry_msgs.msg import TransformStamped
import tf_transformations as tf
import tf2_ros
import yaml
import math
import numpy as np
import pandas as pd
from copy import deepcopy
from known_locations_interfaces.srv import LocationsServer

class KnownLocationsServer(Node):
    def __init__(self):
        super().__init__('locs_server')

        self.declare_parameter("file_name", "/config/known_locations.yaml")
        self.file_name = self.get_parameter("file_name").get_parameter_value().string_value

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.im_server = InteractiveMarkerServer(self, "simple_marker")

        # Service 
        self.srv_add = self.create_service(LocationsServer, 'known_location_add', self.callback_add)
        self.srv_update = self.create_service(LocationsServer, 'update_yaml_interactive', self.callback_update)

 
        self.df = self.yaml_to_df()
        if not self.df.empty:
            for i in range(len(self.df)):
                trans = self.df[['x','y','th']].iloc[i].values
                quat = self.df[['qx','qy','qz','qw']].iloc[i].values
                trans[-1] = 0
                t = self.write_tf(trans, quat, self.df['child_id_frame'].iloc[i])
                self.tf_static_broadcaster.sendTransform(t)
                rclpy.spin_once(self, timeout_sec=0.1)
                self.write_im(trans[0], trans[1], tf.euler_from_quaternion(quat)[2], self.df['child_id_frame'].iloc[i])

        self.get_logger().info(f"Known locations server available using {self.file_name}")


    def write_tf(self, pose, quat, child_frame, parent_frame='map'):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = pose[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        return t

    def read_tf(self, t):
        pose = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        quat = np.array([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
        return pose, quat

    def write_yaml_locs(self, trans, name):
        trans, quat = self.read_tf(trans)
        file_path = get_package_share_directory('known_locations_tf_server') + self.file_name

        con = self.read_yaml()
        if con is None:
            con = {}

        data = [dict(x=round(trans[0],3)), dict(y=round(trans[1],3)), dict(theta=round(tf.euler_from_quaternion(quat)[2],3)),
                dict(qx=round(quat[0],3)), dict(qy=round(quat[1],3)), dict(qz=round(quat[2],3)), dict(qw=round(quat[3],3))]
        con[name] = data

        with open(file_path, 'w') as file:
            yaml.dump(con, file, default_flow_style=False)
        return True

    def read_yaml(self):
        try:
            file_path = get_package_share_directory('known_locations_tf_server') + self.file_name
            with open(file_path, 'r') as file:
                return yaml.safe_load(file)
        except Exception:
            return {}

    def yaml_to_df(self):
        con = self.read_yaml()
        if not con:
            return pd.DataFrame()
        values = []
        locations = []
        for c in con:
            locations.append(c)
            for i in range(len(con[c])):
                values.append(list(con[c][i].values())[0])
        data = np.array(values).reshape((int(len(values)/7), 7))
        df = pd.DataFrame(data)
        df.columns=['x','y','th','qx','qy','qz','qw']
        df['child_id_frame'] = locations
        return df

    def write_im(self, x=0, y=0, yaw=0, name=''):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = name
        int_marker.description = "Known Locs"
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.position.z = 0.1
        q = tf.quaternion_from_euler(0,0,yaw)
        int_marker.pose.orientation.x = q[0]
        int_marker.pose.orientation.y = q[1]
        int_marker.pose.orientation.z = q[2]
        int_marker.pose.orientation.w = q[3]

        box_marker = Marker()
        box_marker.type = Marker.ARROW
        box_marker.scale.x = 0.45
        box_marker.scale.y = 0.15
        box_marker.scale.z = 0.15
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        int_marker.controls.append(box_control)

        axes = [('x',0,1,0,0), ('y',0,0,0,1), ('rotate_z',0,0,1,0)]
        for name_ctrl, w, x, y, z in axes:
            control = InteractiveMarkerControl()
            control.name = name_ctrl
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS if 'move' in name_ctrl else InteractiveMarkerControl.ROTATE_AXIS
            control.orientation.w = w
            control.orientation.x = x
            control.orientation.y = y
            control.orientation.z = z
            int_marker.controls.append(control)

        self.im_server.insert(int_marker)
        self.im_server.applyChanges()
        return True

    def callback_add(self, req, resp=None):
        resp = LocationsServer.Response()
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time()) #cambia al nombre de la base 'base_link'
            trans.child_frame_id = req.location_name
            self.tf_static_broadcaster.sendTransform(trans)
            succ = self.write_yaml_locs(trans, req.location_name)
            resp.success = succ
        except Exception as e:
            self.get_logger().error(f"No TF found: {e}")
            resp.success = False
        return resp

    def callback_update(self, req, resp=None):
        resp = LocationsServer.Response()
        resp.success = True
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

