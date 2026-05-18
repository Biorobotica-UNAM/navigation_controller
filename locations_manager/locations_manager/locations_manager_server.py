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

from known_locations_interfaces.srv import (
    LocationsServer,
    LocationsGet,
    SavePoseLocation
)

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


def _repo_root_from_this_file(
    markers=('package.xml', 'setup.py', 'CMakeLists.txt')
) -> Path:

    p = Path(__file__).resolve()

    for parent in (p.parent,) + tuple(p.parents):
        for m in markers:
            if (parent / m).exists():
                return parent

    return p.parent


def _pkg_yaml_abs_path(rel_path: str) -> str:
    root = _repo_root_from_this_file()
    path = (root / rel_path.lstrip('/')).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    return str(path)


class LocationsManagerServer(Node):

    def __init__(self):

        super().__init__('locations_manager_server')

        self.declare_parameter(
            'known_locations_file',
            'known_locations.yaml'
        )

        self.declare_parameter(
            'map_frame',
            'map'
        )

        self.declare_parameter(
            'base_frame',
            'base_link'
        )

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

        self.load_known_locations(
            self.known_locations_file_name
        )

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # Use dynamic broadcaster editable at runtime
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(
            self
        )

        # Publish periodically
        self.tf_timer = self.create_timer(
            0.5,
            self.publish_all_known_locations
        )

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

        self.get_logger().info(
            "Known locations server ready."
        )

    def load_known_locations(self, file_name):

        yaml_path = _pkg_yaml_abs_path(
            f'config/{file_name}'
        )

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
            f"Initialized default YAML at {file_path}"
        )

    def append_known_loc_to_yaml(self):

        with open(self.yaml_path, 'w') as f:
            yaml.dump(self.known_locations, f)

    def publish_all_known_locations(self):

        for name, pose_dict in self.known_locations.items():

            ts = TransformStamped()

            ts.header.stamp = self.get_clock().now().to_msg()

            ts.header.frame_id = self.map_frame
            ts.child_frame_id = name

            ts.transform.translation.x = pose_dict['x']
            ts.transform.translation.y = pose_dict['y']
            ts.transform.translation.z = 0.0

            # Use stored quaternion directly
            ts.transform.rotation.x = pose_dict['qx']
            ts.transform.rotation.y = pose_dict['qy']
            ts.transform.rotation.z = pose_dict['qz']
            ts.transform.rotation.w = pose_dict['qw']

            self.tf_broadcaster.sendTransform(ts)

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

    def save_pose_dict(self, name, translation, rotation):

        theta = tf.euler_from_quaternion([
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w
        ])[2]

        self.known_locations[name] = {
            'x': translation.x,
            'y': translation.y,
            'theta': theta,
            'qx': rotation.x,
            'qy': rotation.y,
            'qz': rotation.z,
            'qw': rotation.w
        }

        self.append_known_loc_to_yaml()

    def handler_add_known_loc(self, req, _resp=None):

        resp = LocationsServer.Response()

        resp.success = False

        name = req.location_name.strip()

        if not name:

            self.get_logger().warn(
                "Location name is empty."
            )

            return resp

        if name.startswith("customer"):

            self.get_logger().info(
                f"Trying TF lookup for '{name}'"
            )

            if self.save_location_from_buffer(name):

                resp.success = True

                return resp

            self.get_logger().warn(
                f"Could not save '{name}' from TF."
            )

            return resp

        # Save current robot pose
        pose = self.get_robot_current_pose()

        if pose is None:
            return resp

        self.save_pose_dict(name, pose.translation,
                            pose.rotation
                            )

        self.get_logger().info(f"Saved robot pose as '{name}'")

        resp.success = True

        return resp

    def handler_get_known_loc(self, req, _resp=None):
        resp = LocationsGet.Response()
        resp.success = False
        name = req.location_name

        if name not in self.known_locations:
            resp.message = (f"Location '{name}' not found.")
            self.get_logger().warn(resp.message)

            return resp

        loc = self.known_locations[name]
        resp.pose = Pose()

        resp.pose.position.x = loc['x']
        resp.pose.position.y = loc['y']
        resp.pose.position.z = 0.0

        resp.pose.orientation.x = loc['qx']
        resp.pose.orientation.y = loc['qy']
        resp.pose.orientation.z = loc['qz']
        resp.pose.orientation.w = loc['qw']

        resp.success = True
        resp.message = "OK"

        return resp

    def handler_save_new_loc(self, req, _resp=None):
        resp = SavePoseLocation.Response()
        resp.success = False
        name = req.location_name
        pose = req.pose

        if not name:
            self.get_logger().warn("Location name is empty.")
            return resp

        self.save_pose_dict(name, pose.position, pose.orientation)

        self.get_logger().info(f"Saved new location '{name}'")
        resp.success = True

        return resp

    def save_location_from_buffer(self, name):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                name,
                rclpy.time.Time()
            )
            translation = trans.transform.translation
            rotation = trans.transform.rotation

            self.save_pose_dict(name, translation, rotation)
            self.get_logger().info(
                f"Saved '{name}' from TF buffer."
            )
            return True

        except tf2_ros.LookupException:
            self.get_logger().error(
                f"Frame '{name}' does not exist.")

        except tf2_ros.ConnectivityException:
            self.get_logger().error(
                f"TF connectivity error for '{name}'.")

        except tf2_ros.ExtrapolationException:
            self.get_logger().error(
                f"TF extrapolation error for '{name}'.")
        except Exception as e:

            self.get_logger().error(
                f"Unexpected TF error: {e}"
            )
        return False


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
