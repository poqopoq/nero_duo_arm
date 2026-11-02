#!/usr/bin/env python3
"""Publish solved camera extrinsic YAML files as static TF."""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation

from .common import load_yaml


class ExtrinsicPublisher(Node):
    def __init__(self):
        super().__init__('camera_extrinsic_publisher')
        self.declare_parameter('yaml_files', '')
        files = [
            value.strip() for value in
            self.get_parameter('yaml_files').value.split(',') if value.strip()
        ]
        broadcaster = StaticTransformBroadcaster(self)
        transforms = []
        for path in files:
            data = load_yaml(path)['extrinsic']
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = data['parent_frame']
            msg.child_frame_id = data['child_frame']
            msg.transform.translation.x = data['translation'][0]
            msg.transform.translation.y = data['translation'][1]
            msg.transform.translation.z = data['translation'][2]
            q = data['quaternion_xyzw']
            msg.transform.rotation.x = q[0]
            msg.transform.rotation.y = q[1]
            msg.transform.rotation.z = q[2]
            msg.transform.rotation.w = q[3]
            transforms.append(msg)
            self.get_logger().info(
                f'{data["parent_frame"]} -> {data["child_frame"]} from {path}')
        if not transforms:
            raise RuntimeError('No extrinsic YAML files provided')
        broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
