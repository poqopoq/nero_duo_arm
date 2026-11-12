#!/usr/bin/env python3
"""Publish solved camera extrinsic YAML files as static TF."""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation

from .common import load_yaml


def _truthy(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _calibration_status(data: dict) -> str:
    status = str(data.get('status', 'UNKNOWN')).upper()
    return status if status in ('PASS', 'WARN') else 'UNKNOWN'


class ExtrinsicPublisher(Node):
    def __init__(self):
        super().__init__('camera_extrinsic_publisher')
        self.declare_parameter('yaml_files', '')
        self.declare_parameter('allow_degraded', False)
        files = [
            value.strip() for value in
            self.get_parameter('yaml_files').value.split(',') if value.strip()
        ]
        allow_degraded = _truthy(self.get_parameter('allow_degraded').value)
        loaded = [(path, load_yaml(path)['extrinsic']) for path in files]
        rejected = []
        for path, data in loaded:
            status = _calibration_status(data)
            if status == 'PASS':
                continue
            if allow_degraded:
                self.get_logger().warn(
                    f'{path}: calibration status {status}; publishing anyway '
                    'because allow_degraded:=true')
                continue
            rejected.append(f'{path} ({status})')
        if rejected:
            raise RuntimeError(
                'Refusing to publish an extrinsic that failed its quality '
                f'checks: {", ".join(rejected)}. Re-run solve_extrinsics until '
                'it reports CALIBRATION PASS, or set allow_degraded:=true.')
        broadcaster = StaticTransformBroadcaster(self)
        transforms = []
        for path, data in loaded:
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
