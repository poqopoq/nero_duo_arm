from __future__ import annotations

import os
import socket
import tempfile
import threading
import unittest

import cv2
import numpy as np
import torch

from lerobot.async_inference.configs import RobotClientConfig
from lerobot.async_inference.robot_client import RobotClient
from lerobot.robots import RobotConfig, make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot_robot_nero import Nero, NeroConfig
from lerobot_robot_nero.nero import STATE_NAMES
from lerobot_robot_nero.protocol import recv_frame, send_frame


class MockBridge:
    def __init__(self) -> None:
        self._tmp = tempfile.TemporaryDirectory(prefix='nero-vla-plugin-test-')
        self.socket_path = os.path.join(self._tmp.name, 'bridge.sock')
        image = np.zeros((48, 64, 3), dtype=np.uint8)
        image[:, :, 0] = 120
        image[:, :, 1] = 80
        ok, encoded = cv2.imencode('.jpg', image)
        assert ok
        self.image_payload = encoded.tobytes()
        self._server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._server.bind(self.socket_path)
        self._server.listen(1)
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def __enter__(self) -> 'MockBridge':
        self._thread.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self._server.close()
        self._thread.join(timeout=1.0)
        self._tmp.cleanup()

    def _serve(self) -> None:
        try:
            conn, _ = self._server.accept()
        except OSError:
            return
        with conn:
            while True:
                try:
                    request, _ = recv_frame(conn)
                except Exception:
                    break
                command = request.get('cmd')
                if command == 'ping':
                    send_frame(conn, {'ok': True, 'side': 'left', 'dry_run': True})
                elif command == 'get_observation':
                    send_frame(
                        conn,
                        {
                            'ok': True,
                            'state': [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.75],
                            'image': {'key': 'camera1', 'encoding': 'jpeg', 'shape': [48, 64, 3]},
                        },
                        self.image_payload,
                    )
                elif command == 'send_action':
                    send_frame(conn, {'ok': True, 'action': request['action'], 'executed': False})
                elif command == 'stop':
                    send_frame(conn, {'ok': True, 'stopped': True})
                else:
                    send_frame(conn, {'ok': False, 'error': 'unsupported test command'})


class NeroPluginTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        register_third_party_plugins()

    def test_registration_and_features(self) -> None:
        self.assertIs(RobotConfig.get_choice_class('nero_ros'), NeroConfig)
        robot = Nero(NeroConfig(image_width=64, image_height=48))
        self.assertEqual(tuple(robot.action_features), STATE_NAMES)
        self.assertEqual(tuple(robot.observation_features), (*STATE_NAMES, 'camera1'))
        self.assertEqual(robot.observation_features['camera1'], (48, 64, 3))

    def test_closed_loop(self) -> None:
        with MockBridge() as bridge:
            config = NeroConfig(
                bridge_socket=bridge.socket_path,
                image_width=64,
                image_height=48,
                stop_on_disconnect=False,
            )
            robot = make_robot_from_config(config)
            self.assertIsInstance(robot, Nero)
            robot.connect()
            observation = robot.get_observation()
            self.assertEqual(observation['camera1'].shape, (48, 64, 3))
            self.assertAlmostEqual(observation['ee.x'], 0.1)
            self.assertAlmostEqual(observation['ee.gripper'], 0.75)
            applied = robot.send_action({name: observation[name] for name in robot.action_features})
            self.assertAlmostEqual(applied['ee.z'], 0.3)
            robot.disconnect()

    def test_robot_client_feature_mapping(self) -> None:
        with MockBridge() as bridge:
            client = RobotClient(
                RobotClientConfig(
                    robot=NeroConfig(
                        bridge_socket=bridge.socket_path,
                        image_width=64,
                        image_height=48,
                        stop_on_disconnect=False,
                    ),
                    policy_type='smolvla',
                    pretrained_name_or_path='test/checkpoint',
                    actions_per_chunk=10,
                )
            )
            features = client.policy_config.lerobot_features
            self.assertEqual(features['observation.state']['shape'], (7,))
            self.assertEqual(features['observation.images.camera1']['shape'], (48, 64, 3))
            action = client._action_tensor_to_action_dict(torch.arange(7, dtype=torch.float32))
            self.assertEqual(list(action.values()), [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
            client.robot.disconnect()


if __name__ == '__main__':
    unittest.main()

