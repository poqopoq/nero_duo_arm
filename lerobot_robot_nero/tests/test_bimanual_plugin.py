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
from lerobot_robot_nero import NeroBimanual, NeroBimanualConfig
from lerobot_robot_nero.bimanual import state_names
from lerobot_robot_nero.protocol import recv_frame, send_frame


def _jpeg(color: int, shape: tuple[int, int, int] = (48, 64, 3)) -> bytes:
    image = np.zeros(shape, dtype=np.uint8)
    image[:, :, 0] = color
    ok, encoded = cv2.imencode('.jpg', image)
    assert ok
    return encoded.tobytes()


class MockBimanualBridge:
    def __init__(self) -> None:
        self._tmp = tempfile.TemporaryDirectory(prefix='nero-bimanual-plugin-test-')
        self.socket_path = os.path.join(self._tmp.name, 'bridge.sock')
        self.image_payloads = [(_jpeg(120), 48, 64), (_jpeg(80), 48, 64), (_jpeg(40), 48, 64)]
        self._server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._server.bind(self.socket_path)
        self._server.listen(1)
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def __enter__(self) -> 'MockBimanualBridge':
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
                    send_frame(conn, {
                        'ok': True,
                        'bimanual': True,
                        'schema': 'rot6d20',
                        'camera_keys': ['side', 'left_wrist', 'right_wrist'],
                        'dry_run': True,
                    })
                elif command == 'get_observation':
                    state = [
                        0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.75,
                        0.3, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.25,
                    ]
                    payload = bytearray()
                    images = []
                    keys = ('side', 'left_wrist', 'right_wrist')
                    for key, (image, height, width) in zip(keys, self.image_payloads):
                        offset = len(payload)
                        payload.extend(image)
                        images.append({
                            'key': key,
                            'encoding': 'jpeg',
                            'shape': [height, width, 3],
                            'offset': offset,
                            'size': len(image),
                        })
                    send_frame(conn, {
                        'ok': True,
                        'schema': 'rot6d20',
                        'state': state,
                        'images': images,
                    }, bytes(payload))
                elif command == 'send_action':
                    accepted = request['action'][0] != -999.0
                    send_frame(conn, {
                        'ok': True,
                        'action': request['action'],
                        'accepted': accepted,
                        **({} if accepted else {'error': 'motion fault latched'}),
                        'executed': False,
                    })
                elif command == 'stop':
                    send_frame(conn, {'ok': True, 'stopped': True})
                else:
                    send_frame(conn, {'ok': False, 'error': 'unsupported test command'})


class NeroBimanualPluginTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        register_third_party_plugins()

    def test_registration_and_features(self) -> None:
        self.assertIs(
            RobotConfig.get_choice_class('nero_bimanual_ros'),
            NeroBimanualConfig,
        )
        robot = NeroBimanual(NeroBimanualConfig(
            scene_image_width=64,
            scene_image_height=48,
            left_wrist_image_width=64,
            left_wrist_image_height=48,
            right_wrist_image_width=64,
            right_wrist_image_height=48,
        ))
        self.assertEqual(tuple(robot.action_features), state_names('rot6d20'))
        self.assertEqual(len(robot.action_features), 20)
        self.assertEqual(
            tuple(robot.observation_features),
            (*state_names('rot6d20'), 'side', 'left_wrist', 'right_wrist'),
        )
        self.assertEqual(robot.observation_features['side'], (48, 64, 3))
        self.assertEqual(robot.robot_type, 'bi_rdt_gripper')

    def test_closed_loop(self) -> None:
        with MockBimanualBridge() as bridge:
            config = NeroBimanualConfig(
                bridge_socket=bridge.socket_path,
                scene_image_width=64,
                scene_image_height=48,
                left_wrist_image_width=64,
                left_wrist_image_height=48,
                right_wrist_image_width=64,
                right_wrist_image_height=48,
                stop_on_disconnect=False,
            )
            robot = make_robot_from_config(config)
            self.assertIsInstance(robot, NeroBimanual)
            robot.connect()
            observation = robot.get_observation()
            self.assertEqual(observation['side'].shape, (48, 64, 3))
            self.assertEqual(observation['left_wrist'].shape, (48, 64, 3))
            self.assertEqual(observation['right_wrist'].shape, (48, 64, 3))
            self.assertAlmostEqual(observation['left_tcp.x.pos'], 0.1)
            self.assertAlmostEqual(observation['left_gripper.pos'], 0.75)
            self.assertAlmostEqual(observation['right_tcp.x.pos'], 0.3)
            applied = robot.send_action({
                name: observation[name] for name in robot.action_features
            })
            self.assertAlmostEqual(applied['left_tcp.z.pos'], 0.3)
            self.assertAlmostEqual(applied['right_gripper.pos'], 0.25)
            robot.disconnect()

    def test_robot_client_feature_mapping(self) -> None:
        with MockBimanualBridge() as bridge:
            client = RobotClient(
                RobotClientConfig(
                    robot=NeroBimanualConfig(
                        bridge_socket=bridge.socket_path,
                        scene_image_width=64,
                        scene_image_height=48,
                        left_wrist_image_width=64,
                        left_wrist_image_height=48,
                        right_wrist_image_width=64,
                        right_wrist_image_height=48,
                        stop_on_disconnect=False,
                    ),
                    policy_type='smolvla',
                    pretrained_name_or_path='test/checkpoint',
                    actions_per_chunk=10,
                )
            )
            features = client.policy_config.lerobot_features
            self.assertEqual(features['observation.state']['shape'], (20,))
            self.assertEqual(features['observation.images.side']['shape'], (48, 64, 3))
            self.assertEqual(
                features['observation.images.left_wrist']['shape'], (48, 64, 3))
            action = client._action_tensor_to_action_dict(
                torch.arange(20, dtype=torch.float32))
            self.assertEqual(len(action), 20)
            self.assertEqual(list(action.values()), [float(i) for i in range(20)])
            client.robot.disconnect()

    def test_rejected_action_aborts_rollout(self) -> None:
        with MockBimanualBridge() as bridge:
            robot = NeroBimanual(NeroBimanualConfig(
                bridge_socket=bridge.socket_path,
                scene_image_width=64,
                scene_image_height=48,
                left_wrist_image_width=64,
                left_wrist_image_height=48,
                right_wrist_image_width=64,
                right_wrist_image_height=48,
                stop_on_disconnect=False,
            ))
            robot.connect()
            action = {name: 0.0 for name in robot.action_features}
            action['left_tcp.x.pos'] = -999.0
            with self.assertRaisesRegex(RuntimeError, 'motion fault latched'):
                robot.send_action(action)
            robot.disconnect()


if __name__ == '__main__':
    unittest.main()
