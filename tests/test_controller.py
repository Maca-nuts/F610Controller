"""ROS topic integration test; all topics are isolated from the real CAN bridge."""
import subprocess
from pathlib import Path
import sys
import time
import unittest
import uuid

import rclpy
from can_msgs.msg import Frame
from f610_controller.msg import MotorTarget, MotorState


EXECUTABLE = sys.argv.pop(1)


class ControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('controller_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        namespace = '/test_' + uuid.uuid4().hex
        self.namespace = namespace
        self.currents = {}
        self.states = {}
        self.feedback = self.node.create_publisher(Frame, namespace + '/from_can_bus', 10)
        self.target = self.node.create_publisher(MotorTarget, namespace + '/motor_target', 10)
        self.sub = self.node.create_subscription(Frame, namespace + '/to_can_bus', self.receive, 100)
        self.state_sub = None
        if self._testMethodName in ('test_output_position_conversion_and_state', 'test_multiturn_and_zero_position'):
            self.state_sub = self.node.create_subscription(
                MotorState, namespace + '/motor_state',
                lambda state: self.states.update({state.id: state}), 100)
        self.process = subprocess.Popen([
            EXECUTABLE, '--ros-args', '-r', '__ns:=' + namespace,
            '-p', 'velocity_kp:=100.0', '-p', 'velocity_ki:=200.0',
            '-p', 'current_limit_ma:=500.0',
            '-p', 'position_kp:=1.0', '-p', 'position_kd:=0.1',
            '-p', 'position_speed_limit_rps:=2.0', '-p', 'control_period_ms:=10',
            '-p', 'gear_ratio:=' + (
                '36.0' if self.state_sub is not None else '1.0'),
            '-p', 'position_tolerance_rad:=' + (
                '0.05' if self._testMethodName == 'test_position_deadband' else '0.0'),
        ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.addCleanup(self.stop_process)
        deadline = time.monotonic() + 5
        while (self.target.get_subscription_count() == 0 or
               self.feedback.get_subscription_count() == 0 or
               self.sub.get_publisher_count() == 0):
            self.assertIsNone(self.process.poll())
            self.assertLess(time.monotonic(), deadline, 'ROS discovery timed out')
            rclpy.spin_once(self.node, timeout_sec=0.02)
        # DDSの接続数だけでなく、指令が往復するまで待ってから各試験を開始する。
        deadline = time.monotonic() + 3
        while 1 not in self.currents:
            self.assertIsNone(self.process.poll())
            self.assertLess(time.monotonic(), deadline, 'Controller command handshake timed out')
            self.target.publish(MotorTarget(id=1))
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def stop_process(self):
        if self.process.poll() is None:
            self.process.terminate()
        self.process.communicate(timeout=5)

    def tearDown(self):
        self.stop_process()
        self.node.destroy_publisher(self.feedback)
        self.node.destroy_publisher(self.target)
        self.node.destroy_subscription(self.sub)
        if self.state_sub is not None:
            self.node.destroy_subscription(self.state_sub)

    def receive(self, frame):
        start = 1 if frame.id == 0x200 else 5
        for slot in range(4):
            self.currents[start + slot] = int.from_bytes(
                bytes(frame.data[slot * 2:slot * 2 + 2]), 'big', signed=True)

    def drive(self, seconds=0.2, targets=(), feedback=()):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            for target in targets:
                motor_id, rps, pos = target[:3]
                mode = target[3] if len(target) > 3 else MotorTarget.AUTO
                self.target.publish(MotorTarget(id=motor_id, target_rps=float(rps), target_pos=float(pos), mode=mode))
            for motor_id, angle, rpm in feedback:
                frame = Frame(id=0x200 + motor_id, dlc=8)
                frame.data[0] = angle >> 8
                frame.data[1] = angle & 255
                frame.data[2] = (rpm & 65535) >> 8
                frame.data[3] = rpm & 255
                self.feedback.publish(frame)
            rclpy.spin_once(self.node, timeout_sec=0.002)
            rclpy.spin_once(self.node, timeout_sec=0.002)
            time.sleep(0.002)
        # Drain queued output.
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.001)

    def test_pi_and_independent_ids(self):
        self.drive(seconds=0.3, targets=[(1, 1, 0), (2, -1, 0), (5, 1, 0)],
                   feedback=[(1, 0, 0), (2, 0, 0), (5, 0, 0)])
        self.assertGreater(self.currents[1], 110)
        self.assertLess(self.currents[2], -110)
        self.assertGreater(self.currents[5], 110)
        self.assertEqual(self.currents[3], 0)
        self.drive(seconds=0.35, targets=[(1, 0, 0)], feedback=[(1, 0, 0)])
        self.assertEqual(self.currents[1], 0)

    def test_saturation_and_anti_windup(self):
        self.drive(seconds=0.4, targets=[(1, 100, 0)], feedback=[(1, 0, 0)])
        self.assertEqual(self.currents[1], 500)
        self.drive(targets=[(1, -0.1, 0)], feedback=[(1, 0, 0)])
        self.assertLess(self.currents[1], 0)
        self.assertGreater(self.currents[1], -30)

    def test_cascade_wrap_and_damping(self):
        self.drive(feedback=[(1, 8100, 0)])
        self.drive(targets=[(1, 0, 0.05)], feedback=[(1, 100, 0)])
        self.assertLess(self.currents[1], 0)  # +192 counts exceeds target position
        self.drive(targets=[(1, 0, 0)], feedback=[(1, 8100, 0)])
        self.drive(targets=[(1, 0, 1)], feedback=[(1, 8100, 60)])
        self.assertLess(self.currents[1], -50)  # PD speed < 1 rps due to damping
        self.drive(targets=[(1, 0, 0)], feedback=[(1, 8100, 0)])
        self.drive(targets=[(1, 0, 100)], feedback=[(1, 8100, 0)])
        self.assertGreater(self.currents[1], 200)
        self.assertLess(self.currents[1], 300)  # position speed capped at 2 rps

    def test_feedback_and_command_timeouts(self):
        self.drive(targets=[(1, 1, 0)])
        self.assertEqual(self.currents[1], 0)
        self.drive(targets=[(1, 1, 0)], feedback=[(1, 0, 0)])
        self.assertGreater(self.currents[1], 0)
        self.drive(seconds=0.65, targets=[(1, 1, 0)])
        self.assertEqual(self.currents[1], 0)
        self.drive(targets=[(1, 1, 0)], feedback=[(1, 0, 0)])
        self.drive(seconds=0.65, feedback=[(1, 0, 0)])
        self.assertEqual(self.currents[1], 0)

    def test_output_position_conversion_and_state(self):
        import math
        self.drive(seconds=0.2, targets=[(1, 0, 1 / 36)], feedback=[(1, 0, 0)])
        self.assertGreater(self.currents[1], 110)
        self.drive(targets=[(1, 0, 1 / 36)], feedback=[(1, 1024, 0)])
        self.assertAlmostEqual(self.states[1].rotor_position_rad, math.pi / 4)
        self.assertAlmostEqual(self.states[1].position_rad, math.pi / 144)

    def test_position_deadband(self):
        self.drive(seconds=0.3, targets=[(1, 0, 0.02)], feedback=[(1, 0, 0)])
        self.assertEqual(self.currents[1], 0)
        # The deadband removes position forcing but retains velocity damping.
        self.drive(targets=[(1, 0, 0.02)], feedback=[(1, 0, 60)])
        self.assertLess(self.currents[1], -150)

    def test_multiturn_and_zero_position(self):
        import math
        angle = 7000
        self.drive(targets=[(1, 0, 0)], feedback=[(1, angle, 0)])
        self.assertEqual(self.states[1].position_rad, 0.0)
        # Quarter-rotor-turn samples, including both encoder wrap directions.
        for turns, direction in [(101, 1), (202, -1)]:
            for index in range(turns * 36 * 4):
                angle = (angle + direction * 2048) % 8192
                frame = Frame(id=0x201, dlc=8)
                frame.data[0] = angle >> 8
                frame.data[1] = angle & 255
                self.feedback.publish(frame)
                if index % 16 == 0:
                    rclpy.spin_once(self.node, timeout_sec=0)
                    time.sleep(0.001)
            self.drive(targets=[(1, 0, 0, MotorTarget.POSITION)], feedback=[(1, angle, 0)])
            expected = direction * 101 * 2 * math.pi
            self.assertAlmostEqual(self.states[1].position_rad, expected, places=8)
            self.assertLess(self.currents[1] * direction, 0)
            # rad targets retain the full turn count, including local corrections
            # on either side of +/-101 output-shaft turns (gear ratio 36).
            for offset in [0.02, -0.02, 0.0]:
                self.drive(targets=[(1, 0, expected + offset, MotorTarget.POSITION)],
                           feedback=[(1, angle, 0)])
                if offset == 0.0:
                    self.assertEqual(self.currents[1], 0)
                else:
                    self.assertGreater(self.currents[1] * offset, 0)
            # Explicit zero-speed remains velocity control; STOP removes current.
            self.drive(targets=[(1, 0, 0, MotorTarget.STOP)], feedback=[(1, angle, 0)])
            self.assertEqual(self.currents[1], 0)

    def test_explicit_mode_validation(self):
        for target in [(1, 1, 0, MotorTarget.POSITION),
                       (1, 0, 1, MotorTarget.VELOCITY),
                       (1, 0, 1, MotorTarget.STOP), (1, 0, 0, 255)]:
            self.drive(targets=[target], feedback=[(1, 0, 0)])
            self.assertEqual(self.currents[1], 0)

    def test_publisher_turns_and_origin(self):
        self.check_publisher_position_argument('target_turns')

    def test_publisher_radians_and_origin(self):
        self.check_publisher_position_argument('target_pos')

    def check_publisher_position_argument(self, argument):
        import math
        received = []
        subscription = self.node.create_subscription(
            MotorTarget, self.namespace + '/motor_target', received.append, 10)
        self.addCleanup(self.node.destroy_subscription, subscription)
        state_subscription = self.node.create_subscription(
            MotorState, self.namespace + '/motor_state',
            lambda state: self.states.update({state.id: state}), 10)
        self.addCleanup(self.node.destroy_subscription, state_subscription)
        deadline = time.monotonic() + 5
        while 1 not in self.states:
            self.assertLess(time.monotonic(), deadline, 'Initial feedback not acknowledged')
            self.drive(seconds=0.05, feedback=[(1, 1234, 0)])
        publisher = str(Path(EXECUTABLE).parent / 'motor_target_publisher')
        for turns in [101, -101, 0]:
            value = turns if argument == 'target_turns' else turns * 2 * math.pi
            received.clear()
            process = subprocess.Popen([
                publisher, '--ros-args', '-r', '__ns:=' + self.namespace,
                '-p', 'mode:=position', '-p', argument + ':=' + str(value),
            ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            try:
                deadline = time.monotonic() + 5
                expected = turns * 2 * math.pi
                direction = 1 if turns > 0 else -1
                # Publisher-to-test discovery can precede Publisher-to-controller
                # discovery. Wait for the actual command, not a fixed 200 ms.
                while (not received or received[-1].target_pos != expected or
                       self.currents[1] * direction <= 0):
                    self.assertIsNone(process.poll())
                    self.assertLess(time.monotonic(), deadline)
                    self.drive(seconds=0.05, feedback=[(1, 2258, 0)])
                self.assertEqual(received[-1].mode, MotorTarget.POSITION)
                self.assertAlmostEqual(received[-1].target_pos, expected, places=10)
                self.drive(feedback=[(1, 2258, 0)])
                if turns > 0:
                    self.assertGreater(self.currents[1], 0)
                else:
                    self.assertLess(self.currents[1], 0)
            finally:
                process.terminate()
                process.communicate(timeout=5)

    def test_position_crossing_clears_integral(self):
        self.drive(seconds=0.3, targets=[(1, 0, 1)], feedback=[(1, 0, 0)])
        self.assertGreater(self.currents[1], 130)
        # Crossing the target must immediately discard the accumulated forward current.
        self.drive(seconds=0.06, targets=[(1, 0, 1)], feedback=[(1, 2048, 0)])
        self.assertLess(self.currents[1], -55)

    def test_invalid_input(self):
        self.drive(targets=[(1, 1, 0)], feedback=[(1, 0, 0)])
        for target in [(1, 1, 1), (1, float('nan'), 0), (1, 0, float('inf'))]:
            self.drive(targets=[target], feedback=[(1, 0, 0)])
            self.assertEqual(self.currents[1], 0)
        for frame in [Frame(id=0, dlc=8), Frame(id=0x209, dlc=8),
                      Frame(id=0x201, dlc=0), Frame(id=0x201, dlc=8, is_extended=True),
                      Frame(id=0x201, dlc=8, is_rtr=True), Frame(id=0x201, dlc=8, is_error=True)]:
            self.feedback.publish(frame)
        self.drive(seconds=0.65, targets=[(1, 1, 0)])
        self.assertEqual(self.currents[1], 0)
        self.assertIsNone(self.process.poll())


if __name__ == '__main__':
    unittest.main()
