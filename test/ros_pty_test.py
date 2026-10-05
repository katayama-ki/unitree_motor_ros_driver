#!/usr/bin/env python3
"""Exercise the actual ROS node and bundled SDK on an emulated GO serial bus."""

import os
import pty
import select
import signal
import struct
import subprocess
import tempfile
import threading
import time
import tty
import unittest

import roslib.packages
import rospy
import rostest
from unitree_motor_ros_driver.msg import MotorCommand, MotorState


def crc_ccitt(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


class RosPtyTest(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.lock = threading.Lock()
        self.commands = []
        self.states = []
        self.error = None
        self.done = threading.Event()
        self.process = None
        self.output = tempfile.TemporaryFile(mode="w+")
        self.reader = threading.Thread(target=self.emulate, daemon=True)
        self.reader.start()
        self.subscriber = rospy.Subscriber("/pty_driver/state", MotorState, self.state)
        self.publisher = rospy.Publisher("/pty_driver/command", MotorCommand, queue_size=10)
        executable = roslib.packages.find_node("unitree_motor_ros_driver", "unitree_motor_driver_node")[0]
        environment = os.environ.copy()
        environment["UNITREE_TEST_PTY"] = os.ttyname(self.slave)
        environment["LD_PRELOAD"] = os.path.join(os.path.dirname(os.path.dirname(executable)),
                                                 "libpty_serial_shim.so")
        self.process = subprocess.Popen([
            executable, "__name:=pty_driver",
            "_serial_port:=" + os.ttyname(self.slave), "_motor_type:=GO-M8010-6",
            "_motor_id_scan_max:=1", "_command_timeout:=0.2", "_timeout_action:=brake",
            "_active_io_rate:=100", "_idle_io_rate:=20", "_idle_state_pub_rate:=20",
        ], stdout=self.output, stderr=subprocess.STDOUT, env=environment)

    def tearDown(self):
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.done.set()
        self.reader.join(timeout=2)
        self.subscriber.unregister()
        self.publisher.unregister()
        os.close(self.master)
        os.close(self.slave)
        self.output.close()

    def state(self, msg):
        with self.lock:
            self.states.append(msg)

    def emulate(self):
        buffer = b""
        try:
            while not self.done.is_set():
                if not select.select([self.master], [], [], 0.02)[0]:
                    continue
                buffer += os.read(self.master, 4096)
                while len(buffer) >= 17:
                    packet, buffer = buffer[:17], buffer[17:]
                    if packet[:2] != b"\xfe\xee" or crc_ccitt(packet[:-2]) != struct.unpack("<H", packet[-2:])[0]:
                        raise ValueError("Invalid SDK command framing/CRC")
                    mode = packet[2]
                    motor_id = mode & 15
                    values = struct.unpack("<hhiHH", packet[3:15])
                    with self.lock:
                        self.commands.append((motor_id, (mode >> 4) & 7, values))
                    # Output feedback is approx q=1 rad, dq=2 rad/s, tau=3 Nm.
                    # ID 1 has a startup over-current fault; ID 0 stays normal.
                    payload = struct.pack("<2sBhhibH", b"\xfe\xee", mode,
                                          int(3 / 6.33 * 256), int(2 * 6.33 / 6.2832 * 256),
                                          int(6.33 / 6.2832 * 32768), 25,
                                          2 if motor_id == 1 else 0)
                    os.write(self.master, payload + struct.pack("<H", crc_ccitt(payload)))
        except Exception as error:
            self.error = error

    def wait_for(self, predicate, seconds=5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.error:
                self.fail("Serial emulator failed: " + str(self.error))
            if self.process.poll() is not None:
                self.output.seek(0)
                self.fail("Driver exited unexpectedly:\n" + self.output.read())
            with self.lock:
                if predicate():
                    return
            time.sleep(0.01)
        self.output.seek(0)
        self.fail("Timed out waiting for driver behavior:\n" + self.output.read())

    def test_topics_conversion_fault_watchdog_and_shutdown(self):
        self.wait_for(lambda: self.publisher.get_num_connections() > 0 and
                      any(s.id == 0 for s in self.states) and any(s.id == 1 for s in self.states))
        with self.lock:
            normal = next(s for s in self.states if s.id == 0)
            fault = next(s for s in self.states if s.id == 1)
        self.assertAlmostEqual(normal.position, 1, delta=0.001)
        self.assertAlmostEqual(normal.velocity, 2, delta=0.005)
        self.assertAlmostEqual(normal.torque, 3, delta=0.03)
        self.assertEqual(normal.mode, MotorState.BRAKE)
        self.assertEqual(fault.error, 2)
        self.assertEqual(fault.mode, MotorState.BRAKE)
        self.assertGreater(normal.header.stamp.to_nsec(), 0)

        command = MotorCommand(id=0, position=0.4, velocity=-0.3, torque=5, kp=25, kd=0.6)
        self.publisher.publish(command)
        self.wait_for(lambda: any(motor_id == 0 and mode == 1 for motor_id, mode, _ in self.commands))
        with self.lock:
            values = next(values for motor_id, mode, values in self.commands if motor_id == 0 and mode == 1)
        self.assertEqual(values[0], int(5 / 6.33 * 256))
        self.assertEqual(values[3], int(25 / (6.33 ** 2) / 25.6 * 32768))
        self.assertEqual(values[4], int(0.6 / (6.33 ** 2) / 25.6 * 32768))
        self.publisher.publish(MotorCommand(id=1, torque=5))
        self.wait_for(lambda: any(s.id == 0 and s.mode == MotorState.FOC for s in self.states))

        # Invalid commands must not refresh the watchdog or replace the valid command.
        for _ in range(30):
            self.publisher.publish(MotorCommand(id=0, torque=float("nan")))
            time.sleep(0.01)
        self.wait_for(lambda: any(motor_id == 0 and mode == 0 for motor_id, mode, _ in self.commands[-4:]))
        with self.lock:
            self.assertFalse(any(motor_id == 1 and mode == 1 for motor_id, mode, _ in self.commands))
            for motor_id in (0, 1):
                stamps = [s.header.stamp.to_nsec() for s in self.states if s.id == motor_id]
                self.assertEqual(len(stamps), len(set(stamps)))

        self.process.send_signal(signal.SIGINT)
        self.assertEqual(self.process.wait(timeout=5), 0)
        with self.lock:
            self.assertEqual([c[0] for c in self.commands[-2:]], [0, 1])
            for _, mode, values in self.commands[-2:]:
                self.assertEqual(mode, 0)
                self.assertEqual(values, (0, 0, 0, 0, 0))
        self.output.seek(0)
        log = self.output.read()
        self.assertIn("Motor id 1 reported error code 2 (over-current)", log)
        self.assertIn("command watchdog expired", log)
        self.assertIn("shutdown BRAKE acknowledged", log)


if __name__ == "__main__":
    rospy.init_node("ros_pty_test")
    rostest.rosrun("unitree_motor_ros_driver", "ros_pty_test", RosPtyTest)
