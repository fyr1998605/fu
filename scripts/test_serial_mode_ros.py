#!/usr/bin/env python3
"""Check the actual serial executable on an isolated master, without hardware."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rospy
from rm_msgs.msg import RmSerial
from sensor_msgs.msg import Imu


def stop(node):
    if node.poll() is None:
        node.send_signal(signal.SIGINT)
        try:
            node.wait(timeout=4)
        except subprocess.TimeoutExpired:
            node.kill()
            node.wait()


def main():
    if os.environ.get("BUFF_TEST_ISOLATED_MASTER") != "1":
        raise RuntimeError("Use scripts/run_ros_test.py for an isolated ROS master")
    # Never open a user's real port during this regression.
    if any(Path("/dev/%s%d" % (prefix, i)).exists()
           for prefix in ("ttyUSB", "ttyACM", "ttyCH343USB") for i in range(10)):
        print("SKIP: real serial device present; no-device test must not touch hardware")
        return
    rospy.init_node("serial_mode_test", anonymous=True, disable_signals=True)
    serial, imu = [], []
    subscriptions = [rospy.Subscriber("/RmSerialData", RmSerial, serial.append),
                     rospy.Subscriber("/imu/data_raw", Imu, imu.append)]
    with tempfile.TemporaryFile(mode="w+") as log:
        hardware = subprocess.Popen([sys.argv[1], "__name:=serial_mode_hardware",
                                     "_sim_mode:=false"], stdout=log, stderr=log)
        try:
            assert hardware.wait(timeout=6) == 1, "missing hardware must return failure"
            time.sleep(.15)
            assert not serial and not imu, "missing port published synthetic telemetry"
        finally:
            stop(hardware)
        explicit = subprocess.Popen([sys.argv[1], "__name:=serial_mode_explicit",
                                     "_sim_mode:=true", "_shootflag_test:=f"],
                                    stdout=log, stderr=log)
        try:
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline and (len(serial) < 2 or len(imu) < 2):
                time.sleep(.01)
            assert len(serial) >= 2 and len(imu) >= 2, "explicit simulation failed"
            assert all(message.ShootFlag == ord('f') for message in serial)
            assert explicit.poll() is None, "explicit simulation exited"
        finally:
            stop(explicit)
    for subscription in subscriptions:
        subscription.unregister()
    print("PASS: missing hardware fails without fake data; explicitly selected simulation works")


if __name__ == "__main__":
    main()
