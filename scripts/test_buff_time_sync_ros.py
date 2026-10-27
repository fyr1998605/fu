#!/usr/bin/env python3
"""Exercise the actual Fu node with /clock, stamped images and delayed IMU data.

Run through run_ros_test.py on an isolated master. Blank images test time/pose
delivery and rejection, rather than claiming detector or projectile accuracy.
"""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rospy
from geometry_msgs.msg import Vector3, Vector3Stamped
from rm_msgs.msg import RmSerial
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import yaml


def wait_until(predicate, reason, seconds=8):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError(reason)


def main():
    if os.environ.get("BUFF_TEST_ISOLATED_MASTER") != "1":
        raise RuntimeError("Run through scripts/run_ros_test.py to keep the user's ROS master unchanged")
    parser = argparse.ArgumentParser()
    parser.add_argument("--fu-binary", required=True)
    parser.add_argument("--model-path", required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    rospy.set_param("/use_sim_time", True)
    rospy.init_node("buff_sync_ros_test", anonymous=True, disable_signals=True)
    clock = rospy.Publisher("/clock", Clock, queue_size=10, latch=True)
    images = rospy.Publisher("/hikrobot_camera/rgb", Image, queue_size=1)
    serial = rospy.Publisher("/RmSerialData", RmSerial, queue_size=10)
    brackets, debug_images, errors, commands = [], [], [], []
    subscribers = [
        rospy.Subscriber("/buff/sync_bracket", Vector3Stamped, brackets.append),
        rospy.Subscriber("/buff/reprojection_image", Image, debug_images.append),
        rospy.Subscriber("/buff/image_pose_time_error_ms", Float64, errors.append),
        rospy.Subscriber("/auto_angle", Vector3, commands.append),
    ]

    def set_clock(t):
        clock.publish(Clock(rospy.Time.from_sec(t)))
        time.sleep(.04)

    def send_pose(t, yaw=.1, mode='f'):
        message = RmSerial()
        message.header.stamp = rospy.Time.from_sec(t)
        message.Yaw = yaw
        message.Qw = math.cos(yaw / 2)
        message.Qz = math.sin(yaw / 2)
        message.Camp, message.ShootFlag = ord('r'), ord(mode)
        message.BulletVec = 21.0
        serial.publish(message)

    def send_image(t, seq, stream=False):
        message = Image()
        message.header.stamp, message.header.seq = rospy.Time.from_sec(t), seq
        message.header.frame_id = "sync_stream_%d" % seq if stream else "sync_test_camera"
        message.width, message.height, message.step = 320, 240, 960
        pixels = bytearray(320 * 240 * 3)
        if stream:
            pixels[0] = seq % 251 + 1  # Source-payload identity outside the HUD.
        message.encoding, message.data = "bgr8", bytes(pixels)
        images.publish(message)

    def matches(messages, t):
        return [m for m in messages if abs(m.header.stamp.to_sec() - t) < 1e-8]

    with open(root / "src/rm_tracker/config/rm_fu.yaml") as f:
        config = yaml.safe_load(f)
    for key, value in config.items():
        rospy.set_param("/Fu/" + key, value)
    rospy.set_param("/Fu/buff_model_path", str(Path(args.model_path).resolve()))
    rospy.set_param("/Fu/buff_show_image", False)
    rospy.set_param("/Fu/buff_publish_debug_image", True)
    # Leave enough delivery margin for this runtime test; the unit test covers
    # the production 50ms wait bound directly, including a paused clock.
    rospy.set_param("/Fu/buff_max_imu_wait_seconds", .15)

    for mode in ("small", "big"):
        brackets.clear(); debug_images.clear(); errors.clear(); commands.clear()
        rospy.set_param("/Fu/buff_move_mode", mode)
        set_clock(10.1)
        with tempfile.TemporaryFile(mode="w+") as log:
            node = subprocess.Popen([args.fu_binary, "__name:=Fu"], stdout=log,
                                    stderr=log, start_new_session=True)
            try:
                wait_until(lambda: images.get_num_connections() and serial.get_num_connections(),
                           "Fu did not subscribe to image and serial inputs", 20)
                wait_until(lambda: all(s.get_num_connections() for s in subscribers),
                           "Fu output subscribers did not finish reconnecting", 20)
                set_clock(10.1)
                send_pose(10.0)
                send_pose(10.02, .3)
                time.sleep(.03)
                send_image(10.01, 101)
                wait_until(lambda: matches(debug_images, 10.01), "aligned debug image missing")
                bracket = matches(brackets, 10.01)[0]
                assert abs(bracket.vector.x + 10) < 1e-5, bracket.vector
                assert abs(bracket.vector.y - 10) < 1e-5, bracket.vector
                assert matches(debug_images, 10.01)[0].header.frame_id == "sync_test_camera"
                wait_until(lambda: commands, "blank observation did not publish a control state")
                assert all(m.z == 0 for m in commands), "blank observations must not indicate a target or aim"

                # Frame is close to a prior pose, but must wait for the upper sample.
                send_image(10.025, 102)
                time.sleep(.03)
                assert not matches(brackets, 10.025), "nearby past pose was substituted"
                send_pose(10.03)
                wait_until(lambda: matches(debug_images, 10.025), "late upper IMU did not resolve frame")

                # Paused /clock cannot prevent a wall-time wait from expiring.
                send_image(10.04, 103)
                time.sleep(.3)
                assert not matches(brackets, 10.04), "unbracketed frame was processed"
                assert not matches(debug_images, 10.04), "unbracketed frame was published"
                assert commands[-1].z == 0, "missing pose must disable aiming"
                before = len(brackets)
                send_image(10.04, 104)
                send_image(0.0, 105)
                time.sleep(.05)
                assert len(brackets) == before, "duplicate or zero timestamp was processed"

                # A wide bracket is not downgraded to a nearest pose.
                send_pose(10.09)
                send_image(10.06, 106)
                time.sleep(.3)
                assert not matches(brackets, 10.06), "over-wide IMU bracket was accepted"

                # A reset discards old-epoch frames/samples and accepts new time.
                set_clock(1.1)
                send_pose(10.1)
                send_image(10.07, 107)
                time.sleep(.05)
                assert not matches(brackets, 10.07), "old epoch survived clock rollback"
                send_pose(1.02, 3.10)
                send_pose(1.04, -3.10)
                time.sleep(.03)
                send_image(1.03, 108)
                wait_until(lambda: matches(debug_images, 1.03), "new epoch failed to resume")

                # Sustained input must select an older bracketed source, rather
                # than endlessly replacing it with an unbracketed latest image.
                set_clock(2.1)
                for index in range(200):
                    t = 2.1 + index * .01
                    clock.publish(Clock(rospy.Time.from_sec(t)))
                    send_pose(t - .02)
                    send_image(t, 500 + index, stream=True)
                    time.sleep(.01)
                time.sleep(.05)
                stream_images = [m for m in debug_images
                                 if m.header.frame_id.startswith("sync_stream_")]
                assert len(stream_images) > 10, "continuous delayed IMU starved image processing"
                stream_stamps = [m.header.stamp.to_sec() for m in stream_images]
                assert all(b > a for a, b in zip(stream_stamps, stream_stamps[1:])), \
                    "buffered observations went backwards or repeated"
                for source in stream_images:
                    index = int(round((source.header.stamp.to_sec() - 2.1) / .01))
                    assert source.header.frame_id == "sync_stream_%d" % (500 + index), \
                        "selected stamp and image header belong to different frames"
                    assert source.data[0] == (500 + index) % 251 + 1, \
                        "selected image payload was replaced by a newer frame"
                assert all(abs(e.data) < 1e-6 for e in errors)
                assert all(math.isfinite(v) for m in commands for v in (m.x, m.y, m.z))
                assert commands and all(m.z == 0 for m in commands), "blank/expired inputs produced a target state"
                # Release Fu's last command once on mode handoff, then let the
                # armor node own the topic instead of publishing every frame.
                set_clock(4.12)
                before_handoff = len(commands)
                send_pose(4.12, mode='a')
                wait_until(lambda: len(commands) > before_handoff, "Fu mode exit did not clear its command")
                assert commands[-1].z == 0, "Fu mode handoff retained an aiming state"
                after_handoff = len(commands)
                send_image(4.12, 800)
                time.sleep(.2)
                assert len(commands) == after_handoff, "inactive Fu competed with armor control"
                assert node.poll() is None, "Fu exited unexpectedly"
                print(mode, "PASS", "aligned_frames=3", "max_pose_time_error_ms=0",
                      "continuous_stream_aligned=%d/200" % len(stream_images),
                      "delayed_IMU/timeout/duplicate/zero/wide_bracket/clock_reset/source_payload checked")
            except BaseException:
                log.seek(0)
                print(log.read()[-12000:])
                raise
            finally:
                if node.poll() is None:
                    node.send_signal(signal.SIGINT)
                    try:
                        node.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        node.kill()
                        node.wait()
        wait_until(lambda: images.get_num_connections() == 0 and
                   serial.get_num_connections() == 0 and
                   all(s.get_num_connections() == 0 for s in subscribers),
                   "previous Fu connections did not close before the next mode", 10)
    for subscription in subscribers:
        subscription.unregister()


if __name__ == "__main__":
    main()
