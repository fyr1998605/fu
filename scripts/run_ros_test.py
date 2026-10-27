#!/usr/bin/env python3
"""Run a ROS test against its own master, preserving any existing user session."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client


def main():
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    env = dict(os.environ, ROS_MASTER_URI="http://127.0.0.1:%d" % port,
               ROS_HOSTNAME="127.0.0.1", BUFF_TEST_ISOLATED_MASTER="1")
    env.pop("ROS_IP", None)
    with tempfile.TemporaryDirectory(prefix="buff-ros-test-") as folder:
        env["ROS_LOG_DIR"] = folder
        with open(Path(folder) / "master.log", "w+") as log:
            master = subprocess.Popen(["roscore", "-p", str(port)], env=env,
                                      stdout=log, stderr=log, start_new_session=True)
            try:
                endpoint = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
                deadline = time.monotonic() + 15
                while True:
                    try:
                        if endpoint.getPid("buff_test")[0] == 1:
                            break
                    except OSError:
                        pass
                    if master.poll() is not None or time.monotonic() > deadline:
                        log.seek(0)
                        raise RuntimeError("Test ROS master did not start:\n" + log.read())
                    time.sleep(.1)
                return subprocess.run(sys.argv[1:], env=env, timeout=90).returncode
            finally:
                os.killpg(master.pid, signal.SIGINT)
                try:
                    master.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(master.pid, signal.SIGKILL)
                    master.wait()


if __name__ == "__main__":
    sys.exit(main())
