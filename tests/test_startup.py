"""Exercise startup child commands without ROS, screen sessions or hardware.

Run on Linux with: python3 -m unittest discover -s tests -v
"""

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
COMMAND_STUB = r'''#!/usr/bin/env python3
import json
import os
from pathlib import Path
import subprocess
import sys

name = Path(sys.argv[0]).name
args = sys.argv[1:]
if name == "screen" and args[0] == "-dmS":
    sys.exit(subprocess.run(args[2:], stdin=subprocess.DEVNULL).returncode)
elif name == "colcon":
    Path("install").mkdir(exist_ok=True)
    Path("install/setup.bash").write_text(":\n")
elif name == "ros2":
    if args[0] in ("launch", "run"):
        with open(os.environ["STARTUP_LOG"], "a") as log:
            log.write(json.dumps({
                "args": args,
                "rmw": os.environ.get("RMW_IMPLEMENTATION"),
                "qos": os.environ.get("RMW_FASTRTPS_USE_QOS_FROM_XML"),
                "profile": os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"),
            }) + "\n")
    elif args[:2] == ["node", "list"]:
        print("/inference_node\n/joy_node\n/camera/camera\n/depth_node")
    elif args[:2] == ["topic", "list"]:
        print("/camera/camera/depth/image_rect_raw\n/depth_obs")
    elif args[:2] == ["topic", "hz"]:
        print("average rate: 30.0")
elif name == "pgrep":
    sys.exit(1)
'''


@unittest.skipUnless(os.name == "posix" and shutil.which("bash"), "requires Bash")
class StartupTests(unittest.TestCase):
    def run_startup(self, directory, script, arguments, expected_commands):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary) / directory
            (workspace / "tools").mkdir(parents=True)
            # read_text normalizes checkout line endings before invoking Bash.
            (workspace / "tools" / script).write_text(
                (ROOT / "tools" / script).read_text(encoding="utf-8"),
                encoding="utf-8",
            )
            (workspace / "tools/ros_environment.sh").write_text(
                "setup_ros_environment() { return 0; }\n"
            )
            for config in (
                "src/inference/robots/rp1_24dof/robot.yaml",
                "src/inference/robots/rp1_24dof/configs/default.yaml",
                "src/camera/configs/parkour.yaml",
                "src/camera/configs/realsense_d435i.yaml",
                "assets/rt_fastdds_profile.xml",
            ):
                target = workspace / config
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text("<dds/>\n" if config.endswith(".xml") else "{}\n")

            bin_dir = Path(temporary) / "bin"
            bin_dir.mkdir()
            for command in ("screen", "colcon", "ros2", "sleep", "pgrep"):
                stub = bin_dir / command
                stub.write_text(COMMAND_STUB)
                stub.chmod(0o755)
            log = Path(temporary) / "launches.jsonl"
            log.touch()
            env = dict(os.environ, PATH=f"{bin_dir}:{os.environ['PATH']}",
                       STARTUP_LOG=str(log))
            result = subprocess.run(
                ["bash", str(workspace / "tools" / script), *arguments],
                cwd=temporary, env=env, capture_output=True, text=True, timeout=30,
            )
            output = result.stdout + result.stderr
            self.assertEqual(result.returncode, 0, output)
            launches = [json.loads(line) for line in log.read_text().splitlines()]
            self.assertEqual([entry["args"] for entry in launches], expected_commands,
                             output)
            for entry in launches:
                self.assertEqual(entry["rmw"], "rmw_fastrtps_cpp")
                self.assertEqual(entry["qos"], "1")
                self.assertEqual(entry["profile"],
                                 str(workspace / "assets/rt_fastdds_profile.xml"))

    def test_robot_workspace_paths(self):
        for directory in ("rp1", "RP1 workspace", "RP1's workspace"):
            with self.subTest(directory=directory):
                self.run_startup(directory, "start_robot.sh", ["rp1_24dof", "default"], [
                    ["launch", "roboparty_inference", "inference.launch.py",
                     "robot:=rp1_24dof", "policy:=default"],
                    ["run", "joy", "joy_node"],
                ])

    def test_camera_workspace_paths(self):
        for directory in ("rp1", "RP1 workspace", "RP1's workspace"):
            with self.subTest(directory=directory):
                self.run_startup(directory, "start_camera.sh", ["--policy", "parkour"], [
                    ["launch", "realsense2_camera", "rs_launch.py",
                     "config_file:=src/camera/configs/realsense_d435i.yaml"],
                    ["launch", "camera", "depth.launch.py", "policy:=parkour",
                     "start_camera:=false"],
                ])


if __name__ == "__main__":
    unittest.main()
