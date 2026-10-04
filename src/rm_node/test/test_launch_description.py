from pathlib import Path
import unittest


LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"


class EnergyLaunchSourceTest(unittest.TestCase):
    def test_start_fan_launch_has_only_energy_nodes(self):
        text = (LAUNCH_DIR / "start_fan.launch.py").read_text(encoding="utf-8")
        self.assertIn('executable="hikcamera"', text)
        self.assertIn('executable="rm_serial_node"', text)
        self.assertIn('executable="Fu"', text)
        self.assertNotIn("rm_identify", text)
        self.assertNotIn('executable="tracker"', text)

    def test_start_fan_launch_declares_simulation_arguments(self):
        text = (LAUNCH_DIR / "start_fan.launch.py").read_text(encoding="utf-8")
        for argument in ("shootflag_test", "camp_test", "sim_mode"):
            self.assertIn(f'"{argument}"', text)


if __name__ == "__main__":
    unittest.main()
