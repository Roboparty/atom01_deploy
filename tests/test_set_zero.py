"""Hardware-free calibration tests: python3 -m unittest discover -s tests."""

import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


spec = importlib.util.spec_from_file_location(
    "set_zero", Path(__file__).resolve().parents[1] / "scripts" / "set_zero.py"
)
set_zero = importlib.util.module_from_spec(spec)
with mock.patch.dict(sys.modules, {"motors_py": types.SimpleNamespace(
    MotorControlMode=types.SimpleNamespace(MIT=1)
)}):
    spec.loader.exec_module(set_zero)


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.motor = mock.Mock()
        self.motor.get_motor_pos.return_value = 0.0
        self.motor.get_error_id.return_value = 0
        self.info = {
            "motor": self.motor, "motor_id": 1, "interface": "can0"
        }
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.stack.enter_context(mock.patch.object(set_zero.time, "sleep"))
        self.read_key = self.stack.enter_context(
            mock.patch.object(set_zero, "read_key_nonblocking", return_value="\r")
        )

    def test_confirmation_and_skip_disable_motor(self):
        for key, zeroed in (("\r", True), ("\n", True), (" ", False)):
            with self.subTest(key=repr(key)):
                self.motor.reset_mock()
                self.read_key.return_value = key
                self.assertEqual(set_zero.calibrate_motor(self.info), zeroed)
                self.assertEqual(self.motor.set_motor_zero.call_count, int(zeroed))
                self.motor.deinit_motor.assert_called_once_with()

    def test_errors_disable_motor_and_propagate(self):
        for method in (
            "init_motor", "set_motor_control_mode", "motor_mit_cmd",
            "get_motor_pos", "get_error_id", "set_motor_zero",
        ):
            with self.subTest(method=method):
                self.motor.reset_mock(side_effect=True)
                error = RuntimeError(method)
                getattr(self.motor, method).side_effect = error
                with self.assertRaises(RuntimeError) as caught:
                    set_zero.calibrate_motor(self.info)
                self.assertIs(caught.exception, error)
                self.motor.deinit_motor.assert_called_once_with()

    def test_batch_command_error_disables_motor(self):
        self.info["use_batch_mit"] = True
        self.motor.motors_mit_cmd.side_effect = RuntimeError("batch failed")
        with self.assertRaisesRegex(RuntimeError, "batch failed"):
            set_zero.calibrate_motor(self.info)
        self.motor.deinit_motor.assert_called_once_with()

    def test_terminal_error_disables_motor(self):
        self.read_key.side_effect = OSError("terminal unavailable")
        with self.assertRaisesRegex(OSError, "terminal unavailable"):
            set_zero.calibrate_motor(self.info)
        self.motor.deinit_motor.assert_called_once_with()

    def test_ctrl_c_disables_motor_and_propagates(self):
        self.read_key.return_value = "\x03"
        with self.assertRaises(KeyboardInterrupt):
            set_zero.calibrate_motor(self.info)
        self.motor.deinit_motor.assert_called_once_with()

    def test_interrupt_during_setup_disables_motor(self):
        self.motor.set_motor_control_mode.side_effect = KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            set_zero.calibrate_motor(self.info)
        self.motor.deinit_motor.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
