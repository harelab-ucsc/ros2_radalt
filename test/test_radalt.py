"""
Tests for radalt/main.py.

decodePacket tests are pure-Python (numpy only) — no ROS 2 runtime needed.
Serial-loop tests mock rclpy + serial to verify that read_until() is used
instead of the old byte-by-byte busy-poll that consumed 100 % CPU.
"""
import sys
import types
import unittest
from unittest.mock import MagicMock, call, patch

import numpy as np

# ---------------------------------------------------------------------------
# Shim custom_msgs so the module loads in environments where the workspace
# hasn't been built yet (e.g. local dev without colcon build).  In Docker
# after `colcon build + source install/setup.sh` the real package is present
# and setdefault is a no-op.
# ---------------------------------------------------------------------------
if "custom_msgs" not in sys.modules:
    _cm = types.ModuleType("custom_msgs")
    _cm_msg = types.ModuleType("custom_msgs.msg")

    class _AltSNRShim:
        def __init__(self):
            self.header = MagicMock()
            self.altitude = 0.0
            self.snr = 0

    _cm_msg.AltSNR = _AltSNRShim
    _cm.msg = _cm_msg
    sys.modules["custom_msgs"] = _cm
    sys.modules["custom_msgs.msg"] = _cm_msg

from radalt.main import SIZE, decodePacket, talker  # noqa: E402


# ---------------------------------------------------------------------------
# Helper: build a valid 5-byte payload matching the radalt wire format.
#
# Wire layout decoded by decodePacket:
#   [0]  padding / unused
#   [1]  altitude low byte
#   [2]  altitude high byte
#   [3]  SNR
#   [4]  checksum = sum(bytes 0–3) & 0xFF
# ---------------------------------------------------------------------------

def _build_packet(alt_cm: int, snr: int, p0: int = 0x00) -> np.ndarray:
    p1 = alt_cm & 0xFF
    p2 = (alt_cm >> 8) & 0xFF
    p3 = snr & 0xFF
    p4 = (p0 + p1 + p2 + p3) & 0xFF
    return np.array([p0, p1, p2, p3, p4], dtype=np.uint8)


def _node():
    n = MagicMock()
    n.get_logger.return_value.info = MagicMock()
    return n


# ---------------------------------------------------------------------------
# decodePacket unit tests
# ---------------------------------------------------------------------------

class TestDecodePacket(unittest.TestCase):

    def test_valid_high_snr(self):
        """Good checksum + SNR > 13 returns (1, altitude_cm, snr)."""
        pkt = _build_packet(1234, 50)
        result = decodePacket(pkt, _node())
        self.assertEqual(result[0], 1)
        self.assertEqual(int(result[1]), 1234)
        self.assertEqual(int(result[2]), 50)

    def test_valid_low_snr(self):
        """Good checksum but SNR ≤ 13 returns (0,) and logs."""
        node = _node()
        pkt = _build_packet(500, 10)
        self.assertEqual(decodePacket(pkt, node), (0,))
        node.get_logger().info.assert_called_once()

    def test_snr_boundary_below(self):
        """SNR == 13 is below the > 13 threshold."""
        self.assertEqual(decodePacket(_build_packet(100, 13), _node()), (0,))

    def test_snr_boundary_above(self):
        """SNR == 14 just clears the threshold."""
        self.assertEqual(decodePacket(_build_packet(100, 14), _node())[0], 1)

    def test_bad_checksum(self):
        """Corrupted last byte returns (0,) and logs."""
        node = _node()
        pkt = _build_packet(1000, 30)
        pkt[-1] ^= 0xFF
        self.assertEqual(decodePacket(pkt, node), (0,))
        node.get_logger().info.assert_called_once()

    def test_zero_altitude(self):
        """Zero altitude encodes and decodes correctly."""
        result = decodePacket(_build_packet(0, 20), _node())
        self.assertEqual(result[0], 1)
        self.assertEqual(int(result[1]), 0)

    def test_max_altitude(self):
        """Full uint16 altitude round-trips without overflow."""
        result = decodePacket(_build_packet(0xFFFF, 30), _node())
        self.assertEqual(result[0], 1)
        self.assertEqual(int(result[1]), 0xFFFF)


# ---------------------------------------------------------------------------
# Serial-loop behaviour tests
# ---------------------------------------------------------------------------

class TestSerialLoop(unittest.TestCase):
    """
    Verify the refactored talker() loop uses serial.read_until().

    Checks that the sync byte is located with read_until() instead of the
    old byte-by-byte busy-poll.
    """

    def _run_one_cycle(self, payload=None):
        """Patch rclpy + serial, run talker() for one packet, return mock device."""
        if payload is None:
            payload = _build_packet(500, 50).tobytes()

        mock_device = MagicMock()
        mock_device.read_until.return_value = b'\xfe'
        mock_device.read.return_value = payload

        mock_node = MagicMock()
        mock_node.declare_parameter.return_value.value = '/dev/null'

        # rclpy.ok() allows exactly one loop iteration then exits
        _count = [0]

        def _ok():
            _count[0] += 1
            return _count[0] <= 1

        with patch("radalt.main.rclpy") as mock_rclpy, \
                patch("radalt.main.serial.Serial", return_value=mock_device), \
                patch("radalt.main.threading.Thread") as mock_thread:

            mock_rclpy.ok.side_effect = _ok
            mock_rclpy.create_node.return_value = mock_node
            mock_thread.return_value.join = MagicMock()

            talker()

        return mock_device

    def test_read_until_called_with_sync_byte(self):
        r"""Loop must call read_until(b'\xfe') to locate each packet start."""
        device = self._run_one_cycle()
        device.read_until.assert_called_with(b'\xfe')

    def test_no_bare_single_byte_read(self):
        """No bare read() call — that was the busy-spin pattern."""
        device = self._run_one_cycle()
        for c in device.read.call_args_list:
            self.assertNotEqual(
                c, call(),
                "bare read() detected — old busy-spin is still present",
            )

    def test_payload_read_uses_size_constant(self):
        """Payload must be fetched as a single block of SIZE bytes."""
        device = self._run_one_cycle()
        device.read.assert_called_with(SIZE)

    def test_short_read_skips_decode(self):
        """A truncated payload (len < SIZE) must not reach decodePacket."""
        with patch("radalt.main.rclpy") as mock_rclpy, \
                patch("radalt.main.serial.Serial") as mock_serial_cls, \
                patch("radalt.main.threading.Thread"), \
                patch("radalt.main.decodePacket") as mock_decode:

            mock_device = MagicMock()
            mock_device.read_until.return_value = b'\xfe'
            mock_device.read.return_value = b'\x00\x01'  # only 2 bytes
            mock_serial_cls.return_value = mock_device

            mock_node = MagicMock()
            mock_node.declare_parameter.return_value.value = '/dev/null'

            _count = [0]

            def _ok():
                _count[0] += 1
                return _count[0] <= 1

            mock_rclpy.ok.side_effect = _ok
            mock_rclpy.create_node.return_value = mock_node

            talker()

        mock_decode.assert_not_called()


if __name__ == "__main__":
    unittest.main()
