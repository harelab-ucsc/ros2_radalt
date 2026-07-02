import time

import rclpy
from rclpy.node import Node
import serial

from ros2_radalt_msgs.msg import AltSNR

SYNC = 0xFE
PAYLOAD_SIZE = 5


class RadAltNode(Node):

    def __init__(self):
        super().__init__("radalt")

        self.publisher = self.create_publisher(
            AltSNR,
            "rad_altitude",
            10,
        )

        self.port = self.declare_parameter(
            "port",
            "/dev/devRADALT",
        ).value

        self.device = None

    def connect(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Opening {self.port}")

                self.device = serial.Serial(
                    port=self.port,
                    baudrate=115200,
                    timeout=1.0,
                )

                self.reset_input_buffer()

                self.get_logger().info("Connected.")
                return

            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port: {e}")
                time.sleep(1.0)

    def reset_input_buffer(self):
        try:
            self.device.reset_input_buffer()
        except Exception:
            pass

    def read_packet(self):

        #
        # Wait for sync byte.
        #
        while rclpy.ok():

            b = self.device.read(1)

            if len(b) == 0:
                continue

            if b[0] == SYNC:
                break

        payload = self.device.read(PAYLOAD_SIZE)

        if len(payload) != PAYLOAD_SIZE:
            return None

        return payload

    def decode(self, payload):

        checksum = sum(payload[:-1]) & 0xFF

        if checksum != payload[4]:
            self.get_logger().warn("Checksum failure.")
            return None

        altitude_cm = payload[1] | (payload[2] << 8)
        snr = payload[3]

        if snr <= 13:
            return None

        return altitude_cm / 100.0, snr

    def run(self):

        self.connect()

        while rclpy.ok():

            try:

                payload = self.read_packet()

                if payload is None:
                    continue

                decoded = self.decode(payload)

                if decoded is None:
                    continue

                altitude, snr = decoded

                msg = AltSNR()

                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "radalt"

                msg.altitude = altitude
                msg.snr = snr

                self.publisher.publish(msg)

            except serial.SerialException as e:

                self.get_logger().error(f"Serial error: {e}")

                try:
                    self.device.close()
                except Exception:
                    pass

                time.sleep(1.0)
                self.connect()

        if self.device is not None:
            self.device.close()


def main():

    rclpy.init()

    node = RadAltNode()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
