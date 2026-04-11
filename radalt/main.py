import rclpy
import serial
from custom_msgs.msg import AltSNR
from copy import deepcopy
import threading
import numpy as np

SIZE = 5  # bytes after head

# Steve says make a custom message to hold 2 named items: altitude, m (float) and SNR (UInt8)
# create blank msg to populate instead of the zeros list + deepcopies
# next interpret the msg as a struct (builtin package)
# sensor is format string '>H'
# convert int from tuple to float through conversion to meters, stuff into msg


def decodePacket(packet, node):
    check = 0x00
    for item in packet[:-1]:
        check += item
    check &= 0xFF
    if check == packet[-1]:
        # alt = ((np.uint16(packet[2]) << 8) + np.uint16(packet[1])).astype(np.uint16)
        alt = ((np.uint16(packet[2]) << 8) + np.uint16(packet[1]))
        snr = np.uint8(packet[-2])
        if snr > 13:
            return (1, alt, snr)
        else:
            error_msg = (
                'altimeter SNR below manufacturer-defined minimum '
                'threshold (13dB); packet dumped'
            )
            node.get_logger().info(error_msg)
            return (0,)
    else:
        error_msg = 'decoding checksum failed; packet dumped'
        node.get_logger().info(error_msg)
        return (0,)


def talker():
    rclpy.init()
    node = rclpy.create_node('alt_pub')
    # super().__init__('alt_pub')
    pub = node.create_publisher(AltSNR, 'rad_altitude', 10)
    # timer_period = 100  # hz
    # self.timer = self.create_timer(timer_period, self.timer_callback)
    rate = node.create_rate(100)

    port = node.declare_parameter('port', '/dev/devRADALT').value
    # assert isinstance(port, str)
    device = serial.Serial(port=port, baudrate=115200, timeout=1.0)

    blank_packet = [0 for _ in range(SIZE)]
    packet = deepcopy(blank_packet)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    while rclpy.ok():
        val = device.read()  # figure out how this interacts with timeout
        if val == b'\xfe':
            val = device.read(SIZE)
            packet = np.frombuffer(val, dtype=np.uint8)

            ret = decodePacket(packet, node)
            msg = AltSNR()
            if ret[0]:
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'radalt'
                msg.altitude = float(ret[1]/100)
                msg.snr = int(ret[2])
                pub.publish(msg)
            # self.timer.sleep()
            rate.sleep()
        else:
            pass
    thread.join()


def main():
    try:
        talker()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
