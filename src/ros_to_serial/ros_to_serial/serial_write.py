import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial


class PwmWrite(Node):

    def __init__(self):
        super().__init__('serial_read')
        qos_profile = QoSProfile(depth=10)
        ser = serial.Serial(
            '/dev/ttyUSB0',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)
        self.ser = ser


        self.i2c_read = self.create_subscription(
            String,
            'pwm_write',
            self.serial_pwm,
            qos_profile)
        
        # self.status = False
        # self.connect_esp(ser)
        # if self.status:
        #     self.create_timer(0.01, self.serial_pwm)

    # def connect_esp(self, ser):
    #     self.ser = ser
    #     if self.ser.readable():
    #         self.status = True
    #     else:
    #         self.status = False

    def serial_pwm(self, msg):
        self.get_logger().info('Received pwm: {0}'.format(msg.data))
        self.ser.write(str(msg.data).encode())


def main(args=None):
    rclpy.init(args=args)
    node_write = PwmWrite()

    try:
        rclpy.spin(node_write)
    except KeyboardInterrupt:
        node_write.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node_write.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
