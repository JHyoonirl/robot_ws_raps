import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial

ser = serial.Serial(
            '/dev/ttyUSB0',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)


'''
시리얼 데이터를 읽는 부분
'''
class I2cRead(Node):

    def __init__(self):
        super().__init__('serial_read')
        qos_profile = QoSProfile(depth=10)
        
        self.i2c_read = self.create_publisher(
            String,
            'i2c_serial',
            qos_profile)
        
        self.status = False
        self.connect_esp(ser)
        if self.status:
            self.create_timer(0.01, self.publish_serial)

    def connect_esp(self, ser):
        self.ser = ser
        if self.ser.readable():
            self.status = True
        else:
            self.status = False

    def publish_serial(self):
        i2c_data = String()
        EncodeData = self.ser.readline().decode()
        i2c_data.data = str(EncodeData)
        self.i2c_read.publish(i2c_data)
        self.get_logger().info("read: {0}".format(i2c_data.data))

def main(args=None):
    rclpy.init(args=args)
    node_read = I2cRead()

    try:
        rclpy.spin(node_read)
    except KeyboardInterrupt:
        node_read.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node_read.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
