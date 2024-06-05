import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial


# sudo dmesg | grep tty :: usb 포트를 확인하는 코드
'''
시리얼 데이터를 읽는 부분
'''
class ESP32Board(Node):
    ##### Publisher와 Subscriber를 정의, Serial Port 정보를 정의
    def __init__(self):
        super().__init__('esp_serial')
        qos_profile = QoSProfile(depth=10)
        ser = serial.Serial(
            '/dev/ttyAMA1',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)
        self.ser = ser

        self.status = False


        self.i2c_write = self.create_publisher(
            String,
            'i2c_serial_1',
            qos_profile)
        
        self.pwm_read = self.create_subscription(
            String,
            'pwm_write_1',
            self.pwm_reader,
            qos_profile)
        
        self.esp_serial()
        if self.status:
            self.create_timer(0.05, self.publish_serial)


    #  --------------   Publisher def 정의 -------------

    def publish_serial(self):
        i2c_data = String()
        EncodeData = self.ser.readline().decode()[0:-1]
        # print(EncodeData)
        i2c_data.data = str(EncodeData)
        self.i2c_write.publish(i2c_data)
        self.get_logger().info("i2c read_1: {0}".format(i2c_data.data))
        

    # -------------   Subscriber def 정의 ---------------
        
    def pwm_reader(self, msg):
        send_data = str(msg.data)
        Trans="Q" + send_data + "W"
        Trans= Trans.encode()
        self.ser.write(Trans)
        # self.get_logger().info('pwm write: {0}'.format(msg.data))
        
    # -------------  공통 사용 함수 정의 -----------
        
    def esp_serial(self):
        if self.ser.readable():
            self.status = True
        else:
            self.status = False

def main(args=None):
    rclpy.init(args=args)
    node_read = ESP32Board()

    try:
        rclpy.spin(node_read)
    except KeyboardInterrupt:
        node_read.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node_read.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
