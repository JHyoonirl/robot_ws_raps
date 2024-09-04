import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from custominterface.srv import Status
from std_msgs.msg import String, Float64
import serial

class PwmServer(Node):

    def __init__(self):
        super().__init__('motor_node')
        qos_profile = QoSProfile(depth=10)

        self.declare_parameter('usb_port', '/dev/ttyUSB0')  # 기본값을 제공

        usb_port = self.get_parameter('usb_port').get_parameter_value().string_value

        ser = serial.Serial(
            port = usb_port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)
        self.ser = ser
        self.status = False
        self.esp_serial()

        self.resolution = 16
        # self.pwm_range = pow(2,16)
        # pwm initialize
        self.pwm_neut = int(50)
        self.pwm = self.pwm_neut
        self.srv = self.create_service(
            Status,
            'pwm_server',
            self.pwm_server
        )
        
        self.pwm_publisher = self.create_subscription(
            Float64,
            'pwm_signal',
            self.pwm_subscriber,
            qos_profile)
        
        self.create_timer(0.01, self.serial_read)

    def pwm_server(self, request, response):
        if request.pwm_switch == True:
            response.pwm_result = True
        elif request.pwm_switch == False:
            response.pwm_result = False
        self.response = response.pwm_result
        print(self.response)
        return response

    def pwm_subscriber(self, msg):
        # self.get_logger().info('Subscribed pwm: {0}'.format(msg.data))
        self.pwm_signal = str(msg.data)
        if self.status:
            self.pwm_trans()
          
    def pwm_trans(self):
        send_data = self.pwm_signal
        Trans="Q" + send_data + "W"
        Trans= Trans.encode()
        self.ser.write(Trans)
        # self.get_logger().info('pwm signal: {0}'.format(send_data))

    def serial_read(self):
        EncodeData = self.ser.readline().decode()[0:-1]
        self.get_logger().info("serial read: {0}".format(EncodeData))

    def esp_serial(self):
        if self.ser.readable():
            self.status = True
            print("Serial communication Set-up clear")
        else:
            self.status = False
            print("Serial communication Set-up denied")

def main(args=None):
    rclpy.init(args=args)
    node = PwmServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()