import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from custominterface.srv import Status
from std_msgs.msg import String, Float64
import serial

class thrusterServer(Node):

    def __init__(self):
        super().__init__('thruster_node')
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

        self.resolution = 12
        # self.thruster_range = pow(2,16)
        # thruster initialize
        self.thruster_neut = float(50)
        self.thruster = self.thruster_neut
        self.srv = self.create_service(
            Status,
            'thruster_server',
            self.thruster_server
        )
        
        self.thruster_publisher = self.create_subscription(
            Float64,
            'thruster_signal',
            self.thruster_subscriber,
            qos_profile)
        
        self.create_timer(0.005, self.serial_read)

    def thruster_server(self, request, response):
        if request.thruster_switch == True:
            response.thruster_result = True
        elif request.thruster_switch == False:
            response.thruster_result = False
        self.response = response.thruster_result
        # print(self.response)
        return response

    def thruster_subscriber(self, msg):
        # self.get_logger().info('Subscribed thruster: {0}'.format(msg.data))
        thruster_signal = str(self.scaling_fcn(msg.data))
        if self.status:
            self.thruster_trans(thruster_signal)
        else:
            self.thruster_trans(str(int(self.scaling_fcn(50))))

    def scaling_fcn(self, signal):
        return int(signal * 10000)
    
    def thruster_trans(self, signal):
        send_data = signal
        Trans="Q" + send_data + "W"
        Trans= Trans.encode()
        self.ser.write(Trans)
        EncodeData = self.ser.readline().decode()[0:-1]
        self.get_logger().info("serial read: {0}".format(EncodeData))
        # self.get_logger().info('thruster signal: {0}'.format(send_data))

    def serial_read(self):
        pass

    def esp_serial(self):
        if self.ser.readable():
            self.status = True
            print("Serial communication Set-up clear")
        else:
            self.status = False
            print("Serial communication Set-up denied")

def main(args=None):
    rclpy.init(args=args)
    node = thrusterServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()