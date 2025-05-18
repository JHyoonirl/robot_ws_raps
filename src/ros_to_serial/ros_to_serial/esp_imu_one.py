import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import serial


# sudo dmesg | grep tty :: usb 포트를 확인하는 코드
'''
시리얼 데이터를 읽는 부분
'''
class ESP32Board(Node):
    ##### Publisher와 Subscriber를 정의, Serial Port 정보를 정의
    def __init__(self):
        super().__init__('IMU_node')
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


        self.i2c_write = self.create_publisher(
            Vector3,
            'imu_data_shank',
            qos_profile)
        
        self.imu_data_list = [0, 0, 0, 0] # imu number / roll / pitch / yaw
        
        self.esp_serial()
        if self.status:
            self.create_timer(0.005, self.read_serial_data)


    #  --------------   Publisher def 정의 -------------


    def read_serial_data(self):
        data_buffer = ''
        data_in = self.ser.read(self.ser.in_waiting or 1).decode('utf-8', errors='ignore')
        if data_in:
            data_buffer += data_in
            if '\n' in data_buffer:
                lines = data_buffer.split('\n')
                for line in lines[:-1]:
                    self.process_data(line)
                data_buffer = lines[-1]

    def process_data(self, data):
    # 데이터 처리 로직
        # print(data)  # 또는 다른 처리
        # self.get_logger().info("{0}".format(data))
        self.publish_Imu(data)  

    def publish_Imu(self, data):
        imu_data = Vector3()
        EncodeData = data
        EncodedData_indexes = []
        raw_data = []
        # raw_data_index = []
        data_list = []
        try:
            # EncodeData = self.ser.readline().decode()[0:-1]
            find_index_list = ["i", "r", "p", "y"]
            # self.get_logger().info("{0}".format(EncodeData))  

            for i, find_index in enumerate(find_index_list): # index number(start with 0) / string to find out( ex: imu)
                EncodedData_indexes.append(EncodeData.find(find_index))
            

            for i, EncodedData_index in enumerate(EncodedData_indexes): # EncodData_index = [0, 4, 8, 10]
                if i == 3:
                    raw_data = EncodeData[EncodedData_index:-1]
                else:
                    raw_data = EncodeData[EncodedData_index:EncodedData_indexes[i+1]]
                raw_data_index = raw_data.find(":")
                data_list.append(raw_data[raw_data_index+1:])
            
            for i, data in enumerate(data_list):
                # status = False
                try:
                    data = float(data)
                    self.imu_data_list[i] = data
                    # status = True
                except:
                    pass
            

            for i, data in enumerate(self.imu_data_list):
                
                if i == 1:
                    imu_data.x = float(data)
                elif i == 2:
                    imu_data.y = float(data)
                elif i == 3:
                    imu_data.z = float(data)
            
            
            self.i2c_write.publish(imu_data)
            self.get_logger().info("{0}".format(imu_data))
        except Exception as e:
            self.get_logger().info("error{0}".format(e))
        
        
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
