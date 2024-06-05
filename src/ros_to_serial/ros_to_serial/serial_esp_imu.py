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
            Vector3,
            'imu_data',
            qos_profile)
        
        self.imu_data_list = [0, 0, 0]
        
        self.esp_serial()
        if self.status:
            self.create_timer(0.01, self.publish_serial)


    #  --------------   Publisher def 정의 -------------

    def publish_serial(self):
        imu_data = Vector3()
        EncodeData = ""
        EncodedData_indexes = []
        raw_data = []
        # raw_data_index = []
        data_list = []
        EncodeData = self.ser.readline().decode()[0:-1]
        # print(EncodeData)
        find_index_list = ["roll:", "pitch:", "yaw:"]    

        for i, find_index in enumerate(find_index_list):
            EncodedData_indexes.append(EncodeData.find(find_index))

        for i, EncodedData_index in enumerate(EncodedData_indexes):
            if i == 2:
                raw_data = EncodeData[EncodedData_index:-1]
            else:
                raw_data = EncodeData[EncodedData_index:EncodedData_indexes[i+1]]
            raw_data_index = raw_data.find(":")
            data_list.append(raw_data[raw_data_index+1:-1])
        for i, data in enumerate(data_list):
            # status = False
            try:
                data = float(data)
                self.imu_data_list[i] = data
                # status = True
            except:
                pass

        for i, data in enumerate(self.imu_data_list):
            if i == 0:
                imu_data.x = float(data)
            elif i == 1:
                imu_data.y = float(data)
            elif i == 2:
                imu_data.z = float(data)
        
        self.i2c_write.publish(imu_data)
        self.get_logger().info("i2c read_1: {0}".format(imu_data))
        

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
