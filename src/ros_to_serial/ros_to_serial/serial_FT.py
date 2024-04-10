import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial
import time


# sudo dmesg | grep tty :: usb 포트를 확인하는 코드
'''
시리얼 데이터를 읽는 부분
'''
class FTsensor(Node):
    ##### Publisher와 Subscriber를 정의, Serial Port 정보를 정의
    def __init__(self):
        super().__init__('serial_FT')

        qos_profile = QoSProfile(depth=10)
        ser = serial.Serial(
        port='/dev/ttyUSB0',\
        baudrate=115200,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
        timeout=0)
        self.ser = ser

        self.data_read = [0x0A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_bias = [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_baudrate = [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_filter = [0x08, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_name = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        
        self.status = False

        self.force_data = self.create_publisher(
            String,
            'force_data',
            qos_profile)
        self.torque_data = self.create_publisher(
            String,
            'torque_data',
            qos_profile)
        
        self.FT_serial()
        if self.status:
            self.send_data_without_read(self.data_bias)
            self.send_data_without_read(self.data_baudrate)

            time.sleep(0.1)
            self.send_data_without_read(self.data_filter)
            time.sleep(0.1)

            self.create_timer(0.01, self.publish_FT_sensor)

    #  --------------   Publisher def 정의 -------------

    def publish_FT_sensor(self):
        force = String()
        torque = String()
        self.send_data_without_read(self.data_read)
        read_msg = self.ser.read(19)
        force_decoded_data, torque_decoded_data = self.decode_received_data(read_msg)
        if force_decoded_data != False:
            force.data = str(force_decoded_data)
            torque.data = str(torque_decoded_data)
            self.force_data.publish(force)
            self.torque_data.publish(torque)
            self.get_logger().info("FT read: {0}, {1}".format(force.data, torque.data))
        
    # -------------  공통 사용 함수 정의 -----------
        
    def FT_serial(self):
        if self.ser.readable():
            self.status = True
        else:
            self.status = False
        
    def calculate_checksum(self, data):
        """
        데이터의 체크섬을 계산합니다.
        """
        checksum = sum(data) % 256
        return checksum


    def decode_received_data(self, data):
        """
        수신된 데이터에서 Start of Packet (SOP)와 End of Packet (EOP)을 찾아 데이터를 출력합니다.
        """
        sop_index = data.find(bytes([0x55]))  # SOP (Start of Packet)의 인덱스 찾기
        eop_index = data.find(bytes([0xAA]))  # EOP (End of Packet)의 인덱스 찾기
        force_raw = []
        torque_raw = []
        force = []
        torque = []
        try:
            if sop_index != -1 and eop_index != -1:  # SOP와 EOP 모두 존재하는 경우
                received_packet = data[sop_index:eop_index+1]  # SOP부터 EOP까지의 패킷 추출
                # print(received_packet)
                # print(received_packet[0])
                force_raw.append( received_packet[2] << 8 | received_packet[3])
                force_raw.append( received_packet[4] << 8 | received_packet[5])
                force_raw.append( received_packet[6] << 8 | received_packet[7])
                torque_raw.append( received_packet[8] << 8 | received_packet[9])
                torque_raw.append( received_packet[10] << 8 | received_packet[11])
                torque_raw.append( received_packet[12] << 8 | received_packet[13])
                for i in range(0, 3):
                    force_temp  = force_raw[i].to_bytes(2, 'big')
                    torque_temp  = torque_raw[i].to_bytes(2, 'big')
                    # [Divider] Force: 50, Torque: 2000
                    # Note: Resolution of RFT76-HA01 is same with RFT40-SA01
                    force.append(int.from_bytes(force_temp, "big", signed=True) / 50)
                    torque.append(int.from_bytes(torque_temp, "big", signed=True) / 2000)
                # 여기서 데이터 처리를 수행하거나 필요한 작업을 수행할 수 있습니다.
                # print(force)
            return force, torque
            # else:
                # print("Incomplete packet received")
        except:
            return False, False
        
    def send_data_with_read(self, data):
        """
        데이터를 주어진 구조에 맞게 시리얼 포트로 전송합니다.
        """
        sop = bytes([0x55])  # SOP (Start of Packet)
        eop = bytes([0xAA])  # EOP (End of Packet)

        # 데이터와 체크섬 조합
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop

        # 시리얼 포트로 전송
        self.ser.write(packet)
        # print("send packet: {0}".format(packet.hex()))
        sent_data = self.ser.read(19)
        return sent_data

    def send_data_without_read(self, data):
        """
        데이터를 주어진 구조에 맞게 시리얼 포트로 전송합니다.
        """
        sop = bytes([0x55])  # SOP (Start of Packet)
        eop = bytes([0xAA])  # EOP (End of Packet)

        # 데이터와 체크섬 조합
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop

        # 시리얼 포트로 전송
        self.ser.write(packet)


def main(args=None):
    rclpy.init(args=args)
    node_read = FTsensor()

    try:
        rclpy.spin(node_read)
    except KeyboardInterrupt:
        node_read.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node_read.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


        
# # 메인 루프
# if __name__ == "__main__":
    
#     try:
#         send_data_without_read(data_bias)
#         send_data_without_read(data_baudrate)
#         # print(data_bias)
#         time.sleep(0.1)
#         send_data_without_read(data_filter)
#         time.sleep(0.1)
        
#         while True:
#             # Note: 만약 0x0B(Command ID) 일 경우 Continuous data 이므로 while문 밖으로 빼기
#             # name_msg = send_data_with_read(data_name)
#             send_data_without_read(data_read)
#             read_msg = ser.read(19)
#             decode_received_data(read_msg)
#             time.sleep(0.01)

#     except KeyboardInterrupt:
#         print("프로그램을 종료합니다.")
#         ser.close()  # 시리얼 포트 닫기