import sys
import os

sys.path.append(os.path.expanduser('/home/irl/robot_ws_raps'))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from custominterface.srv import Status
from std_msgs.msg import String, Float64, Float64MultiArray
from geometry_msgs.msg import Vector3

from custom_module.thruster_torque_converter import thruster_converter
from custom_module.hydro_compensation import HydroCompensation
import serial
import time
from dataclasses import dataclass
import numpy as np

@dataclass
class PIDGains:
    proportional: float = 0.0
    integral: float = 0.0
    derivative: float = 0.0

@dataclass
class ROMConfig:
    upper: float = 0.0
    lower: float = 0.0

@dataclass
class ControlError:
    errorprev: float = 0.0
    errorintegral: float = 0.0
    errorderivative: float = 0.0

class thrusterServer(Node):

    def __init__(self):
        super().__init__('thruster_node')
        self.qos_profile = QoSProfile(depth=10)

        self.declare_parameter('usb_port', '/dev/ttyAMA4')  # 기본값을 제공

        usb_port = self.get_parameter('usb_port').get_parameter_value().string_value

        ser = serial.Serial(
            port = usb_port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.1)
        self.ser = ser
        self.status = False
        self.esp_serial()

        self.torque_converter = thruster_converter()
        self.hydro_compensater = HydroCompensation()
        self.total_input_torque = self.added_mass = self.drag = 0
        self.input_thruster_generate_torque = 0
        self.assistance_velocity_min = 0

        self.thruster_pwm_input = float(50)
        self.thruster_moment_variation_saturation = 100
        self.thruster_moment_memory = 0
        self.thruster_moment_memory_prev = 0

        self.thruster_total_moment_publisher = self.create_publisher(Float64, 'thruster_total_moment', self.qos_profile)

        self.thruster_moment_publisher = self.create_publisher(Float64, 'thruster_moment', self.qos_profile)

        self.added_mass_publisher = self.create_publisher(Float64, 'added_mass', self.qos_profile)

        self.drag_publisher = self.create_publisher(Float64, 'drag', self.qos_profile)

        self.exercise_desired_trajectory_position_subscriber = self.create_subscription(
            Float64,
            'desired_trajectory_position',
            self.exercise_desired_trajectory_position_callback,
            self.qos_profile)
        
        self.exercise_desired_trajectory_velocity_subscriber = self.create_subscription(
            Float64,
            'desired_trajectory_velocity',
            self.exercise_desired_trajectory_velocity_callback,
            self.qos_profile)

        self.assistance_velocity_min_subscriber = self.create_subscription(
            Float64,
            'assistance_velocity_min',
            self.assistance_velocity_min_callback,
            self.qos_profile)
        
        self.exercise_trajectory_state_subscriber = self.create_subscription(
            Float64,
            'trajectory_state',
            self.exercise_trajectory_state_callback,
            self.qos_profile)
        
        self.exercise_info_subscriber = self.create_subscription(
            Float64MultiArray,
            'Exercise_info',
            self.exercise_info_callback,
            self.qos_profile)
        
        self.passive_parameter_subscriber = self.create_subscription(
            Float64MultiArray,
            'Passive_parameter',
            self.passive_parameter_callback,
            self.qos_profile)
        
        self.assistance_parameter_subscriber = self.create_subscription(
            Float64MultiArray,
            'Assistance_parameter',
            self.assistance_parameter_callback,
            self.qos_profile)
        
        self.resistance_moment_subscriber = self.create_subscription(
            Float64,
            'resistance_moment',
            self.resistance_moement_callback,
            self.qos_profile)
        
        self.imu_knee_angle_subscriber = self.create_subscription(
            Vector3,
            'imu_data_shank',
            self.imu_knee_angle_callback,
            self.qos_profile)

        self.imu_velocity_subscriber = self.create_subscription(
            Float64,
            'imu_data_velocity',
            self.imu_velocity_callback,
            self.qos_profile)
        
        self.imu_acceleration_subscriber = self.create_subscription(
            Float64,
            'imu_data_acceleration',
            self.imu_acceleration_callback,
            self.qos_profile)
        
        self.motor_setting_parameter_subscriber = self.create_subscription(
            Float64MultiArray, 'Motor_setting_parameter', self.motor_setting_parameter_callback, self.qos_profile
        )
        

        ###########################
        ## exercise trajectroy ####
        ###########################
        self.desired_trajectory_position_deg = 0.0
        '''
        deg
        '''
        self.desired_trajectory_velocity_deg = 0.0
        '''
        deg/s
        '''
        self.desired_trajectory_state = 0.0
        '''
        0: stop
        1: start
        2: hold
        '''

        ### exercise info
        self.power_enabled = 0
        '''
        0. off
        1. on
        '''
        # power 관련 내부 변수
        self.power_enabled_prev = None
        self.power_locked = False  # 안전장치 잠금 상태
        self.raw_power_enabled = 0
        self.raw_power_enabled_prev = 0  # 이전 입력 상태 추적
        
        self.control_active = 0
        self.control_active_prev = 0
        '''
        0. stop
        1. start
        '''       

        self.control_mode = 0
        '''
        1. extension constant velocity
        2. sine
        3. passive exercise
        4. assistance exercise
        5. resistance exercise
        6. angle move
        '''
        #### imu variables
        
        self.imu_knee_angle_deg = 0.0
        '''
        deg
        '''
        self.imu_knee_velocity_deg = 0.0
        '''
        deg/s
        '''
        self.imu_knee_acceleration_deg = 0.0
        '''
        deg/S^2
        '''
        self.rom_safe_upper = 0.0 # motor의 안전 upper ROM
        self.rom_safe_lower = 0.0 # motor의 안전 lower ROM

        #### passive parameter
        self.passive_gains = PIDGains()
        self.passive_err_state = ControlError()

        #### assistance parameter
        self.assistance_gain_k = 0.0
        self.assistance_gains = PIDGains()
        self.assistance_err_state = ControlError()

        self.assistance_rate = 3

        #### resistance parameter
        self.resistance_moment = 0.0
        self.resistance_moment_memory = 0.0
        self.resistance_moment_variation = 2
        '''
        1초당 허용 resistance_moment 변화율
        '''
        self.input_hydro_torque = 0
        self.past_time = time.time()

        # Initialize Low Pass Filter parameters
        self.filter_alpha = 0.7  # Smoothing factor (0 < alpha <= 1)
        self.filter_assist_alpha = 0.1  # Smoothing factor for assistance control
        self.resistance_alpha = 0.1
        self.filtered_input_hydro_torque = 0.0  # Initialize filtered value

        self.create_timer(0.0105, self.thruster_control_loop)
        self.response = True

    def thruster_control_loop(self):
        status = self.thruster_control_fcn()
        self.info_pub()
    
    def info_pub(self):
        if self.power_enabled == 1:
            
            self.added_mass_publisher.publish(Float64(data=float(self.added_mass)))
            self.drag_publisher.publish(Float64(data=float(self.drag)))
            self.thruster_moment_publisher.publish(Float64(data=float(self.input_thruster_generate_torque)))
            self.thruster_total_moment_publisher.publish(Float64(data=float(self.total_input_torque)))
        else:
            
            self.added_mass_publisher.publish(Float64(data=0.0))
            self.drag_publisher.publish(Float64(data=0.0))
            self.thruster_moment_publisher.publish(Float64(data=0.0))
            self.thruster_total_moment_publisher.publish(Float64(data=0.0))


    def thruster_control_fcn(self):
        try:
            # knee_angle = self.imu_knee_angle_deg
            if self.rom_safe_upper < self.imu_knee_angle_deg or self.rom_safe_lower > self.imu_knee_angle_deg:
                self.power_enabled = 0
                self.power_locked = True
                
            if self.power_enabled == 0:
                
                if self.power_enabled_prev == self.power_enabled:
                    return self.power_enabled
                else:
                    self.power_enabled_prev = self.power_enabled

                self.thruster_duty_ratio_input(50)
                self.get_logger().info('Thruster off')
                time.sleep(0.005)
                
                return self.power_enabled

            if self.control_mode == 3:
                self.input_thruster_generate_torque = self.passive_pid_control_fcn()
                raw_input_hydro_torque, self.added_mass, self.drag = self.hydro_compensater.calculate_total_moment(
                    self.imu_knee_velocity_deg, self.imu_knee_acceleration_deg
                )
                self.input_hydro_torque = raw_input_hydro_torque


                if self.control_active == 1:
                    self.get_logger().info(f'input pid: {self.input_thruster_generate_torque}')
                    total_input_torque = self.input_thruster_generate_torque - self.input_hydro_torque

                    self.total_input_torque = (
                        self.filter_alpha * total_input_torque
                        + (1 - self.filter_alpha) * self.total_input_torque
                        )
                else:
                    self.total_input_torque = 0

                self.thruster_moment_memory_prev = self.total_input_torque
                input = self.torque_converter.torque_to_percentage(self.total_input_torque)

                self.get_logger().info(f'total input: {input}')
                self.thruster_duty_ratio_input(input)

            elif self.control_mode == 4:
                
                if self.control_active == 1:
                    if self.desired_trajectory_state == 1:
                        if (self.assistance_velocity_min > 0 and self.assistance_velocity_min > self.imu_knee_velocity_deg) or (self.assistance_velocity_min < 0 and self.assistance_velocity_min < self.imu_knee_velocity_deg):
                            
                            velocity_error_rad = np.deg2rad(self.assistance_velocity_min - self.imu_knee_velocity_deg)
                            self.input_thruster_generate_torque = self.assistance_gain_k * velocity_error_rad

                        if self.assistance_velocity_min == 0:
                            self.input_thruster_generate_torque = 0
                            
                        self.assistance_err_state = ControlError()
                    else:  # self.desired_trajectory_state == 2
                        self.input_thruster_generate_torque = self.assistance_pid_control_fcn()
                        self.get_logger().info(f'assistance pid: {self.input_thruster_generate_torque}')
                    # self.total_input_torque = self.input_thruster_generate_torque

                    

                    # Apply Low Pass Filter to total_input_torque
                    self.total_input_torque = (
                        self.filter_assist_alpha * self.input_thruster_generate_torque
                        + (1 - self.filter_assist_alpha) * self.total_input_torque
                        )
                
                else:
                    self.assistance_err_state = ControlError()
                    self.total_input_torque = 0
                    self.input_thruster_generate_torque = 0

                input = self.torque_converter.torque_to_percentage(self.total_input_torque)
                self.get_logger().info(f'assistance input: {input}')
                self.thruster_duty_ratio_input(input)
     
            elif self.control_mode == 5:
                if self.control_active == 1:
                    # if self.desired_trajectory_state == 1:
                    input_hydro_moment, self.added_mass, self.drag = self.hydro_compensater.calculate_total_moment(self.imu_knee_velocity_deg, self.imu_knee_acceleration_deg)
                    # self.resistance_moment = self.resistance_moment_generator_fcn()
                    self.get_logger().info(f'resistance moment: {self.resistance_moment}')
                    raw_total_input_torque = self.resistance_moment  - input_hydro_moment

                    if self.resistance_moment > 0 and raw_total_input_torque < 0:
                        raw_total_input_torque = 0
                    if raw_total_input_torque > self.resistance_moment and raw_total_input_torque > 0:
                        raw_total_input_torque = self.resistance_moment
                    
                    if self.resistance_moment < 0 and raw_total_input_torque > 0:
                        raw_total_input_torque = 0

                    if raw_total_input_torque < self.resistance_moment and raw_total_input_torque < 0:
                        raw_total_input_torque = self.resistance_moment

                    self.total_input_torque = (
                                self.resistance_alpha * raw_total_input_torque
                                + (1 - self.resistance_alpha) * self.total_input_torque
                            )

                else:
                    self.total_input_torque = 0
                input = self.torque_converter.torque_to_percentage(self.total_input_torque)
                # self.get_logger().info(f'resistance input: {input}')
                self.thruster_duty_ratio_input(input)
                
            else:
                self.thruster_duty_ratio_input(50)

            self.past_time = time.time()
            self.power_enabled_prev = self.power_enabled
            return self.power_enabled

        except Exception as e:
            self.thruster_duty_ratio_input(50)
            self.get_logger().error(f"Error in thruster_control_fcn: {e}")
            return

    def thruster_moment_saturation_fcn(self, u_prev, u_desired, u_rate):
        
        dt = time.time() - self.past_time
        
        max_delta = u_rate * dt
        delta_u = u_desired - u_prev

        # saturation 적용
        if delta_u > max_delta:
            delta_u = max_delta
        elif delta_u < -max_delta:
            delta_u = -max_delta

        # 제한된 제어 입력 계산
        u_new = u_prev + delta_u
        return u_new
        

    def passive_pid_control_fcn(self):
        self.dt = time.time() - self.past_time
        self.desired_trajectory_position_rad = np.deg2rad(self.desired_trajectory_position_deg)
        self.imu_knee_angle_rad = np.deg2rad(self.imu_knee_angle_deg)
        self.desired_trajectory_velocity_rad = np.deg2rad(self.desired_trajectory_velocity_deg)
        self.imu_knee_velocity_rad = np.deg2rad(self.imu_knee_velocity_deg)

        self.pos_error = self.desired_trajectory_position_rad - self.imu_knee_angle_rad
        '''
        rad
        '''
        

        gain_p = self.passive_gains.proportional
        gain_i = self.passive_gains.integral
        gain_d = self.passive_gains.derivative

        # if self.pos_error < 0:
        #     if self.imu_knee_angle_deg > 90:
        #         gain_p = self.passive_gains.proportional /1.2
        #     else:
        #         pass
        # elif self.pos_error > 0:
        #     if self.imu_knee_angle_deg < 90:
        #         gain_p = self.passive_gains.proportional/1.2

        proportional = gain_p * self.pos_error
        self.passive_err_state.errorintegral += self.pos_error * self.dt
        integral = gain_i * self.passive_err_state.errorintegral
        # self.get_logger().info(f'ratio: {proportional- integral}')
        derivative = gain_d * (self.desired_trajectory_velocity_rad - self.imu_knee_velocity_rad)

        self.passive_err_state.errorprev = self.pos_error

        if self.control_active == 0:
            self.passive_err_state.errorintegral = 0.0
            self.passive_err_state.errorprev = 0.0
            self.passive_err_state.errorderivative = 0.0
            proportional = 0
            integral = 0
            derivative = 0

        output = proportional + integral + derivative
        return output

    def assistance_pid_control_fcn(self):
        self.dt = time.time() - self.past_time
        self.desired_trajectory_position_rad = np.deg2rad(self.desired_trajectory_position_deg)
        self.imu_knee_angle_rad = np.deg2rad(self.imu_knee_angle_deg)

        self.pos_error = self.desired_trajectory_position_rad - self.imu_knee_angle_rad
        '''
        rad
        '''

        gain_p = self.assistance_gains.proportional
        gain_i = self.assistance_gains.integral
        gain_d = self.assistance_gains.derivative

        proportional = gain_p * self.pos_error
        self.assistance_err_state.errorintegral += self.pos_error * self.dt
        integral = gain_i * self.assistance_err_state.errorintegral
        derivative = gain_d * (self.pos_error - self.assistance_err_state.errorprev) / self.dt

        self.assistance_err_state.errorprev = self.pos_error

        if self.control_active == 0:
            self.assistance_err_state.errorintegral = 0.0
            self.assistance_err_state.errorprev = 0.0
            self.assistance_err_state.errorderivative = 0.0
            proportional = 0
            integral = 0
            derivative = 0

        output = proportional + integral + derivative
        return output

    def resistance_moment_generator_fcn(self):
        self.dt = time.time() - self.past_time
        sign = np.sign(self.desired_trajectory_velocity_deg)
        '''
        +: flexion
        -: extension
        '''
        desired_resistance_moment = -(self.resistance_moment * sign)
        self.get_logger().info(f'desired resistance moment: {desired_resistance_moment}')
        resistance_moment_variation_tmp = (desired_resistance_moment - self.resistance_moment_memory) / self.dt
        if abs(resistance_moment_variation_tmp) > self.resistance_moment_variation:
            if resistance_moment_variation_tmp > 0:
                self.resistance_moment_memory += self.resistance_moment_variation * self.dt
            else:
                self.resistance_moment_memory -= self.resistance_moment_variation * self.dt
        else:
            self.resistance_moment_memory = desired_resistance_moment
        return self.resistance_moment_memory


    def exercise_desired_trajectory_position_callback(self, msg: Float64):

        self.desired_trajectory_position_deg = msg.data
        '''
        deg
        '''
    def exercise_desired_trajectory_velocity_callback(self, msg: Float64):
        self.desired_trajectory_velocity_deg = msg.data
        '''
        deg/s
        '''

    def assistance_velocity_min_callback(self, msg: Float64):
        self.assistance_velocity_min = msg.data
        '''
        deg/s
        '''


    def exercise_trajectory_state_callback(self, msg: Float64):
        self.desired_trajectory_state = msg.data
        '''
        0: stop
        1: start
        2: hold
        '''

    def exercise_info_callback(self, msg: Float64MultiArray):
        exercise_info = msg.data
        raw_power_enabled = int(exercise_info[0]) # 전원 상태 [0: off, 1: on]
        control_active = int(exercise_info[1]) # 제어 활성화 상태 [0: off, 1: on]
        control_mode = int(exercise_info[2]) # 제어 모드
        if raw_power_enabled == 1 and self.raw_power_enabled_prev == 0:
            if self.power_locked:
                self.get_logger().info("Safety lock released.")

            self.power_locked = False

        self.raw_power_enabled_prev = raw_power_enabled

        if not self.power_locked:
            self.power_enabled = raw_power_enabled
        else:
            self.get_logger().warn("Motor power is locked due to safety trigger.")

        if self.power_enabled == 1:
            self.control_active = control_active
            self.control_mode = control_mode
            if self.control_active == 1 and self.control_active_prev != self.control_active:
                self.control_time_stamp = time.time()
            self.control_active_prev = self.control_active
        else:
            self.control_mode = 0
            self.control_active = 0
            self.control_active_prev = 0

    def passive_parameter_callback(self, msg: Float64MultiArray):
        self.passive_gains.proportional = msg.data[0]
        self.passive_gains.integral = msg.data[1]
        self.passive_gains.derivative = msg.data[2]

    def assistance_parameter_callback(self, msg: Float64MultiArray):
        self.assistance_gain_k = msg.data[0]
        self.assistance_gains.proportional = msg.data[1]
        self.assistance_gains.integral = msg.data[2]
        self.assistance_gains.derivative = msg.data[3]

    def resistance_moement_callback(self, msg: Float64):
        self.resistance_moment = msg.data

    def imu_knee_angle_callback(self, msg: Vector3):
        self.imu_knee_angle_deg = msg.x
        '''
        deg
        '''

    def imu_velocity_callback(self, msg: Float64):
        self.imu_knee_velocity_deg = msg.data
        '''
        deg/s
        '''
    def imu_acceleration_callback(self, msg: Float64):
        self.imu_knee_acceleration_deg = msg.data
        '''
        deg/S^2
        '''
    def motor_setting_parameter_callback(self, msg: Float64MultiArray):
        self.rom_safe_upper = msg.data[0] # motor의 안전 upper ROM
        self.rom_safe_lower = msg.data[1] # motor의 안전 lower ROM
    
    def thruster_duty_ratio_input(self, input):
        # self.get_logger().info('Subscribed thruster: {0}'.format(msg.data))
        input_ = input
        if input < 10:
            input_ = 10
        elif input > 93:
            input_ = 93
        scaling_input = str(self.scaling_fcn(input_))
        if self.status:
            self.thruster_trans(scaling_input)
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
        # self.get_logger().info("serial read: {0}".format(EncodeData))
    

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