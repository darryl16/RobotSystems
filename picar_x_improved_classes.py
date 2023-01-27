#!/usr/bin/python3

import time
from math import tan, pi
import numpy as np
import cv2

try:
    from robot_hat import *
    from robot_hat import reset_mcu

    reset_mcu()
    time.sleep(0.01)
except ImportError:
    print(" This computer does not appear to be a PiCar -X system( robot_hat is not present ). Shadowing hardware calls with substitute functions ")
    from sim_robot_hat import *

import logging
logging_format = "%(asctime) s : %(message) s " 
logging.basicConfig(level = logging.INFO) #format = logging_format, level = logging.INFO, )#datefmt ="% H :% M :% S ")

# logging.getLogger().setLevel(logging.DEBUG) 
# comment out this line to remove debugging comments
# logging.debug (message) Use this line to print debugging info

from logdecorator import log_on_start , log_on_end , log_on_error 
# Add these lines to the start of functions
# @log_on_start ( logging . DEBUG , " Message when function starts ") 
# @log_on_error ( logging . DEBUG , " Message when function encounters an error before completing ") 
# @log_on_end ( logging . DEBUG , " Message when function ends successfully ")


class Motors:
    steering_dir_val = 0
    
    def __init__(self):
        self.PERIOD = 4095
        self.PRESCALER = 10
        self.TIMEOUT = 0.01
        
        self.dir_servo_pin = Servo(PWM('P2'))
        self.camera_servo_pin1 = Servo(PWM('P0'))
        self.camera_servo_pin2 = Servo(PWM('P1'))
        self.left_rear_pwm_pin = PWM("P13")
        self.right_rear_pwm_pin = PWM("P12")
        self.left_rear_dir_pin = Pin("D4")
        self.right_rear_dir_pin = Pin("D5")
        

        
        self.Servo_dir_flag = 1
        self.dir_cal_value = -18
        self.cam_cal_value_1 = 5
        self.cam_cal_value_2 = 10
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        self.cali_dir_value = [1, -1]
        self.cali_speed_value = [0, 0]
        
        

        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)
        
        import atexit
        atexit.register(self.cleanup)

    def set_motor_speed(self, motor, speed):
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        else:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        elif direction >= 0:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
            
            
    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value >= 0:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0
        else:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
    def motor_direction_calibration(self, motor, value):
        # 0: positive direction
        # 1:negative direction
        motor = motor- 1
        if value == 1:
            self.cali_dir_value[motor] = (-1)*self.cali_dir_value[motor]
    
    
    def dir_servo_angle_calibration(self, value):
        self.dir_cal_value = value
        self.set_dir_servo_angle(self.dir_cal_value)
        # dir_servo_pin.angle(dir_cal_value)
    
    def set_dir_servo_angle(self, value):
        
        Motors.steering_dir_val = value
        self.dir_servo_pin.angle(value+self.dir_cal_value)
    
    def camera_servo1_angle_calibration(self, value):
        # global cam_cal_value_1
        self.cam_cal_value_1 = value
        self.set_camera_servo1_angle(self.cam_cal_value_1)
        # camera_servo_pin1.angle(cam_cal_value)
    
    def camera_servo2_angle_calibration(self, value):
        self.cam_cal_value_2 = value
        self.set_camera_servo2_angle(self.cam_cal_value_2)
        # camera_servo_pin2.angle(cam_cal_value)
    
    def set_camera_servo1_angle(self, value):
        self.camera_servo_pin1.angle(-1 *(value+self.cam_cal_value_1))
    
    def set_camera_servo2_angle(self, value):
        self.camera_servo_pin2.angle(-1 * (value+self.cam_cal_value_2))
    

    
    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed) 
    
    def backward(self, speed):
        if Motors.steering_dir_val != 0:
            # print('turning angle:',theta)
            turn_radius = 9.5/tan((Motors.steering_dir_val* pi/ 180))
            # print('turn_radius: ',turn_radius)
            angle_vel = speed/turn_radius
            # print('angle_vel:',angle_vel)
            motor_speed = [angle_vel*(turn_radius-5.85), angle_vel*(turn_radius+5.85)]
            motor_speed = [motor_speed[0]/max(motor_speed)*speed, motor_speed[1]/max(motor_speed)*speed]
    
        else:
            motor_speed = [speed,speed]
        
        
        self.set_motor_speed(1, motor_speed[0])
        self.set_motor_speed(2, motor_speed[1])
        # print("left speed", motor_speed[0],"right speed", motor_speed[1],)
    
    def forward(self, speed):
        if Motors.steering_dir_val != 0 and speed != 0:
            
            turn_radius = 9.5/tan(Motors.steering_dir_val* pi/ 180)
            angle_vel = speed/turn_radius
            motor_speed = [angle_vel*(turn_radius+5.85), angle_vel*(turn_radius-5.85)]
            motor_speed = [motor_speed[0]/max(motor_speed)*speed, motor_speed[1]/max(motor_speed)*speed]
        else:
            motor_speed = [speed,speed]
        
        
        self.set_motor_speed(1, -1*motor_speed[0])
        self.set_motor_speed(2, -1*motor_speed[1])
        # print("left speed", (-1*motor_speed[0]),"right speed", (-1*motor_speed[1]),)
    
    def stop(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def cleanup(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def manual_motor_shutdown(self):
        self.stop()

        
if __name__ == "__main__":
    motors = Motors()


    


    
