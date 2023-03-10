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

#from logdecorator import log_on_start , log_on_end , log_on_error
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


class Sensors:
    def __init__(self):
        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')
        self.trig = Pin('D8')
        self.echo = Pin('D9')

    def get_adc_value(self):
        adc_value_list = []
        adc_value_list.append(self.S0.read())
        adc_value_list.append(self.S1.read())
        adc_value_list.append(self.S2.read())
        logging.info("adcs: {0}".format(adc_value_list))
        return adc_value_list

    def get_distance(self):
        timeout = 0.05

        self.trig.low()
        time.sleep(0.01)
        self.trig.high()
        time.sleep(0.000015)
        self.trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        # logging.info("echo: {0}".format(self.echo.value()))

        while self.echo.value() == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1
        while self.echo.value() == 1:
            pulse_end = time.time()
            if pulse_end - timeout_start > timeout:
                # logging.info("pulse_end: {0} timeout_start: {1}".format(pulse_end,timeout_start))
                return -2
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        logging.info("distance(cm): {0}".format(cm))
        return cm


class Interpreters:
    def __init__(self):
        self.sensitivity = 200
        self.polarity = 1  # Means black line

    def get_grayscale_value(self, adcs):
        if abs(adcs[0] - adcs[2]) > self.sensitivity:
            if adcs[0] < adcs[2]:
                if adcs[0] + abs((adcs[2] - adcs[0]) / 4) > adcs[1]:
                    rob_pos = .5 * self.polarity
                else:
                    rob_pos = 1 * self.polarity
            else:
                if adcs[2] + abs((adcs[2] - adcs[0]) / 4) < adcs[1]:
                    rob_pos = -1 * self.polarity
                else:
                    rob_pos = -.5 * self.polarity
        else:
            rob_pos = 0

        logging.info("robot pos: {0}".format(rob_pos))
        return rob_pos


class Controllers:

    def __init__(self, m):
        self.line_steering = -30
        self.m = m

    def line_following(self, rob_pos, speed):
        logging.info("steering angle: {0}, speed: {1}".format(rob_pos * self.line_steering, speed))
        self.m.set_dir_servo_angle(rob_pos * self.line_steering)
        self.m.forward(speed)
        return rob_pos * self.line_steering

    def wall_checking(self, cm):
        if 0 < cm < 5:
            logging.info("About to hit an obstacle @ {0}".format(cm))
            self.m.forward(0)


class CVSteering:

    def __init__(self):
        pass

    def start_cv(self):
        camera = cv2.VideoCapture(-1)
        camera.set(3, 640)
        camera.set(4, 480)
        # while( camera.isOpened()):
        _, image = camera.read()
        return image
        # cv2.imshow('Original', image)

        # b_w_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('B/W', b_w_image)

        # if cv2.waitKey(1) & 0xFF == ord('q') :
        #    break

    cv2.destroyAllWindows()

    def look_for_color(self, frame):
        upper_black = np.array([40, 40, 40])
        lower_black = np.array([0, 0, 0])
        # frame = cv2.imread('/home/pi/DeepPiCar/driver/data/road1_240x320.png')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_black, upper_black)
        edges = cv2.Canny(mask, 200, 400)
        return edges

    def crop_video(self, edges):
        # removing top of image
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0, height * 1 / 2),
            (width, height * 1 / 2),
            (width, height),
            (0, height),
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        return cropped_edges

    def detect_line_segments(self, cropped_edges):

        # looking for line segments
        rho = 1  # distance precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
        min_threshold = 10  # minimal of votes
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold,
                                        np.array([]), minLineLength=8, maxLineGap=4)
        return line_segments

    def average_slope_intercept(self, cvs, frame, line_segments):
        middle_lines = []
        lane_lines = []
        if line_segments is None:
            # logging.info('No line_segment segments detected')
            return []

        for line_segment in line_segments:
            logging.info('line segment: {0}'.format(line_segment))
            for x1, y1, x2, y2 in line_segment:
                if x1 != x2:
                    fit = np.polyfit((x1, x2), (y1, y2), 1)
                    slope = fit[0]
                    intercept = fit[1]
                    middle_lines.append((slope, intercept))
                else:
                    logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)

        if middle_lines is None:
            logging.info('All lines were vertical')
            return []
        lane_lines.append(cvs.make_points(frame, np.average(middle_lines, axis=0)))
        return lane_lines

    def make_points(self, frame, line):
        if line == []:
            return []
        else:
            # logging.info("This is the line chosen by the car {0}".format(line))
            height, width, _ = frame.shape
            slope, intercept = line
            y1 = height  # bottom of the frame
            y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

            # bound the coordinates within the frame
            x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
            x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
            return [x1, y1, x2, y2]

    def steering_angle(self, path):
        # logging.info('This is the paths found by the camera: {0}'.format(path))
        if path != []:
            x1, y1, x2, y2 = path[0]
            x_offset = x2 - x1
            y_offset = y2 - y1
            drive_angle = math.atan(x_offset / y_offset * pi / 180)
        else:
            drive_angle = 0
        return drive_angle

    def steering_angle_adjustment(self, new_angle, turn_limit):
        angle_diff = new_angle - Motors.steering_dir_val
        if abs(angle_diff) > turn_limit:
            adjusted_angle = Motors.steering_dir_val + turn_limit * angle_diff / abs(angle_diff)
        else:
            adjusted_angle = new_angle
        logging.info('Angle from camera: {0}'.format(adjusted_angle))
        return adjusted_angle
if __name__ == "__main__":
    pass


    


    
