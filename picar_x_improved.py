# from robot_hat import Servo, PWM, Pin, ADC
import time
from math import tan, pi

try:
    from robot_hat import *
    from robot_hat import reset_mcu

    reset_mcu()
    time.sleep(0.01)
except ImportError:
    print(
        " This computer does not appear to be a PiCar -X system( robot_hat is not present ). Shadowing hardware calls with substitute functions ")
    from sim_robot_hat import *
import numpy as np
from vilib import vilib
import logging

logging_format = "%(asctime) s : %(message) s "
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="% H :% M :% S ")

logging.getLogger().setLevel(logging.DEBUG)
# comment out this line to remove debugging comments
# logging . debug ( message ) Use this line to print debugging info

from logdecorator import log_on_start, log_on_end, log_on_error

# Add these lines to the start of functions


PERIOD = 4095
PRESCALER = 10
TIMEOUT = 0.02

dir_servo_pin = Servo(PWM('P2'))
camera_servo_pin1 = Servo(PWM('P0'))
camera_servo_pin2 = Servo(PWM('P1'))
left_rear_pwm_pin = PWM("P13")
right_rear_pwm_pin = PWM("P12")
left_rear_dir_pin = Pin("D4")
right_rear_dir_pin = Pin("D5")

S0 = ADC('A0')
S1 = ADC('A1')
S2 = ADC('A2')

Servo_dir_flag = 1
dir_cal_value = 0
cam_cal_value_1 = 0
cam_cal_value_2 = 0
motor_direction_pins = [left_rear_dir_pin, right_rear_dir_pin]
motor_speed_pins = [left_rear_pwm_pin, right_rear_pwm_pin]
cali_dir_value = [-1, 1]
cali_speed_value = [0, 0]

for pin in motor_speed_pins:
    pin.period(PERIOD)
    pin.prescaler(PRESCALER)

@log_on_start ( logging . DEBUG , " Set motor speed start ")
@log_on_error ( logging . DEBUG , " Set motor speed error ")
@log_on_end ( logging . DEBUG , " Set motor speed end ")
def set_motor_speed(motor, speed):
    global cali_speed_value, cali_dir_value
    motor -= 1
    if speed >= 0:
        direction = 1 * cali_dir_value[motor]
    elif speed < 0:
        direction = -1 * cali_dir_value[motor]
    speed = abs(speed)
    if speed != 0:
        speed = int(speed / 2) + 50
    speed = speed - cali_speed_value[motor]
    if direction < 0:
        motor_direction_pins[motor].high()
        motor_speed_pins[motor].pulse_width_percent(speed)
    else:
        motor_direction_pins[motor].low()
        motor_speed_pins[motor].pulse_width_percent(speed)

@log_on_start ( logging . DEBUG , " motor_speed_calibration start ")
@log_on_error ( logging . DEBUG , " motor_speed_calibration error ")
@log_on_end ( logging . DEBUG , " motor_speed_calibration end ")
def motor_speed_calibration(value):
    global cali_speed_value, cali_dir_value
    cali_speed_value = value
    if value < 0:
        cali_speed_value[0] = 0
        cali_speed_value[1] = abs(cali_speed_value)
    else:
        cali_speed_value[0] = abs(cali_speed_value)
        cali_speed_value[1] = 0

@log_on_start ( logging . DEBUG , " motor_direction_calibration start ")
@log_on_error ( logging . DEBUG , " motor_direction_calibration error ")
@log_on_end ( logging . DEBUG , " motor_direction_calibration end ")
def motor_direction_calibration(motor, value):
    # 0: positive direction
    # 1:negative direction
    global cali_dir_value
    motor -= 1
    if value == 1:
        cali_dir_value[motor] = -1 * cali_dir_value[motor]

@log_on_start ( logging . DEBUG , " dir_servo_angle_calibration start ")
@log_on_error ( logging . DEBUG , " dir_servo_angle_calibration error ")
@log_on_end ( logging . DEBUG , " dir_servo_angle_calibration end ")
def dir_servo_angle_calibration(value):
    global dir_cal_value
    dir_cal_value = value
    dir_servo_pin.angle(dir_cal_value)

@log_on_start ( logging . DEBUG , " set_dir_servo_angle start ")
@log_on_error ( logging . DEBUG , " set_dir_servo_angle error ")
@log_on_end ( logging . DEBUG , " set_dir_servo_angle end ")
def set_dir_servo_angle(value):
    global dir_cal_value
    value = round(value)
    # print("set_dir_servo_angle:%s"%value)
    dir_servo_pin.angle(value + dir_cal_value)

@log_on_start ( logging . DEBUG , " camera_servo1_angle_calibration start ")
@log_on_error ( logging . DEBUG , " camera_servo1_angle_calibration error ")
@log_on_end ( logging . DEBUG , " camera_servo1_angle_calibration end ")
def camera_servo1_angle_calibration(value):
    global cam_cal_value_1
    cam_cal_value_1 = value
    camera_servo_pin1.angle(cam_cal_value_1)

@log_on_start ( logging . DEBUG , " camera_servo2_angle_calibration start ")
@log_on_error ( logging . DEBUG , " camera_servo2_angle_calibration error ")
@log_on_end ( logging . DEBUG , " camera_servo2_angle_calibration end ")
def camera_servo2_angle_calibration(value):
    global cam_cal_value_2
    cam_cal_value_2 = value
    camera_servo_pin2.angle(cam_cal_value_2)

@log_on_start ( logging . DEBUG , " set_camera_servo1_angle start ")
@log_on_error ( logging . DEBUG , " set_camera_servo1_angle error ")
@log_on_end ( logging . DEBUG , " set_camera_servo1_angle end ")
def set_camera_servo1_angle(value):
    global cam_cal_value_1
    camera_servo_pin1.angle(value + cam_cal_value_1)

@log_on_start ( logging . DEBUG , " set_camera_servo2_angle start ")
@log_on_error ( logging . DEBUG , " set_camera_servo2_angle error ")
@log_on_end ( logging . DEBUG , " set_camera_servo2_angle end ")
def set_camera_servo2_angle(value):
    global cam_cal_value_2
    camera_servo_pin2.angle(value + cam_cal_value_2)

@log_on_start ( logging . DEBUG , " get_adc_value start ")
@log_on_error ( logging . DEBUG , " get_adc_value error ")
@log_on_end ( logging . DEBUG , " get_adc_value end ")
def get_adc_value():
    adc_value_list = []
    adc_value_list.append(S0.read())
    adc_value_list.append(S1.read())
    adc_value_list.append(S2.read())
    return adc_value_list

@log_on_start ( logging . DEBUG , " set_power start ")
@log_on_error ( logging . DEBUG , " set_power error ")
@log_on_end ( logging . DEBUG , " set_power end ")
def set_power(speed):
    set_motor_speed(1, speed)
    set_motor_speed(2, speed)

@log_on_start ( logging . DEBUG , " backward start ")
@log_on_error ( logging . DEBUG , " backward error ")
@log_on_end ( logging . DEBUG , " backward end ")
def backward(speed, theta):
    if theta != 0:
        # print('turning angle:',theta)
        turn_radius = 9.5 / tan((theta * pi / 180))
        # print('turn_radius: ',turn_radius)
        angle_vel = speed / turn_radius
        # print('angle_vel:',angle_vel)
        motor_speed = [angle_vel * (turn_radius - 5.85), angle_vel * (turn_radius + 5.85)]
        motor_speed = [motor_speed[0] / max(motor_speed) * speed, motor_speed[1] / max(motor_speed) * speed]

    else:
        motor_speed = [speed, speed]

    set_motor_speed(1, motor_speed[0])
    set_motor_speed(2, motor_speed[1])
    # print("left speed", motor_speed[0],"right speed", motor_speed[1],)

@log_on_start ( logging . DEBUG , " forward start ")
@log_on_error ( logging . DEBUG , " forward error ")
@log_on_end ( logging . DEBUG , " forward end ")
def forward(speed, theta):
    if theta == 0:
        motor_speed = [speed, speed]

    else:
        turn_radius = 9.5 / tan(theta * pi / 180)
        angle_vel = speed / turn_radius
        motor_speed = [angle_vel * (turn_radius + 5.83), angle_vel * (turn_radius - 5.83)]
        motor_speed = [motor_speed[0] / max(motor_speed) * speed, motor_speed[1] / max(motor_speed) * speed]

    set_motor_speed(1, -1 * motor_speed[0])
    set_motor_speed(2, -1 * motor_speed[1])

@log_on_start ( logging . DEBUG , " stop start ")
@log_on_error ( logging . DEBUG , " stop error ")
@log_on_end ( logging . DEBUG , " stop end ")
def stop():
    set_motor_speed(1, 0)
    set_motor_speed(2, 0)

@log_on_start ( logging . DEBUG , " Get_distance start ")
@log_on_error ( logging . DEBUG , " Get_distance error ")
@log_on_end ( logging . DEBUG , " Get_distance end ")
def Get_distance():
    timeout = 0.01
    trig = Pin('D8')
    echo = Pin('D9')

    trig.low()
    time.sleep(0.01)
    trig.high()
    time.sleep(0.000015)
    trig.low()
    pulse_end = 0
    pulse_start = 0
    timeout_start = time.time()
    while echo.value() == 0:
        pulse_start = time.time()
        if pulse_start - timeout_start > timeout:
            return -1
    while echo.value() == 1:
        pulse_end = time.time()
        if pulse_end - timeout_start > timeout:
            return -2
    during = pulse_end - pulse_start
    cm = round(during * 340 / 2 * 100, 2)
    # print(cm)
    return cm


ref = 950
DIR_LEFT = -1
DIR_FORWARD = 0
DIR_RIGHT = 1
DIR_OUT = 2

@log_on_start ( logging . DEBUG , " get_line_status start ")
@log_on_error ( logging . DEBUG , " get_line_status error ")
@log_on_end ( logging . DEBUG , " get_line_status end ")
def get_line_status():
    global ref
    fl_list = get_adc_value()
    return [1 if value < ref else 0 for value in fl_list]


# [0,0,0]: DIR_OUT
# [1,0,0]: DIR_LEFT
# [0,1,0]: DIR_FORWARD
# [0,0,1]: DIR_RIGHT
# [1,1,0]: DIR_LEFT
# [1,0,1]: DIR_FORWARD
# [0,1,1]: DIR_RIGHT
# [1,1,1]: DIR_FORWARD

@log_on_start ( logging . DEBUG , " get_direction start ")
@log_on_error ( logging . DEBUG , " get_direction error ")
@log_on_end ( logging . DEBUG , " get_direction end ")
def get_direction():  # 170<x<300
    fl_list = get_line_status()
    # if fl_list[1] <= ref:
    #     return 0
    # # if fl_list[0] > ref and fl_list[2] > ref:
    # #     return 0
    # elif fl_list[0] <= ref:
    #     return -1
    # elif fl_list[2] <= ref:
    #     return 1
    # else:
    #     return 2
    if fl_list in [[0, 1, 0], [1, 1, 1]]:  # , [1,1,1] , [1,0,1]
        return 0
    elif fl_list in [[1, 0, 0], [1, 1, 0]]:
        return -1
    elif fl_list in [[0, 0, 1], [0, 1, 1]]:
        return 1
    elif fl_list in [[0, 0, 0]]:
        return 2


tmp = 0

@log_on_start ( logging . DEBUG , " line_follow start ")
@log_on_error ( logging . DEBUG , " line_follow error ")
@log_on_end ( logging . DEBUG , " line_follow end ")
def line_follow():
    global tmp
    speed = 10
    status = get_direction()
    # print(status)
    if status != DIR_OUT:
        tmp = status
    if status == DIR_FORWARD:
        set_dir_servo_angle(0)
        # time.sleep(0.05)
        forward(speed)
    elif status == DIR_LEFT:
        set_dir_servo_angle(-20)
        # time.sleep(0.05)
        forward(speed)
        # set_dir_servo_angle(0)
        # time.sleep(0.05)
        # set_dir_servo_angle(0)
    elif status == DIR_RIGHT:
        set_dir_servo_angle(20)
        # time.sleep(0.05)
        forward(speed)
        # set_dir_servo_angle(0)
        # time.sleep(0.05)
        # set_dir_servo_angle(0)
    else:
        if tmp == DIR_FORWARD:
            # set_dir_servo_angle(10)
            # time.sleep(0.05)
            # backward(speed)
            # print(tmp)
            pass
        elif tmp == DIR_LEFT:
            set_dir_servo_angle(30)
            # time.sleep(0.1)
            backward(speed)
            # set_dir_servo_angle(0)
        elif tmp == DIR_RIGHT:
            set_dir_servo_angle(-30)
            # time.sleep(0.1)
            backward(speed)
            # set_dir_servo_angle(0)
            # while get_direction() != DIR_FORWARD:
        while True:
            temp = get_direction()
            if temp != tmp:
                break
            time.sleep(0.001)


@log_on_start ( logging . DEBUG , " test_line_detect start ")
@log_on_error ( logging . DEBUG , " test_line_detect error ")
@log_on_end ( logging . DEBUG , " test_line_detect end ")
def test_line_detect():
    print(get_line_status())
    print(get_adc_value())
    print("")
    time.sleep(0.5)


# Vilib.camera_start()
# Vilib.detect_color_name('blue')
pan_angle_color = 0
tilt_angle_color = 0


@log_on_start ( logging . DEBUG , " color_follow start ")
@log_on_error ( logging . DEBUG , " color_follow error ")
@log_on_end ( logging . DEBUG , " color_follow end ")
def color_follow():
    global pan_angle_color, tilt_angle_color
    vilib.color_detect_switch(True)
    status = [vilib.color_detect_object('x'), vilib.color_detect_object('y')]
    size = [vilib.color_detect_object('width'), vilib.color_detect_object('height')]
    time.sleep(0.005)
    # on left -1 on right 1
    if status[0] == -1:
        pan_angle_color = pan_angle_color + 1
        set_camera_servo1_angle(min(90, pan_angle_color))
        set_dir_servo_angle(max(-45, -pan_angle_color))
    elif status[0] == 1:
        pan_angle_color = pan_angle_color - 1
        set_camera_servo1_angle(max(-90, pan_angle_color))
        set_dir_servo_angle(min(45, -pan_angle_color))
    if status[1] == -1:
        tilt_angle_color = tilt_angle_color + 1
        set_camera_servo2_angle(min(45, tilt_angle_color))
    elif status[1] == 1:
        tilt_angle_color = tilt_angle_color - 1
        set_camera_servo2_angle(max(-45, tilt_angle_color))
    elif 0 < size[0] < 100 or 0 < size[1] < 100:
        forward(50)
    else:
        stop()
    # print(size)


pan_angle_human = 0
tilt_angle_human = 0


@log_on_start ( logging . DEBUG , " human_follow start ")
@log_on_error ( logging . DEBUG , " human_follow error ")
@log_on_end ( logging . DEBUG , " human_follow end ")
def human_follow():
    global pan_angle_human, tilt_angle_human
    vilib.human_detect_switch(True)
    status = [vilib.human_detect_object('x'), vilib.human_detect_object('y')]
    time.sleep(0.005)
    # on left -1 on right 1
    if status[0] == -1:
        pan_angle_human = pan_angle_human + 1
        set_camera_servo1_angle(min(90, pan_angle_human))
    elif status[0] == 1:
        pan_angle_human = pan_angle_human - 1
        set_camera_servo1_angle(max(-90, pan_angle_human))
    if status[1] == -1:
        tilt_angle_human = tilt_angle_human + 1
        set_camera_servo2_angle(min(45, tilt_angle_human))
    elif status[1] == 1:
        tilt_angle_human = tilt_angle_human - 1
        set_camera_servo2_angle(max(-45, tilt_angle_human))


@log_on_start ( logging . DEBUG , " Calibratesteering start ")
@log_on_error ( logging . DEBUG , " Calibratesteering error ")
@log_on_end ( logging . DEBUG , " Calibratesteering end ")
def Calibratesteering():
    # Testing different angles
    camera_servo1_angle_calibration(6)
    camera_servo2_angle_calibration(9)
    dir_servo_angle_calibration(-9)
    set_dir_servo_angle(-39)
    time.sleep(1)
    set_dir_servo_angle(0)
    time.sleep(1)
    set_motor_speed(1, 1)
    set_motor_speed(2, 1)
    camera_servo_pin1.angle(0)
    camera_servo_pin2.angle(0)
    time.sleep(1)
    camera_servo_pin1.angle(-39)
    camera_servo_pin2.angle(-39)


@log_on_start ( logging . DEBUG , " manual_motor_shutdown start ")
@log_on_error ( logging . DEBUG , " manual_motor_shutdown error ")
@log_on_end ( logging . DEBUG , " manual_motor_shutdown end ")
def manual_motor_shutdown():
    stop()


import atexit

atexit.register(stop)

if __name__ == "__main__":
    try:
        # dir_servo_angle_calibration(-15) 
        # camera_servo1_angle_calibration(0)
        # camera_servo2_angle_calibration(0)
        # forward(50)
        # time.sleep(1)
        # stop()

        Calibratesteering()
        #     # forward(50)
        #     # line_follow()
        #     # test_line_detect()
        # color_follow()

    finally:
        stop()
        # pass
