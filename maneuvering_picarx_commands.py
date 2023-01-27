#!/usr/bin/python3

import picar_x_improved
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



def move_forward(speed,length,angle):
    picar_x_improved.set_dir_servo_angle(angle)
    picar_x_improved.forward(speed,angle)
    time.sleep(length)
    picar_x_improved.stop()
    print('finished moving forward')
    
def parellel_parking(speed,length, direction=-1):
    # picarx_improved.set_dir_servo_angle(0)
    # time.sleep(.1)
    picar_x_improved.set_dir_servo_angle(direction*40)
    picar_x_improved.backward(speed,direction*40)
    time.sleep(length*.50)
    picar_x_improved.set_dir_servo_angle(-direction*40)
    picar_x_improved.backward(speed,-direction*40)
    time.sleep(length*.50)
    
    
def k_turn(speed,length, direction=-1):
    # picarx_improved.set_dir_servo_angle(0)
    # time.sleep(.1)
    picar_x_improved.set_dir_servo_angle(direction*40)
    picar_x_improved.forward(speed,-direction*40)
    time.sleep(length*.50)
    picar_x_improved.set_dir_servo_angle(-direction*40)
    picar_x_improved.backward(speed,direction*40)
    time.sleep(length*.50)
    picar_x_improved.set_dir_servo_angle(direction*40)
    picar_x_improved.forward(speed,-direction*40)
    time.sleep(length*.50)   
        
    
def test():
    picar_x_improved.set_dir_servo_angle(-40)
    picar_x_improved.camera_servo_pin1.angle(-40)
    picar_x_improved.camera_servo_pin2.angle(-40)
    time.sleep(1)
    picar_x_improved.set_motor_speed(1, 1)
    picar_x_improved.set_motor_speed(2, 1)
    picar_x_improved.set_dir_servo_angle(40)
    picar_x_improved.camera_servo_pin1.angle(40)
    picar_x_improved.camera_servo_pin2.angle(40)
    time.sleep(1)



# import atexit
# atexit.register(picarx_improved.stop)

if __name__ == "__main__":
    # while True:
    # test()
    choice = input('Choose an action to take: (park, forward, kturn)')
    if choice == 'forward':
        print('moving forward...')
        move_forward(48,2,0)
    elif choice == 'park':
        print('parellel_parking...')
        parellel_parking(65, 1.75,-1)
    elif choice == 'kturn':
        print('turning around...')
        k_turn(65,2.25,-1)
    else:
        print('did nothing...')
        pass