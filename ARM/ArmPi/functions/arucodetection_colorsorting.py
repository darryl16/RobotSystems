#!/usr/bin/python3
# coding=utf8
import sys

sys.path.append('/home/pi/ArmPi/')
import cv2
import cv2.aruco as aruco
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_50)
ARUCO_PARAMS = aruco.DetectorParameters_create()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('red')

def getArucoPos(image, aruco_dict, params):
    # Detect aruco markers in the image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, parameters=params)

    # If any markers are detected, return their positions
    if ids is not None and len(ids) > 0:
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, CAMERA_MATRIX, DIST_COEFFS)
        aruco_pos = (tvec[0][0], tvec[0][1], tvec[0][2])
        return aruco_pos
    else:
        return None


# The Closing angle of the gripper
servo1 = 500


# initial position
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)


def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)


# set the rgb light color of the expansion board to make it consistent with the color to be tracked
def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()


count = 0
_stop = False
color_list = []
get_roi = False
__isRunning = False
detect_color = 'None'
start_pick_up = False
start_count_t1 = True


def reset():
    global _stop
    global count
    global get_roi
    global color_list
    global detect_color
    global start_pick_up
    global __target_color
    global start_count_t1

    count = 0
    _stop = False
    color_list = []
    get_roi = False
    __target_color = ()
    detect_color = 'None'
    start_pick_up = False
    start_count_t1 = True


def init():
    print("ColorSorting Init")
    initMove()


def start():
    global __isRunning
    reset()
    __isRunning = True
    print("Aruco Detection Start")


def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("Aruco Detection Stop")


def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("Aruco Detection Exit")


rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0


def move():
    global rect
    global _stop
    global get_roi
    global unreachable
    global __isRunning
    global detect_color 
    global start_pick_up
    global rotation_angle
    global world_X, world_Y

    # place coordinates
    coordinate = {
        'red': (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5, 1.5),
        'blue': (-15 + 0.5, 0 - 0.5, 1.5),
    }
    while True:
        detect_color = 'red'
        if __isRunning:
            if start_pick_up:  # if it is detected that the block has not moved for a period of time, start to pick up.
                # Move to the target position, height 6cm, judge whether the pos can be reached by the returned result
                # If the running time parameter is not given it will automatically be calculated and returned by the result.
                #set_rgb(detect_color)
                #setBuzzer(0.1)
                result = AK.setPitchRangeMoving((world_X, world_Y, 7), -90, -90, 0)
                print(world_X)
                print('now for Y')
                print(world_Y)
                if result == False:
                    unreachable = True
                    print('unreachableeeeee') #I am stuck here
                else:
                    print('is reachableeeeee')
                    unreachable = False
                    time.sleep(result[2] / 1000)  # If you can reach the specified location, get the running time

                    if not __isRunning:
                        continue
                    servo2_angle = getAngle(world_X, world_Y,
                                            rotation_angle)  # Calculate the angle the gripper needs to be rotated
                    Board.setBusServoPulse(1, servo1 - 280, 500)  # Paws Open
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)
                    print('checkpoint Alpha')

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((world_X, world_Y, 1.5), -90, -90, 0, 1000)
                    time.sleep(1.5)
                    print('checkpoint Beta')

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1, 500)  # 夹持器闭合 # holder closure
                    time.sleep(0.8)
                    print('checkpoint Charlie')

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  # Mechanical arm lift up
                    time.sleep(1)
                    print('checkpoint Delta')

                    if not __isRunning:
                        continue
                    result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, 90, 0)
                    time.sleep(result[2] / 1000)
                    print('checkpoint EEEEEEEE')

                    if not __isRunning:
                        continue
                    servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)
                    print('checkpoint F')
                    
                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving(
                        (coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3), -90, -90, 0, 500)
                    time.sleep(0.5)
                    print('checkpoint G')

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                    time.sleep(0.8)
                    print('checkpoint H')

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1 - 200, 500)  # Open the Claws, Put down the object
                    time.sleep(0.8)
                    print('checkpoint I')

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
                    time.sleep(0.8)
                    print('checkpoint J')

                    initMove()  # Return to the initial position
                    time.sleep(1.5)
                    print('checkpoint K')

                    #detect_color = 'None'
                    get_roi = False
                    start_pick_up = False
                    #set_rgb(detect_color)
        else:
            if _stop:
                _stop = False
                Board.setBusServoPulse(1, servo1 - 70, 300)
                time.sleep(0.5)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)
            time.sleep(0.01)


# Run Child Thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

t1 = 0
roi = ()
center_list = []
last_x, last_y = 0, 0
draw_color = range_rgb["black"]

def run(img):
    global _stop
    global count
    global get_roi
    global color_list
    global detect_color
    global start_pick_up
    global start_count_t1
    global __target_color

    # Get an image from the camera
    #image = Camera.captureStream()
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

    if not __isRunning:
        return img

    #image = img
    
    # Detect ArUco marker and get its position
    #aruco_pos = getArucoPos(image, ARUCO_DICT, ARUCO_PARAMS)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, ARUCO_DICT, parameters=ARUCO_PARAMS)
    
    # If ArUco marker is detected
    if ids is not None:
        aruco.drawDetectedMarkers(img, corners, ids)
        for i, ids in enumerate(ids):
            if i == 0:
                x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
                y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]
                
                x_centerPixel = x_sum*0.25
                y_centerPixel = y_sum*0.25
                
                center_cordinates = (int (x_centerPixel), int (y_centerPixel))
                aruco_pos = center_cordinates
                
                radius = 20
                color = (255,0,0)
                thickness = -1
                img = cv2.circle(img, center_cordinates, radius, color, thickness)                     
        # Move the arm to the ArUco marker position
        global world_X
        global world_Y
        world_X, world_Y = convertCoordinate(int (x_centerPixel), int (y_centerPixel), size)
        #AK.setPitchRangeMoving((0, 10, 10), aruco_pos[0]*100, aruco_pos[1]*100, 0, 1000)
        start_pick_up = True
        time.sleep(30)
        
        #if not ret:
         #   return False

        # Wait for some time to stabilize the arm
        time.sleep(0.5)

        # Wait for some time to stabilize the arm
        time.sleep(0.5)

        # Open the gripper
        #Board.setBusServoPulse(1, servo1 - 50, 300)

        # Wait for some time to open the gripper
        #time.sleep(0.5)

        # Move the arm up
        #AK.setPitchRangeMoving((0, 10, 10), 0, -20, -60, 1000)

        # Wait for some time to move the arm up
        #time.sleep(0.5)

        # Close the gripper
        #Board.setBusServoPulse(1, servo1 + 150, 500)

        # Wait for some time to close the gripper
        #time.sleep(0.5)

        # Move the arm up
        #AK.setPitchRangeMoving((0, 10, 10), 0, -20, -60, 1000)

        # Wait for some time to move the arm up
        #time.sleep(0.5)

        # Move the arm to the start position
        #initMove()

        # Wait for some time to move the arm to the start position
        #time.sleep(1)

    # If ArUco marker is not detected, return False
    #else:
       # return False

    return img

if __name__ == '__main__':
    init()
    start()
    __target_color = ('red', 'green', 'blue')
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()