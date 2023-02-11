#!/usr/bin/python3

import time
import concurrent.futures
from threading import Lock
from week3_picar_x_commands_classes import Motors, Sensors, Interpreters, Controllers

try:
    from robot_hat import *
    from robot_hat import reset_mcu

    reset_mcu()
    time.sleep(0.01)
except ImportError:
    print("This computer does not have the required hardware components (robot_hat). Falling back to simulation mode.")
    from sim_robot_hat import *

import logging

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(level=logging.INFO, format=logging_format)

from logdecorator import log_on_start, log_on_end, log_on_error


class DataBus:
    def __init__(self):
        self.message = 0

    def read(self):
        return self.message

    def write(self, msg):
        self.message = msg

    def test(self):
        print(self.read())
        self.write('Testing read/write')
        print(self.read())


def sensor_producer(s, in_bus, delay):
    lock = Lock()
    while True:
        with lock:
            adcs = s.get_adc_value()
        in_bus.write(adcs)
        time.sleep(delay)


def interpreter_cp(i, in_bus, out_bus, delay):
    while True:
        if in_bus.read() != None:
            position = i.get_grayscale_value(in_bus.read())
            out_bus.write(position)
            time.sleep(delay)
        else:
            time.sleep(delay)


def controller_consumer(c, out_bus, delay, speed):
    while True:
        if out_bus.read() != None:
            c.line_following(out_bus.read(), speed)
            time.sleep(delay)
        else:
            time.sleep(delay)


def simultaneity(m, s, i, c, speed):
    sensor_delay = 0.2
    interpreter_delay = 0.2
    controller_delay = 0.2
    in_bus = DataBus()
    out_bus = DataBus()

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        bus_sensor = executor.submit(sensor_producer, s, in_bus, sensor_delay)
        bus_interpreter = executor.submit(interpreter_cp, i, in_bus, out_bus, interpreter_delay)
        bus_controller = executor.submit(controller_consumer, c, out_bus, controller_delay, speed)

    logging.info("Program execution reached the end.")
    logging.info(bus_controller)
    bus_sensor.result()
    bus_interpreter.result()
    bus_controller.result()



        
if __name__ == "__main__":
    b = DataBus()
    b.test()
