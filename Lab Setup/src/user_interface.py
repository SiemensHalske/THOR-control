import setup_functions
import system_classes
import system_functions
import water_distribution_system

import time
import system_functions
from types import SimpleNamespace
import RPi.GPIO as GPIO
import smbus2

from setup_functions import GPIO_setup
from threading import Lock, Thread
from time import sleep

from system_classes import InterruptServiceRoutines, TankGuard, LCD_DRIVER
from water_distribution_system import BUS_IO, PWR_PUMP1

from water_distribution_system import ANALOG_SWITCH_1, ANALOG_SWITCH_2, ANALOG_SWITCH_3, ANALOG_SWITCH_4        


def main_menu(env: SimpleNamespace):
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("User pressed ")
        
    return 0

def ui_setup():
    env = SimpleNamespace()
    
    env.display_obj = LCD_DRIVER(BUS_IO)
    env.display_obj.start()
    
    env.pump_obj = system_classes.PumpControl()
    env.pump_obj.start()
    
    text_block = ["Booting up...       ", "Please wait!        "]
    env.display_obj.text_block = text_block


env = ui_setup()
ret = main_menu(env)
