# ***** Imports *****

import time
import system_functions
from types import SimpleNamespace
import RPi.GPIO as GPIO
import smbus2

from setup_functions import GPIO_setup
from threading import Lock, Thread
from time import sleep

from system_classes import InterruptServiceRoutines, TankGuard


# ***** Global variables *****

LOW = GPIO.LOW
HIGH = GPIO.HIGH

BUS_IO = 1
BUS_LCD = 3

BUZZER = 36

ENABLE_ALARM1 = 38
ENABLE_ALARM2 = 40

LCD_PWM = 12
LCD_RST = 18

INT1 = 7

PWR_PUMP1 = 22  # Pump for clearing the water tank completetly (if necessary)
PWR_PUMP2 = 24  # Circulation pump
PWR_PUMP3 = 26
PWR_PUMP4 = 28

ANALOG_SWITCH_1 = 19
ANALOG_SWITCH_2 = 21
ANALOG_SWITCH_3 = 23
ANALOG_SWITCH_4 = 27

PUMP_LIST = [PWR_PUMP1, PWR_PUMP2, PWR_PUMP3, PWR_PUMP4]
PUMP_STATES = [LOW]*4

RS1 = 11
RS2 = 13
RS3 = 15

ECHO_RESPONSE_0 = 35
TRIGGER_0 = 37

recent_water_level = [None, None]
io_lock = Lock()

# ***** Adresses *****

ADDR_PCF8574 = 0x20
ADDR_LCD = 0x78
ADDR_IO = 0x20

# ****************************


def main(env: SimpleNamespace):
    env.isr_obj.INT_ENABLE = True
    
    while system_functions.read_pcf8574(io_lock)[0]:
        pass    
    return True
    

def set_env_namespace():
    glob = SimpleNamespace()
    
    glob.isr_obj = InterruptServiceRoutines()
    
    return glob


def setup():
    GPIO_setup()
    glob = set_env_namespace()
    guard = TankGuard()
    guard.start()
        
    sleep(.25)
    
    return glob
    

if __name__ == '__main__':
    env = setup()
    
    exit( 0 if main(env) else 1 )