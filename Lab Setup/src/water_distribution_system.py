# ***** Imports *****

import RPi.GPIO as GPIO
import smbus2
import system_functions
import time
from types import SimpleNamespace

from i2c_module_1 import i2c_module_1 as i2c_module_1
from i2c_module_2 import i2c_module_2 as i2c_module_2
from i2c_module_3 import i2c_module_3 as i2c_module_3
from setup_functions import GPIO_setup
from system_classes import InterruptServiceRoutines, LCD_Driver, TankGuard
from system_functions import get_gpio_state as ggpio
from threading import Lock, Thread
from time import sleep


# ***** Global variables *****

LOW = GPIO.LOW
HIGH = GPIO.HIGH

BUS_IO = 1
BUS_LCD = 3
BUS_MODULES = 5

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

RS1 = 11
RS2 = 13
RS3 = 15

ECHO_RESPONSE_0 = 35
TRIGGER_0 = 37

bus_lock = Lock()
io_states = [0]*8

# ***** Adresses *****

ADDR_PCF8574 = 0x20
ADDR_LCD = 0x78
ADDR_IO = 0x20
ADDR_BME680 = 0x76


# ****************************


def read_input_thread():
    io_lock = Lock()
    global io_states
    
    while True:
        io_states = system_functions.read_pcf8574(io_lock)
        sleep(.5)
        

def turn_off_pump(pump_num: int):
    if pump_num == 1:
        GPIO.output(PWR_PUMP1, LOW)
    elif pump_num == 2:
        GPIO.output(PWR_PUMP2, LOW)
    elif pump_num == 3:
        GPIO.output(PWR_PUMP3, LOW)
    elif pump_num == 4:
        GPIO.output(PWR_PUMP4, LOW)
        
def turn_on_pump(pump_num: int):
    if pump_num == 1:
        GPIO.output(PWR_PUMP1, HIGH)
    elif pump_num == 2:
        GPIO.output(PWR_PUMP2, HIGH)
    elif pump_num == 3:
        GPIO.output(PWR_PUMP3, HIGH)
    elif pump_num == 4:
        GPIO.output(PWR_PUMP4, HIGH)


def display_env_measurements(glob: SimpleNamespace, env_measurements: dict[str]):
    pass


def main(env: SimpleNamespace):
    global io_states
    
    io_thread = Thread(target=read_input_thread, daemon=True)
    io_thread.start()
    
    while io_states[0]:
        env.isr_obj.INT_ENABLE = io_states[7]
        
        data = handle_i2c_modules(io_states)
        handle_pumps(env)
        
        if io_states[1]:
            display_env_measurements(env, data[0])
        elif io_states[2]:
            pass
        elif io_states[3]:
            pass


def handle_i2c_modules(io_states):
    i2c_modules_enable = [io_states[1], io_states[2], io_states[3]]

    if i2c_modules_enable[0]:
        data_1 = i2c_module_1(module_bus_lock=bus_lock)
    elif i2c_modules_enable[1]:
        data_2 = i2c_module_2()
    elif i2c_modules_enable[2]:
        data_3 = i2c_module_3()
        
    return [data_1, data_2, data_3]


def handle_pumps(env):
    master_switch = ggpio(ANALOG_SWITCH_1)
    pump_states = [
        ggpio(ANALOG_SWITCH_2),
        ggpio(ANALOG_SWITCH_3) and master_switch,
        ggpio(ANALOG_SWITCH_4) and master_switch
    ]

    display_lines = []
    for i, state in enumerate(pump_states, 1):
        if state:
            display_lines.append(f"Pumpe {i} i")
            turn_on_pump(i)
        else:
            display_lines.append(f"Pumpe {i} o")
            turn_off_pump(i)

    if len(display_lines) == 1:
        env.display_driver.update_lines([display_lines[0], f"Tl: {env.guard.recent_water_level}m"])
    else:
        env.display_driver.update_lines([", ".join(display_lines), f"Tl: {env.guard.recent_water_level}m"])
    

def set_env_namespace():
    glob = SimpleNamespace()
    
    glob.isr_obj = InterruptServiceRoutines()
    glob.display_driver = LCD_Driver()
    glob.guard = TankGuard()
    
    glob.guard.start()
    glob.display_driver.update_display([f"Pump Control",f"Tl:{glob.guard.recent_water_level}m"])
    
    return glob


def setup():
    GPIO_setup()
    glob = set_env_namespace()
        
    sleep(.25)
    
    return glob
    

if __name__ == '__main__':
    env = setup()
    
    exit( 0 if main(env) else 1 )