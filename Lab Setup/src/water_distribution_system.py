# ***** Imports *****

import RPi.GPIO as GPIO
import smbus2
import system_functions
import time
from types import SimpleNamespace

from i2c_module_1 import i2c_module_1 as i2c_module_1, i2c_temperature_module
from i2c_module_2 import i2c_module_2 as i2c_module_2
from i2c_module_3 import i2c_module_3 as i2c_module_3
from setup_functions import GPIO_setup
from system_classes import InterruptServiceRoutines, LCD_Driver, TankGuard
from system_functions import get_gpio_state as ggpio
from threading import Lock, Thread
from time import sleep


# ***** Constants *****
LOW = GPIO.LOW
HIGH = GPIO.HIGH
BUS_IO, BUS_LCD, BUS_MODULES = 1, 3, 5
BUZZER = 36
ENABLE_ALARM1, ENABLE_ALARM2 = 38, 40
LCD_PWM, LCD_RST = 12, 18
INT1 = 7
PWR_PUMPS = [22, 24, 26, 28]  # Pump control pins
ANALOG_SWITCHES = [19, 21, 23, 27]
RS = [11, 13, 15]
ECHO_RESPONSE_0, TRIGGER_0 = 35, 37
ADDR_PCF8574, ADDR_LCD, ADDR_IO, ADDR_BME680 = 0x20, 0x78, 0x20, 0x76
ADDR_MLX90393, ADDR_TLV493D, ADDR_INA3221 = 0x0C, 0x1E, 0x40

# ***** Global variables *****
bus_lock = Lock()
io_states = [0] * 8

# ****** Functions ******


def read_input_thread():
    io_lock = Lock()
    global io_states
    
    while True:
        io_states = system_functions.read_pcf8574_bus_var(BUS_IO, ADDR_PCF8574, io_lock)
        sleep(.5)
        

def control_pump(pump_num: int, state: int):
    pump_pin = PWR_PUMPS[pump_num - 1]
    if GPIO.input(pump_pin) != state:
        GPIO.output(pump_pin, state)


def display_env_measurements(glob: SimpleNamespace, env_measurements: dict[str]):
    i = 0
    
    for datapoint in env_measurements.values():
        glob.display_driver.update_lines([f"Temp: {datapoint[i]}°C", f"Tl: {env.guard.recent_water_level}m"])
        i += 1


def main(env: SimpleNamespace):
    global io_states
    
    io_thread = Thread(target=read_input_thread, daemon=True)
    io_thread.start()
    
    GPIO.add_event_detect(INT1, GPIO.FALLING, callback = InterruptServiceRoutines.ISR_IO) if io_states[7] else None
    env.isr_obj.INT_ENABLE = io_states[7]
    
    while io_states[0]:
        
        data = handle_i2c_modules(io_states)
        handle_pumps(env)
        
        if data != -1:
            display_env_measurements(env, data[0]) if io_states[1] else None

def handle_i2c_modules(io_states):
    i2c_modules_enable = [io_states[1], io_states[2], io_states[3]]

    data_1 = i2c_module_1(module_bus_lock=bus_lock) if i2c_modules_enable[0] else None  # Temperature, humidity, pressure, gas resistance
    data_2 = i2c_module_2(module_bus_lock=bus_lock) if i2c_modules_enable[1] else None  # Magnetic field
    data_3 = i2c_module_3(module_bus_lock=bus_lock) if i2c_modules_enable[2] else None  # Current, voltage
        
    return [data_1, data_2, data_3] if None or -1 not in [data_1, data_2, data_3] else -1


def handle_pumps(env):
    master_switch = ggpio(ANALOG_SWITCHES[0])
    display_lines = []

    for i, state in enumerate([ggpio(switch) and master_switch for switch in ANALOG_SWITCHES[1:]], 1):
        display_lines.append(f"Pumpe {i} {'i' if state else 'o'}")
        control_pump(i, HIGH if state else LOW)

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