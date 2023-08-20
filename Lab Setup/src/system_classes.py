import water_distribution_system
import time
from types import SimpleNamespace
import RPi.GPIO as GPIO
import smbus2
import system_functions

from threading import Lock, Thread
from time import sleep

from water_distribution_system import ADDR_LCD, ANALOG_SWITCH_1, ANALOG_SWITCH_2, ANALOG_SWITCH_3, ANALOG_SWITCH_4, ECHO_RESPONSE_0, PWR_PUMP1, TRIGGER_0, read_pcf8574

LOW = GPIO.LOW
HIGH = GPIO.HIGH


class TankGuard:
    
    pump_override = False
    
    speed_of_sound = 343.2 
    tank_height = 100 
    
    recent_water_level = 0.0
    
    def __init__(self) -> None:
        
        self.guard_running = False
        self.guard_thread = Thread(target=self.guard_controller, daemon=True) 
    
    def send_trigger(self):
        GPIO.output(TRIGGER_0, GPIO.HIGH)
        sleep(0.00001)  # Send 10us pulse
        GPIO.output(TRIGGER_0, GPIO.LOW)
    
    def get_distance(self):
        self.send_trigger()
        
        GPIO.wait_for_edge(ECHO_RESPONSE_0, GPIO.RISING)
        start_time = time.time()
        GPIO.wait_for_edge(ECHO_RESPONSE_0, GPIO.FALLING)
        end_time = time.time()
        
        duration = end_time - start_time
        distance = (duration * 34320) / 2
        
        return distance

    def get_water_level(self):
        return self.tank_height - self.get_distance()
    
    def guard_controller(self):        
        while self.guard_running:
            water_level =  self.get_water_level()
            self.recent_water_level = [water_level, time.time()]
            
            if not self.pump_override or not system_functions.get_gpio_state(ANALOG_SWITCH_1):
                if self.upper_tank_limit - 0.1 <= water_level:
                    water_distribution_system.PUMP_STATES[0] = HIGH
                else:
                    water_distribution_system.PUMP_STATES[0] = LOW
                sleep(.25)
            
    def start(self):
        self.guard_running=True
        self.guard_thread.start()
        
    def stop(self):
        self.guard_running = False
        self.guard_thread.join()


class InterruptServiceRoutines:   
    INT_ENABLE = False
    most_recent_io_list = [None] * 8
         
    def ISR_IO(self, channel):
        self.most_recent_io_list = read_pcf8574() if self.INT_ENABLE else sleep(.002)


class LCD_Driver:
    def __init__(self, i2c_address=0x78, i2c_bus=1):
        self.address = i2c_address
        self.bus = smbus2.SMBus(i2c_bus)
        self.initialize_display()
        self.lines = ["", ""]

    def initialize_display(self):
        # Initialization sequence for the display
        # You might need to adjust this based on the specific datasheet or reference manual
        time.sleep(0.05)
        self.send_command(0x38)  # Function set
        self.send_command(0x39)  # Function set
        self.send_command(0x14)  # Internal OSC frequency
        self.send_command(0x79)  # Display control
        self.send_command(0x50)  # Contrast set
        self.send_command(0x6C)  # Follower control
        time.sleep(0.3)
        self.send_command(0x38)  # Function set
        self.send_command(0x0C)  # Display ON
        self.send_command(0x01)  # Clear display
        time.sleep(0.2)

    def send_command(self, cmd):
        # Send command to the display
        self.bus.write_byte_data(self.address, 0x00, cmd)

    def update_lines(self, lines):
        # Set the lines to be displayed
        self.lines = [line[:20] for line in lines]
        self.update_display()


    def update_display(self):
        # Update the display with the current lines
        self.send_command(0x01)  # Clear display
        time.sleep(0.05)
        self.send_command(0x80)  # Set DDRAM address to 0x00
        for char in self.lines[0]:
            self.bus.write_byte_data(self.address, 0x40, ord(char))
        self.send_command(0xC0)  # Move to the beginning of the second line
        for char in self.lines[1]:
            self.bus.write_byte_data(self.address, 0x40, ord(char))