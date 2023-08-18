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
        global recent_water_level
        
        while self.guard_running:
            water_level =  self.get_water_level()
            recent_water_level = [water_level, time.time()]
            
            if not self.pump_override:
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
        

class LCD_DRIVER:
    text_block = ["--------------------","--------------------"]
    
    def __init__(self, bus_num, lcd_address):
        self.bus = smbus2.SMBus(bus_num)
        self.lcd_address = lcd_address
        self.initialize_lcd()
        
        self.driver_running = False
        self.driver_thread = Thread(target=self.lcd_controller, daemon=True)    
    
    def lcd_controller(self):
        while self.driver_running:
            self.display(self.text_block)
            sleep(.2)
    
    def initialize_lcd(self):
        pass

    def write_line(self, line_number, text):
        
        line_number -= 1
        
        if line_number not in [0, 1] or len(text) > 20:
            print("Invalid line number or text length")
            print("Limiting characters to 20.")
            
            text = text[:20]
        
        command = 0x80 if line_number == 0 else 0xC0  # command to move cursor
        self.bus.write_byte_data(ADDR_LCD, command, 0)

        for char in text:
            self.bus.write_byte_data(self.lcd_address, ord(char), 0)

    def display(self, lines):
        if len(lines) != 2:
            raise ValueError("Exactly two lines required")
        
        for i, line in enumerate(lines):
            self.write_line(i, line)
            
    def start(self):
        self.driver_running = True
        self.driver_thread.start()
        
    def stop(self):
        self.driver_running = False
        self.driver_thread.join()


class InterruptServiceRoutines:   
    INT_ENABLE = False
    most_recent_io_list = [None] * 8
         
    def ISR_IO(self, channel):
        self.most_recent_io_list = read_pcf8574() if self.INT_ENABLE else sleep(.002)
        

class PumpControl:
    condition = [
        system_functions.get_gpio_state(ANALOG_SWITCH_1),
        system_functions.get_gpio_state(ANALOG_SWITCH_2) or system_functions.get_gpio_state(ANALOG_SWITCH_3),
        system_functions.get_gpio_state(ANALOG_SWITCH_4)
    ]
    
    
    def __init__(self) -> None:
        
        self.pump_thread = Thread(target=self.pump_control)
        self.pump_control_running = False
    
    def pump_control(self):
        while self.pump_control_running:
            if self.condition[0]:
                GPIO.output(PWR_PUMP1, GPIO.HIGH)
        print("Pump Control stopped!")
        print("Please take caution when it's raining outside!"),
        
    def start(self):
        self.pump_control_running = True
        self.pump_thread.start()
        self.pump_thread.setDaemon(True)
        
    def stop(self):
        self.pump_control_running = False
        self.pump_thread.join()