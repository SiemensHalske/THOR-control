# ***** Imports *****

import time
import RPi.GPIO as GPIO
import smbus2

from threading import Thread
from time import sleep


# ***** Global variables *****

BUZZER = 36

ENABLE_ALARM1 = 38
ENABLE_ALARM2 = 40

LCD_PWM = 12
LCD_RST = 18

INT1 = 7

PWR_PUMP1 = 22
PWR_PUMP2 = 24
PWR_PUMP3 = 26
PWR_PUMP4 = 28
PUMP_LIST = [PWR_PUMP1, PWR_PUMP2, PWR_PUMP3, PWR_PUMP4]

RS1 = 11
RS2 = 13
RS3 = 15

ECHO_RESPONSE_0 = 35
TRIGGER_0 = 37

recent_water_level = [None, None]

# ***** Adresses *****

ADDR_PCF8574 = 0x20
ADDR_LCD = 0x78

# ****************************


class TankGuard:
    
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
            
            if self.upper_tank_limit - 0.1 <= water_level:
                pass
                # GPIO.output(PWR_PUMP1, GPIO.HIGH)
            else:
                pass
                # GPIO.output(PWR_PUMP1, GPIO.LOW)
            sleep(.25)
            
    def start(self):
        self.guard_running=True
        self.guard_thread.start()
        
    def stop(self):
        self.guard_running = False
        self.guard_thread.join()
        
        
class PumpControl:
    state_pump1 = False
    state_pump2 = False
    state_pump3 = False
    state_pump4 = False
    
    state_list = [state_pump1, state_pump2, state_pump3, state_pump4]
    
    def __init__(self) -> None:
        
        self.pump_control_running = False
        self.pump_controller = Thread(target=self.controller_thread, daemon=True)
        
    def controller_thread(self):
        while self.pump_control_running:
            for i, pump in enumerate(PUMP_LIST):
                GPIO.output(pump, GPIO.HIGH) if self.state_list[i] else GPIO.output(pump, GPIO.LOW)
            sleep(.25)
    
    def start(self):
        self.pump_control_running = True
        self.pump_controller.start()
        
    def stop(self):
        self.pump_control_running = False
        self.pump_controller.join()
        

class LCD_DRIVER:
    def __init__(self, bus_num, lcd_address):
        self.bus = smbus2.SMBus(bus_num)
        self.lcd_address = lcd_address
        self.initialize_lcd()
        
        self.driver_running = False
        self.driver_thread = Thread()    
    
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



class InterruptServiceRoutines:
    def ISR_INT1(channel):
        sleep(.01)
        
    def ISR_ECHO_RESPONSE(channel):
        sleep(.01)


def main():
    pass


def GPIO_setup():
    IN = GPIO.IN
    OUT = GPIO.OUT
    
    GPIO.setup(INT1, IN)
    GPIO.setup(RS1, IN)
    GPIO.setup(RS2, IN)
    GPIO.setup(RS3, IN)
    GPIO.setup(ECHO_RESPONSE_0, IN)
    
    GPIO.setup(PWR_PUMP1, OUT)
    GPIO.setup(PWR_PUMP2, OUT)
    GPIO.setup(PWR_PUMP3, OUT)
    GPIO.setup(PWR_PUMP4, OUT)
    GPIO.setup(TRIGGER_0, OUT)
    GPIO.setup(LCD_PWM, OUT)
    GPIO.setup(LCD_RST, OUT)
    
    GPIO.setup(BUZZER, OUT)
    GPIO.setup(ENABLE_ALARM1, OUT)
    GPIO.setup(ENABLE_ALARM2, OUT)

    GPIO.add_event_detect(ECHO_RESPONSE_0, GPIO.RISING, callback = InterruptServiceRoutines.ISR_ECHO_RESPONSE)
    GPIO.add_event_detect(INT1, GPIO.FALLING, callback = InterruptServiceRoutines.ISR_INT1)

def setup():
    GPIO_setup()
    sleep(.25)
    


if __name__ == '__main__':
    pass