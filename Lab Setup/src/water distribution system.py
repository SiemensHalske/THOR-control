# ***** Imports *****

from threading import Thread
from time import sleep
import time
import RPi.GPIO as GPIO

# ***** Global variables *****

INT1 = 7
RS1 = 11
RS2 = 13
RS3 = 15
ECHO_RESPONSE_0 = 35
TRIGGER_0 = 37
LCD_PWM = 12
LCD_RST = 18
PWR_PUMP1 = 22
PWR_PUMP2 = 24
PWR_PUMP3 = 26
PWR_PUMP4 = 28
BUZZER = 36
ENABLE_ALARM1 = 38
ENABLE_ALARM2 = 40


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
        while self.guard_running:
            water_level =  self.get_water_level()
            
            if self.upper_tank_limit - 0.1 <= water_level:
                GPIO.output(PWR_PUMP1, GPIO.HIGH)
            else:
                GPIO.output(PWR_PUMP1, GPIO.LOW)
            sleep(.25)
            
    def start(self):
        self.guard_running=True
        self.guard_thread.start()
        
    def stop(self):
        self.guard_running = False
        self.guard_thread.join()
        

class LCD_DRIVER:
    
    text_lines = [
        "------- GRID -------", 
        "--------------------"
    ]
    
    def __init__(self) -> None:
        
        self.driver_running = False
        self.driver_thread = Thread(target=self.lcd_controller, daemon=True)    
    
    def lcd_controller(self):
        while self.driver_running:
            pass
        
    def start(self):
        self.driver_running = True
        self.driver_thread.start()
    
    def stop(self):
        self.driver_running = False
        self.driver_thread.join()


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
    pass


if __name__ == '__main__':
    pass