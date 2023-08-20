import time
from types import SimpleNamespace
import RPi.GPIO as GPIO
import smbus2

from threading import Lock, Thread
from time import sleep


from water_distribution_system import INT1, InterruptServiceRoutines
from water_distribution_system import RS1
from water_distribution_system import RS2
from water_distribution_system import RS3
from water_distribution_system import ECHO_RESPONSE_0
from water_distribution_system import PWR_PUMP1
from water_distribution_system import PWR_PUMP2
from water_distribution_system import PWR_PUMP3
from water_distribution_system import PWR_PUMP4
from water_distribution_system import TRIGGER_0
from water_distribution_system import LCD_PWM
from water_distribution_system import LCD_RST
from water_distribution_system import BUZZER
from water_distribution_system import ENABLE_ALARM1
from water_distribution_system import ENABLE_ALARM2


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

    GPIO.add_event_detect(INT1, GPIO.FALLING, callback = InterruptServiceRoutines.ISR_IO)
    
    