'''

'''


import threading
import RPi.GPIO as GPIO

import os
import smbus2
import sys
import time

from threading import Thread
from time import sleep


# ****    DEFINES / Variables   ****

BUS_PWR = 1
BUS_IO = 3
BUS_MATRIX = 5

R_SHUNT = 0.1
R_RASPI_SHUNT = 0.05

GPIO_04 = 7
GPIO_17 = 11
GPIO_10 = 19
GPIO_09 = 21
GPIO_11 = 23
GPIO_00 = 27
GPIO_13 = 33
GPIO_14 = 8
GPIO_15 = 10
GPIO_18 = 12
GPIO_23 = 16
GPIO_24 = 18
GPIO_25 = 22
GPIO_08 = 24
GPIO_07 = 26
GPIO_01 = 28
GPIO_12 = 32
GPIO_16 = 36
GPIO_20 = 38
GPIO_21 = 40


STARTUP_DELAY = 1.5
DEFAULT_VOLTAGE = 5.0
eFUSE_ALERT = False

# I2C addresses 
BUTTON_CURSOR_ADDR = 0x21
BUTTON_MATRIX_ADDR = 0x20
FB_ADJ_LOWER_ADDR = 0x2F
FB_ADJ_UPPER_ADDR = 0x2C
LC_DISPLAY_ADDR = 0x78
PWR_MGMNT_ADDR = 0x40
USB_V_CONTROL_ADDR = 0x48

# INA3221 shunt and bus voltage register addresses for channels 1, 2, and 3
SHUNT_VOLTAGE_REGISTERS = [0x01, 0x03, 0x05]
BUS_VOLTAGE_REGISTERS = [0x02, 0x04, 0x06]
# INA3221 registers
INA3221_REG_MASK_ENABLE = 0x06  # Mask/Enable register
# bits to clear alerts
DISABLE_ALERTS = 0x0000

# ***** ***** *****

MCP23017_IODIRA = 0x00  # Bank A Direction Register
MCP23017_IODIRB = 0x01  # Bank B Direction Register
MCP23017_GPIOA = 0x12   # Bank A Data Register
MCP23017_GPIOB = 0x13   # Bank B Data Register

MCP23017_BANKA = [0, 0, 0, 0, 0, 0, 0, 0]
MCP23017_BANKB = [0, 0, 0, 0, 0, 0, 0, 0]

BUTTON_MATRIX_STATE = [0] * 9
BUTTON_CURSOR_STATE = [0] * 8

ADDR_LIST = [
    BUTTON_CURSOR_ADDR, 
    BUTTON_MATRIX_ADDR, 
    FB_ADJ_LOWER_ADDR, 
    FB_ADJ_UPPER_ADDR, 
    LC_DISPLAY_ADDR, 
    PWR_MGMNT_ADDR, 
    USB_V_CONTROL_ADDR ]

lock = threading.Lock()
io_lock = threading.Lock()
THREADS = []
CONTROLLER = []
# ***********************

GPIO.setWarnings(False)
GPIO.setmode(GPIO.BCM)

class InterruptServiceRoutines:
    def pwr_alert_isr(channel):
        sleep(.01)
        
    def int_cursor_isr(channel):
        sleep(.01)
        
class CoreFunctions:
    def read_ADS1115_differential_voltage(self, address, positive_channel, negative_channel, differential_pair=None):
        voltage_range = 6.144  # voltage range in Volt
        
        with lock:
            config_register = 0b1100000110000011  # Binary: 0xC383

            if (positive_channel == 0 and negative_channel == 1) or differential_pair in [(0, 1)]:
                bus = smbus2.SMBus(1)
                bus.write_i2c_block_data(
                    address, 1, [config_register >> 8, config_register & 0xFF])
                time.sleep(0.01)

                data = bus.read_i2c_block_data(address, 0, 2)
                raw_value = (data[0] << 8) | data[1]
                voltage = (raw_value / 32767.0) * voltage_range

                bus.close()
                return voltage
            else:
                print("Invalid channel pairing. Valid pair: AIN0 & AIN1.")
                return -1


    def read_ads1115_voltage(self, bus_num, address, channel):
        with lock:
            if channel not in range(4):
                print("Invalid channel. Valid channel numbers: 0, 1, 2, 3.")
                return None

            config_register = 0b1100000110000011  # Binary: 0xC383
            config_register |= (channel << 12)

            bus = smbus2.SMBus(bus_num)
            bus.write_i2c_block_data(
                address, 1, [config_register >> 8, config_register & 0xFF])
            time.sleep(0.008)

            data = bus.read_i2c_block_data(address, 0, 2)
            bus.close()

            raw_value = (data[0] << 8) | data[1]
            voltage_range = 6.144  # Spannungsrange in Volt
            voltage = (raw_value / 32767.0) * voltage_range  # Umrechnung in Float-Wert

            return voltage


    def read_INA3221_shunt(self, channel):
        # Check if channel is valid
        with lock:
            if channel not in [1, 2, 3]:
                raise ValueError("Channel must be 1, 2, or 3")

            # Create an SMBus instance
            bus = smbus2.SMBus(BUS_PWR)

            # Read the shunt voltage register
            raw = bus.read_word_data(PWR_MGMNT_ADDR, SHUNT_VOLTAGE_REGISTERS[channel - 1])
            
            bus.close()

            # Convert raw value to voltage (in mV)
            voltage = raw * 0.005

            return voltage


    def read_INA3221_bus(self, channel):
        with lock:
            # Check if channel is valid
            if channel not in [1, 2, 3]:
                raise ValueError("Channel must be 1, 2, or 3")

            # Create an SMBus instance
            bus = smbus2.SMBus(1)

            # Read the bus voltage register
            raw = bus.read_word_data(PWR_MGMNT_ADDR, BUS_VOLTAGE_REGISTERS[channel - 1])

            # Convert raw value to voltage (in V)
            voltage = raw * 0.00125

            return voltage


    # Write a byte to a bank
    def write_mcp23017_bank(self, bank, data):
        with io_lock:
            bus = smbus2.SMBus(BUS_IO)
            
            if bank == 'A':
                bus.write_byte_data(BUTTON_CURSOR_ADDR, MCP23017_GPIOA, data)
            elif bank == 'B':
                bus.write_byte_data(BUTTON_CURSOR_ADDR, MCP23017_GPIOB, data)
            else:
                raise ValueError("Invalid bank. Choose either 'A' or 'B'")
            
            bus.close()

    # Read a byte from a bank
    def read_mcp23017_bank(self, bank):
        with io_lock:
            bus = smbus2.SMBus(BUS_IO)
            
            if bank == 'A':
                return bus.read_byte_data(BUTTON_CURSOR_ADDR, MCP23017_GPIOA)
            elif bank == 'B':
                return bus.read_byte_data(BUTTON_CURSOR_ADDR, MCP23017_GPIOB)
            else:
                raise ValueError("Invalid bank. Choose either 'A' or 'B'")
            
            bus.close()


    def setResistance_ADS5272(self, bus_num, device_address, resistance_int: int):
        with lock:
            bus = smbus2.SMBus(bus_num)
            try:

                # Ensure the resistance_int is within the valid range (0 to 1023)
                resistance_int = max(0, min(1023, resistance_int))

                # Calculate the command byte and data bytes based on the desired resistance_int
                command_byte = 0x00  # Command byte for write to RDAC register
                data_byte1 = (resistance_int >> 8) & 0xFF  # Most significant byte
                data_byte2 = resistance_int & 0xFF         # Least significant byte

                # Send the data to the device
                bus.write_i2c_block_data(device_address, command_byte, [
                                        data_byte1, data_byte2])

                bus.close()
            except IOError:
                print(f"Somethin went wrong while writing to address {device_address}")
                bus.close()
                return -1

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    # Paste cleanup code here when needed in the future.
    # Currently only a message is being displayed.
    sys.exit(0)


def main():
    '''
        Explanation:

        Params:

        Returns:
            None
    '''


def GPIO_setup():
    IN = GPIO.IN
    OUT = GPIO.OUT
    def setup_output():
        GPIO.setup(GPIO_24, OUT)
        GPIO.setup(GPIO_15, OUT)
        GPIO.setup(GPIO_16, OUT)
        GPIO.setup(GPIO_09, OUT)
        GPIO.setup(GPIO_11, OUT)
    
    def setup_input():
        GPIO.setup(GPIO_23, IN)
        GPIO.setup(GPIO_21, IN)
        GPIO.setup(GPIO_20, IN)
        GPIO.setup(GPIO_25, IN)
        GPIO.setup(GPIO_08, IN)
        GPIO.setup(GPIO_07, IN)
        GPIO.setup(GPIO_01, IN)
        GPIO.setup(GPIO_01, IN)
        GPIO.setup(GPIO_25, IN)
        
    setup_output()
    setup_input()
    
    GPIO.add_event_detect(GPIO_23, GPIO.FALLING, callback=InterruptServiceRoutines.pwr_alert_isr)
    GPIO.add_event_detect(GPIO_16, GPIO.FALLING, callback=InterruptServiceRoutines.int_cursor_isr)
    #GPIO.add_event_detect(GPIO_20, GPIO.FALLING, callback=InterruptServiceRoutines.int_matrix_isr)
    #GPIO.add_event_detect(PWR_CRITICAL, GPIO.FALLING, callback = pwr_critical_isr)
    #GPIO.add_event_detect(PWR_PV, GPIO.FALLING, callback = pwr_pv_isr)
    #GPIO.add_event_detect(PWR_WARNING, GPIO.FALLING, callback = pwr_warning_isr)
    #GPIO.add_event_detect(PWR_TC, GPIO.FALLING, callback = pwr_tc_isr)
    
    
def setup():
    '''
        Explanation:
        
        Params:

        Returns:
            None
    '''
    
    GPIO_setup()

    for addr in ADDR_LIST:
        if addr == None:
            sys.exit(0)
    
    GPIO.output(GPIO_04, GPIO.HIGH)
    #setOutput_Voltage_Resistors(DEFAULT_VOLTAGE)  # r_upper = 1024, r_lower = 296

    sleep(.2)
    
    with io_lock:
        bus = smbus2.SMBus(BUS_IO)
        
        # Configuring the MCP23017
        # Configure Bank A: 0b00111000 -> Pins 5, 6 and 7 as output, others as input
        # Configure Bank B: 0b10100000 -> Pins 6 and 7 as output, Pins 0 to 4 as input, others as input
        bus.write_byte_data(BUTTON_CURSOR_ADDR, MCP23017_IODIRA, 0b00111000)
        bus.write_byte_data(BUTTON_CURSOR_ADDR, MCP23017_IODIRB, 0b10100000)
         
        bus.close();  
        del bus
    
    sleep(1)


if __name__ == '__main__':
    sleep(STARTUP_DELAY)
    setup()
    main()

