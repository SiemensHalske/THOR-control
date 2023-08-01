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


# ****    DEFINES    ****

BUS_PWR = 1
BUS_IO = 3
BUS_MATRIX = 5

R_SHUNT = 0.1
R_RASPI_SHUNT = 0.05

LCD_RST_HIGH = 11  # out
WS2812B_DIN = 27  # out
ADJ_RST = 10  # out
PWR_SHDN = 7  # out
PWR_ENABLE = 18  # out

CHRG_STAT = 13  # in
PWR_ALERT = 12  # in
PWR_PV = 22  # in
PWR_CRITICAL = 24  # in
PWR_WARNING = 26  # in
PWR_TC = 28  # in

INT_CURSOR = 40

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

# ***** Variables *****

MCP23017_IODIRA = 0x00  # Bank A Direction Register
MCP23017_IODIRB = 0x01  # Bank B Direction Register
MCP23017_GPIOA = 0x12   # Bank A Data Register
MCP23017_GPIOB = 0x13   # Bank B Data Register

MCP23017_BANKA = [0, 0, 0, 0, 0, 0, 0, 0]
MCP23017_BANKB = [0, 0, 0, 0, 0, 0, 0, 0]

BUTTON_MATRIX_STATE = [0] * 9

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

# ***********************

GPIO.setWarnings(False)
GPIO.setmode(GPIO.BCM)


class ErrorCode:
    def __init__(self, name: str, description: str) -> None:
        self.name = name
        self.detailed_description = description


class LCDControl:
    # LCD command
    LCD_CLEAR_DISPLAY = 0x01
    LCD_RETURN_HOME = 0x02
    LCD_SET_DDRAM_ADDRESS = 0x80

    # LCD line addresses
    LINE_ADDRESSES = [0x00, 0x40]
    
    TEXT_LINES = ["SH Electronics", "Power Supply"]
    
    def __init__(self, name="DisplayController") -> None:
        self.name = name
        
        self.lcd_init()
        
        self.display_controller_running = False
        self.controller_thread = Thread(target=self.display_controller)
    
    def display_controller(self):
        while self.display_controller_running:
            self.lcd_display()
            time.sleep(.25)
        print("Display Controller stopped! No more content displayable.")
    
    def start(self):
        self.display_controller_running = True
        self.controller_thread.start()
    
    def stop(self):
        self.display_controller_running = False
        self.controller_thread.join()
    
    def write_cmd(self, cmd):
        with lock:
            bus = smbus2.SMBus(BUS_IO)
            bus.write_byte_data(LC_DISPLAY_ADDR, 0x00, cmd)
            time.sleep(0.001)  # wait 1ms for command to be processed
            
            bus.close()
    
    def write_data(self, data):
        with lock:
            bus = smbus2.SMBus(BUS_IO)
            bus.write_byte_data(LC_DISPLAY_ADDR, 0x40, data)
            time.sleep(0.001)  # wait 1ms for data to be processed
            
            bus.close()
        
    def lcd_init(self):
        # initialization sequence
        self.write_cmd(0x38)  # function set: 8-bit, 2-line, instruction table 1
        self.write_cmd(0x39)  # function set: 8-bit, 2-line, instruction table 0
        self.write_cmd(0x14)  # bias set, OSC frequency adjust
        self.write_cmd(0x78)  # contrast set
        self.write_cmd(0x56)  # power/icon/contrast control
        self.write_cmd(0x6C)  # follower control
        
        time.sleep(0.2)  # wait 200ms
        
        self.write_cmd(0x38)  # function set: 8-bit, 2-line, instruction table 0
        self.write_cmd(0x0C)  # display control: display ON, cursor OFF, blink OFF
        self.write_cmd(self.LCD_CLEAR_DISPLAY)  # clear display
        
        time.sleep(0.002)  # wait 2ms 
        
    
    def lcd_display(self):
        assert len(self.TEXT_LINES) <= 2, "Only 2 lines of text are supported"
        
        for i, line in enumerate(self.TEXT_LINES):
            assert len(line) <= 20, f"Line {i} is too long: {len(line)} characters"

            # set DDRAM address
            self.write_cmd(self.LCD_SET_DDRAM_ADDRESS + self.LINE_ADDRESSES[i])
            
            # write characters
            for char in line:
                self.write_data(ord(char))


class Raspi_Pwr_Controller:
    def __init__(self, name = "Raspi_Pwr_Controller") -> None:
        self.name = name
        
        self.raspi_pwr_running = False 
        self.raspi_controller_thread = Thread(target=self.raspi_controller, daemon=True)
        
    def raspi_controller (self):
        while self.raspi_pwr_running:
            a=1
            
            sleep(.5)
        print("Power monitoring for the Pi stopped. Power is still available.")
        
    def start(self):
        self.raspi_pwr_running = True
        self.raspi_controller_thread.start()
        
    def stop(self):
        self.raspi_pwr_running = False
        self.raspi_controller_thread.join()


class PowerControl:
    
    CHANNEL_DEFAULT_VOLTAGES = [
        [4.85, 5.15],
        [4.925, 5.075],
        [20.0, 25.5]
    ]
    CHANNEL_CURRENT_LIMITS = [
        1.0,
        4.0,
        2.0
    ]
    CHANNEL_SHUNT_VOLTAGES = [
        0.0,
        0.0,
        0.0
    ]
    CHANNEL_BUS_VOLTAGES = [
        0.0,
        0.0,
        0.0
    ]
    CHANNEL_CURRENT_VALUES = [
        0.0,
        0.0,
        0.0
    ]

    I_LIM_flag = False
    V_ALERT_flag = False
    
    def __init__(self, name="PowerControl") -> None:
        self.name = name

        self.pc_running = False
        self.ControllerThread = Thread(
            thread=self.PowerController, daemon=True)
        
        self.setup_INA3221()

    def PowerController(self):
        uptime = 0.0
        
        while self.pc_running:
            for i in range(len(self.CHANNEL_BUS_VOLTAGES)):
                self.CHANNEL_BUS_VOLTAGES[i] = read_INA3221_bus(i+1)
            for i in range(len(self.CHANNEL_SHUNT_VOLTAGES)):
                self.CHANNEL_SHUNT_VOLTAGES[i] = read_INA3221_shunt(i+1)
                self.CHANNEL_CURRENT_VALUES[i] = self.CHANNEL_SHUNT_VOLTAGES / R_SHUNT
            
            for i in range(len(self.CHANNEL_CURRENT_LIMITS)):
                self.I_LIM_flag = True if self.CHANNEL_SHUNT_VOLTAGES[i] > self.CHANNEL_CURRENT_LIMITS[i]+.1 else None
                self.V_ALERT_flag = True if self.CHANNEL_DEFAULT_VOLTAGES[i][0] <= self.CHANNEL_BUS_VOLTAGES[i] <= self.CHANNEL_DEFAULT_VOLTAGES[i][1] else None
                
        GPIO.output(PWR_ENABLE, GPIO.LOW)
        print("PowerController stopped. Shutting system down.")

    def start(self):
        THREADS.append(self.ControllerThread)
        
        self.pc_running = True
        self.ControllerThread.start()

    def stop(self):
        self.pc_running = False
        self.ControllerThread.join()

    def setup_INA3221(self):
        '''
        TODO
        '''
        with lock:
            bus = smbus2.SMBus(BUS_PWR)
            bus.write_word_data(PWR_MGMNT_ADDR, INA3221_REG_MASK_ENABLE, DISABLE_ALERTS)
            bus.close()


class MatrixKeypad:
    def __init__(self):
        self.scan_thread = Thread(target=self.scan_matrix)
        self.bus = smbus2.SMBus(BUS_MATRIX)
        time.sleep(.05)
        
        self.scan_thread.start()
        

    def scan_matrix(self):
        while True:
            for col in range(3):
                # Set only one column pin to low at a time, others to high
                data = 0b11111 ^ (1 << (7 - col))
                self.bus.write_byte(BUTTON_MATRIX_ADDR, data)
                
                # Read the row pins
                result = self.bus.read_byte(BUTTON_MATRIX_ADDR)

                for row in range(3):
                    # Check if the button at (row, col) is pressed
                    button_pressed = (result & (1 << row)) == 0
                    # Update the button state in the list
                    BUTTON_MATRIX_STATE[row * 3 + col] = button_pressed

            # A small delay to reduce CPU usage
            time.sleep(0.01)


def read_ADS1115_differential_voltage(address, positive_channel, negative_channel, differential_pair=None):
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


def read_ads1115_voltage(bus_num, address, channel):
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


def read_INA3221_shunt(channel):
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


def read_INA3221_bus(channel):
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
def write_mcp23017_bank(bank, data):
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
def read_mcp23017_bank(bank):
    with io_lock:
        bus = smbus2.SMBus(BUS_IO)
        
        if bank == 'A':
            return bus.read_byte_data(BUTTON_CURSOR_ADDR, MCP23017_GPIOA)
        elif bank == 'B':
            return bus.read_byte_data(BUTTON_CURSOR_ADDR, MCP23017_GPIOB)
        else:
            raise ValueError("Invalid bank. Choose either 'A' or 'B'")
        
        bus.close()


def setResistance_ADS5272(bus_num, device_address, resistance_int: int):
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


def setOutput_Voltage_Resistors(voltage: float):
    #
    # V = 0.760 x (1 + x / R2)
    # x=(10000 /19)*(25V−19)
    #
    
    r_upper = 0
    r_lower = 0
    if (voltage == 5.0):
        r_upper = 1024
        r_lower = 296
        
        setResistance_ADS5272(BUS_PWR, FB_ADJ_LOWER_ADDR, r_lower)
        setResistance_ADS5272(BUS_PWR, FB_ADJ_UPPER_ADDR, r_upper)
    else:
        None
        
        
def pwr_tc_isr():
    '''
    TODO
    '''


def pwr_pv_isr():
    '''
    TODO
    '''


def pwr_critical_isr():
    '''
    TODO
    '''
   
    
def pwr_warning_isr():
    '''
    TODO
    '''


def pwr_alert_isr():
    '''
    TODO
    '''


def chrg_stat_isr():
    '''
    TODO
    '''


def pwr_alert_isr():
    global eFUSE_ALERT
    
    eFUSE_ALERT = True
    GPIO.output(PWR_ENABLE, GPIO.LOW)


def int_cursor_isr():
    a=1

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
    GPIO.setup(PWR_SHDN, GPIO.OUT)
    GPIO.setup(PWR_ENABLE, GPIO.OUT)
    GPIO.setup(ADJ_RST, GPIO.OUT)
    GPIO.setup(WS2812B_DIN, GPIO.OUT)
    GPIO.setup(LCD_RST_HIGH, GPIO.OUT)
    
    GPIO.setup(PWR_ALERT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PWR_CRITICAL, GPIO.IN)
    GPIO.setup(PWR_PV, GPIO.IN)
    GPIO.setup(PWR_TC, GPIO.IN)
    GPIO.setup(PWR_WARNING, GPIO.IN)
    GPIO.setup(CHRG_STAT, GPIO.IN)
    
    GPIO.setup(INT_CURSOR, GPIO.IN)
    
    GPIO.output(PWR_SHDN, GPIO.LOW)
    GPIO.output(PWR_ENABLE, GPIO.LOW)
    GPIO.output(ADJ_RST, GPIO.LOW)
    GPIO.output(WS2812B_DIN, GPIO.LOW)
    GPIO.output(LCD_RST_HIGH, GPIO.LOW)
    
    GPIO.add_event_detect(PWR_ALERT, GPIO.FALLING, callback=pwr_alert_isr)
    GPIO.add_event_detect(INT_CURSOR, GPIO.FALLING, callback=int_cursor_isr)
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
            
    pwr_control = PowerControl("PowerControl")
    lcd_controller = LCDControl("DisplayController")
    raspi_controller = Raspi_Pwr_Controller("Controller")

    GPIO.output(PWR_SHDN, GPIO.HIGH)
    setOutput_Voltage_Resistors(DEFAULT_VOLTAGE)   

    pwr_control.start()
    raspi_controller.start()
    lcd_controller.start()
    sleep(.2)
    
    with lock:
        bus = smbus2.SMBus(BUS_IO)
        # Configure Bank A: 0b00111000 -> Pins 5, 6 and 7 as output, others as input
        # Configure Bank B: 0b10100000 -> Pins 6 and 7 as output, Pins 0 to 4 as input, others as input
        bus.write_byte_data(BUTTON_CURSOR_ADDR, MCP23017_IODIRA, 0b00111000)
        bus.write_byte_data(BUTTON_CURSOR_ADDR, MCP23017_IODIRB, 0b10100000)
        
        bus.close()
    
    sleep(1)


if __name__ == '__main__':
    sleep(STARTUP_DELAY)
    setup()
    main()

