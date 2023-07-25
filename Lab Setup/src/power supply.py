'''

'''


import RPi.GPIO as GPIO

import os
import smbus2
import sys
import time

from time import sleep


# ****    DEFINES    ****

BUS_NUM = 1

STARTUP_DELAY = 1.5

R_LIM = 0.0
I_IN = 0.0
V_IN = 0.0

PWR_ENABLE = 40
PWR_ALERT = 38

PWR_SHDN = 36

I_LIM_ADDR = 0xF
I_CONTROL_ADDR = 0x2F
LC_DISPLAY_ADDR = 0x78
V_OUT_CONTROL_ADDR = 0x2C

I_V_IN_MEASURE_ADDR = 0x48
I_V_OUT_MEASURE_ADDR = 0x4A

ADDR_LIST = [I_LIM_ADDR, I_CONTROL_ADDR, LC_DISPLAY_ADDR, V_OUT_CONTROL_ADDR, I_V_IN_MEASURE_ADDR, I_V_OUT_MEASURE_ADDR]

# ***********************

GPIO.setWarnings(False)
GPIO.setmode(GPIO.BCM)

class ErrorCode:
    def __init__(self, name: str, description: str) -> None:
        self.name = name
        self.detailed_description = description
        

class VoltageDivider:
    def V_IN(voltage: float):
        resistor_1 = 1000000
        resistor_2 = 500000

        
def setResistance_ADS5272(device_address, resistance_int: int):
    bus = smbus2.SMBus(BUS_NUM)
    
    # Ensure the resistance_int is within the valid range (0 to 1023)
    resistance_int = max(0, min(1023, resistance_int))

    # Calculate the command byte and data bytes based on the desired resistance_int
    command_byte = 0x00  # Command byte for write to RDAC register
    data_byte1 = (resistance_int >> 8) & 0xFF  # Most significant byte
    data_byte2 = resistance_int & 0xFF         # Least significant byte



    # Send the data to the device
    bus.write_i2c_block_data(device_address, command_byte, [data_byte1, data_byte2])
    
    bus.close()

def setResistance_MCP45HV31(device_address, resistance_int: int):
    def is_reachable():
        try:
            # Try to open the I2C bus and see if the device responds
            i2c_bus = smbus2.SMBus(BUS_NUM)
            i2c_bus.read_byte_data(device_address, 0x00)
            i2c_bus.close()
            return True
        except IOError:
            return False
        
    if not is_reachable():
        return False
        
    bus = smbus2.SMBus(BUS_NUM)
    
    # Ensure the value is within the valid range (0 to 255)
    resistance_int = max(0, min(255, resistance_int))

    command_byte = 0x00  # Command byte for write to RDAC register
    data_byte = resistance_int

    # Send the data to the device
    bus.write_i2c_block_data(device_address, command_byte, [data_byte])
    
    bus.close()

def read_ADS1115_differential_voltage(address, positive_channel, negative_channel):
    # Konfiguration für den Differentialmodus
    config_register = 0b1100000110000011  # Binary: 0xC383

    # Überprüfen, ob die Kanäle gültig sind (AIN0 und AIN1 oder AIN2 und AIN3)
    if (positive_channel == 0 and negative_channel == 1) or (positive_channel == 2 and negative_channel == 3):
        # Öffne den I2C-Bus
        bus = smbus2.SMBus(1)

        # Senden der Konfigurationsdaten an das ADC
        bus.write_i2c_block_data(address, 1, [config_register >> 8, config_register & 0xFF])

        # Warte kurz auf die Konvertierung (abhängig von der gewählten Abtastrate)
        time.sleep(0.01)

        # Lesen der Rohdaten (16 Bit im Two's-Complement-Format)
        data = bus.read_i2c_block_data(address, 0, 2)
        raw_value = (data[0] << 8) | data[1]

        # Der ADC hat eine Auflösung von 16 Bit und misst im Bereich von +/- 6.144V
        # Berechnung der gemessenen Spannung
        voltage_range = 6.144  # voltage range in Volt
        voltage = (raw_value / 32767.0) * voltage_range

        return voltage
    else:
        print("Invalid channel pairing. Valid pairs: AIN0 & AIN1 or AIN2 & AIN3.")
        return -1
    
def read_ads1115_voltage(address, channel):
    # Überprüfen, ob die Kanalnummer gültig ist (0 bis 3)
    if channel not in range(4):
        print("Invalid channel. Valid channel numbers: 0, 1, 2, 3.")
        return None

    # Konfigurationsregister: Single-Shot-Modus, +/- 6.144V Bereich, 128 Samples pro Sekunde
    config_register = 0b1100000110000011  # Binary: 0xC383
    config_register |= (channel << 12)  # Einstellung des gewünschten Kanals im Konfigurationsregister

    # Öffne den I2C-Bus
    bus = smbus2.SMBus(BUS_NUM)

    # Senden der Konfigurationsdaten an das ADC
    bus.write_i2c_block_data(address, 1, [config_register >> 8, config_register & 0xFF])

    # Warte kurz auf die Konvertierung (abhängig von der gewählten Abtastrate)
    time.sleep(0.008)

    # Lesen der Rohdaten (16 Bit im Two's-Complement-Format)
    data = bus.read_i2c_block_data(address, 0, 2)
    bus.close()
    
    raw_value = (data[0] << 8) | data[1]

    # Der ADC hat eine Auflösung von 16 Bit und misst im Bereich von +/- 6.144V
    # Berechnung der gemessenen Spannung
    voltage_range = 6.144  # Spannungsrange in Volt
    voltage = (raw_value / 32767.0) * voltage_range  # Umrechnung in Float-Wert

    return voltage

def main():
    '''
        Explanation:
        
        Params:
        
        Returns:
            None
    '''
    
def setup():
    '''
        Explanation:
            - Checks if the 6 needed i2c-addresses for the system are set properly and no corruptions occur
        Params:
        
        Returns:
            None
    '''
    
    GPIO.setup(PWR_SHDN, GPIO.OUT)
    GPIO.setup(PWR_ENABLE, GPIO.OUT)
    GPIO.setup(PWR_ALERT, GPIO.IN)
    
    GPIO.output(PWR_ENABLE, GPIO.LOW)
    GPIO.output(PWR_SHDN, GPIO.LOW)
    
    for addr in ADDR_LIST:
        if addr == None:
            sys.exit(0)
            
    R_LIM = 0
    setResistance_MCP45HV31(I_LIM_ADDR, R_LIM)
    
    V_IN = read_ADS1115_differential_voltage(I_V_IN_MEASURE_ADDR, 2, 3)
    
    
if __name__ == '__main__':
    sleep(STARTUP_DELAY)
    setup()