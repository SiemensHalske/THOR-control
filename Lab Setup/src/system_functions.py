import smbus2
import RPi.GPIO as GPIO

from water_distribution_system import ADDR_IO, BUS_IO


def read_pcf8574(io_lock):
    with io_lock:
        bus = smbus2.SMBus(BUS_IO) # 1 indicates the I2C bus number, adjust as needed
        data = bus.read_byte(ADDR_IO)
        bus.close()
        data_list = [(data >> i) & 0x01 for i in range(8)]
        return data_list
    
def read_pcf8574_bus_var(bus_num, addr, io_lock):
    with io_lock:
        bus = smbus2.SMBus(bus_num) # 1 indicates the I2C bus number, adjust as needed
        data = bus.read_byte(addr)
        bus.close()
        data_list = [(data >> i) & 0x01 for i in range(8)]
        return data_list

def get_gpio_state(gpio_pin):
    return GPIO.input(gpio_pin)