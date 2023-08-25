from threading import Lock

import smbus2

from water_distribution_system import ADDR_INA3221, BUS_IO


def read_ina3221(module_bus_lock: Lock):
    with module_bus_lock:
        bus = smbus2.SMBus(BUS_IO)

        # Initialize an empty dictionary to store the results
        results = {}

        # Loop through each channel (1, 2, 3)
        for channel in range(1, 4):
            # Calculate the register addresses for shunt and bus voltage
            shunt_reg = 0x01 + (channel - 1) * 2
            bus_reg = 0x02 + (channel - 1) * 2

            # Read shunt voltage (in 40uV units) and convert to volts
            shunt_voltage_raw = bus.read_word_data(ADDR_INA3221, shunt_reg)
            shunt_voltage = ((shunt_voltage_raw & 0xFF) << 8 | (shunt_voltage_raw & 0xFF00) >> 8) * 0.00004

            # Read bus voltage (in 8mV units) and convert to volts
            bus_voltage_raw = bus.read_word_data(ADDR_INA3221, bus_reg)
            bus_voltage = ((bus_voltage_raw & 0xFF) << 8 | (bus_voltage_raw & 0xFF00) >> 8) * 0.008

            # Calculate current based on shunt voltage and resistance (0.1 ohms)
            current = shunt_voltage / 0.1

            # Store the results in the dictionary
            results[channel] = {
                'shunt_voltage': shunt_voltage,
                'bus_voltage': bus_voltage,
                'current': current
            }

        return results if None not in results.values() else -1


def i2c_module_3(module_bus_lock: Lock):
    data = read_ina3221(module_bus_lock=module_bus_lock)
    return data