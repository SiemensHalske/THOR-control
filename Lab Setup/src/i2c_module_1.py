from threading import Lock
import bme680
from water_distribution_system import ADDR_BME680, BUS_MODULES


def i2c_module_1(module_bus_lock: Lock):
    # Create an instance of the BME680 class
    sensor = bme680.BME680(i2c_addr=ADDR_BME680, bus_num=BUS_MODULES)

    # Set up the sensor settings
    sensor.set_humidity_oversample(bme680.OS_2X)
    sensor.set_pressure_oversample(bme680.OS_4X)
    sensor.set_temperature_oversample(bme680.OS_8X)
    sensor.set_filter(bme680.FILTER_SIZE_3)
    sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)
    sensor.set_gas_heater_temperature(320)
    sensor.set_gas_heater_duration(150)
    sensor.select_gas_heater_profile(0)

    # Read measurements
    if sensor.get_sensor_data():
        temperature = sensor.data.temperature
        humidity = sensor.data.humidity
        pressure = sensor.data.pressure
        gas_resistance = sensor.data.gas_resistance if sensor.data.heat_stable else None

        return {
            "temperature": temperature,
            "humidity": humidity,
            "pressure": pressure,
            "gas_resistance": gas_resistance
        }
    else:
        return None