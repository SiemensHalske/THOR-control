from threading import Lock
import busio
import board
import adafruit_mlx90393

def read_mlx90393_parameters(module_bus_lock: Lock):
    with module_bus_lock:
        SCL = 5
        SDA = 6
        i2c = busio.I2C(SCL, SDA)
        sensor = adafruit_mlx90393.MLX90393(i2c)

        # Read the magnetic field and temperature
        x, y, z = sensor.magnetic
        temperature = sensor.temperature

        # Close the I²C bus explicitly
        i2c.deinit()

        return [x, y, z, temperature] if None not in [x, y, z, temperature] else None
        return -1

def i2c_module_2(module_bus_lock: Lock):
    data = read_mlx90393_parameters(module_bus_lock=module_bus_lock)
    return data