# Introduction
Micropython module to communicate with Melexis MLX90621 16 x 4 Pixel Thermal Imager.
# Usage
This module has been created for, and tested with, the Micropython Pyboard v1.1. Changes to the I2C commands may be needed if moving to another Micropython platform and note that the I2C protocol must be able to support repeated start commands.

Usage of the module requires instantiation of an MLX90621 instance (specifying the MLX90621 refresh rate), followed by sensor initialisation, and finally acquiring measurements on demand. Consult the MLX90621 datasheet for details of refresh rate - this defines the rate the sensor samples at, but it is up to the user how frequently new results are pulled from the device itself, and therefore limited to the processing overhead of each sensor_read call. The results of the latest measurement are held in two internal 4 x 16 arrays in the class instance, and are availiable as either floats (as degrees C) or integers (as 10 x degrees C to provide more precision).

```python
# Create MLX90621 instance and initialise sensor at 8 Hz (the MLX90621 is now sampling at 8 Hz)
ir_sensor = MLX90621(refresh_rate=8, i2c=i2c)
ir_sensor.mlx90621_init()

# Read in a single 16 x 4 "frame" of temperature readings (this reads temperatures into python arrays)
ir_sensor.mlx90621_read_ir()

# Access the temperature values for your application (i: 0-3 and j: 0-16)
# degrees C (10x) integers
ir_sensor.temperatures_int[i][j]
# degrees C floats
ir_sensor.temperatures_flt[i][j]
```

# Full Code Example for Micropython Pyboard v1.1
This code should be saved in a file called main.py and should be placed alongside mlx90621.py in the root Pyboard directory
```python
import time
from machine import I2C
from mlx90621 import MLX90621

# Enable I2C bus 1 (X9, X10) on Pyboard v1.1 and set clock rate to 400 kHz
i2c = I2C(1, freq=400000)

# Create MLX90621 instance and initialise sensor
ir_sensor = MLX90621(refresh_rate=8, i2c=i2c)
ir_sensor.mlx90621_init()

# Very basic example of loop to get temperatures and return via serial to terminal
while True:
    ir_sensor.mlx90621_read_ir()
    time.sleep_us(500000) # Long delay to provide sensible screen update
    for i in range(4):
        line = ["{0:.1f}".format(ir_sensor.temperatures_flt[i][j]) for j in range(16)]
        print('\t'.join(line))
    print('') # Print block separator
```
