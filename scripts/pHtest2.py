import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize ADS1115
ads = ADS.ADS1115(i2c)

# Use Single-ended Mode (AIN0)
channel = AnalogIn(ads, ADS.P3)

# Read and Print Voltage
while True:
    voltage = channel.voltage  # Get voltage reading
    #print(f"pH Sensor Voltage: {voltage:.3f} V")
    #print(133.42*voltage**3 - 255.86*voltage**2 + 857.39*voltage, ' voltage', voltage, ' V')
    print(1894*voltage**2 + 534*voltage + 26.7, ' microS/cm')
    time.sleep(1)
