import os, math, time
from gpiozero import InputDevice, OutputDevice
import csv
import adafruit_lps2x
from cedargrove_nau7802 import NAU7802
import board

# thermistor script

# implement lookup table

# init constants
# TODO: some still need to be changed based on testing
SAMPLE_INTERVAL: float = .5 # how often data is sampled (seconds)
ROLLING_AVERAGE_DURATION: float = 10 # number of seconds of data to average
TEMP_UPPER_LIMIT: float = 200 # temp at which the heater should be turned off (celsius)
TEMP_LOWER_LIMIT: float = 180 # temp at which the heater should be turned on (celsius)
ALTITUDE_DISREEF: float = 1500 # altitude to start melting the wire (ft)
ROLLING_AVERAGE_SAMPLES: int = math.ceil(ROLLING_AVERAGE_DURATION / SAMPLE_INTERVAL) # number of samples to average

# init gpio, i2c
apogee_detect = InputDevice(4) # gpio pin to detect apogee from quarkde
heater = OutputDevice(6) # gpio pin to activate heater
# TODO: choose correct gpio pin for thermister and correct inputdevice
thermistor = InputDevice(7) # gpio pin to read thermister
i2c = board.I2C() # i2c connection to read accelerometer data from icm 20649
#icm = adafruit_icm20x.ICM20649(i2c) # accelerometer object
lps = adafruit_lps2x.LPS22(i2c) # barometer / altimeter object
nau7802 = NAU7802(board.I2C(), address=0x2A, active_channels=2)

def zero_channel():
    """Initiate internal calibration for current channel.Use when scale is started,
    a new channel is selected, or to adjust for measurement drift. Remove weight
    and tare from load cell before executing."""
    print(
        "channel %1d calibrate.INTERNAL: %5s"
        % (nau7802.channel, nau7802.calibrate("INTERNAL"))
    )
    print(
        "channel %1d calibrate.OFFSET:   %5s"
        % (nau7802.channel, nau7802.calibrate("OFFSET"))
    )
    print("...channel %1d zeroed" % nau7802.channel)
    
def read_raw_value(samples=2):
    """Read and average consecutive raw sample values. Return average raw value."""
    sample_sum = 0
    sample_count = samples
    while sample_count > 0:
        while not nau7802.available():
            pass
        sample_sum = sample_sum + nau7802.read()
        sample_count -= 1
    return int(sample_sum / samples)

def altitude(pressure: float) -> float:
    """
    Convert pressure to altitude
    """
    return 145366.45 * (1 - (pressure / 1013.25)**(1/5.255))


def temp(voltage: float) -> float:
    """
    Convert voltage to temperature
    """
    

def main():
    enabled = nau7802.enable(True)
    print(f"NAU7802 enabled: {enabled}")
    
    nau7802.channel = 1
    zero_channel()  # Calibrate and zero channel
    nau7802.channel = 2
    zero_channel()  # Calibrate and zero channel
    
    while True:
        nau7802.channel = 1
        value = read_raw_value()
        print("channel %1.0f raw value: %7.0f" % (nau7802.channel, value))

        nau7802.channel = 2
        value = read_raw_value()
        print("channel %1.0f raw value: %7.0f" % (nau7802.channel, value))

    print("READY")
    
    # wait for boost
    # while(not IMU_read() or not quark_read()):
    #   time.sleep(sampleTimeDelay)

    # redundant check for boost
    # wait for apogee
    print('waiting for apogee signal from quark...')
    while(not apogee_detect.is_active): # wait until quark sends apogee signal
        time.sleep(SAMPLE_INTERVAL)
    print('apogee reached.')
    # apoogee reached

    current_altitude = altitude(lps.pressure) # get altitude from lps sensor
    print(current_altitude)
    offset = current_altitude
    print(current_altitude - offset)
    
    # descent, but not dis-reefing yet
    # start warming up heater
    while(current_altitude > ALTITUDE_DISREEF): # while we are above the dis-reef altitude
        # read thermistor
        voltage = thermistor.value # read voltage from thermistor
        # TODO: convert voltage to temperature
        temperature = temp(voltage) # convert voltage to temp via lookup table
        #temp = temp(voltage) # convert voltage to temp via lookup table

        # check if we need to turn on or off the heater
        # by trying to maintain a temperature range
        if(temp < TEMP_LOWER_LIMIT):
            heater.on() # turn on heater 
        if(temp > TEMP_UPPER_LIMIT):
            heater.off() # turn off heater
        time.sleep(SAMPLE_INTERVAL)
        # update altitude for next iteration
        current_altitude = altitude(lps.pressure) - offset
        print(current_altitude)

    print('heater fully on')

    # dis-reefing altitude reached
    heater.on() # turn on heater fully
    # wait for deployment
    time.sleep(20)
    heater.off() # turn off heater
