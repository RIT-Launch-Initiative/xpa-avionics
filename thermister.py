import os, math, time
from gpiozero import InputDevice,OutputDevice
import csv
import adafruit_lps2x
from cedargrove_nau7802 import NAU7802, LDOVoltage
import board

# thermistor script

# implement lookup table

# init constants
# TODO: some still need to be changed based on testing
SAMPLE_INTERVAL: float = .5 # how often data is sampled (seconds)
ROLLING_AVERAGE_DURATION: float = 10 # number of seconds of data to average
ROLLING_AVERAGE_SAMPLES: int = math.ceil(ROLLING_AVERAGE_DURATION / SAMPLE_INTERVAL) # number of samples to average
# heater constants
TEMP_UPPER_LIMIT: float = 150 # temp at which the heater should be turned off (celsius)
TEMP_LOWER_LIMIT: float = 130 # temp at which the heater should be turned on (celsius)
ALTITUDE_DISREEF: float = 1500 # altitude to start melting the wire (ft)
# thermistor constants
THERMISTOR_NOMINAL: float = 10000 # resistance at 25 degrees C   
TEMPERATURE_NOMINAL: float = 25 # temp. for nominal resistance (almost always 25 C)
NUM_SAMPLES: int = 5 # num of samples to average
B_COEFFICIENT: float = 3950; # The beta coefficient of the thermistor (usually 3000-4000)
SERIES_RESISTOR: float = 10000 # The value of the 'other' resistor

# init gpio, i2c
apogee_detect = InputDevice(4) # gpio pin to detect apogee from quark
heater = OutputDevice(6) # gpio pin to activate heater
# TODO: choose correct gpio pin for thermister and correct inputdevice
i2c = board.I2C() # i2c connection to read accelerometer data from icm 20649
#icm = adafruit_icm20x.ICM20649(i2c) # accelerometer object
lps = adafruit_lps2x.LPS22(i2c) # barometer / altimeter object
nau7802 = NAU7802(board.I2C(), address=0x2A, active_channels=1)
nau7802.gain = 1



def altitude(pressure: float) -> float:
    """
    Convert pressure to altitude
    """
    return 145366.45 * (1 - (pressure / 1013.25)**(1/5.255))


def temp(voltage: float) -> float:
    R = -4700*voltage/(voltage-3)
    print(R)
    # 130C = 2.94V
    # 150C = 1.77V
    
    """
    Convert voltage to temperature
    """
    
def zero_channel():
    print("channel %d zero offset: %7.0f" % (nau7802.channel, nau7802.calibrate("INTERNAL")))
    print("channel %d calibrate GAIN: %7.0f" % (nau7802.channel, nau7802.calibrate("GAIN")))
    print("...channel %1d zeroed" % nau7802.channel)

def read_raw_value(samples=2):
    sample_sum=0
    sample_count=samples
    while sample_count > 0:
        while not nau7802.available():
            pass
        sample_sum += nau7802.read()
        sample_count -= 1
    return int(sample_sum / samples)

def main():
    time.sleep(3)
    nau7802.channel=1
    zero_channel()
    

    while True:
        nau7802.channel=1
        value=read_raw_value()
        V=float("%7.0f"%value)
        print(V)
        T=temp(V)
        print(T)

    print("READY")
    
    # wait for boost
    # while(not IMU_read() or not quark_read()):
    #   time.sleep(sampleTimeDelay)

main()

# redundant check for boost
# wait for apogee

#while(not apogee_detect.is_active): # wait until quark sends apogee signal
#    time.sleep(SAMPLE_INTERVAL)

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
    print('voltage: ', voltage)
    temperature = temp(voltage) # convert voltage to temp via lookup table
    print('temperature: ', temperature)
    #temp = temp(voltage) # convert voltage to temp via lookup table

    # check if we need to turn on or off the heater
    # by trying to maintain a temperature range
    if(temperature < TEMP_LOWER_LIMIT):
        heater.on() # turn on heater 
        print('heater on')
    if(temperature > TEMP_UPPER_LIMIT):
        heater.off() # turn off heater
        print('heater off')
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

main()