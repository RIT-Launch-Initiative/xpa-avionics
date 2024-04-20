import os, math, time
from gpiozero import InputDevice,OutputDevice,Button
import csv
import adafruit_lps2x
#3import adafruit_icm20x
from cedargrove_nau7802 import NAU7802, LDOVoltage
import board
import RPi.GPIO as GPIO           # import RPi.GPIO module  


# thermistor script

# implement lookup table

# init constants
SAMPLE_INTERVAL: float = .15 # how often data is sampled (seconds) # TODO:
ROLLING_AVERAGE_DURATION: float = 10 # number of seconds of data to average # TODO: 
ROLLING_AVERAGE_SAMPLES: int = math.ceil(ROLLING_AVERAGE_DURATION / SAMPLE_INTERVAL) # number of samples to average
# heater constants
TEMP_UPPER_LIMIT: float = 150 # temp at which the heater should be turned off (celsius)
TEMP_LOWER_LIMIT: float = 130 # temp at which the heater should be turned on (celsius)
R_UPPER_LIMIT: float = 1770 # resistance at which the heater should be turned off (ohms)
R_LOWER_LIMIT: float = 2970 # resistance at which the heater should be turned on 
ALTITUDE_DISREEF: float = 1000 # altitude to start melting the wire (ft) # TODO:


# init gpio, i2c
apogee_detect = Button(15) # gpio pin to detect apogee from quark # TODO:
heater = OutputDevice(20) # gpio pin to activate heater # TODO: 
i2c = board.I2C() # i2c connection to read accelerometer data from icm 20649
#icm = adafruit_icm20x.ICM20649(i2c) # accelerometer object
lps = adafruit_lps2x.LPS22(i2c) # barometer / altimeter object
nau7802 = NAU7802(board.I2C(), address=0x2A, active_channels=1)
# ser = serial.Serial( # serial connection to read accelerometer data from quark
#     port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
#     baudrate = 9600, 
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE,
#     bytesize=serial.EIGHTBITS,
#     timeout=1
# )
nau7802.gain = 1



def altitude(pressure: float) -> float:
    """
    Convert pressure to altitude
    """
    return 145366.45 * (1 - (pressure / 1013.25)**(1/5.255))


def temp(voltage: float) -> float:
    R = -4700*voltage/(voltage-3)
    print(R)
    # 130C = 2.94 R
    # 150C = 1.77 R
    
    # 140C = 2.27 R 
    # 
    
    """
    Convert voltage to temperature
    """
    return R
    
def imu_read() -> bool:
    """
    Returns true if the acceleration magnitude is greater than 5
    """
    icm.acceleration # read acceleration data
    return math.sqrt(icm.acceleration[0]**2 + icm.acceleration[1]**2 + icm.acceleration[2]**2) > 5
    
    
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
    GPIO.setmode(GPIO.BCM)            # choose BCM or BOARD  
    GPIO.setup(20, GPIO.OUT) # set a port/pin as an output   

    print("READY")
    
    current_altitude = altitude(lps.pressure) # get altitude from lps sensor
    print(current_altitude)
    offset = current_altitude
    print(current_altitude - offset)
    
    # wait for boost
    #while(not imu_read()): # or not quark_read()):
    #    time.sleep(SAMPLE_INTERVAL)
    
    # wait for boost
    while (True):
        break
        current_altitude = altitude(lps.pressure) - offset
        print(current_altitude)
        if current_altitude > 6: # TODO: 
            time.sleep(.5)
            current_altitude = altitude(lps.pressure) - offset
            print(current_altitude)
            if current_altitude > 6:
                break
    
    print('boost detected')
        
    # boost detected

    # wait for apogee
    
    alts = [0, 0, 0]
    n=0
    while (True):
        break
        current_altitude = altitude(lps.pressure) - offset
        print(current_altitude)
        if alts[0] > (current_altitude):
            break
        alts[0] = current_altitude
        n+=1
        if n > 2:
            n-=3
        time.sleep(.66) # TODO: 
        """ 
        if current_altitude > max_altitude:
            max_altitude = current_altitude
        if (current_altitude < ALTITUDE_DISREEF) & (max_altitude > (ALTITUDE_DISREEF+1000)):
            break
         """    
    print('descend detected')
    # descent state
    # wait for dis-reefing altitude

    # descent, but not dis-reefing yet
    # start warming up heater
    flag = 0
    while(True): # current_altitude > ALTITUDE_DISREEF): # while we are above the dis-reef altitude
        # read thermistor
        nau7802.channel=1
        value=read_raw_value()
        V=1.5*float("%7.0f"%value)/(16777215) # convert to voltage
        #print('voltage: ', V)
        # TODO convert voltage to temperature
        #temperature = temp(voltage) # convert voltage to temp via lookup table
        R = temp(V) # convert voltage to resistance
        #print('temperature: ', temperature)
        #temp = temp(voltage) # convert voltage to temp via lookup table

        # check if we need to turn on or off the heater
        # by trying to maintain a temperature range
        #if(temperature < TEMP_LOWER_LIMIT):
        if(R > R_LOWER_LIMIT):
            if (flag == 1):
                break
            heater.on() # turn on heater 
            print('heater on')
            GPIO.output(20, 1)       # set port/pin value to 1/GPIO.HIGH/Truex  
        #if(temperature > TEMP_UPPER_LIMIT):
        if(R < R_UPPER_LIMIT):
            flag = 1
            heater.off() # turn off heater
            print('heater off')
            GPIO.output(20, 0)
        # update altitude for next iteration
        current_altitude = altitude(lps.pressure) - offset
        print(current_altitude)
        if (current_altitude < ALTITUDE_DISREEF):
            pass # break
        

    # dis-reefing altitude reached
    print('disreef')
    GPIO.output(20, 1) # turn on heater fully
    # wait for deployment
    while (True):
        value=read_raw_value()
        V=1.5*float("%7.0f"%value)/(16777215) # convert to voltage
        #print('voltage: ', V)
        R = temp(V)
        print(R)
    #time.sleep(20)
    heater.off() # turn off heater

main()