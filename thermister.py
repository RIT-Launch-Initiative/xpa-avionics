import os, math, time
from gpiozero import InputDevice,OutputDevice,Button
import csv
import adafruit_lps2x
import adafruit_icm20x
from cedargrove_nau7802 import NAU7802, LDOVoltage
import board
import RPi.GPIO as GPIO           # import RPi.GPIO module
from datetime import datetime  


# thermistor script

# implement lookup table

# init constants
SAMPLE_INTERVAL: float = .1 # how often data is sampled (seconds) # TODO:
#ROLLING_AVERAGE_DURATION: float = 10 # number of seconds of data to average # TODO: 
#ROLLING_AVERAGE_SAMPLES: int = math.ceil(ROLLING_AVERAGE_DURATION / SAMPLE_INTERVAL) # number of samples to average
# heater constants
TEMP_UPPER_LIMIT: float = 200 # temp at which the heater should be turned off (celsius)
TEMP_LOWER_LIMIT: float = 190 # temp at which the heater should be turned on (celsius)
R_UPPER_LIMIT: float = 582 # resistance at which the heater should be turned off (ohms)
R_LOWER_LIMIT: float = 650 # resistance at which the heater should be turned on 
ALTITUDE_DISREEF: float = 1500 # altitude to start melting the wire (ft) # TODO:
ALTITUDE_BOOST: float = 300 # altitude considered to be the boost (ft) # TODO:


# init gpio, i2c
#apogee_detect = Button(15) # gpio pin to detect apogee from quark
heater = OutputDevice(20) # gpio pin to activate heater
i2c = board.I2C() # i2c connection to read accelerometer data from icm 20649
icm = adafruit_icm20x.ICM20649(i2c) # accelerometer object
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
    
    # 200C = .582 R
    # 190C = .712 R 
    
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
    # init datalogging
    file = open("/home/pi/data_log.csv", "a")
    if os.stat("/home/pi/data_log.csv").st_size == 0:
        file.write("Time,Altitude,Resistance\n")
            
    time.sleep(3)
    nau7802.channel=1
    zero_channel()
    GPIO.setmode(GPIO.BCM)            # choose BCM or BOARD  
    GPIO.setup(20, GPIO.OUT) # set a port/pin as an output   

    print("READY")
    now = datetime.now()
    file.write(str(now)+",READY\n")
    file.write(str(now)+",PAD STAGE\n")
    
    current_altitude = altitude(lps.pressure) # get altitude from lps sensor
    print(current_altitude)
    now = datetime.now()
    file.write(str(now)+","+current_altitude+"\n")
    offset = current_altitude
    print(current_altitude - offset)
    
    # wait for boost
    #while(not imu_read()): # or not quark_read()):
    #    time.sleep(SAMPLE_INTERVAL)
    
    # wait for boost
    now = datetime.now()
    file.write(str(now)+",Waiting for Boost\n")
    file.flush()
    while (True):
        current_altitude = altitude(lps.pressure) - offset
        print(current_altitude)
        now = datetime.now()
        file.write(str(now)+","+current_altitude+"\n")
        file.flush()
        if current_altitude > ALTITUDE_BOOST: 
            time.sleep(.5)
            current_altitude = altitude(lps.pressure) - offset
            print(current_altitude)
            now = datetime.now()
            file.write(str(now)+","+current_altitude+"\n")
            file.flush()
            if current_altitude > ALTITUDE_BOOST:
                now = datetime.now()
                file.write(str(now)+",Current Altitude > Boost\n")
                file.flush()
                break
    
    print('boost detected')
    now = datetime.now()
    file.write(str(now)+",BOOST STAGE\n")
    file.flush()
        
    # boost detected

    # wait for apogee
    
    alts = [0, 0, 0]
    n=0
    while (True):
        current_altitude = altitude(lps.pressure) - offset
        print(current_altitude)
        now = datetime.now()
        if alts[0] > (current_altitude):
            break
        alts[0] = current_altitude
        n+=1
        if n > 2:
            n-=3
        file.write(str(now)+","+current_altitude+"\n")
        file.flush()
        time.sleep(.33) # TODO: 
        """ 
        if current_altitude > max_altitude:
            max_altitude = current_altitude
        if (current_altitude < ALTITUDE_DISREEF) & (max_altitude > (ALTITUDE_DISREEF+1000)):
            break
         """    
    print('descend detected')
    now = datetime.now()
    file.write(str(now)+",DESCENT STAGE\n")
    file.flush()
    # descent state
    # wait for dis-reefing altitude

    # descent, but not dis-reefing yet
    # start warming up heater
    flag = 0
    while (current_altitude > ALTITUDE_DISREEF): # while we are above the dis-reef altitude
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
        now = datetime.now()
        file.write(str(now)+","+current_altitude+","+R+"\n")
        if(R > R_LOWER_LIMIT):
            heater.on() # turn on heater 
            print('heater on')
            file.write(str(now)+",HEAT ON\n")
            GPIO.output(20, 1)       # set port/pin value to 1/GPIO.HIGH/Truex  
        #if(temperature > TEMP_UPPER_LIMIT):
        if(R < R_UPPER_LIMIT):
            flag = 1
            heater.off() # turn off heater
            print('heater off')
            file.write(str(now)+",HEAT OFF\n")
            GPIO.output(20, 0)
        file.flush()
        # update altitude for next iteration
        time.sleep(SAMPLE_INTERVAL)
        current_altitude = altitude(lps.pressure) - offset
        print(current_altitude)
        if (current_altitude < ALTITUDE_DISREEF):
            now = datetime.now()
            file.write(str(now)+","+current_altitude+"\n")
            break
        

    # dis-reefing altitude reached
    print('disreef')
    file.write(str(now)+",DISREEF STAGE\n")
    
    GPIO.output(20, 1) # turn on heater fully
    # wait for deployment
    while (True):
        current_altitude = altitude(lps.pressure) - offset
        value=read_raw_value()
        V=1.5*float("%7.0f"%value)/(16777215) # convert to voltage
        #print('voltage: ', V)
        R = temp(V)
        print(R)
        file.write(str(now)+","+current_altitude+","+R+"\n")
        
    #time.sleep(20) # TODO: 
    heater.off() # turn off heater
    file.close()

main()