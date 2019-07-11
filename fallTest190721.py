import sys
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BME280')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BMX055')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Camera')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/IM920')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Melting')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Motor')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/TSL2791')
import time
import serial
import pigpio
import BME280
import BMX055
import IM920
import Melting
import Motor
import TSL2791


luxstr = ["lux1", "lux2"]																#variable to show lux returned variables
bme280str = ["temp", "pres", "hum", "alt"]												#variable to show bme280 returned variables
bmx055str = ["accx", "accy", "accz", "gyrx", "gyry", "gyrz", "dirx", "diry", "dirz"]	#variable to show bmx055 returned variables
gpsstr = ["utctime", "lat", "lon", "sHeight", "gHeight"]								#variable to show GPS returned variables

t_setup = 10	#variable to set waiting time after setup

pi = pipgpio.pi()   #object of pigpio

def setup():
    BME280.bme280_setup()
	BME280.bme280_calib_param()
    BMX055.bmx055_setup()
	GPS.openGPS()

def close():
	GPS.closeGPS()
    #it may be necessarry to down low status some gpio pins 


if __name__ == "__main__":
    try:
        # ------------------- Setup Fhase ------------------- #
        setup()
        time.sleep(t_setup)

        # ------------------- Release Fhase ------------------- #


        # ------------------- Landing Fhase ------------------- #


        # ------------------- Melting Fhase ------------------- #
        Melting.Melting()

        # ------------------- Avoidance of Parachute Fhase ------------------- #


    except KeyboardInterrupt:
        close()
        print("Keyboard Interrupt")
    except Exception as e:
        close()
        print(e.message)