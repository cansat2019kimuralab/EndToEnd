# -*- coding: utf-8 -*-
import sys
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BME280')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BMX055')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Camera')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/GPS')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/IM920')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Melting')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Motor')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/TSL2561')
sys.path.append('/home/pi/git/kimuralab/Other')
import binascii
import difflib
import pigpio
import serial
import time
import traceback

import BMX055
import BME280
import Capture
import GPS
import IM920
import Melting
import Motor
import Other
import TSL2561


phaseChk = 0	#variable for phase Check

# --- variable of time setting --- #
t_start = 0		#start time
t_sleep = 3		#sleep time
t_release = 5	#release time
t_landing = 5	#landing time
t_melting = 5	#melting time
t_running = 20	#running time
t1 = 0.0		#vairable to store phase start time
t2 = 0.0		#vairable to store time

# --- variable of Log path --- #
phaseLog = "phaseLo.txt"
sleepLog = "sleepLog.txt"
releaseLog = "releaseLog.txt"
landingLog = "landingLog.txt"
meltingLog = "meltingLog.txt"
runningLog = "runningLog.txt"
photoLog = "photoLog.txt"
errorLog = "errorLog.txt"

photoPath = "photo/"

pi = pigpio.pi()	#object to set pigpio

def setup():
	global phaseChk
	pi.set_mode(17,pigpio.OUTPUT)
	pi.set_mode(22,pigpio.OUTPUT)
	pi.write(22,1)
	pi.write(17,0)
	time.sleep(1)
	BME280.bme280_setup()
	BME280.bme280_calib_param()
	BMX055.bmx055_setup()
	GPS.openGPS()

	with open(phaseLog, 'a') as f:
		pass

	phaseChk = int(Other.phaseCheck(phaseLog))

def close():
	GPS.closeGPS()
	Motor.motor(0, 0, 2)
	Motor.motor_stop()
	pi.write(17, 0)

if __name__ == "__main__":
	try:
		t_start = time.time()
		# ------------------- Setup Fhase ------------------- #
		setup()
		print("Program Start  {0}".format(time.time()))
		print(phaseChk)
		if(phaseChk <= 1):
			IM920.Send("P1S")
			Other.saveLog(phaseLog, "1", "Program Started", time.time() - t_start)
			IM920.Send("P1F")

		# ------------------- Sleep Phase --------------------- #
		print("Sleep Phase Started  {0}".format(time.time()))		
		Other.saveLog(phaseLog, "2", "Sleep Phase Started", time.time() - t_start)
		if(phaseChk <= 2):
			IM920.Send("P2S")
			pi.write(22, 0)		#IM920 Turn Off
			t_wait_start = time.time()
			while(time.time() - t_wait_start <= t_sleep):
				bme280Data = BME280.bme280_read()	#Read BME280 data
				bmx055Data = BMX055.bmx055_read()	#Read BMX055 data
				luxData = TSL2561.readLux()			#Read TSL2561 data
				gpsData = GPS.readGPS()				#Read GPS data
				Other.saveLog(sleepLog, time.time() - t_start, gpsData, bme280Data, bmx055Data, luxData)
				photoName = Capture.Capture(photoPath)
				time.sleep(1)
				t2=time.time()

		# ------------------- Release Fhase ------------------- #
		Other.saveLog(phaseLog, "3", "Release Phase Started", time.time() - t_start)
		print("Release Judgement Program Start  {0}".format(time.time()))
		if(phaseChk <= 3):
			t1 = time.time()
			t2 = t1
			while(t2-t1 <= t_release):
				bme280Data = BME280.bme280_read()	#Read BME280 data
				bmx055Data = BMX055.bmx055_read()	#Read BMX055 data
				luxData = TSL2561.readLux()			#Read TSL2561 data
				gpsData = GPS.readGPS()				#Read GPS data
				Other.saveLog(sleepLog, time.time() - t_start, gpsData, bme280Data, bmx055Data, luxData)
				photoName = Capture.Capture(photoPath)
				time.sleep(1)
				t2=time.time()
			pi.write(22, 1)
			time.sleep(2)
			IM920.Send("P3F")

		# ------------------- landing Fhase ------------------- #
		print("Landing  Judgement Program Start  {0}".format(time.time()))
		Other.saveLog(phaseLog, "4", "Landing Phase Started", time.time() - t_start)
		if(phaseChk <= 4):
			IM920.Send("P4S")
			t1 = time.time()
			t2 = t1
			while(t2-t1 <= t_release):
				bme280Data = BME280.bme280_read()	#Read BME280 data
				bmx055Data = BMX055.bmx055_read()	#Read BMX055 data
				luxData = TSL2561.readLux()			#Read TSL2561 data
				gpsData = GPS.readGPS()				#Read GPS data
				Other.saveLog(sleepLog, time.time() - t_start, gpsData, bme280Data, bmx055Data, luxData)
				photoName = Capture.Capture(photoPath)
				IM920.Send("P4D")
				time.sleep(1)
				t2=time.time()
			IM920.Send("P4F")

		# - Melting-------------- Fhase ------------------- #
		print("Melting Program Start  {0}".format(time.time()))
		Other.saveLog(phaseLog,"5", "Melting Phase Started", time.time() - t_start)
		if(phaseChk <= 5):
			IM920.Send("P5S")
			print("Melting Phase Started")
			Other.saveLog(meltingLog, time.time() - t_start, GPS.readGPS(), "Melting Start")
			Melting.Melting(t_melting)
			Other.saveLog(meltingLog, time.time() - t_start, GPS.readGPS(), "Melting Finished")
			IM920.Send("P5F")

		# -------------------  Running Fhase --- #
		print("Running Program Start  {0}".format(time.time()))
		Other.saveLog(phaseLog, "7", "Running Phase Started", time.time() - t_start)
		t1 = time.time()
		t2 = t1
		if(phaseChk <= 7):
			Motor.motor(30, -30, 1)
			IM920.Send("P7S")
			t1 = time.time()
			t2 = t1
			while(t2-t1 <= t_release):
				bme280Data = BME280.bme280_read()	#Read BME280 data
				bmx055Data = BMX055.bmx055_read()	#Read BMX055 data
				luxData = TSL2561.readLux()			#Read TSL2561 data
				gpsData = GPS.readGPS()				#Read GPS data
				Other.saveLog(sleepLog, time.time() - t_start, gpsData, bme280Data, bmx055Data, luxData)
				photoName = Capture.Capture(photoPath)
				IM920.Send("P7D")
				time.sleep(1)
				t2=time.time()
			IM920.Send("P7F")
			Motor.motor(0, 0, 1)

		# ------------------- Program Finish Fhase ------------------- #
		Other.saveLog(phaseLog, "10", "Program Finished", time.time() - t_start)
		print("Program finished  {0}".format(time.time()))
		Motor.motor(0, 0, 1)
		IM920.Send("P10F")
		close()
	except KeyboardInterrupt:
		Motor.motor_stop()
		close()
		print("\r\nKeyboard Intruppted")
	except:
		IM920.Send("error")
		close()
		print(traceback.format_exc())
		Other.saveLog(errorLog, time.time() - t_start, "Error")
		Other.saveLog(errorLog, traceback.format_exc())
		Other.saveLog(errorLog, "\n")
