# -*- coding: utf-8 -*-
import sys
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Other')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BME280')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BMX055')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Camera')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/GPS')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/IM920')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Melting')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Motor')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/TSL2561')
import binascii
import difflib
import pigpio
import serialimport time
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

luxstr = ["lux1", "lux2"]																#variable to show lux returned variable
bme280str = ["temp", "pres", "hum", "alt"]												#variable to show bme280 returned variable
bmx055str = ["accx", "accy", "accz", "gyrx", "gyry", "gyrz", "dirx", "diry", "dirz"]	#variable to show bmx055 returned variable
gpsstr = ["utctime", "lat", "lon", "sHeight", "gHeight"]								#variable

t = 10	#variable to set waiting time
x = 10	#variable to set releasing time
y = 10	#variable to set landing time
z = 10	#variable to set running time

t1 = 0.0	#vairable to store start time
t2 = 0.0	#vairable to store time

t_sleep = 3		#sleep time
t_release = 5	#release time
t_landing = 5	#landing time
t_melting = 5	#melting time
t_running = 20	#running time

phaseLog = "phaseLo.txt"
sleepLog = "sleepLog.txt"
releaseLog = "releaseLog.txt"
landingLog = "landingLog.txt"
meltingLog = "meltingLog.txt"
runningLog = "runningLog.txt"
errorLog = "errorLog.txt"

count = 0

pi = pigpio.pi()	#object to set pigpio

def gpsSend(gpsData):
	sendData = 'g'
	for i in range(len(gpsData)):
		sendData += str(gpsData[i]) + ','

	IM920.Send(sendData)

def setup():
	pi.set_mode(22,pigpio.OUTPUT)
	pi.write(22,0)
	pi.write(17,0)
	time.sleep(1)
	BME280.bme280_setup()
	BME280.bme280_calib_param()
	BMX055.bmx055_setup()
	GPS.openGPS()

def close():
	GPS.closeGPS()
	Motor.motor(0, 0, 2)
	Motor.motor_stop()
	pi.write(17, 0)

if __name__ == "__main__":
	try:
		# ------------------- Setup Fhase ------------------- #
		print("Program Start  {0}".format(time.time()))
		setup()

		time.sleep(x)
		
		# ------------------- Release Fhase ------------------- #
		print("Release Judgement Program Start  {0}".format(time.time()))
		t1 = time.time()
		t2 = t1
		count = 0
		while(t2-t1 <= t_release):
			bme280Data = BME280.bme280_read()	#Read BME280 data
			luxData = TSL2561.readLux()
			with open('log/releaseLog.txt', 'a') as f:	#Write Log
				f.write(str(count) + "\t")
				for i in range(len(bme280Data)):
					f.write(str(bme280str[i]) + ":" + str(bme280Data[i]) + "\t")
				for i in range(len(luxData)):
					f.write(str(luxstr[i]) + ":" + str(luxData[i]) + "\t")
				f.write("\n")

			Capture.Capture(count)
			count += 1
			t2=time.time()

		# ------------------- IM920 Setup  Fhase ------------------- #
		pi.write(22,1)	#IM920 Setup, set GPIO22 HIGH
		time.sleep(3)

		# ------------------- landing Fhase ------------------- #
		print("Landing Judgement Program Start  {0}".format(time.time()))
		t1 = time.time()
		t2 = t1
		while(t2 - t1 <= t_landing):
			bme280Data = BME280.bme280_read()
			gpsData = GPS.readGPS()
			with open('log/landingLog.txt','a') as f:
				f.write(str(count) + "\t")
				if(gpsData[0] == -1):
					if(gpsData[1] == -1):
						f.write("Reading GPS Error ")
						#IM920.Send("E")
					else:
						f.write("Status V ")
						#IM920.Send("V")
				else:
					for i in range(len(gpsData)):
						f.write(str(gpsstr[i]) + ":" + str(gpsData[i]) + "\t")

				for i in range(len(bme280Data)):
					f.write(str(bme280str[i]) + ":" + str(bme280Data[i]) + "\t")
				f.write("\n")

				gpsSend(gpsData)
				IM920.Send('a'+str(bme280Data[3]))

			Capture.Capture(count)
			count += 1
			time.sleep(1)
			t2 = time.time()

		# ------------------- Melting Fhase ------------------- #
		print("Melting Program Start  {0}".format(time.time()))
		Melting.Melting()

		# ------------------- Running Fhase ------------------- #
		print("Running Program Start  {0}".format(time.time()))
		t1 = time.time()
		t2 = t1
		Motor.motor(30, 30, 1)
		while(t2 - t1 <= t_running):
			gpsData = GPS.readGPS()
			bmx055Data = BMX055.bmx055_read()
			with open('log/runningLog.txt', 'a') as f:
				f.write(str(count) + "\t")
				for i in range(len(gpsData)):
					f.write(str(gpsstr[i]) + ":" + str(gpsData[i]) + "\t")
				for i in range(len(bmx055Data)):
					f.write(str(bmx055str[i]) + ":" + str(bmx055Data[i]) + "\t")
				f.write("\n")

			gpsSend(gpsData)
			Capture.Capture(count)	#Take and store photo
			count += 1
			time.sleep(1)
			t2 = time.time()

		Motor.motor(0, 0, 1)
		print("Program finished  {0}".format(time.time()))
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
