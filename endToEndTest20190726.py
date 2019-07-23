# -*- coding: utf-8 -*-
import sys
sys.path.append('/home/pi/git/kimuralab/Detection/ParachuteDetection')
sys.path.append('/home/pi/git/kimuralab/Detection/ReleaseAndLandingDetection')
sys.path.append('/home/pi/git/kimuralab/Detection/ParachuteDetection')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/ParaAvoidance')
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

import BMX055
import BME280
import Capture
import GPS
import IM920
import Land
import Melting
import Motor
import Other
import ParaDetection
import ParaAvoidance
import Release
import TSL2561

phaseChk = 0	#variable for phase Check

# --- variable of time setting --- #
t_start  = 0.0	#time when program started
t_sleep = 60	#time for sleep phase
t_melt = 5		#time for melting
x = 600			#time for release(loopx)
y = 180			#time for land(loopy)

# --- variable for storing sensor data --- #
gpsData=[0.0,0.0,0.0,0.0,0.0]                       #variable to store GPS data
bme280Data=[0.0,0.0,0.0,0.0]                        #variable to store BME80 data
bmx055data=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]    #variable to store BMX055 data

acount=0
Pcount=0
GAcount=0
deltHmax=5
luxjudge = 0
pressjudge=0

paraExsist = 0 	#variable for Para Detection    0:Not Exsist, 1:Exsist

# --- variable of Log path --- #
phaseLog = "/home/pi/log/phaseLog.txt"
sleepLog = "/home/pi/log/sleepLog.txt"
releaseLog = "/home/pi/log/releaseLog.txt"
landingLog = "/home/pi/log/landingLog.txt"
meltingLog = "/home/pi/log/meltingLog.txt"
paraAvoidanceLog = "/home/pi/log/paraAvoidanceLog.txt"
runningLog = "/home/pi/log/runningLog.txt"
goalDetectionLog = "/home/pi/log/goalDetectionLog.txt"
errorLog = "/home/pi/log/erroLog.txt"

pi=pigpio.pi()


def setup():
	global phaseChk

	pi.set_mode(22,pigpio.OUTPUT)
	pi.write(22,1)	#IM920	Turn On
	pi.write(17,0)	#outcasing
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
	pi.write(22, 0)
	pi.write(17,0)
	Motor.motor(0, 0, 1)
	Motor.motor_stop()


if __name__ == "__main__":
	try:
		t_start = time.time()
		# ------------------- Setup Phase --------------------- #
		IM920.Send("P1S")
		print("Program Start  {0}".format(time.time()))
		setup()
		print(phaseChk)
		IM920.Send("P1F")

		# ------------------- Sleep Phase --------------------- #
		Other.saveLog(phaseLog, "2", "Sleep Phase Started", time.time() - t_start)
		if(phaseChk <= 2):
			IM920.Send("P2S")
			pi.write(22, 0)		#IM920 Turn Off
			t_wait_start = time.time()
			while(time.time() - t_wait_start <= t_sleep):
				Other.saveLog(sleepLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), TSL2561.readLux(), BMX055.bmx055_read())
				print("Sleep")
				time.sleep(1)

		# ------------------- Release Phase ------------------- #
		Other.saveLog(phaseLog, "3", "Release Phase Started", time.time() - t_start)
		if(phaseChk <= 3):
			tx1 = time.time()
			tx2 = tx1
			print("Releasing Judgement Program Start  {0}".format(time.time() - t_start))
			#loopx
			bme280Data=BME280.bme280_read()
			while (tx2-tx1<=x):
				luxjudge = Release.luxjudge()
				pressjudge = Release.pressjudge()

				if luxjudge==1 or pressjudge==1:
					break
				else:
					#pass
		   			print("now in rocket ,taking photo")
				Other.saveLog(releaseLog, time.time() - t_start, TSL2561.readLux(), BME280.bme280_read(), BMX055.bmx055_read())
				#Other.saveLog(releaseLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(0.5)

				Other.saveLog(releaseLog, time.time() - t_start, TSL2561.readLux(), BME280.bme280_read(), BMX055.bmx055_read())
				#Other.saveLog(releaseLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(0.5)
				tx2=time.time()
			else:
				print("RELEASE TIMEOUT")
			print("THE ROVER HAS RELEASED")
			pi.write(22,1)
			time.sleep(2)
			IM920.Send("P3F")

		# ------------------- Landing Phase ------------------- #
		Other.saveLog(phaseLog, "4", "Landing Phase Started", time.time() - t_start)
		if(phaseChk <= 4):
			IM920.Send("P4S")
			print("Landing Judgement Program Start  {0}".format(time.time() - t_start))
			ty1=time.time()
			ty2=ty1
			#loopy
			gpsData = GPS.readGPS()
			bme280Data=BME280.bme280_read()
			while(ty2-ty1<=y):
				IM920.Send("loopY")
				pressjudge=Land.pressjudge()
			#	gpsjudge=Land.gpsjudge()
				if pressjudge ==1 :#and gpsjudge ==1:
					break
				elif pressjudge==0 :#and gpsjudge==0:
				    print("Descend now taking photo")
			#	elif pressjudge==1 or gpsjudge==1:
			#	    print("landjudgementnow")
				gpsData = GPS.readGPS()
				bme280Data=BME280.bme280_read()
				bmx055data=BMX055.bmx055_read()
				Other.saveLog(landingLog ,time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				Other.saveLog(landingLog ,time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				Other.saveLog(landingLog ,time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				ty2=time.time()
			else:
				print("LAND TIMEOUT")
			print("THE ROVER HAS LANDED")
			pi.write(22,1)
			IM920.Send("P4F")

		# ------------------- Melting Phase ------------------- #
		Other.saveLog(phaseLog,"5", "Melting Phase Started", time.time() - t_start)
		if(phaseChk <= 5):
			IM920.Send("P5S")
			print("Melting Phase Started")
			Other.saveLog(meltingLog, time.time() - t_start, GPS.readGPS(), "Melting Start")
			Melting.Melting(t_melt)
			Other.saveLog(meltingLog, time.time() - t_start, GPS.readGPS(), "Melting Finished")
			IM920.Send
		# ------------------- ParaAvoidance Phase ------------------- #
		Other.saveLog(phaseLog, "6", "ParaAvoidance Phase Started", time.time() - t_start)
		if(phaseChk <= 6):
			IM920.Send("P6S")
			print("ParaAvoidance Phase Started")
			Other.saveLog(paraAvoidanceLog, time.time() - t_start, GPS.readGPS(), "ParaAvoidance Start")
			print("START: Judge covered by Parachute")
			ParaAvoidance.ParaJudge()
			print("START: Parachute avoidance")
			paraExsist = ParaAvoidance.ParaAvoidance()
			Other.saveLog(paraAvoidanceLog, time.time() - t_start, GPS.readGPS(), paraExsist)
			Other.saveLog(paraAvoidanceLog, time.time() - t_start, GPS.readGPS(), "ParaAvoidance Finished")
			IM920.Send("P6F")

        # ------------------- Running Phase ------------------- #
		Other.saveLog(phaseLog, "7", "Running Phase Started", time.time() - t_start)
		if(phaseChk <= 7):
			IM920.Send("P7S")
			IM920.Send("P7F")

        # ------------------- GoalDetection Phase ------------------- #
		Other.saveLog(phaseLog, "8", "GoalDetection Phase Started", time.time() - t_start)
		if(phaseChk <= 8):
			IM920.Send("P8S")
			IM920.Send("P8F")

		IM920.Send("P10")
		close()
	except KeyboardInterrupt:
		close()
		print("Keyboard Interrupt")
	except:
		IM920.Send("error")
		close()
    	print(traceback.format_exc())
		Other.saveLog(errorLog, time.time() - t_start, "Error")
		Other.saveLog(errorLog, traceback.format_exc())
		Other.saveLog(errorLog. "\n")