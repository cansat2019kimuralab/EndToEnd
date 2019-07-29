# -*- coding: utf-8 -*-
import sys
sys.path.append('/home/pi/git/kimuralab/Detection/ParachuteDetection')
sys.path.append('/home/pi/git/kimuralab/Detection/ReleaseAndLandingDetection')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Calibration')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Goal')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/ParaAvoidance')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Running')
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
import numpy as np
import pigpio
import serial
import time
import traceback

import BMX055
import BME280
import Capture
import Calibration
import Goal
import GPS
import IM920
import Land
import Melting
import Motor
import Other
import ParaDetection
import ParaAvoidance
import Release
import RunningGPS
import TSL2561

phaseChk = 0	#variable for phase Check

# --- variable of time setting --- #
t_start  = 0.0	#time when program started
t_sleep = 10	#time for sleep phase
t_release = 10	#time for release(loopx)
t_land = 30		#time for land(loopy)
t_melt = 5		#time for melting
t_sleep_start = 0
t_release_start = 0
t_land_start = 0

# --- variable for storing sensor data --- #
gpsData=[0.0,0.0,0.0,0.0,0.0]                       #variable to store GPS data
bme280Data=[0.0,0.0,0.0,0.0]                        #variable to store BME80 data
bmx055data=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]    #variable to store BMX055 data

# --- variable for Judgement --- #
lcount = 0		#lux count for release
acount=0		#press count for release
Pcount=0		#press count for land
GAcount=0		#GPSheight count for land
luxjudge = 0	#for release
pressjudge=0	#for release and land
gpsjudge=0		#for land
paraExsist = 0 	#variable for Para Detection    0:Not Exsist, 1:Exsist

# --- variable for Running --- #
fileCal = "" 						#file path for Calibration
ellipseScale = [0.0, 0.0, 0.0, 0.0] #Convert coefficient Ellipse to Circle
disGoal = 100.0						#Distance from Goal [m]
angGoal = 0.0						#Angle toword Goal [deg]
angOffset = -77.0					#Angle Offset towrd North [deg]
gLat, gLon = 35.918181, 139.907992	#Coordinates of That time
nLat, nLon = 0.0, 0.0		  		#Coordinates of That time
nAng = 0.0							#Direction of That time [deg]
relAng = [0.0, 0.0, 0.0]			#Relative Direction between Goal and Rober That time [deg]
rAng = 0.0							#Median of relAng [deg]
mP, mPL, mPR, mPS = 0, 0, 0, 0		#Motor Power
gpsInterval = 0						#GPS Log Interval Time

# --- variable of Log path --- #
phaseLog =			"/home/pi/log/phaseLog.txt"
sleepLog = 			"/home/pi/log/sleepLog.txt"
releaseLog = 		"/home/pi/log/releaseLog.txt"
landingLog = 		"/home/pi/log/landingLog.txt"
meltingLog = 		"/home/pi/log/meltingLog.txt"
paraAvoidanceLog = 	"/home/pi/log/paraAvoidanceLog.txt"
runningLog = 		"/home/pi/log/runningLog.txt"
goalDetectionLog =	"/home/pi/log/goalDetectionLog.txt"
captureLog = 		"/home/pi/log/captureLog.txt"
calibrationLog = 	"/home/pi/log/calibrationLog"
errorLog = 			"/home/pi/log/erroLog.txt"

photopath = 		"/home/pi/photo/photo"

pi=pigpio.pi()	#object to set pigpio


def setup():
	global phaseChk

	pi.set_mode(17,pigpio.OUTPUT)
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
		#-----setup phase ---------#
		setup()
		print("Program Start  {0}".format(time.time()))
		print(phaseChk)
		if(phaseChk <= 1):
			IM920.Send("P1S")
			Other.saveLog(phaseLog, "1", "Program Started", time.time() - t_start)
			IM920.Send("P1F")

		# ------------------- Sleep Phase --------------------- #
		if(phaseChk <= 2):
			Other.saveLog(phaseLog, "2", "Sleep Phase Started", time.time() - t_start)
			print("Sleep Phase Started  {0}".format(time.time() - t_start))
			IM920.Send("P2S")
			pi.write(22, 0)		#IM920 Turn Off
			t_sleep_start = time.time()
			while(time.time() - t_sleep_start <= t_sleep):
				Other.saveLog(sleepLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), TSL2561.readLux(), BMX055.bmx055_read())
				#print("Sleep")
				time.sleep(1)

		# ------------------- Release Phase ------------------- #
		if(phaseChk <= 3):
			Other.saveLog(phaseLog, "3", "Release Phase Started", time.time() - t_start)
			t_release_start = time.time()
			print("Releasing Phase Started  {0}".format(time.time() - t_start))
			#loopx
			bme280Data=BME280.bme280_read()
			while (time.time() - t_release_start <= t_release):
				#luxjudge,lcount = Release.luxjudge()
				pressjudge,acount = Release.pressjudge()

				if luxjudge==1 or pressjudge==1:
					break
				else:
					#pass
		   			print("now in rocket ,taking photo")
				Other.saveLog(releaseLog, time.time() - t_start, acount, GPS.readGPS(), TSL2561.readLux(), BME280.bme280_read(), BMX055.bmx055_read())
				#Other.saveLog(releaseLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(0.5)

				Other.saveLog(releaseLog, time.time() - t_start, acount, GPS.readGPS(), TSL2561.readLux(), BME280.bme280_read(), BMX055.bmx055_read())
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
		if(phaseChk <= 4):
			Other.saveLog(phaseLog, "4", "Landing Phase Started", time.time() - t_start)
			print("Landing Phase Started  {0}".format(time.time() - t_start))
			IM920.Send("P4S")
			t_land_start = time.time()
			gpsData = GPS.readGPS()
			bme280Data=BME280.bme280_read()
			while(time.time() - t_land_start <= t_land):
				pressjudge, Pcount = Land.pressjudge()
				gpsjudge, gacount = Land.gpsjusdge()
				if pressjudge ==1 and gpsjudge ==1:
					break
				elif pressjudge == 0 and gpsjudge == 0:
				    print("Descend now taking photo")
				elif pressjudge == 1 or gpsjudge == 1:
				    print("landjudgementnow")
				gpsData = GPS.readGPS()
				bme280Data=BME280.bme280_read()
				bmx055data=BMX055.bmx055_read()
				Other.saveLog(landingLog ,time.time() - t_start, Pcount, gacount, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				Other.saveLog(landingLog ,time.time() - t_start, Pcount, gacount, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				Other.saveLog(landingLog ,time.time() - t_start, Pcount, gacount, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				ty2=time.time()
			else:
				print("LAND TIMEOUT")
			print("THE ROVER HAS LANDED")
			IM920.Send("P4F")

		# ------------------- Melting Phase ------------------- #
		if(phaseChk <= 5):
			Other.saveLog(phaseLog,"5", "Melting Phase Started", time.time() - t_start)
			print("Melting Phase Started")
			IM920.Send("P5S")
			Other.saveLog(meltingLog, time.time() - t_start, GPS.readGPS(), "Melting Start")
			Melting.Melting(t_melt)
			Other.saveLog(meltingLog, time.time() - t_start, GPS.readGPS(), "Melting Finished")
			IM920.Send("P5F")

		# ------------------- ParaAvoidance Phase ------------------- #
		if(phaseChk <= 6):
			Other.saveLog(phaseLog, "6", "ParaAvoidance Phase Started", time.time() - t_start)
			IM920.Send("P6S")
			print("ParaAvoidance Phase Started")
			Other.saveLog(paraAvoidanceLog, time.time() - t_start, GPS.readGPS(), "ParaAvoidance Start")
			print("START: Judge covered by Parachute")
			paraExisis =  ParaAvoidance.ParaJudge(70)
			print("START: Parachute avoidance")
			for i in range(2):	#Avoid Parachute two times
				paraExsist, photoName = ParaAvoidance.ParaAvoidance(photopath)
				Other.saveLog(captureLog, time.time() - t_start, photoName)
				Other.saveLog(paraAvoidanceLog, time.time() - t_start, GPS.readGPS(), paraExsist, photoName)
			Other.saveLog(paraAvoidanceLog, time.time() - t_start, GPS.readGPS(), "ParaAvoidance Finished")
			IM920.Send("P6F")

        # ------------------- Running Phase ------------------- #
		if(phaseChk <= 7):
			Other.saveLog(phaseLog, "7", "Running Phase Started", time.time() - t_start)
			print("Running Phase Started")
			IM920.Send("P7S")

			fileCal = Other.fileName(calibrationLog, "txt")

			Motor.motor(60, 0, 2)
			Calibration.readCalData(fileCal)
			Motor.motor(0, 0, 1)
			ellipseScale = Calibration.Calibration(fileCal)
			Other.saveLog(fileCal, ellipseScale)

			gpsInterval = 0

			#Get GPS data
			#print("Getting GPS Data")
			while(not RunningGPS.checkGPSstatus(gpsData)):
				gpsData = GPS.readGPS()
				time.sleep(1)

			while(disGoal >= 5):
				if(RunningGPS.checkGPSstatus(gpsData)):
					nLat = gpsData[1]
					nLon = gpsData[2]

				#Calculate angle
				nAng = RunningGPS.calNAng(ellipseScale, angOffset)

				#Calculate disGoal and relAng
				relAng[2] = relAng[1]
				relAng[1] = relAng[0]
				disGoal, angGoal, relAng[0] = RunningGPS.calGoal(nLat, nLon, gLat, gLon, nAng)
				rAng = np.median(relAng)

				#Calculate Motor Power
				mPL, mPR, mPS = RunningGPS.runMotorSpeed(rAng)

				#Save Log
				print(nLat, nLon, disGoal, angGoal, nAng, rAng, mPL, mPR, mPS)
				Other.saveLog(runningLog, time.time() - t_start, BMX055.bmx055_read(), nLat, nLon, disGoal, angGoal, nAng, rAng, mPL, mPR, mPS)
				gpsData = GPS.readGPS()
				Motor.motor(mPL, mPR, 0.1, 1)
			Motor.motor(0, 0, 1)
			print("Running Phase Finished")
			IM920.Send("P7F")

        # ------------------- GoalDetection Phase ------------------- #
		if(phaseChk <= 8):
			Other.saveLog(phaseLog, "8", "GoalDetection Phase Started", time.time() - t_start)
			print("Goal Detection Phase Started")
			IM920.Send("P8S")
			H_min = 200	#グローバル変数で宣言
			H_max = 10	#グローバル変数で宣言
			S_thd = 120 #グローバル変数で宣言
			goal = Goal.Togoal(photopath, H_min, H_max, S_thd)	#グローバル変数で宣言
			while goal[0] != 0:
				gpsdata = GPS.readGPS()
				goal = Goal.Togoal(photopath, H_min, H_max, S_thd)
				print("goal is",goal)
				Other.saveLog(goalDetectionLog, time.time() - t_start, gpsData, goal)
				Other.saveLog(captureLog, time.time() - t_start, goal)
			print("Goal Detection Phase Finished")
			IM920.Send("P8F")

		print("Program Finished")
		IM920.Send("P10")
		Other.saveLog(phaseLog, "10", "Program Finished", time.time() - t_start)
		close()
	except KeyboardInterrupt:
		close()
		print("Keyboard Interrupt")
	except:
		IM920.Send("EO")
		close()
		print(traceback.format_exc())
		Other.saveLog(errorLog, time.time() - t_start, "Error")
		Other.saveLog(errorLog, traceback.format_exc())
		Other.saveLog(errorLog, "\n")
