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
t_start  = 0.0				#time when program started
t_sleep = 10				#time for sleep phase
t_release = 10				#time for release(loopx)
t_land = 30					#time for land(loopy)
t_melt = 5					#time for melting
t_sleep_start = 0			#time for sleep origin
t_release_start = 0			#time for release origin
t_land_start = 0			#time for land origin
t_calib_origin = 0			#time for calibration origin
t_paraDete_start = 0
t_takePhoto_start = 0		#time for taking photo
timeout_calibration = 180	#time for calibration timeout
timeout_parachute = 60
timeout_takePhoto = 10		#time for taking photo timeout

# --- variable for storing sensor data --- #
gpsData = [0.0,0.0,0.0,0.0,0.0]						#variable to store GPS data
bme280Data = [0.0,0.0,0.0,0.0]						#variable to store BME80 data
bmx055data = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]	#variable to store BMX055 data

# --- variable for Judgement --- #
lcount = 0		#lux count for release
acount = 0		#press count for release
Pcount = 0		#press count for land
GAcount = 0		#GPSheight count for land
luxjudge = 0	#for release
pressjudge = 0	#for release and land
gpsjudge = 0	#for land
paraExsist = 0 	#variable for Para Detection    0:Not Exsist, 1:Exsist
goalFlug = -1	#variable for GoalDetection		-1:Not Detect, 0:Goal, 1:Detect
H_min = 200		#Hue minimam
H_max = 10		#Hue maximam
S_thd = 120		#Saturation threshold

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
kp = 0.8							#Proportional Gain
maxMP = 60							#Maximum Motor Power
mp_min = 10							#motor power for Low level
mp_max = 30							#motor power fot High level
mp_adj = 2							#adjust motor power

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
photoName =			""

pi=pigpio.pi()	#object to set pigpio


def setup():
	global phaseChk
	pi.set_mode(17,pigpio.OUTPUT)
	pi.set_mode(22,pigpio.OUTPUT)
	pi.write(22,1)					#IM920	Turn On
	pi.write(17,0)					#Outcasing Turn Off
	time.sleep(1)
	BME280.bme280_setup()
	BME280.bme280_calib_param()
	BMX055.bmx055_setup()
	GPS.openGPS()

	#if it is End to End Test, then
	phaseChk = int(Other.phaseCheck(phaseLog))

	#if it is debug
	#phaseChk = 7

def close():
	GPS.closeGPS()
	pi.write(22, 1)		#IM920 Turn On
	pi.write(17,0)
	Motor.motor(0, 0, 1)
	Motor.motor_stop()


if __name__ == "__main__":
	try:
		print("Program Start  {0}".format(time.time()))
		t_start = time.time()

		#-----setup phase ---------#
		setup()
		print("Start Phase is {0}".format(phaseChk))
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

			# --- Sleep --- #
			while(time.time() - t_sleep_start <= t_sleep):
				photoName = Capture.Caputure(photopath)
				Other.saveLog(captureLog, time.time() - t_start, photoName)
				Other.saveLog(sleepLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), TSL2561.readLux(), BMX055.bmx055_read())
				time.sleep(1)

		# ------------------- Release Phase ------------------- #
		if(phaseChk <= 3):
			Other.saveLog(phaseLog, "3", "Release Phase Started", time.time() - t_start)
			t_release_start = time.time()
			print("Releasing Phase Started  {0}".format(time.time() - t_start))

			# --- Release Judgement, "while" is for timeout --- #
			while (time.time() - t_release_start <= t_release):
				#luxjudge,lcount = Release.luxjudge()
				pressjudge,acount = Release.pressjudge()

				if luxjudge == 1 or pressjudge == 1:
					Other.saveLog(releaseLog, time.time() - t_start, "Release Judged by Sensor", luxjudge, pressjudge)
					print("Rover has released")
					break
				else:
		   			print("Rover is in rocket")

				# --- Save Log --- #
				Other.saveLog(releaseLog, time.time() - t_start, acount, GPS.readGPS(), TSL2561.readLux(), BME280.bme280_read(), BMX055.bmx055_read())
				#Other.saveLog(releaseLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(0.5)
				Other.saveLog(releaseLog, time.time() - t_start, acount, GPS.readGPS(), TSL2561.readLux(), BME280.bme280_read(), BMX055.bmx055_read())
				#Other.saveLog(releaseLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(0.5)
				
				# --- Take Photo --- #
				photoName = Capture.Caputure(photopath)
				Other.saveLog(captureLog, time.time() - t_start, photoName)
			else:
				Other.saveLog(releaseLog, time.time() - t_start, "Release Judged by Timeout")
				print("Release Timeout")
			pi.write(22, 1)	#Turn on IM920
			time.sleep(1)
			IM920.Send("P3F")

		# ------------------- Landing Phase ------------------- #
		if(phaseChk <= 4):
			Other.saveLog(phaseLog, "4", "Landing Phase Started", time.time() - t_start)
			print("Landing Phase Started  {0}".format(time.time() - t_start))
			IM920.Send("P4S")
			t_land_start = time.time()

			# --- Landing Judgement, "while" is for timeout --- #
			while(time.time() - t_land_start <= t_land):
				pressjudge, Pcount = Land.pressjudge()
				gpsjudge, gacount = Land.gpsjusdge()

				if pressjudge == 1 and gpsjudge == 1:
					Other.saveLog(releaseLog, time.time() - t_start, "Landing Judged by Sensor", pressjudge, gpsjudge)
					print("Rover has Landed")
					break
				elif pressjudge == 0 and gpsjudge == 0:
				    print("Descend now taking photo")
				elif pressjudge == 1 or gpsjudge == 1:
				    print("Landing Judgement Now")
				
				# --- Save Log --- #
				Other.saveLog(landingLog ,time.time() - t_start, Pcount, gacount, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				Other.saveLog(landingLog ,time.time() - t_start, Pcount, gacount, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				Other.saveLog(landingLog ,time.time() - t_start, Pcount, gacount, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)

				# --- Take Photo --- #
				photoName = Capture.Caputure(photopath)
				Other.saveLog(captureLog, time.time() - t_start, photoName)
			else:
				Other.saveLog(landingLog, time.time() - t_start, "Landing Judged by Timeout")
				print("Landing Timeout")
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
			t_paraDete_start = time.time()
			while time.time() - t_paraDete_start < timeout_parachute:
				paraLuxflug, paraLux = ParaDetection.ParaJudge(70)
				if paraLuxflug == 1:
					break

			print("START: Parachute avoidance")
			for i in range(2):	#Avoid Parachute two times
				Motor.motor(30, 30, 0.5)
				Motor.motor(0, 0, 0.2)
				paraExsist, paraArea, photoName = ParaDetection.ParaDetection(photopath, 200, 10, 120)

				if paraExsist == 1:
					Motor.motor(-60, -60, 5)
					Motor.motor(0, 0, 2)

				if paraExsist == 0:
					Motor.motor(60, 60, 5)
					Motor.motor(0 ,0, 2)

				Other.saveLog(captureLog, time.time() - t_start, photoName)
				Other.saveLog(paraAvoidanceLog, time.time() - t_start, GPS.readGPS(), paraLuxflug, paraLux, photoName, paraExsist, paraArea)
			Other.saveLog(paraAvoidanceLog, time.time() - t_start, GPS.readGPS(), "ParaAvoidance Finished")
			IM920.Send("P6F")

		# ------------------- Running Phase ------------------- #
		if(phaseChk <= 7):
			Other.saveLog(phaseLog, "7", "Running Phase Started", time.time() - t_start)
			print("Running Phase Started")
			IM920.Send("P7S")

			# --- Calibration --- #
			fileCal = Other.fileName(calibrationLog, "txt")
			Motor.motor(60, 0, 2)
			Calibration.readCalData(fileCal)
			Motor.motor(0, 0, 1)
			ellipseScale = Calibration.Calibration(fileCal)
			Other.saveLog(fileCal, ellipseScale)

			while(not RunningGPS.checkGPSstatus(gpsData)):
				gpsData = GPS.readGPS()
				time.sleep(1)

			t_calib_origin = time.time()
			t_takePhoto_start = time.time()
			while(disGoal >= 5):
				# --- Get GPS Data --- #
				if(RunningGPS.checkGPSstatus(gpsData)):
					nLat = gpsData[1]
					nLon = gpsData[2]

				# --- Calibration --- #
				#Every [timeout_calibratoin] second,  Calibrate
				if(time.time() - t_calib_origin > timeout_calibration):
					Motor.motor(0, 0, 2)
					print("Calibration")
					fileCal = Other.fileName(calibrationLog, "txt")
					Motor.motor(50, 0, 2)
					Calibration.readCalData(fileCal)
					Motor.motor(0, 0, 1)
					ellipseScale = Calibration.Calibration(fileCal)
					Other.saveLog(fileCal, ellipseScale)
					t_calib_origin = time.time()

				# --- Taking Photo --- #
				if(time.time() - t_takePhoto_start > timeout_takePhoto):
					Motor.motor(0, 0, 1)
					photoName = Capture.Caputure(photopath)
					Other.saveLog(captureLog, time.time() - t_start, photoName)
					t_takePhoto_start = time.time()

				#Calculate angle
				nAng = RunningGPS.calNAng(ellipseScale, angOffset)

				#Calculate disGoal and relAng
				relAng[2] = relAng[1]
				relAng[1] = relAng[0]
				disGoal, angGoal, relAng[0] = RunningGPS.calGoal(nLat, nLon, gLat, gLon, nAng)
				rAng = np.median(relAng)

				#Calculate Motor Power
				mPL, mPR, mPS = RunningGPS.runMotorSpeed(rAng, kp, maxMP)

				# --- Save Log --- #
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
			goal = Goal.Togoal(photopath, H_min, H_max, S_thd, mp_min, mp_max, mp_adj)	
			while goal[0] != 0:
				gpsdata = GPS.readGPS()
				goalFlug, goalArea, goalGAP, photoName = Goal.Togoal(photopath, H_min, H_max, S_thd, mp_min, mp_max, mp_adj)
				print("goal is",goal)
				Other.saveLog(goalDetectionLog, time.time() - t_start, gpsData, goalFlug, goalArea, goalGAP, photoName)
				Other.saveLog(captureLog, time.time() - t_start, photoName)
			print("Goal Detection Phase Finished")
			IM920.Send("P8F")

		print("Program Finished")
		IM920.Send("P10")
		Other.saveLog(phaseLog, "10", "Program Finished", time.time() - t_start)
		close()
	except KeyboardInterrupt:
		close()
		print("Keyboard Interrupt")
		IM920.Send("KI")
	except:
		close()
		print(traceback.format_exc())
		Other.saveLog(errorLog, time.time() - t_start, "Error")
		Other.saveLog(errorLog, traceback.format_exc())
		Other.saveLog(errorLog, "\n")
		IM920.Send("EO")
