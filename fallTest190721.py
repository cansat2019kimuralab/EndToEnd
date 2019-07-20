# -*- coding: utf-8 -*-
import sys
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/GPS')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/IM920')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BME280')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BMX055')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Camera')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/TSL2561')
sys.path.append('/home/pi/git/kimuralab/Detection/ParachuteDetection')
sys.path.append('/home/pi/git/kimuralab/Detection/ReleaseAndLandingDetection')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Melting')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Motor')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/ParaAvoidance')
sys.path.append('/home/pi/git/kimuralab/Other')
import time
import difflib
import pigpio
import serial
import binascii
import IM920
import GPS
import BMX055
import BME280
import Capture
import TSL2561
import Release
import Land
import GPS
import Melting
import Motor
import TSL2561
import ParaDetection
import ParaAvoidance
import Other

phaseChk = 0	#variable for phase Check
luxstr = ["lux1", "lux2"]																#variable to show lux returned variables
bme280str = ["temp", "pres", "hum", "alt"]												#variable to show bme280 returned variables
bmx055str = ["accx", "accy", "accz", "gyrx", "gyry", "gyrz", "dirx", "diry", "dirz"]	#variable to show bmx055 returned variables
gpsstr = ["utctime", "lat", "lon", "sHeight", "gHeight"]								#variable to show GPS returned variables

gpsData=[0.0,0.0,0.0,0.0,0.0]
bme280Data=[0.0,0.0,0.0,0.0]
bmx055data=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

t_setup = 1	#variable to set waiting time after setup
t = 1	#waitingtime
x = 60	#time for release(loopx)
y = 60	#time for land(loopy)

t_start  = 0.0	#time when program started

#lcount=0
acount=0
Pcount=0
GAcount=0
deltHmax=5
luxjudge = 0
pressjudge=0
pi=pigpio.pi()

paraExsist = 0 	#variable used for Para Detection

phaseLog = "/home/pi/log/phaseLog.txt"
waitingLog = "/home/pi/log/waitingLog.txt"
releaseLog = "/home/pi/log/releaseLog.txt"
landingLog = "/home/pi/log/landingLog.txt"
meltingLog = "/home/pi/log/meltingLog.txt"
paraAvoidanceLog = "/home/pi/log/paraAvoidanceLog.txt"


def setup():
	global phaseChk

	pi.set_mode(22,pigpio.OUTPUT)
	pi.write(22,1)	#IM920
	pi.write(17,0)	#outcasing
	time.sleep(1)
	BME280.bme280_setup()
	BME280.bme280_calib_param()
	BMX055.bmx055_setup()
	GPS.openGPS()

	with open(phaseLog, 'a') as f:
		pass

	phaseChk = int(Other.phaseCheck(phaseLog))
	print(phaseChk)

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
		print("Program Start  {0}".format(time.time()))
		setup()
		print(phaseChk)

		# ------------------- Waiting Phase --------------------- #
		Other.saveLog(phaseLog, "2", "Waiting Phase Started", time.time() - t_start)
		if(phaseChk <= 2):
			pi.write(22, 0)
			time.sleep(t_setup)

		# ------------------- Release Phase ------------------- #
		Other.saveLog(phaseLog, "3", "Release Phase Started", time.time() - t_start)
		if(phaseChk <= 3):
			tx1 = time.time()
			tx2 = tx1
			print("Releasing Judgement Program Start  {0}".format(time.time() - t_start))
			#loopx
			bme280Data=BME280.bme280_read()
			while (tx2-tx1<=x):
				luxjudge=Release.luxjudge()
				pressjudge=Release.pressjudge()

				if luxjudge==1 or pressjudge==1:
					break
				else:
		   			print("now in rocket ,taking photo")
				Other.saveLog(releaseLog, GPS.readGPS(), BME280.bme280_read(), TSL2561.readLux(), BMX055.bmx055_read())
				time.sleep(1)
				Other.saveLog(releaseLog, GPS.readGPS(), BME280.bme280_read(), TSL2561.readLux(), BMX055.bmx055_read())
				time.sleep(1)
				tx2=time.time()
			else:
				print("RELEASE TIMEOUT")
			print("THE ROVER HAS RELEASED")
			pi.write(22,1)
			time.sleep(2)
			IM920.Send("RELEASE")

		# ------------------- Landing Phase ------------------- #
		Other.saveLog(phaseLog, "4", "Landing Phase Started", time.time() - t_start)
		if(phaseChk <= 4):
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
				Other.saveLog(landingLog ,GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				Other.saveLog(landingLog ,GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				Other.saveLog(landingLog ,GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				ty2=time.time()
			else:
				print("LAND TIMEOUT")
			print("THE ROVER HAS LANDED")
			pi.write(22,1)
			IM920.Send("LAND")

		# ------------------- Melting Phase ------------------- #
		IM920.Send("Melt")
		Other.saveLog(phaseLog,"5", "Melting Phase Started", time.time() - t_start)
		if(phaseChk <= 5):
			print("Melting Phase Started")
			Other.saveLog(meltingLog, GPS.readGPS(), "Melting Start", time.time() - t_start)
			Melting.Melting()
			Other.saveLog(meltingLog, GPS.readGPS(), "Melting Finished", time.time() - t_start)

		# ------------------- ParaAvoidance Phase ------------------- #
		IM920.Send("ParaAvo")
		Other.saveLog(phaseLog, "6", "ParaAvoidance Phase Started", time.time() - t_start)
		if(phaseChk <= 6):
			print("ParaAvoidance Phase Started")
			Other.saveLog(paraAvoidanceLog, GPS.readGPS(), "ParaAvoidance Start", time.time() - t_start)
			print("START: Judge covered by Parachute")
			ParaAvoidance.ParaJudge()
			print("START: Parachute avoidance")
			paraExsist = ParaAvoidance.ParaAvoidance()
			Other.saveLog(paraAvoidanceLog, GPS.readGPS(), paraExsist, time.time() - t_start)
			Other.saveLog(paraAvoidanceLog, GPS.readGPS(), "ParaAvoidance Fineshed", time.time() - t_start)

		IM920.Send("Progam Finished")
		close()
	except KeyboardInterrupt:
		close()
		print("Keyboard Interrupt")
	except Exception as e:
		IM920.Send("error")
		Other.saveLog("/home/pi/log/errorLog.txt", "Error")
		close()
		print(e.message)
