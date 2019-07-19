	# -*- coding: utf-8 -*-
import sys
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/GPS')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/IM920')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BME280')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BMX055')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Camera')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/TSL2561')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Melting’)
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Motor’)
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/ParaAvoidance’)

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
import ParaAvoidance
		
luxstr = ["lux1", "lux2"]																#variable to show lux returned variables
bme280str = ["temp", "pres", "hum", "alt"]												#variable to show bme280 returned variables
bmx055str = ["accx", "accy", "accz", "gyrx", "gyry", "gyrz", "dirx", "diry", "dirz"]	#variable to show bmx055 returned variables
gpsstr = ["utctime", "lat", "lon", "sHeight", "gHeight"]								#variable to show GPS returned variables
PRESS=[0.0,0.0]
t_setup = 10	#variable to set waiting time after setup

t = 1	#waitingtime
x = 120	#time for release(loopx)
y = 60	#time for land(loopy)
#lcount=0
acount=0
Pcount=0
GAcount=0
luxmax=300
deltHmax=5
pi=pigpio.pi()
def gpsSend(gpsData):
	IM920.Send('g'+str(gpsData[0]) + ',' + str(gpsData[1]) + ',' + str(gpsData[2]) + ',' + str(gpsData[3]) + ',' + str(gpsData[4]) + ',' + str(gpsData[5]))
def setup():
	pi.set_mode(22,pigpio.OUTPUT)
	pi.write(22,0)#IM920
	pi.write(17,0)#outcasing
	time.sleep(1)
	BME280.bme280_setup()
	BME280.bme280_calib_param()
#	BMX055.bmx055_setup()
	GPS.openGPS()
	with open('log/releaseLog.txt', 'a') as f:
		pass
	with open('log/landingLog.txt', 'a') as f2:
		pass
	with open('log/runningLog.txt', 'a') as f3:
		pass

def close():
	GPS.closeGPS()
	#it may be necessarry to down low status some gpio pins 

if __name__ == "__main__":
	try:
		# ------------------- Setup Fhase ------------------- --#
		print("Program Start  {0}".format(time.time()))
		setup()
		time.sleep(t_setup)
	


		tx1 = time.time()
		tx2 = tx1
		# ------------------- Release Fhase ------------------- #
		print("Releasing Judgement Program Start  {0}".format(time.time()))
		#loopx
		bme280Data=BME280.bme280_read()
		PRESS[0]=bme280Data[1]
		while (tx2-tx1<=x):
			luxjudge=Release.luxjudge()
			pressjudge=Release.pressjudge()
			if luxjudge==1 or pressjudge==1:
				break
			else:
		   		print("now in rocket ,taking photo")
			time.sleep(2)
			tx2=time.time()
		else:
			print("RELEASE TIMEOUT")
		print("THE ROVER HAS RELEASED")
		pi.write(22,1)
		# ------------------- Landing Fhase ------------------- #
		print("Releasing Judgement Program Start  {0}".format(time.time()))
		ty1=time.time()
		ty2=ty1
		gpsData = GPS.readGPS()
		bme280Data=BME280.bme280_read()
		PRESS[0]=bme280Data[1]
		while(ty2-ty1<=y):
			pressjudge=Land.pressjudge()
			gpsjudge=Land.gpsjudge()
			if pressjudge ==1 and gpsjudge ==1:
			    break
			elif pressjudge==0 and gpsjudge==0:
			    print("Descend now taking photo")
			elif pressjudge==1 or gpsjudge==1:
			    print("landjudgementnow")
			time.sleep(3)
			ty2=time.time()
		else:
			print("RELEASE TIMEOUT")
		print("THE ROVER HAS LANDED")
				
		# ------------------- Melting Fhase ------------------- #
		Melting.Melting()

		# ------------------- Avoidance of Parachute Fhase ------------------- #
		print("START: Judge covered by Parachute")
		ParaAvodance.ParaJudge()
		print("START: Parachute avoidance")
		ParaAvoidance.ParaAvoidance()

	except KeyboardInterrupt:
		close()
		print("Keyboard Interrupt")
	except Exception as e:
		close()
		print(e.message)
