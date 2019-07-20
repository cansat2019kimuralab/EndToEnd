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

phaseLog = 0	#variable for phase Check
luxstr = ["lux1", "lux2"]																#variable to show lux returned variables
bme280str = ["temp", "pres", "hum", "alt"]												#variable to show bme280 returned variables
bmx055str = ["accx", "accy", "accz", "gyrx", "gyry", "gyrz", "dirx", "diry", "dirz"]	#variable to show bmx055 returned variables
gpsstr = ["utctime", "lat", "lon", "sHeight", "gHeight"]								#variable to show GPS returned variables


t_setup = 1	#variable to set waiting time after setup
t = 1	#waitingtime
x = 120	#time for release(loopx)
y = 60	#time for land(loopy)

#lcount=0
acount=0
Pcount=0
GAcount=0
luxmax=300
deltHmax=5
PRESS=[]
pi=pigpio.pi()

def setup():
	pi.set_mode(22,pigpio.OUTPUT)
	pi.write(22,0)	#IM920
	pi.write(17,0)	#outcasing
	time.sleep(1)
	BME280.bme280_setup()
	BME280.bme280_calib_param()
	BMX055.bmx055_setup()
	GPS.openGPS()

	with open('log/phaseLog.txt', 'a') as f:
		pass
	with open('log/releaseLog.txt', 'a') as f:
		pass
	with open('log/landingLog.txt', 'a') as f:
		pass
	with open('log/runningLog.txt', 'a') as f:
		pass
	phaseLog = Other.phaseCheck('log/phaseLog.txt')
	print(phaseLog)

def close():
	GPS.closeGPS()
	pi.write(22, 0)
	pi.write(17,0)
	Motor.motor(0, 0, 1)
	Motor.motor_stop()

if __name__ == "__main__":
	try:
		with open('log/phaseLog.txt', 'a') as f:
			f.write("1\tProgram Started\t{0}".format(time.time()))	
		# ------------------- Setup Phase --------------------- #
		print("Program Start  {0}".format(time.time()))
		setup()

		# ------------------- Waiting Phase --------------------- #
		with open('log/phaseLog.txt', 'a') as f:
			f.write("2\tRelease Phase Started\t{0}".format(time.time()))		
		if(phaseLog <= 2):
			time.sleep(t_setup)

		# ------------------- Release Phase ------------------- #
		with open('log/phaseLog.txt', 'a') as f:
			f.write("3\tRelease Phase Started\t{0}".format(time.time()))
		if(phaseLog <= 3):
			tx1 = time.time()
			tx2 = tx1
			print("Releasing Judgement Program Start  {0}".format(time.time()))
			#loopx
			bme280Data=BME280.bme280_read()
			while (tx2-tx1<=x):
				luxjudge=Release.luxjudge()
				pressjudge=Release.pressjudge()
				
				if luxjudge==1 or pressjudge==1:
					break
				else:
		   			print("now in rocket ,taking photo")
				gpsData=GPS.readGPS()
				bmx055data=BMX055.bmx055_read()
				luxdata=TSL2561.readLux()
				Other.saveLog('log/releaseLog.txt',gpsData,bme280Data,luxdata,bmx055data)
				time.sleep(1)
				gpsData=GPS.readGPS()
				bmx055data=BMX055.bmx055_read()
				luxdata=TSL2561.readLux()
				Other.saveLog('log/releaseLog.txt',gpsData,bme280Data,luxdata,bmx055data)
				time.sleep(1)
				tx2=time.time()
			else:
				print("RELEASE TIMEOUT")
			print("THE ROVER HAS RELEASED")
			pi.write(22,1)
			IM920.Send("RELEASE")

		# ------------------- Landing Phase ------------------- #
		with open('log/phaseLog.txt', 'a') as f:
			f.write("4\tLanding Phase Started\t{0}".format(time.time()))	
		if(phaseLog <= 4):
			print("Releasing Judgement Program Start  {0}".format(time.time()))
			ty1=time.time()
			ty2=ty1
			#loopy
			gpsData = GPS.readGPS()
			bme280Data=BME280.bme280_read()
			while(ty2-ty1<=y):
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
				Other.saveLog('log/landingLog.txt',gpsData,bme280Data,bmx055data)
				time.sleep(1)
				gpsData=GPS.readGPS()
				bmx055data=BMX055.bmx055_read()
				luxdata=TSL2561.readLux()
				Other.saveLog('log/releaseLog.txt',gpsData,bme280Data,luxdata,bmx055data)
				time.sleep(1)
				gpsData=GPS.readGPS()
				bmx055data=BMX055.bmx055_read()
				luxdata=TSL2561.readLux()
				Other.saveLog('log/releaseLog.txt',gpsData,bme280Data,luxdata,bmx055data)
				time.sleep(1)
				ty2=time.time()
			else:
				print("LAND TIMEOUT")
			print("THE ROVER HAS LANDED")
			IM920.Send("LAND")
				
		# ------------------- Melting Phase ------------------- #
		with open('log/phaseLog.txt', 'a') as f:
			f.write("5\tMelting Phase Started\t{0}".format(time.time()))	
		if(phaseLog <= 5):
			Melting.Melting()

		# ------------------- ParaAvoidance Phase ------------------- #
		with open('log/phaseLog.txt', 'a') as f:
			f.write("6\tParaAvoidance Phase Started\t{0}".format(time.time()))	
		if(phaseLog <= 6):
			print("START: Judge covered by Parachute")
			ParaAvoidance.ParaJudge()
			print("START: Parachute avoidance")
			ParaAvoidance.ParaAvoidance()

		close()
	except KeyboardInterrupt:
		close()
		print("Keyboard Interrupt")

	except Exception as e:
		close()
		print(e.message)
		IM920.Send("error")
