import sys
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BME280')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BMX055')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Camera')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/GPS')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/IM920')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Melting')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Motor')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/TSL2561')
import time
import serial
import pigpio
import BME280
import BMX055
import IM920
import GPS
import Melting
import Motor
import TSL2561
deltPmax=0.1
deltHmax=5
Pcount=0
GAcount=0
bme280Data=[0.0,0.0]
def pressjudge():
	global Pcount
	global bme280Data
	secondlatestPRESS=bme280Data[1]
	bme280Data=BME280.bme280_read()	#更新
	latestPRESS=bme280Data[1]
	deltP=abs(latestPRESS-secondlatestPRESS)
	if abs(deltP)<deltPmax:
		Pcount+=1
	elif abs(deltP)>deltPmax:
		Pcount=0
	if Pcount>4:
		preslandjudge=1
		print("preslandjudge")
	else:
		preslandjudge=0
	print(str(latestPRESS)+"	:	"+"delt	"+str(deltP))
	print("Pcount	"+str(Pcount)
	return preslandjudge

def gpsjudge():
	gpsData=GPS.readGPS()#?
	global GAcount
	Gheight=gpsData[4]
	deltH=Gheight
	gpsData=GPS.readGPS()
	Gheight=gpsData[4]
	deltH=deltH-Gheight
	print(Gheight)
	#3秒ごとに判定
	time.sleep(3)
	if abs(deltH)<deltHmax:
		GAcount+=1
	elif abs(deltH)>deltHmax :
		GAcount=0
	if GAcount>4:
		gpsjudge=1
		print("GPSlandjudge")
	else:
		gpsjudge=0
	#print(str(Gheight))
	return gpsjudge
