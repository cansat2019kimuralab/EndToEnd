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


luxstr = ["lux1", "lux2"]																#variable to show lux returned variables
bme280str = ["temp", "pres", "hum", "alt"]												#variable to show bme280 returned variables
bmx055str = ["accx", "accy", "accz", "gyrx", "gyry", "gyrz", "dirx", "diry", "dirz"]	#variable to show bmx055 returned variables
gpsstr = ["utctime", "lat", "lon", "sHeight", "gHeight"]								#variable to show GPS returned variables

t_setup = 10	#variable to set waiting time after setup

t = 1	#waitingtime
x = 61	#time for release(loopx)
y = 60	#time for land(loopy)
z = 120	#time for run(loopz)
lcount=0
acount=0
Pcount=0
GAcount=0
luxmax=300
deltAmax=0.3
deltPmax=0.05
deltHmax=5


pi = pigpio.pi()   #object of pigpio
def gpsSend(gpsData):
	IM920.Send('g'+str(gpsData[0]) + ',' + str(gpsData[1]) + ',' + str(gpsData[2]) + ',' + str(gpsData[3]) + ',' + str(gpsData[4]) + ',' + str(gpsData[5]))

def setup():
	pi.set_mode(22,pigpio.OUTPUT)
	pi.write(22,0)#IM920
	pi.write(17,0)#outcasing
	time.sleep(1)
	BME280.bme280_setup()
	BME280.bme280_calib_param()
	BMX055.bmx055_setup()
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
    	# ------------------- Setup Fhase ------------------- #
		print("Program Start  {0}".format(time.time()))
		setup()
		time.sleep(t_setup)
		bme280Data=BME280.bme280_read()


		tx1 = time.time()
		tx2 = tx1

		# ------------------- Release Fhase ------------------- #
		print("Releasing Judgement Program Start  {0}".format(time.time()))
		#loopx
		while(1):

			#tx2update
			tx2=time.time()
			#release judgement(lux)
			luxdata=TSL2561.readLux()
			#f.write(str(luxdata[0])+":"+str(luxdata[1]))
			print("lux1: "+str(luxdata[0])+" "+"lux2: "+str(luxdata[1]))
			with open('log/releaseLog.txt', 'a') as f:
				f.write(str(luxdata[0])+"	"+str(luxdata[1])+"		")

			print(lcount)
			print(acount)

			if luxdata[0]>luxmax or luxdata[1]>luxmax:
				lcount+=1
			elif luxdata[0]<luxmax and luxdata[1]<luxmax:
				lcount=0
			if lcount>4:
				luxreleasejudge=True
				print("luxreleasejudge")
			else:
				luxreleasejudge=False
			#releasejudgement(press)
				PRESS=bme280Data[1]
				deltA=PRESS
				bme280Data=BME280.bme280_read()	#update
				PRESS=bme280Data[1]
				deltA=PRESS-deltA
				#f.write("P0:P1"+str(P0)+":"+str(P1))
				print(str(PRESS))
				print(str(deltA))
				time.sleep(2)
				#compare with 3 second ago
				#acout+=1
				if deltA>deltAmax:
					acount+=1
				elif deltA<deltAmax:
					acount=0
				if acount>2:
					presreleasejudge=True
					print("presjudge")

				else:
					presreleasejudge=False
			with open('log/releaseLog.txt', 'a') as f:
				f.write("\n")
				f.write(str(deltA)+"   "+str(PRESS)+"\t")
				f.write("\n")

			if luxreleasejudge or presreleasejudge:
				ty1=time.time()
				ty2=ty1
				print("RELEASE!")
				with open('log/releaseLog.txt', 'a') as f:
					f.write("release")
				pi.write(22,0)
				bme280Data=BME280.bme280_read()
				gpsData = GPS.readGPS()
			# ------------------- Landing Fhase ------------------- #
				while(ty2-ty1<=y):
					print(str(ty2-ty1))
					ty2=time.time()
					#presjudgement
					PRESS=bme280Data[1]
					deltP=PRESS
					bme280Data=BME280.bme280_read()	#update
					PRESS=bme280Data[1]
					deltP=deltP-PRESS
					with open('log/landingLog.txt', 'a') as f2:
						f2.write("\n")
						f2.write(str(deltP))#+":"+str(PRESS)+ "\t")
						f2.write("\n")
					print(deltP)
					print("Pcount:"+str(Pcount))
					if abs(deltP)<deltPmax:
						Pcount+=1
					elif abs(deltP)>deltPmax:
						Pcount=0
					if Pcount>4:
						preslandjudge=True
						print("preslandjudge")
					else:
						preslandjudge=False

					#GPS alt judgement
					Gheight=gpsData[4]
					deltH=Gheight
					gpsData=GPS.readGPS()
					Gheight=gpsData[4]
					deltH=deltH-Gheight
					print(Gheight)
					#judge every 4sec
					time.sleep(3)
					if abs(deltH)<deltHmax:
						GAcount+=1
					elif abs(deltH)>deltHmax :
						GAcount=0
					if GAcount>4:
						GPSlandjudge=True
						print("GPSlandjudge")
					else:
						GPSlandjudge=False


					#take photo when chenging pres and GPSalt
					if not preslandjudge and #ot GPSlandjudge:
						print("satueinow")
					#taking photo
					# out loop y when no change pres and GPSalt
					elif preslandjudge #and  GPSlandjudge:
						break
					else:
						print("Landingjudgement now")
						#ループy中でbreakが起きなければ続行、起きたら全体も抜ける 
				else:
					print("THE ROVER HAS LANDED(timeout y).  {0}".format(time.time()))
					with open('log/landingLog.txt', 'a') as f2:
						f2.write("land")
					break
				print("THE ROVER HAS LANDED.  {0}".format(time.time()))
				with open('log/landingLog.txt', 'a') as f2:
					f2.write("land")
				break
					#放出されず、かつループxでタイムアウト
			elif tx2-tx1>=x:
				tz1=time.time()
				tz2=tz1
				print("X timeout")
				#ループzのタイムアウト判定
				while(tz2-tz1<=z):
					tz2=time.time()
					PRESS=bme280Data[1]
					deltP=PRESS
					bme280Data=BME280.bme280_read()	#更新
					PRESS=bme280Data[1]
					deltP=deltP-PRESS
					print(PRESS)
					#3秒ごとに判定.
					time.sleep(3)
					if abs(deltP)<deltPmax:
						Pcount+=1
					elif abs(deltP)>deltPmax:
						Pcount=0
					if Pcount>5:
						preslandjudge=True
					print("preslandjudge")
					else:
						preslandjudge=False
					#気圧が変化しなければループzを抜ける
					if  preslandjudge:
						break
					#ループz中でbreakが起きなければ続行、起きたら全体も抜ける
				else:
					break
				break
		#溶断へ						
				# ------------------- Melting Fhase ------------------- #
		Melting.Melting()

				# ------------------- Avoidance of Parachute Fhase ------------------- #
				print("START: Judge covered by Parachute")
				ParaJudge()
				print("START: Parachute avoidance")
				ParaAvoidance()

			except KeyboardInterrupt:
				close()
				print("Keyboard Interrupt")
			except Exception as e:
				close()
				print(e.message)
