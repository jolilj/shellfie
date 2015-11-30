import math
import sys
import RTIMU
import time
import RPi.GPIO as GPIO
import motorcontroller
#import cal_vals
#import calibration

CAL_TIME = 5
#initialize PID controller constants
KP = 700
KI = 150
KD = 10
I_THRESHOLD = 2
TARGET_ANGLE = 0

#Setting up I2C comunication with the IMU
SETTINGS_FILE = "RTIMU1"
s = RTIMU.Settings("RTIMU1")
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed");
    sys.exit(1)
else:
    print("IMU Init Succeeded");

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

b= 0
initAngle = 0
if (not imu.IMUInit()):
        print("IMU Init Failed");
        sys.exit(1)
else:
        print("IMU Init succeeded")

def updateAngle(dt, oldTheta):
	global imu, b, initAngle
	data = imu.getIMUData()

	gyroData = data["gyro"]
	accelData = data["accel"]
	theta_a = math.atan2(accelData[1],accelData[2]) - initAngle
	theta_g = oldTheta + (gyroData[0] - b)*dt
	newAngle = 0.98*(theta_g) + 0.02*theta_a 
	return newAngle

def initialAngle():
	global imu,b
	initTime = time.time()
	t = initTime
	omega_y = []
	acc_y = []
	acc_z = []
	while (t - initTime) < CAL_TIME:
		if imu.IMURead():
		  data = imu.getIMUData()
		  gyroData = data["gyro"]
		  accelData = data["accel"]

		  omega_y.append(gyroData[0])
		  acc_y.append(accelData[1])
		  acc_z.append(accelData[2])

		  time.sleep(poll_interval*1.0/1000.0)
		  t = time.time()
	# Remove transient part
	start = int(round(float(len(omega_y)*2)/4)) 
	acc_y = acc_y[start:]
	acc_z = acc_z[start:]
	omega_y = omega_y[start:]
	#Calculate mean(bias)
	b = float(sum(omega_y))/len(omega_y)
	by = float(sum(acc_y))/len(acc_y)
	bz = float(sum(acc_z))/len(acc_z) - 0.982

	print('BIAS Gyro: ' + str(b))
	print('BIAS Y: ' + str(by))
	print('BIAS Z: ' + str(bz))

	angle = math.atan2(accelData[1]-by,accelData[2]-bz)
	return angle

#Initialize pins

#Call calibration here
#calibration.calibrateSensors(500,imu)

initAngle = initialAngle() #Calibration
print(math.degrees(initAngle))

# Wait for key to enter balancing mode
raw_input("------------PRESS ENTER WHEN READY TO START CONTROL LOOP------------")
angle = initAngle
prevTime = time.time()
prevError = 0
err = 0
i_err_time = 0
motorcontroller.set_speed(0)

try:
	while True:
		if imu.IMURead():
			currentTime = time.time()
			dt = currentTime - prevTime
			angle = updateAngle(dt, angle)
			error = angle - TARGET_ANGLE
			
			if i_err_time > 0.7:
				print("Resetting I_term")
				err = 0
				i_err_time = 0
			err = err + dt*error

			#if abs(error) > math.radians(4):
			#	error = error*2


			if abs(error) < math.radians(I_THRESHOLD):
				i_err_time = i_err_time + dt
			else:
				i_err_time = 0

			P_term = KP*error
			I_term = KI*err
			D_term = KD*float((error-prevError))/dt

			#print(str(P_term) + " " + str(I_term))
			speed = P_term+I_term+D_term
			#print(speed)
			motorcontroller.set_speed(speed)
			#print(math.degrees(angle))
			time.sleep(poll_interval*1.0/1000.0)
			prevTime = currentTime
			prevError = error
except KeyboardInterrupt:
	pass
# Stop PWM signals
GPIO.cleanup()


