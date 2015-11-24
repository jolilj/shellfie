__author__ = 'esillen@kth.se'
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

# Initialize pins
input1 = 13
input2 = 15
enableA = 18
input3 = 12
input4 = 16
enableB = 22
# Try to change the frequency!
pwmfreq = 800
GPIO.setup(input1, GPIO.OUT)
GPIO.setup(input2, GPIO.OUT)
GPIO.setup(enableA, GPIO.OUT)
motCtrl1 = GPIO.PWM(enableA, pwmfreq)
motCtrl1.start(0)
GPIO.setup(input3, GPIO.OUT)
GPIO.setup(input4, GPIO.OUT)
GPIO.setup(enableB, GPIO.OUT)
motCtrl2 = GPIO.PWM(enableB, pwmfreq)
motCtrl2.start(0)


def set_speed(speed):
    absspeed = abs(speed)
    absspeed = int(min(absspeed, 99.9))
    if speed > 0:
        GPIO.output(input1, 1)
        GPIO.output(input2, 0)
        motCtrl1.ChangeDutyCycle(absspeed)
        GPIO.output(input3, 1)
        GPIO.output(input4, 0)
        motCtrl2.ChangeDutyCycle(absspeed)
    else:
        GPIO.output(input1, 0)
        GPIO.output(input2, 1)
        motCtrl1.ChangeDutyCycle(absspeed)
        GPIO.output(input3, 0)
        GPIO.output(input4, 1)
        motCtrl2.ChangeDutyCycle(absspeed)


def stop_motors():
    GPIO.output(input1, 1)
    GPIO.output(input2, 1)
    motCtrl1.ChangeDutyCycle(90)
    GPIO.output(input3, 1)
    GPIO.output(input4, 1)
    motCtrl2.ChangeDutyCycle(90)
