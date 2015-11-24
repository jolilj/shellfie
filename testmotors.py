import motorcontroller as motors
__author__ = 'esillen@kth.se'


try:
    while True:
        speed = int(raw_input("speed? "))
        motors.set_speed(speed)
except KeyboardInterrupt:
    motors.stop_motors()
