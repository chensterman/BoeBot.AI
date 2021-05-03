# servo_test - By: Leon Chen - Fri Apr 9 2021

import sensor, image, time, pyb

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()

# Servo settings
servo1 = pyb.Servo(1)   # create a servo object on position P7
servo2 = pyb.Servo(2)   # create a servo object on position P8

def steer(steer_speed1, steer_speed2):
    servo1.pulse_width(steer_speed1) # steer
    servo2.pulse_width(steer_speed2)
    time.sleep_ms(10)

#print(servo1.calibration())
i = 0
back = False
while(True):
    clock.tick()
    img = sensor.snapshot()

    steer(1520, 1520)

