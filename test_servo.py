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

while(True):
    clock.tick()
    img = sensor.snapshot()

    servo1.speed(30)
    servo2.speed(30)

