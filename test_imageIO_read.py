# test_imageIO_read - By: Leon Chen - Mon Apr 12 2021

import sensor, image, time

snapshot_source = False # Set to true once finished to pull data from sensor.

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()

stream = None
if snapshot_source == False:
    stream = image.ImageIO("/stream.bin", "r")

while(True):
    clock.tick()
    if snapshot_source:
        img = sensor.snapshot()
    else:
        img = stream.read(copy_to_fb=True, loop=True, pause=False)
    # Do machine vision algorithms on the image here.
    print(clock.fps())
