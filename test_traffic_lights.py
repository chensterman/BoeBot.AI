# test_traffic_lights - By: Leon Chen - Sat Apr 10 2021

# Traffic light detection

import sensor, image, time, math

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
red_led = (70, 100, 16, 80, -45, 20) # Red LED threshold
green_led = (70, 100, -84, -39, 13, 51) # Green LED threshold

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()

    # Detect red traffic lights
    for blob in img.find_blobs([red_led], pixels_threshold=10, area_threshold=10, merge=True):
        img.draw_rectangle(blob.rect())
        img.draw_string(blob.x(), blob.y(), 'Traffic Light: Red', (0, 0, 255), mono_space=False)
        img.draw_cross(blob.cx(), blob.cy())

    # Detect green traffic lights
    for blob in img.find_blobs([green_led], pixels_threshold=10, area_threshold=10, merge=True):
        img.draw_rectangle(blob.rect())
        img.draw_string(blob.x(), blob.y(), 'Traffic Light: Green', (0, 0, 255), mono_space=False)
        img.draw_cross(blob.cx(), blob.cy())

    # print(clock.fps())

