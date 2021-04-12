# test_stop_sign - By: Leon Chen - Sun Apr 11 2021

# Can detect stop signs

import sensor, time, image

# Reset sensor
sensor.reset()

# Sensor settings
sensor.set_contrast(3)
sensor.set_gainceiling(16)

# Set resolution to VGA.
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)

# Load Haar Cascade
# By default this will use all stages, lower stages is faster but less accurate.
stop_sign = image.HaarCascade("/stop_sign_classifier_2.cascade")
print(stop_sign)

# FPS clock
clock = time.clock()

while (True):
    clock.tick()

    # Capture snapshot
    img = sensor.snapshot()

    # Find stop signs
    # Note: Lower scale factor scales-down the image more and detects smaller objects.
    # Higher threshold results in a higher detection rate, with more false positives.
    detected = img.find_features(stop_sign, threshold=0.5, scale_factor=1.5)
    for target in detected:
        img.draw_rectangle(target)
        img.draw_string(target[0], target[1], 'Stop sign', (0, 0, 255), mono_space=False)
        img.draw_cross(target[0], target[1])

    print(clock.fps())

