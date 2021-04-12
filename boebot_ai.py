# boebot_ai - By: Leon Chen - Sun Apr 11 2021

# Full code base for BoeBot.AI

import sensor, image, time, mjpeg

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
red_led = (70, 100, 16, 80, -45, 20) # Red LED threshold
green_led = (70, 100, -84, -39, 13, 51) # Green LED threshold

# Load Haar Cascade
stop_sign = image.HaarCascade("/stop_sign_classifier_2.cascade")

# Reset sensor
sensor.reset()

# Initialize recording
rec = image.ImageIO("/stream.bin", "w")
frame_count = 0

# Sensor settings
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

    # Find stop signs
    # Note: Lower scale factor scales-down the image more and detects smaller objects.
    # Higher threshold results in a higher detection rate, with more false positives.
    detected = img.find_features(stop_sign, threshold=0.7, scale_factor=1.25)
    for target in detected:
        img.draw_rectangle(target)
        img.draw_string(target[0], target[1], 'Stop sign', (0, 0, 255), mono_space=False)

    # Add frame to recording
    rec.write(img)
    frame_count += 1
    if frame_count / clock.fps() > 10:
        rec.close()
        break

    print(clock.fps())
