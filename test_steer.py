# test_steer - By: Leon Chen - Sat Apr 17 2021

# Black Grayscale Line Following Example, updated for PID

# For this script to work properly you should point the camera at a line at a
# 45 or so degree angle. Please make sure that only the line is within the
# camera's field of view.

import sensor, image, pyb, math, time

# Camera setup...
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB.
sensor.set_framesize(sensor.QVGA) # use QVGA.
sensor.skip_frames(time=2000) # Let new settings take affect.
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock() # Tracks FPS.

w = sensor.width()
h = sensor.height()

# Each roi is (x, y, w, h). The line detection algorithm will try to find the
# centroid of the largest blob in each roi. The x position of the centroids
# will then be averaged with different weights where the most weight is assigned
# to the roi near the bottom of the image and less to the next roi and so on.
ROIS = [ # [ROI, weight]
        (0, 0, w, int(h/3), 0.1), # You'll need to tweak the weights for your app
        (0, int(h/3), w, int(h/3), 0.3), # depending on how your robot is setup.
        (0, int(2*h/3), w, int(h/3), 0.7)
       ]

# Tracks a black line. Use [(128, 255)] for a tracking a white line.
LINE = [(55, 100, -128, 127, -128, 127)]

steer_direction = -1

kp = 1.0   # P term of the PID 0.4
ki = 0.0     # I term of the PID
kd = 1.0     # D term of the PID 0.3

red_led   = pyb.LED(1)
green_led = pyb.LED(2)
blue_led  = pyb.LED(3)
ir_led    = pyb.LED(4)

old_error = 0
measured_angle = 0
set_angle = 90 # this is the desired steering angle (straight ahead)
p_term = 0
i_term = 0
d_term = 0
old_time = pyb.millis()

s1 = pyb.Servo(1) # P7 Motor
s2 = pyb.Servo(2) # P8 Steering
#print(s1.calibration()) # show throttle servo calibration

def constrain(val, mini, maxi):
    if val < mini:
        return mini
    elif val > maxi:
        return maxi
    else:
        return val

def angle():
    centroid_sum = 0
    for r in ROIS:
        blobs = img.find_blobs(LINE, roi=r[0:4], merge=True) # r[0:4] is roi tuple.
        if blobs:
            # Find the index of the blob with the most pixels.
            most_pixels = 0
            largest_blob = 0
            for i in range(len(blobs)):
                if blobs[i].pixels() > most_pixels:
                    most_pixels = blobs[i].pixels()
                    largest_blob = i

            # Draw a rect around the blob.
            img.draw_rectangle(blobs[largest_blob].rect())
            img.draw_cross(blobs[largest_blob].cx(),
                           blobs[largest_blob].cy())

            centroid_sum += blobs[largest_blob].cx() * r[4] # r[4] is the roi weight.

    # Compute the weight divisor (we're computing this so you don't have to make weights add to 1).
    weight_sum = 0
    for r in ROIS:
        weight_sum += r[4] # r[4] is the roi weight.

    center_pos = (centroid_sum / weight_sum) # Determine center of line.

    # Convert the center_pos to a deflection angle. We're using a non-linear
    # operation so that the response gets stronger the farther off the line we
    # are. Non-linear operations are good to use on the output of algorithms
    # like this to cause a response "trigger".
    deflection_angle = 0
    # The 80 is from half the X res, the 60 is from half the Y res. The
    # equation below is just computing the angle of a triangle where the
    # opposite side of the triangle is the deviation of the center position
    # from the center and the adjacent side is half the Y res. This limits
    # the angle output to around -45 to 45. (It's not quite -45 and 45).
    deflection_angle = -math.atan((center_pos-int(w/2))/int(h/2))

    # Convert angle in radians to degrees.
    deflection_angle = math.degrees(deflection_angle)
    return deflection_angle

def steer(steer_speed1, steer_speed2):
    s1.pulse_width(steer_speed1) # steer
    s2.pulse_width(steer_speed2)
    time.sleep_ms(10)

def update_pid():
    global old_time, old_error, measured_angle, set_angle
    global p_term, i_term, d_term
    now = pyb.millis()
    dt = now - old_time
    error = set_angle - measured_angle
    de = error - old_error

    p_term = kp * error
    i_term += ki * error
    i_term = constrain(i_term, 0, 100)
    d_term = (de / dt) * kd

    old_error = error
    steer_angle = steer_direction * (p_term + i_term + d_term)
    steer_speed1 = constrain(1250 - steer_angle * (250/30), 1000, 1500)
    steer_speed2 = constrain(1750 - steer_angle * (250/30), 1500, 2000)
    #print(steer_angle)
    output = (int(steer_speed1), int(steer_speed2))
    return output

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    deflection_angle = angle()

    # Now you have an angle telling you how much to turn the robot by which
    # incorporates the part of the line nearest to the robot and parts of
    # the line farther away from the robot for a better prediction.
    #print("Turn Angle: %f" % deflection_angle)
    now = pyb.millis()
    if  now > old_time + 1.0:  # time has passed since last measurement
        measured_angle = deflection_angle + 90
        steer_speed1, steer_speed2 = update_pid()
        old_time = now
        print(steer_speed1, steer_speed2)
        steer(steer_speed1, steer_speed2)
