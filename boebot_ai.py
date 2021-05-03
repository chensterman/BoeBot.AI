# boebot_ai - By: Leon Chen - Sun Apr 11 2021

# Full code base for BoeBot.AI

import sensor, image, pyb, math, time, mjpeg

# Sensor settings
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_brightness(-3)
sensor.set_saturation(3)
sensor.set_contrast(3)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()

# Camera width and height
WIDTH = sensor.width()
HEIGHT = sensor.height()

# Each roi is (x, y, w, h). The line detection algorithm will try to find the
# centroid of the largest blob in each roi. The x position of the centroids
# will then be averaged with different weights where the most weight is assigned
# to the roi near the bottom of the image and less to the next roi and so on.
ROIS = [ # [ROI, weight]
        (0, 0, WIDTH, int(HEIGHT/3), 0.1), # You'll need to tweak the weights for your app
        (0, int(HEIGHT/3), WIDTH, int(HEIGHT/3), 0.3), # depending on how your robot is setup.
        (0, int(2*HEIGHT/3), WIDTH, int(HEIGHT/3), 0.7)
       ]

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
RED_LED = (38, 100, 40, 90, 2, 76)        # Red LED threshold
GREEN_LED = (60, 100, -90, -35, 50, 80)     # Green LED threshold
LINE = [(70, 100, -30, 30, -17, 60)]    # Line to follow threshold
#(75, 100, -30, 30, -20, 20)

# Load Haar Cascade for stop signs
STOP_SIGN = image.HaarCascade("/stop_sign_classifier_2.cascade")
STOP_CHECK = True
STOP = False
STOP_TIME = 0

# Initialize recording
#REC = image.ImageIO("/stream.bin", "w")
FRAME_COUNT = 0
REC_TIME = 30 # Amount of seconds to record
m = mjpeg.Mjpeg("example7.mjpeg")

# Initialize servo settings
SERVO1 = pyb.Servo(1) # P7 Motor
SERVO2 = pyb.Servo(2) # P8 Steering
#print(SERVO1.calibration())

# PID variables
P = 0
I = 0
D = 0

K_P = 1.0   # P term of the PID 0.4
K_I = 0.0     # I term of the PID
K_D = 1.0     # D term of the PID 0.3

SET_ANGLE = 90 # this is the desired steering angle (straight ahead)
MEASURED_ANGLE = 0
OLD_ERROR = 0
CURR_ERROR = 0
OLD_TIME = pyb.millis()
CURR_TIME = pyb.millis()

def constrain(val, mini, maxi):
    if val < mini:
        return mini
    elif val > maxi:
        return maxi
    else:
        return val

def find_red_lights():
    red_lights = img.find_blobs([RED_LED], pixels_threshold=10, area_threshold=30, merge=True)
    for blob in red_lights:
        img.draw_rectangle(blob.rect())
        img.draw_string(blob.x(), blob.y(), 'Traffic Light: Red', (0, 0, 255), mono_space=False)
        img.draw_cross(blob.cx(), blob.cy())
    if not red_lights:
        return False
    else:
        return True

def find_green_lights():
    green_lights = img.find_blobs([GREEN_LED], pixels_threshold=10, area_threshold=30, merge=True)
    for blob in green_lights:
        img.draw_rectangle(blob.rect())
        img.draw_string(blob.x(), blob.y(), 'Traffic Light: Green', (0, 0, 255), mono_space=False)
        img.draw_cross(blob.cx(), blob.cy())
    if not green_lights:
        return False
    else:
        return True

# Find stop signs
# Note: Lower scale factor scales-down the image more and detects smaller objects.
# Higher threshold results in a higher detection rate, with more false positives.
def find_stop_signs():
    stop_signs = img.find_features(STOP_SIGN, threshold=0.3, scale_factor=1.25)
    for target in stop_signs:
        img.draw_rectangle(target)
        img.draw_string(target[0], target[1], 'Stop sign', (0, 0, 255), mono_space=False)
    if not stop_signs:
        return False
    else:
        return True

def get_angle():
    centroid_sum = 0

    for r in ROIS:

        blobs = img.find_blobs(LINE, roi=r[0:4], merge=True) # r[0:4] is roi tuple.
        if blobs:

        #lines = img.find_line_segments(r[0:4], merge_distance=1000, max_theta_difference=100)
        #for l in lines:
            #if (l.theta() < 160) and (l.theta() > 20):
                #img.draw_line(l.line(), color = (255, 0, 0))
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
            #line_cx = (l.x1() + l.x2()) // 2
            #line_cy = (l.y1() + l.y2()) // 2
            #img.draw_cross(line_cx, line_cy)
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
    deflection_angle = -math.atan((center_pos-int(WIDTH/2))/int(HEIGHT/2))

    # Convert angle in radians to degrees.
    deflection_angle = math.degrees(deflection_angle)
    return deflection_angle + SET_ANGLE

def steer(steer_speed1, steer_speed2):
    SERVO1.pulse_width(steer_speed1) # steer
    SERVO2.pulse_width(steer_speed2)
    time.sleep_ms(10)

def stopped():
    SERVO1.pulse_width(1520)    # 1520 is stop speed
    SERVO2.pulse_width(1520)
    time.sleep_ms(10)

def update_pid():
    global P, I, D, OLD_ERROR, CURR_ERROR, OLD_TIME, CURR_TIME

    CURR_TIME = pyb.millis()
    dt = CURR_TIME - OLD_TIME
    CURR_ERROR = SET_ANGLE - MEASURED_ANGLE
    de = CURR_ERROR - OLD_ERROR

    P = K_P * CURR_ERROR
    I += K_I * CURR_ERROR
    I = constrain(I, 0, 100)
    D = (de / dt) * K_D

    OLD_ERROR = CURR_ERROR
    steer_angle = -1 * (P + I + D)
    steer_speed1 = constrain(1250 - steer_angle * (250/30), 1000, 1500)
    steer_speed2 = constrain(1750 - steer_angle * (250/30), 1500, 2000)
    #print(steer_angle)
    return (int(steer_speed1), int(steer_speed2))

'''
def rec_stream(img):
    global FRAME_COUNT
    REC.write(img)
    FRAME_COUNT += 1
'''
def rec_mjpeg(img):
    global FRAME_COUNT
    m.add_frame(img)
    FRAME_COUNT += 1

while(True):
    clock.tick()
    img = sensor.snapshot()

    CURR_TIME = pyb.millis()
    MEASURED_ANGLE = get_angle()

    red_light = find_red_lights()
    green_light = find_green_lights()
    stop_display = find_stop_signs()

    if STOP_CHECK:
        STOP = stop_display
    if STOP:
        stopped()
        STOP_CHECK = False
        if CURR_TIME - OLD_TIME > 5000:
            STOP_TIME = pyb.millis()
            STOP = False
        continue
    else:
        if CURR_TIME - STOP_TIME > 5000:
            STOP_CHECK = True

    #rec_mjpeg(img)
    #if FRAME_COUNT / clock.fps() > REC_TIME:
    #    break

    if red_light and not green_light:
        stopped()
        #print("stopped")
        OLD_TIME = pyb.millis()
        continue

    # Now you have an angle telling you how much to turn the robot by which
    # incorporates the part of the line nearest to the robot and parts of
    # the line farther away from the robot for a better prediction.
    #print("Turn Angle: %f" % deflection_angle)
    if  CURR_TIME > OLD_TIME + 1.0:  # time has passed since last measurement
        steer_speed1, steer_speed2 = update_pid()
        OLD_TIME = CURR_TIME
        print(steer_speed1, steer_speed2)
        steer(steer_speed1, steer_speed2)

    #print(clock.fps())
m.close(clock.fps())
print("Recording done")
