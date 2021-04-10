# Untitled - By: Leon Chen - Fri Apr 9 2021

import sensor, image, time, os, tf, pyb

# Settings
person_threshold = 0.7

# LED set
led = pyb.LED(1)
led.off()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time = 2000)

# Load TF materials
net = tf.load('person_detection')
labels = ['unsure', 'person', 'no_person']
idx = labels.index('person')

clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()

    for obj in net.classify(img, min_scale=1.0, scale_mul=0.0, x_overlap=0.0, y_overlap=0.0):
        img.draw_rectangle(obj.rect())
        img.draw_string(obj.x(), obj.y(), labels[obj.output().index(max(obj.output()))], mono_space=False)

        if obj.output()[idx] > person_threshold:
            led.on()
        else:
            led.off()
