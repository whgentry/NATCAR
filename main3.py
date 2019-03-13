# Jimmy Hoang
# William Gentry
# Daniel Burr

import sensor, image, math, time, pyb, motors, bluetooth
from pyb import Pin, Timer, UART, LED, ADC
from math import sqrt
from motors import DCMotor, ServoMotor
from bluetooth import Bluetooth

##### Global Variables and Initialization #####

## UART
uart = UART(1, 115200)
bt = Bluetooth(uart)

## DC motor control
dc_motor = DCMotor(tim_num=2, channel=4, frequency=100, pin="P5")
dc_motor.set_control(in_a="P2", in_b="P3", en_a="P4", en_b="P8")
dc_motor.set_current_sense(cs="P6", cs_dis="P9")
dutycyclePW = 0

## Servo motor control
servo_max = 0.0019 #turning left
servo_min = 0.001 #turning right
servo_center = (servo_max + servo_min) / 2
servo_offset = servo_max - servo_center
servo_motor = ServoMotor(tim_num=4, channel=1, frequency=300, pin="P7")
servo_motor.set_range(max_pw=servo_max, min_pw=servo_min)
sec = servo_center
servo_motor.set_pulse_width(sec)

enable = 0

## Camera Control
thresholds = (265, 275) #245 to 255

roi_s1 = 10
roi_s2 = 40
roi_num = 6
roi_dist = 5

roi_array_1 = [(0,roi_s1,160,10)] * roi_num
roi_array_2 = [(0,roi_s2,160,10)] * roi_num

for i in range(roi_num):
    roi_array_1[i] = (0,roi_s1 + roi_dist * i,160,10)
    roi_array_2[i] = (0,roi_s2 + roi_dist * i,160,10)

dist_array = [0] * roi_num
center = 80

dist = lambda cb,cr: abs(cr-cb)
pin1 = Pin('P1', Pin.OUT_PP, Pin.PULL_NONE)

x_1 = 80
x_2 = 80
y_1 = 0
y_2 = 1

x_diff1 = 0
x_err1 = [0, 0, 0, 0]
dist2center = 0
motor = 0

# Configure camera
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()                # Create a clock object to track the FPS.

## Control Values
# PID
Kp_s = 1.4
Kd_s = 0.014
max_pwm = 35
min_pwm = 25

# brake control
turning = 0
braking = 0
brake_counter = 0
brake_max = 20

##### MAIN LOOP #####
while(True):
    if (bt.any()):
        dc_motor.brake_vcc()
        cmd, value = bt.get_cmd_value_blocking()
        if (cmd == "g"):
            enable = 1
        if (cmd == "x"):
            enable = 0

        if (cmd == "Kp"):
            Kp_s = value
        if (cmd == "Kd"):
            Kd_s = value
        if (cmd == "b"):
            brake_max = value

        if (cmd =="max_pwm"):
            max_pwm = value
        if (cmd == "min_pwm"):
            min_pwm = value

    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image.
    #print(clock.fps())

    ##### Blob Detection #####

    #ROI_ARRAY1
    for i in range(roi_num):
        blobs = img.find_blobs([thresholds], roi = roi_array_1[i], pixels_threshold=10, area_threshold=10)

        if (len(blobs) > 0):
            if (len(blobs) == 3 and i == 0):
                line1_dist = abs(blobs[1].cx() - blobs[0].cx())
                line2_dist = abs(blobs[1].cx() - blobs[2].cx())
                if (line1_dist < 25 and line1_dist > 10 and line2_dist < 25 and line2_dist > 10):
                    print(line1_dist)
                    print("3 lines detected")
                    enable = 0

            else:
                minblob = blobs[0]
                mindist = dist(minblob.cx(),x_1)

                for blob in blobs:
                    if (dist(blob.cx(), x_1) < mindist):
                        minblob = blob

                img.draw_cross(minblob.cx( ), minblob.cy(), color = 0)
                dist_array[i] = minblob.cx()

    x_1 = int(sum(dist_array) / len(dist_array))
    y_1 = 10
    img.draw_cross(x_1, 120, color = 0)

    #ROI_ARRAY2
    for i in range(roi_num):
        blobs = img.find_blobs([thresholds], roi = roi_array_2[i], pixels_threshold=10, area_threshold=10)

        if (len(blobs) > 0):
            minblob = blobs[0]
            mindist = dist(minblob.cx(),x_2)

            for blob in blobs:
                if (dist(blob.cx(), x_2) < mindist):
                    minblob = blob

            img.draw_cross(minblob.cx( ), minblob.cy(), color = 0)
            dist_array[i] = minblob.cx()

    x_2 = int(sum(dist_array) / len(dist_array))
    y_2 = 40
    img.draw_cross(x_2, 120, color = 0)

    angle = math.atan((x_1 - x_2) / (y_2 - y_1))
    angle = math.degrees(angle)

    ##### SERVO MOTOR CONTROL #####

    #DIFFERENTIAL
    for i in range(3):
         x_err1[i] = x_err1[i+1]
    x_err1[3] = center - x_1 #dist2center #changing to dist2center makes it follow derivative of far line in a straight away!!
    x_diff1 = (x_err1[3] - x_err1[0] + (3*x_err1[2]) - (3*x_err1[1]))

    #PROPORTIONAL
    dist2center = center - x_1
    sec = servo_center + servo_offset*( ((dist2center/center) * (Kp_s)) + (x_diff1 * Kd_s) )

    if sec > servo_max :
        sec = servo_max
    if sec < servo_min:
        sec = servo_min

    #DC MOTOR CONTROL
    if (enable == 0):
        dc_motor.brake_vcc()

    elif (enable == 1):

        dc_motor.forward()
        dutycyclePW =  max_pwm  - (abs(dist2center/center) * (max_pwm - min_pwm)) #DC
        print(angle)
        print(turning)
        print(braking)

        if (turning == 1):
            if (braking == 1):
                if(brake_counter <= brake_max):
                    dc_motor.brake_gnd()
                    brake_counter += 1
                else:
                    dc_motor.forward()
                    braking = 0

            if (abs(angle) < 5 and abs(angle) != 0):
                turning = 0
        else:
            if (abs(angle) > 10):
                turning = 1
                braking = 1
                brake_counter = 0

    dc_motor.set_duty_cycle(dutycyclePW)
    servo_motor.set_pulse_width(sec)
