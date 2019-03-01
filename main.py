# LAB3
# Jimmy Hoang
# William Gentry
# Daniel Burr

import sensor, image, math, time, pyb, motors, bluetooth
from pyb import Pin, Timer, UART, LED, ADC
from math import sqrt
from motors import DCMotor, ServoMotor
from bluetooth import Bluetooth

##### Global Variables and Initialization #####
## uart
uart = UART(1, 115200)
bt = Bluetooth(uart)

## DC motor control
dc_motor = DCMotor(tim_num=2, channel=4, frequency=100, pin="P5")
dc_motor.set_control(in_a="P2", in_b="P3", en_a="P4", en_b="P8")
dc_motor.set_current_sense(cs="P6", cs_dis="P9")
dutycyclePW = 0
enable = 0

## Servo motor control
servo_max = 0.0019 #turning left
servo_min = 0.001 #turning right
servo_center = (servo_max + servo_min) / 2
servo_offset = servo_max - servo_center
servo_motor = ServoMotor(tim_num=4, channel=1, frequency=300, pin="P7")
servo_motor.set_range(max_pw=servo_max, min_pw=servo_min)
sec = servo_center
servo_motor.set_pulse_width(sec)

## Camera Control
thresholds = (255, 255) #245 to 255

roi1 = (0, 20, 160, 10)
roi2 = (0, 0, 160, 10)  ###### note changed to 90
roi3 = (0, 30, 160, 10)  ###### note changed to 90


dist = lambda cb,cr: abs(cr-cb)
pin1 = Pin('P1', Pin.OUT_PP, Pin.PULL_NONE)
x_1 = 0
y_1 = 0
x_2 = 0
y_2 = 0
x_3 = 0
y_3 = 1
x_diff1 = 0
x_err1 = [0, 0, 0, 0]
dist2center = 0
blobs1 = []
blobs2 = []
blobs3 = []
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
Kp_min_s = 1.5
Kd_s = 0.025
max_pwm = 35
min_pwm = 25

# brake control
turning = 0
braking = 0
brake_counter = 0

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
            Kd = value
        if (cmd == "Kd"):
            Kd_s = value

        if (cmd =="max_pwm"):
            max_pwm = value
        if (cmd == "min_pwm"):
            min_pwm = value

    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image.


    ##### Blob Detection #####

    # REGION1
    blobs1 = img.find_blobs([thresholds], roi = roi1, pixels_threshold=10, area_threshold=10)
    center = roi1[2]/2

    if (len(blobs1) > 0):
        if (len(blobs1) == 3 and (abs(blobs1[1].cx() - blobs1[0].cx()) < 30 and abs(blobs1[1].cx() - blobs1[2].cx()) < 30)):
            dc_motor.brake_vcc()
            print("3 lines detected")
            enable = 0
        else:
            minblob1 = blobs1[0]
            mindist1 = dist(minblob1.cx(),center)


            for blob in blobs1:
                if (dist(blob.cx(), center) < mindist1):
                    minblob1 = blob

            img.draw_rectangle(minblob1.rect(), color = 0)
            img.draw_cross(minblob1.cx( ), minblob1.cy(), color = 0)
            x_1 = minblob1.cx()
            y_1 = minblob1.cy()

    blobs2 = img.find_blobs([thresholds], roi = roi2, pixels_threshold=10, area_threshold=10)

    if (len(blobs2) > 0):
        minblob2 = blobs2[0]
        mindist2 = dist(minblob2.cx(),center)


        for blob in blobs2:
            if (dist(blob.cx(), center) < mindist2):
                minblob2 = blob

        img.draw_rectangle(minblob2.rect(), color = 0)
        img.draw_cross(minblob2.cx( ), minblob2.cy(), color = 0)
        x_2 = minblob2.cx()
        y_2 = minblob2.cy()

    blobs3 = img.find_blobs([thresholds], roi = roi3, pixels_threshold=10, area_threshold=10)

    if (len(blobs3) > 0):
        minblob3 = blobs3[0]
        mindist3 = dist(minblob3.cx(),center)


        for blob in blobs3:
            if (dist(blob.cx(), center) < mindist3):
                minblob3 = blob

        img.draw_rectangle(minblob3.rect(), color = 0)
        img.draw_cross(minblob3.cx( ), minblob3.cy(), color = 0)
        x_3 = minblob3.cx()
        y_3 = minblob3.cy()

    if(y_3 - y_2 != 0):
        angle = math.atan((x_2 - x_3) / (y_3 - y_2))
        angle = math.degrees(angle)

    if (len(blobs2) > len(blobs3) or len(blobs3) > len(blobs2) or len(blobs2) == 0):
        angle = 0

    #PROPORTIONAL

    if (len(blobs1) == 0): #uses the last dist2center if off track to correct
        if(dist2center > 0):
            dist2center = center
        else:
            dist2center = -center

    if(len(blobs1) > 1):
        dist2center = dist2center
        dist2center = dist2center

    else:
        dist2center = center - x_1


    #DIFFERENTIAL
    for i in range(3):
         x_err1[i] = x_err1[i+1]
    x_err1[3] = center - x_1 #dist2center #changing to dist2center makes it follow derivative of far line in a straight away!!
    x_diff1 = (x_err1[3] - x_err1[0] + (3*x_err1[2]) - (3*x_err1[1]))

    #DC MOTOR CONTROL
    if (enable == 0):
        dc_motor.brake_vcc()

    elif (enable == 1):

        dc_motor.forward()
        dutycyclePW =  max_pwm  - (abs(dist2center/center) * (max_pwm - min_pwm)) #DC

        #print(angle)
        #print(turning)
        #print(braking)

        if (turning == 1):
            if (braking == 1):
                if(brake_counter <= 10):
                    dutycyclePW = 80
                    dc_motor.brake_gnd()
                    brake_counter += 1
                else:
                    dc_motor.forward()
                    braking = 0

            if (abs(angle) < 5 and abs(angle) != 0):
                turning = 0
        else:
            if (abs(angle) > 10 ):
                turning = 1
                braking = 1
                brake_counter = 0


    #SERVO MOTOR CONTROL
    sec = servo_center + servo_offset *( (dist2center/center) * (Kp_min_s) + (x_diff1 * Kd_s) )

    if sec > servo_max :
        sec = servo_max
    if sec < servo_min:
        sec = servo_min

    servo_motor.set_pulse_width(sec)
    dc_motor.set_duty_cycle(dutycyclePW)
