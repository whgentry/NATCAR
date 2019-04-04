# Trial 1
# Daniel Burr
# William Gentry
# Jimmy Hoang

import sensor, image, math, time, pyb, motors, bluetooth
from pyb import Pin, Timer, UART, LED, ADC
from math import sqrt
from motors import DCMotor, ServoMotor
from bluetooth import Bluetooth

##### Global Variables and Initialization #####
uart = UART(1, 115200)
bt = Bluetooth(uart)

## DC motor control
dc_motor = DCMotor(tim_num=2, channel=4, frequency=100, pin="P5")
dc_motor.set_control(in_a="P2", in_b="P3", en_a="P4", en_b="P8")
dc_motor.set_current_sense(cs="P6", cs_dis="P9")
dutycyclePW = 0
enable = 0

## Servo motor control
servo_max = 0.0019
servo_min = 0.0010
servo_center = (servo_max + servo_min) / 2
servo_offset = servo_max - servo_center
servo_motor = ServoMotor(tim_num=4, channel=1, frequency=300, pin="P7")
servo_motor.set_range(max_pw=servo_max, min_pw=servo_min)
sec = servo_center
sec_prev = sec

## Camera Control
thresholds = (265, 275)

roi_brake = [0, 0, 160, 10];

roi_s = 10 #ROI start point
roi_num = 10 #Number of ROIs
roi_dist = 3 #Distance between each ROI
roi_array = [(0,roi_s,160,10)] * roi_num
for i in range(roi_num): #Creation of each ROI
    roi_array[i] = (0,roi_s + roi_dist * i,160,10)
dist_array = [0] * roi_num
center = 80

dist = lambda cb,cr: abs(cr-cb)
pin1 = Pin('P1', Pin.OUT_PP, Pin.PULL_NONE)

x_1 = 80
y_1 = 0
x_diff1 = 0
x_err1 = [0, 0, 0, 0]
dist2center = 0

# Configure camera
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()                # Create a clock object to track the FPS.

# Motor Control Values
Kp_s = 1.4
Kd_s = 0.014
Kp = Kp_s
Kd = Kd_s
max_pwm = 45
min_pwm = 35

# Brake Control
brake_counter = 0
S = 10

frame_timeout = 1000
frame_counter = frame_timeout

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
        if (cmd == "S"):
            S = value
        if (cmd =="max_pwm"):
            max_pwm = value
        if (cmd == "min_pwm"):
            min_pwm = value

        if (cmd == "servo_max"):
            servo_max = value
            sec = servo_max
        if (cmd == "servo_min"):
            servo_min = value
            sec = servo_min

    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image.

    ##### Blob Detection #####

    if (frame_counter > 0): #used for running start
            frame_counter -= 1
            print(frame_counter)

    # ROI Array
    for i in range(roi_num):
        blobs = img.find_blobs([thresholds], roi = roi_array[i], pixels_threshold=10, area_threshold=10)

        if (len(blobs) > 0): #if blobs are being detected
            if (len(blobs) == 3 and i == 0 and frame_counter == 0): #checking for stop point
                line1_dist = abs(blobs[1].cx() - blobs[0].cx()) #distance from center of the three blobs
                line2_dist = abs(blobs[1].cx() - blobs[2].cx())
                if (line1_dist < 30 and line1_dist > 10 and line2_dist < 30 and line2_dist > 10): #checks if the distance is within expected ranges
                    print(line1_dist)
                    print("3 lines detected")
                    enable = 0

            minblob = blobs[0] #initialize the minblob
            mindist = dist(minblob.cx(),x_1)

            for blob in blobs: #finding the minblob
                if (dist(blob.cx(), x_1) < mindist):
                    minblob = blob

            img.draw_cross(minblob.cx( ), minblob.cy(), color = 0)
            dist_array[i] = minblob.cx()

    blobs_brake = img.find_blobs([thresholds], roi = roi_brake, pixels_threshold=10, area_threshold=10)

    # Average
    x_1 = int(sum(dist_array) / len(dist_array)) #averaging the x positions of the 10 minblobs
    img.draw_cross(x_1, 120, color = 0)

    ##### MOTOR CONTROL #####

    #DIFFERENTIAL
    for i in range(3): #shifting index 2, 3, and 4 down
         x_err1[i] = x_err1[i+1]
    x_err1[3] = center - x_1 #update index 4 with new value
    x_diff1 = (x_err1[3] - x_err1[0] + (3*x_err1[2]) - (3*x_err1[1]))

    #PROPORTIONAL
    dist2center = center - x_1

    #DC MOTOR CONTROL
    if (enable == 0):
        frame_counter = frame_timeout
        servo_motor.set_range(max_pw=servo_max, min_pw=servo_min)
        dc_motor.brake_vcc()

    elif (enable == 1):

        # Braking Logic
        dc_motor.forward()

        if(len(blobs_brake) != 0):
            dutycyclePW =  max_pwm  - (abs(dist2center/center) * (max_pwm - min_pwm)) #DC Motor speed
            Kp = Kp_s
            Kd = Kd_s
            brake_counter = S
        else:
            if(brake_counter != 0):
                brake_counter -= 1
                dc_motor.reverse()
            else:
                dc_motor.forward()
                dutycyclePW =  min_pwm #DC Motor speed

            Kp = Kp_s*2.5
            Kd = Kd_s*2.5

    #SERVO MOTOR CONTROL
    sec = servo_center + servo_offset *( ((dist2center/center) * (Kp)) + (x_diff1 * Kd) ) #Servo Turn Magnitude

    if sec > servo_max: #prevent servo motor from overturning
        sec = servo_max
    if sec < servo_min:
        sec = servo_min

    if(abs(sec - sec_prev) < 0.00005):
        sec = sec_prev
    else:
        sec_prev = sec

    dc_motor.set_duty_cycle(dutycyclePW)
    servo_motor.set_pulse_width(sec)
