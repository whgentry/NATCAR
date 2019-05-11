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

## Servo motor control
servo_max = 0.0019
servo_min = 0.0010
servo_center = (servo_max + servo_min) / 2
servo_offset = servo_max - servo_center
servo_motor = ServoMotor(tim_num=4, channel=1, frequency=300, pin="P7")
servo_motor.set_range(max_pw=servo_max, min_pw=servo_min)
sec = servo_center

enable = 0

## Camera Control
thresholds = (245, 255) #245 to 255

roi_far = (0,5,160,10)

roi_s = 30
roi_num = 10
roi_dist = 3
roi_array = [(0,roi_s,160,10)] * roi_num
for i in range(roi_num):
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
blobs1 = []
motor = 0
# Configure camera
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
sensor.set_auto_gain(False, gain_db=9) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()                # Create a clock object to track the FPS.

print(sensor.get_gain_db())

## Control Values
# PID#2.5
Kp_max_s = 2.0
Kp = Kp_max_s
Kd_s = 0.012
Kd = Kd_s
turbo_pwm = 50
max_pwm = 35
min_pwm = 35
# brake control
straight_counter = 0
brake_wait_counter = 0
brake_wait = 50
brake_counter = 0
brake = 10
turn_thres = 0.5
straight = 0
straight_counter = 0
straight_counter_far = 0
brake_counter = 0
brake_counter_far = 0
S = 40

frame_timeout = 1000
frame_counter = frame_timeout

no_blob = 0
no_brake= 0

# manual control
command_str = "0"
dc_command = 0
ser_command = 0.00145 # value for straight

##### MAIN LOOP #####
while(True):
    #print(clock.fps())
    if (bt.any()):
        dc_motor.brake_vcc()
        cmd, value = bt.get_cmd_value_blocking()
        if (cmd == "g"):
            enable = 1
        if (cmd == "x"):
            enable = 0

        if (cmd == "Kp_max"):
            Kp_max_s = value
        if (cmd == "KD"):
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

        if (cmd == "brake_wait"):
            brake_wait = value
        if (cmd == "brake"):
            brake = value
        if (cmd == "turn_thres"):
            turn_thres = value


    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image.
    #print(clock.fps())


    ##### Blob Detection #####
    instant_change = abs((servo_center - sec)/servo_offset)
    if (frame_counter > 0):
        frame_counter -= 1
        #print(frame_counter)

    # Normal Roi
    for i in range(roi_num):
        blobs = img.find_blobs([thresholds], roi = roi_array[i], pixels_threshold=10, area_threshold=10)

        if (len(blobs) > 0):
            if (len(blobs) == 3 and i == 0 and frame_counter == 0):
                line1_dist = abs(blobs[1].cx() - blobs[0].cx())
                line2_dist = abs(blobs[1].cx() - blobs[2].cx())
                if (line1_dist < 40 and line1_dist > 5 and line2_dist < 40 and line2_dist > 5):
                    print(line1_dist)
                    print("3 lines detected")
                    enable = 0

            minblob = blobs[0]
            mindist = dist(minblob.cx(),x_1)

            for blob in blobs:
                if (dist(blob.cx(), x_1) < mindist):
                    minblob = blob

            img.draw_cross(minblob.cx( ), minblob.cy(), color = 0)
            dist_array[i] = minblob.cx()

    x_1 = int(sum(dist_array) / len(dist_array))
    img.draw_cross(x_1, 120, color = 0)

    ##### SERVO MOTOR CONTROL #####

    #DIFFERENTIAL
    for i in range(3):
         x_err1[i] = x_err1[i+1]
    x_err1[3] = center - x_1 #dist2center #changing to dist2center makes it follow derivative of far line in a straight away!!
    x_diff1 = (x_err1[3] - x_err1[0] + (3*x_err1[2]) - (3*x_err1[1]))


    #PROPORTIONAL
    dist2center = center - x_1
    sec = servo_center + servo_offset*( ((dist2center/center) * (Kp)) + (x_diff1 * Kd) )
    #sec = servo_center

    if sec > servo_max :
        sec = servo_max
    if sec < servo_min:
        sec = servo_min

    #DC MOTOR CONTROL
    if (enable == 0):
        frame_counter = frame_timeout
        servo_motor.set_range(max_pw=servo_max, min_pw=servo_min)
        dc_motor.brake_vcc()

    elif (enable == 1):
        #dc_motor.forward()

        #if(brake_counter == 0 and brake_counter_far == 0):
        if(brake_counter == 0):
            dc_motor.forward()
            no_brake += 1
        else:
            dc_motor.brake_vcc()
            no_brake = 0

        if(brake_counter > 0):
            brake_counter -= 1

        #if(brake_counter_far > 0):
            #brake_counter_far -= 1

        # Close Brake
        straight = (abs(dist2center) < S )# S = 15 is pretty good...
        if straight:
            straight_counter += 1
        elif straight_counter > 60: #and (str_avg > .85) :
            straight_counter = 0
            brake_counter = 15
        else:
            straight_counter = 0

        # Far Brake
        blobs = img.find_blobs([thresholds], roi = roi_far, pixels_threshold=10, area_threshold=10)
        if (len(blobs) == 0 and no_blob < 100):
            no_blob += 1
        if (len(blobs) > 0):
            img.draw_cross(blobs[0].cx(), blobs[0].cy(), color = 0)
            no_blob = 0
            straight_counter_far += 1
        elif no_blob > 3 and straight_counter_far > 100 and no_brake > 200:
            straight_counter_far = 0
            brake_counter = 50
            no_blob = 0
            print("brake")

        dutycyclePW =  max_pwm  - (abs(dist2center/center) * (max_pwm - min_pwm)) #DC

        #Reduce PD gains if Striaght away and increase pwm
        if (straight_counter_far > 75 and no_brake > 200):
            Kp = 1
            Kd = 0
            #dutycyclePW = turbo_pwm
        else:
            Kp = Kp_max_s
            Kd = Kd_s

    dc_motor.set_duty_cycle(dutycyclePW)
    servo_motor.set_pulse_width(sec)
