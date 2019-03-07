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
thresholds = (265, 275) #245 to 255

max_roi = 50
min_roi = 40
diff_roi = max_roi - min_roi
percent_change = 0

roi1 = (0, 0, 160, 10)

roi_s = 30
roi_num = 5
roi_dist = 7
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
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()                # Create a clock object to track the FPS.

## Control Values
# PID
Kp_min_s = 1.5 #2.5
Kp_max_s = 1.5
Kd_s = 0.02
max_pwm = 25
min_pwm = 25
# brake control
straight_counter = 0
brake_wait_counter = 0
brake_wait = 50
brake_counter = 0
brake = 10
turn_thres = 0.5

# manual control
command_str = "0"
dc_command = 0
ser_command = 0.00145 # value for straight

##### MAIN LOOP #####
while(True):
    if (bt.any()):
        dc_motor.brake_vcc()
        cmd, value = bt.get_cmd_value_blocking()
        if (cmd == "g"):
            enable = 1
        if (cmd == "x"):
            enable = 0

        if (cmd == "Kp_min"):
            Kp_min_s = value
        if (cmd == "Kp_max"):
            Kp_max_s = value
        if (cmd == "KD"):
            Kd_s = value

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

        if (cmd == "max_roi"):
            max_roi = value
            diff_roi = max_roi - min_roi
        if (cmd == "min_roi"):
            min_roi = value
            diff_roi = max_roi - min_roi

        if (cmd == "brake_wait"):
            brake_wait = value
        if (cmd == "brake"):
            brake = value
        if (cmd == "turn_thres"):
            turn_thres = value


    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image.


    ##### Blob Detection #####

    # dynamic ROI, more the wheels are turned, the closer the roi

    instant_change = abs((servo_center - sec)/servo_offset)
    #if (instant_change > .8):
        #percent_change += 0.015
    #else:
        #percent_change -= 0.015

    #if(percent_change > 1):
        #percent_change = 1
    #if(percent_change < 0):
        #percent_change = 0

    #blob_height = int( min_roi + diff_roi * (percent_change) )
    #roi1 = (0, blob_height, 160, 10)

    ## REGION1
    #blobs1 = img.find_blobs([thresholds], roi = roi1, pixels_threshold=10, area_threshold=10)
    #center = roi1[2]/2

    #if (len(blobs1) > 0):
        #if (len(blobs1) == 3 and (abs(blobs1[1].cx() - blobs1[0].cx()) < 30 and abs(blobs1[1].cx() - blobs1[2].cx()) < 30)):
            #dc_motor.brake_vcc()
            #print("3 lines detected")
            #enable = 0
        #else:
            #minblob1 = blobs1[0]
            #mindist1 = dist(minblob1.cx(),center)

            #for blob in blobs1:
                #if (dist(blob.cx(), center) < mindist1):
                    #minblob1 = blob

            #img.draw_rectangle(minblob1.rect(), color = 0)
            #img.draw_cross(minblob1.cx( ), minblob1.cy(), color = 0)
            #x_1 = minblob1.cx()
            #y_1 = minblob1.cy()
            #rect_1 = minblob1.rect()

    # Brake Roi
    for i in range(roi_num):
        blobs = img.find_blobs([thresholds], roi = roi_array[i], pixels_threshold=10, area_threshold=10)

        if (len(blobs) > 0):
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
    sec = servo_center + servo_offset*( ((dist2center/center) * (Kp_min_s + (Kp_max_s-Kp_min_s)*percent_change)) + (x_diff1 * Kd_s) )
    #sec = servo_center

    if sec > servo_max :
        sec = servo_max
    if sec < servo_min:
        sec = servo_min

    #DC MOTOR CONTROL
    if (enable == 0):
        servo_motor.set_range(max_pw=servo_max, min_pw=servo_min)
        dc_motor.brake_vcc()

    elif (enable == 1):
        dc_motor.forward()
        dutycyclePW =  max_pwm  - (abs(dist2center/center) * (max_pwm - min_pwm)) #DC

        #print(instant_change)
        #if instant_change > turn_thres and brake_wait_counter == 0:
            #brake_wait_counter = brake_wait
            #brake_counter = brake
        #elif brake_wait_counter > 0:
            #brake_wait_counter -= 1

        #if brake_counter > 0:
            #print("braking")
            #dutycyclePW = 70
            #dc_motor.brake_gnd()
            #brake_counter -= 1




    dc_motor.set_duty_cycle(dutycyclePW)
    servo_motor.set_pulse_width(sec)
