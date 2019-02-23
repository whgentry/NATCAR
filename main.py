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
dc_motor = DCMotor(tim_num=2, channel=4, frequency=500, pin="P5")
dc_motor.set_control(in_a="P2", in_b="P3", en_a="P4", en_b="P8")
dc_motor.set_current_sense(cs="P6", cs_dis="P9")
dutycyclePW = 0

## Servo motor control
servo_max = 0.00183
servo_min = 0.00102
servo_center = (servo_max + servo_min) / 2
servo_offset = servo_max - servo_center
servo_motor = ServoMotor(tim_num=4, channel=1, frequency=300, pin="P7")
servo_motor.set_range(max_pw=servo_max, min_pw=servo_min)
sec = servo_center

enable = 0

## Camera Control
thresholds = (265, 275) #245 to 255

max_roi = 90
min_roi = 20
diff_roi = max_roi - min_roi
percent_change = 0

roi1 = (0, 0, 160, 10)
roi2 = (0, 55, 160, 10)  ###### note changed to 90
roi3= (0, 110, 160, 10)
dist = lambda cb,cr: abs(cr-cb)
pin1 = Pin('P1', Pin.OUT_PP, Pin.PULL_NONE)
x_1 = 0
y_1 = 0
x_2 = 0
y_2 = 1
x_3 = 0
y_3 = 1
x_diff1 = 0
x_diff2 = 0
x_err1 = [0, 0, 0, 0]
x_err2 = [0, 0, 0, 0]
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
Kp_min_s = 2 #2.5
Kp_max_s = 2.5
Kd_s = 0.02
max_pwm = 18
min_pwm = 13
# brake control
brake_counter = 0
brake_pwm = ((max_pwm-min_pwm) * .50) + min_pwm
straight_counter = 0
# manual control
command_str = "0"
dc_command = 0
ser_command = 0.00145 # value for straight

# file writing
frame_count = 0
log_str = "Current Sense\n"
buff_size = 10
current_buff = [0] * buff_size

##### MAIN LOOP #####
while(True):
    if (bt.any()):
        dc_motor.brake_gnd()
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


    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image.


    ##### Blob Detection #####

    # dynamic ROI, more the wheels are turned, the closer the roi

    if (abs((servo_center - sec)/servo_offset) > .8):
        percent_change += 0.01
    else:
        percent_change -= 0.01

    if(percent_change > 1):
        percent_change = 1
    if(percent_change < 0):
        percent_change = 0

    #print(percent_change)


    #blob_height = int( min_roi + -diff_roi * (sqrt(1-percent_change**2) - 1) )
    blob_height = int( min_roi + diff_roi * (percent_change) )

    roi1 = (0, blob_height, 160, 10)

    # region 1
    blobs1 = img.find_blobs([thresholds], roi = roi1, pixels_threshold=10, area_threshold=10)
    center = roi1[2]/2

    if (len(blobs1) > 0):
        minblob1 = blobs1[0]
        mindist1 = dist(minblob1.cx(),center)

        for blob in blobs1:
            if (dist(blob.cx(), center) < mindist1):
                minblob1 = blob

        img.draw_rectangle(minblob1.rect(), color = 0)
        img.draw_cross(minblob1.cx( ), minblob1.cy(), color = 0)
        x_1 = minblob1.cx()
        y_1 = minblob1.cy()
        rect_1 = minblob1.rect()

    ##### Servo and DC motor control #####
    if (enable == 0):
        servo_motor.set_range(max_pw=servo_max, min_pw=servo_min)
        dc_motor.brake_vcc()

    elif (enable == 1):
        dc_motor.forward()

        if (len(blobs2) == 0 or len(blobs3) == 0 or len(blobs1) == 0): # this might help for higher speeds, uses the last dist2center if off track to correct
            if(dist2center > 0):
                dist2center = center
            else:
                dist2center = -center

        # update differential array
        for i in range(3):
             x_err1[i] = x_err1[i+1]
        x_err1[3] = center - x_1 #dist2center #chaning to dist2center makes it follow derivative of far line in a straight away!!

        # calculate differential value
        x_diff1 = (x_err1[3] - x_err1[0] + (3*x_err1[2]) - (3*x_err1[1]))  # /6

        for i in range(3):
             x_err2[i] = x_err2[i+1]
        x_err2[3] = center - x_2 #dist2center #chaning to dist2center makes it follow derivative of far line in a straight away!!

        # calculate differential value
        x_diff2 = (x_err2[3] - x_err2[0] + (3*x_err2[2]) - (3*x_err2[1]))  # /6


        ############## U(t) ###############

        dist2center = center - x_1
        sec = servo_center + servo_offset*( ((dist2center/center) * (Kp_min_s + (Kp_max_s-Kp_min_s)*percent_change)) + (x_diff1 * Kd_s) )
        if sec > servo_max :
            sec = servo_max
        if sec < servo_min:
            sec = servo_min
        # calculate duty cycle
        dutycyclePW =  max_pwm # - (abs(dist2center/center) * (max_pwm - min_pwm)) #DC


        for i in range(buff_size-1):
             current_buff[i] = current_buff[i+1]
        current_buff[buff_size - 1] = dc_motor.get_current()

        average = 0
        for val in current_buff:
            average += val
        average /= buff_size

        log_str += str(average) + '\n'
        if (frame_count > 1000):
            dc_motor.brake_vcc()
            log = open("log.csv","w")
            log.write(log_str)
            log.close()
            while(True):
                dc_motor.brake_vcc()

        frame_count += 1

    # set the DC duty cycle
    #dc_motor.set_duty_cycle(0)
    dc_motor.set_duty_cycle(dutycyclePW)
    # setthe servo pusle width
    servo_motor.set_pulse_width(sec)
