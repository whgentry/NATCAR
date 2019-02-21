# LAB3
# Jimmy Hoang
# William Gentry
# Daniel Burr

import sensor, image, math, time, pyb, motors, bluetooth
from pyb import Pin, Timer, UART, LED, ADC
from motors import DCMotor, ServoMotor
from bluetooth import Bluetooth

##### Global Variables and Initialization #####
## uart
uart = UART(1, 115200)
bt = Bluetooth(uart)

## DC motor control
dc_motor = DCMotor(tim_num=2, channel=1, frequency=100, pin="P6")
dc_motor.set_control(in_a="P2", in_b="P3", en_a="P4", en_b="P5")

## Servo motor control
servo_motor = ServoMotor(tim_num=4, channel=1, frequency=300, pin="P7")
servo_motor.set_range(max_pw=0.00187, min_pw=0.0011)

sec = 0.0015 #initialize seconds value goes from 0.0011 to 0.0019
dutycyclePW = 0
enable = 0

## Camera Control
thresholds = (265, 275) #245 to 255
roi1 = (0, 40, 160, 10)
roi2 = (0, 80, 160, 10)  ###### note changed to 90
roi3= (0, 10, 160, 10)
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
Kp_s = 3
Kd_s = 0.02
max_pwm = 18
min_pwm = 12
# brake control
brake_counter = 0
brake_pwm = ((max_pwm-min_pwm) * .50) + min_pwm
straight_counter = 0
# manual control
command_str = "0"
dc_command = 0
ser_command = 0.00145 # value for straight

# file write test
frame_count = 0
log_str = "Derivative\n"

##### MAIN LOOP #####
while(True):
    if (bt.any()):
        dc_motor.brake_gnd()
        cmd, value = bt.get_cmd_value_blocking()
        if (cmd == "g"):
            enable = 1
        if (cmd == "x"):
            enable = 0

    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image.


    ##### Blob Detection #####
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

    # region 2
    blobs2 = img.find_blobs([thresholds], roi = roi2, pixels_threshold=10, area_threshold=10)
    center = roi2[2]/2

    if (len(blobs2) > 0):
        minblob2 = blobs2[0]
        mindist2 = dist(minblob2.cx(),center)

        for blob in blobs2:
            if (dist(blob.cx(), center) < mindist2):
                minblob2 = blob

        img.draw_rectangle(minblob2.rect(), color = 0)
        img.draw_cross(minblob2.cx(), minblob2.cy(), color = 0)
        x_2 = minblob2.cx()
        y_2 = minblob2.cy()
        rect_2 = minblob2.rect()

    blobs3 = img.find_blobs([thresholds], roi = roi3, pixels_threshold=10, area_threshold=10)
    center = roi3[2]/2

    if (len(blobs3) > 0):
        minblob3 = blobs3[0]
        mindist3 = dist(minblob3.cx(),center)

        for blob in blobs3:
            if (dist(blob.cx(), center) < mindist3):
                minblob3 = blob

        img.draw_rectangle(minblob3.rect(), color = 0)
        img.draw_cross(minblob3.cx(), minblob3.cy(), color = 0)
        x_3 = minblob3.cx()
        y_3 = minblob3.cy()
        rect_3 = minblob3.rect()


    ##### Servo and DC motor control #####
    if (enable == 0):
        sec = 0.00145
        dc_motor.brake_gnd()

    elif (enable == 1):

        dc_motor.forward()

        if (len(blobs1) == 0 or len(blobs2) == 0): # this might help for higher speeds, uses the last dist2center if off track to correct
            if(dist2center > 0):
                dist2center = center
            else:
                dist2center = -center
        #print("x_1, \n", x_2)
        #print("x_2, \n", x_1)

        # update differential array
        for i in range(3):
             x_err1[i] = x_err1[i+1]
        x_err1[3] = center - x_1 #dist2center #chaning to dist2center makes it follow derivative of far line in a straight away!!

        # calculate differential value
        x_diff1 = (x_err1[0] - x_err1[3] + (3*x_err1[1]) - (3*x_err1[2]))  # /6
        #print(x_diff1)

        #for i in range(3):
             #x_err2[i] = x_err2[i+1]
        #x_err2[3] = center - x_3 #dist2center #chaning to dist2center makes it follow derivative of far line in a straight away!!

        ## calculate differential value
        #x_diff2 = (x_err2[0] - x_err2[3] + (3*x_err2[1]) - (3*x_err2[2]))  # /6
        #print(x_diff2)

        # calculate angle1
        angle1 = math.atan((x_2 - x_1)/(y_2 - y_1))
        angle1 = math.degrees(angle1)
         #saturate angle
        if angle1 > 60:
            angle1 = 60
        if angle1 < -60:
            angle1 = -60

        # calculate angle2
        angle2 = math.atan((x_1 - x_3)/(y_1 - y_3))
        angle2 = math.degrees(angle2)
        #saturate angle
        if angle2 > 60:
           angle2 = 60
        if angle2 < -60:
           angle2 = -60

        ############## U(t) ###############

        # brake
        #if (brake_counter == 0):
            #dc_motor.foward()
        #else:
            #dc_motor.brake_gnd()
            #brake_counter -= 1

        ## straight bool
        #straight = abs(center-x_2) < 10 and (abs(center-x_1) < 15) and (abs(center-x_3) < 20)

        ## straight count
        #if straight and dutycyclePW > brake_pwm:
            #straight_counter += 1
            #print(straight_counter)
        #elif straight_counter > 40:
            #straight_counter = 0
            #brake_counter = 30
        #else:
            #staight_counter = 0

        # check multiple blobs on further roi
        if(len(blobs1) > len(blobs2)):
            dist2center = center - x_2
            sec = 0.001485 + 0.000385*( ((dist2center/center) * Kp_s) +  (x_diff1 * -Kd_s) )
        else:
            dist2center = (center - x_1)
            sec = 0.001485 + 0.000385*( ((dist2center/center) * Kp_s) +  (x_diff1 * -Kd_s) )

        # calculate duty cycle
        dutycyclePW =  max_pwm  - ((abs(angle2)/60) * (max_pwm - min_pwm)) #DC

        log_str += str(x_diff1) + "\n"
        if (frame_count > 1000):
            dc_motor.brake_gnd()
            log = open("log.csv","w")
            log.write(log_str)
            log.close()
            while(True):
                dc_motor.set_duty_cycle(0)
                dc_motor.brake_gnd()

        frame_count += 1

    # set the DC duty cycle
    #dc_motor.set_duty_cycle(0)
    dc_motor.set_duty_cycle(dutycyclePW)
    # setthe servo pusle width
    servo_motor.set_pulse_width(sec)

