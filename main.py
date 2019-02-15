# LAB3
# Jimmy Hoang
# William Gentry
# Daniel Burr

import sensor, image, math, time, pyb, motor
from pyb import Pin, Timer, UART, LED, ADC
from motor import DCMotor

##### Global Variables and Initialization #####
## uart
uart = UART(1, 115200)
uart.write("AT\r\n") #checking connection
pyb.delay(200)
print(uart.readline())
uart.write("AT+VERSION\r\n") #checking version
pyb.delay(200)
print(uart.readline())
pyb.delay(200)
uart.write("AT+PSWD:\"3333\"\r\n") #setting pincode
pyb.delay(200)
print(uart.readline())
pyb.delay(200)
uart.write("AT+NAME=jimmywilldaniel\r\n") #setting the name
pyb.delay(200)
print(uart.readline())
pyb.delay(200)
uart.write("AT+UART=115200,0,0\r\n") #changing the baud rate to 115200
pyb.delay(200)
print(uart.readline())
pyb.delay(200)

## motor control
dc_motor = DCMotor(tim_num=2, channel=1, frequency=100, pin="P6")
dc_motor.set_control(in_a="P2", in_b="P3", en_a="P4", en_b="P5")

## PWM Control
timA = Timer(4, freq=300) # Frequency in Hz
#timB = Timer(2, freq=100) # DC motor
sec = 0.0015 #initialize seconds value goes from 0.0011 to 0.0019
dutycyclePW = 0
sourcefA = Timer.source_freq(timA)#declaration of source clock/prescaler values
prescalerA = Timer.prescaler(timA)
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
y_3 = 0
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
Kp_s = 2.5
Kd_s = 0.02
max_pwm = 30
min_pwm = 15
# brake control
brake_counter = 0
brake_pwm = ((max_pwm-min_pwm) * .50) + min_pwm
straight_counter = 0
# manual control
command_str = "0"
dc_command = 0
ser_command = 0.00145 # value for straight



##### MAIN LOOP #####
while(True):
    if (uart.any()):#for any uart input
        command_str = ""
        current_char = uart.readchar()#check, store, and output the character
        while (current_char != 13): # newline character
            # check for stop
            if (current_char == 120):
                enable = 0
                dc_motor.brake_gnd()
                break
            if (current_char != -1):
                command_str += chr(current_char)
            current_char = uart.readchar()

        #print(command_str)
        if (len(command_str) > 1):
            if (command_str[0] == "d"):
                try:
                    command_str = command_str[1:]
                    dc_command = int(command_str)
                except ValueError :
                    print("You messed up")
            elif (command_str[0] == "s"):
                try:
                    command_str = command_str[1:]
                    ser_command = float(command_str)/100000
                except ValueError :
                    print("You messed up")
            if (command_str[0] == "x"):
                enable = 0
            if (command_str[0] == "g"):
                enable = 1

    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image.
    #pin1.value(not pin1.value())

    if (enable == 0):
        # convert angle to pulse width
        #if (ser_command < 0.0019 or ser_command > 0.0011):
            #sec = ser_command
        #else:
        sec = 0.00145

        #print(ser_command)

        dutycyclePW = 0

    elif (enable == 1):


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
        if (brake_counter == 0):
            dc_motor.foward()
        else:
            dc_motor.brake_gnd()
            brake_counter -= 1

        # straight bool
        straight = abs(center-x_2) < 10 and (abs(center-x_1) < 15) and (abs(center-x_3) < 20)

        # straight count
        if straight and dutycyclePW > brake_pwm:
            straight_counter += 1
            print(straight_counter)
        elif straight_counter > 40:
            straight_counter = 0
            brake_counter = 30
        else:
            staight_counter = 0

        # check multiple blobs on further roi
        if(len(blobs1) > len(blobs2)):
            dist2center = center - x_2
            sec = 0.001485 + 0.000385*( ((dist2center/center) * Kp_s) +  (x_diff1 * -Kd_s) )
        #elif (abs(center-x_2 < 10) and abs(center-x_1) and abs(center-x_3)) and straight_counter > 100:
            #dist2center = (center - x_3)
            #sec = 0.001485 + 0.000385*( ((dist2center/center) * Kp_s) +  (x_diff1 * -Kd_s) )
        else:
            dist2center = (center - x_1)
            sec = 0.001485 + 0.000385*( ((dist2center/center) * Kp_s) +  (x_diff1 * -Kd_s) )


        #+ (x_diff1/10)) #(angle/60) # Servo

        # calculate duty cycle
        #dutycyclePW =  max_pwm  - ((abs(dist2center)/center) * (max_pwm - min_pwm))
        dutycyclePW =  max_pwm  - ((abs(angle2)/60) * (max_pwm - min_pwm)) #DC

        if(sec > 0.00187):
            sec = 0.00187
        elif(sec < 0.0011):
            sec = 0.0011

    # Generate a 300Hz square wave on TIM4 and with pw as pulse_width
    #sec = .0011
    pwA = sec*sourcefA/(prescalerA+1) #This is the conversion for pulsewidth
    #chB = timB.channel(4, Timer.PWM, pin=Pin("P5"), pulse_width_percent=dutycyclePW)
    dc_motor.set_duty_cycle(dutycyclePW)
    chA = timA.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width=round(pwA)) #setting up the channel for oscilloscope


# test commit

#if (motor == 1):# and command_str[0] != "x"):
        # Generate a 100Hz square wave on TIM2
        #angle = math.atan((x_2 - x_1)/(y_2 - y_1))
        #angle = math.degrees(angle)
        # saturate angle
        #if angle > 60:
        #    angle = 60
        #if angle < -60:
        #    angle = -60
