# Bluetooth control

import sensor, image, math, time, pyb
from pyb import Pin, Timer, UART, LED, ADC

class Bluetooth:
    def __init__(self, uart_config):
        self.uart = uart_config
        #self.uart.write("AT\r\n") #checking connection
        #pyb.delay(200)
        #print(self.uart.readline())
        #self.uart.write("AT+VERSION\r\n") #checking version
        #pyb.delay(200)
        #print(self.uart.readline())
        #pyb.delay(200)
        #self.uart.write("AT+PSWD:\"3333\"\r\n") #setting pincode
        #pyb.delay(200)
        #print(self.uart.readline())
        #pyb.delay(200)
        #self.uart.write("AT+NAME=jimmywilldaniel\r\n") #setting the name
        #pyb.delay(200)
        #print(self.uart.readline())
        #pyb.delay(200)
        #self.uart.write("AT+UART=115200,0,0\r\n") #changing the baud rate to 115200
        #pyb.delay(200)
        #print(self.uart.readline())
        #pyb.delay(200)

    def any(self):
        return self.uart.any()

    def get_cmd_value_blocking(self):
        command_str = ""
        cmd = ""
        value = -1

        command_str = str(self.uart.readline(), "utf-8")
        cmd_array = command_str.split(" ")

        if len(cmd_array) == 1:
            cmd =  cmd_array[0]
        elif len(cmd_array) > 1:
            cmd = cmd_array[0]
            try:
                value = float(cmd_array[1])
            except ValueError :
                value = -1

        return cmd, value








#if (uart.any()):#for any uart input
    #command_str = ""
    #current_char = uart.readchar()#check, store, and output the character
    #while (current_char != 13): # newline character
        ## check for stop
        #if (current_char == 120):
            #enable = 0
            #dc_motor.brake_gnd()
            #break
        #if (current_char != -1):
            #command_str += chr(current_char)
        #current_char = uart.readchar()

    ##print(command_str)
    #if (len(command_str) > 1):
        #if (command_str[0] == "d"):
            #try:
                #command_str = command_str[1:]
                #dc_command = int(command_str)
            #except ValueError :
                #print("You messed up")
        #elif (command_str[0] == "s"):
            #try:
                #command_str = command_str[1:]
                #ser_command = float(command_str)/100000
            #except ValueError :
                #print("You messed up")
        #if (command_str[0] == "x"):
            #enable = 0
        #if (command_str[0] == "g"):
            #enable = 1
