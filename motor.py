# Motor Control Modules
# Contains classes for motor, dc and servo

import sensor, image, math, time, pyb
from pyb import Pin, Timer, UART, LED, ADC

# Class for DC Motor
class DCMotor:
    def __init__(self, tim_num, channel, frequency, pin):
        self.tim = Timer(tim_num, freq=frequency)
        self.pin = Pin(pin)
        self.channel = channel

    def set_control(self, in_a, in_b, en_a, en_b):
        self.INA = Pin(in_a, Pin.OUT_PP)
        self.INB = Pin(in_b, Pin.OUT_PP)
        self.ENA = Pin(en_a, Pin.OUT_PP)
        self.ENB = Pin(en_b, Pin.OUT_PP)
        self.ENA.high()
        self.ENB.high()

    def set_current_sense(self, cs, cs_dis):
        self.CS_DIS = Pin(cs_dis, Pin.OUT_PP)
        self.CS = Pin(cs, Pin.IN)
        self.current = ADC(self.CS)

    def get_current(self):
        return self.current.read()

    def set_duty_cycle(self, duty_cycle):
        self.tim.channel(self.channel, Timer.PWM, pin=self.pin, pulse_width_percent=duty_cycle)

    def foward(self):
        self.INA.low()
        self.INB.high()

    def reverse(self):
        self.INA.high()
        self.INB.low()

    def brake_gnd(self):
        self.INA.low()
        self.INB.low()

    def brake_vcc(self):
        self.INA.high()
        self.INB.high()


# Class for Servo Motor
class ServoMotor:
    def __init__(self, tim_num, channel, frequency, pin):
        self.tim = Timer(tim_num, freq=frequency)
        self.pin = Pin(pin)
        self.channel = channel
        self.scale = self.tim.source_freq()/(self.tim.prescaler()+1)

    def set_range(self, max_pw, min_pw):
        self.max_pw = max_pw
        self.min_pw = max_pw
        self.mid_pw = (max_pw + min_pw) / 2

    def saturate_pw(self, sec):
        if(sec > max_pw):
            sec = max_pw
        elif(sec < min_pw):
            sec = min_pw
        return sec

    def set_pulse_width(self, sec):
        sec = self.saturate_pw(sec)
        timA.channel(self.channel, Timer.PWM, pin=self.pin, pulse_width=round(sec * scale))
