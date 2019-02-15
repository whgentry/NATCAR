# Motor Control Modules
# Contains classes for motor, dc and servo

import sensor, image, math, time, pyb
from pyb import Pin, Timer, UART, LED, ADC

# PWM
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
