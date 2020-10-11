# -*- coding:utf-8 -*-
import smbus
import time
import math

import RPi.GPIO as GPIO
import wiringpi2 as wiringpi

#accelartionSetting
I2C_ADDR=0x1D
# Get I2C bus
bus = smbus.SMBus(1)
# Select Control register, 0x2A(42)
#               0x00(00)        StandBy mode
bus.write_byte_data(I2C_ADDR, 0x2A, 0x00)
# Select Control register, 0x2A(42)
#               0x01(01)        Active mode
bus.write_byte_data(I2C_ADDR, 0x2A, 0x01)
# Select Configuration register, 0x0E(14)
#               0x00(00)        Set range to +/- 2g
bus.write_byte_data(I2C_ADDR, 0x0E, 0x00)
time.sleep(0.5)

#servoMotorSetting
PWM18 =18
interval = float( 2.5 )
upper_pulse = float( 2.4 )
under_pulse = float( 0.5 )
range = 1024
duty_ratio = upper_pulse / interval
hz = 1 / ( interval * 0.001 )
clock = int( 18750 / hz )
duty = int( duty_ratio * range )

print("pin = ", PWM18, " interval[ms] = ", interval, " upper_pulse[ms] = ", upper_pulse )
print("clock = ", clock, " duty=", duty, " duty_ratio=", duty_ratio )
# 初期設定
wiringpi.wiringPiSetupGpio()
wiringpi.pinMode( PWM18, wiringpi.GPIO.PWM_OUTPUT )
wiringpi.pwmSetMode( wiringpi.GPIO.PWM_MODE_MS )

# ClockとDutyを設定してPWMを生成する
wiringpi.pwmSetClock( clock )
wiringpi.pwmWrite( PWM18, duty )
wiringpi.pwmSetRange(range)

#max min値を計算
min_duty=int((under_pulse/interval)*range*1.1)
max_duty=int((upper_pulse - under_pulse)/interval*range + min_duty*.9)
print("min_duty = ", min_duty, " max_duty=", max_duty)
value = min_duty

def DagToVal(dag):
    if dag < 0:
        dag =0
    if dag >180:
        dag =180
    par_dag = (max_duty-min_duty)/180.0
    return int(dag * par_dag + min_duty)

if __name__ == "__main__":
    try:

        while True:
            data = bus.read_i2c_block_data(I2C_ADDR, 0x00, 7)
            xAccl = (data[1] * 256 + data[2]) / 16
            if xAccl > 2047 :
                xAccl -= 4096
            yAccl = (data[3] * 256 + data[4]) / 16
            if yAccl > 2047 :
                yAccl -= 4096
            zAccl = (data[5] * 256 + data[6]) / 16
            if zAccl > 2047 :
                zAccl -= 4096
            
            yzMinus =yAccl if math.fabs(yAccl) > math.fabs(zAccl) else zAccl
            yzMinus =1 if yzMinus >0 else -1
            xzMinus =xAccl if math.fabs(xAccl) > math.fabs(zAccl) else zAccl
            xzMinus =1 if xzMinus >0 else -1
            xyMinus =xAccl if math.fabs(xAccl) > math.fabs(yAccl) else yAccl
            xyMinus =1 if xyMinus >0 else -1
            
            Accl = math.sqrt(math.pow(xAccl, 2) +math.pow(yAccl, 2) +math.pow(yAccl, 2) )
            xDag = math.degrees(math.atan2(xAccl ,yzMinus *math.sqrt(yAccl *yAccl +zAccl *zAccl)))
            yDag = math.degrees(math.atan2(yAccl ,xzMinus *math.sqrt(xAccl *xAccl +zAccl *zAccl)))
            zDag = math.degrees(math.atan2(zAccl ,xyMinus *math.sqrt(xAccl *xAccl +yAccl *yAccl)))
            print( "X,Y,Z-accele:({0:3.2f},{1:3.2f},{2:3.2f})".format(xAccl, yAccl, zAccl))
            print( "X,Y,Z-dagras:({0:3.2f},{1:3.2f},{2:3.2f})".format(xDag, yDag, zDag))
            
            value =DagToVal(xDag)
            print ("value:",value)
            wiringpi.pwmWrite(PWM18, value)
            time.sleep(0.1)
    except KeyboardInterrupt:
        wiringpi.pinMode(PWM18, wiringpi.GPIO.INPUT)
        pass
