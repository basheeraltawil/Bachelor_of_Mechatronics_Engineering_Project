# RoboCraft / graduation-project source
# Extracted from: codesappendix.pdf
# Source status: legacy appendix extraction; not independently hardware-tested here.
# The original thesis uses Raspberry Pi + Python, Arduino/ATmega, I2C, and DC-motor control.
# Review pin assignments, calibration constants, dependencies, and safety limits before use.

import numpy as np
import sympy
import math as m
import matplotlib.pyplot as plt
from pylab import *
import time
import serial
import Rpi.GPIO as GPIO
import smbus
bus = smbus.SMBus(1)
address = 0x04 #ADRESS CAN BE CHANGE
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
sevo1=17
servo2=18
servo3=19
                                  ##HEXAGONAL CORNERS CORDINATIONS FINDING START
GPIO.setup(servo1, GPIO.OUT) # setting servo pin for first manipulator(gripper1)
GPIO.setup(servo2, GPIO.OUT) # setting servo pin for second manipulator(gripper2)
GPIO.setup(servo3, GPIO.OUT) # setting servo pin for third manipulator(gripper3)
def setServoAngle(servo, angle):



pwm.start(8)

dutyCycle = angle / 18. + 2.

pwm.ChangeDutyCycle(dutyCycle)


sleep(0.3)

pwm.stop()
def writeNumber(a,b):
    bus.write_i2c_block_data(address, a, [b])
    return -1
                                                          ###HEXAGONAL CODE START HERE##
##CENTER AND FIRST CIRCLE
xc1, yc1 = sympy.symbols("xc1  yc1", real=True)
xx1=140;yy1=80;xx2=80;yy2=80
C1E=(xc1-xx1)**2+(yc1-yy1)**2
C2E=(xc1-xx2)**2+(yc1-yy2)**2
RR=60**2
E1Q1=sympy.Eq(C1E,RR)
E1Q2=sympy.Eq(C2E,RR)
SOLUTION1=sympy.solve([E1Q1,E1Q2])
print(SOLUTION1)
SS1=SOLUTION1[0]
SS2=SOLUTION1[1]
VV1I=[list(SS1.values()),list(SS2.values())]##values for both solution
xc10=float(VV1I[0][0])  ##X-FIRST CORDINATE
yc10=float(VV1I[0][1])  ##Y-FIRST CORDINATE
xc20=float(VV1I[1][0])  ##X-SECOND CORDINATE
yc20=float(VV1I[1][1])  ##Y-SECOND CORDINATE
print(xc10,yc10,xc20,yc20)
##CENTER AND SECOND CIRCLE
xc2, yc2 = sympy.symbols("xc2  yc2", real=True)
xx12=140;yy12=80;xx22=200;yy22=80
C1E2=(xc2-xx12)**2+(yc2-yy12)**2
C2E2=(xc2-xx22)**2+(yc2-yy22)**2

RR=60**2
E2Q1=sympy.Eq(C1E2,RR)
E2Q2=sympy.Eq(C2E2,RR)
SOLUTION2=sympy.solve([E2Q1,E2Q2])
print(SOLUTION2)
S2S1=SOLUTION2[0]
S2S2=SOLUTION2[1]
VV2I=[list(S2S1.values()),list(S2S2.values())]##values for both solution
xc11=float(VV2I[0][0])  ##X-THIRD CORDINATE
yc11=float(VV2I[0][1])  ##Y-THIRD CORDINATE
xc21=float(VV2I[1][0])  ##X-FOURTH CORDINATE
yc21=float(VV2I[1][1])  ##Y-FOURTH CORDINATE
print(xc11,yc11,xc21,yc21)
xx1c=xx1-60
yy1c=yy1
xx2c=xx1+60
yy2c=yy1
print(xx1c,yy1c)
print(xx2c,yy2c)
                              ###HEXAGONAL CODE FINISH HERE##
                             ###FORWARD KINEMATIC CODE START HERE##
##forward kinametic for FIRST MANIPULATOR
a1=230;b1=230;XF1=50;YF1=40;TH1F1=20;TH1F2=330
XE1=XF1+b1*m.cos((TH1F1+TH1F2)*m.pi/180)+a1*m.cos(TH1F1*m.pi/180); #tethas coming from Pic
YE1=YF1+b1*m.sin((TH1F1+TH1F2)*m.pi/180)+a1*m.sin(TH1F1*m.pi/180);
##forward kinametic for  SECOND MANIPULATOR
a2=230;b2=230;XF2=10;YF2=10;TH2F1=20;TH2F2=330
XE2=XF2+b2*m.cos((TH2F1+TH2F2)*m.pi/180)+a2*m.cos(TH2F1*m.pi/180);
YE2=YF2+b2*m.sin((TH2F1+TH2F2)*m.pi/180)+a2*m.sin(TH2F1*m.pi/180);

##forward kinametic for  THIRD MANIPULATOR
a3=230;b3=230;XF3=-50;YF3=100;TH3F1=20;TH3F2=330
XE3=XF3+b3*m.cos((TH3F1+TH3F2)*m.pi/180)+a3*m.cos(TH3F1*m.pi/180);
YE3=YF3+b3*m.sin((TH3F1+TH3F2)*m.pi/180)+a3*m.sin(TH3F1*m.pi/180);
print(XE3,YE3)
                            ##THE END OF FORWARD
                        ###ROTATİON TASK CODE START HERE##
for x in range(6):
#INVERSE FOR FINAL POINT OF FİRST MANİPULATOR
    aa=230;bb=230  #links length
    AFM =-2*aa*xc11;
    BFM =-2*aa*yc11;
    CFM= xc11**2 +aa**2 + yc11**2 -bb**2;
    th10h=2*m.atan2((-BFM -m.sqrt(BFM**2 + AFM**2 - CFM**2)),(CFM -AFM))*(180/m.pi);
    th20h=(m.atan2(yc11-aa*m.sin(th10h*m.pi/180),xc11-aa*m.cos(th10h*m.pi/180))*180/m.pi)-th10h;
    #th10h ve th20h Pic'e yollanacak raspberryden !!!!
    while True:
        try:
            writeNumber(th10h,th20h)
            time.sleep(1)                    #delay one second

        except KeyboardInterrupt:
            quit()

    #INVERSE kinematic FOR FINAL POINT OF SECOND MANİPULATOR
    aa=230;bb=230  #links length
    ASM =-2*aa*xc21;
    BSM =-2*aa*yc21;
    CSM= xc21**2 +aa**2 + yc21**2 -bb**2;

    th11h=2*m.atan2((-BSM -m.sqrt(BSM**2 + ASM**2 - CSM**2)),(CSM -ASM))*(180/m.pi);
    th21h=(m.atan2(yc11-aa*m.sin(th11h*m.pi/180),xc21-aa*m.cos(th11h*m.pi/180))*180/m.pi)-th11h;
    #th11h ve th21h Pic'e yollanacak raspberryden
    while True:
        try:
            writeNumber(th11h,th21h)
            time.sleep(1)                    #delay one second

        except KeyboardInterrupt:
            quit()

    #INVERSE FOR FINAL POINT OF THİRD MANİPULATOR
    aa=230;bb=230  #links length
    ATM =-2*aa*xc20;
    BTM =-2*aa*yc20;
    CTM= xc20**2 +aa**2 + yc20**2 -bb**2;
    th12h=2*m.atan2((-BTM -m.sqrt(BTM**2 + ATM**2 - CTM**2)),(CTM -ATM))*(180/m.pi);
    th22h=(m.atan2(yc20-aa*m.sin(th12h*m.pi/180),xc20-aa*m.cos(th12h*m.pi/180))*180/m.pi)-th12h;
    #th12h ve th22h Pic'e yollanacak raspberryden !!!!
    while True:
        try:
            writeNumber(th12h,th22h)
            time.sleep(1)                    #delay one second

        except KeyboardInterrupt:
            quit()
    if __name__ == '__main__':
        setServoAngle(servo3, 180) #180 degree suppose gripper is closing at that angle


    GPIO.cleanup()

    #INVERSE FOR FINAL POINT OF FİRST MANİPULATOR GOİNG BACK
    if __name__ == '__main__':
        setServoAngle(servo1, 0) #0 degree suppose gripper is opening at that angle
    GPIO.cleanup()
    aa=230;bb=230  #links length
    AFM =-2*aa*xc10;
    BFM =-2*aa*yc10;
    CFM= xc10**2 +aa**2 + yc10**2 -bb**2;
    th10h=2*m.atan2((-BFM -m.sqrt(BFM**2 + AFM**2 - CFM**2)),(CFM -AFM))*(180/m.pi);
    th20h=(m.atan2(yc10-aa*m.sin(th10h*m.pi/180),xc10-aa*m.cos(th10h*m.pi/180))*180/m.pi)-th10h;
    #th10h ve th20h Pic'e yollanacak raspberryden!!!!
    while True:
        try:
            writeNumber(th10h,th20h)
            time.sleep(1)                    #delay one second

        except KeyboardInterrupt:
            quit()
    if __name__ == '__main__':
        setServoAngle(servo1, 180) #180 degree suppose gripper is closing at that angle
    GPIO.cleanup()
    #INVERSE FOR FINAL POINT OF SECOND MANİPULATOR GOİNG BACK
    if __name__ == '__main__':
        setServoAngle(servo2, 0) #0 degree suppose gripper is opening at that angle

    GPIO.cleanup()


    aa=230;bb=230  #links length
    ASM =-2*aa*xx2c;
    BSM =-2*aa*yy2c;
    CSM= xx2c**2 +aa**2 + yy2c**2 -bb**2;
    th11h=2*m.atan2((-BSM -m.sqrt(BSM**2 + ASM**2 - CSM**2)),(CSM -ASM))*(180/m.pi);
    th21h=(m.atan2(yy2c-aa*m.sin(th11h*m.pi/180),xx2c-aa*m.cos(th11h*m.pi/180))*180/m.pi)-th11h;
    #th11h ve th21h Pic'e yollanacak raspberryden !!!!
    while True:
        try:
            writeNumber(th11h,th21h)
            time.sleep(1)                    #delay one second

        except KeyboardInterrupt:
            quit()
    if __name__ == '__main__':
        setServoAngle(servo2, 180) #180 degree suppose gripper is closing at that angle

    GPIO.cleanup()

    #INVERSE FOR FINAL POINT OF THİRD MANİPULATOR
    if __name__ == '__main__':
        setServoAngle(servo3, 0) #0 degree suppose gripper is opening at that angle

    GPIO.cleanup()
    aa=230;bb=230  #links length
    ATM =-2*aa*XE3;
    BTM =-2*aa*YE3;
    CTM= XE3**2 +aa**2 + YE3**2 -bb**2;
    th12h=2*m.atan2((-BTM -m.sqrt(BTM**2 + ATM**2 - CTM**2)),(CTM -ATM))*(180/m.pi);
