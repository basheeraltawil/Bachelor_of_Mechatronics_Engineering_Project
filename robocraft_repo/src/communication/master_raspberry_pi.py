# RoboCraft / graduation-project source
# Extracted from: codesappendix.pdf
# Source status: legacy appendix extraction; not independently hardware-tested here.
# The original thesis uses Raspberry Pi + Python, Arduino/ATmega, I2C, and DC-motor control.
# Review pin assignments, calibration constants, dependencies, and safety limits before use.

bb23=('%0.0f'%aq23)
    #print(bb23)
    tt23.append(int(bb23))
    #time.sleep(1)
tm22s1=tt23  #list to be send for first segment of second motor
#print(tm22s1)
for aq24 in t22s2*100:
    bb24=('%0.0f'%aq24)
    ##print(bb4)
    tt24.append(int(bb24))
    #time.sleep(1)
tm22s2=tt24  #list to be send for second segment of second motor
#print(tm22s2)
print(tm21s1)
print(tm22s1)
print(tm21s2)
print(tm22s2)


# 2-Main code to communicate between Raspberry Pi and Atmega
microcontroller
# In this code we design a standard algorithm that will be used in our
communication between master and other slaves
# 2-a-Master code(Python)
 import sys
import smbus
import time
import numpy as np

import math as m
import matplotlib.pyplot as plt
from pylab import *
import time
import serial
import sympy
import RPi.GPIO as GPIO
from smbus2 import SMBus  #communication protocol that used in i2c
bus = smbus.SMBus(1)
address = 0x08 #communication address
def writeNumber(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,aa,bb,cc,dd,ii,jj,kk,ll,mm,nn,oo,pp,rr,ss,tt,uu):
    bus.write_i2c_block_data(address, a,[
b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,aa,bb,cc,dd,ii,jj,kk,ll,mm,nn,oo,pp,rr,ss,tt,uu]) ##the total ammount of data
that will be send to atmega
    return -1
def requestreading(): ##reading function definition
    block=bus.read_i2c_block_data(address,0,3) ##to read potontiometer values that will be taken from
arduino chip

    return block

p=1
while True:
    try:
        b=requestreading() ##data requesting from arduino
        p1=int(b[0]*4.0118) #to convet analog values of potontiometer 1 to its original value 1023
        tetha11=int(p1*(5/1023)*720) ##convert analog value to angle value
        p2=int(b[2]*4.0118)  #to convet analog values of potontiometer 1 to its original value 1023
        tetha12=int(p2*(5/1023)*720)   ##convert analog value to angle value
        print("p1 %s  p2 %s :"%(tetha11,tetha12)) ##to print those values into serial monitor

        ##foroward kinamtic  calculation
        a2=230;b2=230;TH2F1=(tetha11-1172); TH2F2=(tetha12-1158);
        XE2=b2*m.cos((TH2F1+TH2F2)*m.pi/180)+a2*m.cos(TH2F1*m.pi/180);
        YE2=b2*m.sin((TH2F1+TH2F2)*m.pi/180)+a2*m.sin(TH2F1*m.pi/180);
        #print(XE2,YE2)
                               ##starting point of trajectory generation
        tf21=8;tf22=8  #time period definition
        t=np.arange(0,8,1);   ##time sectioning slices definition for quadratic equation
        #x2=80;y2=200 #second station of end effector
        a2=230;b2=230  #links length
        #algorithm to obtain via point and end effector positions
        x22, y22 = sympy.symbols("x22 y22", real=True)  #to solve system equation
        x21=XE2;x23=0;y21=YE2;y23=321;#position w.r.t manipulator itself
        WW2=m.sqrt((y23-y21)**2+(x23-x21)**2)
        THE2=30      ##bending angle for via point inclination (it can be change)
        QQ21=(x22-x21)**2+(y22-y21)**2  ##first circle equ
        QQ22=(x22-x23)**2+(y22-y23)**2  ##second circle equ
        L2=(WW2/2)/(m.cos(THE2*m.pi/180))  ##midpoint of distance
        CC21=L2**2  ##radius of 1st circle
        CC22=L2**2  ##radius for 2nd circle
        eq21 = sympy.Eq(QQ21, CC21)  ##first eqation symboling
        eq22 = sympy.Eq(QQ22, CC22)    ##first eqation symboling
        solution2=sympy.solve([eq21, eq22])  ##system equations solution
        #print(solution2)
        S21=solution2[0]
        S22=solution2[1]
        VI2=[list(S21.values()),list(S22.values())]##values for both solution
        ##first solution
        x22=VI2[0][0]

        y22=VI2[0][1]
        ######via point has finish###
        ###INVERSE K FOR INTIAL POINT
        A2O =-2*a2*x21;
        B2O =-2*a2*y21;
        C2O = x21**2 +a2**2 + y21**2 -b2**2;
        th210=2*m.atan2((-B2O -m.sqrt(B2O**2 + A2O**2 - C2O**2)),(C2O - A2O))*(180/m.pi);  ##first
motor joint angle
        th220=(m.atan2(y21-a2*m.sin(th210*m.pi/180),x21-a2*m.cos(th210*m.pi/180))*180/m.pi)-th210;
##second motor joint angle
        #INVERSE K FOR via POINT
        AM2=-2*a2*x22;
        BM2=-2*a2*y22;
        CM2=x22**2 +a2**2 + y22**2 -b2**2;
        th211=2*m.atan2((-BM2 -m.sqrt(BM2**2 + AM2**2 - CM2**2)),(CM2 - AM2))*(180/m.pi);
        th221=(m.atan2(y22-a2*m.sin(th211*m.pi/180),x22-a2*m.cos(th211*m.pi/180))*180/m.pi)-th211;
        #INVERSE K FOR FINAL POINT
        AF2=-2*a2*x23;
        BF2=-2*a2*y23;
        CF2=x23**2 +a2**2 +y23**2-b2**2;
        th212=2*m.atan2((-BF2 -m.sqrt(BF2**2 + AF2**2 - CF2**2)),(CF2 -AF2))*(180/m.pi);
        th222=(m.atan2(y23-a2*m.sin(th212*m.pi/180),x23-a2*m.cos(th212*m.pi/180))*180/m.pi)-th212;
        #8 by 8 matrices FOR FIRST ACTUATOR that contains coifficients of perameters
        B21=np.array([[th210],[th211],[th211],[th212],[0],[0],[0],[0]]) #values of abgles obtained from
inverse kinematic 1.st actuator
        A21=np.array([[1 ,0 ,0, 0, 0 ,0, 0, 0],[1 ,tf21, tf21*tf21, tf21*tf21*tf21, 0, 0, 0 ,0],
            [0, 0 ,0 ,0 ,1 ,0, 0, 0 ],[0 ,0 ,0 ,0, 1, tf22, tf22*tf22, tf22*tf22*tf22],
            [0 ,1 ,0 ,0, 0 ,0, 0, 0 ],[0, 0, 0, 0, 0, 1, 2*tf22, 3*tf22*tf22],
            [0 ,1 ,2*tf21+3*tf21*tf21, 0, 0, -1, 0 ,0,] ,[0, 0, 2, 6*tf21, 0 ,0 ,-2, 0] ])
        A21N= np.linalg.inv(A21)

        C21=np.matmul(A21N,B21)
        #print(th10,th11,th12)
        t21s1=C21[0,0]+C21[1,0]*t+C21[2,0]*t**2+C21[3,0]*t**3; #first segment equation for 1.st act
        t21s2=C21[4,0]+C21[5,0]*t+C21[6,0]*t**2+C21[7,0]*t**3; #second segment equation  for 1st act
        #8 by 8 matrices FOR second ACTUATOR
        B22=np.array([[th220],[th221],[th221],[th222],[0],[0],[0],[0]]) #values of angles obtained from
inverse kinematic 1.st actuator
        A22=np.array([[1 ,0 ,0, 0, 0 ,0, 0, 0],[1 ,tf21, tf21*tf21, tf21*tf21*tf21, 0, 0, 0 ,0],
            [0, 0 ,0 ,0 ,1 ,0, 0, 0 ],[0 ,0 ,0 ,0, 1, tf22, tf22*tf22, tf22*tf22*tf22],
            [0 ,1 ,0 ,0, 0 ,0, 0, 0 ],[0, 0, 0, 0, 0, 1, 2*tf22, 3*tf22*tf22],
            [0 ,1 ,2*tf21+3*tf21*tf21, 0, 0, -1, 0 ,0,] ,[0, 0, 2, 6*tf21, 0 ,0 ,-2, 0] ])
        A2N=np.linalg.inv(A22)
        C22=np.matmul(A2N,B22)
        #print(th20,th21,th22)
        t22s1=C22[0,0]+C22[1,0]*t+C22[2,0]*t**2+C22[3,0]*t**3; #first segment equation for 1.st act
        t22s2=C22[4,0]+C22[5,0]*t+C22[6,0]*t**2+C22[7,0]*t**3; #second segment equation  for
        tt21=[];tt22=[];tt23=[];tt24=[]  #empty lists for thetas erray
        for aq21 in t21s1*100:
            bb21=('%0.0f'%aq21)
            #print(bb1)
            tt21.append(int(bb21))
            #tt1.append(int(bb21)/100)
            #time.sleep(1)
        tm21s1=tt21  #list to be send for first segment of first motor
        #print(tm21s1)
        for aq22 in t21s2*100:

            bb22=('%0.0f'%aq22)
            #print(bb22)

            tt22.append(int(bb22))
            #time.sleep(1)
        tm21s2=tt22  #list to be send for second segment of first motor
        #print(tm21s2)
        for aq23 in t22s1*100:
            bb23=('%0.0f'%aq23)
            #print(bb23)
            tt23.append(int(bb23))
            #time.sleep(1)
        tm22s1=tt23  #list to be send for first segment of second motor
        #print(tm22s1)
        for aq24 in t22s2*100:
            bb24=('%0.0f'%aq24)
            ##print(bb4)
            tt24.append(int(bb24))
            #time.sleep(1)
        tm22s2=tt24  #list to be send for second segment of second motor
        #print(tm22s2)

##data that will be send toward atmega
TK=[tm21s1[0]//255,tm21s1[1]//255,tm21s1[2]//255,tm21s1[3]//255,tm21s1[4]//255,tm21s1[5]//255,t
m21s1[6]//255,tm21s1[7]//255,tm22s1[0]//255,tm22s1[1]//255,tm22s1[2]//255,tm22s1[3]//255,tm22s
1[4]//255,tm22s1[5]//255,tm22s1[6]//255,tm22s1[7]//255]
TM=[tm21s1[0]%255,tm21s1[1]%255,tm21s1[2]%255,tm21s1[3]%255,tm21s1[4]%255,tm21s1[5]%255,t
m21s1[6]%255,tm21s1[7]%255,tm22s1[0]%255,tm22s1[1]%255,tm22s1[2]%255,tm22s1[3]%255,tm22s
1[4]%255,tm22s1[5]%255,tm22s1[6]%255,tm22s1[7]%255]
TK2=[tm21s2[0]//255,tm21s2[1]//255,tm21s2[2]//255,tm21s2[3]//255,tm21s2[4]//255,tm21s2[5]//255
,tm21s2[6]//255,tm21s2[7]//255,tm22s2[0]//255,tm22s2[1]//255,tm22s2[2]//255,tm22s2[3]//255,tm2
2s2[4]//255,tm22s2[5]//255,tm22s2[6]//255,tm22s2[7]//255]
TM2=[tm21s2[0]%255,tm21s2[1]%255,tm21s2[2]%255,tm21s2[3]%255,tm21s2[4]%255,tm21s2[5]%255
,tm21s2[6]%255,tm21s2[7]%255,tm22s2[0]%255,tm22s2[1]%255,tm22s2[2]%255,tm22s2[3]%255,tm22
s2[4]%255,tm22s2[5]%255,tm22s2[6]%255,tm22s2[7]%255]
writeNumber(TK[0],TK2[0],TK[1],TK2[1],TK[2],TK2[2],TK[3],TK2[3],TK[4],TK2[4],TK[5],TK2[5],TK[6],TK2[6]

,TK[7],TK2[7],TK[8],TK2[8],TK[9],TK2[9],TK[10],TK2[10],TK[11],TK2[11],TK[12],TK2[12],TK[13],TK2[13],TK[
14],TK2[14],TK[15],TK2[15])
writeNumber(TM[0],TM2[0],TM[1],TM2[1],TM[2],TM2[2],TM[3],TM2[3],TM[4],TM2[4],TM[5],TM2[5],T
M[6],TM2[6],TM[7],TM2[7],TM[8],TM2[8],TM[9],TM2[9],TM[10],TM2[10],TM[11],TM2[11],TM[12],TM2[
12],TM[13],TM2[13],TM[14],TM2[14],TM[15],TM2[15])
        print(tm21s1)
        print(tm22s1)
        print(tm21s2)
        print(tm22s2)
        time.sleep(0.5)                    #delay one second
        if p>0:
            break
    except KeyboardInterrupt:
        quit()
