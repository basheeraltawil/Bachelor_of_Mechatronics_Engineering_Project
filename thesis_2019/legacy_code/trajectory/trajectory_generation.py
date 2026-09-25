# RoboCraft / graduation-project source
# Extracted from: codesappendix.pdf
# Source status: legacy appendix extraction; not independently hardware-tested here.
# The original thesis uses Raspberry Pi + Python, Arduino/ATmega, I2C, and DC-motor control.
# Review pin assignments, calibration constants, dependencies, and safety limits before use.

import sys                                                                ###main code for trajectory generation
import time
import numpy as np
import math as m
import matplotlib.pyplot as plt
from pylab import *
import time
import serial
import sympy
a2=230;b2=230;TH2F1=(1214-1172); TH2F2=(1228-1158);
XE2=b2*m.cos((TH2F1+TH2F2)*m.pi/180)+a2*m.cos(TH2F1*m.pi/180);
YE2=b2*m.sin((TH2F1+TH2F2)*m.pi/180)+a2*m.sin(TH2F1*m.pi/180);
print(XE2,YE2)                             ##THE END OF FORWARD
tf21=8;tf22=8  #time period definition
t=np.arange(0,8,1);# path in between points

#x2=80;y2=200 #second station of end effector
a2=230;b2=230  #links length
#algorithm to obtain via point and end effector positions
x22, y22 = sympy.symbols("x22 y22", real=True)  #to solve system equation

x21=XE2;x23=0;y21=YE2;y23=321;#position w.r.t manipulator itself
#print(x21,x23,y21,y23)
WW2=m.sqrt((y23-y21)**2+(x23-x21)**2)
#print(WW2)
THE2=30      ##bending angle (it can be change)
QQ21=(x22-x21)**2+(y22-y21)**2  ##first circle equ
QQ22=(x22-x23)**2+(y22-y23)**2  ##second circle equ
L2=(WW2/2)/(m.cos(THE2*m.pi/180))  ##midpoint of distance
CC21=L2**2  ##radius of 1st circle
CC22=L2**2  ##radius for 2nd circle
eq21 = sympy.Eq(QQ21, CC21)
eq22 = sympy.Eq(QQ22, CC22)
solution2=sympy.solve([eq21, eq22])  ##system equations solution
print(solution2)
S21=solution2[0]
S22=solution2[1]
VI2=[list(S21.values()),list(S22.values())]##values for both solution
##first solution
print(VI2)
x22=VI2[0][0]
y22=VI2[0][1]
######via point has finish###
###INVERSE K FOR INTIAL POINT
A2O =-2*a2*x21;

B2O =-2*a2*y21;
C2O = x21**2 +a2**2 + y21**2 -b2**2;
th210=2*m.atan2((-B2O -m.sqrt(B2O**2 + A2O**2 - C2O**2)),(C2O - A2O))*(180/m.pi);
th220=(m.atan2(y21-a2*m.sin(th210*m.pi/180),x21-a2*m.cos(th210*m.pi/180))*180/m.pi)-th210;
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
#8 by 8 matrices FOR FIRST ACTUATOR (perameters coiffients)
B21=np.array([[th210],[th211],[th211],[th212],[0],[0],[0],[0]]) #values of abgles obtained from inverse
kinematic 1.st actuator
A21=np.array([[1 ,0 ,0, 0, 0 ,0, 0, 0],[1 ,tf21, tf21*tf21, tf21*tf21*tf21, 0, 0, 0 ,0],
    [0, 0 ,0 ,0 ,1 ,0, 0, 0 ],[0 ,0 ,0 ,0, 1, tf22, tf22*tf22, tf22*tf22*tf22],
    [0 ,1 ,0 ,0, 0 ,0, 0, 0 ],[0, 0, 0, 0, 0, 1, 2*tf22, 3*tf22*tf22],
    [0 ,1 ,2*tf21+3*tf21*tf21, 0, 0, -1, 0 ,0,] ,[0, 0, 2, 6*tf21, 0 ,0 ,-2, 0] ])
A21N= np.linalg.inv(A21)
C21=np.matmul(A21N,B21)
print(C21)
#print(th10,th11,th12)
t21s1=C21[0,0]+C21[1,0]*t+C21[2,0]*t**2+C21[3,0]*t**3; #first segment equation for 1.st act
t21s2=C21[4,0]+C21[5,0]*t+C21[6,0]*t**2+C21[7,0]*t**3; #second segment equation  for 1st act

#8 by 8 matrices FOR second ACTUATOR
B22=np.array([[th220],[th221],[th221],[th222],[0],[0],[0],[0]]) #values of angles obtained from inverse
kinematic 1.st actuator
#8 by 8 matrices FOR second ACTUATOR (perameters coiffients)
A22=np.array([[1 ,0 ,0, 0, 0 ,0, 0, 0],[1 ,tf21, tf21*tf21, tf21*tf21*tf21, 0, 0, 0 ,0],
    [0, 0 ,0 ,0 ,1 ,0, 0, 0 ],[0 ,0 ,0 ,0, 1, tf22, tf22*tf22, tf22*tf22*tf22],
    [0 ,1 ,0 ,0, 0 ,0, 0, 0 ],[0, 0, 0, 0, 0, 1, 2*tf22, 3*tf22*tf22],
    [0 ,1 ,2*tf21+3*tf21*tf21, 0, 0, -1, 0 ,0,] ,[0, 0, 2, 6*tf21, 0 ,0 ,-2, 0] ])
A2N=np.linalg.inv(A22)
C22=np.matmul(A2N,B22)
#print(th20,th21,th22)
t22s1=C22[0,0]+C22[1,0]*t+C22[2,0]*t**2+C22[3,0]*t**3; #first segment equation for 1.st act
t22s2=C22[4,0]+C22[5,0]*t+C22[6,0]*t**2+C22[7,0]*t**3; #second segment equation  for
tt21=[];tt22=[];tt23=[];tt24=[]  #empty lists for thetas erray to be send like integers
for aq21 in t21s1*100:
    bb21=('%0.0f'%aq21)
    #print(bb1)
    tt21.append(int(bb21))
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
