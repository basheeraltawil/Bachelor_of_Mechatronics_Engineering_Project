# RoboCraft / graduation-project source
# Extracted from: codesappendix.pdf
# Source status: legacy appendix extraction; not independently hardware-tested here.
# The original thesis uses Raspberry Pi + Python, Arduino/ATmega, I2C, and DC-motor control.
# Review pin assignments, calibration constants, dependencies, and safety limits before use.

th22h=(m.atan2(YE3-aa*m.sin(th12h*m.pi/180),XE3-aa*m.cos(th12h*m.pi/180))*180/m.pi)-th12h;
    #th12h ve th22h Pic'e yollanacak raspberryden !!!!!
    while True:
        try:
            writeNumber(th12h,th22h)
            time.sleep(1)                    #delay one second

        except KeyboardInterrupt:
            quit()





# 4-Drawing square task(pure translation)
# In this task we used just second and third  manipulator without any attaching and detaching operation
so that it can go segment by segment until we reach to the final position of platform center .This task
can be configure and apply into laser cutting job  which we chose it to be our main task configuration.


                                              #Note:to move indentations or dedent press ctr+] or ctr+[
                                                 #just second and third manipulator will be implemented
import numpy as np
import sympy
import math as m
import matplotlib.pyplot as plt
from pylab import *
import time
import serial
def f(xg21,xg23,yg21,yg23,xg31,xg33,yg31,yg33): #DEFING function for instantly chanegable variables
                                                    ######################second manipulator code start here
    tf21=10;tf22=10  #time period definition
    t=np.arange(0,10,0.5);
Second
manipulator
footprint
Third
manipulator
footprint
Starting
point

    #x2=80;y2=200 #second station of end effector
    a2=230;b2=230  #links length
    #algorithm to obtain via point and end effector positions
    x22, y22 = sympy.symbols("x22 y22", real=True)  #to solve system equation
    x21=-(277.13-xg21);    x23=-(277.13-xg23);     y21=-(160-yg21);       y23=-(160-yg23);#position w.r.t
manipulator itself
    WW2=m.sqrt((y23-y21)**2+(x23-x21)**2)
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
    #8 by 8 matrices FOR FIRST ACTUATOR
    B21=np.array([[th210],[th211],[th211],[th212],[0],[0],[0],[0]]) #values of abgles obtained from inverse
kinematic 1.st actuator
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
    B22=np.array([[th220],[th221],[th221],[th222],[0],[0],[0],[0]]) #values of angles obtained from inverse
kinematic 1.st actuator
    A22=np.array([[1 ,0 ,0, 0, 0 ,0, 0, 0],[1 ,tf21, tf21*tf21, tf21*tf21*tf21, 0, 0, 0 ,0],

        [0, 0 ,0 ,0 ,1 ,0, 0, 0 ],[0 ,0 ,0 ,0, 1, tf22, tf22*tf22, tf22*tf22*tf22],
        [0 ,1 ,0 ,0, 0 ,0, 0, 0 ],[0, 0, 0, 0, 0, 1, 2*tf22, 3*tf22*tf22],
        [0 ,1 ,2*tf21+3*tf21*tf21, 0, 0, -1, 0 ,0,] ,[0, 0, 2, 6*tf21, 0 ,0 ,-2, 0] ])
    A2N=np.linalg.inv(A22)
    C22=np.matmul(A2N,B22)
    #print(th20,th21,th22)
    t22s1=C22[0,0]+C22[1,0]*t+C22[2,0]*t**2+C22[3,0]*t**3; #first segment equation for 1.st act
    t22s2=C22[4,0]+C22[5,0]*t+C22[6,0]*t**2+C22[7,0]*t**3; #second segment equation  for  1.st act
    tt21=[];tt22=[];tt23=[];tt24=[]  #empty lists for thetas erray
    for aq21 in t21s1*100:
        bb21=('%0.0f'%aq21)
        #print(bb1)
        tt21.append(int(bb21))
        #tt1.append(int(bb21)/100)
        #time.sleep(1)
    tm21s1=tt21  #list to be send for first segment of first motor
    print(tm21s1)
    for aq22 in t21s2*100:

        bb22=('%0.0f'%aq22)
        #print(bb22)
        tt22.append(int(bb22))
        #time.sleep(1)
    tm21s2=tt22  #list to be send for second segment of first motor
    print(tm21s2)
    for aq23 in t22s1*100:
        bb23=('%0.0f'%aq23)
        #print(bb23)
        tt23.append(int(bb23))

        #time.sleep(1)
    tm22s1=tt23  #list to be send for first segment of second motor
    print(tm22s1)
    for aq24 in t22s2*100:
        bb24=('%0.0f'%aq24)
        ##print(bb4)
        tt24.append(int(bb24))
        #time.sleep(1)
    tm22s2=tt24  #list to be send for second segment of second motor
    print(tm22s2)
    print('Second manipulator data')
                      ##second manipulator code end point
                                                                    ######################thid manipulator code start here
    tf31=10;tf32=10  #time period definition
    t=np.arange(0,10,0.5);
    #x2=80;y2=200 #second station of end effector
    a3=230;b3=230  #links length
    #algorithm to obtain via point start
    x32, y32 = sympy.symbols("x32 y32", real=True)  #to solve system equation
    x31=-(-277.13-xg31);       x33=-(-277.13-xg33);       y31=-(160-yg31);       y33=-(160-yg33);#first and last
station of end effector
    WW3=m.sqrt((y33-y31)**2+(x33-x31)**2)
    THE3=30      ##bending angle (it can be change)
    QQ31=(x32-x31)**2+(y32-y31)**2  ##first circle equ
    QQ32=(x32-x33)**2+(y32-y33)**2  ##second circle equ
    L3=(WW3/2)/(m.cos(THE3*m.pi/180))  ##midpoint of distance
    CC31=L3**2  ##radius of 1st circle
    CC32=L3**2  ##radius for 2nd circle
    eq31 = sympy.Eq(QQ31, CC31)

    eq32 = sympy.Eq(QQ32, CC32)
    solution3=sympy.solve([eq31, eq32])  ##system equations solution
    print(solution3)
    S31=solution3[0]
    S32=solution3[1]
    VI3=[list(S31.values()),list(S32.values())]##values for both solution
    ##first solution
    x32=VI3[0][0]
    y32=VI3[0][1]
    ######via point has finish###
    ###INVERSE K FOR INTIAL POINT
    A3O =-2*a3*x31;
    B3O =-2*a3*y31;
    C3O = x31**2 +a3**2 + y31**2 -b3**2;
    th310=2*m.atan2((-B3O -m.sqrt(B3O**2 + A3O**2 - C3O**2)),(C3O - A3O))*(180/m.pi);
    th320=(m.atan2(y31-a3*m.sin(th310*m.pi/180),x31-a3*m.cos(th310*m.pi/180))*180/m.pi)-th310;
    #INVERSE K FOR via POINT
    AM3=-2*a3*x32;
    BM3=-2*a3*y32;
    CM3=x32**2 +a3**2 + y32**2 -b3**2;
    th311=2*m.atan2((-BM3 -m.sqrt(BM3**2 + AM3**2 - CM3**2)),(CM3 - AM3))*(180/m.pi);
    th321=(m.atan2(y32-a3*m.sin(th311*m.pi/180),x32-a3*m.cos(th311*m.pi/180))*180/m.pi)-th311;
    #INVERSE K FOR FINAL POINT
    AF3=-2*a3*x33;
    BF3=-2*a3*y33;
    CF3=x33**2 +a3**2 +y33**2-b3**2;
    th312=2*m.atan2((-BF3 -m.sqrt(BF3**2 + AF3**2 - CF3**2)),(CF3 -AF3))*(180/m.pi);
    th322=(m.atan2(y33-a3*m.sin(th312*m.pi/180),x33-a3*m.cos(th312*m.pi/180))*180/m.pi)-th312;
    #8 by 8 matrices FOR FIRST ACTUATOR

    B31=np.array([[th310],[th311],[th311],[th312],[0],[0],[0],[0]]) #values of abgles obtained from inverse
kinematic 1.st actuator
    A31=np.array([[1 ,0 ,0, 0, 0 ,0, 0, 0],[1 ,tf31, tf31*tf31, tf31*tf31*tf31, 0, 0, 0 ,0],
        [0, 0 ,0 ,0 ,1 ,0, 0, 0 ],[0 ,0 ,0 ,0, 1, tf32, tf32*tf32, tf32*tf32*tf32],
        [0 ,1 ,0 ,0, 0 ,0, 0, 0 ],[0, 0, 0, 0, 0, 1, 2*tf32, 3*tf32*tf32],
        [0 ,1 ,2*tf31+3*tf31*tf31, 0, 0, -1, 0 ,0,] ,[0, 0, 2, 6*tf31, 0 ,0 ,-2, 0] ])
    A31N= np.linalg.inv(A31)
    C31=np.matmul(A31N,B31)
    #print(th30,th31,th12)
    t31s1=C31[0,0]+C31[1,0]*t+C31[2,0]*t**2+C31[3,0]*t**3; #first segment equation for 1.st act
    t31s2=C31[4,0]+C31[5,0]*t+C31[6,0]*t**2+C31[7,0]*t**3; #second segment equation  for 1st act
    #8 by 8 matrices FOR second ACTUATOR
    B32=np.array([[th320],[th321],[th321],[th322],[0],[0],[0],[0]]) #values of angles obtained from inverse
kinematic 1.st actuator
    A32=np.array([[1 ,0 ,0, 0, 0 ,0, 0, 0],[1 ,tf31, tf31*tf31, tf31*tf31*tf31, 0, 0, 0 ,0],
        [0, 0 ,0 ,0 ,1 ,0, 0, 0 ],[0 ,0 ,0 ,0, 1, tf32, tf32*tf32, tf32*tf32*tf32],
        [0 ,1 ,0 ,0, 0 ,0, 0, 0 ],[0, 0, 0, 0, 0, 1, 2*tf32, 3*tf32*tf32],
        [0 ,1 ,2*tf31+3*tf31*tf31, 0, 0, -1, 0 ,0,] ,[0, 0, 2, 6*tf31, 0 ,0 ,-2, 0] ])
    A3N=np.linalg.inv(A32)
    C32=np.matmul(A3N,B32)
    #print(th20,th21,th22)
    t32s1=C32[0,0]+C32[1,0]*t+C32[2,0]*t**2+C32[3,0]*t**3; #first segment equation for 1.st act
    t32s2=C32[4,0]+C32[5,0]*t+C32[6,0]*t**2+C32[7,0]*t**3; #second segment equation  for
    tt31=[];tt32=[];tt33=[];tt34=[]  #empty lists for thetas erray
    for aq31 in t31s1*100:
        bb31=('%0.0f'%aq31)
        #print(bb3)
        tt31.append(int(bb31))
        #time.sleep(1)

    tm31s1=tt31  #list to be send for first segment of first motor
    print(tm31s1)
    for aq32 in t31s2*100:

        bb32=('%0.0f'%aq32)
        #print(bb22)
        tt32.append(int(bb32))
        #time.sleep(1)
    tm31s2=tt32  #list to be send for second segment of first motor
    print(tm31s2)
    for aq33 in t32s1*100:
        bb33=('%0.0f'%aq33)
        #print(bb23)
        tt33.append(int(bb33))
        #time.sleep(1)
    tm32s1=tt33  #list to be send for first segment of second motor
    print(tm32s1)
    for aq34 in t32s2*100:
        bb34=('%0.0f'%aq34)
        ##print(bb4)
        tt34.append(int(bb34))
        #time.sleep(1)
    tm32s2=tt34  #list to be send for second segment of second motor
    print(tm32s2)
    print('Third manipulator data')
    #print("verification done")


for i in [0,1,2,3,4]:  ##slicing operation into five segment

    if i==0: #first segment will be run from stationary point of end effector to first corner of square
        xg21=60*m.cos(60*m.pi/180);  xg23=60*m.cos(60*m.pi/180);    yg21=160+60*m.sin(60*m.pi/180);
yg23=100+60*m.sin(60*m.pi/180);  ##position w.r.t global frame(2nd manipulator)
        xg31=-60;               xg33=-60;        yg31=160;          yg33=100;  ##position w.r.t global frame (third
manipulator)
        f(xg21,xg23,yg21,yg23,xg31,xg33,yg31,yg33)##calling function to evaluatee suitable condition
        print("stationary  line^^^^^^^ ")
        print("----------------------------------------------------------------------------------------------------------------------------
-------- ")
        time.sleep(1)
    if i==1:#second segment will be run from first to second corner of square
        xg21=60*m.cos(60*m.pi/180);  xg23=130;    yg21=100+60*m.sin(60*m.pi/180);
yg23=100+60*m.sin(60*m.pi/180);##position w.r.t global frame(2nd manipulator)
        xg31=-60;               xg33=40;        yg31=100;          yg33=100;##position w.r.t global frame (third
manipulator)
        f(xg21,xg23,yg21,yg23,xg31,xg33,yg31,yg33)##calling function to evaluatee suitable condition
        print("square first  line CCW^^^^^^^ ")
        print("----------------------------------------------------------------------------------------------------------------------------
-------- ")
        time.sleep(1)
    if i==2:#third segment will be run from second to third corner of square
         xg21=130;  xg23=130;   yg21=151.96; yg23=51.96 ##position w.r.t global frame(2nd manipulator)
         xg31=40;  xg33=40;   yg31=100;          yg33=0;##position w.r.t global frame (third manipulator)
         f(xg21,xg23,yg21,yg23,xg31,xg33,yg31,yg33)##calling function to evaluatee suitable condition
         print("square second  line CCW^^^^^^^ ")
         print("---------------------------------------------------------------------------------------------------------------------------
----------- ")
         time.sleep(1)

    if i==3:  #fourth  segment will be run from third to fourth corner of square
        xg21=100+60*m.cos(60*m.pi/180);  xg23=30;    yg21=60*m.sin(60*m.pi/180);
yg23=60*m.sin(60*m.pi/180);##position w.r.t global frame(2nd manipulator)

        xg31=40;               xg33=-60;        yg31=0;          yg33=0;##position w.r.t global frame (third
manipulator)
        f(xg21,xg23,yg21,yg23,xg31,xg33,yg31,yg33)##calling function to evaluatee suitable condition
        print("square third  line CCW^^^^^^^ ")
        print("----------------------------------------------------------------------------------------------------------------------------
------------- ")
        time.sleep(1)
    if i==4:#fifth segment will be run from fourth to first corner of square
        xg21=60*m.cos(60*m.pi/180);  xg23=60*m.cos(60*m.pi/180);    yg21=60*m.sin(60*m.pi/180);
yg23=100+60*m.sin(60*m.pi/180);##position w.r.t global frame(2nd manipulator)
        xg31=-60;               xg33=-60;        yg31=0;          yg33=100;##position w.r.t global frame (third
manipulator)
        f(xg21,xg23,yg21,yg23,xg31,xg33,yg31,yg33)##calling function to evaluatee suitable condition
        print("square fourth  line CCW ^^^^^^^^^^")
        print("----------------------------------------------------------------------------------------------------------------------------
----------- ")
        time.sleep(1)
