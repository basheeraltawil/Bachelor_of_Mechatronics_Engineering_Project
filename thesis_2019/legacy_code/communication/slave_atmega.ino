// RoboCraft / graduation-project source
// Extracted from: codesappendix.pdf
// Source status: legacy appendix extraction; not independently hardware-tested here.
// Review pin assignments, calibration constants, motor direction, and safety limits before use.

//HABERLESME TANIMLAMALARI
#include <Wire.h>
#define SLAVE_ADDRESS 0x08
int data [32];
int data11 [32];
int datanew[16];
int datanew11[16];
int x = 0;
int y = 0;
volatile int16_t data1[2];
volatile int16_t datanew1[2];
                         //PATHDEN GELEN TANIMLAMALAR
//lİNK lENGHT CONSTANT

float a =230.0;
float b = 230.0;
//İnital positions of the our robot arms end effector CONSTANT
float A1x, A1y;
//forward kinematics Angle of manipulator1
float tetha12 ; //pottan alınacak
float tetha11 ;
//global point that will be taken from user
float Ax=0;
float Ay=0;
//end effector position defining
float Axx;
float Ayy;
//inital distance(offsets)
float x1 = 0;      //IT WİLL CHANGE FOR EACH MANİPULATOR THESE ONE FOR 1.ST MANİPULATIOR
float y1 =-325.0 ; //IT WİLL CHANGE FOR EACH MANİPULATOR THESE ONE FOR 1.ST MANİPULATIOR
//inverse kinematics angle of link1
float th12, th11;
//inverse kinematics angle of link1 to origin
float tht12, tht11;
//Saranın tanımlamaları
float Kp =10.0;
float Kp2=0.8;
int i = 0, sum = 0;
float angle, angle2, voltage, voltage2, pot1, pot2, reftheta, reftheta2, error, error2, pwm, pwm2;
 int p=1;
void setup() {
Serial.begin(9600);
Wire.begin(SLAVE_ADDRESS);

Wire.onReceive(receiveData);
Wire.onRequest(sendAnalogReading);

}

void loop() {

   for( int l = 0; l<2; l++)
  {
    data1[l] = analogRead( A4 + l);
    delay(50);
    datanew1[l]=map(data1[l],0,1023,0,255);
    //Serial.println(datanew1[l]);
  }
  /*for( int p = 0; p<16; p++)
  {
    Serial.print(datanew[p]);
    Serial.print("\t");
    Serial.println(datanew11[p]);

    delay(50);
  }*/
for(int m=0;m<8;m++)
{
  pot1 = analogRead( A4);
  pot2=analogRead( A5);
  voltage = pot1 * (5.0 / 1023);
  tetha11 = voltage * 720.0 ; //initial angle of link1
  voltage2 = pot2 * (5.0 / 1023);

  tetha12 = voltage2 * 720.0 ; //initial angle link2
 //Serial.println(tetha11);
/*Serial.print(tetha11);
Serial.print("\t");
Serial.println(tetha12);*/
   th11=datanew[m];
   th12=datanew[m+8];
   delay(100);
   //SARANIN KODU BURDA OLACAK
  error = (tetha11)-(th11+1172.0);   //675 FOR FİRST MANİPULATOR FİRST MOTOR REFERENCE ANGLE.IT
WİLL CAHANGE FOR EACH MANİPULATOR
  error2 = (tetha12)-(th12+1158.0);  //418 FOR FİRST MANİPULATOR SECOND MOTOR REFERENCE
ANGLE.IT WİLL CAHANGE FOR EACH MANİPULATOR
  /*Serial.print(error);
  Serial.print("\t");
  Serial.println(error2);*/
  if (error > 0)
  {
    pwm = (Kp) * error;
    if (pwm > 255)
    {
      pwm = 255;
    }
    analogWrite(8, pwm); //ccw
    analogWrite(9, 0);

  }
  if (error < 0)
  {

    pwm = -(Kp * error); // pwm can not be negative
    if (pwm > 255)
    {
      pwm = 255;
    }
    analogWrite(9, pwm); //cc
    analogWrite(8, 0);

  }

  if (error2 > 0)
  {
    pwm2 = (Kp2) * error2;
    if (pwm2 > 255)
    {
      pwm2 = 255;
    }
    analogWrite(5, pwm2); //ccw
    analogWrite(7, 0);

  }
  if (error2 < 0)
  {
    pwm2 = -(Kp2 * error2); // pwm can not be negative
    if (pwm2 > 255)
    {
      pwm2 = 255;
    }
    analogWrite(7, pwm2); //cc

    analogWrite(5, 0);

  }
  Serial.print(pwm);
  Serial.print("\t");
  Serial.println(pwm2);
}
Serial.println("-------------------------------------------------------------------------------------------");
delay(1000);
  for(int n=0;n<8;n++)
  {
 pot1 = analogRead(A4);
  pot2 = analogRead(A5);
  voltage = pot1 * (5.0 / 1023);
  tetha11 = voltage * 720.0 ; //initial angle of link1
  voltage2 = pot2 * (5.0 / 1023);
  tetha12 = voltage2 * 720.0 ; //initial angle link2
   /*Serial.print(tetha11);
    Serial.print("\t");
    Serial.print(tetha12);
    Serial.println();*/
 th11=datanew11[n];
 th12=datanew11[n+8];
 delay(100);
  /*Serial.print(th12);
  Serial.print("\t");
  Serial.println(th11);*/
  //SARANIN KODU BURDA OLACAK

  error = (tetha11)-(th11+1172.0);   //675 FOR FİRST MANİPULATOR FİRST MOTOR REFERENCE ANGLE.IT
WİLL CAHANGE FOR EACH MANİPULATOR
  error2 = (tetha12)-(th12+1158.0);  //418 FOR FİRST MANİPULATOR SECOND MOTOR REFERENCE
ANGLE.IT WİLL CAHANGE FOR EACH MANİPULATOR
 /* Serial.print(error);
  Serial.print("\t");
  Serial.println(error2);*/
  if (error > 0)
  {
    pwm = (Kp) * error;
    if (pwm > 255)
    {
      pwm = 255;
    }
    analogWrite(8, pwm); //ccw
    analogWrite(9, 0);

  }
  if (error < 0)
  {
    pwm = -(Kp * error); // pwm can not be negative
    if (pwm > 255)
    {
      pwm = 255;
    }
    analogWrite(9, pwm); //cc
    analogWrite(8, 0);

  }


  if (error2 > 0)
  {
    pwm2 = (Kp2) * error2;
    if (pwm2 > 255)
    {
      pwm2 = 255;
    }
    analogWrite(5, pwm2); //ccw
    analogWrite(7, 0);

  }
  if (error2 < 0)
  {
    pwm2 = -(Kp2 * error2); // pwm can not be negative
    if (pwm2 > 255)
    {
      pwm2 = 255;
    }
    analogWrite(7, pwm2); //cc
    analogWrite(5, 0);

  }
   Serial.print(pwm);
  Serial.print("\t");
  Serial.println(pwm2);
  }
}
void receiveData(int byteCount) {

   while(Wire.available()) {               //Wire.available() returns the number of bytes available for retrieval
with Wire.read(). Or it returns TRUE for values >0.
       data[x]=Wire.read();
       data11[y]=Wire.read();
         x++;
       y++;

 }
     for(int i=0;i<16;i++)
     {
     //Serial.println((data[i])*(255)+data[i+16]);
     datanew[i]=((data[i])*(255)+data[i+16])/100;
     datanew11[i]=((data11[i])*(255)+data11[i+16])/100;
     }
     for(int k=0;k<16;k++)
     {
      if(datanew[k]<0)
      {
        datanew[k]=datanew[k]+256;
      }
       if(datanew11[k]<0)
      {
        datanew11[k]=datanew11[k]+256;
      }
     }
   }
void sendAnalogReading(){
 Wire.write( (uint8_t *) datanew1, sizeof( datanew1));
}
