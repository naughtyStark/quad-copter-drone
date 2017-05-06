/*
 * throttle -pin 8
 * yaw- pin 9
 * pitch- pin 10
 * roll - pin 11
 */
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


#define LED 7
#define Kp 1
#define Kd 0.8
#define YawKp 0.5
#define minValue 1100

unsigned long FL,FR,BL,BR; //(Front left, front right, back left, back right motor)
long esc_timer;
#define FLpin B11110111   // binary codes for pulling a pin low. refer to motorWrite() function to understand what is happening 
#define FRpin B11101111   // the pins are set high at the begining of each loop
#define BLpin B11011111
#define BRpin B10111111

//-----ACCEL-GYRO STUFF BEGINS---------------------
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t a[3];  //accelerations from mpu6050
int16_t g[3];  //gyration rates from mpu6050

float A[3],G[3],lastA[3]={0,0,0},lastG[3]={0,0,0},offsetA[3],offsetG[3],T[2]; //x=0,y=1,z=2, T=tilt.
int i,j,connection;

//-----------ACCEL-GYRO STUFF ENDS----------------



//------variables for interrupt-------
volatile unsigned long timer[5];
volatile byte last_channel[4]={0,0,0,0};
volatile int input[4]={1000,1500,1500,1500};
//---------------------------

void setup()
{
  //enable interrupts on B port
  PCICR |= (1 << PCIE0);   
  PCMSK0 |= (1 << PCINT0); //8
  PCMSK0 |= (1 << PCINT1); //9
  PCMSK0 |= (1 << PCINT2); //10
  PCMSK0 |= (1 << PCINT3); //11
 //Serial.begin(38400);

  //===========attaching motors=====================
  DDRD |= B01111000;                           //set pins 3,4,5,6 as output

  PORTD |= B01111000;         //pull the pins high, bitwise or means that if the pin was originally low, 1|0 =1 so it will be pulled high
  delayMicroseconds(1000);
  PORTD &= B10000111;        //pull all the pins low
  
  //==============done==============================

  //===========ACCELGYRO SETUP BEGINS===============
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 12;              // for 400KHz
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    accelgyro.initialize();
    accelgyro.testConnection() ? connection=1 : connection=0 ; 
    // offsets
    //
//544.25||-826.98||16656.07||-128.42||-103.63||19.43||

    offsetA[0]=544;        //these offsets were calculated beforehand
    offsetA[1]=-826; 
    offsetA[2]=16656;
    offsetG[0]=(-128);
    offsetG[1]=(-103);
    offsetG[2]=19.43;
    
    for(j=0;j<2000;j++)   //taking 2000 samples for finding initial orientation
    {
      accelgyro.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);  //get the values
      for(i=0;i<2;i++)                                      
      {
        A[i]=a[i];         //transfer value
        A[i]-=offsetA[i]; //subtracting offset
        A[i]*=0.006103;    //convert to real world value
        lastA[i]+=A[i];    //store in lastA[i]
      }
    }
    A[0]=lastA[0]*0.0005;   //take average of 2000 readings
    A[1]=lastA[1]*0.0005;
    lastA[0]=0;  //lastA[i] was used here only as a place holder for a "sum" variable. it's purpose as a sum variable
    lastA[1]=0;  // has been fullfilled, therefore it will now be restored to 0 so that it can be used for it's origianl purpose
   
    T[0]=(-57.3*asin(A[1]/9.8));  //initial orientation 
    T[1]= 57.3*asin(A[0]/9.8);
//------------ACCEL-GYRO SETUP ENDS-------------

    pinMode(LED,OUTPUT);  //led pin

}


int throttle=1000;
float rollsetp=0.0,yawsetp=0.0,pitchsetp=0.0;
int p,r,y;
long lastTime;
#define dt 0.0025  //cycle time in seconds

long failsafe=0;
bool servoWrite=0;
bool arm=0;
bool state=0;


inline int deadBand(int input)
{
  if(input>1492&&input<1508)
  {
    return 1500;
  }
  return input;
}


inline void motorWrite()
{   
   while(PORTD>8)
   {
     if(micros()>= FL)
     {
        PORTD &= FLpin;
     }
     if(micros()>= FR)
     {
        PORTD &= FRpin;
     }
     if(micros()>= BL)
     {
        PORTD &= BLpin;
     }
     if(micros()>= BR)
     {
        PORTD &= BRpin;
     }
   }
}

inline void correction()
{
   if(throttle>1800)
   {
      throttle=1800;    //cap the max throttle value
   }
   if(throttle>minValue)   //if throttle is above minimum value 
   {
     ((throttle+p+r+y) <= minValue)? FL=minValue+esc_timer: FL=(throttle+p+r+y)+esc_timer;// adding esc_timer to FL  
                                                                                          //saves us time in the if()
     ((throttle+p-r-y) <= minValue)? FR=minValue+esc_timer: FR=(throttle+p-r-y)+esc_timer;//conditions in the motorWrite()
                                                                                          //function,making it more precise 
     ((throttle-p+r-y) <= minValue)? BL=minValue+esc_timer: BL=(throttle-p+r-y)+esc_timer;//and consuming less time as 
                                                                                           //addition takes~4-5us
     ((throttle-p-r+y) <= minValue)? BR=minValue+esc_timer: BR=(throttle-p-r+y)+esc_timer;
   }
   else
   {
      FL=(1000)+esc_timer;
      FR=(1000)+esc_timer;
      BL=(1000)+esc_timer;
      BR=(1000)+esc_timer;
   }   //~120us by now
   
   motorWrite();  
} 


void loop()
{
 if(connection==0)  //in case accelgyro connection fails
 {
    lastTime=esc_timer=micros();     //get time stamp
    PORTD |= B01111000;
    FL=(1000);
    FR=(1000);
    BL=(1000);
    BR=(1000);
    motorWrite();
    digitalWrite(LED,HIGH);
 }
 else
 {
   
   lastTime=esc_timer=micros();     //get time stamp
   PORTD |= B01111000;  //pull pins 3,4,5,6 high 
   
   callimu();   //takes 670us 
  
   if(yawsetp<(-40)&&throttle<minValue)   //arming sequence
   {
      arm=1;
   }
   else if(yawsetp>40&&throttle<minValue)  //disarming sequence 
   {
      arm=0;
   }
   //~700 us by now (max)
   
   r=Kp*(rollsetp-T[1]) + Kd*G[1];   //reducing time be not creating a function at all for these tiny tasks
   
   p=Kp*(pitchsetp-T[0]) + Kd*G[0];
   
   y=YawKp*(yawsetp-G[2]);

    //~765us
    
   if(servoWrite)          //when new info from receiver was received 
   {
      //transferring inputs
      throttle =input[0];
      yawsetp  =(1500-deadBand(input[1]))*0.1;   //this is yaw rate(deg/sec)
      pitchsetp =(1500-deadBand(input[2]))*0.1;   //roll, pitch setp in degrees
      rollsetp=(deadBand(input[3])-1500)*0.1;   
      
      servoWrite=0;     //to servo write ko false karo
      failsafe=millis();     //failsafe ko time stamp do
   }
   
   ((millis()-failsafe<1000)&&arm)? state=1 : state=0 ;     //if receiver is still connected or reconnects within a second, resume 
  
    //~850us by now
   if(state)   
   {
      correction();    
   }
   else if(!state)  //if receiver does not reconnect within a second or is lost, kill power
   {
      throttle=1000;
      correction();
   }
 }
 while(micros()-lastTime<2500);  //wait for the 2500 us to be over
}



ISR(PCINT0_vect)
{
  timer[0]=micros();
  //channel 1 ----
  
  if(last_channel[0]==0&& PINB & B00000001) //makes sure that the first pin was initially low and is now high
  {                                         //PINB & B00000001 is equivalent to digitalRead but faster
    last_channel[0]=1;
    timer[1]=timer[0];          
  }
  else if(last_channel[0]==1 && !(PINB & B00000001))
  {
    last_channel[0]=0;
    input[0]=timer[0]-timer[1];
  }

  //channel 2---
  if(last_channel[1]==0&& PINB & B00000010) //makes sure that the first pin was initially low and is now high
  {                                         //PINB & B00000001 is equivalent to digitalRead but faster
    last_channel[1]=1;
    timer[2]=timer[0];          
  }
  else if(last_channel[1]==1 && !(PINB & B00000010))
  {
    last_channel[1]=0;
    input[1]=timer[0]-timer[2];
  }

  //channel 3---
  if(last_channel[2]==0&& PINB & B00000100) //makes sure that the first pin was initially low and is now high
  {                                         //PINB & B00000001 is equivalent to digitalRead but faster
    last_channel[2]=1;
    timer[3]=timer[0];          
  }
  else if(last_channel[2]==1 && !(PINB & B00000100))
  {
    last_channel[2]=0;
    input[2]=timer[0]-timer[3];
  }
  
  //channel 4---
  if(last_channel[3]==0&& PINB & B00001000) //makes sure that the first pin was initially low and is now high
  {                                         //PINB & B00000001 is equivalent to digitalRead but faster
    last_channel[3]=1;
    timer[4]=timer[0];          
  }
  else if(last_channel[3]==1 && !(PINB & B00001000))
  {
    last_channel[3]=0;
    input[3]=timer[0]-timer[4];
    
    servoWrite=true;
  }
  
}
