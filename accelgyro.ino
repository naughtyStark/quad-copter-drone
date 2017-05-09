#define my_asin(a) a*(1+0.5*a*a) // mclaurin's expansion of asin() upto 2nd term. good enough accuracy for the purpose

inline void callimu()
{
  //-------EXTRACTION AND PROCESSING OF ACCEL-GYRO DATA BEGINS--
  accelgyro.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]); //calling imu for values , takes ~300us
  for(i=0;i<3;i++)
  {
    A[i]=a[i];
    A[i]-=offsetA[i]; //subtracting offset
    A[i]*=0.0006103; //mapping to real world values in m/s^2 , instead of dividing by 1638.4 i multiply by 1/1638.4 to save on time as multiplication is faster 
    
    A[i]= (0.8*A[i])+(0.2*lastA[i]); //applying the steady state assumption that the Acceleration can't change too much within 2.5 ms 
    lastA[i]=A[i]; 

    G[i]=g[i];
    G[i]-=offsetG[i]; // subtracting offset
    G[i]*=0.007633;   //mapping to degrees per second by multiplying with 1/131 
    G[i]=(0.8*G[i])+(0.2*lastG[i]);  //buffer filter,same as that for accel.
    lastG[i]=G[i]; 
  }   // we are at about the 480us mark as this for loop takes ~180 us to execute 
  //-----EXTRACTION OF ACCEL-GYRO DATA ENDS------
  
  orientationUpdate();   //~170us function   
}                            //we reach the 650us mark here


inline void orientationUpdate()    
{
  
  T[0]+=float(G[0]*dt);  //T[0]=pitch,T[1]=roll,T[2]=yaw.    //~30us 
  T[1]+=float(G[1]*dt);
  //yaw compensation for pitch and roll
  /*actual formula- 
   * T[0] += T[1]*sin(G[2]*0.0000436);
   * T[1] -= T[0]*sin(G[2]*0.0000436);
   * even this is an approximation from sin(pitch)=sin(roll)*sin(yaw) as the pitch vs yaw curve is approx a sin function. 
  */
  T[0] += T[1]*(G[2]*0.0000436); //0.0000436=dt/57.3 because sin takes radian not degrees,also, sin(x)=x as x->0 .this saves me fucking 450us
  T[1] -= T[0]*(G[2]*0.0000436);  //because sin() takes fucking 255 us(max, min=180us) to execute. fuck that i can live with approximations
                                            // ~60us
  if((A[1]*A[1])<81&&(A[0]*A[0])<81)  //making sure that magnitude of A[0] and A[1] is less than 9.not using mod because it takes 70us to execute once               
  {
    T[0]= 0.95*T[0]-2.865*my_asin(A[1]*0.102);
    T[1]= 0.95*T[1]+2.865*my_asin(A[0]*0.102);
  }                                 //~100us
}

