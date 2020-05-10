/*
 * Team Id: <0367>
 * Top 5 in eyantra robotic competition
 * Author:Sanchit Kulkarni
 * Filename: code_documentation
 * Theme: Biped patrol(Balancin robot)
 * Functions: getaccelang,kalmanfilter,compute,Drive_Motor,Drive_Motor_Left,Drive_Motor_Right,electromagnet_on,electromagnet_on1,electromagnet_off,electromagnet_off1(),
  */ 


// Libraries
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#define G 16384.0      //Both Values obtained from MPU6050 Datasheet
#define GYRO_SCL 131.0

 // Global Variables:
uint32_t loopStartTime;
const int IN1 = 8; // motor 1
const int IN2 = 7;  
const int EN = 9;   //Enable1
const int EN1=2;     //Enable2
const int IN3 = 11;  //motor 2
const int IN4 = 12;
const int green_pin=43;
const int common=45;
const int red_pin=47;
const int buzzer=31;
int var_flag=0; //flag for  setting direction(forward backward left right)
int incr=0;  //flags for increamenting set points
int incr1=0;  
int incrb=0;
int incrf=0;  
int flag=0;
int start_time=0; //flag for counting time 
int donestart_time=0;
double torque = 0;//variable to write voltage on pwm
char Direction = 'f'; //  Set default forward


int ser=0;
int digi=0;
int ana_fb=0;
int ana_rl=0;
int magnet=5;
int magnet1=23;





//1.4 MPU6050 Variables
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
double acc_angle;
double gyro_rate;

//1.5 Kalman Filter Variables
float Q_angle  =  0.001;
float Q_gyro   =  0.003;
float R_angle  =  0.03;

double actAngle; //Output Angle
double angle = 0;  // Reset the angle
float bias = 0;   // Reset bias
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;  //error covirance matrix reset to 0
float dt, y, S;
float K_0, K_1;
int flag1=0;
 //PID Variables
//Setpoint. where the robot is balanced.  
double Setpoint =0.18;
//distance away from setpoint
double error, ITerm, lastInput, dInput;
double Kp = 160.0, Ki =60, Kd = 4;
unsigned long SampleTime = 100;
unsigned long lastTime;
double outMin = -255.0, outMax = 255.0;
double output; //Temp Var for debugging

/*
▪ * Function Name: getaccelang
▪ * Input:  ay accleration in y Direction,  az acc in z Directionection
▪ * Output:  angle in degrees
▪ * Logic:   calculates acc angle  using ay and az
            Convert to 360 degrees resolution
             atan2 outputs the value of -π to π (radians)
             We are then convert it to 0 to 2π and then from radians to degrees
▪ * Example Call: getaccelang( ay, az)
▪ */ 

  
inline double getaccelang(int16_t ay, int16_t az) {
 
    double angle = ( ( atan2( -ay , -az ) + PI ) * RAD_TO_DEG );
    
    if( angle >= 180 ) {  //to map the range from -180 to 180
      angle -= 360;
    }
    return angle;
}


/*
▪ * Function Name: kalmanfilter
▪ * Input:  newAngle, newRate, looptime
▪ * Output: angle
▪ * Logic: It predicts and updates the tilt angle of the bot.
▪ * Example Call: kalmanfilter( newAngle,  newRate, looptime)
▪ */ 

//4.2 Kalman Function
double kalmanfilter(float newAngle, float newRate,int looptime) {
    
    dt = looptime / 1000.0;
    angle += dt * (newRate - bias);  //angle = rate * timesample
    
    P_00 +=  dt * ( dt * P_11 - P_01 - P_10 + Q_angle);
    P_01 -=  dt * P_11;
    P_10 -=  dt * P_11;
    P_11 +=  Q_gyro * dt;
    
    S = P_00 + R_angle;
    K_0 = P_00 / S;
    K_1 = P_10 / S;
    
    y = newAngle - angle;    
    angle +=  K_0 * y;
    bias  +=  K_1 * y;
    
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;
    
    return angle;
}  //Kalman


/*
▪ * Function Name:Drive_Motor
▪ * Input:  pwm value  Direction is r(reverse) f(forward)
▪ * Output: none
▪ * Logic: to drive motors by providing required torque 
▪ * 
▪ * Example Call: Drive_Motor( torque,  Direction)
▪ */ 

//4.3 Drive Motors
void Drive_Motor( double torque, char Direction) {
  
  if(Direction == 'f') { //Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else if( Direction == 'r' ) {  //reverse motion
    digitalWrite (IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
  }
  
  analogWrite(EN, (int) torque);  //PWM is written last to ensure the Motor move in the
  analogWrite(EN1, (int) torque);                         //correct Directionection
  
}  


/*
▪ * Function Name:Drive_Motor_Right
▪ * Input: torque Direction
▪ * Output: none
▪ * Logic: to drive the bot in right Directionection 
▪ * 
▪ * Example Call: Drive_Motor_Right(torque,  Direction)
▪ */ 

//Drive_Motor
void Drive_Motor_Right( double torque, char Direction) {
  
  if(Direction == 'f') { //Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  else if( Direction == 'r' ) {  //reverse motion
    digitalWrite (IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(IN3, LOW);
  }
  
  analogWrite(EN, (int) torque);  //PWM is written last to ensure the Motor move in the
  analogWrite(EN1, (int) torque);                         //correct Directionection
  
}

/*
▪ * Function Name:Drive_Motor_Left
▪ * Input: torque Direction
▪ * Output: none
▪ * Logic: to drive the bot in left Directionection 
▪ * 
▪ * Example Call: Drive_Motor_Left(torque,  Direction)
▪ */
void Drive_Motor_Left( double torque, char Direction) {
  
  if(Direction == 'f') { //Forward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else if( Direction == 'r' ) {  //reverse motion
    digitalWrite (IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
  }
  
  analogWrite(EN, (int) torque);  //PWM is written 
  analogWrite(EN1, (int) torque);                         
}

/*
▪ * Function Name:SampTime
▪ * Input: newsampletime
▪ * Output: none
▪ * Logic: This sets the sampletime for the code 
▪ * 
▪ * Example Call: SampTime(Newsamp_time)
▪ */ 


void SampTime(int Newsamp_time)
{
   if (Newsamp_time > 0)
   {
      double ratio  = (double)Newsamp_time
                      / (double)SampleTime;
      Ki *= ratio;
      Kd /= ratio;
      SampleTime = (unsigned long)Newsamp_time;
   }
}

/*
▪ * Function Name:Compute
▪ * Input: none
▪ * Output:none
▪ * Logic: sets the torque to the pwm pin using Kp Ki Kd 
▪ * Example Call: Compute()
▪ */ 


void Compute()
{
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange >= SampleTime)
   {
         
      ITerm += (Ki * error);  //Computing error var 
      
      if(ITerm > outMax) ITerm= outMax;      //To reduce the time ITerm needs to settle down after error was recovered 
                                            
      else if(ITerm < outMin) ITerm= outMin;
      
      dInput = (actAngle - lastInput);
      
      /*Compute PID Output*/
      output = Kp * error + ITerm - Kd * dInput;
      
      if(output > outMax) output = outMax;
      if(output < outMin) output = outMin;
      if(output < 0) output = abs(output);
      
      torque = output;
    
      /*Remember some variables for next time*/
      lastInput = actAngle;
      lastTime = now;
   }
}
/*
▪ * Function Name:electromagnet_off
▪ * Input: none
▪ * Output: none
▪ * Logic: to switch off the magnet
▪ * Example Call: electromagnet_off()
▪ */ 

void electromagnet_off()
{
     Serial.print("LOW, ");
      digitalWrite(magnet,LOW);
}
/*
▪ * Function Name:electromagnet_on
▪ * Input: none
▪ * Output: none
▪ * Logic: to switch off the magnet1
▪ * Example Call: electromagnet_on()
▪ */ 


void electromagnet_on()
{
     Serial.print("HIGH, ");
      digitalWrite(magnet,HIGH);
}


/*
▪ * Function Name:electromagnet_off1
▪ * Input: none
▪ * Output: none
▪ * Logic: to switch off the magnet2
▪ * Example Call: electromagnet_off1()
▪ */ 

void electromagnet_off1()
{
     Serial.print("LOW1, ");
      digitalWrite(magnet1,LOW);
}
/*
▪ * Function Name:electromagnet_on1
▪ * Input: none
▪ * Output: none
▪ * Logic: to switch on the magnet
▪ * Example Call: electromagnet_on1()
▪ */ 


void electromagnet_on1()
{
     Serial.print("HIGH1, ");
      digitalWrite(magnet1,HIGH);
}
/*
▪ * Function Name:remote_control
▪ * Input: none
▪ * Output: none
▪ * Logic: for controlling the bot using remote
▪ * 
▪ * Example Call: remote_control()
▪ */ 

void remote_control()
{

if(Serial.available()>22)   //wait for 22 bytes of data 
 {
 for(int i=0;;i++)
 { 
  ser=Serial.read();  //ser is a byte of data 
  if(ser==126)        //0x7E marks new set of data (0x7E in decimal 126) 
    break;            //read only till the next 0x7E arrives 
    if(i==11)
    {
      digi=ser;
      if((digi==16)||(digi==0))
      {
        electromagnet_off();
      }
      if((digi==24)||(digi==8))
      {
         
        start_time++;    //to set green led to 600ms
       if(flag==0)
        {
          if(start_time<7)
          {
          digitalWrite(common,HIGH);
          digitalWrite(green_pin,HIGH);
          digitalWrite(red_pin,LOW);//
          pinMode(buzzer,OUTPUT);
          }
          if(start_time>=7)
          {
           
          digitalWrite(green_pin,LOW);
          digitalWrite(red_pin,LOW);
          digitalWrite(common,LOW);
          
          }
          if(start_time>=11)
          {
          digitalWrite(buzzer,HIGH);
                    //setting green led only once
          flag=1;  //once flag in 1 it never comes to this loop again
          }
          }
        electromagnet_on();
      }
      if((digi==8)||(digi==0))
      {
     electromagnet_off1();
      }
      if((digi==16)||(digi==24))
      {

       
      electromagnet_on1();
      }
     
    }
    if(i==13)        //ADC1 (joystic 1) corresponding data pos at 13
    {
      ana_fb=ser;      
      if((ana_fb>89)&&(ana_fb<121)) //Configured the analog values acoording to the voltage Vin to joystic
      {
        var_flag=0;
        Setpoint =0.18;
        incr=0;//all flags are reset when joystick is at relaxed position
        incr1=0;
        incrf=0;
        incrb=0;
        flag1=0; 
        Kd = 4;
       
       
       
      }
      if((ana_fb>190)&&(ana_fb<220))  // based on joystic analog values
      {
        var_flag=1;
       Serial.print("FORWARD");
         
         Kd = 3.5;
         Setpoint=-0.7;
       
      }
      if(ana_fb==255)
      {
        var_flag=1;
       
        Setpoint=0.9;
         Kd = 3.5;
      }

      if((ana_fb>140)&&(ana_fb<155))  // based on joystic analog values
      {
        var_flag=1;//185,.2,3.5
       Serial.print("STRONG FORWARD");  //can be used for high slopes elevation
         
           Kd = 3.5;
           if(incr==0)
           {                              //decreament setpoint 
            Setpoint=-2.5;
            incr=1;
           }
           else
           {
            if(flag1==0)
            {  
             if(Setpoint<-4.7)
             
             {
              Setpoint+=5.3;
              flag1=1;
              Kd=6;
             }
            Setpoint-=0.18;
            }
            else
            {
              Setpoint=0;
            }
           }
       
      }
       if((ana_fb>48)&&(ana_fb<53))  // based on joystic analog values
      {
        var_flag=1;
       Serial.print("STRONG BACKWARD"); //can be used for downhill
         
           Kd = 8;
           Setpoint=3.7;
         
      }
     
      
    }
    if(i==15)
    { ana_rl=ser;
     
      if((ana_rl>190)&&(ana_rl<220))
      {
         var_flag=2;//set flag for right
         Kd = 8;
         Setpoint = -0.4;
         Serial.print("right");
        // right;
      }
      if((ana_rl==255))
      {
         var_flag=3;////set flag for left
         Kd = 8;
         Setpoint = -0.4;
         Serial.print("left");
        // left;
      }
      
    }
    
   if((ana_rl==255)&&(ana_fb==255)) //led
   {
    donestart_time++; //counter increanebt
  // Serial.println(millis()); //to check at what time it arrives at this point
  
    if((donestart_time>402)&&(donestart_time<602))  //setting up led only once and for 600ms
    {
    
   
    digitalWrite(buzzer,LOW);
    digitalWrite(common,HIGH);
    digitalWrite(red_pin,HIGH);
    digitalWrite(green_pin,LOW);
   
    }
    if(donestart_time>=602)       //setting up buzzer only once and for 1s
    {
     
     digitalWrite(buzzer,HIGH);
     digitalWrite(red_pin,LOW);
    digitalWrite(green_pin,LOW);
    digitalWrite(common,LOW);
   
    
    }
    var_flag=0;
    Setpoint =0.18;
  //  Kd = 3.5;
    }
}
 } 
}


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    
    // initializeSerial communication
   Serial.begin(115200); //baud rate of 115200

    // initialize device
   
    accelgyro.initialize();

   
    
    loopStartTime = millis();
    
     SampTime(5);
    lastTime = millis()-SampleTime;
    
    //configure Arduino Pins
    pinMode(EN, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(EN1, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    pinMode(magnet1,OUTPUT);
    pinMode(magnet,OUTPUT);
    pinMode(red_pin,OUTPUT);
    pinMode(green_pin,OUTPUT);
    pinMode(common,OUTPUT);
    pinMode(red_pin,OUTPUT);
    
  }  //setup




//3.2 Execution Loop
void loop() {

  // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);    //Debug_Raw();
    
    acc_angle = getaccelang(ay, az);
    gyro_rate = gx / GYRO_SCL;    //Debug_Orient();
    
    actAngle = kalmanfilter( acc_angle, gyro_rate, millis() - loopStartTime );
    loopStartTime = millis();    //Debug_Filtered();
    
    error = Setpoint - actAngle; //distance away from setpoint
    // var_flaf is 0 for no input
     if (var_flag==0)
     {
      Setpoint =0.18;
      Kd = 4;
     }
     //drive motors forward or backward according to the error
    if( error > 0 ) {
      Direction = 'f';
    }
    else if ( error < 0 ) {
      Direction = 'r';
    }
    //computes the torque here
    
    Compute();    
    
    if(torque < 75)
      torque = 0;    //no point in balancing the bot wait for human
    
    if ( actAngle < -50 || actAngle > 50)
      torque = 0;    //No point of Trying to balance the robot if it is after 30 degrees

    
  
    //var_flag is set to 2 for right
    if(var_flag==2)
    {
      Drive_Motor_Right( torque, Direction);
        
    }
    //var_flag is set to 3 for left
    else if(var_flag==3)
    {
      Drive_Motor_Left( torque, Direction);
    
    }
    //otherwise normal balancing 
    else 
    {
    
    Drive_Motor( torque, Direction);
    
    }
 remote_control(); // for input from controller 
 

}//loop
