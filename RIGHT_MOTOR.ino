/*Libraries*/
#include <TimerOne.h>
#include <Wire.h>
#define RIGHTMOTOR
//#define LEFTMOTOR
#define FILTERED

/******************Encoder_Filter_Intialize*******************/
/*PINS*/

#define ENCODER_PINA 2
#define ENCODER_PINB 3
#define DIR_PIN1 6
#define DIR_PIN2 7
#define PWM_PIN 5
/*Constants*/
#ifdef RIGHTMOTOR
  #define encoderPPR 380.0 
  #define kp 0.685
  #define ki 0.5
  #define kd 0
#endif 
#ifdef LEFTMOTOR
  #define encoderPPR 400.0 
  #define kp 0.8
  #define ki 0.5
  #define kd 0.003
  #define SLAVE_ADDRESS 0x12 // Address of the slave right arduino
#endif

#define timerPeriod (200000) //in micro-seconds

/*Variables*/
double output;
double LastOutput;
double maxOutput = 255;
double minOutput = 0;
double deadZone = 130;

volatile int pulses = 0;
int pulsesPrev = 0;
int pulsesDifference = 0;

double error = 0;
double errorPrev = 0;

double RPM = 0;
double RPMPrev = 0 ; 


double setPoint = 0;
bool flag = false;
double dt = timerPeriod*1e-6; //in seconds
 int x  ; 
/*----------------Functions-------------------------------------*/
/*EncoderISR*/
void encoderISR() {
  pulses++;
}
/*TimerISR*/
void timerISR() {
  noInterrupts();
  RPM =  (pulses*1.0/encoderPPR)/dt*60;
  pulses=0;
  PID_CONTROL();
if (x == 0 || x == 1 || x ==2 || x == 3 )
  {
   directionController(x);
  }
  
  interrupts();
}
void PID_CONTROL()
{
  LastOutput = output; 
  error = setPoint - int(RPM);
  double DeltaError = error - errorPrev;
  output = LastOutput;
  output += kp*error;
  output += ki*DeltaError*dt;
  output += kd*(RPMPrev-RPM)/dt;
  #ifdef FILTERED
    if((RPM>RPMPrev+1)&&abs(RPM-setPoint)<8) RPM = RPMPrev +1;
    if((RPM<RPMPrev-1)&&abs(RPM-setPoint)<8) RPM = RPMPrev -1;
  #endif
  if(output>maxOutput) output = maxOutput;
  if(output<minOutput) output = minOutput;

  RPMPrev = RPM;
  errorPrev = error ; 
}

void setup() {
  Serial.begin(9600);
/*I2C */
 //Wire.begin(SLAVE_ADDRESS); // Initialize I2C communication with the specified address
 //Wire.onReceive(receiveData); // Set the function to be called when data is received



  /*Motor_Setup*/
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  
  pinMode(ENCODER_PINA, INPUT);
  
  digitalWrite(DIR_PIN1, 1);
  digitalWrite(DIR_PIN2, 0);
  
  output = 0;
  //setPoint = 150.0;
  delay(1000);
 
  /*Timer_Interrupts_Setup*/
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), encoderISR, RISING);
  Timer1.initialize(timerPeriod);       // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(timerISR);
}

void PWMCONTROLLER()
{
  double pw = output*2000.0/255.0;
 digitalWrite(PWM_PIN,1);
  delayMicroseconds(int(pw));
 digitalWrite(PWM_PIN,0);
  delayMicroseconds(int(2000-pw));
  
}

void directionController(int x)
{
  switch (x)
    {
      case 0 : 
                setPoint = 0 ;
                digitalWrite(DIR_PIN1, 0);
                digitalWrite(DIR_PIN2, 0);
                break;
      //stop
      case  1 : 
                setPoint = 200 ;
                digitalWrite(DIR_PIN1, 1);
                digitalWrite(DIR_PIN2, 0);
                break;
      //FW
      case 2 : 
            #ifdef RIGHTMOTOR
                setPoint = 200 ;
                digitalWrite(DIR_PIN1, 1);
                digitalWrite(DIR_PIN2,0); 
            #endif
            #ifdef LEFTMOTOR
                setPoint =100 ;
                digitalWrite(DIR_PIN1, 0);
                digitalWrite(DIR_PIN2, 1);
            #endif 
            break;
            //Right
      case 3 : //left
            #ifdef RIGHTMOTOR
                setPoint = 100 ;
                digitalWrite(DIR_PIN1, 0);
                digitalWrite(DIR_PIN2, 1); 
            #endif
            #ifdef LEFTMOTOR
                setPoint = 200 ;
                digitalWrite(DIR_PIN1, 1);
                digitalWrite(DIR_PIN2, 0);
            #endif 
            break;
      default:;
    }
}
void loop() { 
  if (Serial.available())
  {
  x = Serial.read(); 

   if (x == 0 || x == 1 || x == 2 || x == 3)
   {
  Serial.print("x = ");
  Serial.println(x);
   }
   else
   {
     x = 0 ;
   }
  
  }
  PWMCONTROLLER();
}

/*
void receiveData(int byteCount) {
  if (Wire.available()) {
    mega_x = Wire.read(); // Read the received data
  }
}
*/
