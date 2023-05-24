#include <Wire.h>


#define SLAVE_ADDRESS1 0x12  // Address of the slave 1 right motor
#define PROXIMITY_PIN 7
 

const int x = 1; 
const int y = 2; 
const int z = 3; 
const int a = 0 ;
int proximityReading;
void setup() {
  pinMode(PROXIMITY_PIN,INPUT);
  Wire.begin();        // Initialize I2C communication
  Serial.begin(9600);  // initialize serial communication
}

void loop() {
  proximityReading = digitalRead(PROXIMITY_PIN);
  if(proximityReading == 0)
  {     
        Serial.println("Proximity Case");
        Wire.beginTransmission(SLAVE_ADDRESS1);
        Wire.write(a);  // setpoint1 must be > setpoint2 to turn left
        Wire.endTransmission();
  }
  else 
  {
      if (Serial.available() > 0) {        // if data is available to read
    //  Serial.println("We are here ");
      long error_x = Serial.parseInt();  // read the integer value from serial
      if (error_x != 0) {
        if (error_x > 50) {
          // **********************************************turn RIGHT code********************************************************************//
          Serial.println("Turn RIGHT\n");
          Wire.beginTransmission(SLAVE_ADDRESS1);
          Wire.write(z);  // setpoint1 must be > setpoint2 to turn left
          Wire.endTransmission();
  
        } else if (error_x < -50) {
          // **********************************************turn LEFT code********************************************************************//
          Serial.println("Turn LEFT");
          Wire.beginTransmission(SLAVE_ADDRESS1);
          Wire.write(y);  // setpoint1 must be < setpoint2 to turn right
          Wire.endTransmission();
       
          
        } else if (error_x >= 50 || error_x <= 50) {
          // **********************************************Forward code******************************************************************** //
          Serial.println("Move Forward\n");
         Wire.beginTransmission(SLAVE_ADDRESS1);
         Wire.write(x);  // setpoint1 must be = setpoint2 to move forward
         Wire.endTransmission();
        }
      }
    } else
      {
        Serial.println("No Human Detected");
         Wire.beginTransmission(SLAVE_ADDRESS1);
         Wire.write(a);  // setpoint1 must be = setpoint2 to move forward
         Wire.endTransmission();
      }
  }

}
