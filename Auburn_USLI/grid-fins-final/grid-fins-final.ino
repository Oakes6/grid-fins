/* USLI 2016-2017 Grid Fin Final Code
 *  
 *  Garrett Knecht, Tanner Oakes
 */

#include <Servo.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_10DOF.h>
#include "SD.h"
#include <SPI.h>
#include "RTClib.h"
#include <Wire.h>



 //==================================================================================\\
//================================== INITIALIZATION ==================================\\
//*********************************
//* initializes the accelerometer *
//*   and the altimeter for the   *
//*      10-DOF IMU Breakout      *
//* DO         NOT         CHANGE *
//*********************************


Adafruit_10DOF dof = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);   
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
RTC_DS1307 RTC; // define the Real Time Clock object



//Set our Servo class instances
Servo altServo;
Servo firstServo;
Servo secondServo;


// the logging file
File logfile;

Sd2Card card;
 

//========================================================
//===================== Variables ========================

// -- servo connection ports --
int altServoPort = 8;
int firstServoPort = 11;
int secondServoPort = 9;
// ----------------------------
const int chipSelect = 10;


// A simple data logger for the Arduino analog pins
#define ECHO_TO_SERIAL   0 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

// the digital pins that connect to the LEDs
#define redLEDpin 3
#define greenLEDpin 4

bool hasLaunched = false;                  //bool statement before launch
bool hasBurnedOut = false;                 //bool statement for motor burnout

float accelZ;                               //z acceleration
float altCurrent = 0;                       //current altitude
float altInitial = 0;                       //initial calculated altitude
float altPrevious = 0;                      //previous altitude
float apogee = 4550;                        //desired final height
float calculatedApogee;                     //apogee calculated from previous variables
float startTimer;                           //start of rocket launch
float velTimer1 = 0;                        //initial velocity timer
float velTimer2 = 0;                        //final velocity timer
float velZ = 0;                             //z velocity



int initialAltServoPosition = 0;            //initial angle for servos set to 90 degrees
int altServoPosition = initialAltServoPosition;     //position to store alt servo data
int position;                               //position variable for servo




//===================== Variables ========================
//========================================================
















//========================================================
//===================== Functions ========================


float calcAlt() {

  /* Get a new sensor event (needed for the 10 dof) */ 
  sensors_event_t event;
  bmp.getEvent(&event);

  float alt;

  if (event.pressure)
  {   
     
    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);

    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

    //calc altitude, convert to feet (from meters)
    alt = bmp.pressureToAltitude(seaLevelPressure,event.pressure) * 3.280840;

    return alt;
  }
}






float getAccelZ () {

  float zAccel;

  sensors_event_t event2; 
  accel.getEvent(&event2);

  zAccel = event2.acceleration.z;
  //calc to ft/s
  zAccel *= -3.280840;

  return zAccel;
  
}




float calcVelZ(float timer1, float timer2, float altCurrent, float altPrevious) {
  float velocity;
  float timerDiff;

  //loop timer
  timerDiff = (velTimer2 - velTimer1) / 1000;
  
  //calc vel
  velocity = (altCurrent - altPrevious) / timerDiff;

  return velocity;
}



float calcApogee(float velocity, float accel) {
  float apogee;
  apogee = (velocity*velocity) / (2*accel);
  return apogee;  
}

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  while(1);
}


//===================== Functions ========================
//========================================================





















void setup() {

  //Start serial port
  Serial.begin(9600);
  Serial.println("yo 1");
  

  
//**********************************
//* Initialize and check altimeter *
//**********************************
  if(!bmp.begin())
  {
    // There was a problem detecting the BMP085 ... check your connections 
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  
//**************************************
//* Initialize and check accelerometer *
//**************************************
  if(!accel.begin())
    {
      // There was a problem detecting the ADXL345 ... check your connections
      Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
      while(1);
    }


    
//**************************************
//* Initialize and check gyro *
//**************************************
if(!gyro.begin())
  {
    // There was a problem detecting the ADXL345 ... check your connections 
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  
  //----- Initialization Statements ----
  //------------------------------------



  //------ Set Up Servos ------

  // connect to ports
  altServo.attach(altServoPort);
  //firstServo.attach(firstServoPort);
  //secondServo.attach(secondServoPort);

  Serial.println("yo 2");
  //servo set function
  int pos = 0;
  for (pos = 0; pos <= 90; pos++) {
    altServo.write(pos);
    delay(15);
  }
  for (pos = 90; pos >= 0; pos--) {
    altServo.write(pos);
    delay(30);
  }
  Serial.println("yo 3");

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  

//******
  Serial.println("yo 4");
  float temp;
  Serial.println("yo 5");
  // set initial altitude
  for (int i = 0; i < 20; i++) {
    Serial.println("yo 6");
    //temp = calcAlt();
    altInitial += calcAlt();
    
  }
  Serial.println("yo 7");

  altInitial /= 20;
//******
  Serial.println("yo 8"); 
}












void loop() {

  // hold until rocket launches
  while (!hasLaunched) {
    accelZ = getAccelZ();
    Serial.println(accelZ);
    Serial.println(altServoPosition);
    //if accel is pos, we have launched
    if (accelZ > 0) {
      hasLaunched = true;
      //start timer
      startTimer = millis();
    }
    
  }
//****** Prints telemetric data to card
  logfile = SD.open("flight.txt", FILE_WRITE);
  Serial.print(altServoPosition);
  Serial.println("");
  logfile.print("Current Altitude: ");
  logfile.print(altCurrent);
  logfile.print("\t");
  logfile.print("Velocity: ");
  logfile.print(velZ);
  logfile.print("\t");
  logfile.print("Acceleration: ");
  logfile.print(accelZ);
  logfile.print("\t");
  logfile.print("Apogee: ");
  logfile.print(calculatedApogee);
  logfile.print("\t");
  logfile.print("Timer Difference: ");
  logfile.print(velTimer2 - velTimer1);
  logfile.print("\t");
  logfile.print("alt Servo Angle: ");
  logfile.print(altServoPosition);
  logfile.println("");
  logfile.close();

  //************* ADDED THIS CODE *************
  //calc current accel
  accelZ = getAccelZ();
  if (accelZ < 0) {
    hasBurnedOut = true;
  }
  //************* ADDED THIS CODE *************
  
  //calculated current alt
  altPrevious = altCurrent;
  altCurrent = calcAlt() - altInitial;

  //calculate velocity
  velTimer2 = millis();
  velZ = calcVelZ(velTimer1, velTimer2, altCurrent, altPrevious);
  velTimer1 = millis();


  //calc apogee
  calculatedApogee = calcApogee(velZ, accelZ);

  // motor burnout  
  //if statement only works after burnout and if V < 445
  if (hasBurnedOut && velZ < 445) {

    //servo commands go in here
    //increase pitch
    if (calculatedApogee > (apogee + 5) && altServoPosition < initialAltServoPosition + 45) {
      altServoPosition += 2;
      position = altServoPosition;
      altServo.write(position);
      delay(5);
    }

    //decrease pitch
    if (calculatedApogee < (apogee - 5) && altServoPosition > initialAltServoPosition) {
      altServoPosition -= 2;
      position = altServoPosition;
      altServo.write(position);
      delay(5);
    }

    //if we're going way too low
    if (calculatedApogee < (apogee - 20)) {
      while (altServoPosition > initialAltServoPosition) {
        altServoPosition -= 2;
        position = altServoPosition;
        altServo.write(position);
        delay(5);
      }
    }
  
  }
}
