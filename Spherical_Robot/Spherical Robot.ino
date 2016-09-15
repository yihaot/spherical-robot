//connect your bluetooth HC-05 module to the RX and TX pins
//the 16ch servo driver uses I2C, address 0x40
//the 10dof imu sensor uses I2C, address 0x6b

//FOR LED OUTPUT
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#define PIN            6 //LED PIN
#define NUMPIXELS      8
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//FOR BLUETOOTH
#define BTpin 12 //use as 5V supply for Bluetooth module
int i = 0; //for tracking how many times there is no response from BT or no finger press

//state machine
int stay = 0;
int start = 1;

//FOR PID CONTROL
#define Px 1
#define Py 1
#define Pz 1
#define baseServoSpeed 1

//FOR SERVO CONTROL
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//#define SERVOMIN  400 //130 // this is the 'minimum' pulse length count (out of 4096)
int servoMini[] = {110, 118, 135, 118, 230, 217, 110, 80, 95, 130, 95, 90} ; //servo min for each servo (0-5 at 10 deg)
int servoMaxi[] = {485, 510, 535, 525, 620, 590, 520, 565, 530, 535, 515, 540} ; //servo max for each servo
//#define SERVOMAX  490 //520 // this is the 'maximum' pulse length count (out of 4096)
// our servo # counter
uint8_t servonum = 0;
int nt = 90;
int lastServoPos[] = {nt, nt, nt, nt, nt, nt, nt, nt, nt, nt, nt, nt, nt, nt, nt, nt} ;
#define numOfServo 16
//#define midPt map(90, 0, 180, SERVOMIN, SERVOMAX)
int direct = -1; //sets the direction as -1neutral,6forward,24FL,30L,96RL,102reverse,120RR,126right,128FR,224stop
int oldDirect = -1; //tracks last known direction
#define servoFwdAngle 10 //how much the servo turns to move forward
#define servoRevAngle 150 //how much the servo turns to move backwards
#define neutralAngle  90//what is the 90deg angle
#define trimAngle1 0 //how much to trim the servo by
#define trimAngle2 0
int servoNumber2 = 0;
int servoNumber3 = 0;

//FOR SPEED CONTROL
int servoSpeedMultiple = 1;
float Wx = 0;
int baseSpeed = 1;
#define desRollSpeed 5 //desired rolling speed of the ball in rad/s when forward button is pressed
#define desRollSpeedThres 0.5 //threshold to deactivate/activate servo in power saving mode
#define speedThres1 100 //rad/s speed to control servo speed
#define speedThres2 300
#define speedThres3 350
#define speedThres4 450
#define speedThres5 500
#define speedThres6 600



//FOR IMU READING
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
//#include "RTPressure.h"
#include "CalLib.h"
#include <EEPROM.h>
RTIMU * imu;                                          // the IMU object
//RTPressure *pressure;                                 // the pressure object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object
//  DISPLAY_INTERVAL sets the rate at which results are displayed
#define DISPLAY_INTERVAL  10                         // interval between pose displays
//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  19200
unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;
RTFLOAT x_roll;
RTFLOAT y_pitch;
RTFLOAT z_yaw;
int sector = 17; //sector to keep track of its current sector position
int oldSector = 18;
//get gyro data
#include <Adafruit_L3GD20_U.h>
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);



void setup() {

  //LED OUTPUT
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setPixelColor(i, pixels.Color(0, 0, 150)); // Moderately bright green color.
  pixels.show();

  //FOR BLUETOOTH CONTROL
  pinMode(BTpin, OUTPUT);
  digitalWrite(BTpin, HIGH);
  //Serial.begin(9600); //for bluetooth module

  //FOR SERVO CONTROL
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  yield();

  //Servo initialisation to mid-point
  //  for (int i = 0; i <= numOfServo; i++) {
  //    servoMove(i, 100);
  //  }
  servoF(0, neutralAngle); //90
  servoF(1, neutralAngle);
  servoF(2, neutralAngle);
  servoF(3, neutralAngle);
  servoF(4, neutralAngle);
  servoF(5, neutralAngle);



  //FOR IMU READING
  int errcode;

  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  imu = RTIMU::createIMU(&settings);                        // create the imu object
  //  pressure = RTPressure::createPressure(&settings);         // create the pressure sensor
  //
  //  if (pressure == 0) {
  //    Serial.println("No pressure sensor has been configured - terminating");
  //    while (1) ;
  //  }

  Serial.print("ArduinoIMU10 starting using IMU "); Serial.print(imu->IMUName());
  //  Serial.print(", pressure sensor "); Serial.println(pressure->pressureName());
  if ((errcode = imu->IMUInit()) < 0) {
    Serial.print("Failed to init IMU: "); Serial.println(errcode);
  }

  //  if ((errcode = pressure->pressureInit()) < 0) {
  //    Serial.print("Failed to init pressure sensor: "); Serial.println(errcode);
  //  }

  if (imu->getCalibrationValid())
    Serial.println("Using compass calibration");
  else
    Serial.println("No valid compass calibration data");

  lastDisplay = lastRate = millis();
  sampleCount = 0;
  fusion.setSlerpPower(0.02);
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);

  start = 0;
}



void loop() {


  //FOR BLUETOOTH CONTROL
  if (Serial.available()) { //to receive bluetooth data
    // read the incoming byte:
    direct = int(Serial.read());
    //Serial.println(Serial.read());

    // say what you got:
    //Serial.print("I received: ");
    //Serial.println(direct);
    i = 0;
  }
  else {
    i += 1;
    if (i > 30) direct = -1 ;
  }
  //direct = 6;
  //Serial.println(direct);


  //FOR IMU READING
  unsigned long now = millis();
  unsigned long delta;
  //  float latestPressure;
  //  float latestTemperature;
  int loopCount = 1;
  while (imu->IMURead()) {                                // get the latest data if ready yet
    // this flushes remaining data in case we are falling behind
    if (++loopCount >= 10)
      continue;
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;
    if ((delta = now - lastRate) >= 1000) {
      sampleCount = 0;
      lastRate = now;
    }
    if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
      lastDisplay = now;
      RTMath::getRollPitchYaw((RTVector3&)fusion.getFusionPose(), x_roll, y_pitch, z_yaw);
      //Serial.println(x_roll);
      //Serial.print (",X_Roll:"); Serial.println(x_roll); // to get the roll value
      //            Serial.print (",Y_Pitch:"); Serial.print(y_pitch); // to get the pitch value
      //            Serial.print (",Z_Raw:"); Serial.print(z_yaw); // to get the yaw value

      // Serial.print(((RTVector3&)fusion.getFusionPose()).x());


      //      if (pressure->pressureRead(latestPressure, latestTemperature)) {
      //        Serial.print(", pressure: "); Serial.print(latestPressure);
      //        Serial.print(", temperature: "); Serial.print(latestTemperature);
      //      }
      //      Serial.println();
    }
  }


  if (direct == 6) { //[button forward detected]
    if (stay == 1) {
      //      servoF(0, neutralAngle); //90
      //      servoF(1, neutralAngle);
      //      servoF(2, neutralAngle);
      //      servoF(3, neutralAngle);
      //      servoF(4, neutralAngle);
      //      servoF(5, neutralAngle);
      servoKeep();
      servoKeep();
      stay = 0;
    }
    else
      FRmove(1, servoFwdAngle);
    Serial.println(sector);
  }
  else if (direct == 128) { //[stop button pressed]
    Serial.println("STOPPPP");
    if (stay == 0) {
      standing();
      stay = 1;
      Serial.println("STOPPPPnow");
    }
  }

  //  else if (direct == 102) { //[move reverse]
  //    FRmove(5, servoRevAngle);
  //  }
  else if ((direct == 30)) { //[move forward left]
    if (stay == 1) {
      //      servoF(0, neutralAngle); //90
      //      servoF(1, neutralAngle);
      //      servoF(2, neutralAngle);
      //      servoF(3, neutralAngle);
      //      servoF(4, neutralAngle);
      //      servoF(5, neutralAngle);
      servoKeep();
      servoKeep();
      stay = 0;
    }
    else
      LRmove(0, servoFwdAngle); //leftServos =1 means using right servos, then set it to go fwd or back (using one side servo to change direction)
  }
  //  else if (direct == 96) { //[move reverse left]
  //    LRmove(0, servoRevAngle);
  //  }
  else if ((direct == 126)) { //[move forward right]
    if (stay == 1) {
      //      servoF(0, neutralAngle); //90
      //      servoF(1, neutralAngle);
      //      servoF(2, neutralAngle);
      //      servoF(3, neutralAngle);
      //      servoF(4, neutralAngle);
      //      servoF(5, neutralAngle);
      servoKeep();
      servoKeep();
      stay = 0;
    }
    else
      LRmove(1, (180 - servoFwdAngle));
  }
  //  else if (direct == 120) { //[move reverse right]
  //    LRmove(1, servoRevAngle);
  //  }

  Serial.println(stay);

  if (sector != oldSector) Serial.println(sector);
  oldSector = sector;
  oldDirect = direct;

}


bool speedChecker() { //to check if the current rolling speed is with in the threshold, if too fast, don't need servo, if too slow, need servo
  //read gyro data
  //  sensors_event_t event;
  //  gyro.getEvent(&event);
  //  Wx = event.gyro.x; //our current angular speed
//return true;
  if (Wx > 200) { //false = don't need to activate servo as its over upper ceiling
    return false;
  }
  else if (Wx <= 200) { //true = activate servo as its below lower ceiling
    return true;
  }
}

void speedo() { //change the speed according to the rad/s
  if (Wx < speedThres1) {
    for (int i = 0; i < 8; i++) {
      pixels.setPixelColor(i, pixels.Color(150, 0, 0));
    } //red
  }
  else if (Wx < speedThres2) {
    for (int i = 0; i < 8; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 150, 0));
    } //green
  }
  else if (Wx < speedThres3) {
    for (int i = 0; i < 8; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, 150));
    } //blue
  }
  else if (Wx < speedThres4) {
    for (int i = 0; i < 8; i++) {
      pixels.setPixelColor(i, pixels.Color(150, 150, 0));
    } //yellow
  }
  else if (Wx < speedThres5) {
    for (int i = 0; i < 8; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 150, 150));
    } //cyan
  }
  else if (Wx < speedThres6) {
    for (int i = 0; i < 8; i++) {
      pixels.setPixelColor(i, pixels.Color(150, 0, 150));
    } //purple
  }
  else if (Wx < 1100) {
    for (int i = 0; i < 8; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 255, 0));
    } //bright yellow
  }
  else if (Wx > 1200) {
    for (int i = 0; i < 8; i++) {
      pixels.setPixelColor(i, pixels.Color(150, 150, 150));
    } // white
  }

  pixels.show(); // This sends the updated pixel color to the hardware.
}

void speedControl() {

  //read gyro data
  sensors_event_t event;
  gyro.getEvent(&event);
  Wx = event.gyro.x; //our current angular speed
  speedo();
  // Serial.print(F("GYRO  "));
  //Serial.print("X: "); Serial.print(event.gyro.x); Serial.println("  "); //angular velocity of X
  //  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  //  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s ");

  if (abs(Wx) < speedThres1) {
    servoSpeedMultiple = 1;
  }
  else if (abs(Wx) < speedThres2) {
    servoSpeedMultiple = 2;
  }
  else if (abs(Wx) < speedThres3) {
    servoSpeedMultiple = 3;
  }
  else if (abs(Wx) < speedThres4) {
    servoSpeedMultiple = 4;
  }
  else if (abs(Wx) < speedThres5) {
    servoSpeedMultiple = 5;
  }
  else if (abs(Wx) >= speedThres5) {
    servoSpeedMultiple = 6;
  }
}


void FRmove(int inputDirection, int servoAngle) { //code to move forward or backward
  //mapping imu to servo movement (FOR 6 SERVO PER SIDE ONLY)
  //Serial.println(x_roll);
  speedControl(); //to change the servo speed
  if (speedChecker() == true) { //only activates if it's below speed threshold
    sectorChecker();
    if ((sector == oldSector) && (direct == oldDirect)) {
    }
    else if (abs(Wx) < speedThres1) {
      servoF(sector, servoFwdAngle); //servo number is the sector number
    }
    else if (abs(Wx) < speedThres2) {
      servoF((sector - 0), servoFwdAngle); //servo number is the sector number
    }
    else if (abs(Wx) < speedThres3) {
      servoF((sector - 0), servoFwdAngle); //servo number is the sector number
    }
    //  else if (abs(Wx) < speedThres4) {
    //    servoF((sector-3), servoFwdAngle); //servo number is the sector number
    //  }
    //  else if (abs(Wx) < speedThres5) {
    //    servoF((sector-4), servoFwdAngle); //servo number is the sector number
    //  }
    //  else if (abs(Wx) >= speedThres5) {
    //    servoF((sector-5), servoFwdAngle); //servo number is the sector number
    //  }

  }
}

void sectorChecker() {
  RTMath::getRollPitchYaw((RTVector3&)fusion.getFusionPose(), x_roll, y_pitch, z_yaw);
  switch (int(x_roll)) {
    case 0 ... 60:
      sector = 2;
      break;
    case 61 ... 120:
      sector = 1;
      break;
    case 121 ... 180:
      sector = 0;
      break;
    case -180 ... -121:
      sector = 5;
      break;
    case -120 ... -61:
      sector = 4;
      break;
    case -60 ... -1:
      sector = 3;
      break;
  }
}


void LRmove(int leftServos, int servoAngle) { //code to move forward left,forward right,reverse left or reverse right, leftServos? =0 means using right servos
  speedControl();
  sectorChecker();
  if ((sector == oldSector) && (direct == oldDirect)) {
  }
  else {
    servoLR((sector + 6 * leftServos), servoAngle);
  }

}


void servoLR(int servoNumber, int deg) { //code to move each servo
  if (servoNumber == -1) servoNumber = 5;
  else if (servoNumber == -2) servoNumber = 4;
  else if (servoNumber == -3) servoNumber = 3;
  int prevSector = sector;

  //if (abs(Wx) < speedThres1) {
  if (deg >= 90) {
    Serial.print(servoNumber);
    Serial.print(" activated to go: ");
    Serial.println(deg);
    for (uint16_t servoPos = 90; servoPos <= deg; servoPos += servoSpeedMultiple) { //moves it to desired position
      RTMath::getRollPitchYaw((RTVector3&)fusion.getFusionPose(), x_roll, y_pitch, z_yaw);
      Serial.println(x_roll);
      sectorChecker();
      if ((sector == prevSector) | (start == 1)) {
        pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
      }
      else break;
    }
    for (uint16_t servoPos = deg; servoPos > 90 ; servoPos -= 2) { //returns it back to 90deg
      pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber])); //comment this out to do servo angle calibration
    }
  }
  else if (deg < 90) {
    Serial.print(servoNumber);
    Serial.print(" activated to gogo: ");
    Serial.println(deg);
    for (uint16_t servoPos = 90; servoPos >= deg; servoPos -= servoSpeedMultiple) { //moves it to desired position
      RTMath::getRollPitchYaw((RTVector3&)fusion.getFusionPose(), x_roll, y_pitch, z_yaw);
      Serial.println(x_roll);
      sectorChecker();
      if ((sector == prevSector) | (start == 1)) {
        pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
      }
      else break;
    }
    for (uint16_t servoPos = deg; servoPos < 90 ; servoPos += 2) { //returns it back to 90deg
      pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber])); //comment this out to do servo angle calibration
    }



    // }
  }
}


void standing() {
  Serial.println("STANDING");
  speedControl();
  sectorChecker();
  servoHold(sector, servoFwdAngle);
}







void servoF(int servoNumber, int deg) { //code to move each servo forward (speed of servo output)
  //  uint16_t pulselen = map(deg, 0, 180, SERVOMIN, SERVOMAX);
  if (servoNumber == -1) servoNumber = 5;
  else if (servoNumber == -2) servoNumber = 4;
  else if (servoNumber == -3) servoNumber = 3;
  int prevSector = sector;

  if (abs(Wx) < speedThres1) {
    if (deg >= 90) {
      Serial.print(servoNumber);
      Serial.print(" activated to go: ");
      Serial.println(deg);
      for (uint16_t servoPos = 90; servoPos <= deg; servoPos += servoSpeedMultiple) { //moves it to desired position
        RTMath::getRollPitchYaw((RTVector3&)fusion.getFusionPose(), x_roll, y_pitch, z_yaw);
        Serial.println(x_roll);
        sectorChecker();
        if ((sector == prevSector) | (start == 1)) {
          pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
          pwm.setPWM((servoNumber + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)])));
          pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
          pwm.setPWM((servoNumber + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)])));
          pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
          pwm.setPWM((servoNumber + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)])));

        }
        else break;
      }
      for (uint16_t servoPos = deg; servoPos > 90 ; servoPos -= 2) { //returns it back to 90deg
        pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber])); //comment this out to do servo angle calibration
        pwm.setPWM((servoNumber + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)]))); //comment this out to do servo angle calibration

      }
    }
    else if (deg < 90) {
      Serial.print(servoNumber);
      Serial.print(" activated to gogo: ");
      Serial.println(deg);
      for (uint16_t servoPos = 90; servoPos >= deg; servoPos -= servoSpeedMultiple) { //moves it to desired position
        RTMath::getRollPitchYaw((RTVector3&)fusion.getFusionPose(), x_roll, y_pitch, z_yaw);
        Serial.println(x_roll);
        sectorChecker();
        if ((sector == prevSector) | (start == 1)) {
          pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
          pwm.setPWM((servoNumber + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)])));
          pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
          pwm.setPWM((servoNumber + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)])));
          pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
          pwm.setPWM((servoNumber + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)])));

        }
        else break;

      }
      for (uint16_t servoPos = deg; servoPos < 90 ; servoPos += 2) { //returns it back to 90deg
        pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber])); //comment this out to do servo angle calibration
        pwm.setPWM((servoNumber + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)]))); //comment this out to do servo angle calibration

      }
    }
  }


  else {
    if (deg >= 90) {
      Serial.print(servoNumber);
      Serial.print(" activated to go: ");
      Serial.println(deg);
      for (uint16_t servoPos = 90; servoPos <= deg; servoPos += servoSpeedMultiple) { //moves it to desired position
        RTMath::getRollPitchYaw((RTVector3&)fusion.getFusionPose(), x_roll, y_pitch, z_yaw);
        Serial.println(x_roll);
        sectorChecker();
        if ((sector == prevSector) | (start == 1)) {
          pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
          pwm.setPWM((servoNumber + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)])));
        }

        else break;

      }
      for (uint16_t servoPos = deg; servoPos > 90 ; servoPos -= 5) { //returns it back to 90deg
        pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
        pwm.setPWM((servoNumber + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)])));

      }
    }
    else if (deg < 90) {
      Serial.print(servoNumber);
      Serial.print(" activated to gogo: ");
      Serial.println(deg);
      for (uint16_t servoPos = 90; servoPos >= deg; servoPos -= servoSpeedMultiple) { //moves it to desired position
        RTMath::getRollPitchYaw((RTVector3&)fusion.getFusionPose(), x_roll, y_pitch, z_yaw);
        Serial.println(x_roll);
        sectorChecker();
        if ((sector == prevSector) | (start == 1)) {
          pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
          pwm.setPWM((servoNumber + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)])));
        }

        else break;

      }
      for (uint16_t servoPos = deg; servoPos < 90 ; servoPos += 5) { //returns it back to 90deg
        pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
        pwm.setPWM((servoNumber + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)])));

      }
    }
  }

  //lastServoPos[servoNumber] = 90; //reset array back to 90deg
}

void servoHold(int servoNumber, int deg) {
  //Serial.println("SERVOHOLD");
  //  int prevSector = sector;
  //  servoNumber -= 1;
  //  if (servoNumber == -1) servoNumber = 5;
  //  if (servoNumber == 5) {
  //    servoNumber2 = 0;
  //  }
  //  else servoNumber2 = servoNumber + 1;
  //  if (servoNumber2 == 5) {
  //    servoNumber3 = 0;
  //  }
  //  else servoNumber3 = servoNumber2 + 1;

  //  for (uint16_t servoPos = 90; servoPos >= deg; servoPos -= servoSpeedMultiple) { //moves it to desired position
  //    pwm.setPWM(servoNumber, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
  //    pwm.setPWM((servoNumber + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)])));
  //    pwm.setPWM(servoNumber2, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
  //    pwm.setPWM((servoNumber2 + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)])));
  //    pwm.setPWM(servoNumber3, 0, map((servoPos - trimAngle1), 0, 180, servoMini[servoNumber], servoMaxi[servoNumber]));
  //    pwm.setPWM((servoNumber3 + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(servoNumber + 6)], servoMaxi[(servoNumber + 6)])));

  Serial.print(servoNumber);
  Serial.print(" stand ");
  Serial.println(deg);
  for (uint16_t servoPos = 90; servoPos >= deg; servoPos -= servoSpeedMultiple) { //moves it to desired position
    //      RTMath::getRollPitchYaw((RTVector3&)fusion.getFusionPose(), x_roll, y_pitch, z_yaw);
    //      Serial.println(x_roll);
    //      sectorChecker();
    for (int i = 0; i <= 5; i++) { //deploys all servos
      pwm.setPWM(i, 0, map((servoPos - trimAngle1), 0, 180, servoMini[i], servoMaxi[i]));
      pwm.setPWM((i + 6), 0, (map((180 - servoPos + trimAngle2), 0, 180, servoMini[(i + 6)], servoMaxi[(i  + 6)])));

    }

  }
}
void servoKeep() {
  for (int i = 0 ; i <= 11; i++) {
    pwm.setPWM(i, 0, map(neutralAngle, 0, 180, servoMini[i], servoMaxi[i]));
  }
}

