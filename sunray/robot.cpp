// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "robot.h"
#include "comm.h"
#include "WiFiEsp.h"
#include "SparkFunMPU9250-DMP.h"
#include "pinman.h"
#include "ble.h"
#include "motor.h"
#include "battery.h"
#include "ublox.h"
#include "buzzer.h"
#include "map.h"
#include "config.h"
#include "helper.h"
#include "pid.h"
#include "i2c.h"
#include <Arduino.h>

// #define I2C_SPEED  10000
#define _BV(x) (1 << (x))


MPU9250_DMP imu;
Motor motor;
Battery battery;
PinManager pinMan;
UBLOX gps(GPS,GPS_BAUDRATE);
BLEConfig bleConfig;
Buzzer buzzer;
Map maps;
PID pidLine(0.2, 0.01, 0); // not used
PID pidAngle(2, 0.1, 0);  // not used

OperationType stateOp = OP_IDLE; // operation-mode
Sensor stateSensor = SENS_NONE; // last triggered sensor
unsigned long controlLoops = 0;
float stateX = 0;  // position-east (m)
float stateY = 0;  // position-north (m)
float stateDelta = 0;  // direction (rad)
float stateDeltaGPS = 0;
float stateDeltaIMU = 0;
float stateGroundSpeed = 0; // m/s
float setSpeed = 0.1; // linear speed (m/s)
unsigned long stateLeftTicks = 0;
unsigned long stateRightTicks = 0;
unsigned long lastFixTime = 0;
int fixTimeout = 0;
bool absolutePosSource = false;
double absolutePosSourceLon = 0;
double absolutePosSourceLat = 0;
bool finishAndRestart = false;

UBLOX::SolType lastSolution = UBLOX::SOL_INVALID;    
unsigned long nextStatTime = 0;
unsigned long statIdleDuration = 0; // seconds
unsigned long statChargeDuration = 0; // seconds
unsigned long statMowDuration = 0; // seconds
unsigned long statMowDurationFloat = 0; // seconds
unsigned long statMowDurationFix = 0; // seconds
unsigned long statMowFloatToFixRecoveries = 0; // counter
unsigned long statImuRecoveries = 0; // counter
float statMowMaxDgpsAge = 0; // seconds
float statMowDistanceTraveled = 0; // meter


float lastPosN = 0;
float lastPosE = 0;


unsigned long linearMotionStartTime = 0;
unsigned long nextControlTime = 0;
unsigned long lastComputeTime = 0;

unsigned long nextImuTime = 0;
unsigned long imuDataTimeout = 0;
bool imuFound = false;
float lastIMUYaw = 0; 

bool wifiFound = false;
char ssid[] = WIFI_SSID;      // your network SSID (name)
char pass[] = WIFI_PASS;        // your network password
WiFiEspServer server(80);
WiFiEspClient client = NULL;
int status = WL_IDLE_STATUS;     // the Wifi radio's status


// get free memory
int freeMemory() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}

// reset motion measurement
void resetMotionMeasurement(){
  linearMotionStartTime = millis();  
  //stateGroundSpeed = 1.0;
}


void startWIFI(){
  WIFI.begin(WIFI_BAUDRATE); 
  WIFI.print("AT\r\n");  
  delay(500);
  String res = "";  
  while (WIFI.available()){
    char ch = WIFI.read();    
    res += ch;
  }
  if (res.indexOf("OK") == -1){
    CONSOLE.println("WIFI (ESP8266) not found!");
    return;
  }    
  WiFi.init(&WIFI);  
  if (WiFi.status() == WL_NO_SHIELD) {
    CONSOLE.println("ERROR: WiFi not present");       
  } else {
    wifiFound = true;
    CONSOLE.println("WiFi found!");       
    if (START_AP){
      CONSOLE.print("Attempting to start AP ");  
      CONSOLE.println(ssid);
      // uncomment these two lines if you want to set the IP address of the AP
      #ifdef WIFI_IP  
        IPAddress localIp(WIFI_IP);
        WiFi.configAP(localIp);  
      #endif            
      // start access point
      status = WiFi.beginAP(ssid, 10, pass, ENC_TYPE_WPA2_PSK);         
    } else {
      while ( status != WL_CONNECTED) {
        CONSOLE.print("Attempting to connect to WPA SSID: ");
        CONSOLE.println(ssid);      
        status = WiFi.begin(ssid, pass);
        #ifdef WIFI_IP  
          IPAddress localIp(WIFI_IP);
          WiFi.config(localIp);  
        #endif
      }    
    }    
    CONSOLE.print("You're connected with SSID=");    
    CONSOLE.print(WiFi.SSID());
    CONSOLE.print(" and IP=");        
    IPAddress ip = WiFi.localIP();    
    CONSOLE.println(ip);   
    server.begin();
  }    
}


// https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide#using-the-mpu-9250-dmp-arduino-library
// start IMU sensor and calibrate
void startIMU(bool forceIMU){    
  // detect MPU9250
  uint8_t data = 0;
  int counter = 0;  
  while ((forceIMU) || (counter < 1)){          
     I2CreadFrom(0x69, 0x75, 1, &data, 1); // whoami register
     CONSOLE.print(F("MPU ID=0x"));
     CONSOLE.println(data, HEX);     
     #if defined MPU6050 || defined MPU9150      
       if (data == 0x68) {
         CONSOLE.println("MPU6050/9150 found");
         imuFound = true;
         break;
       }
     #endif
     #if defined MPU9250 
       if (data == 0x73) {
         CONSOLE.println("MPU9255 found");
         imuFound = true;
         break;
       } else if (data == 0x71) {
         CONSOLE.println("MPU9250 found");
         imuFound = true;
         break;
       }
     #endif
     CONSOLE.println(F("MPU6050/9150/9250/9255 not found - Did you connect AD0 to 3.3v and choose it in config.h?"));          
     I2Creset();  
     Wire.begin();    
     #ifdef I2C_SPEED
       Wire.setClock(I2C_SPEED);   
     #endif
     counter++;
  }  
  if (!imuFound) return;  
  while (true){
    if (imu.begin() == INV_SUCCESS) break;
    CONSOLE.print("Unable to communicate with IMU.");
    CONSOLE.print("Check connections, and try again.");
    CONSOLE.println();
    delay(1000);
  }            
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT  // Enable 6-axis quat
               |  DMP_FEATURE_GYRO_CAL // Use gyro calibration
              , 5); // Set DMP FIFO rate to 5 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive  
  CONSOLE.print("IMU gyro calibration - robot must be static for 8 seconds");
  for (int i=0; i < 9; i++){
    CONSOLE.print(".");    
    buzzer.sound(SND_PROGRESS, true);        
    unsigned long stopTime = millis() + 1000;
    while (millis() < stopTime) buzzer.run();
  }
  CONSOLE.println();    
  imu.resetFifo();
  imuDataTimeout = millis() + 1000;
}


// read IMU sensor (and restart if required)
void readIMU(){
  if (!imuFound) return;
  // Check for new data in the FIFO  
  unsigned long startTime = millis();
  bool avail = (imu.fifoAvailable() > 0);
  // check time for I2C access : if too long, there's an I2C issue and we need to restart I2C bus...
  unsigned long duration = millis() - startTime;    
  //CONSOLE.print("duration:");
  //CONSOLE.println(duration);  
  if (duration > 10){
    CONSOLE.print("ERROR IMU timeout: ");
    CONSOLE.println(duration);          
    motor.stopImmediately(true);    
    startIMU(true); // restart I2C bus
    statImuRecoveries++;
    resetMotionMeasurement();
    return;
  }      
  if (avail) {    
    //CONSOLE.println("fifoAvailable");
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {      
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      //CONSOLE.print("imu data: ");
      //CONSOLE.println(imu.yaw);
      stateDeltaIMU = distancePI(imu.yaw/180.0*PI, lastIMUYaw);  
      lastIMUYaw = imu.yaw/180.0*PI;      
      imuDataTimeout = millis() + 10000;
    }     
  }  
  if (millis() > imuDataTimeout){
    // no IMU data within timeout - this should not happen (I2C module error)
    CONSOLE.println("ERROR IMU data timeout");
    stateSensor = SENS_IMU_TIMEOUT;
    setOperation(OP_ERROR);
    //buzzer.sound(SND_STUCK, true);        
  }
}

// check for RTC module
bool checkAT24C32() {
  byte b = 0;
  int r = 0;
  unsigned int address = 0;
  Wire.beginTransmission(AT24C32_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Wire.beginTransmission(AT24C32_ADDRESS);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(AT24C32_ADDRESS, 1);
      while (Wire.available() > 0 && r < 1) {        
        b = (byte)Wire.read();        
        r++;
      }
    }
  }
  return (r == 1);
}

// robot start routine
void start(){  
  pinMan.begin();     
  // keep battery switched ON
  pinMode(pinBatterySwitch, OUTPUT);    
  digitalWrite(pinBatterySwitch, HIGH);  
  buzzer.begin();      
  CONSOLE.begin(CONSOLE_BAUDRATE);  
  Wire.begin();      
  unsigned long timeout = millis() + 2000;
  while (millis() < timeout){
    if (!checkAT24C32()){
      CONSOLE.println(F("PCB not powered ON or RTC module missing"));      
      I2Creset();  
      Wire.begin();    
      #ifdef I2C_SPEED
        Wire.setClock(I2C_SPEED);     
      #endif
    } else break;
  }
  CONSOLE.println(VER);          
  battery.begin();      
  
  bleConfig.run();   
  BLE.println(VER);  
    
  motor.begin();
  gps.begin();   
  maps.begin();
  
  startIMU(false);      
  
  // initialize ESP module
  startWIFI();  
  
  buzzer.sound(SND_READY);  
  battery.allowSwitchOff(true);
}

// calculate statistics
void calcStats(){
  if (millis() >= nextStatTime){
    nextStatTime = millis() + 1000;
    switch (stateOp){
      case OP_IDLE:
        statIdleDuration++;
        break;
      case OP_MOW:      
        statMowDuration++;
        if (gps.solution == UBLOX::SOL_FIXED) statMowDurationFix++;
          else if (gps.solution == UBLOX::SOL_FLOAT) statMowDurationFloat++;   
        if (gps.solution != lastSolution){      
          if ((lastSolution == UBLOX::SOL_FLOAT) && (gps.solution == UBLOX::SOL_FIXED)) statMowFloatToFixRecoveries++;
          lastSolution = gps.solution;
        } 
        statMowMaxDgpsAge = max(statMowMaxDgpsAge, (millis() - gps.dgpsAge)/1000.0);        
        break;
      case OP_CHARGE:
        statChargeDuration++;
        break;
    }     
  }   
}


// compute robot state (x,y,delta)
void computeRobotState(){  
  long leftDelta = motor.motorLeftTicks-stateLeftTicks;
  long rightDelta = motor.motorRightTicks-stateRightTicks;  
  stateLeftTicks = motor.motorLeftTicks;
  stateRightTicks = motor.motorRightTicks;    
    
  float distLeft = ((float)leftDelta) / ((float)motor.ticksPerCm);
  float distRight = ((float)rightDelta) / ((float)motor.ticksPerCm);  
  float dist  = (distLeft + distRight) / 2.0;
  float deltaOdometry = -(distLeft - distRight) / motor.wheelBaseCm;    
  
  float posN = 0;
  float posE = 0;
  if (absolutePosSource){
    relativeLL(absolutePosSourceLat, absolutePosSourceLon, gps.lat, gps.lon, posN, posE);    
  } else {
    posN = gps.relPosN;  
    posE = gps.relPosE;     
  }   
  
  if ((gps.solutionAvail) 
      && ((gps.solution == UBLOX::SOL_FIXED) || (gps.solution == UBLOX::SOL_FLOAT))  ){
    gps.solutionAvail = false;        
    stateGroundSpeed = 0.5 * stateGroundSpeed + 0.5 * gps.groundSpeed;    
    //CONSOLE.println(stateGroundSpeed);
    float dist = sqrt( sq(posN-lastPosN)+sq(posE-lastPosE) );
    if (dist > 0.3){
      lastPosN = posN;
      lastPosE = posE;
    } else if (dist > 0.1){      
      if (abs(motor.linearSpeedSet) > 0){       
        stateDeltaGPS = scalePI(atan2(posN-lastPosN, posE-lastPosE));    
        //stateDeltaGPS = scalePI(2*PI-gps.heading+PI/2);
        float diffDelta = distancePI(stateDelta, stateDeltaGPS);                 
        if (abs(diffDelta/PI*180) > 45){
          stateDelta = stateDeltaGPS;
        } else {
          // delta fusion
          stateDeltaGPS = scalePIangles(stateDeltaGPS, stateDelta);
          stateDelta = fusionPI(0.9, stateDelta, stateDeltaGPS);               
        }
      }
      lastPosN = posN;
      lastPosE = posE;
    } 
    if (gps.solution == UBLOX::SOL_FIXED) {
      // fix
      lastFixTime = millis();
      stateX = posE;
      stateY = posN;        
    } else {
      // float
      //stateX += dist/100.0 * cos(stateDelta);
      //stateY += dist/100.0 * sin(stateDelta);        
      stateX = posE;
      stateY = posN;        
    }
  } else {     
    // odometry
    stateX += dist/100.0 * cos(stateDelta);
    stateY += dist/100.0 * sin(stateDelta);        
    if (stateOp == OP_MOW) statMowDistanceTraveled += dist/100.0;
  }   
  if (imuFound){
    // IMU available
    stateDelta = scalePI(stateDelta + stateDeltaIMU );  
    stateDeltaIMU = 0;
  } else {
    // odometry
    stateDelta = scalePI(stateDelta + deltaOdometry);  
  }
}


// control robot velocity (linear,angular)
void controlRobotVelocity(){  
  if (millis() >= nextControlTime){        
    nextControlTime = millis() + 20; 
    controlLoops++;    
    if (battery.chargerConnected()){
      setOperation(OP_CHARGE);
      battery.resetIdle();
    }       
    if (stateOp == OP_MOW){      
      pt_t target = maps.targetPoint;
      pt_t lastTarget = maps.lastTargetPoint;
      float linear = 1.0;
      float angular = 0;
      float dX = target.x - stateX;
      float dY = target.y - stateY;
      float targetDist = sqrt( sq(dX) + sq(dY) );    
      float targetDelta = atan2(dY, dX);   
      targetDelta = scalePIangles(targetDelta, stateDelta);                  
      float diffDelta = distancePI(stateDelta, targetDelta);                         
      float lateralError = distanceLine(stateX, stateY, lastTarget.x, lastTarget.y, target.x, target.y);      
              
      if (abs(diffDelta/PI*180.0) > 20){
        // angular control (if angle to far away, rotate to next waypoint)
        linear = 0;
        angular = 0.5;        
        if (diffDelta < 0) angular *= -1;           
        resetMotionMeasurement();
      } else {
        // line control (if angle ok, follow path to next waypoint)                 
        if (     ((setSpeed > 0.2) && (maps.distanceToTargetPoint(stateX, stateY) < 0.3)) 
              || ((linearMotionStartTime != 0) && (millis() < linearMotionStartTime + 3000))              
           )
          linear = 0.1; // reduce speed when approaching/leaving waypoints
        else {
          if (gps.solution == UBLOX::SOL_FLOAT)        
            linear = min(setSpeed, 0.1); // reduce speed for float solution
          else
            linear = setSpeed;         // desired speed
        }
        angular = 0.5 * diffDelta + 0.5 * lateralError;       // correct for path errors
        /*pidLine.w = 0;              
        pidLine.x = lateralError;
        pidLine.max_output = PI;
        pidLine.y_min = -PI;
        pidLine.y_max = PI;
        pidLine.compute();
        angular = -pidLine.y;   */
        //CONSOLE.print(lateralError);        
        //CONSOLE.print(",");        
        //CONSOLE.println(angular/PI*180.0);        
      }
      if (fixTimeout != 0){
        if (millis() > lastFixTime + fixTimeout * 1000.0){
          // stop on fix solution timeout (fixme: optionally: turn on place if fix-timeout)
          linear = 0;
          angular = 0;
          stateSensor = SENS_GPS_FIX_TIMEOUT;
          //angular = 0.2;
        } else {
          if (stateSensor == SENS_GPS_FIX_TIMEOUT) stateSensor = SENS_NONE; // clear fix timeout
        }       
      }                  
      motor.setLinearAngularSpeed(linear, angular);    
      if ((gps.solution == UBLOX::SOL_FIXED) || (gps.solution == UBLOX::SOL_FLOAT)){        
        if (linear > 0.06) {
          if ((millis() > linearMotionStartTime + 5000) && (stateGroundSpeed < 0.03)){
            // if in linear motion and not enough ground speed => obstacle
            CONSOLE.println("obstacle!");
            stateSensor = SENS_OBSTACLE;
            setOperation(OP_ERROR);
            buzzer.sound(SND_STUCK, true);        
          }
        } else {
          resetMotionMeasurement();
        }  
      }
      if (maps.distanceToTargetPoint(stateX, stateY) < 0.1){
        // next waypoint
        if (!maps.nextWaypoint()){
          // finish        
          CONSOLE.println("mowing finished!");
          if (!finishAndRestart){
            setOperation(OP_IDLE); 
          }                    
        }
      }
      battery.resetIdle();
      if (battery.underVoltage()){
        stateSensor = SENS_BAT_UNDERVOLTAGE;
        setOperation(OP_IDLE);
        //buzzer.sound(SND_OVERCURRENT, true);        
      }      
    }  else if (stateOp == OP_CHARGE){      
      if (!battery.chargerConnected()){
        setOperation(OP_IDLE);        
      }       
    }
  }
}


// robot main loop
void run(){  
  buzzer.run();
  battery.run();
  motor.run();
  maps.run();  
  
  // IMU
  if (millis() > nextImuTime){
    nextImuTime = millis() + 150;    
    readIMU();    
    //imu.resetFifo();    
  }  
  
  gps.run();
    
  calcStats();  
  
  computeRobotState();  
  
  controlRobotVelocity();      
    
  // ----- read serial input (BT/console) -------------
  processConsole();     
  processBLE();     
  processWifi();
  outputConsole();    
}


// set new robot operation
void setOperation(OperationType op){  
  if (stateOp == op) return;  
  CONSOLE.print("setOperation op=");
  CONSOLE.println(op);
  switch (op){
    case OP_IDLE:
      motor.setLinearAngularSpeed(0,0);
      motor.setMowState(false);
      break;
    case OP_MOW:      
      if (maps.targetWaypointIdx >= maps.waypointsCount) {
        CONSOLE.println("error: no waypoints!");
        op = stateOp;        
      } else {
        resetMotionMeasurement();        
        maps.setLastTargetPoint(stateX, stateY);        
        stateSensor = SENS_NONE;
        motor.setMowState(true);
      }
      break;
    case OP_CHARGE:
      motor.setLinearAngularSpeed(0,0);
      motor.setMowState(false);
      break;
    case OP_ERROR:
      motor.setLinearAngularSpeed(0,0);
      motor.setMowState(false);
      break;
  }
  stateOp = op;  
}

