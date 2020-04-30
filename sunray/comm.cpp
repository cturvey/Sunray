#include "comm.h"
#include "config.h"
#include "robot.h"
#include "WiFiEsp.h"

unsigned long nextInfoTime = 0;

String cmd;
String cmdResponse;

// use a ring buffer to increase speed and reduce memory allocation
ERingBuffer buf(8);
int reqCount = 0;                // number of requests received
unsigned long stopClientTime = 0;


// answer Bluetooth with CRC
void cmdAnswer(String s){  
  byte crc = 0;
  for (int i=0; i < s.length(); i++) crc += s[i];
  s += F(",0x");
  if (crc <= 0xF) s += F("0");
  s += String(crc, HEX);  
  s += F("\r\n");             
  //CONSOLE.print(s);  
  cmdResponse = s;
}


// request operation
void cmdControl(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;
  int mow=-1;          
  int op = -1;
  float wayPerc = -1;  
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){
      int intValue = cmd.substring(lastCommaIdx+1, idx+1).toInt();
      float floatValue = cmd.substring(lastCommaIdx+1, idx+1).toFloat();
      if (counter == 1){                            
          if (intValue >= 0) motor.setMowState(intValue == 1);
      } else if (counter == 2){                                      
          if (intValue >= 0) op = intValue; 
      } else if (counter == 3){                                      
          if (floatValue >= 0) setSpeed = floatValue; 
      } else if (counter == 4){                                      
          if (intValue >= 0) fixTimeout = intValue; 
      } else if (counter == 5){
          if (intValue >= 0) finishAndRestart = (intValue == 1);
      } else if (counter == 6){
          if (floatValue >= 0) maps.setTargetWaypointPercent(floatValue);
      }
      counter++;
      lastCommaIdx = idx;
    }    
  }      
  /*CONSOLE.print("linear=");
  CONSOLE.print(linear);
  CONSOLE.print(" angular=");
  CONSOLE.println(angular);*/    
  if (op >= 0) setOperation((OperationType)op);
  String s = F("C");
  cmdAnswer(s);
}

// request motor 
void cmdMotor(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;
  float linear=0;
  float angular=0;
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){
      float value = cmd.substring(lastCommaIdx+1, idx+1).toFloat();
      if (counter == 1){                            
          linear = value;
      } else if (counter == 2){
          angular = value;
      } 
      counter++;
      lastCommaIdx = idx;
    }    
  }      
  /*CONSOLE.print("linear=");
  CONSOLE.print(linear);
  CONSOLE.print(" angular=");
  CONSOLE.println(angular);*/
  motor.setLinearAngularSpeed(linear, angular);
  String s = F("M");
  cmdAnswer(s);
}

void cmdMotorTest(){
  String s = F("E");
  cmdAnswer(s);
  motor.test();  
}


// request waypoint
void cmdWaypoint(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;
  int widx=0;  
  float x=0;
  float y=0;
  bool success = true;
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){            
      float intValue = cmd.substring(lastCommaIdx+1, idx+1).toInt();
      float floatValue = cmd.substring(lastCommaIdx+1, idx+1).toFloat();
      if (counter == 1){                            
          widx = intValue;
      } else if (counter == 2){
          x = floatValue;
      } else if (counter == 3){
          y = floatValue;
          if (!maps.setWaypoint(widx, widx+1, x, y)){
            success = false;
            break;
          }          
          widx++;
          counter = 1;
      } 
      counter++;
      lastCommaIdx = idx;
    }    
  }      
  /*CONSOLE.print("waypoint (");
  CONSOLE.print(widx);
  CONSOLE.print("/");
  CONSOLE.print(count);
  CONSOLE.print(") ");
  CONSOLE.print(x);
  CONSOLE.print(",");
  CONSOLE.println(y);*/  
  if (success){    
    String s = F("W,");
    s += widx;              
    cmdAnswer(s);       
  }  
}

// request position mode
void cmdPosMode(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;  
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){
      int intValue = cmd.substring(lastCommaIdx+1, idx+1).toInt();
      double doubleValue = cmd.substring(lastCommaIdx+1, idx+1).toDouble();
      if (counter == 1){                            
          absolutePosSource = bool(intValue);
      } else if (counter == 2){                                      
          absolutePosSourceLon = doubleValue; 
      } else if (counter == 3){                                      
          absolutePosSourceLat = doubleValue; 
      } 
      counter++;
      lastCommaIdx = idx;
    }    
  }        
  CONSOLE.print("absolutePosSource=");
  CONSOLE.print(absolutePosSource);
  CONSOLE.print(" lon=");
  CONSOLE.print(absolutePosSourceLon);
  CONSOLE.print(" lat=");
  CONSOLE.println(absolutePosSourceLat);
  String s = F("P");
  cmdAnswer(s);
}

// request version
void cmdVersion(){
  String s = F("V,");
  s += F(VER);
  cmdAnswer(s);
}

// request summary
void cmdSummary(){
  String s = F("S,");
  s += battery.batteryVoltage;  
  s += ",";
  s += stateX;
  s += ",";
  s += stateY;
  s += ",";
  s += stateDelta;
  s += ",";
  s += gps.solution;
  s += ",";
  s += stateOp;
  s += ",";
  s += maps.targetWaypointIdx;
  s += ",";
  s += (millis() - gps.dgpsAge)/1000.0;
  s += ",";
  s += stateSensor;
  cmdAnswer(s);  
}

// request statistics
void cmdStats(){
  String s = F("T,");
  s += statIdleDuration;  
  s += ",";
  s += statChargeDuration;
  s += ",";
  s += statMowDuration;
  s += ",";
  s += statMowDurationFloat;
  s += ",";
  s += statMowDurationFix;
  s += ",";
  s += statMowFloatToFixRecoveries;
  s += ",";  
  s += statMowDistanceTraveled;  
  s += ",";  
  s += statMowMaxDgpsAge;
  s += ",";
  s += statImuRecoveries;
  cmdAnswer(s);  
}

// process request
void processCmd(){
  cmdResponse = "";
  if (cmd.length() < 4) return;      
  if (cmd[0] != 'A') return;
  if (cmd[1] != 'T') return;
  if (cmd[2] != '+') return;
  if (cmd[3] == 'S') cmdSummary();
  if (cmd[3] == 'M') cmdMotor();
  if (cmd[3] == 'C') cmdControl();
  if (cmd[3] == 'W') cmdWaypoint();
  if (cmd[3] == 'V') cmdVersion();  
  if (cmd[3] == 'P') cmdPosMode();  
  if (cmd[3] == 'T') cmdStats();
  if (cmd[3] == 'E') cmdMotorTest();  
}

// process console input
void processConsole(){
  char ch;      
  if (CONSOLE.available()){
    battery.resetIdle();  
    while ( CONSOLE.available() ){               
      ch = CONSOLE.read();          
      if ((ch == '\r') || (ch == '\n')) {        
        CONSOLE.println(cmd);
        processCmd();              
        CONSOLE.print(cmdResponse);    
        cmd = "";
      } else if (cmd.length() < 500){
        cmd += ch;
      }
    }
  }     
}
  
// process Bluetooth input
void processBLE(){
  char ch;   
  if (BLE.available()){
    battery.resetIdle();  
    while ( BLE.available() ){    
      ch = BLE.read();      
      if ((ch == '\r') || (ch == '\n')) {        
        CONSOLE.println(cmd);
        processCmd();              
        BLE.print(cmdResponse);    
        cmd = "";
      } else if (cmd.length() < 500){
        cmd += ch;
      }
    }    
  }  
}  

// process WIFI input
void processWifi()
{
  if (!wifiFound) return;
  // listen for incoming clients    
  if (client != NULL){
    if (stopClientTime != 0) {
      if (millis() > stopClientTime){
        client.stop();
        stopClientTime = 0;
        client = NULL;           
      }
      return;    
    }     
  }
  client = server.available();      
  if (client != NULL) {                               // if you get a client,
    battery.resetIdle();
    //CONSOLE.println("New client");             // print a message out the serial port
    buf.init();                               // initialize the circular buffer
    unsigned long timeout = millis() + 50;
    while ( (client.connected()) && (millis() < timeout) ) {              // loop while the client's connected
      if (client.available()) {               // if there's bytes to read from the client,        
        char c = client.read();               // read a byte, then
        timeout = millis() + 50;
        buf.push(c);                          // push it to the ring buffer
        // you got two newline characters in a row
        // that's the end of the HTTP request, so send a response
        if (buf.endsWith("\r\n\r\n")) {
          cmd = "";
          while ((client.connected()) && (client.available()) && (millis() < timeout)) {
            char ch = client.read();
            timeout = millis() + 50;
            cmd = cmd + ch;
          }
          CONSOLE.println(cmd);
          if (client.connected()) {
            processCmd();
            client.print(
              "HTTP/1.1 200 OK\r\n"
              "Access-Control-Allow-Origin: *\r\n"              
              "Content-Type: text/html\r\n"              
              "Connection: close\r\n"  // the connection will be closed after completion of the response
              // "Refresh: 1\r\n"        // refresh the page automatically every 20 sec                        
              );
            client.print("Content-length: ");
            client.print(cmdResponse.length());
            client.print("\r\n\r\n");                        
            client.print(cmdResponse);                                   
          }
          break;
        }
      }
    }    
    // give the web browser time to receive the data
    stopClientTime = millis() + 20;
    //delay(10);
    // close the connection
    //client.stop();
    //CONSOLE.println("Client disconnected");
  }                  
}



// output summary on console
void outputConsole(){
  if (millis() > nextInfoTime){        
    nextInfoTime = millis() + 5000;               
    CONSOLE.print ("ctlDur=");    
    CONSOLE.print (1.0 / (controlLoops/5.0));
    controlLoops=0;
    CONSOLE.print ("  op=");    
    CONSOLE.print (stateOp);
    CONSOLE.print ("  freem=");
    CONSOLE.print (freeMemory ());
    CONSOLE.print(" volt=");
    CONSOLE.print(battery.batteryVoltage);
    CONSOLE.print(" wpts=");
    CONSOLE.print(maps.waypointsCount);
    CONSOLE.print(" x=");
    CONSOLE.print(stateX);
    CONSOLE.print(" y=");
    CONSOLE.print(stateY);
    CONSOLE.print(" delta=");
    CONSOLE.print(stateDelta);    
    CONSOLE.print("  tow=");
    CONSOLE.print(gps.iTOW);
    CONSOLE.print("\tlon=");
    CONSOLE.print(gps.lon,8);
    CONSOLE.print("\tlat=");
    CONSOLE.print(gps.lat,8);    
    CONSOLE.print("\tn=");
    CONSOLE.print(gps.relPosN);
    CONSOLE.print("\te=");
    CONSOLE.print(gps.relPosE);
    CONSOLE.print("\td=");
    CONSOLE.print(gps.relPosD);
    CONSOLE.print("\tsol=");    
    CONSOLE.print(gps.solution);
    CONSOLE.print("\tage=");    
    CONSOLE.print((millis()-gps.dgpsAge)/1000.0);
    CONSOLE.println();
  }
}

