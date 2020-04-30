// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "battery.h"
#include "config.h"
#include "helper.h"
#include "motor.h"
#include "robot.h"
#include "buzzer.h"
#include <Arduino.h>


void Battery::begin()
{
  // keep battery switched ON
  pinMode(pinBatterySwitch, OUTPUT);    
  digitalWrite(pinBatterySwitch, HIGH);  

  pinMode(pinChargeRelay, OUTPUT);
  pinMode(pinBatteryVoltage, INPUT);
  pinMode(pinChargeVoltage, INPUT);
  pinMode(pinChargeCurrent, INPUT);
  enableCharging(false);
  allowSwitchOff(false);
  resetIdle();
  
  pinMode(pinChargeCurrent, INPUT);
  pinMode(pinBatteryVoltage, INPUT);
  pinMode(pinChargeVoltage, INPUT);
  
  nextCheckTime = 0;
	timeMinutes=0;
  chargerConnectedState = false;    
  
  batteryFactor = (100+10) / 10;    // ADC voltage to battery voltage
  currentFactor = 0.5;         // ADC voltage to current ampere
  batMonitor = true;              // monitor battery and charge voltage?    
  batGoHomeIfBelow = 23.7;     // drive home voltage (Volt)  
  batSwitchOffIfBelow = 21.7;  // switch off battery if below voltage (Volt)  
  batSwitchOffIfIdle = 300;      // switch off battery if idle (seconds)
  batFullCurrent  = 0.4;      // current flowing when battery is fully charged
  startChargingIfBelow = 26.0; // start charging if battery Voltage is below  
  batteryVoltage = 0;
}


void Battery::enableCharging(bool flag){
  DEBUG(F("enableCharging "));
  DEBUGLN(flag);
  chargingEnabled = flag;
  digitalWrite(pinChargeRelay, flag);      
	nextPrintTime = 0;  	   	   	
}

bool Battery::chargerConnected(){
  return (chargingVoltage > 5);
}
  

bool Battery::shouldGoHome(){
  return (batteryVoltage < batGoHomeIfBelow);
}

bool Battery::underVoltage(){
  return (batteryVoltage < batSwitchOffIfBelow);
}

void Battery::resetIdle(){
  switchOffTime = millis() + batSwitchOffIfIdle * 1000;    
}

void Battery::allowSwitchOff(bool flag){
  switchOffAllowed = flag;
  if (flag) resetIdle();
}


void Battery::run(){  
  chargingVoltage = ((float)ADC2voltage(analogRead(pinChargeVoltage))) * batteryFactor;  
  float w = 0.9;
  if (batteryVoltage < 5) w = 0;
  batteryVoltage = w * batteryVoltage + (1-w) * ((float)ADC2voltage(analogRead(pinBatteryVoltage))) * batteryFactor;  
  chargingCurrent = 0.9 * chargingCurrent + 0.1 * ((float)ADC2voltage(analogRead(pinChargeCurrent))) * currentFactor;    
	
	if (chargerConnected()){           
      if (!chargerConnectedState){
	      chargerConnectedState = true;		    
		    DEBUGLN(F("CHARGER CONNECTED"));      	              
        buzzer.sound(SND_OVERCURRENT, true);
        enableCharging(true);        				
	    }
  } else {
      if (chargerConnectedState){        
        chargerConnectedState = false;
        DEBUGLN(F("CHARGER DISCONNECTED"));              				
        enableCharging(false);				
      }
  }      		
  
  if (millis() >= nextCheckTime){    
    nextCheckTime = millis() + 5000;  	   	   	
    timeMinutes = (millis()-chargingStartTime) / 1000 /60;
    if (switchOffAllowed) {
      if (underVoltage()) {
        DEBUGLN(F("SWITCHING OFF (undervoltage)"));              
        buzzer.sound(SND_OVERCURRENT, true);
        digitalWrite(pinBatterySwitch, LOW);    
      } else if (millis() >= switchOffTime) {
        DEBUGLN(F("SWITCHING OFF (idle timeout)"));              
        buzzer.sound(SND_OVERCURRENT, true);
        digitalWrite(pinBatterySwitch, LOW);    
      } else digitalWrite(pinBatterySwitch, HIGH);          
    }
    	  
    if (chargerConnected()){           
      if (chargerConnectedState){	      
        // charger in connected state
        if (chargingEnabled){
          if ((timeMinutes > 180) || (chargingCurrent < batFullCurrent)) {        
            // stop charging
            enableCharging(false);
          }          
        } else {
           if (batteryVoltage < startChargingIfBelow) {
              // start charging
              enableCharging(true);
              chargingStartTime = millis();  
          }        
        }
      }      
    } 
		
		if (millis() >= nextPrintTime){
			nextPrintTime = millis() + 60000;  	   	   	
			//print();			
			/*DEBUG(F("charger conn="));
			DEBUG(chargerConnected());
			DEBUG(F(" chgEnabled="));
			DEBUG(chargingEnabled);
			DEBUG(F(" chgTime="));      
			DEBUG(timeMinutes);
			DEBUG(F(" charger: "));      
			DEBUG(chargingVoltage);
			DEBUG(F(" V  "));    
			DEBUG(chargingCurrent);   
			DEBUG(F(" A "));         
			DEBUG(F(" bat: "));
			DEBUG(batteryVoltage);
			DEBUG(F(" V  "));    
			DEBUG(F("switchOffAllowed="));   
			DEBUG(switchOffAllowed);      
			DEBUGLN();      */					
    }	
  }
}
