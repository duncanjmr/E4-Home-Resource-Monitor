/*
 * Home Rescource Monitor
 * E4 Main Design Project
 * Duncan Rocha, Rebecca Wreblewski, Matthew Calligaro, Isac Zinda
 * Version 1.0
 * 
 * Update History:
 * 5/1/17: Version 1.0 Complete. Includes library HRMlib with temp, windspeed, window distance, and motion sensing.
 * 
 * 
 * NOTE: The library NewPing must be installed for this code to run, it relies on it for the distance sensor. The
 * code can be found and installed here: https://github.com/PaulStoffregen/NewPing
 * 
 */

#include <HRM.h>
#include <NewPing.h>
HRM livno;

#define trigPin 11
#define echoPin 10
#define maxDist 200

#define PIRpin 5

#define tempSensorPin 7

#define analogPinRV 1
#define analogPinTMP 0

//NewPing sonar(trigPin, echoPin, maxDist);

String Data;


void setup() {
  Serial.begin(9600);
  livno.initializeSonar(trigPin,echoPin,maxDist);

  // Powering pins using digital I/O pins:
  pinMode(2,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(12,OUTPUT);
  digitalWrite(2,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(8,HIGH);
  digitalWrite(12,HIGH);
  
}

void loop() {
  livno.tempUpdate(tempSensorPin);
  livno.sonarUpdate();
  livno.windflowTempUpdate(analogPinRV, analogPinTMP);
  livno.windspeedUpdate(analogPinRV, analogPinTMP);
  livno.PIRupdate(PIRpin);


  Data = (
    "WindowDistance: " + String(livno.windowDist) + " Windspeed: " + String(livno.windspeed)+
    " WindflowTemp: " + String(livno.windflowTemp) +
    " TempsensorReading: " + String(livno.temp) +
    " PIRreading: " + String(livno.motion)
  );
  delay(400);

  Serial.println(Data);
}
