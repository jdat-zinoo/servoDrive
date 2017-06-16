//#define analogResolution 8

#define servoFaultLed 5
#define servoReadyLed 4
#define joyFaultLed 3
#define rxDataLed 2

#define dacPin 9
//#define dacRange 4096

#define potPin A0
#define pFaultPin A1
#define deadZonePin A2
#define speedPin A3

#define potLowFault 6
#define potHiFault 1015

#define faultLow 6
#define faultHi 1015

#define resetFaultPin 13
#define calibratePin 12

#define joyFault -2
#define joyNotReady -1
#define resetFault -4

#define outMax 255
#include "Button.h"


Button calibrateButton = Button(calibratePin, PULLUP);
Button faultResetButton = Button(resetFaultPin, PULLUP);

int calMin = 500;
int calMax = 500;
int calMid = 500;

int deadBand = 50;
int slope = 1;

void calibratePot() {
  int a;
  a = analogRead(potPin);
  if (a <= calMin) {
    calMin = a;
  }
  if (a >= calMax) {
    calMax = a;
  }
}
void setup() {
  // put your setup code here, to run once:
  pinMode(dacPin, OUTPUT);
  //analogReadResolution(10);
  //analogWriteResolution(analogResolution);
  Serial.begin(9600);
  pinMode(calibratePin, INPUT_PULLUP);
  pinMode(resetFaultPin, INPUT_PULLUP);
  pinMode(servoFaultLed,OUTPUT);
  pinMode(servoReadyLed,OUTPUT);
  pinMode(joyFaultLed,OUTPUT);
  pinMode(rxDataLed,OUTPUT);
  
  //while (!Serial) {
  ; // wait for serial port to connect. Needed for native USB port only
  //}

  //analogWrite(dacPin,dacRange/2);
  //analogWrite(dacPin,0);

}

uint8_t j;
void loop(){
  for (uint8_t i=2;i<=5;i++){
    if (i==j){
      digitalWrite(i,HIGH);
    } else {
      digitalWrite(i,LOW);
    }
  }
  j++;
  if (j<2) j=2;
  if (j>5) j=2;
  delay(300);
}
/*
int out;
int pot;
int fault;
int slopeTmp;
void loop() {
  pot = analogRead(potPin);
  fault = analogRead(pFaultPin);
  deadBand = map(analogRead(deadZonePin), 0, 1024, 10, 500);
  slope = map(analogRead(speedPin), 0, 1024, 1, 10);
  slopeTmp = outMax / slope;

  if ((pot < potLowFault) || (pot > potHiFault) || (fault > faultHi) || (fault > faultHi) ) {
    //Serial.println("Joystic fault");
    Serial.println(joyFault);
    //analogWrite(dacPin,0);
  } else if (fault < faultLow) {
    //Serial.println("Fault or button release");
    Serial.println(joyNotReady);
    //analogWrite(dacPin,0);
  } else {
    if ( (pot < (calMid - deadBand)) || (pot > (calMid + deadBand))) {
      //out=map(pot,calMin,calMax,1,outMax);

      out = map(pot, calMin, calMax, outMax / 2 - slopeTmp, outMax / 2 + slopeTmp);
      Serial.println(out);
      //analogWrite(dacPin,out);
    } else {
      Serial.println(map(calMid, calMin, calMax, 1, outMax));
      //analogWrite(dacPin,0);
    }
  }


  if (calibrateButton.uniquePress()) {
    calMin = 500;
    calMax = 500;
    calMid = 500;
  }
  if (calibrateButton.isPressed()) {
    calibratePot();
  }
  if (calibrateButton.stateChanged()) {
    calMid = analogRead(potPin);
  }
  if (faultResetButton.isPressed()){
    Serial.println(-4);
  }
  //Serial.print();
  //Serial.print(",");
  //Serial.println();
  //  Serial.print("J: ");
  //  Serial.print(pot);
  //  Serial.print(",");
  //  Serial.println(fault);
  //    Serial.print("C: ");
  //    Serial.print(calMid);
  //    Serial.print(",");
  //    Serial.print(calMin);
  //    Serial.print(",");
  //    Serial.println(calMax);

  delay(100);
  //out=map(analogRead(potPin),0,255,50,4045);
  //analogWrite(dacPin,out);
  //delayMicroseconds(1000);

}
*/
