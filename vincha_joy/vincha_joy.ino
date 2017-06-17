//#define debug

#define outMax 512
#define outMin 1
#define slopeRange 10
#define deadBandRange 250

#define potLowFault 6
#define potHiFault 1015

#define faultLow 6
#define faultHi 1015

#define serialTimeout 50

#define servoFaultLed 5
#define servoReadyLed 4
#define joyFaultLed 3
#define rxDataLed 2

#define potPin A0
#define pFaultPin A1
#define deadZonePin A2
#define speedPin A3

#define resetFaultPin 13
#define calibratePin 12
#define driveButtonPin 11

#define joyReady -1
#define resetFault -2
#define joyFault -4

#include "Button.h"

Button calibrateButton = Button(calibratePin, PULLUP);
Button faultResetButton = Button(resetFaultPin, PULLUP);
Button driveButton = Button(driveButtonPin, PULLUP);

int calMin = 500;
int calMax = 500;
int calMid = 500;

int deadBand = 50;
int slope = 1;
int pot;
int fault;
int out = 0;
int indata;
bool doCalibrate = true;

void calibratePot() {
  pot = analogRead(potPin);
  fault = analogRead(pFaultPin);
  out = joyReady;

  if (doCalibrate) {
    digitalWrite(joyFaultLed, HIGH);
  } else {
    digitalWrite(joyFaultLed, LOW);
  }
  if (calibrateButton.uniquePress()) {
    //detect 1st calibrate press
    calMin = 500;
    calMax = 500;
    calMid = 500;
  }
  if (calibrateButton.stateChanged() && !calibrateButton.isPressed()) {
    // detect calibrate button release
    // calibrate complete
    calMid = pot;
    doCalibrate = false;
  }
  if (calibrateButton.isPressed()) {
    //calibrate while calibrate button is pressed
    if (pot < calMin) calMin = pot;
    if (pot > calMax) calMax = pot;
  }

  if (faultResetButton.isPressed()) {
    out = resetFault;
  }
  parseSerial();
}

void parseSerial(){
  static bool rxLedState;
  indata=abs(Serial.parseInt());
  if (bitRead(indata,0)){
    digitalWrite(servoReadyLed,HIGH);
  } else {
    digitalWrite(servoReadyLed,LOW);
  }
  
  if (bitRead(indata,1)){
    digitalWrite(servoFaultLed,HIGH);
  } else {
    digitalWrite(servoFaultLed,LOW);
  }
  
  if (indata==0){
    digitalWrite(rxDataLed,HIGH);
  } else {
    digitalWrite(rxDataLed,LOW);
  }
  
}
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(serialTimeout);
  
  pinMode(servoFaultLed, OUTPUT);
  pinMode(servoReadyLed, OUTPUT);
  pinMode(joyFaultLed, OUTPUT);
  pinMode(rxDataLed, OUTPUT);

  doCalibrate = true;
  Serial.print(out);
#ifdef debug
  Serial.print(",");
  Serial.print(calMin);
  Serial.print(",");
  Serial.print(calMid);
  Serial.print(",");
  Serial.print(calMax);
  Serial.print(",");
  Serial.print(pot);
  Serial.print(",");
  Serial.print(fault);
#endif
  Serial.println();
}

int slopeTmp;

void loop() {
  deadBand = map(analogRead(deadZonePin), 0, 1024, 10, deadBandRange + 1);
  slope = map(analogRead(speedPin), 0, 1024, 1, slopeRange + 1);
  slopeTmp = outMax / (2 * slope);

  calibratePot();
  if ((pot < calMin) || (pot > calMax)) {
    doCalibrate = true;
  } 
  if (!doCalibrate) {
    if ((pot < potLowFault) || (pot > potHiFault) || (fault > faultHi) || (fault < faultLow) ) {
      // Hardware fault
      out = joyFault;
    } else if (!driveButton.isPressed()) {
      // Drive button released
      out = joyReady;
    } else {
      // Joystick is ready to go
      // process data
      if ((pot < (calMid - deadBand)) || (pot > (calMid + deadBand))) {
        // joustick in active area
        // calculate data
        if ((pot < (calMid - deadBand))) {
          // Joystick in active mode lower part
          //out = map(pot, calMin, calMid - deadBand, outMin , outMax / 2);
          out = map(pot, calMin, calMid - deadBand, outMax / 2 - slopeTmp + 1 , outMax / 2);
        } else if ((pot > (calMid + deadBand))) {
          // Joystick in active mode upper part
          //out = map(pot, calMid + deadBand, calMax, outMax / 2, outMax );
          out = map(pot, calMid + deadBand, calMax, outMax / 2, outMax / 2 + slopeTmp - 1 );
        }
  
        // slope data from old
        // out = map(pot, calMin, calMax, outMax / 2 - slopeTmp, outMax / 2 + slopeTmp);
      } else {
        //in deadBand. stop motion
        out = joyReady;
      }
    }
  }  
/*
  if (faultResetButton.isPressed()) {
    out = resetFault;
  }
*/
  Serial.print(out);
#ifdef debug
  Serial.print(",");
  Serial.print(indata);
  Serial.print(",");
  Serial.print(calMin);
  Serial.print(",");
  Serial.print(calMid);
  Serial.print(",");
  Serial.print(calMax);
  Serial.print(",");
  Serial.print(pot);
  Serial.print(",");
  Serial.print(fault);
  Serial.print(",");
  Serial.print(deadBand);
  Serial.print(",");
  Serial.print(slope);
#endif
  Serial.println();

  delay(10);

}

