#define debug

#define joyMax 512
#define joyMin 1

#define pwmMin 10
#define pwmMax 4095

#define serialTimeout 1
#define parseErrorTreshold 5

#define servoFaultLed 5
#define servoReadyLed 4
#define joyFaultLed 3
#define rxDataLed 2

#define servoReadyPin 39
#define servoFaultPin 37
#define servoFaultResetPin 27

#define pwmPin 9

#define joyReady -1
#define resetFault -2
#define joyFault -4

int out = 0;
int indata;
int pot;

uint16_t countParseError;
static bool parserState = false;

void parseSerial() {
  indata = Serial3.parseInt();

  if (indata == 0) {
    countParseError++;
  } else {
    parserState = true;
    countParseError = 0;
    digitalWrite(rxDataLed, LOW);
  }

  if (countParseError > parseErrorTreshold) {
    parserState = false;
    digitalWrite(rxDataLed, HIGH);
  } else {
    parserState = true;
    digitalWrite(rxDataLed, LOW);
  }

  if (parserState && (countParseError == 0) && (indata!=0)) {
    if ((indata >= 1) && (indata <= joyMax)) {
      pot = map(indata, 1, joyMax, pwmMin, pwmMax);
      //analogWrite(pwmPin,map(indata,1,joyMax,1,4096));
      analogWrite(pwmPin, pot);
      return;
    } else  {
      analogWrite(pwmPin, 0);
    }

    if (indata < 0) { // indata=abs(indata);
      //if (bitRead(indata,2)){
      if (indata == -4) {
        digitalWrite(joyFaultLed, HIGH);
        return;
      } else  {
        digitalWrite(joyFaultLed, LOW);
      }

      //if (bitRead(indata,1)){
      if (indata == -2) {
        digitalWrite(servoFaultResetPin, HIGH);
        return;
      } else {
        digitalWrite(servoFaultResetPin, LOW);
      }

    }
  }
  if (!parserState) {
    digitalWrite(joyFaultLed, LOW);
    digitalWrite(servoFaultResetPin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void setup() {
  Serial.begin(38400);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial3.begin(38400);
  Serial3.setTimeout(serialTimeout);

  pinMode(servoFaultLed, OUTPUT);
  pinMode(servoReadyLed, OUTPUT);
  pinMode(joyFaultLed, OUTPUT);
  pinMode(rxDataLed, OUTPUT);

  pinMode(servoFaultResetPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  analogWriteResolution(12);

  pinMode(servoReadyPin, INPUT_PULLUP);
  pinMode(servoFaultPin, INPUT_PULLUP);

  Serial3.println(out);
}

void loop() {

  parseSerial();

  out = 8;
  //if (servoReady.isPressed()){
  if (digitalRead(servoReadyPin) == LOW) {
    digitalWrite(servoReadyLed, HIGH);
    bitSet(out, 0);
  } else {
    digitalWrite(servoReadyLed, LOW);
    bitClear(out, 0);
  }

  //if (servoFault.isPressed()){
  if (digitalRead(servoFaultPin) == LOW) {
    digitalWrite(servoFaultLed, HIGH);
    bitSet(out, 1);
  } else {
    digitalWrite(servoFaultLed, LOW);
    bitClear(out, 1);
  }

  //bitSet(out,15);
  //out=8;
  Serial3.println(out);
#ifdef debug
  Serial.print(out);
  Serial.print("\t");
  Serial.print(indata);
  Serial.print("\t");
  Serial.print(countParseError);
  Serial.print("\t");
  Serial.print(parserState);
  Serial.print("\t");
  Serial.print(pot);

  Serial.println();
#endif

  delay(10);

}

