

// Author: alex.pennington@organicengineer.com

#define calWeight 530 // the weight of the calibration object

#include <EEPROM.h>

#include <SoftwareSerial.h>
SoftwareSerial swSerial(2, 3); // RX, TX

#include <LiquidCrystal.h>
//LCD pin to Arduino
const int pin_RS = 8;
const int pin_EN = 9;
const int pin_d4 = 4;
const int pin_d5 = 5;
const int pin_d6 = 6;
const int pin_d7 = 7;
const int pin_BL = 10;
LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);

bool debug_flag = false;

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A2;  // Analog input pin that the potentiometer is attached to

float vADC;  // the voltage
float fValue;  //the filtered value from the last reading
float weighting = 0.7; //the weight (between 0 and 1) given to the previous filter value
float nWeighting;  //the weighting given to the new input value. weighting + nWeighting must equal 1 so we just calculate nWeighting as 1 - weighting
float vReference = 5.0; //using the supply voltage as the reference
unsigned long previousMillis = 0;
int interval = 100; //choose the time between readings in milliseconds


int sensorValue = 0;
int pValue = 0; // pressure
int outputValue = 0;
int tareValue = 241;
float calFactor = 1.81;
int addr = 10;
#define rTsize 50
int rT[rTsize];
int rTindex = 0;
unsigned int rTsum = 0;
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete


unsigned long button_LastChange = 0;
unsigned long button_DebounceTime = 1000;
int button_PressedIndex = 0;
int button_LastPressedIndex = 0;
void setup() {
  // initialize serial communications at 9600 bps:
  delay(2000);
  Serial.begin(9600);
  inputString.reserve(200);
  analogReference(DEFAULT);  //on the uno & nano this is VSupply
  nWeighting = 1 - weighting; //calculate weight to be applied to the new reading
  fValue = analogRead(analogInPin);  //initialise to current value of input
  //EEPROM.get(addr, calFactor);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Loader Scale");
  lcd.setCursor(0, 1);
  //lcd.print("Press Key:");

  swSerial.begin(4800);
  swSerial.println("JD5055e OK");
}

void loop() {
  unsigned long currentMillis = millis();  //get the time
  
  if (currentMillis - previousMillis >= interval) {  // if its time for a new reading
    previousMillis = currentMillis;                  //update time of most recent reading
    sensorValue = analogRead(analogInPin);                    // read the input at the analog pin
    fValue = sensorValue * nWeighting + fValue * weighting; //calculate new fValue based on input and previous fValue
    pValue = map(fValue, tareValue, 1023, 0, 3000);
    outputValue = calFactor * pValue;
    lcd.setCursor(0, 1);
    lcd.print ("                 ");
    lcd.setCursor(0, 1);
    lcd.print (outputValue);
  }

  if ((currentMillis - button_LastChange) >  button_DebounceTime) {
    int x;
    x = analogRead (0);
    lcd.setCursor(1, 1);
    if (x < 60) {
      lcd.print ("Right ");
    }
    else if (x < 200) {
      lcd.print ("Up    ");
    }
    else if (x < 400) {
      lcd.print ("Down  ");
    }
    else if (x < 600) {
      lcd.print ("Left  ");
    }
    else if (x < 800) {
      button_PressedIndex = 5;
      lcd.print (outputValue);
      swSerial.println(outputValue);
    }
    button_LastChange = currentMillis;
    button_LastPressedIndex = button_PressedIndex;
  }

  if (stringComplete) {
    //Serial.println(inputString);
    if (inputString == "tare") {
      tareValue = fValue;
    }
    else if (inputString == "debug") {
      debug_flag = !debug_flag;
    }
    else if (inputString == "+") {
      if (rTindex < rTsize) {
        rT[rTindex] = outputValue;
        Serial.print(rTindex);
        Serial.print(" = ");
        Serial.println(rT[rTindex]);
        rTindex++;
      } else {
        Serial.println("Buffer Full");
      }
    }
    else if (inputString == "-") {
      if (rTindex >= 0 ) {
        rT[rTindex] = 0;
        if (rTindex > 0 ) {
          rTindex--;
        }
      }
    }
    else if (inputString == "read") {
      if (debug_flag) {
        Serial.print("sensor = ");
        Serial.print(sensorValue);
        Serial.print("\t p = ");
        Serial.print(pValue);
        Serial.print("\t t = ");
        Serial.print(tareValue);
        Serial.print("\t c = ");
        Serial.print(calFactor);
        Serial.print("\t i = ");
        Serial.print(rTindex);
        Serial.print("\t ");
      }
      Serial.print("weight = ");
      Serial.println(outputValue);
    }
    else if (inputString == "sum") {
      rTsum = 0;
      for (int i = 0; i < rTindex; i++) {
        rTsum += rT[i];
        Serial.print(i);
        Serial.print(" = ");
        Serial.println(rT[i]);
      }
      Serial.print("sum = ");
      Serial.println(rTsum);
    }
    else if (inputString == "clear") {
      for (int i = 0; i < rTsize; i++) {
        rT[i] = 0;
      }
    }
    else if (inputString == "cal") {
      calFactor = float(calWeight) / float(pValue);
      EEPROM.put(addr,  calFactor);
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
