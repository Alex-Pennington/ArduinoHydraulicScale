

// Author: alex.pennington@organicengineer.com

#define calWeight 530 // the weight of the calibration object
#include <Arduino.h>
#include <EEPROM.h>

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
float smoothedValue;  //the filtered value from the last reading
float alpha = 0.1; //the weight (between 0 and 1) given to the previous filter value
float vReference = 5.0; //using the supply voltage as the reference
unsigned long previousMillis = 0;
int interval = 500; //choose the time between readings in milliseconds
bool display_rolling_weight = true;

int sensorValue = 0;
int sensorVoltage = 0; // the voltage
int pressureValue = 0; // pressure
int outputValue = 0;
int tareValue = 0;
float calFactor = 1.0; //calibration factor
int sensorMaxPSI = 3000; // maximum pressure in PSI
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


void  serialEvent();

void setup() {
  // analogReadResolution(14);
  // initialize serial communications at 9600 bps:
  delay(2000);
  Serial1.begin(9600);
  inputString.reserve(200);
  smoothedValue = analogRead(analogInPin);  //initialise to current value of input
  EEPROM.get(addr, calFactor);
  EEPROM.get(addr+4, tareValue);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Loader Scale");
  lcd.setCursor(0, 1);
  lcd.print("Press Key:");

  Serial1.println("JD5055e OK");
}

void loop() {
  unsigned long currentMillis = millis();  //get the time
  if (currentMillis - previousMillis >= interval) {  // if its time for a new reading
    previousMillis = currentMillis;                  //update time of most recent reading
    int numReadings = 10;
    float total = 0;
    analogReadResolution(14); // set the resolution to 14 bits
    for (int i = 0; i < numReadings; i++) {
      total += analogRead(analogInPin);
      delay(10);
    }
    sensorValue = total / numReadings;
    sensorVoltage = map((int)sensorValue, 0, 16384, 0, (int)(vReference * 1000.0));                    // read the input at the analog pin
    smoothedValue = (smoothedValue * alpha) + (sensorValue * (1 - alpha)); //calculate new smoothedValue based on input and previous smoothedValue
    pressureValue = map((int)smoothedValue, 0, 16384, 0, sensorMaxPSI);
    outputValue = (float)calFactor * ((float)smoothedValue - (float)tareValue); //calculate the output value
    if (display_rolling_weight) {
      lcd.setCursor(0, 1);
      lcd.print ("                 ");
      lcd.setCursor(0, 1);
      lcd.print (outputValue);
    }
  }

  if ((currentMillis - button_LastChange) >  button_DebounceTime) {
    int x;
    analogReadResolution(10); // set the resolution to 10 bits
    x = analogRead (0);
    lcd.setCursor(0, 1);
    if (x < 60) { //Right
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      inputString = "+";
      stringComplete = true;
      lcd.print (inputString);
    }
    else if (x < 200) { //Up
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      inputString = "sum";
      stringComplete = true;
      lcd.print (inputString);
    }
    else if (x < 400) { //Down
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      inputString = "b";
      stringComplete = true;
    }
    else if (x < 600) { //Left
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      inputString = "-";
      stringComplete = true;
      lcd.print (inputString);
    }
    else if (x < 800) { //Select
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      inputString = "tare";
      stringComplete = true;
      lcd.print (inputString);
    }
    button_LastChange = currentMillis;
    button_LastPressedIndex = button_PressedIndex;
  }
  
  serialEvent(); // check if data has been sent

  if (stringComplete) {
    //Serial.println(inputString);
    if (inputString == "tare") {
      tareValue = sensorValue;
      EEPROM.put(addr+4, tareValue);
      Serial1.println("tare");
    }
    else if (inputString == "debug") {
      debug_flag = !debug_flag;
    }
    else if (inputString == "+") {
      if (rTindex < rTsize) {
        rT[rTindex] = outputValue;
        Serial1.print(rTindex);
        Serial1.print(" = ");
        Serial1.println(rT[rTindex]);
        lcd.print(outputValue);
        rTindex++;
      } else {
        Serial1.println("Buffer Full");
        lcd.print ("                           ");
        lcd.setCursor(0, 1);
        lcd.print ("Buffer Full");
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
        Serial1.print("a = ");
        Serial1.print(sensorValue);
        Serial1.print("\t v = ");
        Serial1.print(sensorVoltage);        
        Serial1.print("\t p = ");
        Serial1.print(pressureValue);
        Serial1.print("\t t = ");
        Serial1.print(tareValue);
        Serial1.print("\t c = ");
        Serial1.print(calFactor);
        Serial1.print("\t sm = ");
        Serial1.print(sensorMaxPSI);        
        Serial1.print("\t i = ");
        Serial1.print(rTindex);
        Serial1.print("\t ");
      }
      Serial1.print("weight = ");
      Serial1.println(outputValue);
      lcd.setCursor(0, 1);
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      //lcd.print("weight = ");
      lcd.print(outputValue);
    }
    else if (inputString == "sum") {
      rTsum = 0;
      for (int i = 0; i < rTindex; i++) {
        rTsum += rT[i];
        Serial1.print(i);
        Serial1.print(" = ");
        Serial1.println(rT[i]);
      }
      Serial1.print("sum = ");
      Serial1.println(rTsum);
      lcd.setCursor(0, 1);
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      lcd.print("sum = ");
      lcd.print(rTsum);
    }
    else if (inputString == "b") {
      for (int i = 0; i < rTsize; i++) {
        rT[i] = 0;
      }
      rTindex = 0;
      rTsum = 0;
      Serial1.println("Buffer Cleared");
      lcd.setCursor(0, 1);
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      lcd.print("Buffer Cleared");
    }
    else if (inputString == "kv") {
      calFactor = float(calWeight) / (float(smoothedValue) - (float)tareValue);
      EEPROM.put(addr,  calFactor);
    }
    else if (inputString.charAt(0) == 'c') {
      inputString.remove(0, 1); // remove the first character
      calFactor = float(inputString.toFloat());
      EEPROM.put(addr,  calFactor);
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      lcd.print("c = ");
      lcd.print(inputString);
    }
    else if (inputString.startsWith("sm")) {
      inputString.remove(0, 2); // remove the first character
      sensorMaxPSI = inputString.toInt();
      //EEPROM.put(addr,  calFactor);
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      lcd.print("sm = ");
      lcd.print(sensorMaxPSI);
    }
    // clear the string:
    if (display_rolling_weight) {delay(1000);}
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
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
