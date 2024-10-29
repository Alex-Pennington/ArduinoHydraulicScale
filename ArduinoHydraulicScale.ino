// Author: alex.pennington@organicengineer.com

#define calWeight 530 // the weight of the calibration object

#include <EEPROM.h>

bool debug_flag = true;

const int analogInPin = A1;  // Analog input pin that the potentiometer is attached to

float fValue;  //the filtered value from the last reading
float weighting = 0.7; //the weight (between 0 and 1) given to the previous filter value
float nWeighting;  //the weighting given to the new input value. weighting + nWeighting must equal 1 so we just calculate nWeighting as 1 - weighting
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
int interval = 10; //choose the time between readings in milliseconds
int interval2 = 5000; //choose the time between readings in milliseconds

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

/*
  ArduinoMqttClient - WiFi Simple Sender

  This example connects to a MQTT broker and publishes a message to
  a topic once a second.

  The circuit:
  - Arduino MKR 1000, MKR 1010 or Uno WiFi Rev2 board

  This example code is in the public domain.
*/

#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_GIGA) || defined(ARDUINO_OPTA)
#include <WiFi.h>
#elif defined(ARDUINO_PORTENTA_C33)
#include <WiFiC3.h>
#elif defined(ARDUINO_UNOR4_WIFI)
#include <WiFiS3.h>
#endif

#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

// To connect with SSL/TLS:
// 1) Change WiFiClient to WiFiSSLClient.
// 2) Change port value from 1883 to 8883.
// 3) Change broker value to a server with a known SSL/TLS root certificate
//    flashed in the WiFi module.

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = SECRET_MQTT_BROKER;
int        port     = 1883;
const char topic[]  = "arduino";
const char username[] = SECRET_MQTT_USER;
const char password[]  =  SECRET_MQTT_PASS;


const int button1Pin = D2;    // the number of the pushbutton pin
const int ledPin = D13;      // the number of the LED pin

int ledState = LOW;         // the current state of the output pin
int button1State;             // the current reading from the input pin
int lastButton1State = LOW;   // the previous reading from the input pin

unsigned long lastDebounceTime1 = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

#define ScaleReadingsIndex_MAX  100
float ScaleReadings[ScaleReadingsIndex_MAX] = {};
int ScaleReadingsIndex = 0;
float ScaleFluctuationThreshold = 120;

bool isFluctuating(float reading) {
  ScaleReadings[ScaleReadingsIndex] = reading;
  if (ScaleReadingsIndex = ( ScaleReadingsIndex_MAX - 1 )) {
    ScaleReadingsIndex = 0;
  } else {
    ScaleReadingsIndex++;
  }

  float sumOfDifferences = 0;
  for (int i = 1; i < ScaleReadingsIndex_MAX; i++) {
    sumOfDifferences += fabs(ScaleReadings[i] - ScaleReadings[i - 1]);
  }

  float averageDifference = sumOfDifferences / (ScaleReadingsIndex_MAX - 1);
  //Serial.println(averageDifference);
  if (averageDifference > ScaleFluctuationThreshold) {
    return true;
  } else {
    return false;
  }

}

/*
  Arduino 2x16 LCD - Detect Buttons
  modified on 18 Feb 2019
  by Saeed Hosseini @ Electropeak
  https://electropeak.com/learn/
*/
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

void setup() {
  //setup button
  pinMode(button1Pin, INPUT);
  pinMode(ledPin, OUTPUT);
  // set initial LED state
  digitalWrite(ledPin, ledState);

  // initialize serial communications at 9600 bps:
  delay(2000);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // attempt to connect to WiFi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // You can provide a unique client ID, if not set the library uses Arduino-millis()
  // Each client must have a unique client ID
  // mqttClient.setId("clientId");

  // You can provide a username and password for authentication
  mqttClient.setUsernamePassword(username, password);

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  analogReadResolution(14); //change to 14-bit resolution
  inputString.reserve(200);
  //analogReference(DEFAULT);  //on the uno & nano this is VSupply
  nWeighting = 1 - weighting; //calculate weight to be applied to the new reading
  fValue = analogRead(analogInPin);  //initialise to current value of input
  //EEPROM.get(addr, calFactor);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print(WiFi.localIP());
  lcd.setCursor(0, 1);
  lcd.print("Press Key:");
}

void loop() {

  int x;
  x = analogRead (A0);
  lcd.setCursor(10, 1);
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
    lcd.print ("Select");
  }

  mqttClient.poll();
  unsigned long currentMillis = millis();  //get the time

  /*
    // read the state of the switch into a local variable:
    int readingButton1 = digitalRead(button1Pin);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (readingButton1 != lastButton1State) {
      // reset the debouncing timer
      lastDebounceTime1 = currentMillis;
    }

    if ((currentMillis - lastDebounceTime1) > debounceDelay) {
      if (readingButton1 != button1State) {
        button1State = readingButton1;
        if (button1State == HIGH) {
          ledState = !ledState;
        }
      }
    }

    if (((currentMillis - lastDebounceTime1) > 10000)) { // return to fluctuation indicator after timeout
      ledState = isFluctuating(outputValue);
    }

    digitalWrite(ledPin, ledState);
    lastButton1State = readingButton1;
  */

  //scale averaging
  if (currentMillis - previousMillis >= interval) {  // if its time for a new reading
    previousMillis = currentMillis;                  //update time of most recent reading
    sensorValue = analogRead(analogInPin);                    // read the input at the analog pin
    fValue = sensorValue * nWeighting + fValue * weighting; //calculate new fValue based on input and previous fValue
    pValue = map(fValue, tareValue, 1023, 0, 3000);
    outputValue = calFactor * pValue;
    ledState = isFluctuating(outputValue);
    digitalWrite(ledPin, ledState);
  }

  //send sensor data
  if (currentMillis - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis;
    /*if (debug_flag) {
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
      Serial.print("weight = ");
      Serial.println(outputValue);
      }*/
    // send message, the Print interface can be used to set the message contents
    char topic_temp[30];
    String str_temp;
    str_temp = String(topic);
    str_temp.concat("/debug");
    str_temp.toCharArray(topic_temp, 30);
    mqttClient.beginMessage(topic_temp);
    char oV[10];
    str_temp = String(outputValue);
    str_temp.toCharArray(oV, 10);
    mqttClient.print(oV);
    mqttClient.endMessage();
  }

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
      if (debug_flag) {
        Serial.println("RECEIVED COMMAND");
      }
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    Serial.println(inputString);
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

        // send message, the Print interface can be used to set the message contents
        char topic_temp[30];
        String str_temp;
        str_temp = String(topic);
        str_temp.concat("/weight");
        str_temp.toCharArray(topic_temp, 10);
        mqttClient.beginMessage(topic_temp);
        char oV[10];
        str_temp = String(outputValue);
        str_temp.toCharArray(oV, 10);
        mqttClient.print(oV);
        mqttClient.endMessage();
        Serial.println("OK");
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
      Serial.println("OK");
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
      Serial.println("OK");
    }
    else if (inputString == "cal") {
      calFactor = float(calWeight) / float(pValue);
      EEPROM.put(addr,  calFactor);
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(2);
}
