// Author: alex.pennington@organicengineer.com

#define calWeight 530 // the weight of the calibration object

#include <EEPROM.h>

bool debug_flag = false;

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

float vADC;  // the voltage
float fValue;  //the filtered value from the last reading
float weighting = 0.7; //the weight (between 0 and 1) given to the previous filter value
float nWeighting;  //the weighting given to the new input value. weighting + nWeighting must equal 1 so we just calculate nWeighting as 1 - weighting
float vReference = 5.0; //using the supply voltage as the reference
unsigned long previousMillis = 0;
int interval = 10; //choose the time between readings in milliseconds
int interval2 = 60000; //choose the time between readings in milliseconds

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
const char topic[]  = "arduino/";
const char username[] = SECRET_MQTT_USER;
const char password[]  =  SECRET_MQTT_PASS;


// constants won't change. They're used here to set pin numbers:
const int button1Pin = D2;    // the number of the pushbutton pin
const int ledPin = D13;      // the number of the LED pin

// Variables will change:
int ledState = HIGH;         // the current state of the output pin
int button1State;             // the current reading from the input pin
int lastButton1State = LOW;   // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime1 = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

#define ScaleReadingsIndex_MAX  100
float ScaleReadings[ScaleReadingsIndex_MAX] = {};
int ScaleReadingsIndex = 0;
float ScaleFluctuationThreshold = 0.3;

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


}

void loop() {
  mqttClient.poll();

  // read the state of the switch into a local variable:
  int readingButton1 = digitalRead(button1Pin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (readingButton1 != lastButton1State) {
    // reset the debouncing timer
    lastDebounceTime1 = millis();
  }

  if ((millis() - lastDebounceTime1) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (readingButton1 != button1State) {
      button1State = readingButton1;

      // only toggle the LED if the new button state is HIGH
      if (button1State == HIGH) {
        ledState = !ledState;
      }
    }
  }

  if (((millis() - lastDebounceTime1) > 30000) & ledState == HIGH) { // after 30 Seconds turn led off
    ledState = isFluctuating(outputValue);
  }

  // set the LED:
  
  digitalWrite(ledPin, ledState);

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButton1State = readingButton1;

  unsigned long currentMillis = millis();  //get the time

  //scale averaging
  if (currentMillis - previousMillis >= interval) {  // if its time for a new reading
    previousMillis = currentMillis;                  //update time of most recent reading
    sensorValue = analogRead(analogInPin);                    // read the input at the analog pin
    fValue = sensorValue * nWeighting + fValue * weighting; //calculate new fValue based on input and previous fValue
    pValue = map(fValue, tareValue, 1023, 0, 3000);
    outputValue = calFactor * pValue;
  }
  
  //send sensor data
  if (currentMillis - previousMillis >= interval2) {}

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


  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(2);
}

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

int isFluctuating(float reading) {
  ScaleReadings[ScaleReadingsIndex] = reading;
  if (ScaleReadingsIndex >= ScaleReadingsIndex_MAX) {
    ScaleReadingsIndex = 0;
  } else {
    ScaleReadingsIndex++;
  }
  int numReadings = sizeof(ScaleReadings) / sizeof(ScaleReadings[0]);
  if (numReadings < 2) {
    return 0; // Not enough readings to determine fluctuation
  }

  float sumOfDifferences = 0;
  for (int i = 1; i < numReadings; i++) {
    sumOfDifferences += fabs(ScaleReadings[i] - ScaleReadings[i - 1]);
  }

  float averageDifference = sumOfDifferences / (numReadings - 1);

  return averageDifference > ScaleFluctuationThreshold;
}
