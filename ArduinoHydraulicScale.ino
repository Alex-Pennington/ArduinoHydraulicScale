// Author: alex.pennington@organicengineer.com

#define calWeight 530 // the weight of the calibration object

#include <EEPROM.h>

bool debug_flag = false;

const int buttonPin = 2;     // the number of the pushbutton pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
bool buttonPRESSED = false;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

float fValue;  //the filtered value from the last reading
float weighting = 0.7; //the weight (between 0 and 1) given to the previous filter value
float nWeighting;  //the weighting given to the new input value. weighting + nWeighting must equal 1 so we just calculate nWeighting as 1 - weighting
unsigned long previousMillis = 0;
int interval = 10; //choose the time between readings in milliseconds


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
    Meshtastic send/receive client

    Connects to a Meshtastic node via WiFi or Serial (and maybe one day Bluetooth),
    and instructs it to send a text message every SEND_PERIOD milliseconds.
    The destination and channel to use can be specified.

    If the Meshtastic nodes receives a text message, it will call a callback function,
    which prints the message to the serial console.
*/

#include <Meshtastic.h>

// Pins to use for SoftwareSerial. Boards that don't use SoftwareSerial, and
// instead provide their own Serial1 connection through fixed pins
// will ignore these settings and use their own.
#define SERIAL_RX_PIN D0
#define SERIAL_TX_PIN D1
// A different baud rate to communicate with the Meshtastic device can be specified here
#define BAUD_RATE 9600

// Send a text message every this many seconds
#define SEND_PERIOD 300

uint32_t next_send_time = 0;
bool not_yet_connected = true;
// This callback function will be called whenever the radio connects to a node
void connected_callback(mt_node_t *node, mt_nr_progress_t progress) {
  if (not_yet_connected)
    Serial.println("Connected to Meshtastic device!");
  not_yet_connected = false;
}

// This callback function will be called whenever the radio receives a text message
void text_message_callback(uint32_t from, const char* text) {
  // Do your own thing here. This example just prints the message to the serial console.
  Serial.print("Received a text message from ");
  Serial.print(from);
  Serial.print(": ");
  Serial.println(text);
}
// end Meshtastic SendReceiveClient insert


void setup() {
  delay(2000);
  pinMode(buttonPin, INPUT);
  // Try for up to five seconds to find a serial port; if not, the show must go on
  Serial.begin(9600);
  while (true) {
    if (Serial) break;
    if (millis() > 5000) {
      Serial.print("Couldn't find a serial port after 5 seconds, continuing anyway");
      break;
    }
  }
  inputString.reserve(200);
  //analogReference(DEFAULT);  //on the uno & nano this is VSupply
  nWeighting = 1 - weighting; //calculate weight to be applied to the new reading
  fValue = analogRead(analogInPin);  //initialise to current value of input
  //EEPROM.get(addr, calFactor);

  Serial.print("Booted Meshtastic send/receive client in ");

  // Change to 1 to use a WiFi connection
#if 0
#include "arduino_secrets.h"
  Serial.print("wifi");
  mt_wifi_init(WIFI_CS_PIN, WIFI_IRQ_PIN, WIFI_RESET_PIN, WIFI_ENABLE_PIN, WIFI_SSID, WIFI_PASS);
#else
  Serial.print("serial");
  mt_serial_init(SERIAL_RX_PIN, SERIAL_TX_PIN, BAUD_RATE);
#endif
  Serial.println(" mode");

  // Set to true if you want debug messages
  mt_set_debug(false);

  randomSeed(micros());

  // Initial connection to the Meshtastic device
  mt_request_node_report(connected_callback);

  // Register a callback function to be called whenever a text message is received
  set_text_message_callback(text_message_callback);
}

void loop() {
  // Record the time that this loop began (in milliseconds since the device booted)
  unsigned long currentMillis = millis();  //get the time
  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(2);
  if (currentMillis - previousMillis >= interval) {  // if its time for a new reading
    previousMillis = currentMillis;                  //update time of most recent reading
    sensorValue = analogRead(analogInPin);                    // read the input at the analog pin
    fValue = sensorValue * nWeighting + fValue * weighting; //calculate new fValue based on input and previous fValue
    pValue = map(fValue, tareValue, 1023, 0, 3000);
    outputValue = calFactor * pValue;
  }
  
/*
  // read the state of the switch into a local variable:
//  int buttonINPUT = digitalRead(buttonPin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
//  if (buttonINPUT != lastButtonState) {
//    // reset the debouncing timer
//    lastDebounceTime = millis();
//  }
//
//  if ((millis() - lastDebounceTime) > debounceDelay) {
//    // whatever the reading is at, it's been there for longer than the debounce
//    // delay, so take it as the actual current state:
//
//    // if the button state has changed:
//    if (buttonINPUT != buttonState) {
//      buttonState = buttonINPUT;
//
//      // only toggle the LED if the new button state is HIGH
//      if (buttonState == HIGH) {
//        buttonPRESSED = true;
//      }
//    }
//  }
//  if (buttonPRESSED == true) {
//    // Run the Meshtastic loop, and see if it's able to send requests to the device yet
//    bool can_send = mt_loop(now);
//
//    // If we can send, and it's time to do so, send a text message and schedule the next one.
//    if (can_send) {
//
//      // Change this to a specific node number if you want to send to just one node
//      uint32_t dest = BROADCAST_ADDR;
//      // Change this to another index if you want to send on a different channel
//      uint8_t channel_index = 0;
//      char oV[10];
//      String str;
//      str = String(outputValue);
//      str.toCharArray(oV, 10);
//      mt_send_text(oV, dest, channel_index);
//
//      buttonPRESSED = LOW;
//    }
//  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
//  lastButtonState = buttonINPUT;
*/

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
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
        Serial.print(tareValue);
        Serial.print("\t c = ");
        Serial.print(calFactor);
        Serial.print(rTindex);
        Serial.print("\t ");
      }
      Serial.print("weight = ");
      Serial.println(outputValue);

      // Run the Meshtastic loop, and see if it's able to send requests to the device yet
      bool can_send = mt_loop(currentMillis);

      // If we can send, and it's time to do so, send a text message and schedule the next one.
      if (can_send) {

        // Change this to a specific node number if you want to send to just one node
        uint32_t dest = BROADCAST_ADDR;
        // Change this to another index if you want to send on a different channel
        uint8_t channel_index = 1;
        char oV[10];
        String str;
        str = String(outputValue);
        str.toCharArray(oV, 10);
        mt_send_text(oV, dest, channel_index);
      }
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
