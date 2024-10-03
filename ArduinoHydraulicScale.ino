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


int sensorValue = 0;        // value read from the pot
int pValue = 0; // pressure
int outputValue = 0;        // value output to the PWM (analog out)
int tareValue = 0;        // value output to the PWM (analog out)
float calFactor = 0.0;        // value output to the PWM (analog out)
int addr = 10;
#define rTsize 30
int rT[rTsize];
int rTindex = 0;
unsigned int rTsum = 0;
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void setup() {
  // initialize serial communications at 9600 bps:
  delay(2000);
  Serial.begin(9600);
  inputString.reserve(200);
  analogReference(DEFAULT);  //on the uno & nano this is VSupply
  nWeighting = 1 - weighting; //calculate weight to be applied to the new reading
  fValue = analogRead(analogInPin);  //initialise to current value of input
  EEPROM.get(addr, calFactor);
}

void loop() {
  unsigned long currentMillis = millis();  //get the time
  if (currentMillis - previousMillis >= interval) {  // if its time for a new reading
    previousMillis = currentMillis;                  //update time of most recent reading
    sensorValue = analogRead(analogInPin);                    // read the input at the analog pin
    fValue = sensorValue * nWeighting + fValue * weighting; //calculate new fValue based on input and previous fValue
    pValue = map(fValue, tareValue, 1023, 0, 3000);
    outputValue = calFactor * pValue;
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
      if (rTindex <= (rTindex - 1)) {
        rT[rTindex] = outputValue;
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
      } else {
        Serial.println("Buffer Empty");
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
        Serial.print("\t ");
      }
      Serial.print("weight = ");
      Serial.println(outputValue);
    }
    else if (inputString == "sum") {
      for (int i = 0; i <= (rTindex - 1); i++) {
        rTsum += rT[i];
      }
    }
    else {
      calFactor = float(inputString.toInt()) / float(pValue);
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
