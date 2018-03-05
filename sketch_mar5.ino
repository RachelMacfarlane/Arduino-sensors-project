/* 
 *  This sketch is an attempt to allow a set text message to be sent when a sensor is triggered.
 It uses button and FonaTest */
/*
  Button

  Turns on and off a light emitting diode(LED) connected to digital pin 13,
  when pressing a pushbutton attached to pin 2.

  The circuit:
  - LED attached from pin 13 to ground
  - pushbutton attached to pin 2 from +5V
  - 10K resistor attached to pin 2 from ground

  - Note: on most Arduinos there is already an LED on the board
    attached to pin 13.

  created 2005
  by DojoDave <http://www.0j0.org>
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Button
*/

/***************************************************
Arduino 1.5.7
 ****************************************************/

/***************************************************
  This is an example for our Adafruit FONA Cellular Module

  Designed specifically to work with the Adafruit FONA
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963
  ----> http://www.adafruit.com/products/2468
  ----> http://www.adafruit.com/products/2542

  These cellular modules use TTL Serial to communicate, 2 pins are
  required to interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/*
THIS CODE IS STILL IN PROGRESS!

Open up the serial console on the Arduino at 115200 baud to interact with FONA

Note that if you need to set a GPRS APN, username, and password scroll down to
the commented section below at the end of the setup() function.
*/

// constants won't change. They're used here to set pin numbers:
const int waterLevelPin = 4;     // the number of the water level sensor pin
const int gasPin = 5;       // the number of the gas sensor pin 
const int ledPin =  13;      // the number of the LED pin

// variables will change:
int waterLevelState = 0;         // variable for reading the water level status
int gasState = 0;           // variable for reading the gas sensor status

/////////////////////////////

#include "Adafruit_FONA.h"

#define ONE_SECOND 1000
#define FONA_TX 2
#define FONA_RX 3
#define FONA_RST 9

// this is a large buffer for replies
char replybuffer[255];

boolean first = true;

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX,FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Use this one for FONA 3G
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

uint8_t type;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the water level pin as an input:
  pinMode(waterLevelPin, INPUT);
  // initialize the gas sensor pin as an input:
  pinMode(gasPin, INPUT);
  ////////////////////////////////////////
  while (!Serial);

  Serial.begin(115200);
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  type = fona.type();
  
  // Print module IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }
}

void loop() {
  // read the state of the water level sensor:
  waterLevelState = digitalRead(waterLevelPin);
  // read the state of the gas sensor:
  gasState = digitalRead(gasPin);

  // check if the water level sensor is triggered. If it is, the waterLevelState is LOW:
  if (waterLevelState == LOW) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);

  // check if the gas sensor is triggered. If it is, the gasState is HIGH:
  //if (gasState == LOW) {
    // turn LED on:
    //digitalWrite(ledPin, HIGH);
    
  /////////////////////////////
  if (first) {
    Serial.print(F("FONA> "));
      // send an SMS!
      char sendto[11] = "0468693673", message[] = "Adding gas sensor";
      flushSerial();
      if (!fona.sendSMS(sendto, message)) {
        Serial.println(F("Failed"));
      } else {
        Serial.println(F("Sent!"));
        first = false;
        // turn LED off:
        digitalWrite(ledPin, LOW);
      }
    }
    // UNCOMMENT THE FOLLOWING TO SEND EVERY 30SECONDS
    //first = true;
      
  // flush input
  flushSerial();
  while (fona.available()) {
    Serial.write(fona.read());
  }

  delay(ONE_SECOND*30);
  //////////////////////////////
    
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }

}

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

