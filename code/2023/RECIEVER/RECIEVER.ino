/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)

 RADIO NUMBER 1
 ROLE RX (FALSE)
 */
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

RF24 radio(7, 8);  // using pin 7 for the CE pin, and pin 8 for the CSN pin
uint8_t address[][6] = { "1Node", "2Node" };

const int pwmWeaponPin = 6;
const int dirLeftPin = 2;
const int pwmLeftPin = 3;
const int dirRightPin = 4;
const int pwmRightPin = 5;                                           //
const int stepPin = 9;
const int dirPin = 10;

int handPosition = 0;    // 'real' hand position
int maxMotorSpeed = 255;  // 77 for arduino, 19000 for blue pill

struct Controls {
  int hand;     // value for hand position  || from 0 to 100
  bool weapon;  // value for weapon         || true/false
  int left;     // value for left wheel     || from -77 to +77       || from -19 000 to +19 000
  int right;    // value for right wheel    || from -77 to +77       || from -19 000 to +19 000
};
Controls payload;

void setup() {
  //Serial.begin(9600);

  pinMode(pwmWeaponPin, OUTPUT);                                     //
  pinMode(dirLeftPin, OUTPUT);
  pinMode(pwmLeftPin, OUTPUT);
  pinMode(dirRightPin, OUTPUT);
  pinMode(pwmRightPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {                                              //
    //Serial.println(F("radio hardware is not responding!!"));
    while (1) {}                                                     // hold in infinite loop
  }

  radio.setPALevel(RF24_PA_LOW);                                     // RF24_PA_MAX is default.

  radio.setPayloadSize(sizeof(payload));                             // float datatype occupies 4 bytes
  
  radio.openWritingPipe(address[1]);                                 // always uses pipe 0
  radio.openReadingPipe(1, address[0]);                              // using pipe 1
  radio.startListening();                                            // put radio in RX mode
} // setup
  
void loop() {
  uint8_t pipe;
  
  // check for recieved commands
  if (radio.available(&pipe)) {                                      // get the pipe number that recieved it
    uint8_t bytes = radio.getPayloadSize();                          // get the size of the payload
    radio.read(&payload, bytes);                                     // fetch payload from FIFO
    //Serial.print(F("Received "));
  }

  // move the hand in the chosen direction                           //
    if(payload.hand > handPosition) {
      digitalWrite(dirPin, HIGH);
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
      handPosition++;
    } else if (payload.hand < handPosition) {
      digitalWrite(dirPin, LOW);
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
      handPosition--;
    } else {
      digitalWrite(stepPin, LOW);
    }

    // moving the cutting disk                                       //
    if(payload.weapon == true) {
      analogWrite(pwmWeaponPin, maxMotorSpeed);
    } else {
      analogWrite(pwmWeaponPin, 0);
    } 
    
    // moving the left wheel                                         //
    if(payload.left > 0) {
      digitalWrite(dirLeftPin, HIGH);
      analogWrite(pwmLeftPin, payload.left);
    } else {
      payload.left *= -1;
      digitalWrite(dirLeftPin, LOW);
      analogWrite(pwmLeftPin, payload.left);
    }

    //moving the right wheel                                         //
    if(payload.right > 0) {
      digitalWrite(dirRightPin, HIGH);
      analogWrite(pwmRightPin, payload.right);
    } else {
      payload.right *= -1;
      digitalWrite(dirRightPin, LOW);
      analogWrite(pwmRightPin, payload.right);
    }
} // loop
