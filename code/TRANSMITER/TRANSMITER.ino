/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)

 RADIO NUMBER 0
 ROLE TX (TRUE)
 */
#include <SPI.h>                                                     //
#include "printf.h"
#include "RF24.h"

RF24 radio(7, 8);  // pin 7 for CE, pin 8 for CSN                    //
uint8_t address[][6] = { "1Node", "2Node" };                         //

int downHandPin = 2;
int upHandPin = 4;
int weaponPin = 5;
int leftPin = A2;
int rightPin = A1;

int downHandValue;                                                   //
int upHandValue;
int weaponValue;
int leftValue;
int rightValue;

int motorStep = 10;

struct Controls {
  int hand;     // value for hand position  || from 0 to 100
  bool weapon;  // value for weapon         || true/false
  int left;     // value for left wheel     || from -77 to +77       || from -19 000 to +19 000
  int right;    // value for right wheel    || from -77 to +77       || from -19 000 to +19 000
};
Controls payload;

void setup() {
  Serial.begin(9600);                                                //

  pinMode(downHandPin, INPUT_PULLUP);
  pinMode(upHandPin, INPUT_PULLUP);
  pinMode(weaponPin, INPUT_PULLUP);

  payload.hand = 0;
  payload.weapon = false;
  payload.left = 0;
  payload.right = 0;

  // initialize the transceiver on the SPI bus                       //
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.         //
  
  radio.setPayloadSize(sizeof(payload));                             //
  
  radio.openWritingPipe(address[0]);                                 // always uses pipe 0
  radio.stopListening();
} // setup
  
void loop() {
  checkInputs(); // check control inputs                             //
  
  // transmit & save the report                                      //
  bool report = radio.write(&payload, sizeof(payload));
  
  if (report) {
      // payload was delivered                                       //
      Serial.print(F("Transmission successful! "));  
    } else {
      // payload was not delivered                                   //
      Serial.println(F("Transmission failed or timed out")); 
    }

  delay(500);                                                        // to make readable
} // loop

void checkInputs() {                                                 //
  downHandValue = digitalRead(downHandPin);
  upHandValue = digitalRead(upHandPin);
  weaponValue = digitalRead(weaponPin);

  getHandPosition();  // calculating the hand position               //
  checkWeapon();      // checking the weapon permition               //
  getRightPWM();      // reading right speed                         //
  getLeftPWM();       // reading left speed                          //
} // checkInputs

void getHandPosition() {                                             //
  if (downHandValue == LOW) {
    payload.hand -= motorStep;
  }
  if (upHandValue == LOW) {
    payload.hand += motorStep;
  }
} // getHandPosition

void checkWeapon() {                                                 //
  if (weaponValue == LOW) {
    payload.weapon = true;
  } else {
    payload.weapon = false;
  }
} // checkWeapon

void getRightPWM() {                                                 //
  // reading the potentiometer value                                 //
  rightValue = analogRead(A1);

  // manipulating the potentiometer value
  if (rightValue > 455 && rightValue < 485) {
    rightValue = 0;
  } else if (rightValue <= 455) {
    rightValue -= 355;
    if (rightValue  < 0) {
      rightValue = 100;
    }
    rightValue *= -1;
  } else if (rightValue >= 485) {
    rightValue -= 485;
  }

  // putting the result in the correct interval
  if (rightValue > 100) {
    rightValue = 100;
  }

  payload.right = rightValue;
} // getRightPWM

void getLeftPWM() {                                                  //
  // reading the potentiometer value                                 //
  leftValue = analogRead(A2);
  leftValue -= 30;

  // manipulating the potentiometer value
  if (leftValue > 485 && leftValue < 515) {
    leftValue = 0;
  } else if (leftValue <= 485) {
    leftValue -= 385;
    leftValue = 100 - leftValue;
    if (leftValue  < 0) {
      leftValue = 100;
    }
    leftValue *= -1;
  } else if (leftValue >= 515) {
    leftValue -= 515;
  }

  // putting the result in the correct interval
  if (leftValue > 100) {
    leftValue = 100;
  }

  payload.left = leftValue;
} // getLeftPWM
