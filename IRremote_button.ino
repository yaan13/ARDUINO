#include <Arduino.h>
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>
#define DECODE_NEC

const byte ledPin = 2;       // Builtin-LED pin
const byte interruptPin = 0; // BOOT/IO0 button pin
volatile byte state = LOW;


void setup(){
  pinMode(LED_BUILT_IN,OUTPUT);
  Serial.begin(115200);
  Serial.println(F("START "__FILE__ " from "__DATE__" \r\nUsing library version " VERSION_IRREMOTE;

  IrSender.begin();
  Serial.pint(F("Ready to send IR signals at pin "));
  Serial.printn(IR_SEND_PIN); 

  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
}


void loop() {
  digitalWrite(ledPin, state);
}


void blink() {
  state = !state;
}
