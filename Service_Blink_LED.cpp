/*
 * Service_Blink_LED.c
 *
 *  Created on: Feb 19, 2017
 *      Author: jeff
 */
//--------------------------------------------------------------

#include <Arduino.h>
#include "rotator_features.h"
#include "rotator_pins_HCO_board.h"

void service_blink_led()
{
  static unsigned long last_blink_led_transition = 0;
  static byte blink_led_status = 0;

  #ifdef blink_led
  if ((millis() - last_blink_led_transition) >= 1000){
    if (blink_led_status){
      digitalWrite(blink_led, LOW);
      blink_led_status = 0;
    } else {
      digitalWrite(blink_led, HIGH);
      blink_led_status = 1;
    }
    last_blink_led_transition = millis();
  }
  #endif //blink_led
}

