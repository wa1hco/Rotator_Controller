/* Arduino Rotator Controller "wa1hco Edition"
 *
   Jeff Millar, WA1HCO
   wa1hco@gmail.com
   
   Anthony Good, K3NG
   anthony.good@gmail.com

   Contributions from John Eigenbode, W3SA
   w3sa@arrl.net
   Contributions: AZ/EL testing and debugging, AZ/EL LCD Enhancements, original North center code, Az/El Rotator Control Connector Pins

   Contributions from Jim Balls, M0CKE
   makidoja@gmail.com
   Contributions: Rotary Encoder Preset Support
   
   Contributions from Gord, VO1GPK
   Contribution: FEATURE_ADAFRUIT_BUTTONS code

   ***************************************************************************************************************

    This program is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License
    
                              http://creativecommons.org/licenses/by-nc-sa/3.0/

                          http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode


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

