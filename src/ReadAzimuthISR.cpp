/* Arduino Rotator Controller "wa1hco Edition"
 *
   Jeff Millar, WA1HCO
   wa1hco@gmail.com
   
   Anthony Good, K3NG
   anthony.good@gmail.com

   Contributions from John Eigenbode, W3SA
   w3sa@arrl.net
   Contributions: AZ/EL testing and debugging, AZ/EL LCD Enhancements, original North center code, Az/El Rotator Control Connector Pins

*/

// Arduino environment
#include <Arduino.h>
#include <avr/interrupt.h>
#include <MsTimer2.h>

// C++ functions
#include <math.h> 

// Project configuration
#include "global_variables.h"
#include "rotator_features.h"
#include "rotator_pins_HCO_board.h"

#include <FIR.h>
extern FIR<float, 31> fir_top;
extern FIR<float, 31> fir_bot;

//*********************************************************************************
// Entered at TimeBetweenInterrupt intervals, 
// Called from interrupt context, ADC reads occur at 2 ms, 500 Hz
// read both ADC input close together in time
// then do the peak processing
// This function is called from MsTimer2 every TIME_BETWEEN_INTERRUPTS msec
void ReadAzimuthISR() 
{
  // read + and - ends of pot with grounded wiper

  float ADCt = (float) analogRead(PositionPosPin); // adc reading for top    of azimuth pot
  float ADCb = (float) analogRead(PositionNegPin); // adc reading for bottom of azimuth pot

  // FIR filter output, LPF, 12 Hz pass, 40 Hz and up at -60 dB, Fs = 200 Hz, 31 taps
  float FIRt = fir_top.processReading(ADCt);
  float FIRb = fir_bot.processReading(ADCb);

  // constants related to hardware
  static float Vs     =    3.3; // Volts, azimuth pot bias voltage
  static float Rb     =  330;   // Ohms, bias resistor to each end of pot
  static float Rp     =  500;   // Ohms, rotator pot specified 500 Ohms
  static float Rc     =    1;   // Ohms, cable resistance
  static float ADCmax = 1023;

  Vt = Vs * FIRt / ADCmax; // Volts, at top    of azimuth pot
  Vb = Vs * FIRb / ADCmax; // Volts, at bottom of azimuth pot

  // WA8USA equations
  // float Vwt=Vs-((Vs-Vtop)/Rb*(Rb+Rc+Rtop))
  // float Vwb=Vs-((Vs-Vbot)/Rb*(Rb+Rc+Rbot))
  // Setting these two to be equal, cancelling U on both sides, 
  // and then multiplying both sides by Rb, and substituting 500-Rtop for Rbot gives
  // (Vtop-U)*(Rb+Rc+Rtop)=(Vbot-U)*(Rb+Rc+500-Rtop)
  // Isolating all the terms with Rtop on the left results in
  // Rtop*(2*U-Vtop-Vbot)=(Rb+Rc)*(Vtop-Vbot)+500*(U-Vbot)
  // rename Rtop to analog_az for compatibility with K3NG functions
  float Rt = ((Rb + Rc) * (Vt - Vb) + Rp * (Vs - Vb)) / (2 * Vs - Vt - Vb);
  analog_az = Rt;

  // map(value, fromLow, fromHigh, toLow, toHigh)
  
  //azimuth = map(analog_az, 0, 500, 0, 360); // map from pot to azimuth reading
  float analog_az_ccw = configuration.analog_az_full_ccw; // ohms, min analog_az
  float analog_az_cw  = configuration.analog_az_full_cw;  // Ohms, max analog_az
  float Az_start      = configuration.azimuth_starting_point * HEADING_MULTIPLIER;
  float Az_capability = configuration.azimuth_rotation_capability;  // degress of rotation
  float Az_stop       = Az_start + Az_capability * HEADING_MULTIPLIER;

  if (analog_az < analog_az_ccw)  analog_az = analog_az_ccw; // clamp to lower limit
  if (analog_az > analog_az_cw)   analog_az = analog_az_cw;  // clamp to upper limit

  azimuth = (int) map(analog_az, analog_az_ccw, analog_az_cw, Az_start, Az_stop);

  AzFiltered = azimuth;  // for compatibility

  #ifdef DEBUG_HCO_ADC
  if (debug_mode == 1)
  {
    static bool isFirstWrite = true;
    float Time_usec;
    // format for csv file with header
    if (isFirstWrite)
    {
      Serial.println("HCO Board analog");
      Serial.println("Time, Vt, Vb, analog_az, AzTop, AzMap"); // header line
      isFirstWrite = false;
    }
    Time_usec = micros();;
    Serial.print(Time_usec);
    Serial.print(", ");
    Serial.print(Vt);         // top voltage
    Serial.print(", ");
    Serial.print(Vb);         // bottom voltage
    Serial.print(", ");
    Serial.print(analog_az);  // Ohms, Rt, top part of pot
    Serial.print(", ");
    Serial.print(azimuth);    // deg, from mapping Rt and Cal to azimuth
    Serial.println();
  }
  #endif // ifdef debug HCO
  
} // ReadAzimuthISR()
