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
*/

// Arduino environment
#include <Arduino.h>

// C++ functions
#include <math.h> 

// Project configuration
#include "dependencies.h"

extern FIR<float, 31> lpf_top;
extern FIR<float, 31> lpf_bot;

//*********************************************************************************
// Entered at TimeBetweenInterrupt intervals, 
// Called from interrupt context, ADC reads occur at 2 ms, 500 Hz
// read both top and bottom ADC inputs
// ADC voltages biased up because motor and brake current ripple can drive top of pot negative
// ADC circuit has twin-T 60 Hz notch to suppress most ripple
// FIR filter further suppresses ripple, and smooths the values
// Both top and bottom reading are combined to cancel variable wiper resistance

// This function is called from MsTimer2 every TIME_BETWEEN_INTERRUPTS msec
void ReadAzimuthCDE() 
{
  // read + and - ends of pot with grounded wiper
  float ADC_top = (float) analogRead(AzPositionTopPin); // adc reading for top    of azimuth pot
  float ADC_bot = (float) analogRead(AzPositionBotPin); // adc reading for bottom of azimuth pot

  // FIR filter the ADC readings, LPF, 12 Hz pass, 40 Hz and up at -60 dB, Fs = 200 Hz, 31 taps
  float LPF_top = lpf_top.processReading(ADC_top);
  float LPF_bot = lpf_bot.processReading(ADC_bot);

  // constants related to hardware
  #define      ADC_MAX  1023.0  
  #define      VS        3.3     // Volts, azimuth pot bias voltage
  #define      RBIAS     330     // Ohms, bias resistor to each end of pot
  #define      RPOT      500     // Ohms, rotator pot specified 500 Ohms
  #define      RCABLE      1     // Ohms, each wire of 100 ft of rotor cable

  // Calculate the filtered voltage from low pass filtered ADC reading
  // ADC voltage ranges near 0 to 1.95 volts due to bias resistor
  float Vtop = VS * LPF_top / ADC_MAX; // Volts, at top    of azimuth pot
  float Vbot = VS * LPF_bot / ADC_MAX; // Volts, at bottom of azimuth pot

  // WA8USA analysis
  // The simplest solution is to express the voltage at the wiper in terms 
  // of the drop in each side from the supply. 
  // The voltage drop to the wiper is that current times RBIAS+RCABLE+Rtop.  
  // Subtracting that voltage drop from the supply gives the voltage at the wiper.  

  //    Vw = VS - ((VS - Vtop) / RBIAS * (RBIAS + RCABLE + Rtop));
  //    Vw = VS - ((VS - Vbot) / RBIAS * (RBIAS + RCABLE + Rbot));

  // Setting these two to be equal, cancelling VS on both sides, 
  // and then multiplying both sides by RBIAS, and substituting 500-Rtop for Rbot gives

  //    (Vtop-VS) * (RBIAS+RCABLE+Rtop) = (Vbot-VS) * (RBIAS+RCABLE+500-Rtop)

  // Isolating all the terms with Rtop on the left results in

  //    Rtop*(2*VS-Vtop-Vbot)=(RBIAS+RCABLE)*(Vtop-Vbot)+500*(VS-Vbot)

 // The current at the two ends of the pot... 
  float Itop = (VS - Vtop) / RBIAS;    // Amps, current through top leg
  float Ibot = (VS - Vbot) / RBIAS;    // Amps, current through bottom leg
  float Iwiper = Itop + Ibot;

  // rename Rtop to analog_az for compatibility with K3NG functions
  float Rtop = ((RBIAS + RCABLE) * (Vtop - Vbot) + RPOT * (VS - Vbot)) / (2 * VS - Vtop - Vbot);
  float Rbot = 500 - Rtop;

  analog_az = Rtop; // legacy variable name

  // The two equations for the voltage Vw, Vw at the wiper
  float Vwipert = Vtop - (Itop * (Rtop + RCABLE));
  float Vwiperb = Vbot - (Ibot * (Rbot + RCABLE)) ;  

  float Rwipert = Vwipert / Iwiper;
  float Rwiperb = Vwiperb / Iwiper;

  float analog_az_ccw = configuration.analog_az_full_ccw; // ohms, min analog_az
  float analog_az_cw  = configuration.analog_az_full_cw;  // Ohms, max analog_az
  float Az_start      = configuration.azimuth_starting_point * HEADING_MULTIPLIER;
  float Az_capability = configuration.azimuth_rotation_capability;  // degress of rotation
  float Az_stop       = Az_start + Az_capability * HEADING_MULTIPLIER;

  if (analog_az < analog_az_ccw)  analog_az = analog_az_ccw; // clamp to lower limit
  if (analog_az > analog_az_cw)   analog_az = analog_az_cw;  // clamp to upper limit

  // Rtop is nominally 0 to 500 Ohms for full CCW to full CW rotation
  // typically analog_az from 8 to 483 (Ohms) maps to 0 to 360 degrees
  raw_azimuth = (int) map(analog_az, analog_az_ccw, analog_az_cw, Az_start, Az_stop);
  azimuth = raw_azimuth;  // Expand this to include wrapping if necessary
  
  // calculate the wiper resistance
  // Itop + Ibot flows through wiper

  //    Vw=VS-((VS-Vtop)/RBIAS * (RBIAS+RCABLE+Rtop+Rwiper))
  //    Vw=VS-((VS-Vbot)/RBIAS * (RBIAS+RCABLE+Rbot+Rwiper))

  //    (Vtop-VS) * (RBIAS+RCABLE+Rtop+Rwiper) = (Vbot-VS) * (RBIAS+RCABLE+500-Rtop+Rwiper)
  //    (Vtop * (RBIAS+RCABLE+Rtop+Rwiper) - (VS * (RBIAS+RCABLE+Rtop+Rwiper)) = (Vbot * (RBIAS+RCABLE+Rbot+Rwiper) - (VS * (RBIAS+RCABLE+Rbot+Rwiper)) 

  // this writes lines of ADC data to the serial port for each ADC read
  //
  #ifdef DEBUG_HCO_ADC
  if (debug_mode == 1)
  {
    static bool isFirstWrite = true;
    float Time_usec;
    // format for csv file with header
    if (isFirstWrite)
    {
      Serial.println("HCO Board analog");
      Serial.println("Time, Vtop, Vbot, Itop (ma), Ibot (ma), Vwt, Vwb, Rwt, Rwb, Rtop, Az"); // header line
      isFirstWrite = false;
    }
    Time_usec = micros();;
    Serial.print(Time_usec);
    Serial.print(", ");
    Serial.print(Vtop);       // volts, az pot top, biased up, CW makes higher
    Serial.print(", ");
    Serial.print(Vbot);       // volts, az pot bottom, biased up, CCW makes higher
    Serial.print(", ");
    Serial.print(Vwipert);    // volts, wiper top, derived from from Itop and Rtop
    Serial.print(", ");
    Serial.print(Vwiperb);    // volts, wiper bottom, derived from from Ibot and Rbot
    Serial.print(", ");
    Serial.print(Rwipert);    // ohms, wiper resistance, derived from top and bottom currents
    Serial.print(", ");
    Serial.print(Rwiperb);    // ohms, wiper resistance, derived from top and bottom currents
    Serial.print(", ");
    Serial.print(Rtop);       // ohms, Rtop, top to wiper, CW makes higher
    Serial.print(", ");
    Serial.print(azimuth);    // deg, azimuth, filtered and calibrated
    Serial.println();

  }
  #endif // ifdef debug HCO
  
} // ReadAzimuthCDE()
