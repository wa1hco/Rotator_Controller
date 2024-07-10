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
#include <MsTimer2.h>

// C++ functions
#include <math.h> 

// Project configuration
#include "dependencies.h"

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
  float ADCtop = (float) analogRead(AzPositionTopPin); // adc reading for top    of azimuth pot
  float ADCbot = (float) analogRead(AzPositionBotPin); // adc reading for bottom of azimuth pot

  // FIR filter output, LPF, 12 Hz pass, 40 Hz and up at -60 dB, Fs = 200 Hz, 31 taps
  float FIRtop = fir_top.processReading(ADCtop);
  float FIRbot = fir_bot.processReading(ADCbot);

  // constants related to hardware
  static float Vs     =    3.3; // Volts, azimuth pot bias voltage
  static float Rbias  =  330;   // Ohms, bias resistor to each end of pot
  static float Rpot   =  500;   // Ohms, rotator pot specified 500 Ohms
  static float Rc     =    1;   // Ohms, cable resistance
  static float ADCmax = 1023;

  Vtop = Vs * FIRtop / ADCmax; // Volts, at top    of azimuth pot
  Vbot = Vs * FIRbot / ADCmax; // Volts, at bottom of azimuth pot

  // WA8USA equations
  // The simplest solution is to express the voltage at the wiper in terms 
  // of the drop in each side from the supply.  
  // The current in the top path is (Vs-Vtop)/Rbias.  
  // The voltage drop to the wiper is that current times Rs+Rc+Rtop.  
  // Subtracting that voltage drop from the supply gives the voltage at the wiper.  
  // The two equations for the voltage Vw at the wiper are

  // Vw=Vs-((Vs-Vtop)/Rbias * (Rbias+Rc+Rtop))
  // Vw=Vs-((Vs-Vbot)/Rbias * (Rbias+Rc+Rbot))

  // Setting these two to be equal, cancelling Vs on both sides, 
  // and then multiplying both sides by Rbias, and substituting 500-Rtop for Rbot gives

  // (Vtop-Vs) * (Rbias+Rc+Rtop) = (Vbot-Vs) * (Rbias+Rc+500-Rtop)

  // Isolating all the terms with Rtop on the left results in

  // Rtop*(2*Vs-Vtop-Vbot)=(Rbias+Rc)*(Vtop-Vbot)+500*(Vs-Vbot)

   // rename Rtop to analog_az for compatibility with K3NG functions
  float Rtop = ((Rbias + Rc) * (Vtop - Vbot) + Rpot * (Vs - Vbot)) / (2 * Vs - Vtop - Vbot);
  analog_az = Rtop;

  float analog_az_ccw = configuration.analog_az_full_ccw; // ohms, min analog_az
  float analog_az_cw  = configuration.analog_az_full_cw;  // Ohms, max analog_az
  float Az_start      = configuration.azimuth_starting_point * HEADING_MULTIPLIER;
  float Az_capability = configuration.azimuth_rotation_capability;  // degress of rotation
  float Az_stop       = Az_start + Az_capability * HEADING_MULTIPLIER;

  if (analog_az < analog_az_ccw)  analog_az = analog_az_ccw; // clamp to lower limit
  if (analog_az > analog_az_cw)   analog_az = analog_az_cw;  // clamp to upper limit

  // Rtop is nominally 0 to 500 Ohms for full CCW to full CW rotation
  // typically analog_az from 8 to 483 (Ohms) maps to 0 to 360 degrees
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
      Serial.println("Time, Vtop, Vbot, analog_az, AzTop, AzMap"); // header line
      isFirstWrite = false;
    }
    Time_usec = micros();;
    Serial.print(Time_usec);
    Serial.print(", ");
    Serial.print(Vtop);         // top voltage
    Serial.print(", ");
    Serial.print(Vbot);         // bottom voltage
    Serial.print(", ");
    Serial.print(analog_az);  // Ohms, Rtop, top part of pot
    Serial.print(", ");
    Serial.print(azimuth);    // deg, from mapping Rtop and Cal to azimuth
    Serial.println();
  }
  #endif // ifdef debug HCO
  
} // ReadAzimuthISR()
