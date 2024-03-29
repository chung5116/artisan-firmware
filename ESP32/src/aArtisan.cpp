#include<Arduino.h>
#include <thermocouple.h>

#include <cmndproc.h>
// aArtisan.ino
// ------------

// Written to support the Artisan roasting scope //http://code.google.com/p/artisan/

// *** BSD License ***
// ------------------------------------------------------------------------------------------
// Copyright (c) 2011, MLG Properties, LLC
// All rights reserved.
//
// Contributor:  Jim Gallt
//
// Redistribution and use in source and binary forms, with or without modification, are 
// permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this list of 
//   conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this list 
//   of conditions and the following disclaimer in the documentation and/or other materials 
//   provided with the distribution.
//
//   Neither the name of the copyright holder(s) nor the names of its contributors may be 
//   used to endorse or promote products derived from this software without specific prior 
//   written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ------------------------------------------------------------------------------------------

#define BANNER_ARTISAN "aARTISAN V3.10"

// Revision history:
// 20110408 Created.
// 20110409 Reversed the BT and ET values in the output stream.
//          Shortened the banner display time to avoid timing issues with Artisan
//          Echo all commands to the LCD
// 20110413 Added support for Artisan 0.4.1
// 20110414 Reduced filtering levels on BT, ET
//          Improved robustness of checkSerial() for stops/starts by Artisan
//          Revised command format to include newline character for Artisan 0.4.x
// 20110528 New command language added (major revision)
//          Use READ command to poll the device for up to 4 temperature channels
//          Change temperature scale using UNITS command
//          Map physical channels on ADC to logical channels using the CHAN command
//          Select SSR output duty cycle with OT1 and OT2 commands
//          Select PWM logic level output on I/O3 using IO3 command
//          Directly control digital pins using DPIN command (WARNING -- this might not be smart)
//          Directly control analog pins using APIN command (WARNING -- this might not be smart)
// 20110601 Major rewrite to use cmndproc.h library
//          RF2000 and RC2000 set channel mapping to 1200
// 20110602 Added ACKS_ON to control verbose output
// ----------- Version 1.10
// 20111011 Added support for type J and type T thermocouples
//          Better error checking on EEPROM reads
// ----------- Version 2.00 (for Arduino 1.0)
// 20120126 Created .ino source file.
//          Serial output now fixed at one decimal place
//          Better echo display of serial commands.
//          Raw ADC conversions are filtered before converting to temperatures.
// ------------ Version 2.10
// 20130125 Permits use of different TC types on individual channels
// ------------- 13-April 2014 Version 3RC1
//          Increase output precision to 2 decimal places
//          Increases serial baud rate to 115,200
//          Places limitations on fan ramp-up rate (slew rate) with new DCFAN command
//          Abandon support for legacy rf2000 and rc2000 commands
// ------------- 15-April-2014 Release Version 3.0
// --------------17-April-2014
//          PID commands added, limited testing done.
// --------------19-April-2014
//          Added PID,CT command for adjustable sample time
// -------------- 04-Aug-2014
//          Code to explicitly turn off pin 13 LED at startup
// --------------22-October-2014
//          Added outputs for heater level, fan level, and SV
// -----28-October-2104
//          Add FILT command for runtime digital filtering levels
// -----01-July-2015  release 3.10
//          No revisions -- this is just an official release of version 3.10

// this library included with the arduino distribution
#include <Wire.h>

// The user.h file contains user-definable and some other global compiler options
// It must be located in the same folder as aArtisan.pde
#include "user.h"

// command processor declarations -- must be in same folder as aArtisan
#include "cmndreader.h"

#ifdef MEMORY_CHK
// debugging memory problems
#include "MemoryFree.h"
#endif

// these "contributed" libraries must be installed in your sketchbook's arduino/libraries folder
#include <cmndproc.h> // 20140426 version 1.03 or later required
#include <thermocouple.h> // type K, type J, and type T thermocouple support
#include <cADC.h> // MCP3424
#include <PWM16.h> // for SSR output
#include <PID_v1.h> // Brett Beauregard's PID library (GPLv3)
#include <mcEEPROM.h> // EEPROM calibration data
#ifdef LCD
#include <cLCD.h> // required only if LCD is used
#endif

// ------------------------ other compile directives
#define MIN_DELAY 300   // ms between ADC samples (tested OK at 270)
#define DP 2  // decimal places for output on serial port
#define D_MULT 0.001 // multiplier to convert temperatures from int to float
#define DELIM "; ,=" // command line parameter delimiters
#define LEDPIN 13 // on board LED

mcEEPROM eeprom;
calBlock caldata;

float AT; // ambient temp
float T[NC];  // final output values referenced to physical channels 0-3
uint8_t actv[NC];  // identifies channel status, 0 = inactive, n = physical channel + 1
#ifdef CELSIUS // only affects startup conditions -- UNITS command determines runtime units
boolean Cscale = true;
#else
boolean Cscale = false;
#endif

int levelOT1, levelOT2, levelIO3;  // parameters to control output levels
uint32_t lcd_count; // for echo display of serial commands

// class objects
cADC adc( A_ADC ); // MCP3424
ambSensor amb( A_AMB ); // MCP9800
filterRC fT[NC]; // filter for logged ET, BT
PWM16 ssr;  // object for SSR output on OT1, OT2
CmndInterp ci( DELIM ); // command interpreter object

//Define PID Variables we'll be connecting to
double Setpoint = 0, Input, Output;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, PRO, INT, DER, DIRECT);
uint8_t pid_chan = PID_CHAN; // identify PV

// array of thermocouple types
tcBase * tcp[4];
TC_TYPE1 tc1;
TC_TYPE2 tc2;
TC_TYPE3 tc3;
TC_TYPE4 tc4;

// ---------------------------------- LCD interface definition
#ifdef LCD
// LCD output strings
char st1[6],st2[6];
#ifdef LCDAPTER
  #define BACKLIGHT lcd.backlight();
  cLCD lcd; // I2C LCD interface
#else // parallel interface, standard LiquidCrystal
  #define BACKLIGHT ;
  #define RS 2
  #define ENABLE 4
  #define D4 7
  #define D5 8
  #define D6 12
  #define D7 13
  LiquidCrystal lcd( RS, ENABLE, D4, D5, D6, D7 ); // standard 4-bit parallel interface
#endif
#endif
// --------------------------------------------- end LCD interface

// ------------- wrapper for the command interpreter's serial line reader
void checkSerial() {
  const char* result = ci.checkSerial();
  if( result != NULL ) { // some things we might want to do after a command is executed
    #ifdef LCD
    lcd.setCursor( 0, 1 ); // echo all commands to the LCD
    lcd.print( "         ");
    lcd.setCursor( 0, 1 ); // echo all commands to the LCD
    lcd.print( result );
    lcd_count = millis();
    #endif
    #ifdef MEMORY_CHK
    Serial.print("# freeMemory()=");
    Serial.println(freeMemory());
    #endif
  }
}

// ----------------------------------------------------
float convertUnits ( float t ) {
  if( Cscale ) return F_TO_C( t );
  else return t;
}

// ---------------------- PID calculations
void doPID() {
  if( myPID.GetMode() != MANUAL ) { // If PID in AUTOMATIC mode calc new output and assign to OT1
    // pid_chan is 1, 2, 3, or 4, based on the order of temperature reading outputs
    // Order of temperature channel outputs is set by the "chan" command
    uint8_t k = actv[pid_chan - 1];  // k = physical channel corresponding with logical channel
    if( k != 0 ) --k; // adjust for 0-based array index
    // Input is the SV for the PID algorithm
    Input = convertUnits(T[k]);
    myPID.Compute();  // do PID calcs
    levelOT1 = Output; // update OT1 based on PID output
    #ifdef ACKS_ON
    Serial.print("# PID input = " ); Serial.print( Input ); Serial.print( "  ");
    
    Serial.print("# PID output = " ); Serial.println( levelOT1 );
    #endif
    ssr.Out( levelOT1, levelOT2 ); // by default, PID controls the output on OT1
  }
}

// ----------------------------------
void checkStatus( uint32_t ms ) { // this is an active delay loop
  uint32_t tod = millis();
  while( millis() < tod + ms ) {
    checkSerial();
    dcfan.slew_fan(); // keep the fan smoothly increasing in speed
    doPID();  // update output if PID is running
  }
}



// ------------------------------------------------------------------
void logger()
{
// print ambient
  Serial.print( convertUnits( AT ), DP );
  Serial1.print("hello.");
// print active channels
  for( uint8_t jj = 0; jj < NC; ++jj ) {
    uint8_t k = actv[jj];
    // jj = logical channel, k = physical channel
    if( k > 0 ) {
      --k;
      Serial.print(",");
 
      Serial.print( convertUnits( T[k] ), DP );
      
    }
  }
// check to see if PID is running, and output additional values if true
  if( myPID.GetMode() != MANUAL ) { // If PID in AUTOMATIC mode
    Serial.print(",");
    Serial.print( HEATER_DUTY ); 
    Serial.print(",");
    Serial.print( FAN_DUTY );
    Serial.print(",");
    Serial.print( SV );
  }  
  Serial.println();
}

// --------------------------------------------------------------------------
void get_samples() // this function talks to the amb sensor and ADC via I2C
{
  int32_t v;
  tcBase * tc;
  float tempF;
//  int32_t itemp;
  
  uint16_t dly = amb.getConvTime(); // use delay based on slowest conversion
  uint16_t dADC = adc.getConvTime();
  dly = dly > dADC ? dly : dADC;
 
  for( uint8_t jj = 0; jj < NC; jj++ ) { // one-shot conversions on both chips
    uint8_t k = actv[jj]; // map logical channels to physical ADC channels
    // jj = logical channel; k = physical channel
    if( k > 0 ) {
      --k;
      tc = tcp[k]; // each channel may have its own TC type
      adc.nextConversion( k ); // start ADC conversion on physical channel k
      amb.nextConversion(); // start ambient sensor conversion
      checkStatus( dly ); // give the chips time to perform the conversions
      amb.readSensor(); // retrieve value from ambient temp register
      v = adc.readuV(); // retrieve microvolt sample from MCP3424
      tempF = tc->Temp_F( 0.001 * v, amb.getAmbF() ); // convert uV to Celsius
      // filter on direct ADC readings, not computed temperatures
      v = fT[k].doFilter( v << 10 );  // multiply by 1024 to create some resolution for filter
      v >>= 10; 
      AT = amb.getAmbF();
      T[k] = tc->Temp_F( 0.001 * v, AT ); // convert uV to Fahrenheit;
    }
  }
};

#ifdef LCD
// --------------------------------------------
void updateLCD() {
  
 // AT
  int it01 = round( convertUnits( AT ) );
  if( it01 > 999 ) 
    it01 = 999;
  else
    if( it01 < -999 ) it01 = -999;
  sprintf( st1, "%4d", it01 );
  lcd.setCursor( 0, 0 );
  lcd.print("AMB:");
  lcd.print(st1);

  // display the first 2 active channels encountered, normally BT and ET
  uint8_t jj,j;
  uint8_t k;
  for( jj = 0, j = 0; jj < NC && j < 2; ++jj ) {
    k = actv[jj];
    if( k != 0 ) {
      ++j;
      it01 = round( convertUnits( T[k-1] ) );
      if( it01 > 999 ) 
        it01 = 999;
      else
        if( it01 < -999 ) it01 = -999;
      sprintf( st1, "%4d", it01 );
      if( j == 1 ) {
        lcd.setCursor( 9, 0 );
        lcd.print("T1:");
      }
      else {
        lcd.setCursor( 9, 1 );
        lcd.print( "T2:" );
      }
      lcd.print(st1);  
    }
  }
  if( millis() - lcd_count >= 100 ) { // display for 100 ms only
    lcd.setCursor( 0, 1 );
    lcd.print( "         " );
    lcd_count = 0;
  }
}
#endif



// ------------------------------------------------------------------------
// MAIN
//


void setup()
{
  delay(100);
  Wire.begin(); 
  Serial.begin(9600);
  Serial1.begin(9600 );
  pinMode( LEDPIN, OUTPUT );
  digitalWrite( LEDPIN, 0 );  // force on board LED pin off
  amb.init( AMB_FILTER );  // initialize ambient temp filtering

#ifdef LCD
  lcd.begin(16, 2);
  BACKLIGHT;
  lcd.setCursor( 0, 0 );
  lcd.print( BANNER_ARTISAN ); // display version banner
#endif // LCD

#ifdef MEMORY_CHK
  Serial.print("# freeMemory()=");
  Serial.println(freeMemory());
#endif

  adc.setCal( CAL_GAIN, UV_OFFSET );
  amb.setOffset( AMB_OFFSET );

// read calibration and identification data from eeprom
  if( readCalBlock( eeprom, caldata ) ) {
    #ifdef ACKS_ON
    Serial.println("# EEPROM data read: ");
    Serial.print("# ");
    Serial.print( caldata.PCB); Serial.print("  ");
    Serial.println( caldata.version );
    Serial.print("# ");
    Serial.print( caldata.cal_gain, 4 ); Serial.print("  ");
    Serial.println( caldata.K_offset, 2 );
    #endif
    adc.setCal( caldata.cal_gain, caldata.cal_offset );
    amb.setOffset( caldata.K_offset );
  }
  else { // if there was a problem with EEPROM read, then use default values
//    Serial.println("# Failed to read EEPROM.  Using default calibration data. ");
      
    adc.setCal( CAL_GAIN, UV_OFFSET );
    amb.setOffset( AMB_OFFSET );
  }   

  // initialize filters on all channels
  fT[0].init( ET_FILTER ); // digital filtering on ET
  fT[1].init( BT_FILTER ); // digital filtering on BT
  fT[2].init( ET_FILTER);
  fT[3].init( ET_FILTER);
  
  // set up output variables
  ssr.Setup( TIME_BASE );
  levelOT1 = levelOT2 = levelIO3 = 0;
  
  // initialize the active channels to default values
  actv[0] = 1;  // ET on TC1
  actv[1] = 2;  // BT on TC2
  actv[2] = 0; // default inactive
  actv[3] = 0;
  
  // assign thermocouple types
  tcp[0] = &tc1;
  tcp[1] = &tc2;
  tcp[2] = &tc3;
  tcp[3] = &tc4;

// add active commands to the linked list in the command interpreter object
  ci.addCommand( &dwriter );
  ci.addCommand( &awriter );
  ci.addCommand( &units );
  ci.addCommand( &chan );
  ci.addCommand( &io3 );
  ci.addCommand( &ot2 );
  ci.addCommand( &ot1 );
/*  
  ci.addCommand( &rf2000 );
  ci.addCommand( &rc2000 );
*/  
  ci.addCommand( &reader );
  ci.addCommand( &dcfan );
  ci.addCommand( &pid );
  ci.addCommand( &filt );

  dcfan.init(); // initialize conditions for dcfan
  
// initialize PID
  myPID.SetSampleTime(CT); // set sample time to default in user.h
  myPID.SetOutputLimits(MIN_OT1, MAX_OT1); // set output limits to user defined limits
  myPID.SetControllerDirection(DIRECT); // set PID to be direct acting mode. Increase in output leads to increase in input
  myPID.SetTunings(PRO, INT, DER); // set initial PID tuning values
  myPID.SetMode(MANUAL); // start with PID control off
  
#ifdef LCD
  delay( 500 );
  lcd.clear();
#endif
}

// -----------------------------------------------------------------
void loop()
{
  get_samples();
  #ifdef LCD
  updateLCD();
  #endif
  //checkSerial();  // Has a command been received?
  doPID(); // perform PID calculations if enabled
  //Serial.print("Hello");
}
