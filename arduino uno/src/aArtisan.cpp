#include<Arduino.h>
#include"thermocouple.h"
#include <cmndproc.h>
#include <SoftwareSerial.h>

SoftwareSerial BT(6, 5); // recieve-black, send-gray
char val;  // 


// *** BSD License ***

// ------------------------------------------------------------------------------------------

#define BANNER_ARTISAN "aARTISAN V3.10"


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

    BT.print("# freeMemory()=");
    BT.println(freeMemory());
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
    Input = convertUnits( T[k] );
    myPID.Compute();  // do PID calcs
    levelOT1 = Output; // update OT1 based on PID output
    #ifdef ACKS_ON
    Serial.print("# PID input = " ); Serial.print( Input ); Serial.print( "  ");
    BT.print("# PID input = " ); BT.print( Input ); BT.print( "  ");  
    
    Serial.print("# PID output = " ); Serial.println( levelOT1 );
    BT.print("# PID input = " ); BT.print( Input ); BT.print( "  ");
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
  BT.print("this is logger");
  /*AT為攝氏or華氏*/
  BT.print(convertUnits( AT ), DP );
  BT.print("this is temp\n");
  
// print active channels
  for( uint8_t jj = 0; jj < NC; ++jj ) {
    uint8_t k = actv[jj];
    // jj = logical channel, k = physical channel
    if( k > 0 ) {
      --k;
      Serial.print(",");
      BT.print(",");
      Serial.print( convertUnits( T[k] ), DP );
      /*T[k]的數據跟前面的溫度轉換一樣*/
      BT.print( convertUnits( T[k] ), DP );
      //BT.print("line 239");
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
    //==========
    BT.print(",");
    BT.print( HEATER_DUTY ); 
    BT.print(",");
    BT.print( FAN_DUTY );
    BT.print(",");
    BT.print( SV );
    BT.print("line 257");
  }  
  Serial.println();
  BT.println();
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
  BT.begin(9600);
  
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
  BT.print("# freeMemory()=");
  Serial.println(freeMemory());
  BT.print("# freeMemory()=");
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

    //=======================
    BT.println("# EEPROM data read: ");
    BT.print("# ");
    BT.print( caldata.PCB); BT.print("  ");
    BT.println( caldata.version );
    BT.print("# ");
    BT.print( caldata.cal_gain, 4 ); BT.print("  ");
    BT.println( caldata.K_offset, 2 );
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
}
