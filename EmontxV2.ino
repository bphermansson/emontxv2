
/*
  By Patrik Hermansson

  Sensor for Emoncms network. This device is mounted outdoor at the electricity meter pole.
  Sensors used:
  -BMP180, air pressure/temperature, I2C.
     Code uses Sparkfuns lib for BMP180, 
     https://learn.sparkfun.com/tutorials/bmp180-barometric-pressure-sensor-hookup-?_ga=1.148112447.906958391.1421739042
  -HTU21D, humidity/temperature, I2C. Mounted at a good(external) location.
     https://learn.sparkfun.com/tutorials/htu21d-humidity-sensor-hookup-guide/htu21d-overview
  -PT333, phototransistor reading the led on the electricity meter.
  -ML8511 UV index sensor.
  -
  -Voltage divider on A0 reading the solar cell voltage.
  -Battery voltage monitored internally. 
     http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/)
 

  Uses a Atmega328 bootloaded as a Arduino Mini Pro. Uses a 8 MHz crystal to 
  be able to run down to 2.4 volts. 
  Power supply is a LiFePo4 charged by a solar cell, parts taken from a garden solar
  cell lamp. 
  
  
  Based on "emonTX LowPower Temperature Example"
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3
 
  Authors: Glyn Hudson, Trystan Lea
  Builds upon JeeLabs RF12 library and Arduino

  THIS SKETCH REQUIRES:

  Libraries in the standard arduino libraries folder:
	- JeeLib		https://github.com/jcw/jeelib


   Reminder for Github:
   git commit EmontxV2.ino
   git push origin master
*/

#define RF_freq RF12_433MHZ   // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
const int nodeID = 23;        // emonTx RFM12B node ID - should be unique on network, see recomended node ID range below
const int networkGroup = 210; // emonTx RFM12B wireless network group - needs to be same as emonBase and emonGLCD

/*Recommended node ID allocation
------------------------------------------------------------------------------------------------------------
-ID-	-Node Type- 
0	- Special allocation in JeeLib RFM12 driver - reserved for OOK use
1-4     - Control nodes 
5-10	- Energy monitoring nodes
11-14	--Un-assigned --
15-16	- Base Station & logging nodes
17-30	- Environmental sensing nodes (temperature humidity etc.)
31	- Special allocation in JeeLib RFM12 driver - Node31 can communicate with nodes on any network group
-------------------------------------------------------------------------------------------------------------
*/
                                           
const int time_between_readings= 120000;                                  //120s in ms - FREQUENCY OF READINGS 
//const int time_between_readings= 20000;                                  //20s in ms - FREQUENCY OF READINGS 
#define RF69_COMPAT 0 // set to 1 to use RFM69CW 
#include <JeeLib.h>   // make sure V12 (latest) is used if using RFM69CW
#include <avr/sleep.h>
ISR(WDT_vect) { Sleepy::watchdogEvent(); }                              // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption 

// Solar cell monitor
int sensorPin = A0;    // select the input pin for the potentiometer
float r1 = 1000000; // Resistor between Vcc and A0
float r2 = 470000;  // Resistor between A0 and Gnd

// BMP180
#include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 pressure;
#define ALTITUDE 54.0 // Altitude of Såtenäs (my location) in meters

// HTU21D
#include "SparkFunHTU21D.h"
//Create an instance of the object
HTU21D htu21d;
// Wire.h is already included

// ML8511 
int mlOutPin = A1;
int mlEnPin = 5;

// Payload to send to base. Split in two to avoid max packet length limit.
typedef struct {
  	  int BMPtemp;
//  	  int HTUtemp;
  	  unsigned int pressure;
  	  int humidity;
  	  int battery;     
      int solarvolt;   
      float uvIntensity;                                      
} Payload;
Payload emontx;
/*
typedef struct {             
      int battery;  
      
} Payload2;
Payload2 emontx2;
*/
void setup() {
  Serial.begin(57600);
  Serial.println("PH emonTX v2"); 
  Serial.println("OpenEnergyMonitor.org");
  Serial.print("Node: "); 
  Serial.print(nodeID); 
  Serial.print(" Freq: "); 
  if (RF_freq == RF12_433MHZ) Serial.print("433Mhz");
  if (RF_freq == RF12_868MHZ) Serial.print("868Mhz");
  if (RF_freq == RF12_915MHZ) Serial.print("915Mhz"); 
  Serial.print(" Network: "); 
  Serial.println(networkGroup);

  rf12_initialize(nodeID, RF_freq, networkGroup);                          // initialize RFM12B
  rf12_control(0xC040);                                                 // set low-battery level to 2.2V i.s.o. 3.1V
  delay(10);
  rf12_sleep(RF12_SLEEP);

  // ML8511
  pinMode(mlOutPin, INPUT);
  pinMode(mlEnPin, OUTPUT);

  // Is battery voltage low?
  int lobat = rf12_lowbat();
  Serial.print ("Battery low ");
  Serial.println(lobat); 

  // Initialize the BMP180 (it is important to get calibration values stored on the device).
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.

    // Init humidity sensor
    htu21d.begin();
  }
  Serial.println ("Setup done");
}

void loop()
{ 
  // For BMP180
  char status;
  double T,P,p0,a;
  int iBMPtemp, iBMPpres;
  
  // Read battery voltage
  long batt = readVcc();
  Serial.print ("Battery: ");
  Serial.print (batt);
  Serial.println(" volt.");
  
  // Read solar cell voltage
  // Read A0 and calculate value
  float v = (analogRead(sensorPin) * batt) / 1024.0;
  int solarvolt = v / (r2 / (r1 + r2));

  Serial.print ("Solar cell = ");
  Serial.print (solarvolt);
  Serial.println (" volt.");


  // UV-value
  // ML8511 
  // int mlOutPin = A1;
  // int mlEnPin = 5;
  digitalWrite(mlEnPin, HIGH);   // Enable sensor
  delay(100);
  float uv = (analogRead(mlOutPin) * batt) / 1024.0;
  int uv_raw = analogRead(mlOutPin);
  digitalWrite(mlEnPin, LOW);   // Disable sensor

  // Inspired by https://github.com/sparkfun/ML8511_Breakout/blob/master/firmware/MP8511_Read_Example/MP8511_Read_Example.ino
  float outputVoltage = 3.3 / batt * uv_raw;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
  Serial.print("MP8511 output: ");
  Serial.println(uv_raw);

  Serial.print(" MP8511 voltage: ");
  Serial.println(outputVoltage);

  Serial.print(" UV Intensity (mW/cm^2): ");
  Serial.println(uvIntensity);
  Serial.print("UV raw (ML8511 output voltage): ");
  Serial.println(uv);

  // Air pressure, you must read the BMP180:s temp too.
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("BMP180 temperature: ");
      Serial.print(T,2);
      Serial.println(" deg C, ");
      // Convert to int
      double T2=T*100; // Preserve decimals
      //iBMPtemp = (int) T2;
      //Serial.print("BMP180 temp as int:");
      //Serial.println(iBMPtemp);
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          //Serial.print("absolute pressure: ");
          //Serial.print(P,2);
          //Serial.println(" mb/hPa");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0,2);
          Serial.println(" mb/hPa");

          // Convert to int
          //iBMPpres = (int) p0;
          //Serial.print("Pressure as int:");
          //Serial.println(iBMPpres);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

  // Read humidity sensor
  float humd = htu21d.readHumidity();
  int ihumd = (int) (humd * 10);  // Preserve one decimal by multiplication
  float htutemp = htu21d.readTemperature();
  int ihtutemp = (int) (htutemp * 100);

  /*
  Serial.print("Humidity:");
  Serial.println(humd);
  Serial.print(ihumd);
  Serial.println(" (as int)");
  
  Serial.print("HTD Temp :");
  Serial.println(htutemp);
  Serial.print(ihtutemp);
  Serial.println(" (as int)");
  */
  // Add values to emontx payload
  emontx.BMPtemp= (int) (T*10);      // Temp from the BMP180
  //emontx.HTUtemp=ihtutemp;   // Temp from the HTU21D
  emontx.pressure=int (p0);
  emontx.humidity=int (htutemp); 
  emontx.battery = int (batt);
  emontx.solarvolt = int (solarvolt);
  emontx.uvIntensity = int (uvIntensity*100);

/*
  Serial.print("emontx.BMPtemp: ");
  Serial.println(emontx.BMPtemp);
  Serial.print("emontx.pressure: ");
  Serial.println(emontx.pressure);
*/
  
  delay(10);
  
  // Send to emonbase    
  rf12_sleep(RF12_WAKEUP);
  // if ready to send + exit loop if it gets stuck as it seems too
  int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
  rf12_sendStart(0, &emontx, sizeof emontx);
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
  rf12_sendWait(2);
  rf12_sleep(RF12_SLEEP); 
  delay(50);  
  
  // Payload 2
//  i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
//  rf12_sendStart(0, &emontx2, sizeof emontx2);
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
//  rf12_sendWait(2);
//  rf12_sleep(RF12_SLEEP); 
//  delay(50);  
  
  Sleepy::loseSomeTime(time_between_readings);
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

/* Calibration
DWM=3.30V
Readvcc=3.242
internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
1.1*3.300*3.242 = 11.76846
scale_constant = internal1.1Ref * 1023 * 1000 
11.76846 * 1023 * 1000 = 12039134 ? 
Use calibration value determined by trial and error instead.

With solar call:
Vcc read 3286
DVM 3.27
*/ 
  long result = (high<<8) | low;
  result = 1147000L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
