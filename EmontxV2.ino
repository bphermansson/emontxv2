/*
  By Patrik Hermansson

  Sensor for Emoncms network. This device is mounted outdoor at the electricity meter pole.
  Sensors used:
  -BMP180, air pressure, I2C.
  -HTU21D, humidity/temperature, I2C.
  -PT333, phototransistor reading the led on the electricity meter.
  -Voltage divider on A0 reading the solar cell voltage.
  -Battery voltage monitored internally. (http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/)
  

  Uses a Atmega328 bootloaded as a Arduino Mini Pro. Uses a 8 MHz crystal to 
  be able to run down to 2.4 volts. 
  Power supply is a LiFePo4 charged by a solar cell, parts taken from a garden solar
  cell lamp. 
  , http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  
  
  Based on "emonTX LowPower Temperature Example"
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3
 
  Authors: Glyn Hudson, Trystan Lea
  Builds upon JeeLabs RF12 library and Arduino

  THIS SKETCH REQUIRES:

  Libraries in the standard arduino libraries folder:
	- JeeLib		https://github.com/jcw/jeelib
        - DHT22 Humidity        https://github.com/adafruit/DHT-sensor-library - be sure to rename the sketch folder to remove the '-'


 
*/

#define RF_freq RF12_433MHZ                                                // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
const int nodeID = 23;                                                  // emonTx RFM12B node ID - should be unique on network, see recomended node ID range below
const int networkGroup = 210;                                           // emonTx RFM12B wireless network group - needs to be same as emonBase and emonGLCD

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
                                           
//const int time_between_readings= 60000;                                  //60s in ms - FREQUENCY OF READINGS 
const int time_between_readings= 3000;                                  //60s in ms - FREQUENCY OF READINGS 
#define RF69_COMPAT 0 // set to 1 to use RFM69CW 
#include <JeeLib.h>   // make sure V12 (latest) is used if using RFM69CW
#include <avr/sleep.h>
ISR(WDT_vect) { Sleepy::watchdogEvent(); }                              // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption 

// Solar cell monitor
int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
float volt;

typedef struct {
  	  int temp;
      int humidity;                                  
	    int battery;		                                      
} Payload;
Payload emontx;

int oldtemp, oldhumidity, oldbattery;
boolean firstrun = true;

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
}

void loop()
{ 
  // Read battery voltage
  long batt = readVcc();
  Serial.print ("readVcc: ");
  Serial.print (batt);
  Serial.println("V");
  
  // Read solar cell voltage
  sensorValue = analogRead(sensorPin);
  delay(10);
  volt = sensorValue * (3.32 / 1023.0);

  // Raw values
  Serial.print ("Solar cell = ");
  Serial.print (volt);
  Serial.print (" V, AD = ");
  Serial.println (sensorValue);
  
  emontx.battery = (int) (volt*100);
  
  delay(10);
  
  // Print results on serial port...
  Serial.print("Battery = ");
  Serial.println(emontx.battery );
  delay(5);

  
   // check if returns are valid, if they are NaN (not a number) then something went wrong!
  //if (isnan(emontx.temp) || isnan(emontx.humidity)) {
  //  Serial.println("Failed to read from DHT");}
  //  else
  //  {
      
      rf12_sleep(RF12_WAKEUP);
      // if ready to send + exit loop if it gets stuck as it seems too
      int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
      rf12_sendStart(0, &emontx, sizeof emontx);
      // set the sync mode to 2 if the fuses are still the Arduino default
      // mode 3 (full powerdown) can only be used with 258 CK startup fuses
      rf12_sendWait(2);
      rf12_sleep(RF12_SLEEP); 
      delay(50);
   // }
    
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
*/ 
  long result = (high<<8) | low;
  result = 1147000L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
