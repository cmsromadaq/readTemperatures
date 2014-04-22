/*
 * Example code for SHT1x or SHT7x sensors demonstrating blocking calls
 * for temperature and humidity measurement in the setup routine and
 * non-blocking calls in the main loop.  The pin 13 LED is flashed as a
 * background task while temperature and humidity measurements are made.
 * Note that the status register read/write demonstration code places the
 * sensor in low resolution mode.  Delete it to stay in high res mode.
 *
 * This example contains two versions of the code: one that checks library
 * function return codes for error indications and one that does not.
 * The version with error checking may be useful in debuging possible
 * connection issues with the sensor.  A #define selects between versions.
 */

#include <Sensirion.h>

const byte sht75_dataPin =  2;                 // SHTxx serial data
const byte sht75_sclkPin =  3;                 // SHTxx serial clock
const byte lm35_analogPin =  1;                // LM35 analog input
const byte ledPin  = 13;                 // Arduino built-in LED
const unsigned long TRHSTEP   = 10000UL;  // Sensor query period
const unsigned long BLINKSTEP =  250UL;  // LED blink period

Sensirion sht = Sensirion(sht75_dataPin, sht75_sclkPin);

unsigned int rawData;

float sens1_temperature;
float sens1_humidity;
float sens1_dewpoint;

float sens2_temperature;

byte ledState = 0;
byte measActive = false;
byte measType = TEMP;

unsigned long trhMillis = 0;             // Time interval tracking
unsigned long blinkMillis = 0;

// This version of the code checks return codes for errors
byte error = 0;

int i;

float readTemperature_LM35()
{
    int tempread=0;
    int tmp_t;
    //Oversampling technique according to http://www.atmel.com/dyn/resources/prod_documents/doc8003.pdf
    for(i = 0; i<=4; i++){
      delay(60);
      if (i==0)
        continue;
      tempread += analogRead(lm35_analogPin);
    }
    tempread = tempread>>2;
    sens2_temperature = 110 * (float)tempread / 1024; 
}

void setup() {
  byte stat;
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  delay(15);                             // Wait >= 11 ms before first cmd
// Demonstrate status register read/write
  if (error = sht.readSR(&stat))         // Read sensor status register
    logError(error);
  Serial.print("Status reg = 0x");
  Serial.println(stat, HEX);
//  if (error = sht.writeSR(LOW_RES))      // Set sensor to low resolution
//    logError(error);
//  if (error = sht.readSR(&stat))         // Read sensor status register again
//    logError(error);
//  Serial.print("Status reg = 0x");
//  Serial.println(stat, HEX);

//Setting analog reference to internal to improve the precision of LM35
  analogReference(INTERNAL);
}

void loop() {

  unsigned long curMillis = millis();          // Get current time

//  //Rapidly blink LED.  Blocking calls take too long to allow this.
//  if (curMillis - blinkMillis >= BLINKSTEP) {  // Time to toggle the LED state?
//    ledState = 1;
//    digitalWrite(ledPin, ledState);
//    blinkMillis = curMillis;
//  }

  // Demonstrate non-blocking calls
  if (curMillis - trhMillis >= TRHSTEP) {      // Time for new measurements?
    measActive = true;
    measType = TEMP;
    if (error = sht.meas(TEMP, &rawData, NONBLOCK)) // Start temp measurement
      logError(error);
    trhMillis = curMillis;
  }
  if (measActive && (error = sht.measRdy())) { // Check measurement status
    if (error != S_Meas_Rdy)
      logError(error);
    if (measType == TEMP) {                    // Process temp or humi?
      measType = HUMI;
      sens1_temperature = sht.calcTemp(rawData);     // Convert raw sensor data
      if (error = sht.meas(HUMI, &rawData, NONBLOCK)) // Start humi measurement
        logError(error);
    } else {
      measActive = false;
      sens1_humidity = sht.calcHumi(rawData, sens1_temperature); // Convert raw sensor data
      sens1_dewpoint = sht.calcDewpoint(sens1_humidity, sens1_temperature);
      logDataSensor1();
      readTemperature_LM35(); //oversampling
      logDataSensor2();
    }
  }
}

void logDataSensor1() {
  Serial.print("SENS1 T=");   Serial.print(sens1_temperature);
  Serial.print(" H=");  Serial.print(sens1_humidity);
  Serial.print(" D=");  Serial.println(sens1_dewpoint);
}

void logDataSensor2() {
  Serial.print("SENS2 T=");   Serial.println(sens2_temperature);
}

// The following code is only used with error checking enabled
void logError(byte error) {
  switch (error) {
  case S_Err_NoACK:
    Serial.println("Error: No response (ACK) received from sensor!");
    break;
  case S_Err_CRC:
    Serial.println("Error: CRC mismatch!");
    break;
  case S_Err_TO:
    Serial.println("Error: Measurement timeout!");
    break;
  default:
    Serial.println("Unknown error received!");
    break;
  }
}
