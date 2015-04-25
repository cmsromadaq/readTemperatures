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



const byte ledPin  = 13;                 // Arduino built-in LED

const unsigned long TRHSTEP   = 10000UL;  // Sensor query period
const unsigned long BLINKSTEP =  250UL;  // LED blink period

const byte sht75_dataPin =  3;                 // SHTxx serial data
const byte sht75_sclkPin =  2;                 // SHTxx serial clock
Sensirion sht = Sensirion(sht75_dataPin, sht75_sclkPin);
unsigned int sht75_rawData;
float sht75_temperature;
float sht75_humidity;
float sht75_dewpoint;

#define LM35_SENSORS 2
float lm35_temperature[LM35_SENSORS];
const byte lm35_analogPin[LM35_SENSORS] = {1,2}; //anlogPins
const byte lm35_pwmNoisePin[LM35_SENSORS] = {10,11}; //noisePins
const byte lm35_pwmNoise_duty= 127; //LM35 noise PWM 50% Duty square generator
const byte lm35_oversampling_averages=32;

byte ledState = 0;
byte measActive = false;
byte measType = TEMP;

unsigned long trhMillis = 0;             // Time interval tracking
unsigned long blinkMillis = 0;

// This version of the code checks return codes for errors
byte error = 0;

int i;

//#define DEBUG

void readTemperature_LM35(byte lm35sensor)
{
    int tempread=0;
    int tmp_t;
    float temperature;
    //Oversampling technique according to http://www.atmel.com/dyn/resources/prod_documents/doc8003.pdf
    for(i = 0; i<=lm35_oversampling_averages; i++){
      delay(60);
      if (i==0)
        continue;
      tmp_t = analogRead(lm35_analogPin[lm35sensor]);
#ifdef DEBUG
      Serial.print("sensor "); Serial.print(lm35sensor); Serial.print(" i= "); Serial.print(i);
      Serial.print(" adc= "); Serial.println(tmp_t);
#endif
      tempread += tmp_t;
    }
    //tempread = tempread>>4;
    temperature = 110 * (float)tempread / 1024. / (float) lm35_oversampling_averages; 
    lm35_temperature[lm35sensor]=temperature;
#ifdef DEBUG
    Serial.print("===> temp LM35 "); Serial.print(lm35sensor); Serial.print("= "); Serial.println(temperature);
#endif
    return; 
}
 
void setup() {
  byte stat;
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  for (byte i=0;i<LM35_SENSORS;++i)
      pinMode(lm35_pwmNoisePin[i], OUTPUT);

  delay(15);                             // Wait >= 11 ms before first cmd
// Demonstrate status register read/write
  if (error = sht.readSR(&stat))         // Read sensor status register
    logError(error);
#
  Serial.print("SHT75 Init Status = 0x");
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

  for (byte i=0;i<LM35_SENSORS;++i)
      analogWrite(lm35_pwmNoisePin[i],lm35_pwmNoise_duty);
  
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
    if (error = sht.meas(TEMP, &sht75_rawData, NONBLOCK)) // Start temp measurement
      logError(error);
    trhMillis = curMillis;
  }
  if (measActive && (error = sht.measRdy())) { // Check measurement status
    if (error != S_Meas_Rdy)
      logError(error);
    if (measType == TEMP) {                    // Process temp or humi?
      measType = HUMI;
      sht75_temperature = sht.calcTemp(sht75_rawData);     // Convert raw sensor data
      if (error = sht.meas(HUMI, &sht75_rawData, NONBLOCK)) // Start humi measurement
        logError(error);
    } else {
      measActive = false;
      sht75_humidity = sht.calcHumi(sht75_rawData, sht75_temperature); // Convert raw sensor data
      sht75_dewpoint = sht.calcDewpoint(sht75_humidity, sht75_temperature);
      for (byte i=0;i<LM35_SENSORS;++i)
	  readTemperature_LM35(i); //oversampling
      logData();
    }
  }
}

void logData() {
  Serial.print("T1=");   Serial.print(sht75_temperature);
  for (byte i=0;i<LM35_SENSORS;++i)
    {  
  Serial.print(" T");Serial.print(i+2); Serial.print("=");   Serial.print(lm35_temperature[i]);
}
  Serial.print(" H=");  Serial.print(sht75_humidity);
  Serial.print(" D=");  Serial.println(sht75_dewpoint);
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
