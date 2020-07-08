/*---------------------------------------------------------------------------------

  MAX30101/MAX30102/MAXM86161 for oximeter

  Optical SP02 Detection (SPK Algorithm)
  
  This demo shows heart rate and SPO2 levels.

  Modified by: ZWang

Integrated LEDS+ optical photodetector module for HR and SpO2 measurement

MAX30101 and MAX30102 are similarly, only LEDs different.
The MAX30101 - 1.8V power supply, and 5.0V for two internal LEDs. 
The MAX30102 - 1.8V power supply, and 3.3V for three internal LEDs. 

MAXM86161 is single 3.0~5.5V power supply and shrink optical size by 40 % 
(OLGA package, 2.9mm x 4.3mm x 1.4 mm). 
three LEDs — red and infrared for SpO2 measurement and green for heart rate. 
The MAXM86161 is $4.41 (1000 pcs); The evaluation kit is $150.
---------------------------------------------------------------------------------*/

/*
  MAX3010X Breakout: Output all the raw Red/IR/Green readings, check INT pin and interrupt register
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  Outputs all Red/IR/Green values as fast as possible
  Checks the interrupt pin to see if an interrupt occurred
  Checks the interrupt register to see if a bit was set

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
 
  The MAX3010X Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/
#include "Arduino.h"
#include "firmware.h"
#include <Wire.h>
#include "spo2_max3010x.h"

extern void maxim_heart_rate_and_oxygen_saturation( \
  uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, \
  int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid);

MAX3010X spo2Sensor;
/*---------------------------------------------------------------------------------
 code for product 
---------------------------------------------------------------------------------*/
#define MAX_BRIGHTNESS 255

static uint32_t irBuffer [100]; //infrared LED sensor data
static uint32_t redBuffer[100];//red LED sensor data

//buffer length of 100 stores 4 seconds of samples running at 25sps
const int32_t bufferLength = 100;   //data length


int8_t  validSPO2;      //indicator to show if the SPO2 calculation is valid
int32_t heartRate;      //heart rate value
int8_t  validHeartRate; //indicator to show if the heart rate calculation is valid

void initMax3010xSpo2()
{
  int32_t spo2Value;      //SPO2 value

  // Initialize sensor
  if (!spo2Sensor.begin(Wire, MAX3010X_I2C_ADDRESS_W, MAX3010X_I2C_ADDRESS_R)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX3010X was not found. Please check wiring/power."));
  }

  byte ledBrightness = 60;  //Options: 0=Off to 255=50mA
  byte sampleAverage = 4;   //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;         //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
                            //FIXME MAX30101 = 3 LEDs
  byte sampleRate = 100;    //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth  = 411;    //Options: 69, 118, 215, 411
  int adcRange    = 4096;   //Options: 2048, 4096, 8192, 16384

  spo2Sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings


  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (spo2Sensor.available() == false) //do we have new data?
      spo2Sensor.check(); //Check the sensor for new data

    redBuffer[i] = spo2Sensor.getRed();
    irBuffer[i]  = spo2Sensor.getIR();
    spo2Sensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2Value, &validSPO2, &heartRate, &validHeartRate);
}

void handleMax3010xSpo2()
{
  int32_t spo2Value;      //SPO2 value

  if (spo2.interrupt_flag == false) 
    return;   // continue wait for data ready pin interrupt
  else   
    spo2.clear_interrupt();
  // interrupt captured, process the data

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 25; i < 100; i++)
  {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer [i - 25] = irBuffer [i];
  }

  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 75; i < 100; i++)
  {
    while (spo2Sensor.available() == false) //do we have new data?
      spo2Sensor.check(); //Check the sensor for new data

    redBuffer[i] = spo2Sensor.getRed();
    irBuffer [i] = spo2Sensor.getIR();
    spo2Sensor.nextSample(); //We're finished with this sample so move to next sample

    //send samples and calculation result to terminal program through UART
    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.print(irBuffer[i], DEC);

    Serial.print(F(", HR="));
    Serial.print(heartRate, DEC);

    Serial.print(F(", HRvalid="));
    Serial.print(validHeartRate, DEC);

    Serial.print(F(", SPO2="));
    Serial.print(spo2Value, DEC);

    Serial.print(F(", SPO2Valid="));
    Serial.println(validSPO2, DEC);

    //FIXME: spo2.save_to_ppg_buffer implementation here
  }

  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2Value, &validSPO2, &heartRate, &validHeartRate);
 
  if (validSPO2)
  {
    SpO2Level = (uint8_t) spo2Value;
    SpO2Ready = true;
  }  
  
}
/*---------------------------------------------------------------------------------
 Take IR reading to sense whether the human body presence
---------------------------------------------------------------------------------*/ 

// use this number to define HUMAN_BODY_PRESENT_IR_THRESHOLD in firmware.h
void read_unblocked_IR_value()
{ 
  long unblocked_IR_value = 0; //Average IR without attached to human body
  
  //spo2Sensor.setPulseAmplitudeRed(0);   //Turn off Red LED
  //spo2Sensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  //Take an average of IR readings at power up
  for (byte x = 0 ; x < 32 ; x++)
  {
    unblocked_IR_value += spo2Sensor.getIR(); //Read the IR value
  }
  unblocked_IR_value /= 32;

  Serial.print("IR value without human body: ");
  Serial.println(unblocked_IR_value);
}

boolean if_spo2_sensor_on_body()
{
  if (spo2Sensor.getIR()  > HUMAN_BODY_PRESENT_IR_THRESHOLD)
  {
    Serial.print(" Something is there!");
    return true;
  }
  else
    return false;
}
/*---------------------------------------------------------------------------------
  TEST CODE 

  you may use Arduino's serial plotter to display it.
  just call Serial.println(spo2Sensor.getIR()); //Send raw data to plotter
---------------------------------------------------------------------------------*/
#if 1  
//FIXME the codes below must be removed in the future
extern bool checkForBeat(int32_t sample);

void TEST_SPO2()
{
  const byte RATE_SIZE = 4;        //Increase this for more averaging. 4 is good.
  static byte rates[RATE_SIZE];    //Array of heart rates
  static byte rateSpot = 0;
  static long lastBeat = 0;        //Time at which the last beat occurred
  float beatsPerMinute;
  int   beatAvg;

  spo2Sensor.enableAFULL();         //Enable the almost full interrupt (default is 32 samples)
  spo2Sensor.setFIFOAlmostFull(3);  //Set almost full int to fire at 29 samples

  spo2Sensor.setPulseAmplitudeRed(0x0A);  //Turn Red LED to low to indicate sensor is running
  spo2Sensor.setPulseAmplitudeGreen(0);   //Turn off Green LED

  long irValue = spo2Sensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  Serial.println();

  spo2Sensor.check();             //Check the sensor, read up to 3 samples

  if (spo2Sensor.available())     //do we have new data?
  {
    Serial.print(" R:");
    Serial.print(spo2Sensor.getRed());
    Serial.print(" IR:");
    Serial.print(spo2Sensor.getIR());
    Serial.print(" G:");
    Serial.print(spo2Sensor.getGreen());

    if (digitalRead(OXIMETER_INT_PIN) == LOW) //Hardware way of reading interrupts
    {
      Serial.print(" INT!");
    }

    byte flags = spo2Sensor.getINT1(); //Software way of reading interrupts
    if (flags)
    {
      Serial.print(" I[");
      Serial.print(flags, BIN);
      Serial.print("]");
    }

    Serial.println();

    spo2Sensor.nextSample(); //We're finished with this sample so move to next sample
  }

  #if 1 //temperature sensor
  //FIXME need test LED heat

  /*
  onboard temperature sensor
  
  The accuracy is +/-1 C, but the real precision of 0.0625 C.
  
  The LEDs are very low power and won't affect the temp reading much but
  you may want to turn off the LEDs to avoid any local heating
  */
  spo2Sensor.setup(0); //Configure sensor. Turn off LEDs
  spo2Sensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.

  float temperature = spo2Sensor.readTemperature();

  Serial.print("temperatureC=");
  Serial.print(temperature, 4);
  #endif ////temperature sensor
}  
#endif