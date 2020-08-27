/*---------------------------------------------------------------------------------

  MAX30101/MAX30102/MAXM86161 for oximeter

  Optical SP02 Detection (SPK Algorithm)
  
  This demo shows heart rate and SPO2 levels.

  The code below has been modified by ZWang, and much different from the original one.

Integrated LEDS+ optical photodetector module for HR and SpO2 measurement

MAX30101 and MAX30102 are similarly, only LEDs different.
The MAX30101 - 1.8V power supply, and 5.0V for two internal LEDs. 
The MAX30102 - 1.8V power supply, and 3.3V for three internal LEDs. 

MAXM86161 is single 3.0~5.5V power supply and shrink optical size by 40 % 
(OLGA package, 2.9mm x 4.3mm x 1.4 mm). 
three LEDs — red and infrared for SpO2 measurement and green for heart rate. 
The MAXM86161 is $4.41 (1000 pcs); The evaluation kit is $150.

EVB problem:

MAX30102 board ordered from China have two faulty problems:

1. I2C's SDA and SCL are connected with 1.8V, not 3.3V, might cause communication unstable.
https://reedpaper.wordpress.com/2018/08/22/pulse-oximeter-max30100-max30102-how-to-fix-wrong-board/

2. Some cloned sensors may have the RED and IR channels swapped; 
https://github.com/aromring/MAX30102_by_RF/issues/13 

3. New detection method and code (not used yet)
https://www.instructables.com/id/Pulse-Oximeter-With-Much-Improved-Precision/
https://github.com/aromring/MAX30102_by_RF
---------------------------------------------------------------------------------*/

/*
  MAX3010X Breakout: Output all the raw Red/IR/Green readings, check INT pin and interrupt register
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016

  https://www.sparkfun.com/products/14045
  https://github.com/sparkfun/MAX30105_Breakout

  MAX30101 breakout board
  https://www.sparkfun.com/products/16474
  https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library   

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
#include "firmware.h"
#include <Wire.h>
#include "spo2_max3010x.h"

void maxim_heart_rate_and_oxygen_saturation(
    uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer,
    float *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid);

MAX3010X spo2Sensor;

uint32_t unblocked_IR_value; //Average IR without attached to human body
// threshold of detect human body = read_unblocked_IR_value() + tolerance range
#define HUMAN_BODY_PRESENT_IR_THRESHOLD   1000

/*---------------------------------------------------------------------------------
 DC offset filter (high pass)
 use EMA Exponential Moving Average to remove DC signal from the samples.
 
 Reference:
 https://www.norwegiancreations.com/2015/10/tutorial-potentiometers-with-arduino-and-filtering/
 https://www.norwegiancreations.com/2016/03/arduino-tutorial-simple-high-pass-band-pass-and-band-stop-filtering/ 

---------------------------------------------------------------------------------*/
int32_t EMA_algorithm_Red(int32_t sensorValue)
{
  // sensorValue equivalent to EMA Y
  static float   EMA_a = 0.2;     //initialization of EMA alpha
  static int32_t EMA_S = 0;        //initialization of EMA S
  uint32_t highpass = 0;
 
  EMA_S = (EMA_a * sensorValue) + ((1-EMA_a)*EMA_S);  //run the EMA
  highpass = sensorValue - EMA_S;                   //calculate the high-pass signal
  return highpass;
}

int32_t EMA_algorithm_InfraRed(int32_t sensorValue)
{
  // sensorValue equivalent to EMA Y
  static float   EMA_a = 0.2;     //initialization of EMA alpha
  static int32_t EMA_S = 0;        //initialization of EMA S
  uint32_t highpass = 0;
 
  EMA_S = (EMA_a * sensorValue) + ((1-EMA_a)*EMA_S);  //run the EMA
  highpass = sensorValue - EMA_S;                   //calculate the high-pass signal
  return highpass;
}
void initMax3010xSpo2()
{
  // Initialize sensor
  if (!spo2Sensor.begin(Wire, MAX3010X_I2C_ADDRESS_W, MAX3010X_I2C_ADDRESS_R)) //Use default I2C port, 400kHz speed
  {
    Serial.println("!! SPO2 sensor MAX3010X is not found.");
    system_init_error++;  
  }

  byte ledBrightness = 60;  //Options: 0=Off to 255=50mA
  byte sampleAverage = 4;   //Options: 1, 2, 4, 8, 16, 32

  //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  #if     (SPO2_TYPE==OXI_MAX30102)  // with 2 color LEDs
    byte ledMode = 2;         
  #elif if(SPO2_TYPE==OXI_MAX30101)  // with 3 color LEDs
    byte ledMode = 3;     
    #error double check this code                      
  #elif if(SPO2_TYPE==OXI_MAXM86161) // with 3 color LEDs
    byte ledMode = 3;                 
    #error double check this code     
  #endif           

  // sampleRate, FreqS, sampleAverage are related, do not change them.
  byte sampleRate = 100;    //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth  = 411;    //Options: 69, 118, 215, 411
  int adcRange    = 4096;  //Options: 2048, 4096, 8192, 16384

  //Configure sensor with these settings
  spo2Sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); 

  spo2Sensor.clearFIFO();

  //Take an average of IR readings at power up
  for (int x = 0 ; x < 32 ; x++)
    unblocked_IR_value += spo2Sensor.getIR(); //Read the IR value
  unblocked_IR_value = unblocked_IR_value/32 + HUMAN_BODY_PRESENT_IR_THRESHOLD;
  Serial.print("IR value without human body: ");
  Serial.println(unblocked_IR_value);

  //maxim_max30102_write_reg(REG_INTR_ENABLE_1,0xc0); // INTR setting
  //maxim_max30102_write_reg(REG_INTR_ENABLE_2,0x00);
  //maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  //uint8_t uch_dummy;
  //pinMode(OXIMETER_INT_PIN, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102

}
/*---------------------------------------------------------------------------------
 IC internal FIFO size is 32 samples, make sure the sample rate, average, 
 and .check() is called frequently to meet that buffer not overflow.
---------------------------------------------------------------------------------*/
void handleMax3010xSpo2()
{
  static int buffer_index = 0;

  #define SPO2_BUFFER_SIZE  100
  static uint32_t irBuffer [SPO2_BUFFER_SIZE]; //infrared LED samples
  static uint32_t redBuffer[SPO2_BUFFER_SIZE]; //red LED samples

  float   spo2_value;
  int8_t  spo2_valid;       // = 1 when the SPO2 calculation is valid
  int32_t heart_rate;     
  int8_t  heart_rate_valid; // = 1 when the heart rate calculation is valid

  spo2Sensor.check(); //Check the sensor, read up to 3 samples
  if (spo2Sensor.available() < SPO2_BUFFER_SIZE) 
    return;

  digitalWrite (LED_PIN, HIGH);  // LED on  

  for (int i = 0; i < SPO2_BUFFER_SIZE; i++)
  {
    //FIXME red and infrared LED data swapped.
    redBuffer[buffer_index] = spo2Sensor.getFIFOIR();
    irBuffer [buffer_index] = spo2Sensor.getFIFORed();

    spo2Sensor.nextSample(); //We're finished with this sample so move to next sample

    /*------------------------------------------------------
    Take IR reading to sense whether the human body presence
    -------------------------------------------------------*/ 
    //if (irBuffer [buffer_index] > unblocked_IR_value)
    //Serial.printf("%d %d\r\n", 
    //              -EMA_algorithm_Red((int32_t)redBuffer[buffer_index]), 
    //              -EMA_algorithm_InfraRed((int32_t)irBuffer[buffer_index]));
    //Serial.printf("%u\r\n", ((int32_t)redBuffer[buffer_index]));

    buffer_index++;
    if (buffer_index>=SPO2_BUFFER_SIZE)
        buffer_index = 0;
  }
  
  maxim_heart_rate_and_oxygen_saturation(irBuffer, SPO2_BUFFER_SIZE, redBuffer, 
                                        &spo2_value, &spo2_valid, 
                                        &heart_rate, &heart_rate_valid);

  //while(digitalRead(OXIMETER_INT_PIN)==1);  //wait until the interrupt pin asserts
  //FIXME: spo2.save_to_ppg_buffer implementation here
 
  //if ((heart_rate_valid==1)&&(spo2_valid==1))
  
  Serial.printf("HR %d SPO2 %f\r\n", 
                heart_rate, //heart_rate_valid,
                spo2_value //spo2_valid,
                );
                
  if (spo2_valid)
  {
    SpO2Level = (uint8_t) spo2_value;
    SpO2Ready = true;
  }  
  digitalWrite (LED_PIN, LOW);  // LED off
}
/*---------------------------------------------------------------------------------
---------------------------------------------------------------------------------*/
#if 0 //temperature sensor
  //FIXME need test LED heat

  /*
  onboard temperature sensor
  
  The accuracy is +/-1 C, but the real precision of 0.0625 C.
  
  The LEDs are very low power and won't affect the temp reading much but
  you may want to turn off the LEDs to avoid any local heating
  */
  spo2Sensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.

  float temperature = spo2Sensor.readTemperature();
  Serial.print("temperatureC=");
  Serial.print(temperature, 4);
#endif ////temperature sensor