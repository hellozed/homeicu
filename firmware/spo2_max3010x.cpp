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

  interrupt feature is not used in this code
  //maxim_max30102_write_reg(REG_INTR_ENABLE_1,0xc0); // INTR setting
  //maxim_max30102_write_reg(REG_INTR_ENABLE_2,0x00);
  //maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  //pinMode(OXIMETER_INT_PIN, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102
  //while(digitalRead(OXIMETER_INT_PIN)==1);  //wait until the interrupt pin asserts

*/
#include "firmware.h"
#include <Wire.h>
#include "spo2_max3010x.h"
#include "cppQueue.h"
extern  Queue ppg_queue;

MAX3010X spo2Sensor;

// average IR without attached to human body		
// threshold of detect human body = read_unblocked_IR_value() + tolerance range		
uint32_t unblocked_IR_value; 
#define HUMAN_BODY_PRESENT_IR_THRESHOLD   2000

/*---------------------------------------------------------------------------------
 DC offset filter (high pass)
 use EMA Exponential Moving Average to remove DC signal from the samples.
 
 Reference:
 https://www.norwegiancreations.com/2015/10/tutorial-potentiometers-with-arduino-and-filtering/
 https://www.norwegiancreations.com/2016/03/arduino-tutorial-simple-high-pass-band-pass-and-band-stop-filtering/ 

---------------------------------------------------------------------------------*/
class EMA_algorithm{
  private:
    const float  EMA_a = 0.2;     //initialization of EMA alpha
    uint32_t EMA_S;
  
  public:
    EMA_algorithm(void)           //constrator
    {
      static uint32_t EMA_S = 0;   //initialization of EMA S 
    }  
  
    int32_t high_pass_filter(uint32_t data_input)
    {
      // data_input equivalent to EMA Y
      EMA_S = (EMA_a * data_input) + ((1-EMA_a)*EMA_S);  //run the EMA
      return data_input - EMA_S;
    }
};
EMA_algorithm EMA_ppg;
/*---------------------------------------------------------------------------------
 init the spo2 sensor
---------------------------------------------------------------------------------*/
void initMax3010xSpo2()
{
  // Initialize sensor
  if (!spo2Sensor.begin(Wire, MAX3010X_I2C_ADDRESS_W, MAX3010X_I2C_ADDRESS_R)) //Use default I2C port, 400kHz speed
  {
    Serial.println("!! SPO2 sensor MAX3010X is not found.");
    system_init_error++;  
  }

  uint8_t ledBrightness = 60;  //Options: 0=Off to 255=50mA
  uint8_t sampleAverage = 4;   //Options: 1, 2, 4, 8, 16, 32

  //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  #if     (SPO2_TYPE==OXI_MAX30102)  // with 2 color LEDs
    uint8_t ledMode = 2;         
  #elif if(SPO2_TYPE==OXI_MAX30101)  // with 3 color LEDs
    uint8_t ledMode = 3;     
    #error double check this code                      
  #elif if(SPO2_TYPE==OXI_MAXM86161) // with 3 color LEDs
    uint8_t ledMode = 3;                 
    #error double check this code     
  #endif           

  // sampleRate, FreqS, sampleAverage are related, do not change them.
  uint8_t sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth  = 411;    //Options: 69, 118, 215, 411
  int adcRange    = 4096;   //Options: 2048, 4096, 8192, 16384

  //Configure sensor with these settings
  spo2Sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); 

  spo2Sensor.clearFIFO();

   //Take an average of IR readings at power up		
   for (int x = 0 ; x < 32 ; x++)		
      unblocked_IR_value += spo2Sensor.getRed();  //FIXME red and infrared LED data swapped.
   unblocked_IR_value = unblocked_IR_value/32 + HUMAN_BODY_PRESENT_IR_THRESHOLD;		
}
/*---------------------------------------------------------------------------------
 IC internal FIFO size is 32 samples, make sure the sample rate, average, 
 and .check() is called frequently to meet that buffer not overflow.
---------------------------------------------------------------------------------*/
#define SPO2_BUFFER_SIZE      100
#define SPO2_READ_SIZE        5         //each time read so many samples
#define SPO2_EACH_CALCULATION 25
uint32_t irBuffer [SPO2_BUFFER_SIZE]; //infrared LED samples
uint32_t redBuffer[SPO2_BUFFER_SIZE]; //red LED samples

void calculate_spo2(uint32_t *ir_buffer)
{ 
  float   spo2_value;
  int8_t  spo2_valid;       // = 1 when the SPO2 calculation is valid
  int32_t heart_rate_value;     
  int8_t  heart_rate_valid; // = 1 when the heart rate calculation is valid

  maxim_heart_rate_and_oxygen_saturation
      (irBuffer, SPO2_BUFFER_SIZE, redBuffer, 
      &spo2_value, &spo2_valid, 
      &heart_rate_value, &heart_rate_valid);
      
  if (spo2_valid) 
    spo2_percent = (uint8_t) spo2_value;
  else //invalid data
    spo2_percent = 0;
    
  if (heart_rate_valid)
    ppg_heart_rate = (uint8_t)heart_rate_value;       
  else
    ppg_heart_rate = 0; 
}
  
void handleMax3010xSpo2()
{
  int i;
  int32_t  sample32; 
  int16_t  sample16;

  // keep track average Ir reading
  static uint32_t averageIrValue = 0;
  
  // count how many new samples received, then call calculate_spo2
  static int newSampleCounter = 0; 

  spo2Sensor.check(); //Check the sensor, read up to 3 samples
  if (spo2Sensor.available() < SPO2_READ_SIZE) 
    return;

  // dump old samples, and shift buffer forward
  for (i = SPO2_READ_SIZE; i < SPO2_BUFFER_SIZE; i++)
  {
    redBuffer[i-SPO2_READ_SIZE] = redBuffer[i];
    irBuffer [i-SPO2_READ_SIZE] = irBuffer [i];
  }

  for (i = SPO2_BUFFER_SIZE - SPO2_READ_SIZE; i < SPO2_BUFFER_SIZE; i++)
  {
    //FIXME red and infrared LED data swapped.
    redBuffer[i] = spo2Sensor.getFIFOIR();
    irBuffer [i] = spo2Sensor.getFIFORed();

    spo2Sensor.nextSample(); //We're finished with this sample so move to next sample

    //the ADC is 18 bits -> 16 bits by removing DC offset, and push to BLE tx queue
    sample32 = irBuffer [i];  //uint32 -> int32
    sample32 = EMA_ppg.high_pass_filter(sample32);
    sample16 = (int16_t)sample32;
    sample16 = - sample16;        //reverse the signal

    /* // TEST Code vvv
    {static int16_t  x = 0; if (x >= 100)x = 0; sample16 = x++;}
    */

    averageIrValue += irBuffer[i] / SPO2_EACH_CALCULATION;
    
    // only push data to ble tx buffer when 
    // 1. ble is connected, and
    // 2. human boy present to spo2 sensor
    if ((bleDeviceConnected)&&(averageIrValue>unblocked_IR_value))
      ppg_queue.push(&sample16);    //FIFO for BLE

    if (++newSampleCounter>=SPO2_EACH_CALCULATION)
    {
      calculate_spo2(irBuffer);
      newSampleCounter = 0;
      averageIrValue   = 0;
    }

  }
}

/*
max30102 temperature function

1. the accuracy is +/-1 C, but the real precision of 0.0625 C.
2. the enviroment measurement is 2°C higher than the real temperature sensor.
    due to the LED heating. The LEDs are very low power and won't affect the 
    temp reading much but you may want to turn off the LEDs to avoid any 
    local heating.
3. the body measurement is 1°C lower than real temperature sensor.
*/

/*
float   spo2_temperature;
spo2_temperature = spo2Sensor.readTemperature();
*/              
