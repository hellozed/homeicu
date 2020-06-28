/*---------------------------------------------------------------------------------
  main code of the HomeICU project.

  Arduino IDE and VSCode is needed to build/download into boards. 

  Arduino Language Reference:
  https://www.arduino.cc/en/Reference/

  Heart rate and respiration computation based on original code from Texas Instruments
  requires a g4p_control graphing library for processing. 

  Downloaded from Processing IDE Sketch->Import Library->Add Library->G4P Install

  To view the build-out, please go to the build folder. 
---------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------
 Arduino/ESP32 library
---------------------------------------------------------------------------------*/
#include <SPI.h>
#include <Wire.h>         // I2C library
#include <ArduinoOTA.h>   // On-The-Air upload (wifi)
/*---------------------------------------------------------------------------------
 HomeICU driver code
---------------------------------------------------------------------------------*/
#include "ADS1292r.h"
#include "ecg_resp.h"
#include "AFE4490_Oximeter.h"
#include "spo2_algorithm.h"
#include "version.h"
#include "firmware.h"
#include "TMP117.h"
#include "MAX30205.h"

class AFE4490 afe4490;
class ADS1292 ads1292;
void initBLE();
void handleBLEstack();
void handleWebClient();
void setupWebServer();
void setupBasicOTA();
void getTestData(void);
/*---------------------------------------------------------------------------------
 Temperature sensor (ONLY turn on one of them)
---------------------------------------------------------------------------------*/
#define TEMP_SENSOR_MAX30325  false
#define TEMP_SENSOR_TMP117    false  
/*---------------------------------------------------------------------------------
 constant and global variables
---------------------------------------------------------------------------------*/
volatile uint32_t buttonInterruptTime = 0;
volatile int      buttonEventPending = false;

volatile SemaphoreHandle_t timerSemaphore;
hw_timer_t * timer = NULL;

portMUX_TYPE buttonMux  = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE ads1292Mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE AFE4490Mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux   = portMUX_INITIALIZER_UNLOCKED;

bool temperatureReady   = false;
bool batteryDataReady   = false;

uint8_t battery_percent = 100;

union FloatByte bodyTemperature;

#if TEMP_SENSOR_MAX30325
MAX30205        tempSensor;
#endif

#if TEMP_SENSOR_TMP117
TMP117          tempSensor; 
#endif

int system_init_error = 0;  
/*---------------------------------------------------------------------------------
Interrupt Routine should: 

    Keep it short
    Don't use delay ()
    Don't do serial prints
    Make variables shared with the main code volatile and protected by "critical sections"
    Don't try to turn interrupts off or on

ESP32 has 2 cores, each has 6 internal peripheral interrupts:
    3 x timer comparators
    1 x performance monitor
    3 x software interrupts.

Arduino normally use "NoInterrupts" and "Interrupts" to enable/disable interrupt
for critical sections. But "NoInterrupts" and "Interrupts" have not been 
implemented in ESP32 Arduino, please use the following methods.

    portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

    portENTER_CRITICAL(&myMutex);
    //critical section
    portEXIT_CRITICAL(&myMutex);

    portENTER_CRITICAL_ISR(&myMutex);
    //ISR section
    portEXIT_CRITICAL_ISR(&myMutex);

use semaphore if using FreeRtos. 

Refer to:

http://www.iotsharing.com/2017/06/how-to-use-binary-semaphore-mutex-counting-semaphore-resource-management.html
http://www.gammon.com.au/interrupts
*/
/*---------------------------------------------------------------------------------
  Button interrupt handler
---------------------------------------------------------------------------------*/
void IRAM_ATTR button_interrupt_handler()
{
  portENTER_CRITICAL_ISR(&buttonMux);
    buttonEventPending++;
    buttonInterruptTime = millis();
  portEXIT_CRITICAL_ISR(&buttonMux);
}

// if button was pressed
void doButton() {
  if (buttonEventPending>0)
  {
    if ((millis() - buttonInterruptTime) > 100)    //after a delay
    {
      if (digitalRead(PUSH_BUTTON_PIN) == 0)  //still LOW    
      {
        Serial.println("BUTTON Yes");
      }
      portENTER_CRITICAL(&buttonMux);
        buttonEventPending--;                 //clear the event pending flag
      portEXIT_CRITICAL(&buttonMux);
    } 
  } 
}  
/*---------------------------------------------------------------------------------
 Repeat Timer interrupt

    Note: code for killing the timer
    if (timer) {
        timerEnd(timer);
        timer = NULL;
    }
---------------------------------------------------------------------------------*/
void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&timerMux);
  // Do something here
  portEXIT_CRITICAL_ISR(&timerMux);

  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void setupTimer() {
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every x microseconds).
  // Repeat the alarm (third parameter)

  #define TIMER_MICRO_SECONDS 1000000*0.2 // * second
  timerAlarmWrite(timer, TIMER_MICRO_SECONDS, true); 

  // Start an alarm
  timerAlarmEnable(timer);
}

void doTimer() 
{
  // if Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);

    portEXIT_CRITICAL (&timerMux);

    #if JOYTICK_TEST
    // ESP32 ADC return a 12bit value 0~4095 (uint16_t) 
    {
    int x, y;
    pinMode(JOYY_PIN,   INPUT);  
    y = analogRead(JOYY_PIN);
    // Serial.printf("Y:%05d BAT:%3d Tmp:%4f\r", y, battery_percent, bodyTemperature.f);
    }
    #endif 

    #if ECG_BLE_TEST
    ads1292.getTestData();
    #endif 
    
    #if SIM_PPG
    afe4490.simulateData();
    #endif 
  }  
}
/*---------------------------------------------------------------------------------
 battery measurement
---------------------------------------------------------------------------------*/
void read_battery_value()
{
  #define BAT_SAMPLE  5     
  
  static    uint16_t    adcSample[BAT_SAMPLE];
  static unsigned long  LastReadTime = 0;

  unsigned int          adc = 0;
  int i;

  #define BATTERY_READ_INTERVAL     (0.2*1000)   //millis
  
  // check if the right time to read
  if (millis() - LastReadTime < BATTERY_READ_INTERVAL)
    return;   // not the right time

  // it is the right time, continue read adc
  LastReadTime = millis();    

  // average several adc sample to reduce noise
  
  // shift adc sample into buffer      
  for (i=BAT_SAMPLE-1; i>0; i--)
    adcSample[i] = adcSample[i-1];
  adcSample[0] = analogRead(SENSOR_VP_PIN);

  // sum sample data together   
  for (i=0; i<BAT_SAMPLE; i++)
    adc += adcSample[i];

  // divide it by sample number  
  adc = adc/BAT_SAMPLE;

  #if SIM_BATTERY
  // ESP32 ADC return a 12bit value 0~4095 (uint16_t) 
  // i = 0~4095 simulate y = 3600~4100
  // y = 3600 + (4100-3600)*i/4095
  adc = 3600 + (4100-3600)*adc/4095;
  #endif 
  
  /*
  ADC value (3600~4100) -> Battery Life (20%~100%)
  It is not linear relationship between ADC and battery.
  Adc : Batterry 
  4100 100%
  4000 80%
  3900 60%
  3800 45%
  3700 30%
  3600 20%
  Batter Life = x Hours
  */
  #define Percent(adcNow, adcH, adcL, percentH, percentL) \
          (percentL+(percentH-percentL)*(adcNow-adcL)/(adcH-adcL))

  if (adc >=4000)
    battery_percent = Percent(adc, 4100, 4000,100, 80);
  else if (adc >=3900)
    battery_percent = Percent(adc, 4000, 3900, 80, 60);
  else if (adc >=3800)
    battery_percent = Percent(adc, 3900, 3800, 60, 45);
  else if (adc >=3700)
    battery_percent = Percent(adc, 3800, 3700, 45, 30);
  else if (adc >=3600)
    battery_percent = Percent(adc, 3700, 3600, 30, 20);         
  else
    battery_percent = 10;

  //Serial.printf("adc:%u average+adjust %u battery: %u%% \r", 
  //              adcSample[0], adc, battery_percent);
  batteryDataReady = true; 

}
/*---------------------------------------------------------------------------------
 body temperature measurement
---------------------------------------------------------------------------------*/
void read_temperature_value()
{
  #define TEMPERATURE_READ_INTERVAL     (2*1000)  //millis
  
  static unsigned long LastReadTime = 0;

  if (millis() - LastReadTime > TEMPERATURE_READ_INTERVAL)
  { 
    LastReadTime = millis();     

    #if TEMP_SENSOR_MAX30325
    bodyTemperature.f = tempSensor.getTemperature()*100;    // °C 
    #endif

    #if TEMP_SENSOR_TMP117
    // Data Ready is a flag for the conversion modes
    // the dataReady flag should always be high in continuous conversion 
    if (tempSensor.dataReady()) 
      bodyTemperature.f = tempSensor.readTempC();;
    #endif 
    
    #if SIM_TEMPERATURE
    bodyTemperature.f =  (float)analogRead(SENSOR_TEMP)*100/4096;
    #endif
  
    temperatureReady = true;
  }
}
/*---------------------------------------------------------------------------------
 The ESP32 has four SPI buses, only two of them are available to use, HSPI and VSPI. 
 Simply using the SPI API as illustrated in Arduino examples will use VSPI, leaving HSPI unused.
 
 If we simply initialize two instances of the SPI class for both
 of these buses both can be used. When using these the Arduino
 way only will be outputting at a time.

 SPI	  MOSI	  MISO	  CLK	    CS
 VSPI	GPIO23	GPIO19	GPIO18	GPIO5
 HSPI	GPIO13	GPIO12	GPIO14	GPIO15

 This design only uses VSPI, the default CS pin is IO5.
 
---------------------------------------------------------------------------------*/
void initSPI()
{
  SPI.begin();
  SPI.setClockDivider (SPI_CLOCK_DIV16);
  SPI.setBitOrder     (MSBFIRST);
  delay(10);           // delay 10ms
}
/*---------------------------------------------------------------------------------
The setup() function is called when a sketch starts. Use it to initialize variables, 
pin modes, start using libraries, etc. The setup() function will only run once, 
after power-up or reset of the  board.
---------------------------------------------------------------------------------*/
void setup()
{
  uint64_t chipid;
  
  // Make sure serial port on first
  // Setup serial port U0UXD for programming and reset/boot
  Serial.begin  (115200);   // Baudrate for serial communication
  chipid=ESP.getEfuseMac(); // chip ID is MAC address(6 bytes).
  
  Serial.println("************************************************");
  Serial.println("HomeICU Firmware");
  Serial.println("************************************************");
  Serial.printf ("Version: %s Git commits: %s \r\n", homeicu_version, homeicu_commits);
	Serial.printf ("MAC address: %04X %08X \r\n",(uint16_t)(chipid>>32), (uint32_t)chipid);

  // initialize the data ready and chip select pins:
  // Pin numbers are defined as ESP-WROOM-32, not as ESP32 processor
  pinMode(ADS1292_DRDY_PIN,   INPUT);  
  pinMode(ADS1292_CS_PIN,     OUTPUT);    
  pinMode(ADS1292_START_PIN,  OUTPUT);
  pinMode(ADS1292_PWDN_PIN,   OUTPUT);  
  pinMode(LED_PIN,            OUTPUT); 
  pinMode(AFE4490_PWDN_PIN,   OUTPUT);
  pinMode(AFE4490_CS_PIN,     OUTPUT);  // slave select
  pinMode(AFE4490_DRDY_PIN,   INPUT);   // data ready 
  pinMode(PUSH_BUTTON_PIN,    INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_PIN), button_interrupt_handler, FALLING);
 
  setupTimer();               

  initBLE();                  // low energy blue tooth 
   
  initSPI();                  // initialize SPI
  afe4490.init();             // SPI controls ADS1292 and AFE4490,
  ads1292.init();             // with different CS pin and SPI mode.
   
  Wire.begin(25,22);          // initialize I2C, SDA=25pin SCL=22pin
  
  setupBasicOTA();            // uploading by Over The Air code
  #if WEB_UPDATE
  setupWebServer();           // uploading by Web server
  #endif 
  
  // data ready for reading for ADS1292 and AFE4490
  attachInterrupt(digitalPinToInterrupt(ADS1292_DRDY_PIN),ads1292_interrupt_handler, FALLING); 
  attachInterrupt(digitalPinToInterrupt(AFE4490_DRDY_PIN),afe4490_interrupt_handler, RISING ); 

  #if (TEMP_SENSOR_MAX30325 | TEMP_SENSOR_TMP117)
  if (tempSensor.begin()) 
    Serial.println("Temperature sensor: OK.");
  else
    Serial.println("Temperature sensor: Missing.");
  #endif 

  Serial.printf("Setup() done. Found %d errors\r\n\r\n",system_init_error);
}
/*---------------------------------------------------------------------------------
setup() => loop() 
---------------------------------------------------------------------------------*/
void loop()
{
  doTimer();                  // process timer event
  
  doButton();                 // process button event

  ArduinoOTA.handle();        // "On The Air" update function 

  handleBLEstack();           // handle bluetooth low energy

  ads1292.getData();          // handle ECG and RESP
  
  afe4490.getData();          // handle SpO2 and PPG 
  
  read_temperature_value();   // battery power percent

  read_battery_value();       // measure body temperature

  #if WEB_UPDATE
  handleWebClient();          // web server
  #endif 
}