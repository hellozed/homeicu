/*---------------------------------------------------------------------------------
  main code of the HomeICU project.

  Arduino IDE and VSCode is needed to build/download into boards. 

  Arduino Language Reference:
  https://www.arduino.cc/en/Reference/

  Heart rate and respiration computation based on original code from Texas Instruments.
---------------------------------------------------------------------------------*/
#include "firmware.h"
#include <SPI.h>
#include <Wire.h>         // i2c library
#include "version.h"
#include <Smoothed.h> 	  // SMA library, need be installed from Arduino/Sketch/Include libraries 

void initECG();
void handleECG();
void getTestData(void);
void handleCLI();
/*---------------------------------------------------------------------------------
 Temperature sensor (ONLY turn on one of them)
---------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------
 constant and global variables
---------------------------------------------------------------------------------*/
volatile uint32_t buttonInterruptTime = 0;
volatile int      buttonEventPending = false;
volatile bool     spo2_interrupt_flag = false;
volatile SemaphoreHandle_t timerSemaphore;
hw_timer_t * timer = NULL;

portMUX_TYPE buttonMux    = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE ads1292rMux  = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE oximeterMux  = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux     = portMUX_INITIALIZER_UNLOCKED;

Smoothed <float> batteryADC; 

uint8_t   ecg_heart_rate, old_ecg_heart_rate  = 0;
uint8_t   ppg_heart_rate, old_ppg_heart_rate  = 0;
uint8_t   spo2_percent,   old_spo2_percent    = 0;
uint8_t   battery_percent,old_battery_percent = 120; // number outside 0~100 will trigger ble sending

union FloatByte body_temp;
float old_temperature = 120; // number outside 0~45 will trigger ble sending

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
  button interrupt
---------------------------------------------------------------------------------*/
void IRAM_ATTR oximeter_interrupt_handler()
{
  portENTER_CRITICAL_ISR(&oximeterMux);
  spo2_interrupt_flag = true;
  portEXIT_CRITICAL_ISR (&oximeterMux);  
}
 
void clear_interrupt()
{
  portENTER_CRITICAL_ISR(&oximeterMux);
  spo2_interrupt_flag = false;
  portEXIT_CRITICAL_ISR (&oximeterMux);  
}

/*---------------------------------------------------------------------------------
  button interrupt
---------------------------------------------------------------------------------*/
void IRAM_ATTR button_interrupt_handler()
{
  portENTER_CRITICAL_ISR(&buttonMux);
    buttonEventPending++;
    buttonInterruptTime = millis();
  portEXIT_CRITICAL_ISR(&buttonMux);
}

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
  repeatable timer interrupt

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

void initTimer() {
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every x microseconds).
  // Repeat the alarm (third parameter)

  #define TIMER_MICRO_SECONDS 1000000*1 // * second
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
  }  
}
/*---------------------------------------------------------------------------------
 messure battery voltage levle

 get smoothed values and filter out noise: 
 -----------------------------------------------*/
void measureBattery()
{
  unsigned int adc;

  static unsigned long  LastReadTime = 0;
  #define BATTERY_READ_INTERVAL     2000   //millis
  
  // check if the right time to read
  if (millis() - LastReadTime < BATTERY_READ_INTERVAL)
    return;   // continue wait


  LastReadTime = millis();    

  // get adc sample and smooth it    
  batteryADC.add(analogRead(SENSOR_VP_PIN));
  adc = batteryADC.get();

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

  battery_percent = battery_percent & 0b11111110;  //reduce resolution
}
/*---------------------------------------------------------------------------------
 body temperature measurement
---------------------------------------------------------------------------------*/
void measureTemperature()
{
  #define TEMPERATURE_READ_INTERVAL     (2*1000)  //millis
  
  static unsigned long LastReadTime = 0;

  if (millis() - LastReadTime > TEMPERATURE_READ_INTERVAL)
  { 
    LastReadTime = millis();     

    body_temp.f = getTemperature();    // °C 
    
    #if SIM_TEMPERATURE
    body_temp.f =  (float)analogRead(SENSOR_TEMP)*100/4096;
    #endif
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
  SPI.setBitOrder     (MSBFIRST);

  // the higher speed will cause ecg stop working
  // example code use SPI_CLOCK_DIV16 
  SPI.setClockDivider (SPI_CLOCK_DIV32);  
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
  // pin numbers are defined as ESP-WROOM-32, not as ESP32 processor
  pinMode(ADS1292_DRDY_PIN,   INPUT);  
  pinMode(ADS1292_CS_PIN,     OUTPUT);    
  pinMode(ADS1292_START_PIN,  OUTPUT);
  pinMode(ADS1292_PWDN_PIN,   OUTPUT);  
  pinMode(LED_PIN,            OUTPUT); 
  pinMode(SPO2_START_PIN,     OUTPUT);
  pinMode(AFE4490_CS_PIN,     OUTPUT);  // slave select
  pinMode(OXIMETER_INT_PIN,   INPUT);   // data ready 
  pinMode(PUSH_BUTTON_PIN,    INPUT_PULLUP);

  // init battery voltage measurement
  batteryADC.begin(SMOOTHED_AVERAGE, 10);	  //use moving average to smooth the signal
  for (int i=0;i<10;i++)                    //force to clear buffer (constructor function has bug) 
    batteryADC.add( analogRead(SENSOR_VP_PIN) ); 
    
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_PIN), button_interrupt_handler, FALLING);
 
  initTimer();               

  initBLE();                  // low energy blue tooth 
  //------------------------------------------------ 
  // initilize, and scan i2c device address
  // -----------------------------------------------
  Wire.begin( I2C_SDA_PIN,    // initialize I2C
              I2C_SCL_PIN,
              400000);        //standard speed is 100000, fast speed is 400000
  {
    uint8_t error, address;
    Serial.println("Scanning I2C...");
    for(address = 1; address < 127; address++ ) {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0) {
        Serial.print("I2C device found at address 0x");
        if (address<16) Serial.print("0");
          Serial.println(address,HEX);
      }
    }
  }
  //------------------------------------------------ 
  initSPI();                  // initialize SPI
  attachInterrupt(digitalPinToInterrupt(ADS1292_DRDY_PIN),ads1292r_interrupt_handler, FALLING); 
  initECG();                  // with different CS pin and SPI mode.
  //------------------------------------------------
  // init spo2

  #if   (SPO2_TYPE==OXI_AFE4490)
    digitalWrite(SPO2_START_PIN, LOW);
    delay(500);
    digitalWrite(SPO2_START_PIN, HIGH);
    delay(500);
    afe4490.init();             // SPI controls ADS1292R and AFE4490,
    attachInterrupt(digitalPinToInterrupt(OXIMETER_INT_PIN), 
                    oximeter_interrupt_handler, RISING ); 
  #elif (SPO2_TYPE==OXI_MAX30102)
    initMax3010xSpo2();
    attachInterrupt(digitalPinToInterrupt(OXIMETER_INT_PIN), 
                    oximeter_interrupt_handler, FALLING ); 
  #endif 
  //------------------------------------------------
  initAcceleromter();

  if (initTemperature()) 
    Serial.println("Temperature sensor: OK.");
  else
  {
    Serial.println("!! temperature sensor missing.");
    system_init_error++;
  }
  //------------------------------------------------
  initBasicOTA();             // uploading by Over The Air code
  #if WEB_UPDATE
  setupWebServer();           // uploading by Web server
  #endif 
  //------------------------------------------------
  Serial.printf("Setup() done. Found %d errors\r\n\r\n",system_init_error);
  //------------------------------------------------
  #if CLI_FEATURE
    Serial.println("command line interface (CLI) enabled.");
    Serial.println("Type \"help\".");
  #endif 
}
/*---------------------------------------------------------------------------------
 setup() => loop() 
---------------------------------------------------------------------------------*/
void loop()
{
  handleCLI();

  doTimer();                  // process timer event
  
  doButton();                 // process button event

  handleOTA();                // "On The Air" update function 

  handleBLE();                // handle bluetooth low energy

  handleECG();                // handle ECG and RESP

  #if   (SPO2_TYPE==OXI_AFE4490)
    afe4490.getData();          // handle SpO2 and PPG 
  #elif (SPO2_TYPE==OXI_MAX30102)
    handleMax3010xSpo2();       // handel SpO2 and PPG
  #endif   

  measureTemperature();       // body temperature

  handelAcceleromter();       // motion detection with accelerometer

  measureBattery();           // measure battery power percent

  #if WEB_UPDATE
  handleWebClient();          // web server
  #endif 
}
