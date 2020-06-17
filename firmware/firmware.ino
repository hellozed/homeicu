/*---------------------------------------------------------------------------------
  Main code of HomeICU project.

  Please use Arduino IDE to build and download into ESP32 boards. 

  Arduino language Reference:
  https://www.arduino.cc/en/Reference/

  Heart rate and respiration computation based on original code from Texas Instruments
  equires g4p_control graphing library for processing. 
  Downloaded from Processing IDE Sketch->Import Library->Add Library->G4P Install

  To view the build out, please go to the build folder. 
---------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------
 Arduino/ESP32 library
---------------------------------------------------------------------------------*/
#include <SPI.h>
#include <Wire.h>         // I2C library
#include <Update.h>
#include <SPIFFS.h>       // ESP file system
#include <FS.h>           // File System Headers
#include <ArduinoOTA.h>   // On-The-Air upload (wifi)
/*---------------------------------------------------------------------------------
 HomeICU driver code
---------------------------------------------------------------------------------*/
#include "ADS1292r.h"
#include "ecg_resp_signal_processing.h"
#include "AFE4490_Oximeter.h"
#include "spo2_algorithm.h"
#include "version.h"

void handle_BLE_stack(void);
void initBLE(void);
void handleWebClient(void);
void setupWebServer(void);
void setupBasicOTA(void);

/*---------------------------------------------------------------------------------
 Temperature sensor (ONLY turn either one)
---------------------------------------------------------------------------------*/
//#define TEMP_SENSOR_MAX30325  
#define TEMP_SENSOR_TMP117      

#ifdef TEMP_SENSOR_MAX30325
#include "MAX30205.h"
#endif

#ifdef TEMP_SENSOR_TMP117
#include "TMP117.h"
#endif 
/*---------------------------------------------------------------------------------
  PIN number defined by ESP-WROOM-32 IO port number
---------------------------------------------------------------------------------*/
const int ADS1292_DRDY_PIN  = 26;
const int ADS1292_CS_PIN    = 13;
const int ADS1292_START_PIN = 14;
const int ADS1292_PWDN_PIN  = 27;
const int PUSH_BUTTON_PIN   = 0;
const int AFE4490_CS_PIN    = 21; 
const int AFE4490_DRDY_PIN  = 39; 
const int AFE4490_PWDN_PIN  = 4;
const int LED_PIN           = 2;
const int SENSOR_VP_PIN     = 36;   //GPIO36, ADC1_CH0
/*---------------------------------------------------------------------------------
 constant and global variables
---------------------------------------------------------------------------------*/
#define CES_CMDIF_PKT_START_1         0x0A
#define CES_CMDIF_PKT_START_2         0xFA
#define CES_CMDIF_DATA_LEN_LSB        20
#define CES_CMDIF_DATA_LEN_MSB        0
#define CES_CMDIF_TYPE_DATA           0x02
#define CES_CMDIF_PKT_STOP_1          0x00
#define CES_CMDIF_PKT_STOP_2          0x0B
#define TEMPERATURE_READ_INTERVAL     10000
#define LINELEN                       34 
#define HISTGRM_DATA_SIZE             12*4
#define HISTGRM_CALC_TH               10
#define MAX                           20

unsigned int array[MAX];
int rear = -1;
int sqsum;
int hist[] = {0};
int k=0;
int count = 0;
int min_f=0;
int max_f=0;
int max_t=0;
int min_t=0;
int index_cnt = 0;
int data_count;
int status_size;
uint8_t temperature;
int number_of_samples = 0;
int battery=0;
int bat_count=0;
int bt_rem = 0;
int flag=0;

float sdnn;
float sdnn_f;
float rmssd;
float mean_f;
float rmssd_f;
float per_pnn;
float pnn_f=0;
float tri =0;
 
volatile uint8_t heart_rate = 0;
volatile uint8_t HeartRate_prev = 0;
volatile uint8_t RespirationRate=0;
volatile uint8_t RespirationRate_prev = 0;
volatile uint8_t npeakflag = 0;
volatile long time_count=0;
volatile long hist_time_count=0;
volatile bool histogram_ready_flag = false;
volatile unsigned int RR;

uint8_t ecg_data_buff[20];
uint8_t resp_data_buff[2];
uint8_t ppg_data_buff[20];
uint8_t lead_flag = 0x04;
uint8_t data_len = 20;
uint8_t heartbeat,sp02,respirationrate;
uint8_t histogram_percent_bin[HISTGRM_DATA_SIZE/4];
uint8_t hr_percent_count = 0;
uint8_t hrv_array[20];

uint16_t ecg_stream_cnt = 0;
uint16_t resp_stream_cnt = 0;
uint16_t ppg_stream_cnt = 0;
uint16_t ppg_wave_ir;

int16_t ecg_wave_sample,ecg_filterout;
int16_t res_wave_sample,resp_filterout;

uint32_t heart_rate_histogram[HISTGRM_DATA_SIZE];

bool deviceConnected    = false;
bool oldDeviceConnected = false;
bool temperature_data_ready = false;
bool SpO2_calc_done     = false;
bool ecg_buf_ready      = false;
bool ppg_buf_ready      = false;
bool hrv_ready_flag     = false;
bool battery_data_ready = false;
bool leadoff_detected   = true;
bool startup_flag       = true;

char DataPacket[30];

String strValue = "";

static int bat_prev = 100;
uint8_t battery_percent = 100;

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

const char DataPacketHeader[] = { CES_CMDIF_PKT_START_1, 
                                  CES_CMDIF_PKT_START_2, 
                                  CES_CMDIF_DATA_LEN_LSB, 
                                  CES_CMDIF_DATA_LEN_MSB, 
                                  CES_CMDIF_TYPE_DATA};
const char DataPacketFooter[] = { CES_CMDIF_PKT_STOP_1, 
                                  CES_CMDIF_PKT_STOP_2};

ads1292r        ADS1292R;   // define class ads1292r
ads1292r_processing ECG_RESPIRATION_ALGORITHM; // define class ecg_algorithm
AFE4490         afe4490;
spo2_algorithm  spo2;
ads1292r_data   ads1292r_raw_data;
afe44xx_data    afe44xx_raw_data;

#ifdef TEMP_SENSOR_MAX30325
MAX30205        tempSensor;
#endif

#ifdef TEMP_SENSOR_TMP117
TMP117          tempSensor; 
#endif

int system_init_error = 0;  
/*---------------------------------------------------------------------------------
 Button handle
---------------------------------------------------------------------------------*/
void push_button_intr_handler()
{
  // BOOT button is pressed down

  Serial.println("BOOT is down");

}
/*---------------------------------------------------------------------------------
 battery level check
---------------------------------------------------------------------------------*/
void read_battery_value()
{
  static int adc_val = analogRead(SENSOR_VP_PIN);
  battery += adc_val;

  if (bat_count == 9)
  {
    battery = (battery / 10);
    battery = ((battery * 2) - 400);

    if (battery > 4100)
      battery = 4100;
    else if (battery < 3600)
      battery = 3600;

    if (startup_flag == true)
    {
      bat_prev = battery;
      startup_flag = false;
    }

    bt_rem = (battery % 100); 

    if(bt_rem>80 && bt_rem < 99 && (bat_prev != 0))
      battery = bat_prev;

    if((battery/100)>=41)
      battery = 100;
    else if((battery/100)==40)
      battery = 80;
    else if((battery/100)==39)
      battery = 60;
    else if((battery/100)==38)
      battery=45;
    else if((battery/100)==37)
      battery=30;
    else if((battery/100)<=36)
      battery = 20;

    battery_percent = (uint8_t) battery;
    bat_count=0;
    battery=0;
    battery_data_ready = true;
  }
  else
    bat_count++;
}
 
void add_heart_rate_histogram(uint8_t hr)
{
  uint8_t index = hr/10;
  heart_rate_histogram[index-4]++;
  uint32_t sum = 0;

  if(hr_percent_count++ > HISTGRM_CALC_TH)
  {
    hr_percent_count = 0;
    
    for(int i = 0; i < HISTGRM_DATA_SIZE; i++)
    {
      sum += heart_rate_histogram[i];
    }

    if(sum != 0)
    {
      for(int i = 0; i < HISTGRM_DATA_SIZE/4; i++)
      {
        uint32_t percent = ((heart_rate_histogram[i] * 100) / sum);
        histogram_percent_bin[i] = percent;
      }
    }
    histogram_ready_flag = true;
  }
}
/*---------------------------------------------------------------------------------

---------------------------------------------------------------------------------*/
uint8_t* read_send_data(uint8_t peakvalue,uint8_t respirationrate)
{  
  int meanval;
  uint16_t sdnn;
  uint16_t pnn;
  uint16_t rmsd;
  RR = peakvalue;
  k++;

  if(rear == MAX-1)
  {
    for(int i=0;i<(MAX-1);i++)
    {
      array[i]=array[i+1];
    }

    array[MAX-1] = RR;   
  }
  else
  {     
    rear++;
    array[rear] = RR;
  }

  if(k>=MAX)
  { 
    max_f = HRVMAX(array);
    min_f = HRVMIN(array);
    mean_f = mean(array);
    sdnn_f = sdnn_ff(array);
    pnn_f = pnn_ff(array);
    rmssd_f=rmssd_ff(array);
  
    meanval = mean_f*100;
    sdnn= sdnn_f*100;
    pnn= pnn_f*100;
    rmsd=rmssd_f*100;

    hrv_array[0]= meanval;
    hrv_array[1]= meanval>>8;
    hrv_array[2]= meanval>>16;
    hrv_array[3]= meanval>>24;
    hrv_array[4]= sdnn;
    hrv_array[5]= sdnn>>8;
    hrv_array[6]= pnn;
    hrv_array[7]= pnn>>8;
    hrv_array[10]=rmsd;
    hrv_array[11]=rmsd>>8;
    hrv_array[12]=respirationrate;
    hrv_ready_flag= true;
  }
}
/*---------------------------------------------------------------------------------
heart-rate variability (HRV)
---------------------------------------------------------------------------------*/
int HRVMAX(unsigned int array[])
{  
  for(int i=0;i<MAX;i++)
  {
    if(array[i]>max_t)
    {
      max_t = array[i];
    }
  }
  return max_t;
}

int HRVMIN(unsigned int array[])
{   
  min_t = max_f;
  for(int i=0;i<MAX;i++)
  {
    if(array[i]< min_t)
    {
      min_t = array[i]; 
    }
  }
  return min_t;
}

float mean(unsigned int array[])
{ 
  int sum = 0;
  float mean_rr;
 
  for(int i=0;i<(MAX);i++)
  {
    sum = sum + array[i];
  }
  mean_rr = (((float)sum)/ MAX);  
  return mean_rr;
} 

float sdnn_ff(unsigned int array[])
{
  int sumsdnn = 0;
  int diff;
 
  for(int i=0;i<(MAX);i++)
  {
    diff = (array[i]-(mean_f));
    diff = diff*diff;
    sumsdnn = sumsdnn + diff;   
  }

  sdnn = (sqrt(sumsdnn/(MAX)));
  return   sdnn;
}

float pnn_ff(unsigned int array[])
{ 
  unsigned int pnn50[MAX];
  long l;

  count = 0;
  sqsum = 0;

  for(int i=0;i<(MAX-2);i++)
  {
    l = array[i+1] - array[i];       //array[] is unsigned integer 0-65535, l = -65535 ~ +65535
    pnn50[i]= abs(l);                //abs() return 0~65535
    sqsum = sqsum + (pnn50[i]*pnn50[i]);

    if(pnn50[i]>50)
    {
      count = count + 1;    
    }

  }
  per_pnn = ((float)count/MAX)*100;
  return per_pnn;
}

float rmssd_ff(unsigned int array[])
{
  unsigned int pnn50[MAX];
  long l;

  sqsum = 0;

  for(int i=0;i<(MAX-2);i++)
  {
    l = array[i+1] - array[i];       //array[] is unsigned integer 0-65535, l = -65535 ~ +65535
    pnn50[i]= abs(l);                //abs() return 0~65535
    sqsum = sqsum + (pnn50[i]*pnn50[i]);
  }

  rmssd = sqrt(sqsum/(MAX-1));
  return rmssd;
}
void halt_and_flash(void)
{
  // Only for debuging
  Serial.println("System Halt!");
  while (true)
  {
    delay(800);
    digitalWrite(LED_PIN, HIGH);
    delay(800);
    digitalWrite(LED_PIN, LOW);
  }
}

/*---------------------------------------------------------------------------------
 The ESP32 has four SPI buses, only two of them are available to use, HSPI and VSPI. 
 Simply using the SPI API as illustrated in Arduino examples will use VSPI, leaving HSPI unused.
 
 However if we simply intialise two instance of the SPI class for both
 of these buses both can be used. However when just using these the Arduino
 way only will actually be outputting at a time.

 SPI	  MOSI	  MISO	  CLK	    CS
 VSPI	GPIO23	GPIO19	GPIO18	GPIO5
 HSPI	GPIO13	GPIO12	GPIO14	GPIO15
 FIXME This design only uses VSPI, the default CS pin is IO5, but this design use IO21.
 TODO: IDE provides another example of SPI code.
---------------------------------------------------------------------------------*/
void initSPI(void)
{
  SPI.begin();
  Wire.begin(25,22);  //FIXME test this line
  SPI.setClockDivider (SPI_CLOCK_DIV16);
  SPI.setBitOrder     (MSBFIRST);
  SPI.setDataMode     (SPI_MODE0);
  delay(10);        //delay 10ms
  afe4490.afe44xxInit (AFE4490_CS_PIN,AFE4490_PWDN_PIN);
  delay(10); 
  SPI.setDataMode (SPI_MODE1);          //Set SPI mode as 1
  delay(10);
}
/*---------------------------------------------------------------------------------
The setup() function is called when a sketch starts. Use it to initialize variables, 
pin modes, start using libraries, etc. The setup() function will only run once, 
after each powerup or reset of the  board.
---------------------------------------------------------------------------------*/
void setup()
{
  // Make sure serial port on first
  // Setup serial port U0UXD for programming and reset/boot
  Serial.begin  (115200);   // Baudrate for serial communication
  
  Serial.write(12);         // ASCII for a Form feed to "clear" the screen
  Serial.println("************************");
  Serial.println("HomeICU Firmware");
  Serial.println("************************");
  Serial.println("version:");
  Serial.println(homeicu_version); 
  Serial.println("commits:");
  Serial.println(homeicu_commits);
 
  // initalize the  data ready and chip select pins:
  // Pin numbers are defined as ESP-WROOM-32, not as ESP32 processor
  pinMode(ADS1292_DRDY_PIN,   INPUT);  
  pinMode(ADS1292_CS_PIN,     OUTPUT);    
  pinMode(ADS1292_START_PIN,  OUTPUT);
  pinMode(ADS1292_PWDN_PIN,   OUTPUT);  
  pinMode(LED_PIN,            OUTPUT); 
  pinMode(AFE4490_PWDN_PIN,   OUTPUT);
  pinMode(AFE4490_CS_PIN,     OUTPUT);//Slave Select
  pinMode(AFE4490_DRDY_PIN,   INPUT);// data ready 

  pinMode(PUSH_BUTTON_PIN,    INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_PIN), push_button_intr_handler, FALLING);

  initBLE();                  //low energy blue tooth 
  setupBasicOTA();            //Over The Air for code uploading
  setupWebServer();           //Web server for code uploading
  //TODO initSPI();            //initalize SPI

  //Initialize SPI file system
  if(!SPIFFS.begin(true)) 
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    system_init_error++;
  }
  else
    Serial.println("SPIFFS Init OK!");

  ADS1292R.ads1292_Init(ADS1292_CS_PIN,ADS1292_PWDN_PIN,ADS1292_START_PIN);  //initalize ADS1292 slave
  delay(10); 
  // Digital2 is attached to Data ready pin of AFE is interrupt0 in ARduino
  attachInterrupt(digitalPinToInterrupt(ADS1292_DRDY_PIN),ads1292r_interrupt_handler, FALLING ); 
  
  if (tempSensor.begin()) 
    Serial.println("Temperature sensor: OK.");
  else
    Serial.println("Temperature sensor: Missing.");
 
  if (system_init_error>0)
    halt_and_flash(); 
  else   
    Serial.println("Initialization done!");
}
/*---------------------------------------------------------------------------------
After creating a setup() function, which initializes and sets the initial values, 
the loop() function loops consecutively, allowing your program to change and respond.
---------------------------------------------------------------------------------*/
void loop()
{
  boolean ret;

  ArduinoOTA.handle();        // This is for "On The Air" update function 
//FIXME  handle_BLE_stack();
  handleWebClient();
  delay(2);
#ifdef SSSS  //FIXME
  ret = ADS1292R.getAds1292r_Data_if_Available(ADS1292_DRDY_PIN,ADS1292_CS_PIN,&ads1292r_raw_data);

  if (ret == true)
  {  
    ecg_wave_sample = (int16_t)(ads1292r_raw_data.raw_ecg >> 8) ;  // ignore the lower 8 bits out of 24bits 
    res_wave_sample = (int16_t)(ads1292r_raw_data.raw_resp>>8) ;
  
    if (!((ads1292r_raw_data.status_reg & 0x1f) == 0))
    {
      // the measure lead is OFF the body 
      leadoff_detected  = true; 
      lead_flag         = 0x04;
      ecg_filterout     = 0;
      resp_filterout    = 0;      
      DataPacket[14]    = 0;
      DataPacket[16]    = 0;
    }  
    else
    {
      // the measure lead is ON the body 
      leadoff_detected  = false;
      lead_flag = 0x06;

      // filter out the line noise @40Hz cutoff 161 order
      ECG_RESPIRATION_ALGORITHM.Filter_CurrentECG_sample  (&ecg_wave_sample, &ecg_filterout);   
      ECG_RESPIRATION_ALGORITHM.Calculate_HeartRate       (ecg_filterout,&heart_rate,&npeakflag); // calculate
      ECG_RESPIRATION_ALGORITHM.Filter_CurrentRESP_sample (res_wave_sample, &resp_filterout);
      ECG_RESPIRATION_ALGORITHM.Calculate_RespRate        (resp_filterout,&RespirationRate);   
   
      if(npeakflag == 1)
      {
        read_send_data(heart_rate,RespirationRate);
        add_heart_rate_histogram(heart_rate);
        npeakflag = 0;
      }
   
      ecg_data_buff[ecg_stream_cnt++] = (uint8_t)ecg_wave_sample;//ecg_filterout;
      ecg_data_buff[ecg_stream_cnt++] = (ecg_wave_sample>>8);//(ecg_filterout>>8);
      
      if(ecg_stream_cnt >=18)
      {
          ecg_buf_ready = true;
          ecg_stream_cnt = 0;
      }
       
      DataPacket[14] = RespirationRate;
      DataPacket[16] = heart_rate;
    }
  
    memcpy(&DataPacket[0], &ecg_filterout, 2);
    memcpy(&DataPacket[2], &resp_filterout, 2);
    SPI.setDataMode (SPI_MODE0);
    afe4490.get_AFE4490_Data(&afe44xx_raw_data,AFE4490_CS_PIN,AFE4490_DRDY_PIN);
    ppg_wave_ir = (uint16_t)(afe44xx_raw_data.IR_data>>8);
    ppg_wave_ir = ppg_wave_ir;
    
    ppg_data_buff[ppg_stream_cnt++] = (uint8_t)ppg_wave_ir;
    ppg_data_buff[ppg_stream_cnt++] = (ppg_wave_ir>>8);
  
    if(ppg_stream_cnt >=18)
    {
      ppg_buf_ready = true;
      ppg_stream_cnt = 0;
    }
  
    memcpy(&DataPacket[4], &afe44xx_raw_data.IR_data, sizeof(signed long));
    memcpy(&DataPacket[8], &afe44xx_raw_data.RED_data, sizeof(signed long));
  
    if( afe44xx_raw_data.buffer_count_overflow)
    {
      
      if (afe44xx_raw_data.spo2 == -999)
      {
        DataPacket[15] = 0;
        sp02 = 0;
      }
      else
      { 
        DataPacket[15] =  afe44xx_raw_data.spo2;
        sp02 = (uint8_t)afe44xx_raw_data.spo2;       
      }

      SpO2_calc_done = true;
      afe44xx_raw_data.buffer_count_overflow = false;
    }
   
    DataPacket[17] = 80;  //bpsys
    DataPacket[18] = 120; //bp dia
    DataPacket[19]=  ads1292r_raw_data.status_reg;  

    SPI.setDataMode (SPI_MODE1);
   
    if ((time_count++ * (1000/SAMPLING_RATE)) > TEMPERATURE_READ_INTERVAL)
    {      
      float temp;
      #ifdef TEMP_SENSOR_MAX30325
      temp = tempSensor.getTemperature()*100; // read temperature for every 100ms
      temperature =  (uint16_t) temp;         // °C 
      #endif

      #ifdef TEMP_SENSOR_TMP117
        // Data Ready is a flag for the conversion modes - in continous conversion the dataReady flag should always be high
      if (tempSensor.dataReady()) // Function to make sure that there is data ready to be printed, only prints temperature values when data is ready
      {
        float tempC = tempSensor.readTempC();
        float tempF = tempSensor.readTempF();
        // Print temperature in °C and °F
        Serial.println(); // Create a white space for easier viewing
        Serial.print("Temperature in Celsius: ");
        Serial.println(tempC);
        Serial.print("Temperature in Fahrenheit: ");
        Serial.println(tempF);
        temperature = tempC;
      }
      #endif 

      time_count = 0;
      DataPacket[12] = (uint8_t) temperature; 
      DataPacket[13] = (uint8_t) (temperature >> 8);
      temperature_data_ready = true;
      //reading the battery with same interval as temperature sensor
      read_battery_value();
    }
  }
#endif 
}