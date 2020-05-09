/* 
   Heartrate and respiration computation based on original code from Texas Instruments
   Requires g4p_control graphing library for processing. 
   Downloaded from Processing IDE Sketch->Import Library->Add Library->G4P Install
*/

/*---------------------------------------------------------------------------------
 wifi for OTA Update
---------------------------------------------------------------------------------*/

const char* wifi_ssid = "...";
const char* wifi_password = ".....";


// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif
#define abs(x) ((x)>0?(x):-(x))


/*---------------------------------------------------------------------------------
 Library Headers
---------------------------------------------------------------------------------*/
#include <SPI.h>
#include <Wire.h>
#include <ESPmDNS.h>
#include <Update.h>
#include "SPIFFS.h"
#include <FS.h>      //Include File System Headers
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoOTA.h>

// HomeICU driver code
#include "ADS1292r.h"
#include "ecg_resp_signal_processing.h"
#include "AFE4490_Oximeter.h"
#include "MAX30205.h"
#include "spo2_algorithm.h"
/*---------------------------------------------------------------------------------

---------------------------------------------------------------------------------*/
#define Heartrate_SERVICE_UUID        (uint16_t(0x180D))
#define Heartrate_CHARACTERISTIC_UUID (uint16_t(0x2A37))
#define sp02_SERVICE_UUID             (uint16_t(0x1822)) 
#define sp02_CHARACTERISTIC_UUID      (uint16_t(0x2A5E))
#define DATASTREAM_SERVICE_UUID       (uint16_t(0x1122)) 
#define DATASTREAM_CHARACTERISTIC_UUID (uint16_t(0x1424))
#define TEMP_SERVICE_UUID             (uint16_t(0x1809)) 
#define TEMP_CHARACTERISTIC_UUID      (uint16_t(0x2a6e))
#define BATTERY_SERVICE_UUID          (uint16_t(0x180F)) 
#define BATTERY_CHARACTERISTIC_UUID   (uint16_t(0x2a19))
#define HRV_SERVICE_UUID              "cd5c7491-4448-7db8-ae4c-d1da8cba36d0"
#define HRV_CHARACTERISTIC_UUID       "01bfa86f-970f-8d96-d44d-9023c47faddc"
#define HIST_CHARACTERISTIC_UUID      "01bf1525-970f-8d96-d44d-9023c47faddc"

#define BLE_MODE                      0X01
#define WEBSERVER_MODE                0X02
#define CES_CMDIF_PKT_START_1         0x0A
#define CES_CMDIF_PKT_START_2         0xFA
#define CES_CMDIF_DATA_LEN_LSB        20
#define CES_CMDIF_DATA_LEN_MSB        0
#define CES_CMDIF_TYPE_DATA           0x02
#define CES_CMDIF_PKT_STOP_1          0x00
#define CES_CMDIF_PKT_STOP_2          0x0B
#define MAX30205_READ_INTERVAL        10000
#define LINELEN                       34 
#define HISTGRM_DATA_SIZE             12*4
#define HISTGRM_CALC_TH               10
#define MAX                           20

/*---------------------------------------------------------------------------------

---------------------------------------------------------------------------------*/

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
int pass_size; 
int data_count;
int ssid_size;
int status_size;
int temperature;
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
float temp;
 
volatile uint8_t global_HeartRate = 0;
volatile uint8_t global_HeartRate_prev = 0;
volatile uint8_t global_RespirationRate=0;
volatile uint8_t global_RespirationRate_prev = 0;
volatile uint8_t npeakflag = 0;
volatile long time_count=0;
volatile long hist_time_count=0;
volatile bool histgrm_ready_flag = false;
volatile unsigned int RR;

uint8_t ecg_data_buff[20];
uint8_t resp_data_buff[2];
uint8_t ppg_data_buff[20];
uint8_t Work_Mode = BLE_MODE;
uint8_t lead_flag = 0x04;
uint8_t data_len = 20;
uint8_t heartbeat,sp02,respirationrate;
uint8_t histgrm_percent_bin[HISTGRM_DATA_SIZE/4];
uint8_t hr_percent_count = 0;
uint8_t hrv_array[20];

uint16_t ecg_stream_cnt = 0;
uint16_t resp_stream_cnt = 0;
uint16_t ppg_stream_cnt = 0;
uint16_t ppg_wave_ir;

int16_t ecg_wave_sample,ecg_filterout;
int16_t res_wave_sample,resp_filterout;

uint32_t hr_histgrm[HISTGRM_DATA_SIZE];

bool deviceConnected    = false;
bool oldDeviceConnected = false;
bool temp_data_ready    = false;
bool spo2_calc_done     = false;
bool ecg_buf_ready      = false;
bool resp_buf_ready     = false;
bool ppg_buf_ready      = false;
bool hrv_ready_flag     = false;
bool mode_write_flag    = false;
bool processing_intrpt  = false;
bool success_flag       = false;
bool STA_mode_indication= false;
bool bat_data_ready     = false;
bool leadoff_detected   = true;
bool startup_flag       = true;

char DataPacket[30];
char ssid[32];
char password[64];
char modestatus[32];
char tmp_ecgbuf[1200];

String password_to_connect;
String tmp_ecgbu;
String strValue = "";

static int bat_prev=100;
static uint8_t bat_percent = 100;

// PIN numbers are defined as ESP-WROOM-32 IO port number
const int ADS1292_DRDY_PIN  = 26;
const int ADS1292_CS_PIN    = 13;
const int ADS1292_START_PIN = 14;
const int ADS1292_PWDN_PIN  = 27;

const int PUSH_BUTTON_PIN   = 17;

const int AFE4490_CS_PIN    = 21; 
const int AFE4490_DRDY_PIN  = 39; 
const int AFE4490_PWDN_PIN  = 4;

const int LED1_PIN          = 12;
const int LED2_PIN          = 15;
const int SENSOR_VP_PIN     = 36;   //GPIO36, ADC1_CH0

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

const char DataPacketHeader[] = {CES_CMDIF_PKT_START_1, CES_CMDIF_PKT_START_2, CES_CMDIF_DATA_LEN_LSB, CES_CMDIF_DATA_LEN_MSB, CES_CMDIF_TYPE_DATA};
const char DataPacketFooter[] = {CES_CMDIF_PKT_STOP_1, CES_CMDIF_PKT_STOP_2};

BLEServer         *pServer                    = NULL;
BLECharacteristic *Heartrate_Characteristic   = NULL;
BLECharacteristic *sp02_Characteristic        = NULL;
BLECharacteristic *datastream_Characteristic  = NULL;
BLECharacteristic *battery_Characteristic     = NULL;
BLECharacteristic *temperature_Characteristic = NULL;
BLECharacteristic *hist_Characteristic        = NULL;
BLECharacteristic *hrv_Characteristic         = NULL;

ads1292r ADS1292R;   // define class ads1292r
ads1292r_processing ECG_RESPIRATION_ALGORITHM; // define class ecg_algorithm
AFE4490 afe4490;
MAX30205 tempSensor;
spo2_algorithm spo2;
ads1292r_data ads1292r_raw_data;
afe44xx_data afe44xx_raw_data;

class MyServerCallbacks: public BLEServerCallbacks 
{
  void onConnect(BLEServer* pServer)
  {
    deviceConnected = true;
    Serial.println("connected");
  }

  void onDisconnect(BLEServer* pServer)
  {
    deviceConnected = false;
  }
};

class MyCallbackHandler: public BLECharacteristicCallbacks 
{
  void onWrite(BLECharacteristic *datastream_Characteristic)
  {
    std::string value = datastream_Characteristic->getValue();
    int len = value.length();
    strValue = "0";

    if (value.length() > 0) 
    {
      Serial.print("New value: ");

      for (int i = 0; i < value.length(); i++)
      {
        Serial.print(String(value[i]));
        strValue += value[i];
      }

      Serial.println();
    }

  }
};
 
 



void push_button_intr_handler()
{

  if(Work_Mode != WEBSERVER_MODE)
  {
    detachInterrupt(ADS1292_DRDY_PIN);
    mode_write_flag = true;
  }

}

void delLine(fs::FS &fs, const char * path, uint32_t line,const int char_to_delete)
{ 
  File file = fs.open(path, FILE_WRITE);
  
  if(!file)
  {
    Serial.println("- failed to open file for writing");
    return;
  }
  
  uint32_t S= (line-1)*LINELEN;
  file.seek(S);  
  char ch[35]; 

  // build the 'delete line'
  for(uint8_t i=0;i<char_to_delete;i++) 
  {
    ch[i]=' ';
  }

  file.print(ch); // all marked as deleted! yea!
  file.close();
  Serial.println("file closed");
}

void deleteFile(fs::FS &fs, const char * path)
{
  Serial.printf("Deleting file: %s\r\n", path);
  
  if(fs.remove(path))
  {
    Serial.println("- file deleted");
  } 
  else 
  {
    Serial.println("- delete failed");
  }
 
}

bool readFile(fs::FS &fs, const char * path)
{
  Serial.printf("Reading file: %s\r\n", path);
  uint8_t rd_config = 0;
  File file = fs.open(path, FILE_READ);
  
  if(!file || file.isDirectory())
  {
    Serial.println("- failed to open file for reading");
    return false;
  }

  Serial.println("- read from file:");
  rd_config = file.read();
  Serial.println(rd_config);
 
  if(rd_config == 0x0f)
  {
    Work_Mode = WEBSERVER_MODE;
    delLine(SPIFFS,"/v4_mode.txt",1,5);
  }
  else
  {
    return false;
  }

  file.close();
  Serial.println("file closed");
  return true;
}

bool fileread(fs::FS &fs, const char * path)
{
  Serial.printf("Reading file: %s\r\n", path);
  uint8_t md_config = 0;
  File file = fs.open(path, FILE_READ);
  
  if(!file || file.isDirectory())
  {
    Serial.println("- failed to open file for reading");
    return false;
  }

  Serial.println("- read from file:");
  md_config = file.read();
  Serial.println(md_config);
 
  if(md_config == 0x0a)
  {
    Work_Mode = WEBSERVER_MODE;
    delLine(SPIFFS,"/web_mode.txt",1,5);
  }
  else if(md_config == 0x0b)
  {
    Work_Mode = WEBSERVER_MODE;
    delLine(SPIFFS,"/web_mode.txt",1,5);
  }
  else if(md_config == 0x0c)
  {
    Work_Mode = WEBSERVER_MODE;
    delLine(SPIFFS,"/web_mode.txt",1,5);
  }
  else
  {
    return false;
  }

  file.close();
  Serial.println("file closed");
  return true;
}

void writeFile(fs::FS &fs, const char * path, const char * message)
{
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, FILE_WRITE);

  if(!file)
  {
    Serial.println("- failed to open file for writing");
    return;
  }
  
  if(file.print(message))
  {
    Serial.println("- file written");
  } 
  else 
  {
    Serial.println("- frite failed");
  }

  file.close();
  Serial.println("file closed");
}

void readFile(fs::FS &fs, const char * path, int* data_count, char* file_data)
{
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, FILE_READ);
  
  if(!file || file.isDirectory())
  {
    Serial.println("- failed to open file for reading");    
  }

  Serial.println("- read from file:"); 
  int i=0;
  
  while (file.available())
  {
    file_data[i++] = file.read();
    Serial.write(file_data[i-1]);
  }

  *data_count = file.size();
  file.close();
  Serial.println("file closed");
}

void BLE_Init()
{
  BLEDevice::init("Healthypi v4"); // Create the BLE Device
  pServer = BLEDevice::createServer();   // Create the BLE Server
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *HeartrateService = pServer->createService(Heartrate_SERVICE_UUID);   // Create the BLE Service
  BLEService *sp02Service = pServer->createService(sp02_SERVICE_UUID);   // Create the BLE Service
  BLEService *TemperatureService = pServer->createService(TEMP_SERVICE_UUID);
  BLEService *batteryService = pServer->createService(BATTERY_SERVICE_UUID);
  BLEService *hrvService = pServer->createService(HRV_SERVICE_UUID);
  BLEService *datastreamService = pServer->createService(DATASTREAM_SERVICE_UUID);

  Heartrate_Characteristic = HeartrateService->createCharacteristic(
                              Heartrate_CHARACTERISTIC_UUID,
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_WRITE  |
                              BLECharacteristic::PROPERTY_NOTIFY
                              );

  sp02_Characteristic = sp02Service->createCharacteristic(
                        sp02_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_WRITE  |
                        BLECharacteristic::PROPERTY_NOTIFY
                        );

  temperature_Characteristic = TemperatureService->createCharacteristic(
                                TEMP_CHARACTERISTIC_UUID,
                                BLECharacteristic::PROPERTY_READ   |
                                BLECharacteristic::PROPERTY_WRITE  |
                                BLECharacteristic::PROPERTY_NOTIFY
                                );
                                                                      
  battery_Characteristic = batteryService->createCharacteristic(
                            BATTERY_CHARACTERISTIC_UUID,
                            BLECharacteristic::PROPERTY_READ   |
                            BLECharacteristic::PROPERTY_WRITE  |
                            BLECharacteristic::PROPERTY_NOTIFY
                            );

  hrv_Characteristic = hrvService->createCharacteristic(
                        HRV_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_WRITE  |
                        BLECharacteristic::PROPERTY_NOTIFY
                        );

  hist_Characteristic = hrvService->createCharacteristic(
                          HIST_CHARACTERISTIC_UUID,
                          BLECharacteristic::PROPERTY_READ   |
                          BLECharacteristic::PROPERTY_WRITE  |
                          BLECharacteristic::PROPERTY_NOTIFY
                          );
 
  datastream_Characteristic = datastreamService->createCharacteristic(
                              DATASTREAM_CHARACTERISTIC_UUID,
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_WRITE  |
                              BLECharacteristic::PROPERTY_NOTIFY 
                              );
                
  Heartrate_Characteristic->addDescriptor(new BLE2902());
  sp02_Characteristic->addDescriptor(new BLE2902());
  temperature_Characteristic->addDescriptor(new BLE2902());
  battery_Characteristic->addDescriptor(new BLE2902());
  hist_Characteristic->addDescriptor(new BLE2902());
  hrv_Characteristic->addDescriptor(new BLE2902());
  datastream_Characteristic->addDescriptor(new BLE2902());
  datastream_Characteristic->setCallbacks(new MyCallbackHandler()); 

  // Start the service
  HeartrateService->start();
  sp02Service->start();
  TemperatureService->start();
  batteryService->start();
  hrvService->start();
  datastreamService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(Heartrate_SERVICE_UUID);
  pAdvertising->addServiceUUID(sp02_SERVICE_UUID);
  pAdvertising->addServiceUUID(TEMP_SERVICE_UUID);
  pAdvertising->addServiceUUID(BATTERY_SERVICE_UUID);
  pAdvertising->addServiceUUID(HRV_SERVICE_UUID);
  pAdvertising->addServiceUUID(DATASTREAM_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x00);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  ble_advertising(); 
  Serial.println("Waiting a client connection to notify...");
}

void read_battery_value()
{
  static int adc_val = analogRead(SENSOR_VP_PIN);
  battery += adc_val;
  
  if(bat_count == 9)
  {         
    battery = (battery/10);
    battery=((battery*2)-400);
    
    if (battery > 4100)
    {
      battery = 4100;
    }
    else if(battery < 3600 )
    {
      battery = 3600;
    }

    if (startup_flag == true)
    {
      bat_prev = battery;
      startup_flag = false;
    }

    bt_rem = (battery % 100); 

    if(bt_rem>80 && bt_rem < 99 && (bat_prev != 0))
    {
      battery = bat_prev;
    }

    if((battery/100)>=41)
    {
      battery = 100;
    }
    else if((battery/100)==40)
    {
      battery = 80;
    }
    else if((battery/100)==39)
    {
      battery = 60;
    }
    else if((battery/100)==38)
    {
      battery=45;
    }
    else if((battery/100)==37)
    {
      battery=30;
    }
    else if((battery/100)<=36)
    {
      battery = 20;
    }

    bat_percent = (uint8_t) battery;
    bat_count=0;
    battery=0;
    bat_data_ready = true;
  }
  else
  {
    bat_count++;
  }

}
 
void add_hr_histgrm(uint8_t hr)
{
  uint8_t index = hr/10;
  hr_histgrm[index-4]++;
  uint32_t sum = 0;

  if(hr_percent_count++ > HISTGRM_CALC_TH)
  {
    hr_percent_count = 0;
    
    for(int i = 0; i < HISTGRM_DATA_SIZE; i++)
    {
      sum += hr_histgrm[i];
    }

    if(sum != 0)
    {

      for(int i = 0; i < HISTGRM_DATA_SIZE/4; i++)
      {
        uint32_t percent = ((hr_histgrm[i] * 100) / sum);
        histgrm_percent_bin[i] = percent;
      }

    }
    
    histgrm_ready_flag = true;
  }

}

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
  count = 0;
  sqsum = 0;

  for(int i=0;i<(MAX-2);i++)
  {
    pnn50[i]= abs(array[i+1] - array[i]);
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
  sqsum = 0;

  for(int i=0;i<(MAX-2);i++)
  {
    pnn50[i]= abs(array[i+1] - array[i]);
    sqsum = sqsum + (pnn50[i]*pnn50[i]);
  }

  rmssd = sqrt(sqsum/(MAX-1));
  return rmssd;
}

//Led_indications
void ble_advertising()
{

  while((deviceConnected==false)&&(mode_write_flag==false))
  {
    digitalWrite(LED2_PIN, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a 100ms
    digitalWrite(LED2_PIN, HIGH);    // turn the LED off by making the voltage LOW
    delay(3000);
  }

}

void restart_indication()
{
  digitalWrite(LED2_PIN, LOW);
  delay(2500);
  digitalWrite(LED2_PIN, HIGH);
  delay(2500);
}
 
 

void handle_ble_stack()
{

  if(strValue == "\0")
  {
    
    if(ecg_buf_ready)
    {
      ecg_buf_ready = false;
      datastream_Characteristic->setValue(ecg_data_buff, 18);    
      datastream_Characteristic->notify();
    }

  }
  else if(strValue =="0spo2")
  {
    
    if(ppg_buf_ready)
    {
      ppg_buf_ready = false;
      datastream_Characteristic->setValue(ppg_data_buff, 18);    
      datastream_Characteristic->notify();
    }

  }
 
  //send notifications if connected to a client
  if(global_HeartRate_prev != global_HeartRate)
  {
    global_HeartRate_prev = global_HeartRate;
    uint8_t hr_att_ble[2];
    hr_att_ble[0] = lead_flag;
    hr_att_ble[1] = (uint8_t)global_HeartRate;    
    Heartrate_Characteristic->setValue(hr_att_ble, 2);
    Heartrate_Characteristic->notify(); 
  }  
    
  if(spo2_calc_done)
  {
    // afe44xx_raw_data.buffer_count_overflow = false;
    uint8_t spo2_att_ble[5];
    spo2_att_ble[0] = 0x00;
    spo2_att_ble[1] = (uint8_t)sp02;
    spo2_att_ble[2] = (uint8_t)(sp02>>8);
    spo2_att_ble[3] = 0;
    spo2_att_ble[4] = 0;
    sp02_Characteristic->setValue(spo2_att_ble, 5);     
    sp02_Characteristic->notify();        
    spo2_calc_done = false;
  }

  if(hrv_ready_flag)
  {
    hrv_Characteristic->setValue(hrv_array, 13);
    hrv_Characteristic->notify(); 
    hrv_ready_flag = false;
  }
    
  if(temp_data_ready)
  {
    temperature_Characteristic->setValue((uint8_t *)&temperature, 2);
    temperature_Characteristic->notify();
    temp_data_ready = false;
  }
  
  if(histgrm_ready_flag )
  {
    histgrm_ready_flag = false;
    hist_Characteristic->setValue(histgrm_percent_bin, 13);
    hist_Characteristic->notify();
  }
 
  if(bat_data_ready)
  {
    battery_Characteristic->setValue((uint8_t *)&bat_percent, 1);
    battery_Characteristic->notify();
    bat_data_ready = false;
  }

  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    ble_advertising();
    oldDeviceConnected = deviceConnected;
  }
  
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
   // do stuff here on connecting
   oldDeviceConnected = deviceConnected;
  } 

}
 
// https://lastminuteengineers.com/esp32-ota-updates-arduino-ide/
CTS / RTS


void setupBasicOTA() 
{
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) 
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
      {
        type = "filesystem";
        SPIFFS.end();
      }
      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

}
/*---------------------------------------------------------------------------------
The setup() function is called when a sketch starts. Use it to initialize variables, 
pin modes, start using libraries, etc. The setup() function will only run once, 
after each powerup or reset of the  board.
---------------------------------------------------------------------------------*/
void setup()
{
  delay(2000);

  // Make sure serial port on first
  // Setup serial port U0UXD for programming and reset/boot
  Serial.begin  (115200);   // Baudrate for serial communication
  Serial.println("HomeICU is starting...");
  
  setupBasicOTA();
  Serial.println("Basic OTA: On");

  // initalize the  data ready and chip select pins:
  // Pin numbers are defined as ESP-WROOM-32, not as ESP32 processor
  pinMode(ADS1292_DRDY_PIN,   INPUT);  
  pinMode(ADS1292_CS_PIN,     OUTPUT);    
  pinMode(ADS1292_START_PIN,  OUTPUT);
  pinMode(ADS1292_PWDN_PIN,   OUTPUT);  
  pinMode(LED1_PIN,           OUTPUT); 
  pinMode(LED2_PIN,           OUTPUT); 
  pinMode(AFE4490_PWDN_PIN,   OUTPUT);
  pinMode(AFE4490_CS_PIN,     OUTPUT);//Slave Select
  pinMode(AFE4490_DRDY_PIN,   INPUT);// data ready 

  pinMode(PUSH_BUTTON_PIN,    INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_PIN), push_button_intr_handler, FALLING);
  
  Serial.println("SPIFFS initialization ...");

  // initialize SPI file system
  if(!SPIFFS.begin())
  {
    Serial.println("Error!");
    return;
  }
  else
    Serial.println("OK!");
  
  
  restart_indication();
  Work_Mode = BLE_MODE;
  Serial.println("Starts in bluetooth mode");
  BLE_Init();
  
  /*
  ???
  need move Wire out?
  do the SPI config first, and then call .begin()?

  */

  SPI.begin();
  Wire.begin(25,22);
  SPI.setClockDivider (SPI_CLOCK_DIV16);
  SPI.setBitOrder     (MSBFIRST);
  SPI.setDataMode     (SPI_MODE0);
  delay(10);  //delay 10ms
  afe4490.afe44xxInit (AFE4490_CS_PIN,AFE4490_PWDN_PIN);
  delay(10); 
  SPI.setDataMode (SPI_MODE1);          //Set SPI mode as 1
  delay(10);
 
  ADS1292R.ads1292_Init(ADS1292_CS_PIN,ADS1292_PWDN_PIN,ADS1292_START_PIN);  //initalize ADS1292 slave
  delay(10); 
  attachInterrupt(digitalPinToInterrupt(ADS1292_DRDY_PIN),ads1292r_interrupt_handler, FALLING ); // Digital2 is attached to Data ready pin of AFE is interrupt0 in ARduino
 
  tempSensor.begin();
  Serial.println("Initialization is complete");
}
/*---------------------------------------------------------------------------------
After creating a setup() function, which initializes and sets the initial values, 
the loop() function loops consecutively, allowing your program to change and respond.
---------------------------------------------------------------------------------*/
void loop()
{
  boolean ret;

  ArduinoOTA.handle();        // This is for "On The Air" update function 

  ret = ADS1292R.getAds1292r_Data_if_Available(ADS1292_DRDY_PIN,ADS1292_CS_PIN,&ads1292r_raw_data);

  if (ret == true)
  {  
    ecg_wave_sample = (int16_t)(ads1292r_raw_data.raw_ecg >> 8) ;  // ignore the lower 8 bits out of 24bits 
    res_wave_sample = (int16_t)(ads1292r_raw_data.raw_resp>>8) ;
  
    if (!((ads1292r_raw_data.status_reg & 0x1f) == 0))
    {
      leadoff_detected  = true; 
      lead_flag         = 0x04;
      ecg_filterout     = 0;
      resp_filterout    = 0;      
      DataPacket[14]    = 0;
      DataPacket[16]    = 0;
    }  
    else
    {
      leadoff_detected  = false;
      lead_flag = 0x06;

      // filter out the line noise @40Hz cutoff 161 order
      ECG_RESPIRATION_ALGORITHM.Filter_CurrentECG_sample  (&ecg_wave_sample, &ecg_filterout);   
      
      ECG_RESPIRATION_ALGORITHM.Calculate_HeartRate       (ecg_filterout,&global_HeartRate,&npeakflag); // calculate
      
      ECG_RESPIRATION_ALGORITHM.Filter_CurrentRESP_sample (res_wave_sample, &resp_filterout);
      
      ECG_RESPIRATION_ALGORITHM.Calculate_RespRate        (resp_filterout,&global_RespirationRate);   
   
      if(npeakflag == 1)
      {
        read_send_data(global_HeartRate,global_RespirationRate);
        add_hr_histgrm(global_HeartRate);
        npeakflag = 0;
      }
   
      if(Work_Mode == BLE_MODE)
      {
        ecg_data_buff[ecg_stream_cnt++] = (uint8_t)ecg_wave_sample;//ecg_filterout;
        ecg_data_buff[ecg_stream_cnt++] = (ecg_wave_sample>>8);//(ecg_filterout>>8);
      
       if(ecg_stream_cnt >=18)
       {
          ecg_buf_ready = true;
          ecg_stream_cnt = 0;
       }
       
      }
   
      DataPacket[14] = global_RespirationRate;
      DataPacket[16] = global_HeartRate;
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

      spo2_calc_done = true;
      afe44xx_raw_data.buffer_count_overflow = false;
    }
   
    DataPacket[17] = 80;  //bpsys
    DataPacket[18] = 120; //bp dia
    DataPacket[19]=  ads1292r_raw_data.status_reg;  

    SPI.setDataMode (SPI_MODE1);
   
    if ((time_count++ * (1000/SAMPLING_RATE)) > MAX30205_READ_INTERVAL)
    {      
      temp = tempSensor.getTemperature()*100; // read temperature for every 100ms
      temperature =  (uint16_t) temp;
      time_count = 0;
      DataPacket[12] = (uint8_t) temperature; 
      DataPacket[13] = (uint8_t) (temperature >> 8);
      temp_data_ready = true;
      //reading the battery with same interval as temp sensor
      read_battery_value();
    }
  
    if(Work_Mode == BLE_MODE)
    {
      handle_ble_stack();
    }

  }

  if(mode_write_flag)
  {
    mode_write_flag = false;
    const char t = 0x0f;
    writeFile(SPIFFS, "/v4_mode.txt", &t);
    Serial.println("setting webserver mode..\n retsrts in 3 sec"); 
    delay(3000);
    restart_indication();
    ESP.restart();
  }
 
 
  if (success_flag)
  {
    detachInterrupt(ADS1292_DRDY_PIN);
    writeFile(SPIFFS,"/mode_status.txt","datawritten");
    success_flag = false;
    const char u = 0x0a;
    writeFile(SPIFFS,"/web_mode.txt",&u);
    restart_indication();
    ESP.restart();   
  }
 
  if (STA_mode_indication)
  {
    
    for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle=dutyCycle-3)
    {
      // changing the LED brightness with PWM
      ledcWrite(ledChannel, dutyCycle);   
      delay(25);
    }

    STA_mode_indication = false;
  }

}

