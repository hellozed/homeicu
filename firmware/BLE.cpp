/*---------------------------------------------------------------------------------
 Bluetooth Low Energy (BLE)

 Base station read data from the board through BLE

 Reference
 https://randomnerdtutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/

---------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "firmware.h"
/*---------------------------------------------------------------------------------
 UUID Define 
 
 Alert: These definition value must be same as ble.dart in the Flutter project  
---------------------------------------------------------------------------------*/
#define BLEDeviceName                   "HomeICU" 

#define HeartRate_SERVICE_UUID          (uint16_t(0x180D))
#define HeartRate_CHARACTERISTIC_UUID   (uint16_t(0x2A37))

#define SPO2_SERVICE_UUID               (uint16_t(0x1822)) 
#define SPO2_CHARACTERISTIC_UUID        (uint16_t(0x2A5E))

#define DATASTREAM_SERVICE_UUID         (uint16_t(0x1122)) 
#define ECG_STREAM_CHARACTERISTIC_UUID  (uint16_t(0x1424))
#define PPG_STREAM_CHARACTERISTIC_UUID  (uint16_t(0x1425))  //FIXME config this uuid

#define TEMP_SERVICE_UUID               (uint16_t(0x1809)) 
#define TEMP_CHARACTERISTIC_UUID        (uint16_t(0x2a6e))

#define BATTERY_SERVICE_UUID            (uint16_t(0x180F)) 
#define BATTERY_CHARACTERISTIC_UUID     (uint16_t(0x2a19))

#define HRV_SERVICE_UUID                "cd5c7491-4448-7db8-ae4c-d1da8cba36d0"
#define HRV_CHARACTERISTIC_UUID         "01bfa86f-970f-8d96-d44d-9023c47faddc"
#define HIST_CHARACTERISTIC_UUID        "01bf1525-970f-8d96-d44d-9023c47faddc"

/*---------------------------------------------------------------------------------
 
---------------------------------------------------------------------------------*/

BLEServer         *pServer                    = NULL;
BLECharacteristic *heartRate_Characteristic   = NULL;
BLECharacteristic *SpO2_Characteristic        = NULL;
BLECharacteristic *ecgStream_Characteristic   = NULL;
BLECharacteristic *ppgStream_Characteristic   = NULL;
BLECharacteristic *battery_Characteristic     = NULL;
BLECharacteristic *temperature_Characteristic = NULL;
BLECharacteristic *hist_Characteristic        = NULL;
BLECharacteristic *hrv_Characteristic         = NULL;

/*---------------------------------------------------------------------------------
 FIXME: These global variables must be checked for interrupt access conflict
---------------------------------------------------------------------------------*/
bool deviceConnected    = false;
bool oldDeviceConnected = false;

extern bool     temperatureReady;
extern bool     SpO2_calc_done;
extern bool     ecgBufferReady;
extern bool     ppgBufferReady;
extern bool     hrvDataReady;
extern bool     batteryDataReady;

extern uint8_t  ecg_data_buff[ECG_BUFFER_SIZE];
extern uint8_t  ppg_data_buff[PPG_BUFFER_SIZE];
extern uint8_t  lead_flag;
extern uint8_t  hrv_array[HVR_ARRAY_SIZE];
extern uint8_t  Sp02;
extern float    bodyTemperature;
extern uint8_t  battery_percent;

extern volatile uint8_t heart_rate;
extern volatile uint8_t HeartRate_prev;
extern uint8_t  histogram_percent[HISTGRM_PERCENT_SIZE];
extern bool     histogramReady;
/*---------------------------------------------------------------------------------

---------------------------------------------------------------------------------*/
class MyServerCallbacks: public BLEServerCallbacks 
{
  void onConnect(BLEServer* pServer)
  {
    deviceConnected = true;
    pServer->startAdvertising();
    Serial.println("BLE: connected");
  }

  void onDisconnect(BLEServer* pServer)
  {
    Serial.println("BLE: Disconnect");
    deviceConnected = false;
  }
};

class ecgCallbackHandler: public BLECharacteristicCallbacks 
{
  void onWrite(BLECharacteristic *characteristic) 
  {
    std::string value = characteristic->getValue();
    int len = value.length();

    if (value.length() > 0) 
    {
      Serial.print("BLE: ppg rec: ");
      for (int i = 0; i < value.length(); i++)
      {
        Serial.print(String(value[i]));
      }
      Serial.println();
    }
  }
};
class ppgCallbackHandler: public BLECharacteristicCallbacks 
{
  void onWrite(BLECharacteristic *characteristic) 
  {
    std::string value = characteristic->getValue();
    int len = value.length();

    if (value.length() > 0) 
    {
      Serial.print("BLE: ppg: ");
      for (int i = 0; i < value.length(); i++)
      {
        Serial.print(String(value[i]));
      }
      Serial.println();
    }
  }
};
/*---------------------------------------------------------------------------------

---------------------------------------------------------------------------------*/
void bleSend(bool * readyFlag, uint8_t * data, int length, BLECharacteristic * chr)
{
  if (*readyFlag){
    chr->setValue(data, length);
    chr->notify();
    *readyFlag = false;
  }
}

void handleBLEstack(void)
{
 
  //send notifications if connected to a client
  if(HeartRate_prev != heart_rate)
  {
    bool heart_rate_ready = true;
    uint8_t hr_att_ble[2];
    hr_att_ble[0] = lead_flag;
    hr_att_ble[1] = (uint8_t)heart_rate;    
    bleSend(&heart_rate_ready,hr_att_ble,sizeof(hr_att_ble),heartRate_Characteristic);
    
    HeartRate_prev = heart_rate;
  }  
  
  if(SpO2_calc_done)
  {
    // afe44xx_raw_data.buffer_count_overflow = false;
    uint8_t SpO2_att_ble[5];
    SpO2_att_ble[0] = 0x00;
    SpO2_att_ble[1] = (uint8_t) Sp02;
    SpO2_att_ble[2] = (uint8_t)(Sp02>>8);
    SpO2_att_ble[3] = 0;
    SpO2_att_ble[4] = 0;
    bleSend(&SpO2_calc_done,SpO2_att_ble,sizeof(SpO2_att_ble),SpO2_Characteristic);
  }

  bleSend(&hrvDataReady,    hrv_array,          sizeof(hrv_array),        hrv_Characteristic);
  bleSend(&histogramReady,  histogram_percent,  sizeof(histogram_percent),hist_Characteristic); 
  bleSend(&batteryDataReady,&battery_percent,   sizeof(battery_percent),  battery_Characteristic);  
  bleSend(&temperatureReady,(uint8_t *) &bodyTemperature,sizeof(bodyTemperature),temperature_Characteristic);  

  bleSend(&ecgBufferReady,ecg_data_buff,ECG_BUFFER_SIZE,ecgStream_Characteristic);  //FIXME data length check
  bleSend(&ppgBufferReady,ppg_data_buff,PPG_BUFFER_SIZE,ppgStream_Characteristic); //FIXME data length check
     
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("BLE: start advertising");
    oldDeviceConnected = deviceConnected;
  }
  
  // connecting
  if (deviceConnected && !oldDeviceConnected)
    oldDeviceConnected = deviceConnected;
}
/*---------------------------------------------------------------------------------
 initialize bluetooth 
---------------------------------------------------------------------------------*/
void initBLE(void)
{
  BLEDevice::init(BLEDeviceName);                 // Create Device
  pServer = BLEDevice::createServer();            // Create Server
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  BLEService *heartRateService  = pServer->createService(HeartRate_SERVICE_UUID); 
  BLEService *sp02Service       = pServer->createService(SPO2_SERVICE_UUID);
  BLEService *temperatureService= pServer->createService(TEMP_SERVICE_UUID);
  BLEService *batteryService    = pServer->createService(BATTERY_SERVICE_UUID);
  BLEService *hrvService        = pServer->createService(HRV_SERVICE_UUID);
  BLEService *datastreamService = pServer->createService(DATASTREAM_SERVICE_UUID);

  #define PROPERTY (BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_WRITE|BLECharacteristic::PROPERTY_NOTIFY)

  heartRate_Characteristic    = heartRateService->createCharacteristic(
                                HeartRate_CHARACTERISTIC_UUID,
                                PROPERTY);

  SpO2_Characteristic         = sp02Service->createCharacteristic(
                                SPO2_CHARACTERISTIC_UUID,
                                PROPERTY);

  temperature_Characteristic  = temperatureService->createCharacteristic(
                                TEMP_CHARACTERISTIC_UUID,
                                PROPERTY);
                                                                      
  battery_Characteristic      = batteryService->createCharacteristic(
                                BATTERY_CHARACTERISTIC_UUID,
                                PROPERTY);

  hrv_Characteristic          = hrvService->createCharacteristic(
                                HRV_CHARACTERISTIC_UUID,
                                PROPERTY);

  hist_Characteristic         = hrvService->createCharacteristic(
                                HIST_CHARACTERISTIC_UUID,
                                PROPERTY);
 
  ecgStream_Characteristic    = datastreamService->createCharacteristic(
                                ECG_STREAM_CHARACTERISTIC_UUID,
                                PROPERTY);
  ppgStream_Characteristic    = datastreamService->createCharacteristic(
                                PPG_STREAM_CHARACTERISTIC_UUID,
                                PROPERTY);
                
  heartRate_Characteristic  ->addDescriptor(new BLE2902());
  SpO2_Characteristic       ->addDescriptor(new BLE2902());
  temperature_Characteristic->addDescriptor(new BLE2902());
  battery_Characteristic    ->addDescriptor(new BLE2902());
  hist_Characteristic       ->addDescriptor(new BLE2902());
  hrv_Characteristic        ->addDescriptor(new BLE2902());
  ecgStream_Characteristic  ->addDescriptor(new BLE2902());
  ppgStream_Characteristic  ->addDescriptor(new BLE2902());
  ecgStream_Characteristic  ->setCallbacks (new ecgCallbackHandler());
  ppgStream_Characteristic  ->setCallbacks (new ppgCallbackHandler()); 

  // Start the service
  heartRateService  ->start();
  sp02Service       ->start();
  temperatureService->start();
  batteryService    ->start();
  hrvService        ->start();
  datastreamService ->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(HeartRate_SERVICE_UUID);
  pAdvertising->addServiceUUID(SPO2_SERVICE_UUID);
  pAdvertising->addServiceUUID(TEMP_SERVICE_UUID);
  pAdvertising->addServiceUUID(BATTERY_SERVICE_UUID);
  pAdvertising->addServiceUUID(HRV_SERVICE_UUID);
  pAdvertising->addServiceUUID(DATASTREAM_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x00);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("BLE: adverting");     // wait for connection to notify
}

/*---------------------------------------------------------------------------------
 fake ecg data for testing
---------------------------------------------------------------------------------*/
#if ECG_BLE_TEST
const uint8_t fakeEcgSample[180] = { \
223, 148, 148,  30, 178, 192, 214, 184, 180, 162, \
168, 176, 229, 152, 182, 112, 184, 218, 231, 203, \
185, 168, 177, 181, 225, 170,  99, 184, 201, 228, \
207, 185, 164, 165, 170, 178, 241, 165, 122, 174, \
189, 216, 203, 189, 172, 176, 184, 223, 165, 168, \
104, 190, 222, 225, 200, 189, 194, 194, 246, 189, \
160, 137, 241, 239, 238, 196, 174, 160, 161, 186, \
153, 164,  43, 180, 205, 221, 185, 182, 166, 168, \
178, 228, 170, 155,  40, 192, 205, 225, 185, 170, \
147, 155, 164, 165, 186, 152,  69, 180, 190, 216, \
193, 180, 162, 153, 161, 182, 148, 147,  12, 177, \
194, 207, 168, 165, 146, 156, 160, 216, 142, 160, \
 79, 182, 208, 203, 188, 176, 176, 178, 211, 176, \
177,  76, 211, 245, 222, 204, 189,  39, 169, 181, \
148, 169, 182, 196, 194, 207, 205, 214, 209, 214, \
209,  23, 197, 138, 215, 226, 246, 243, 226, 218, \
190, 208, 211,  17, 203, 209, 151, 236,   3,  14, \
223, 211, 197, 201, 204, 208, 216, 188,  94, 211, \
};
void getTestData(uint8_t *p, int len)
{
  static uint8_t index=0;

  // build the data when the function called at the first time 
  for(int i=0;i<len;i++)
    {
      p[i] = fakeEcgSample[index];
      index++;
      if (index == 180) 
        index = 0;
    }
}
#endif //ECG_BLE_TEST
