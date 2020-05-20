/******************************************************************************
BLE.cpp



******************************************************************************/
#include <Arduino.h>
#include <BLEDevice.h>    //bluetooth low energy
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "BLE.h"
/*---------------------------------------------------------------------------------

---------------------------------------------------------------------------------*/
#define Heartrate_SERVICE_UUID        (uint16_t(0x180D))
#define Heartrate_CHARACTERISTIC_UUID (uint16_t(0x2A37))
#define SPO2_SERVICE_UUID             (uint16_t(0x1822)) 
#define SPO2_CHARACTERISTIC_UUID      (uint16_t(0x2A5E))
#define DATASTREAM_SERVICE_UUID       (uint16_t(0x1122)) 
#define DATASTREAM_CHARACTERISTIC_UUID (uint16_t(0x1424))
#define TEMP_SERVICE_UUID             (uint16_t(0x1809)) 
#define TEMP_CHARACTERISTIC_UUID      (uint16_t(0x2a6e))
#define BATTERY_SERVICE_UUID          (uint16_t(0x180F)) 
#define BATTERY_CHARACTERISTIC_UUID   (uint16_t(0x2a19))
#define HRV_SERVICE_UUID              "cd5c7491-4448-7db8-ae4c-d1da8cba36d0"
#define HRV_CHARACTERISTIC_UUID       "01bfa86f-970f-8d96-d44d-9023c47faddc"
#define HIST_CHARACTERISTIC_UUID      "01bf1525-970f-8d96-d44d-9023c47faddc"


BLEServer         *pServer                    = NULL;
BLECharacteristic *Heartrate_Characteristic   = NULL;
BLECharacteristic *SpO2_Characteristic        = NULL;
BLECharacteristic *datastream_Characteristic  = NULL;
BLECharacteristic *battery_Characteristic     = NULL;
BLECharacteristic *temperature_Characteristic = NULL;
BLECharacteristic *hist_Characteristic        = NULL;
BLECharacteristic *hrv_Characteristic         = NULL;

/*---------------------------------------------------------------------------------
 These global variables must be checked for interrupt access conflict ???
---------------------------------------------------------------------------------*/
extern bool deviceConnected;
extern bool oldDeviceConnected;
extern bool temperature_data_ready;
extern bool SpO2_calc_done;
extern bool ecg_buf_ready;
extern bool ppg_buf_ready;
extern bool hrv_ready_flag;
extern bool battery_data_ready;

extern uint8_t  ecg_data_buff[];
extern uint8_t  ppg_data_buff[];
extern volatile uint8_t heart_rate;
extern volatile uint8_t HeartRate_prev;
extern uint8_t  lead_flag;
extern uint8_t  hrv_array[];
extern uint8_t          histogram_percent_bin[];
extern volatile bool    histogram_ready_flag;
extern uint8_t  heartbeat,sp02,respirationrate;
extern int      temperature;
extern uint8_t  battery_percent;

extern String strValue;
/*---------------------------------------------------------------------------------

---------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------

---------------------------------------------------------------------------------*/
void handle_BLE_stack(void)
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
  if(HeartRate_prev != heart_rate)
  {
    HeartRate_prev = heart_rate;
    uint8_t hr_att_ble[2];
    hr_att_ble[0] = lead_flag;
    hr_att_ble[1] = (uint8_t)heart_rate;    
    Heartrate_Characteristic->setValue(hr_att_ble, 2);
    Heartrate_Characteristic->notify(); 
  }  
    
  if(SpO2_calc_done)
  {
    // afe44xx_raw_data.buffer_count_overflow = false;
    uint8_t SpO2_att_ble[5];
    SpO2_att_ble[0] = 0x00;
    SpO2_att_ble[1] = (uint8_t)sp02;
    SpO2_att_ble[2] = (uint8_t)(sp02>>8);
    SpO2_att_ble[3] = 0;
    SpO2_att_ble[4] = 0;
    SpO2_Characteristic->setValue(SpO2_att_ble, 5);     
    SpO2_Characteristic->notify();        
    SpO2_calc_done = false;
  }

  if(hrv_ready_flag)
  {
    hrv_Characteristic->setValue(hrv_array, 13);
    hrv_Characteristic->notify(); 
    hrv_ready_flag = false;
  }
    
  if(temperature_data_ready)
  {
    temperature_Characteristic->setValue((uint8_t *)&temperature, 2);
    temperature_Characteristic->notify();
    temperature_data_ready = false;
  }
  
  if(histogram_ready_flag )
  {
    histogram_ready_flag = false;
    hist_Characteristic->setValue(histogram_percent_bin, 13);
    hist_Characteristic->notify();
  }
 
  if(battery_data_ready)
  {
    battery_Characteristic->setValue((uint8_t *)&battery_percent, 1);
    battery_Characteristic->notify();
    battery_data_ready = false;
  }

  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
   // do stuff here on connecting
   oldDeviceConnected = deviceConnected;
  } 
}
/*---------------------------------------------------------------------------------
 bluetooth low energy initialization

---------------------------------------------------------------------------------*/
void BLE_Init(void)
{
  BLEDevice::init("HomeICU Device");        // Create the BLE Device
  pServer = BLEDevice::createServer();      // Create the BLE Server
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *HeartrateService  = pServer->createService(Heartrate_SERVICE_UUID); // Create the BLE Service
  BLEService *sp02Service       = pServer->createService(SPO2_SERVICE_UUID);      // Create the BLE Service
  BLEService *TemperatureService= pServer->createService(TEMP_SERVICE_UUID);
  BLEService *batteryService    = pServer->createService(BATTERY_SERVICE_UUID);
  BLEService *hrvService        = pServer->createService(HRV_SERVICE_UUID);
  BLEService *datastreamService = pServer->createService(DATASTREAM_SERVICE_UUID);

  Heartrate_Characteristic      = HeartrateService->createCharacteristic(
                                  Heartrate_CHARACTERISTIC_UUID,
                                  BLECharacteristic::PROPERTY_READ   |
                                  BLECharacteristic::PROPERTY_WRITE  |
                                  BLECharacteristic::PROPERTY_NOTIFY );

  SpO2_Characteristic           = sp02Service->createCharacteristic(
                                  SPO2_CHARACTERISTIC_UUID,
                                  BLECharacteristic::PROPERTY_READ   |
                                  BLECharacteristic::PROPERTY_WRITE  |
                                  BLECharacteristic::PROPERTY_NOTIFY
                                  );

  temperature_Characteristic    = TemperatureService->createCharacteristic(
                                  TEMP_CHARACTERISTIC_UUID,
                                  BLECharacteristic::PROPERTY_READ   |
                                  BLECharacteristic::PROPERTY_WRITE  |
                                  BLECharacteristic::PROPERTY_NOTIFY
                                  );
                                                                      
  battery_Characteristic        = batteryService->createCharacteristic(
                                  BATTERY_CHARACTERISTIC_UUID,
                                  BLECharacteristic::PROPERTY_READ   |
                                  BLECharacteristic::PROPERTY_WRITE  |
                                  BLECharacteristic::PROPERTY_NOTIFY );

  hrv_Characteristic            = hrvService->createCharacteristic(
                                  HRV_CHARACTERISTIC_UUID,
                                  BLECharacteristic::PROPERTY_READ   |
                                  BLECharacteristic::PROPERTY_WRITE  |
                                  BLECharacteristic::PROPERTY_NOTIFY );

  hist_Characteristic           = hrvService->createCharacteristic(
                                  HIST_CHARACTERISTIC_UUID,
                                  BLECharacteristic::PROPERTY_READ   |
                                  BLECharacteristic::PROPERTY_WRITE  |
                                  BLECharacteristic::PROPERTY_NOTIFY );
 
  datastream_Characteristic     = datastreamService->createCharacteristic(
                                  DATASTREAM_CHARACTERISTIC_UUID,
                                  BLECharacteristic::PROPERTY_READ   |
                                  BLECharacteristic::PROPERTY_WRITE  |
                                  BLECharacteristic::PROPERTY_NOTIFY );
                
  Heartrate_Characteristic  ->addDescriptor(new BLE2902());
  SpO2_Characteristic       ->addDescriptor(new BLE2902());
  temperature_Characteristic->addDescriptor(new BLE2902());
  battery_Characteristic    ->addDescriptor(new BLE2902());
  hist_Characteristic       ->addDescriptor(new BLE2902());
  hrv_Characteristic        ->addDescriptor(new BLE2902());
  datastream_Characteristic ->addDescriptor(new BLE2902());
  datastream_Characteristic ->setCallbacks (new MyCallbackHandler()); 

  // Start the service
  HeartrateService  ->start();
  sp02Service       ->start();
  TemperatureService->start();
  batteryService    ->start();
  hrvService        ->start();
  datastreamService ->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(Heartrate_SERVICE_UUID);
  pAdvertising->addServiceUUID(SPO2_SERVICE_UUID);
  pAdvertising->addServiceUUID(TEMP_SERVICE_UUID);
  pAdvertising->addServiceUUID(BATTERY_SERVICE_UUID);
  pAdvertising->addServiceUUID(HRV_SERVICE_UUID);
  pAdvertising->addServiceUUID(DATASTREAM_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x00);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}