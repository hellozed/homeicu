/*---------------------------------------------------------------------------------
 Bluetooth Low Energy (BLE)

 Base station read data from the board through BLE
---------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <BLEDevice.h>
//#include <BLEServer.h>
//#include <BLEUtils.h>
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
#define DATASTREAM_CHARACTERISTIC_UUID  (uint16_t(0x1424))

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
BLECharacteristic *datastream_Characteristic  = NULL;
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

extern uint8_t  ecg_data_buff[];
extern uint8_t  ppg_data_buff[];
extern uint8_t  lead_flag;
extern uint8_t  hrv_array[];
extern uint8_t  Sp02;
extern uint8_t  bodyTemperature;
extern uint8_t  battery_percent;

extern volatile uint8_t heart_rate;
extern volatile uint8_t HeartRate_prev;
extern uint8_t  histogram_percent_bin[];
extern bool     histogramReady;

extern String strValue;
/*---------------------------------------------------------------------------------

---------------------------------------------------------------------------------*/
class MyServerCallbacks: public BLEServerCallbacks 
{
  void onConnect(BLEServer* pServer)
  {
    deviceConnected = true;
    Serial.println("BLE: connected");
  }

  void onDisconnect(BLEServer* pServer)
  {
    Serial.println("BLE: Disconnect");
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
      Serial.print("BLE: New value: ");
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
void bleSend(bool * readyFlag, uint8_t * data, int lentgh, BLECharacteristic * chr)
{
  if (*readyFlag){
    chr->setValue(data, lentgh);
    chr->notify();
    *readyFlag = false;
  }
}

void handleBLEstack(void)
{
  if(strValue == "\0")
    bleSend(&ecgBufferReady,ecg_data_buff,18,datastream_Characteristic);
  else if(strValue =="0spo2")
    bleSend(&ppgBufferReady,ppg_data_buff,18,datastream_Characteristic);
 
  //send notifications if connected to a client
  if(HeartRate_prev != heart_rate)
  {
    bool heart_rate_ready = true;
    uint8_t hr_att_ble[2];
    hr_att_ble[0] = lead_flag;
    hr_att_ble[1] = (uint8_t)heart_rate;    
    bleSend(&heart_rate_ready,hr_att_ble,2,heartRate_Characteristic);
    
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
    bleSend(&SpO2_calc_done,SpO2_att_ble,5,SpO2_Characteristic);
  }

  bleSend(&hrvDataReady,hrv_array,13,hrv_Characteristic);
  bleSend(&temperatureReady,&bodyTemperature,2,temperature_Characteristic);  
  bleSend(&histogramReady,histogram_percent_bin,13,hist_Characteristic);  
  bleSend(&batteryDataReady,&battery_percent,1,battery_Characteristic);  
 
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
 
  datastream_Characteristic   = datastreamService->createCharacteristic(
                                DATASTREAM_CHARACTERISTIC_UUID,
                                PROPERTY);
                
  heartRate_Characteristic  ->addDescriptor(new BLE2902());
  SpO2_Characteristic       ->addDescriptor(new BLE2902());
  temperature_Characteristic->addDescriptor(new BLE2902());
  battery_Characteristic    ->addDescriptor(new BLE2902());
  hist_Characteristic       ->addDescriptor(new BLE2902());
  hrv_Characteristic        ->addDescriptor(new BLE2902());
  datastream_Characteristic ->addDescriptor(new BLE2902());
  datastream_Characteristic ->setCallbacks (new MyCallbackHandler()); 

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