/*---------------------------------------------------------------------------------
 Bluetooth Low Energy (BLE)

 Base station read data from the board through BLE

 Reference
 https://randomnerdtutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/
 https://www.arduino.cc/en/Reference/ArduinoBLE
 https://www.arduino.cc/reference/en/libraries/esp32-ble-arduino/

 ArduinoBLE is another lib, but it only support Nordic nRF52840 processor and 
 using SerialHCI to drive the seperated bluetooth card, does not support esp32.

 the maximum size of single data packet determined by MTU size which is 23bytes 
 for BLE 4.0 (20b of data + 3b protocol wrapper). The MTU size is usually set 
 during connection establishment with "MTU Request" command.  

 -> notify() vs. ->indicate()
 notif() is more energy efficient and indicate() is more reliable.
 indicate() will hold the device until it receives acknowledgement or time out.

 flutter_blue (lib for mobile app code) has not implement notify feature yet.
---------------------------------------------------------------------------------*/
#include "firmware.h"
#if BLE_FEATURE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "cppQueue.h"

/*---------------------------------------------------------------------------------
 define uuid 
 
 Alert: These definition value must be same as ble.dart in the Flutter project  
---------------------------------------------------------------------------------*/
#define BLEDeviceName                   "HomeICU" 

#define HeartRate_SERVICE_UUID          (uint16_t(0x180D))
#define HeartRate_CHARACTERISTIC_UUID   (uint16_t(0x2A37))

#define SPO2_SERVICE_UUID               (uint16_t(0x1822)) 
#define SPO2_CHARACTERISTIC_UUID        (uint16_t(0x2A5E))

#define DATASTREAM_SERVICE_UUID         (uint16_t(0x1122)) 
#define ECG_STREAM_CHARACTERISTIC_UUID  (uint16_t(0x1424))
#define PPG_STREAM_CHARACTERISTIC_UUID  (uint16_t(0x1425)) 

#define TEMP_SERVICE_UUID               (uint16_t(0x1809)) 
#define TEMP_CHARACTERISTIC_UUID        (uint16_t(0x2a6e))

#define BATTERY_SERVICE_UUID            (uint16_t(0x180F)) 
#define BATTERY_CHARACTERISTIC_UUID     (uint16_t(0x2a19))

#define HRV_SERVICE_UUID                "cd5c7491-4448-7db8-ae4c-d1da8cba36d0"
#define HRV_CHARACTERISTIC_UUID         "01bfa86f-970f-8d96-d44d-9023c47faddc"
#define HIST_CHARACTERISTIC_UUID        "01bf1525-970f-8d96-d44d-9023c47faddc"

/*---------------------------------------------------------------------------------
 local declarations
---------------------------------------------------------------------------------*/
BLEServer         *pServer                    = NULL;
BLECharacteristic *heartRate_Characteristic   = NULL;
BLECharacteristic *spo2_Characteristic        = NULL;
BLECharacteristic *ecgStream_Characteristic   = NULL;
BLECharacteristic *ppgStream_Characteristic   = NULL;
BLECharacteristic *battery_Characteristic     = NULL;
BLECharacteristic *temp_Characteristic        = NULL;
BLECharacteristic *hist_Characteristic        = NULL;
BLECharacteristic *hrv_Characteristic         = NULL;

volatile bool  bleDeviceConnected = false;
         bool  oldDeviceConnected = false;
/*---------------------------------------------------------------------------------
  PPG queque for BLE transfer

  infrared samples are stored in the FIFO queue, then sent to base station over BLE 
  the queue array size will be automatically resized while push data into it.

  Reference and intallation guide:   
  https://github.com/SMFSW/Queue/

---------------------------------------------------------------------------------*/
Queue	ppg_queue(PPG_QUEUE_LEN, PPG_QUEUE_SIZE, FIFO);
Queue	ecg_queue(ECG_QUEUE_LEN, ECG_QUEUE_SIZE, FIFO);
/*---------------------------------------------------------------------------------
 extern variables
---------------------------------------------------------------------------------*/
extern uint8_t  hrv_array[HVR_ARRAY_SIZE];
extern uint8_t  heart_rate_pack[3];
extern uint8_t  battery_percent, old_battery_percent;
extern union    FloatByte body_temp;
extern float    old_temperature;
extern uint8_t  histogram_percent[HISTGRM_PERCENT_SIZE];
extern uint8_t  LeadStatus;
/*---------------------------------------------------------------------------------

---------------------------------------------------------------------------------*/
class MyServerCallbacks: public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    bleDeviceConnected = true;
    pServer->startAdvertising();
    Serial.println("BLE: connected");
  }

  void onDisconnect(BLEServer *pServer)
  {
    Serial.println("BLE: disconnected");
    bleDeviceConnected = false;
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
      Serial.print("BLE: ecg rec: ");
      for (int i = 0; i < value.length(); i++)
      {
        Serial.print(String(value[i]));
      }
      Serial.println();
    }
  }
  /*void onStatus(BLECharacteristic* pCharacteristic, Status s, uint32_t code)
  {
    switch(s)
    {
      case SUCCESS_INDICATE:        Serial.printf("ecg SUCCESS_INDICATE\r\n");        break;
      case ERROR_INDICATE_DISABLED: Serial.printf("ecg ERROR_INDICATE_DISABLED\r\n"); break;  
      case SUCCESS_NOTIFY:          Serial.printf("ecg SUCCESS_NOTIFY\r\n");          break;
      case ERROR_NOTIFY_DISABLED:   Serial.printf("ecg ERROR_NOTIFY_DISABLED\r\n");   break;
		  case ERROR_NO_CLIENT:         Serial.printf("ecg ERROR_NO_CLIENT\r\n");         break;
      default:                      Serial.printf("ecg other = %d\r\n", s);           break;
    }     
  }*/
};

class ppgCallbackHandler: public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *characteristic)
  {
    std::string value = characteristic->getValue();
    int len = value.length();

    if (value.length() > 0)
    {
      Serial.print("APP->PPG: ");
      for (int i = 0; i < value.length(); i++)
      {
        Serial.print(String(value[i]));
      }
      Serial.println();

      // received "OK" from APP
      if ((value[0]='O')&&(value[0]='K'))
      {
        // re-send everything when re-connect ble
        Serial.println("BLE: re-send");
        old_ecg_heart_rate  = 0xff;
        old_spo2_percent    = 0xff;
        old_temperature     = 0xff;
        old_battery_percent = 0xff;
        hrvDataReady        = true;  
        histogramReady      = true;
        ppg_queue.flush();
        ecg_queue.flush();    
      }
    }
  }

/*void onStatus(BLECharacteristic* pCharacteristic, Status s, uint32_t code)
  {
    return; //disabled
    switch(s)
    {
      case SUCCESS_INDICATE:        Serial.printf("SUCCESS_INDICATE\r\n");        break;
      case ERROR_INDICATE_DISABLED: Serial.printf("ERROR_INDICATE_DISABLED\r\n"); break;  
      case SUCCESS_NOTIFY:          Serial.printf("SUCCESS_NOTIFY\r\n");          break;
      case ERROR_NOTIFY_DISABLED:   Serial.printf("ERROR_NOTIFY_DISABLED\r\n");   break;
		  case ERROR_NO_CLIENT:         Serial.printf("ERROR_NO_CLIENT\r\n");         break;
      default:                      Serial.printf("other = %d\r\n", s);           break;
    }     
  }*/
};
/*---------------------------------------------------------------------------------
 called in the loop()
---------------------------------------------------------------------------------*/
void handleBLE(void)
{ 
  // characteristic value can be up to 512 bytes long. 
  // bluetooth stack can be congestion, if too many packets are sent, add delay() 
  #define ecg_tx_size 10
  #define ppg_tx_size  5

  static uint16_t ecg_serial_number = 0;
  static uint16_t ppg_serial_number = 0;

  // the last byte is serial number of the tx package
  uint16_t ecg_tx_data[ecg_tx_size+1];   
   int16_t ppg_tx_data[ppg_tx_size+1];
  
  // disconnecting
  if (!bleDeviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = bleDeviceConnected;
  }
  // connecting
  if (bleDeviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = bleDeviceConnected;
  }

  // only tx when ble app is connected
  if (!bleDeviceConnected) 
    return;         

  ////////////////////////////////////////////
  // send to BLE
  ////////////////////////////////////////////

  //heart rate
  #define HEART_BEAT_READ_INTERVAL  1000
  static unsigned long heartBeatTimer = 0;

  if (millis() - heartBeatTimer > HEART_BEAT_READ_INTERVAL)
  { 
    heartBeatTimer = millis();     

    //when ECG lead off, use PPG heart rate replace
    if ((LeadStatus==0)&&(ppg_heart_rate!=0))  
      ecg_heart_rate = ppg_heart_rate; 

    if(old_ecg_heart_rate!= ecg_heart_rate)
    {
      heart_rate_pack[0]  = ecg_heart_rate; // calculated by QRS_Algorithm_Interface()
      heart_rate_pack[1]  = ppg_heart_rate; 
      heart_rate_pack[2]  = ecg_lead_off; 
      old_ecg_heart_rate  = ecg_heart_rate;
      heartRate_Characteristic->setValue(&heart_rate_pack[0], sizeof(heart_rate_pack));
      heartRate_Characteristic->notify();
      delay(3);
      Serial.println("ble:send heart");
    }  
  }
  

  //spo2 percentage
  if (old_spo2_percent!= spo2_percent) { 
    old_spo2_percent = spo2_percent;
    spo2_Characteristic->setValue(&spo2_percent, sizeof(spo2_percent));
    spo2_Characteristic->notify();
    delay(3);
    Serial.println("ble:send spo2");
  }

  //body temperature
  if (old_temperature != body_temp.f){
    old_temperature  = body_temp.f;
    temp_Characteristic->setValue(body_temp.b, sizeof(body_temp.b));
    temp_Characteristic->notify();
    delay(3);
    Serial.println("ble:send temp");
  }  
   
  //battery life
  if (old_battery_percent != battery_percent){
    old_battery_percent  = battery_percent;
    battery_Characteristic->setValue(&battery_percent, sizeof(battery_percent));
    battery_Characteristic->notify();
    delay(3);
    Serial.println("ble:send battery");
  }  
  
  //heart rate variability
  if (hrvDataReady){
    hrv_Characteristic->setValue(&hrv_array[0], sizeof(hrv_array));
    hrv_Characteristic->notify();
    hrvDataReady = false;
    delay(3);
    Serial.println("ble:send hrv");
  }

  //heart rate histogram
  if (histogramReady){
    hist_Characteristic->setValue(&histogram_percent[0],sizeof(histogram_percent));
    hist_Characteristic->notify();
    histogramReady = false;
    delay(3);
    Serial.println("ble:send hist");
  }

  // ECG
  while (ecg_queue.getCount()>=ecg_tx_size){
    for (int i = 0; i < ecg_tx_size; i++)
      ecg_queue.pop(&ecg_tx_data[i]);
    ecg_tx_data[ecg_tx_size] = ecg_serial_number++;

    ecgStream_Characteristic->setValue((uint8_t *)ecg_tx_data, sizeof(ecg_tx_data));
    ecgStream_Characteristic->notify();
    delay(3);
  }

  // PPG
  while (ppg_queue.getCount()>=ppg_tx_size){
    for (int i = 0; i < ppg_tx_size; i++)
      ppg_queue.pop(&ppg_tx_data[i]);
    ppg_tx_data[ppg_tx_size] = ppg_serial_number++;

    ppgStream_Characteristic->setValue((uint8_t *)ppg_tx_data, sizeof(ppg_tx_data));
    ppgStream_Characteristic->notify();
    delay(3);
  }

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
  BLEService *tempService       = pServer->createService(TEMP_SERVICE_UUID);
  BLEService *batteryService    = pServer->createService(BATTERY_SERVICE_UUID);
  BLEService *hrvService        = pServer->createService(HRV_SERVICE_UUID);
  BLEService *datastreamService = pServer->createService(DATASTREAM_SERVICE_UUID);

  // add the characteristic to the service 
  // the indicate feature is not working on flutter_blue of the iOS app.
  #define PROPERTY (BLECharacteristic::PROPERTY_READ    | \
                    BLECharacteristic::PROPERTY_WRITE   | \
                    BLECharacteristic::PROPERTY_INDICATE| \
                    BLECharacteristic::PROPERTY_NOTIFY)

  heartRate_Characteristic    = heartRateService->createCharacteristic (HeartRate_CHARACTERISTIC_UUID,PROPERTY);
  spo2_Characteristic         = sp02Service->createCharacteristic      (SPO2_CHARACTERISTIC_UUID,PROPERTY);
  temp_Characteristic         = tempService->createCharacteristic      (TEMP_CHARACTERISTIC_UUID,PROPERTY);
  battery_Characteristic      = batteryService->createCharacteristic   (BATTERY_CHARACTERISTIC_UUID,PROPERTY);
  hrv_Characteristic          = hrvService->createCharacteristic       (HRV_CHARACTERISTIC_UUID,PROPERTY);
  hist_Characteristic         = hrvService->createCharacteristic       (HIST_CHARACTERISTIC_UUID,PROPERTY);
  ecgStream_Characteristic    = datastreamService->createCharacteristic(ECG_STREAM_CHARACTERISTIC_UUID,PROPERTY);
  ppgStream_Characteristic    = datastreamService->createCharacteristic(PPG_STREAM_CHARACTERISTIC_UUID,PROPERTY);

  heartRate_Characteristic  ->addDescriptor(new BLE2902());
  spo2_Characteristic       ->addDescriptor(new BLE2902());
  temp_Characteristic       ->addDescriptor(new BLE2902());
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
  tempService       ->start();
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

#else //BLE_FEATURE
#include "firmware.h"
void initBLE(void){}
void handleBLE(void){}
#endif //BLE_FEATURE