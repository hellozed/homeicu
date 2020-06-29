/*---------------------------------------------------------------------------------
  AFE4490 driver - hardware for SpO2 and PPG
---------------------------------------------------------------------------------*/
#include "Arduino.h"
#include <SPI.h>
#include <string.h>
#include <math.h>
#include "AFE4490_Oximeter.h"
#include "spo2_algorithm.h"
#include "firmware.h"

static  int32_t an_x[ BUFFER_SIZE];
static  int32_t an_y[ BUFFER_SIZE];

volatile bool     AFE4490_interrupt_flag  = false;
volatile int8_t   n_buffer_count; //data length

int dec=0;

int32_t spo2_value;               //SPO2 value

int32_t heart_rate_from_afe4490;  //heart rate value

uint16_t aun_ir_buffer [100];     //infrared LED sensor data
uint16_t aun_red_buffer[100];     //red LED sensor data

int8_t ch_spo2_valid;             //if the SPO2 calculation is valid
int8_t ch_hr_valid;               //if the heart rate calculation is valid

spo2_algorithm spo2;

void IRAM_ATTR afe4490_interrupt_handler(void)
{
  portENTER_CRITICAL_ISR(&AFE4490Mux);
  AFE4490_interrupt_flag = true;
  portEXIT_CRITICAL_ISR (&AFE4490Mux);  
}

bool      ppgBufferReady      = false;
uint8_t   ppg_data_buff[PPG_BUFFER_SIZE];
uint16_t  ppg_data_cnt        = 0;
uint8_t   SpO2Level;
bool      SpO2Ready           = false;

void AFE4490 :: getData(void)
{
  signed   long afe4490_IR_data, afe4490_RED_data;
  unsigned long IRtemp,REDtemp;

  if (AFE4490_interrupt_flag == false) 
    return;   // continue wait for data ready pin interrupt
  
  // interrupt captured, process the data

  SPI.setDataMode(SPI_MODE0); 
  portENTER_CRITICAL_ISR(&AFE4490Mux);
  AFE4490_interrupt_flag = false;
  portEXIT_CRITICAL_ISR (&AFE4490Mux);  

  writeData(CONTROL0, 0x000001);
  IRtemp = readData(LED1VAL);

  writeData(CONTROL0, 0x000001);
  REDtemp = readData(LED2VAL);
  
  IRtemp = (unsigned long) (IRtemp << 10);
  afe4490_IR_data = (signed long) (IRtemp);
  afe4490_IR_data = (signed long) ((afe4490_IR_data) >> 10);

  REDtemp = (unsigned long) (REDtemp << 10);
  afe4490_RED_data = (signed long) (REDtemp);
  afe4490_RED_data = (signed long) ((afe4490_RED_data) >> 10);

  if (dec == 20)
  {
    aun_ir_buffer [n_buffer_count] = (uint16_t) ((afe4490_IR_data ) >> 4);
    aun_red_buffer[n_buffer_count] = (uint16_t) ((afe4490_RED_data) >> 4);
    n_buffer_count++;
    dec = 0;
  }
  dec++;

  //////////////////////////
  // save SPO2 to BLE buffer
  //////////////////////////
  if (n_buffer_count > 99)
  {
    spo2.estimate_spo2(aun_ir_buffer, 100, aun_red_buffer, &spo2_value, &ch_spo2_valid, &heart_rate_from_afe4490, &ch_hr_valid);
    n_buffer_count = 0;
    
    if (spo2_value == -999)
      SpO2Level = 0;
    else
    {
      SpO2Level = (uint8_t)spo2_value;       
      SpO2Ready = true;
    }
  }

  //////////////////////////
  // save PPG in BLE buffer
  //////////////////////////
  {
  uint16_t  ppg_wave_ir;
  ppg_wave_ir = (uint16_t)(afe4490_IR_data>>8);  
  ppg_data_buff[ppg_data_cnt++] = (uint8_t)ppg_wave_ir;
  ppg_data_buff[ppg_data_cnt++] = (ppg_wave_ir>>8);
  }

  if(ppg_data_cnt >=PPG_BUFFER_SIZE)
  {
    ppgBufferReady = true;
    ppg_data_cnt = 0;
  }
}

void AFE4490 :: init(void)
{
  SPI.setDataMode(SPI_MODE0); 
  digitalWrite(AFE4490_PWDN_PIN, LOW);
  delay(500);
  digitalWrite(AFE4490_PWDN_PIN, HIGH);
  delay(500);
  writeData(CONTROL0,     0x000000);
  writeData(CONTROL0,     0x000008);
  writeData(TIAGAIN,      0x000000); // CF = 5pF, RF = 500kR
  writeData(TIA_AMB_GAIN, 0x000001);
  writeData(LEDCNTRL,     0x001414);
  writeData(CONTROL2,     0x000000); // LED_RANGE=100mA, LED=50mA
  writeData(CONTROL1,     0x010707); // Timers ON, average 3 samples
  writeData(PRPCOUNT,     0X001F3F);
  writeData(LED2STC,      0X001770);
  writeData(LED2ENDC,     0X001F3E);
  writeData(LED2LEDSTC,   0X001770);
  writeData(LED2LEDENDC,  0X001F3F);
  writeData(ALED2STC,     0X000000);
  writeData(ALED2ENDC,    0X0007CE);
  writeData(LED2CONVST,   0X000002);
  writeData(LED2CONVEND,  0X0007CF);
  writeData(ALED2CONVST,  0X0007D2);
  writeData(ALED2CONVEND, 0X000F9F);
  writeData(LED1STC,      0X0007D0);
  writeData(LED1ENDC,     0X000F9E);
  writeData(LED1LEDSTC,   0X0007D0);
  writeData(LED1LEDENDC,  0X000F9F);
  writeData(ALED1STC,     0X000FA0);
  writeData(ALED1ENDC,    0X00176E);
  writeData(LED1CONVST,   0X000FA2);
  writeData(LED1CONVEND,  0X00176F);
  writeData(ALED1CONVST,  0X001772);
  writeData(ALED1CONVEND, 0X001F3F);
  writeData(ADCRSTCNT0,   0X000000);
  writeData(ADCRSTENDCT0, 0X000000);
  writeData(ADCRSTCNT1,   0X0007D0);
  writeData(ADCRSTENDCT1, 0X0007D0);
  writeData(ADCRSTCNT2,   0X000FA0);
  writeData(ADCRSTENDCT2, 0X000FA0);
  writeData(ADCRSTCNT3,   0X001770);
  writeData(ADCRSTENDCT3, 0X001770);
  delay(1000);
}

void AFE4490 :: writeData (uint8_t address, uint32_t data)
{
  digitalWrite (AFE4490_CS_PIN, LOW);     // enable device
  SPI.transfer (address);                 // send address to device
  SPI.transfer ((data >> 16) & 0xFF);     // write top 8 bits
  SPI.transfer ((data >> 8)  & 0xFF);     // write middle 8 bits
  SPI.transfer (data & 0xFF);             // write bottom 8 bits
  digitalWrite (AFE4490_CS_PIN, HIGH);    // disable device
}
 
unsigned long AFE4490 :: readData (uint8_t address)
{
  unsigned long data = 0;
  digitalWrite (AFE4490_CS_PIN, LOW);     // enable device
  SPI.transfer (address);                 // send address to device
  data |= ((unsigned long)SPI.transfer (0) << 16); // read top 8 bits data
  data |= ((unsigned long)SPI.transfer (0) << 8); // read middle 8 bits  data
  data |= SPI.transfer (0);               // read bottom 8 bits data
  digitalWrite (AFE4490_CS_PIN, HIGH);    // disable device
  return data;                            // return with 24 bits of read data
}

void AFE4490 :: simulateData(void)
{
  if (ppgBufferReady == false )     // refill the data
  {
    pinMode(JOYX_PIN, INPUT);  
    for(int i=0;i<PPG_BUFFER_SIZE;i++)
      ppg_data_buff[i] = analogRead(JOYX_PIN);
    ppgBufferReady = true; 
 }
}