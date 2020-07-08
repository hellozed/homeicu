/*---------------------------------------------------------------------------------
  Oximeter -  AFE4490 driver - hardware for SpO2 and PPG


  "AFE4490 and AFE4400 are pin to pin compatible and have similar functionality.
  AFE4400 is cheaper and has a lower performance spec.

  No changes are required for the external circuitry but it requires few firmware 
  modifications to migrate from AFE4490 to AFE4400.

  AFE4400 does not support the following:
  1. Separate gain and bandwidth control
  2. Higher LED current ranges (supports up to 50 mA)
  3. ADC bypass mode"

  Quoted from https://e2e.ti.com/support/data-converters/f/73/t/547701?AFE4490-vs-AFE4400

---------------------------------------------------------------------------------*/
#include "Arduino.h"
#include <SPI.h>
#include <string.h>
#include <math.h>
#include "firmware.h"
extern  void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid);


//afe4490 Register definition
#define CONTROL0      0x00
#define LED2STC       0x01
#define LED2ENDC      0x02
#define LED2LEDSTC    0x03
#define LED2LEDENDC   0x04
#define ALED2STC      0x05
#define ALED2ENDC     0x06
#define LED1STC       0x07
#define LED1ENDC      0x08
#define LED1LEDSTC    0x09
#define LED1LEDENDC   0x0a
#define ALED1STC      0x0b
#define ALED1ENDC     0x0c
#define LED2CONVST    0x0d
#define LED2CONVEND   0x0e
#define ALED2CONVST   0x0f
#define ALED2CONVEND  0x10
#define LED1CONVST    0x11
#define LED1CONVEND   0x12
#define ALED1CONVST   0x13
#define ALED1CONVEND  0x14
#define ADCRSTCNT0    0x15
#define ADCRSTENDCT0  0x16
#define ADCRSTCNT1    0x17
#define ADCRSTENDCT1  0x18
#define ADCRSTCNT2    0x19
#define ADCRSTENDCT2  0x1a
#define ADCRSTCNT3    0x1b
#define ADCRSTENDCT3  0x1c
#define PRPCOUNT      0x1d
#define CONTROL1      0x1e
#define SPARE1        0x1f
#define TIAGAIN       0x20
#define TIA_AMB_GAIN  0x21
#define LEDCNTRL      0x22
#define CONTROL2      0x23
#define SPARE2        0x24
#define SPARE3        0x25
#define SPARE4        0x26
#define SPARE4        0x26
#define RESERVED1     0x27
#define RESERVED2     0x28
#define ALARM         0x29
#define LED2VAL       0x2a
#define ALED2VAL      0x2b
#define LED1VAL       0x2c
#define ALED1VAL      0x2d
#define LED2ABSVAL    0x2e
#define LED1ABSVAL    0x2f
#define DIAG          0x30

volatile int8_t   n_buffer_count; //data length

int dec=0;


static uint32_t irBuffer [100];     //infrared LED sensor data
static uint32_t redBuffer[100];     //red LED sensor data


class AFE4490  afe4490;

void AFE4490 :: getData(void)
{
  signed   long afe4490_IR_data, afe4490_RED_data;
  unsigned long IRtemp,REDtemp;

  if (spo2.interrupt_flag == false) 
    return;   // continue wait for data ready pin interrupt
  else 
    spo2.clear_interrupt();
  
  // interrupt captured, process the data

  SPI.setDataMode(SPI_MODE0); 
  

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
    irBuffer [n_buffer_count] = (uint32_t) ((afe4490_IR_data ) >> 4);
    redBuffer[n_buffer_count] = (uint32_t) ((afe4490_RED_data) >> 4);
    n_buffer_count++;
    dec = 0;
  }
  dec++;

  //////////////////////////
  // save SPO2 to BLE buffer
  //////////////////////////
  if (n_buffer_count > 99)
  {
    int32_t spo2_value;
    int8_t ch_spo2_valid;             //if the SPO2 calculation is valid
    int8_t ch_hr_valid;               //if the heart rate calculation is valid

    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, 100, redBuffer, 
      &spo2_value, &ch_spo2_valid, 
      &heart_rate_from_oximeter, &ch_hr_valid);

    n_buffer_count = 0;
    
    if (ch_spo2_valid)
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
  spo2.save_to_ppg_buffer((uint8_t)ppg_wave_ir);
  spo2.save_to_ppg_buffer((uint8_t)ppg_wave_ir>>8);
  }
}

void AFE4490 :: init(void)
{
  SPI.setDataMode(SPI_MODE0); 
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
