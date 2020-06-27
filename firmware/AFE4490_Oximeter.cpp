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

volatile bool     AFE4490_intr_flag  = false;
volatile boolean  afe44xx_data_ready = false;
volatile int8_t   n_buffer_count; //data length

int dec=0;

unsigned long IRtemp,REDtemp;

int32_t n_spo2;               //SPO2 value
int32_t n_heart_rate;         //heart rate value

uint16_t aun_ir_buffer [100]; //infrared LED sensor data
uint16_t aun_red_buffer[100]; //red LED sensor data

int8_t ch_spo2_valid;         //indicator to show if the SPO2 calculation is valid
int8_t ch_hr_valid;           //indicator to show if the heart rate calculation is valid

const uint8_t uch_spo2_table[184]={ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
                                    99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                                   100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
                                    97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
                                    90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
                                    80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
                                    66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
                                    49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
                                    28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
                                    3,   2,  1  } ;

spo2_algorithm spo2;

void IRAM_ATTR afe4490_interrupt_handler(void)
{
  portENTER_CRITICAL_ISR(&AFE4490Mux);
  AFE4490_intr_flag = true;
  portEXIT_CRITICAL_ISR (&AFE4490Mux);  
}

boolean AFE4490 :: getData(afe44xx_data *afe44xx_raw_data)
{
  if (AFE4490_intr_flag) 
  {
    SPI.setDataMode(SPI_MODE0); 
    portENTER_CRITICAL_ISR(&AFE4490Mux);
    AFE4490_intr_flag = false;
    portEXIT_CRITICAL_ISR (&AFE4490Mux);  

    writeData(CONTROL0, 0x000001);
    IRtemp = readData(LED1VAL);
    writeData(CONTROL0, 0x000001);
    REDtemp = readData(LED2VAL);
    afe44xx_data_ready = true;
    IRtemp = (unsigned long) (IRtemp << 10);
    afe44xx_raw_data->IR_data = (signed long) (IRtemp);
    afe44xx_raw_data->IR_data = (signed long) ((afe44xx_raw_data->IR_data) >> 10);
    REDtemp = (unsigned long) (REDtemp << 10);
    afe44xx_raw_data->RED_data = (signed long) (REDtemp);
    afe44xx_raw_data->RED_data = (signed long) ((afe44xx_raw_data->RED_data) >> 10);

    if (dec == 20)
    {
      aun_ir_buffer[n_buffer_count] = (uint16_t) ((afe44xx_raw_data->IR_data) >> 4);
      aun_red_buffer[n_buffer_count] = (uint16_t) ((afe44xx_raw_data->RED_data) >> 4);
      n_buffer_count++;
      dec = 0;
    }

    dec++;

    if (n_buffer_count > 99)
    {
      spo2.estimate_spo2(aun_ir_buffer, 100, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
      afe44xx_raw_data->spo2 = n_spo2;
      afe44xx_raw_data->heart_rate = n_heart_rate;
      n_buffer_count = 0;
      afe44xx_raw_data->buffer_count_overflow = true;
    }
    afe44xx_data_ready = false;
    return true;
  }
  else 
    return false;
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
  digitalWrite (AFE4490_CS_PIN, LOW);    // enable device
  SPI.transfer (address);             // send address to device
  SPI.transfer ((data >> 16) & 0xFF); // write top 8 bits
  SPI.transfer ((data >> 8)  & 0xFF); // write middle 8 bits
  SPI.transfer (data & 0xFF);         // write bottom 8 bits
  digitalWrite (AFE4490_CS_PIN, HIGH);   // disable device
}
 
unsigned long AFE4490 :: readData (uint8_t address)
{
  unsigned long data = 0;
  digitalWrite (AFE4490_CS_PIN, LOW);    // enable device
  SPI.transfer (address);             // send address to device
  data |= ((unsigned long)SPI.transfer (0) << 16); // read top 8 bits data
  data |= ((unsigned long)SPI.transfer (0) << 8); // read middle 8 bits  data
  data |= SPI.transfer (0);           // read bottom 8 bits data
  digitalWrite (AFE4490_CS_PIN, HIGH);   // disable device
  return data;                        // return with 24 bits of read data
}
 