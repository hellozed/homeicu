/*---------------------------------------------------------------------------------
  ADS1292 driver - hardware for ECG
---------------------------------------------------------------------------------*/
#include "Arduino.h"
#include <SPI.h>
#include "ADS1292r.h"
#include "firmware.h" 

volatile bool       ads1292dataReceived = false;
volatile bool       ads1292r_intr_flag  = false;

void IRAM_ATTR ads1292_interrupt_handler(void)
{
  portENTER_CRITICAL_ISR(&ads1292Mux);
  ads1292r_intr_flag = true;
  portEXIT_CRITICAL_ISR (&ads1292Mux);  
}

//FIXME too many delays

void ADS1292 :: init(void)
{
  SPI.setDataMode(SPI_MODE1);

  // start the SPI library:
  pin_high_time(ADS1292_PWDN_PIN,100);
  pin_low_time (ADS1292_PWDN_PIN,100);
  pin_high_time(ADS1292_PWDN_PIN,100);
  
  pin_low_time (ADS1292_START_PIN,20);       // disable Start
  pin_high_time(ADS1292_START_PIN,20);       // enable Start
  pin_low_time (ADS1292_START_PIN,100);      //  hard Stop 
  
  SendCommand(START); // Send 0x08, start data conv
  SendCommand(STOP);  // Send 0x0A, soft stop
  delay(50);
  SendCommand(SDATAC);// Send 0x11, stop read data continuous
  delay(300);
  WriteRegister(ADS1292_REG_CONFIG1,      0x00);  //Set sampling rate to 125 SPS
  WriteRegister(ADS1292_REG_CONFIG2,0b10100000);	//Lead-off comp off, test signal disabled
  WriteRegister(ADS1292_REG_LOFF,   0b00010000);	//Lead-off defaults
  WriteRegister(ADS1292_REG_CH1SET, 0b01000000);	//Ch 1 enabled, gain 6, connected to electrode in
  WriteRegister(ADS1292_REG_CH2SET, 0b01100000);	//Ch 2 enabled, gain 6, connected to electrode in
  WriteRegister(ADS1292_REG_RLDSENS,0b00101100);	//RLD settings: fmod/16, RLD enabled, RLD inputs from Ch2 only
  WriteRegister(ADS1292_REG_LOFFSENS,     0x00);	//LOFF settings: all disabled
  													                  //Skip register 8, LOFF Settings default
  WriteRegister(ADS1292_REG_RESP1,  0b11110010);	//Respiration: MOD/DEMOD turned only, phase 0
  WriteRegister(ADS1292_REG_RESP2,  0b00000011);	//Respiration: Calib OFF, respiration freq defaults

  SendCommand(RDATAC);					              // Send 0x10, Start Read Data Continuous
  delay(10);

  pin_high_time (ADS1292_START_PIN,20);                // enable Start
}

boolean ADS1292 :: getData(ads1292r_data *data_struct)
{
  uint8_t     LeadStatus  = 0;
  signed long secgtemp    = 0;
  long        status_byte = 0;

  unsigned long uecgtemp  = 0;
  unsigned long resultTemp= 0;

  // Sampling rate is set to 125SPS ,DRDY ticks for every 8ms
  if (ads1292r_intr_flag)      
  {
    SPI.setDataMode(SPI_MODE1);

    portENTER_CRITICAL_ISR(&ads1292Mux);
    ads1292r_intr_flag = false;
    portEXIT_CRITICAL_ISR (&ads1292Mux);  

    ReadToBuffer(); // Read the data to SPI_ReadBuffer

    ads1292dataReceived = true;
    uecgtemp = (unsigned long) (((unsigned long)SPI_ReadBuffer[3] << 16)|  \
                                ((unsigned long)SPI_ReadBuffer[4] << 8) |  \
                                 (unsigned long)SPI_ReadBuffer[5]);
    uecgtemp = (unsigned long)(uecgtemp << 8);
    secgtemp = (signed long)  (uecgtemp);
    data_struct->raw_resp    = secgtemp;

    uecgtemp = (unsigned long)(((unsigned  long)SPI_ReadBuffer[6] << 16) | \
                                ((unsigned long)SPI_ReadBuffer[7] <<  8) | \
                                 (unsigned long)SPI_ReadBuffer[8]);
    
    uecgtemp = (unsigned long)(uecgtemp << 8);
    secgtemp = (signed long)  (uecgtemp);
    secgtemp = (signed long)  (secgtemp >> 8);
    data_struct->raw_ecg     = secgtemp;

    status_byte = (long)((long)SPI_ReadBuffer[2] |      \ 
                        ((long)SPI_ReadBuffer[1]) <<8 | \
                        ((long) SPI_ReadBuffer[0])<<16); // First 3 bytes represents the status
    
    status_byte  = (status_byte & 0x0f8000) >> 15;  // bit15 gives the lead status
    LeadStatus = (uint8_t ) status_byte ;
    data_struct->status_reg = LeadStatus;
    ads1292dataReceived = false;
    return true;
  }
  else
    return false;
}

void ADS1292 :: ReadToBuffer(void)
{
  pin_low_time (ADS1292_CS_PIN,0);

  for (int i = 0; i < 9; ++i)
    SPI_ReadBuffer[i] = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
  
  pin_high_time(ADS1292_CS_PIN,0);
}
 
void ADS1292 :: SendCommand(uint8_t data_in)
{
  pin_low_time (ADS1292_CS_PIN,2);
  pin_high_time(ADS1292_CS_PIN,2);
  pin_low_time (ADS1292_CS_PIN,2);

  SPI.transfer(data_in);
  
  pin_high_time(ADS1292_CS_PIN,2);
}

void ADS1292 :: WriteRegister(uint8_t READ_WRITE_ADDRESS, uint8_t DATA)
{
  switch (READ_WRITE_ADDRESS)
  {
    case 1:
            DATA = DATA & 0x87;
	          break;
    case 2:
            DATA = DATA & 0xFB;
	          DATA |= 0x80;
	          break;
    case 3:
      	    DATA = DATA & 0xFD;
      	    DATA |= 0x10;
      	    break;
    case 7:
      	    DATA = DATA & 0x3F;
      	    break;
    case 8:
    	      DATA = DATA & 0x5F;
	          break;
    case 9:
      	    DATA |= 0x02;
      	    break;
    case 10:
      	    DATA = DATA & 0x87;
      	    DATA |= 0x01;
      	    break;
    case 11:
      	    DATA = DATA & 0x0F;
      	    break;
    default:
            break;
  }
  pin_low_time (ADS1292_CS_PIN,2);
  pin_high_time(ADS1292_CS_PIN,2);

  pin_low_time (ADS1292_CS_PIN,2);    // select the device

  SPI.transfer(uint8_t (READ_WRITE_ADDRESS | WREG)); //Send register location
  SPI.transfer(0x00);		    //number of register to wr
  SPI.transfer(DATA);		    //Send value to record into register
  delay(2);

  pin_high_time (ADS1292_CS_PIN,10);  // de-select the device
}

void ADS1292 :: pin_high_time(int pin, uint32_t ms)
{
  digitalWrite(pin, HIGH);
  delay(ms);
}

void ADS1292 :: pin_low_time(int pin, uint32_t ms)
{
  digitalWrite(pin, LOW);
  delay(ms);
}