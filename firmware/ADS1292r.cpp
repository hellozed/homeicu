/*---------------------------------------------------------------------------------
  ADS1292 driver - hardware for ECG
---------------------------------------------------------------------------------*/
#include "Arduino.h"
#include <SPI.h>
#include "ADS1292r.h"
#include "ecg_resp.h"
#include "firmware.h" 

#define HISTGRM_CALC_TH               10

uint32_t  heart_rate_histogram[HISTGRM_DATA_SIZE];
uint8_t   histogram_percent   [HISTGRM_PERCENT_SIZE];
uint8_t   respirationRate;
uint8_t   hr_percent_count    = 0;
uint8_t   hrv_array[HVR_ARRAY_SIZE];
bool      ecgBufferReady      = false;
bool      hrvDataReady        = false;
bool      histogramReady      = false;

volatile uint8_t  heart_rate  = 0;
volatile uint8_t  npeakflag   = 0;
volatile uint8_t  HeartRate_prev = 0;
volatile uint8_t  RespirationRate=0;
volatile bool     ads1292_intr_flag   = false;

ADS1290Process  ECG_RESPIRATION_ALGORITHM; 
ads1292_data    ads1292_raw_data;

uint8_t   lead_flag = 0;
uint8_t   ecg_data_buff[ECG_BUFFER_SIZE];
int16_t   ecg_wave_sample,  ecg_filterout ;
int16_t   res_wave_sample,  resp_filterout;
uint16_t  ecg_stream_cnt = 0;

void IRAM_ATTR ads1292_interrupt_handler(void)
{
  portENTER_CRITICAL_ISR(&ads1292Mux);
  ads1292_intr_flag = true;
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
  WriteRegister(ADS1292_REG_RESP1,  0b11110010);	//Respiration: MOD/DEMOD turned only, phase 0
  WriteRegister(ADS1292_REG_RESP2,  0b00000011);	//Respiration: Calib OFF, respiration freq defaults
  //Skip register 8, LOFF Settings default

  SendCommand(RDATAC);					              // Send 0x10, Start Read Data Continuous
  delay(10);

  pin_high_time (ADS1292_START_PIN,20);                // enable Start
}

void ADS1292 :: getData()
{
  uint8_t     LeadStatus  = 0;
  signed long secgtemp    = 0;
  long        status_byte = 0;

  unsigned long uecgtemp  = 0;
  unsigned long resultTemp= 0;

  // Sampling rate is set to 125SPS ,DRDY ticks for every 8ms
  if (ads1292_intr_flag==false)
    return;   //wait data to be ready

  // processing the data
  SPI.setDataMode(SPI_MODE1);

  portENTER_CRITICAL_ISR(&ads1292Mux);
  ads1292_intr_flag = false;
  portEXIT_CRITICAL_ISR (&ads1292Mux);  

  ReadToBuffer(); // Read the data to SPI_ReadBuffer

  uecgtemp = (unsigned long) (((unsigned long)SPI_ReadBuffer[3] << 16)|  \
                              ((unsigned long)SPI_ReadBuffer[4] << 8) |  \
                                (unsigned long)SPI_ReadBuffer[5]);
  uecgtemp = (unsigned long)(uecgtemp << 8);
  secgtemp = (signed long)  (uecgtemp);
  ads1292_raw_data.raw_resp    = secgtemp;

  uecgtemp = (unsigned long)(((unsigned  long)SPI_ReadBuffer[6] << 16) | \
                              ((unsigned long)SPI_ReadBuffer[7] <<  8) | \
                                (unsigned long)SPI_ReadBuffer[8]);
  
  uecgtemp = (unsigned long)(uecgtemp << 8);
  secgtemp = (signed long)  (uecgtemp);
  secgtemp = (signed long)  (secgtemp >> 8);
  ads1292_raw_data.raw_ecg     = secgtemp;

  status_byte = (long)((long)SPI_ReadBuffer[2] |      \ 
                      ((long)SPI_ReadBuffer[1]) <<8 | \
                      ((long) SPI_ReadBuffer[0])<<16); // First 3 bytes represents the status
  
  status_byte  = (status_byte & 0x0f8000) >> 15;  // bit15 gives the lead status
  LeadStatus = (uint8_t ) status_byte ;
  ads1292_raw_data.status_reg = LeadStatus;

  /////////////////////////////////////
  // ignore the lower 8 bits out of 24bits 
  ecg_wave_sample = (int16_t)(ads1292_raw_data.raw_ecg  >> 8);  
  res_wave_sample = (int16_t)(ads1292_raw_data.raw_resp >> 8);

  if (!((ads1292_raw_data.status_reg & 0x1f) == 0))
  { // measure lead is OFF the body 
    lead_flag         = 0;
    ecg_filterout     = 0;
    resp_filterout    = 0;      
  }  
  else
  { // the measure lead is ON the body 
    lead_flag         = 1;
    // filter out the line noise @40Hz cutoff 161 order
    ECG_RESPIRATION_ALGORITHM.Filter_CurrentECG_sample  (&ecg_wave_sample,&ecg_filterout);   
    ECG_RESPIRATION_ALGORITHM.Calculate_HeartRate       (ecg_filterout,   &heart_rate,  &npeakflag); 
    ECG_RESPIRATION_ALGORITHM.Filter_CurrentRESP_sample (res_wave_sample, &resp_filterout);
    ECG_RESPIRATION_ALGORITHM.Calculate_RespRate        (resp_filterout,  &RespirationRate);   
    if(npeakflag == 1)
    {
      fillTxBuffer(heart_rate, RespirationRate);
      add_heart_rate_histogram(heart_rate);
      npeakflag = 0;
    }
  
    ecg_data_buff[ecg_stream_cnt++] = (uint8_t)ecg_wave_sample; //ecg_filterout;
    ecg_data_buff[ecg_stream_cnt++] = (ecg_wave_sample>>8);     //ecg_filterout>>8;
    
    if(ecg_stream_cnt >=ECG_BUFFER_SIZE)
    {
      ecgBufferReady = true;
      ecg_stream_cnt = 0;
    }
  }
  /////////////////////////////////////
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
/*--------------------------------------------------------------------------------- 
 heart rate variability (HRV)
---------------------------------------------------------------------------------*/
#define ARRAY_SIZE                           20

uint8_t* ADS1292 :: fillTxBuffer(uint8_t peakvalue,uint8_t respirationRate)
{  
  unsigned int array[ARRAY_SIZE];
  int rear = -1;
  int sqsum;
  int k=0;
  int count = 0;
  int min_f=0;
  int max_f=0;
  int max_t=0;
  int min_t=0;
  volatile unsigned int RR;

  int meanval;

  float sdnn_f;
  float mean_f;
  float rmssd_f;
  float pnn_f;

  uint16_t sdnn;
  uint16_t pnn;
  uint16_t rmsd;
  RR = peakvalue;
  k++;

  if(rear == ARRAY_SIZE-1)
  {
    for(int i=0;i<(ARRAY_SIZE-1);i++)
      array[i]=array[i+1];
    array[ARRAY_SIZE-1] = RR;   
  }
  else
  {     
    rear++;
    array[rear] = RR;
  }

  if(k>=ARRAY_SIZE)
  { 
    // 1. HRVMAX
    for(int i=0;i<ARRAY_SIZE;i++)
    {
      if(array[i]>max_t)
        max_t = array[i];
    }
    max_f = max_t;

    // 2. HRVMIN
    min_t = max_f;
    for(int i=0;i<ARRAY_SIZE;i++)
    {
      if(array[i]< min_t)
        min_t = array[i]; 
    }
    min_f = min_t;

    // 3. mean
    { 
      int sum = 0;
      float mean_rr;
      for(int i=0;i<(ARRAY_SIZE);i++)
        sum = sum + array[i];
      mean_rr = (((float)sum)/ ARRAY_SIZE);  
      mean_f = mean_rr;
    } 

    // 3. sdnn_ff
    {
      int sumsdnn = 0;
      int diff;
      for(int i=0;i<(ARRAY_SIZE);i++)
      {
        diff = (array[i]-(mean_f));
        diff = diff*diff;
        sumsdnn = sumsdnn + diff;   
      }
      sdnn_f = sqrt(sumsdnn/(ARRAY_SIZE));
    }

    // 4. pnn_ff
    { 
      unsigned int pnn50[ARRAY_SIZE];
      long l;

      count = 0;
      sqsum = 0;

      for(int i=0;i<(ARRAY_SIZE-2);i++)
      {
        l = array[i+1] - array[i];       //array[] is unsigned integer 0-65535, l = -65535 ~ +65535
        pnn50[i]= abs(l);                //abs() return 0~65535
        sqsum = sqsum + (pnn50[i]*pnn50[i]);

        if(pnn50[i]>50)
          count = count + 1;    
      }
      pnn_f = ((float)count/ARRAY_SIZE)*100;

    // 5. rmssd_ff
      rmssd_f = sqrt(sqsum/(ARRAY_SIZE-1));
    }
    ////////////////////////////////////////

    meanval = mean_f *100;
    sdnn    = sdnn_f *100;
    pnn     = pnn_f  *100;
    rmsd    = rmssd_f*100;

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
    hrv_array[12]=respirationRate;
    hrvDataReady = true;
  }
}

void ADS1292 :: add_heart_rate_histogram(uint8_t hr)
{
  uint8_t   index = hr/10;
  uint32_t  sum   = 0;

  heart_rate_histogram[index-4]++;

  if(hr_percent_count++ > HISTGRM_CALC_TH)
  {
    hr_percent_count = 0;

    for(int i = 0; i < HISTGRM_DATA_SIZE; i++)
      sum += heart_rate_histogram[i];

    if(sum != 0)
    {
      for(int i = 0; i < HISTGRM_DATA_SIZE/4; i++)
      {
        uint32_t percent = ((heart_rate_histogram[i] * 100) / sum);
        histogram_percent[i] = percent;
      }
    }
    histogramReady = true;
  }
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

void ADS1292 :: getTestData(void)
{
  static uint8_t index=0;
  if (ecgBufferReady == false )     // refill the data
  {
    // build the data when the function called at the first time 
    for(int i=0;i<ECG_BUFFER_SIZE;i++)
      {
        ecg_data_buff[i] = fakeEcgSample[index];
        index++;
        if (index == 180) 
          index = 0;
      }
    ecgBufferReady = true; 
 }
}
#endif //ECG_BLE_TEST