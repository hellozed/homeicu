/*
The ADS1292R channel 1 with respiration enabled mode cannot be used to acquire ECG signals. If the right arm
(RA) and left arm (LA) leads are intended to measure respiration and ECG signals, the two leads can be wired
into channel 1 for respiration and channel 2 for ECG signals, as shown in
*/
#if 0
#include <SPI.h>
#define SPI_DUMMY_DATA   0xFF

#define TEMPERATURE 0
#define FILTERORDER         161
/* DC Removal Numerator Coeff*/
#define NRCOEFF (0.992)
#define WAVE_SIZE  1

//******* ecg filter *********
#define MAX_PEAK_TO_SEARCH         5
#define MAXIMA_SEARCH_WINDOW      25
#define MINIMUM_SKIP_WINDOW       30
#define SAMPLING_RATE             125
#define TWO_SEC_SAMPLES           2 * SAMPLING_RATE
#define QRS_THRESHOLD_FRACTION    0.4
#define TRUE 1
#define FALSE 0

volatile uint8_t  SPI_Dummy_Buff[30];
uint8_t DataPacketHeader[16];
volatile signed long s32DaqVals[8];
uint8_t data_len = 7;
volatile byte SPI_RX_Buff[15] ;
volatile static int SPI_RX_Buff_Count = 0;
volatile char *SPI_RX_Buff_Ptr;
volatile bool ads1292dataReceived = false;
unsigned long uecgtemp = 0;
signed long secgtemp = 0;
int i, j;
 

#define TEMPERATURE 0
#define FILTERORDER         161
/* DC Removal Numerator Coeff*/
#define NRCOEFF (0.992)
#define WAVE_SIZE  1

//******* ecg filter *********
volatile uint8_t  SPI_Dummy_Buff[30];
uint8_t DataPacketHeader[16];
volatile signed long s32DaqVals[8];
uint8_t data_len = 7;
volatile byte SPI_RX_Buff[15] ;
volatile static int SPI_RX_Buff_Count = 0;
volatile char *SPI_RX_Buff_Ptr;
volatile bool ads1292dataReceived = false;
unsigned long u_ecg_temp = 0;
signed long signed_ecg_temp = 0;
int i, j;

int16_t ECG_WorkingBuff[2 * FILTERORDER];
unsigned char Start_Sample_Count_Flag = 0;
unsigned char first_peak_detect = FALSE ;
unsigned int sample_index[MAX_PEAK_TO_SEARCH + 2] ;
uint16_t scaled_result_display[150];
uint8_t indx = 0;

int cnt = 0;
volatile uint8_t flag = 0;


static uint16_t QRS_B4_Buffer_ptr = 0 ;
/*   Variable which holds the threshold value to calculate the maxima */
unsigned int sample_count = 0 ;

/* Variable which will hold the calculated heart rate */
volatile uint16_t QRS_Heart_Rate = 0 ;
int16_t ecg_wave_buff[1], ecg_filterout[1];

volatile uint8_t global_HeartRate = 0;
volatile uint8_t global_RespirationRate = 0;
long status_byte=0;
uint8_t LeadStatus=0;
boolean leadoff_deteted = true;


void loop()
{
  if ((digitalRead(ADS1292_DRDY_PIN)) == LOW)      // Sampling rate is set to 125SPS ,DRDY ticks for every 8ms
  {
    SPI_RX_Buff_Ptr = ADS1292.ads1292_Read_Data(); // Read the data,point the data to a pointer
    for (i = 0; i < 9; i++)
      SPI_RX_Buff[SPI_RX_Buff_Count++] = SPI_RX_Buff_Ptr[i];  // store the result data in array
    ads1292dataReceived = true;
  }

  if (ads1292dataReceived == true)      // process the data
  {
    j = 0;
    for (i = 3; i < 9; i += 3)         // data outputs is (24 status bits + 24 bits Respiration data +  24 bits ECG data)
    {
      u_ecg_temp = (unsigned long) (((unsigned long)SPI_RX_Buff[i + 0] << 16) | 
                                    ((unsigned long)SPI_RX_Buff[i + 1] << 8) |  
                                      (unsigned long) SPI_RX_Buff[i + 2]);
      u_ecg_temp = (unsigned long) (u_ecg_temp << 8);
      signed_ecg_temp = (signed long) (u_ecg_temp);
      signed_ecg_temp = (signed long) (signed_ecg_temp >> 8);
      s32DaqVals[j++] = signed_ecg_temp;  //s32DaqVals[0] is Resp data and s32DaqVals[1] is ECG data
    }

    status_byte = (long)((long)SPI_RX_Buff[2] | ((long) SPI_RX_Buff[1]) <<8 | ((long) SPI_RX_Buff[0])<<16); // First 3 bytes represents the status
    status_byte  = (status_byte & 0x0f8000) >> 15;  // bit15 gives the lead status
    LeadStatus = (unsigned char ) status_byte ;  

    if(!((LeadStatus & 0x1f) == 0 ))
      leadoff_deteted  = true; 
    else
      leadoff_deteted  = false;
    
    ecg_wave_buff[0] = (int16_t)(s32DaqVals[1] >> 8) ;  // ignore the lower 8 bits out of 24bits

    if(leadoff_deteted == false) 
       {
          ECG_ProcessCurrSample(ecg_wave_buff, ecg_filterout);   // filter out the line noise @40Hz cutoff 161 order
          QRS_Algorithm_Interface(ecg_filterout[0]);             // calculate 
       }
    else
    {
      ecg_filterout[0] = 0;
      Serial.println("LEAD OFF ");//wzg
    }

    DataPacketHeader[7] = s32DaqVals[0];            // 4 bytes ECG data
    DataPacketHeader[8] = s32DaqVals[0] >> 8;
    DataPacketHeader[9] = s32DaqVals[0] >> 16;
    DataPacketHeader[10] = s32DaqVals[0] >> 24;
    DataPacketHeader[11] = global_HeartRate ; 

    /*
    for (i = 0; i < 14; i++)
    {
      Serial.write(DataPacketHeader[i]);  // transmit the data over USB
    }
    */

    //wzg
    
    Serial.printf("%u %u %u %u\r\n", 
    DataPacketHeader[7],
    DataPacketHeader[8],
    DataPacketHeader[9],
    DataPacketHeader[10]
    ); 
  }

  ads1292dataReceived = false;
  SPI_RX_Buff_Count = 0;
}
 
#endif //0