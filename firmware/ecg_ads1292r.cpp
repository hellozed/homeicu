/*---------------------------------------------------------------------------------
  ADS1292R driver - hardware for ECG

Inspired by:
Healthy Pi V4 project
https://github.com/ericherman/eeg-mouse
http://www.mccauslandcenter.sc.edu/CRNL/ads1298
https://bois083.wordpress.com/lucid-dreaming-device/version-5-0/ads1292-firmware/


SCLK   |  Device attempts to decode and execute commands every eight serial clocks. 
       |  It is recommended that multiples of 8 SCLKs be presented every serial transfer 
       |  to keep the interface in a normal operating mode.
       |
DRDY   |  low = new data are available, regardless of CS
       |
START  |  keep low if using start opcode. The START pin or the START command is used to 
       |  place the device either in normal data capture mode or pulse data capture mode.
       |
PWDN   |  When PWDN is pulled low, all on-chip circuitry is powered down
       |
CS     |  low = SPI is active, must remain low during communication, then wait 4-5 tCLK 
       |  cycles before returning to high, set CS high then low to reset the communication
RESET  |  low = force a reset, RESET is automatically issued to the digital filter whenever 
       |  registers CONFIG1 and RESP are set to new values with a WREG command
       |
CLKSEL |  internal clock

SPI library
the spi clock divider (setClockDivider) sets relative to the system clock, the frequency of SLK 
relative to the chip frequency. it needs to be at least 32, or the clock is too slow, 64 to be safe

Quote from http://www.mit.edu/~gari/CODE/ECG_lab/ecg_ads1292.ino
----------------------------------------------------------------------

Another solution is to use ADS1293, but ADS1293 and ADS1292R has very little in common on the 
interface and register map definition.

-ADS1292 has digital lead-off detection. ADS1293 has both analog and digital lead-off detection
-ADS1293 can generate Wilson or Goldberger References.
-ADS1293 has a flexible routing switch that can connect any input pin to any channel. 
-ADS1292 has assigned input pins per channel
-ADS1293 has lower power-consumption per channel and has shutdown control bits for individual blocks
-ADS1293’s SPI has a read back feature that can read a specific register
-ADS1293 diagnostics is interrupt driven (alarm pin). ADS1293 polls error flags to detect diagnostic condition
-ADS1293 does not have a respiration channel, ADS1292R can do respiration measurement. 
Quote from "https://e2e.ti.com/support/data-converters/f/73/t/300611?1292-vs-1293"

Other IC for ECG : MAX30001 ~ MAX30004
https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6952.html


The tri-wavelength MAX30101 sensor and bi-wavelength MAX30102 sensor were evaluated on the forehead, in a truly non-obtrusive wearable friendly position. Heart rate, respiration rate and blood oxygen saturation were extracted from both sensors and compared with a FDA/TGA/CE approved photoplethysmography device placed on the finger. All data were captured simultaneously and at rest. The MAX30101 sensor was more accurate in measuring heart rate, blood oxygen saturation and respiration rate compared to the MAX30102. 


---------------------------------------------------------------------------------*/
#include "firmware.h" 
#include <SPI.h>

#define SPI_DUMMY_DATA  0xFF

/*
fCLK = 512 kHz and CLK_DIV = 0 
sample rate = 125 SPS
tCLK = (use 2us) 1775~2170 ns, when DVDD = 3.3V, when CLK_DIV = 0
tMOD = (use 8us) 4 tCLK, when CLK_DIV = 0. 
*/
#define CS_LOW_TIME    1 // tCSSC - CS low to first SCLK, setup time. > 6 ns 
#define CS_HIGH_TIME   1 // tCSH  - CS high pulse, > 2 tCLK (use 4us)
#define PWDN_TIME_LOW  2 // hold > 29 tMOD to power down the device is. (use 250 us)
#define PWDN_TIME_LOW  2 // hold > 29 tMOD to power down the device is. (use 250 us)
#define PWDN_TIME_HIGH 40// tPOR Wait after power-up until reset > 4096 tMOD (32ms)
#define START_TIME     3 // START Pin, wait >4100 tMOD to use (2ms)
 
// Register Read Commands
#define RREG    0x20    //Read n nnnn registers starting at address r rrrr
                        //first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define WREG    0x40    //Write n nnnn registers starting at address r rrrr
                        //first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)
//System Commands
#define START		0x08		// start/restart (synchronize) conversions
#define STOP		0x0A		// stop conversion

// Data Read Commands
#define RDATAC  0x10	  // enable Read Data Continuously mode (power-up default).
#define SDATAC	0x11		// stop   Read Data Continuously mode
#define RDATA   0x12    // read data by command; supports multiple read back.


  // remember to change SAMPLING_RATE value if you change here
const uint8_t register_settings[] = {
//set value      register name        address
  0x73,       // ADS1292_REG_ID			  0x00  read only
  0x00,       // ADS1292_REG_CONFIG1	0x01  set sampling rate to 125 SPS
  0b10100000, // ADS1292_REG_CONFIG2	0x02  lead-off comp off, test signal disabled
  0b00010000, // ADS1292_REG_LOFF		  0x03  lead-off defaults
  0b01000000, // ADS1292_REG_CH1SET		0x04  Ch 1 enabled, gain 6, connected to electrode in
  0b01100000, // ADS1292_REG_CH2SET		0x05  Ch 2 enabled, gain 6, connected to electrode in
  0b00101100, // ADS1292_REG_RLDSENS	0x06  RLD settings: fmod/16, RLD enabled, RLD inputs from Ch2 only
  0x00,       // ADS1292_REG_LOFFSENS 0x07  LOFF settings: all disabled
  0x00,       // ADS1292_REG_LOFFSTAT 0x08  Skip register 8, LOFF Settings default
  0b11110010, // ADS1292_REG_RESP1	  0x09  respiration: MOD/DEMOD turned only, phase 0
  0b00000011, // ADS1292_REG_RESP2	  0x0A  respiration: Calib OFF, respiration freq defaults
  0b00001100  // ADS1292_REG_GPIO     0x0B

/*  
  //TI 
  0x73,       // REG_ID			  0x00  read only
  0x00,       // REG_CONFIG1	0x01  set sampling rate to 125 SPS
  0xE0, // REG_CONFIG2	0x02  lead-off DC comparators ON, test signal disabled
  0xF0, // REG_LOFF		  0x03  lead-off comparator threshold. 000-least responsive, 111-max responsive. 
  0x00, // REG_CH1SET		0x04  Ch 1 enabled, gain 6 , connected to electrode in
  0x00, // REG_CH2SET		0x05  Ch 2 enabled, gain 12, connected to electrode in
  0x2C, // REG_RLDSENS	0x06  RLD settings: fmod/16, RLD enabled, RLD inputs from Ch2 only
  0x0F,       // REG_LOFFSENS 0x07  LOFF settings: all disabled
  0x00,       // REG_LOFFSTAT 0x08  Skip register 8, LOFF Settings default
  0xEA, // REG_RESP1	  0x09  respiration: MOD/DEMOD turned only, phase 0
  0x03, // REG_RESP2	  0x0A  respiration: Calib OFF, respiration freq defaults
  0b00001100  // REG_GPIO     0x0B
*/
/*  
//set value      register name        address
  0x73,       // REG_ID			  0x00  read only
  0x00,       // REG_CONFIG1	0x01  set sampling rate to 125 SPS
  0b10100000, // REG_CONFIG2	0x02  lead-off DC comparators ON, test signal disabled
  0b00010000, // REG_LOFF		  0x03  lead-off comparator threshold. 000-least responsive, 111-max responsive. 
  0b01000000, // REG_CH1SET		0x04  Ch 1 enabled, gain 6 , connected to electrode in
  0b01100000, // REG_CH2SET		0x05  Ch 2 enabled, gain 12, connected to electrode in
  0b00101100, // REG_RLDSENS	0x06  RLD settings: fmod/16, RLD enabled, RLD inputs from Ch2 only
  0x00,       // REG_LOFFSENS 0x07  LOFF settings: all disabled
  0x00,       // REG_LOFFSTAT 0x08  Skip register 8, LOFF Settings default
  0b11110010, // REG_RESP1	  0x09  respiration: MOD/DEMOD turned only, phase 0
  0b00000011, // REG_RESP2	  0x0A  respiration: Calib OFF, respiration freq defaults
  0b00001100  // REG_GPIO     0x0B
*/  
}; 
/*
  0b11000000, // REG_CONFIG2  0x02
  0b00010000, // REG_LOFF		  0x03 
  0b00101100, // REG_RLDSENS	0x06
  0x00,       // REG_LOFFSENS 0x07
*/
#define HISTGRM_CALC_TH        10

uint32_t  heart_rate_histogram[HISTGRM_DATA_SIZE];
uint8_t   histogram_percent   [HISTGRM_PERCENT_SIZE];
uint8_t   hr_percent_count    = 0;
uint8_t   hrv_array[HVR_ARRAY_SIZE];
uint8_t   heart_rate_pack  [3];
bool      ecgBufferReady      = false;
bool      hrvDataReady        = false;
bool      histogramReady      = false;
bool      heartRateReady      = false;

volatile uint8_t    npeakflag   = 0;
volatile uint16_t   heart_rate_prev = 0;
volatile uint8_t    respirationRate = 0;
volatile bool       ads1292r_interrupt_flag   = false;

class ADS1292R
{
public:
  void      init(void);
  void      getData(void);
  void      getTestData(void);
private:
  uint8_t * fillTxBuffer  (uint8_t peakvalue,uint8_t respirationRate);
  void      add_heart_rate_histogram(uint8_t hr);
  uint8_t   mask_register_bits(uint8_t address, uint8_t data_in);
};

ADS1292R    ads1292r;
uint8_t     lead_flag = 0;
uint8_t     ecg_data_buff[ECG_BUFFER_SIZE];
int16_t     ecg_wave_sample,  ecg_filterout;
int16_t     res_wave_sample,  resp_filterout;
uint16_t    ecg_stream_cnt = 0;

void IRAM_ATTR ads1292r_interrupt_handler(void)
{
  portENTER_CRITICAL_ISR(&ads1292rMux);
  ads1292r_interrupt_flag = true;
  portEXIT_CRITICAL_ISR (&ads1292rMux);  
}

void initECG()    {ads1292r.init(); }
void handleECG()  {ads1292r.getData();}  
 
void pin_level_high(uint8_t pin, uint32_t ms)
{
  digitalWrite(pin, HIGH);
  delay(ms);
}

void pin_level_low(uint8_t pin, uint32_t ms)
{
  digitalWrite(pin, LOW);
  delay(ms);
}

void ADS1292R :: init(void)
{
  uint8_t data;
  uint8_t r[12]; 
  int i, address;

  SPI.setDataMode(SPI_MODE1);

  // after power on, wait device boot up
  while (millis()<PWDN_TIME_HIGH)
  {
    Serial.println("wait ads129r bootup...");
    delay(2);
  }

  // reset device
  pin_level_high(ADS1292_CS_PIN,CS_HIGH_TIME); //initial CS

  pin_level_low(ADS1292_PWDN_PIN,PWDN_TIME_LOW);  
  pin_level_high(ADS1292_PWDN_PIN,PWDN_TIME_HIGH);  
  //------------------------------------------------
  // conversion stop, when "START pin is low" OR "receive STOP opcode"
  //------------------------------------------------
  pin_level_low(ADS1292_START_PIN,START_TIME);       // stop 

  //------------------------------------------------
  pin_level_low(ADS1292_CS_PIN,CS_LOW_TIME);
  //------------------------------------------------
                     
  //SPI.transfer(START);  
  //SPI.transfer(STOP);  
  SPI.transfer(SDATAC);         // stop data reading mode before write regiters

  //------------------------------------------------
  // write registers
  //Write n nnnn registers starting @ address r rrrr
  // 010rrrrr, rrrrr = the starting register address.
  uint8_t OPCODE1 = 0x00 | WREG;  
  // 000nnnnn, nnnnn = the number of registers to write – 1
  uint8_t OPCODE2 = sizeof(register_settings) - 1;                   
  
  SPI.transfer(OPCODE1);   
  SPI.transfer(OPCODE2);	
  
  for(i = 0,address = 0x00; i<sizeof(register_settings); address++, i++)	
  {
    data = mask_register_bits(address, register_settings[i]);
    SPI.transfer(data);	
  } 
  //------------------------------------------------
  // verify registers

  //read n nnnn registers starting at address r rrrr  
  // 001r rrrr (rrrrr = the starting register address)
  OPCODE1 = 0x00 | RREG;  
  // 000nnnnn, nnnnn = the number of registers to write – 1
  OPCODE2 = sizeof(r) - 1;                 
  SPI.transfer(OPCODE1);   
  SPI.transfer(OPCODE2);	

  for(i=0; i<sizeof(r);i++)	
  {
    r[i] = SPI.transfer(0x00);
    if (r[i]!=register_settings[i])
    {
      Serial.printf("!! ecg register error @%02x = %02x, expecting %02x\r\n", 
                    i, r[i], register_settings[i]);
      system_init_error++;
    }
  } 
  // start to read data continuously
  SPI.transfer(RDATAC);  
  
  //------------------------------------------------
  pin_level_high(ADS1292_CS_PIN,CS_HIGH_TIME);
  //------------------------------------------------
  
  // Conversions begin, when "START pin is high" OR "START opcode is received"
  pin_level_high (ADS1292_START_PIN,START_TIME);

  Serial.println("ECG config ok.");
}

void ADS1292R :: getData()
{
  uint8_t       LeadStatus;
  static uint8_t old_LeadStatus = 0;
  signed long   signed_24bits_temp;
  unsigned long unsigned_24bits_temp;
  unsigned long resultTemp= 0;

  #define   SPI_BUFFER_SIZE   9
  uint8_t   SPI_ReadBuffer[SPI_BUFFER_SIZE];

  // Sampling rate is set to 125SPS ,DRDY ticks for every 8ms
  if (ads1292r_interrupt_flag==false)
    return;   //wait data to be ready

  portENTER_CRITICAL_ISR(&ads1292rMux);
  ads1292r_interrupt_flag = false;
  portEXIT_CRITICAL_ISR (&ads1292rMux);  

  // processing the data
  SPI.setDataMode(SPI_MODE1);

  // Read the data to SPI_ReadBuffer
  pin_level_low (ADS1292_CS_PIN,CS_LOW_TIME);
  for (int i = 0; i < SPI_BUFFER_SIZE; i++)
    SPI_ReadBuffer[i] = SPI.transfer(SPI_DUMMY_DATA);
  pin_level_high(ADS1292_CS_PIN,CS_HIGH_TIME); 


  unsigned_24bits_temp = (unsigned long)(((unsigned long)SPI_ReadBuffer[3] << 16)|  
                                         ((unsigned long)SPI_ReadBuffer[4] << 8) |  
                                          (unsigned long)SPI_ReadBuffer[5]);
  unsigned_24bits_temp = (unsigned long)(unsigned_24bits_temp << 8);
  signed_24bits_temp   =   (signed long)(unsigned_24bits_temp);
  signed_24bits_temp   =   (signed long)(  signed_24bits_temp >> 8);
  res_wave_sample   = (int16_t)(signed_24bits_temp >> 8); // ignore the lower 8 bits
  

  unsigned_24bits_temp = (unsigned long)(((unsigned long)SPI_ReadBuffer[6] << 16) | 
                                         ((unsigned long)SPI_ReadBuffer[7] <<  8) | 
                                          (unsigned long)SPI_ReadBuffer[8]);
  unsigned_24bits_temp = (unsigned long)(unsigned_24bits_temp << 8);
  signed_24bits_temp   =   (signed long)(unsigned_24bits_temp);
  signed_24bits_temp   = (signed long) (signed_24bits_temp >> 8);

  ecg_wave_sample   = (int16_t)(signed_24bits_temp >> 8); // ignore the lower 8 bits

  /*
   the first 3 bytes is the status word, 24-bit as below:
   1100 + LOFF_STAT[4:0] + GPIO[1:0] + 13 '0's).
   0000 1111 | 1000 0000 |0000 0000 (0x0f8000)
  */
  
  // BIT 4    | BIT 3   | BIT 2   | BIT 1   | BIT 0
  // RLD_STAT | IN2N_OFF| IN2P_OFF| IN1N_OFF| IN1P_OFF
  LeadStatus    = ((SPI_ReadBuffer[0]<<1)|(SPI_ReadBuffer[1]>>7))&0x0F; //ignore RLD lead
LeadStatus=1;//FIXME
/*  
  if(old_LeadStatus!=LeadStatus)
  {
    old_LeadStatus=LeadStatus;
    Serial.printf("%u %u %u %u\r\n", 
                  (LeadStatus & 0b00001000)>>3,
                  (LeadStatus & 0b00000100)>>2,
                  (LeadStatus & 0b00000010)>>1,
                  (LeadStatus & 0b00000001)>>0
                  );
  }

  if (!(LeadStatus == 0))
  { // measure lead is OFF the body 
    lead_flag         = 0;
    ecg_filterout     = 0;
    resp_filterout    = 0;      
  }  
  else */
  { // the measure lead is ON the body 
    lead_flag         = 1;
    // filter out the line noise @40Hz cutoff 161 order
    Filter_CurrentECG_sample  (&ecg_wave_sample,&ecg_filterout);   
    QRS_Algorithm_Interface   (ecg_filterout); 

    Filter_CurrentRESP_sample (res_wave_sample, &resp_filterout);
    Calculate_RespRate        (resp_filterout,  &respirationRate);   
    
    //-------------------------------------------
    // only enable this line, and use Aduino 
    // Serial Plotter to display ecg graphic
    //-------------------------------------------  
    Serial.printf("%d %d\r\n", ecg_filterout, QRS_Heart_Rate-30 );
    //-------------------------------------------
    
    if(heart_rate_prev != QRS_Heart_Rate)
    {
      heart_rate_pack[0] = (uint8_t) QRS_Heart_Rate; // calculated by QRS_Algorithm_Interface()
      heart_rate_pack[1] = (uint8_t) heart_rate_from_oximeter; 
      heart_rate_pack[2] = lead_flag; 
      heart_rate_prev = QRS_Heart_Rate;
    }  

    if(npeakflag == 1)
    {
      fillTxBuffer((uint8_t)QRS_Heart_Rate, respirationRate);
      add_heart_rate_histogram((uint8_t)QRS_Heart_Rate);
      npeakflag = 0;
    }
  
    //FIXME
    ecg_data_buff[ecg_stream_cnt++] = (uint8_t)ecg_filterout; 
    ecg_data_buff[ecg_stream_cnt++] = (ecg_filterout>>8);
    
    if(ecg_stream_cnt >=ECG_BUFFER_SIZE)
    {
      ecgBufferReady = true;
      ecg_stream_cnt = 0;
    }
  }
} 
/*--------------------------------------------------------------------------------- 
 heart rate variability (HRV)
---------------------------------------------------------------------------------*/
#define ARRAY_SIZE                           20

uint8_t* ADS1292R :: fillTxBuffer(uint8_t peakvalue,uint8_t respirationRate)
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

void ADS1292R :: add_heart_rate_histogram(uint8_t hr)
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

uint8_t ADS1292R::mask_register_bits(uint8_t address, uint8_t data_in)
{
  uint8_t data = data_in;

  switch (address)
  {
    case 1:
      data = data & 0x87;
	    break;
    case 2:
      data = data & 0xFB;
	    data |= 0x80;		
	    break;
    case 3:
	    data = data & 0xFD;
	    data |= 0x10;
	    break;
    case 7:
	    data = data & 0x3F;
	    break;
    case 8:
    	data = data & 0x5F;
	    break;
    case 9:
	    data |= 0x02;
	    break;
    case 10:
	    data = data & 0x87;
	    data |= 0x01;
	    break;
    case 11:
	    data = data & 0x0F;
	    break;
    default:
	    break;		
  }
  return data;
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

void ADS1292R :: getTestData(void)
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


void ecg_reg_set(uint8_t address, uint8_t data)
{
  //------------------------------------------------
  pin_level_low(ADS1292_CS_PIN,CS_LOW_TIME);
  SPI.transfer(SDATAC);         // stop data reading mode before write regiters
  //------------------------------------------------
  // write registers
  //Write n nnnn registers starting @ address r rrrr
  // 010rrrrr, rrrrr = the starting register address.
  uint8_t OPCODE1 = address | WREG;  
  // 000nnnnn, nnnnn = the number of registers to write – 1
  uint8_t OPCODE2 = 0x00;                   
  
  SPI.transfer(OPCODE1);   
  SPI.transfer(OPCODE2);	 
  SPI.transfer(data);	
 
  //------------------------------------------------
  // verify registers

  //read n nnnn registers starting at address r rrrr  
  // 001r rrrr (rrrrr = the starting register address)
  OPCODE1 = address | RREG;  
  // 000nnnnn, nnnnn = the number of registers to write – 1
  OPCODE2 = 0x00;        

  SPI.transfer(OPCODE1);   
  SPI.transfer(OPCODE2);	

  if (SPI.transfer(0x00)==data)
    Serial.println("write ok");
  else   
    Serial.println("write error!");
  // start to read data continuously
  SPI.transfer(RDATAC);  
  
  pin_level_high(ADS1292_CS_PIN,CS_HIGH_TIME);
  

  //pin_level_high (ADS1292_START_PIN,START_TIME);
}
