/*---------------------------------------------------------------------------------
 The code here is from healthypi project, 
 Code here requires tidy up and testing!

ECG front-end IC provide 24-bit two-channel ADC,
It can sample at a rate of 125 ~ 8kSPS (samples per second),
the default is 500 sample per second.

The typical ECG detection system internal converters is 12 bits, and 125 SPS.
---------------------------------------------------------------------------------*/
#include "firmware.h"
#include <SPI.h>

static void QRS_process_buffer();
static void QRS_check_sample_crossing_threshold( uint16_t scaled_result);  
static void ECG_FilterProcess(int16_t * WorkingBuff, int16_t * CoeffBuf, int16_t* FilterOut);

#define FILTERORDER         161
/* DC Removal Numerator Coeff*/
#define NRCOEFF           (0.992)

//******* ecg filter *********
#define MAX_PEAK_TO_SEARCH        5
//FIXME check if this number can improve performance
//#define MAXIMA_SEARCH_WINDOW			40
//#define MINIMUM_SKIP_WINDOW				50
  #define MAXIMA_SEARCH_WINDOW      25  
  #define MINIMUM_SKIP_WINDOW       30

#define SAMPLING_RATE             125
#define TWO_SEC_SAMPLES           2 * SAMPLING_RATE

/*Flag which identifies the duration for which sample count has to be incremented*/
static unsigned char Start_Sample_Count_Flag = 0;
static unsigned char first_peak_detect = false ;
/* Variable which will hold the calculated heart rate */
static unsigned int sample_count = 0 ; 
static unsigned int sample_index[MAX_PEAK_TO_SEARCH + 2] ;

/* Variables to hold the sample data for calculating the 1st and 2nd */
/* differentiation                          */
static int QRS_Second_Prev_Sample = 0 ;
static int QRS_Prev_Sample = 0 ;
static int QRS_Current_Sample = 0 ;
static int QRS_Next_Sample = 0 ;
static int QRS_Second_Next_Sample = 0 ;
/*   Variable which holds the threshold value to calculate the maxima*/
static int16_t QRS_Threshold_Old = 0;
static int16_t QRS_Threshold_New = 0;

/* Variable which holds the threshold value to calculate the maxima */
/*  Pointer which points to the index in B4 buffer where the processed data*/
/*  has to be filled */
static uint16_t QRS_B4_Buffer_ptr = 0 ; 

uint8_t Respiration_Rate = 0 ;
static int RESP_Second_Prev_Sample = 0 ;
static int RESP_Prev_Sample = 0 ;
static int RESP_Current_Sample = 0 ;
static int RESP_Next_Sample = 0 ;
static int RESP_Second_Next_Sample = 0 ;
static int16_t RESP_WorkingBuff[2 * FILTERORDER];
static int16_t Pvev_DC_Sample=0, Pvev_Sample=0;

static void Resp_FilterProcess(int16_t * RESP_WorkingBuff, int16_t * CoeffBuf, int16_t* FilterOut);
static void Respiration_Rate_Detection(int16_t Resp_wave,volatile uint8_t *respirationRate);

//************** ecg *******************
int16_t CoeffBuf_40Hz_LowPass[FILTERORDER] =
{
  -72,    122,    -31,    -99,    117,      0,   -121,    105,     34,
  -137,     84,     70,   -146,     55,    104,   -147,     20,    135,
  -137,    -21,    160,   -117,    -64,    177,    -87,   -108,    185,
  -48,   -151,    181,      0,   -188,    164,     54,   -218,    134,
  112,   -238,     90,    171,   -244,     33,    229,   -235,    -36,
  280,   -208,   -115,    322,   -161,   -203,    350,    -92,   -296,
  361,      0,   -391,    348,    117,   -486,    305,    264,   -577,
  225,    445,   -660,     93,    676,   -733,   -119,    991,   -793,
  -480,   1486,   -837,  -1226,   2561,   -865,  -4018,   9438,  20972,
  9438,  -4018,   -865,   2561,  -1226,   -837,   1486,   -480,   -793,
  991,   -119,   -733,    676,     93,   -660,    445,    225,   -577,
  264,    305,   -486,    117,    348,   -391,      0,    361,   -296,
  -92,    350,   -203,   -161,    322,   -115,   -208,    280,    -36,
  -235,    229,     33,   -244,    171,     90,   -238,    112,    134,
  -218,     54,    164,   -188,      0,    181,   -151,    -48,    185,
  -108,    -87,    177,    -64,   -117,    160,    -21,   -137,    135,
  20,   -147,    104,     55,   -146,     70,     84,   -137,     34,
  105,   -121,      0,    117,    -99,    -31,    122,    -72
};

/* Coeff for lowpass Fc=2Hz @ 125 SPS*/
int16_t RespCoeffBuf[FILTERORDER] = 
{ 120,    124,    126,    127,    127,    125,    122,    118,    113,  
  106,     97,     88,     77,     65,     52,     38,     24,      8,
   -8,    -25,    -42,    -59,    -76,    -93,   -110,   -126,   -142,
 -156,   -170,   -183,   -194,   -203,   -211,   -217,   -221,   -223,
 -223,   -220,   -215,   -208,   -198,   -185,   -170,   -152,   -132,
 -108,    -83,    -55,    -24,      8,     43,     80,    119,    159,
  201,    244,    288,    333,    378,    424,    470,    516,    561,
  606,    650,    693,    734,    773,    811,    847,    880,    911,
  939,    964,    986,   1005,   1020,   1033,   1041,   1047,   1049,
 1047,   1041,   1033,   1020,   1005,    986,    964,    939,    911,
  880,    847,    811,    773,    734,    693,    650,    606,    561,
  516,    470,    424,    378,    333,    288,    244,    201,    159,
  119,     80,     43,      8,    -24,    -55,    -83,   -108,   -132,
 -152,   -170,   -185,   -198,   -208,   -215,   -220,   -223,   -223,
 -221,   -217,   -211,   -203,   -194,   -183,   -170,   -156,   -142,
 -126,   -110,    -93,    -76,    -59,    -42,    -25,     -8,      8,
   24,     38,     52,     65,     77,     88,     97,    106,    113,
  118,    122,    125,    127,    127,    126,    124,    120       };

/*********************************************************************************************************
** Function Name : ECG_FilterProcess()                         **
** Description    :                                    **
**         The function process one sample filtering with 161 ORDER             **
**         FIR multiband filter 0.5 t0 150 Hz and 50/60Hz line nose.            **
**         The function supports compile time 50/60 Hz option              **
**                                                **
** Parameters    :                                    **
**         - WorkingBuff    - In - input sample buffer                  **
**         - CoeffBuf      - In - Co-eficients for FIR filter.              **
**         - FilterOut      - Out - Filtered output                **
** Return       : None                                    **
*********************************************************************************************************/
static void ECG_FilterProcess(int16_t * WorkingBuff, int16_t * CoeffBuf, int16_t * FilterOut)
{
  int acc = 0;   // accumulator for MACs
  int  k;
  // perform the multiply-accumulate

  for ( k = 0; k < 161; k++ )
    acc += (int)(*CoeffBuf++) * (int)(*WorkingBuff--);
  // saturate the result

  if ( acc > 0x3fffffff )
    acc = 0x3fffffff;
  else if ( acc < -0x40000000 )
    acc = -0x40000000;
  // convert from Q30 to Q15
  *FilterOut = (int16_t)(acc >> 15);
}
/*********************************************************************************************************
** Function Name : ECG_ProcessCurrSample()                            **
** Description    :                                    **
**         The function process one sample of data at a time and           **
**         which stores the filtered out sample in the Leadinfobuff.            **
**         The function does the following :-                    **
**                                                **
**         - DC Removal of the current sample                    **
**         - Multi band 161 Tab FIR Filter with Notch at 50Hz/60Hz.             **
** Parameters    :                                    **
**         - ECG_WorkingBuff    - In - ECG. input sample buffer                **
**         - FilterOut      - Out - Filtered output                **
** Return       : None                                    **
*********************************************************************************************************/
void Filter_CurrentECG_sample(int16_t CurrAqsSample, int16_t *FilteredOut)
{
  static uint16_t ECG_bufStart = 0, ECG_bufCur = FILTERORDER - 1, ECGFirstFlag = 1;
  static int16_t ECG_Pvev_DC_Sample, ECG_Pvev_Sample;/* Working Buffer Used for Filtering*/
  static int16_t ECG_WorkingBuff[2 * FILTERORDER];
  int16_t *CoeffBuf;
  int16_t temp1, temp2, ECGData;
  /* Count variable*/
  uint16_t Cur_Chan;
  int16_t FiltOut = 0;
  CoeffBuf = CoeffBuf_40Hz_LowPass;   // filter option is 40Hz LowPass (Default)

  if  ( ECGFirstFlag )                // First Time initialize static variables.
  {
    for ( Cur_Chan = 0 ; Cur_Chan < FILTERORDER; Cur_Chan++)
      ECG_WorkingBuff[Cur_Chan] = 0;

    ECG_Pvev_DC_Sample = 0;
    ECG_Pvev_Sample = 0;
    ECGFirstFlag = 0;
  }

  temp1 = NRCOEFF * ECG_Pvev_DC_Sample;       //First order IIR
  ECG_Pvev_DC_Sample = (CurrAqsSample - ECG_Pvev_Sample) + temp1;
  ECG_Pvev_Sample = CurrAqsSample;
  temp2 = ECG_Pvev_DC_Sample >> 2;
  ECGData = (int16_t) temp2;
  /* Store the DC removed value in Working buffer in millivolts range*/
  ECG_WorkingBuff[ECG_bufCur] = ECGData;
  ECG_FilterProcess(&ECG_WorkingBuff[ECG_bufCur], CoeffBuf, (int16_t*)&FiltOut);
  /* Store the DC removed value in ECG_WorkingBuff buffer in millivolts range*/
  ECG_WorkingBuff[ECG_bufStart] = ECGData;
  /* Store the filtered out sample to the LeadInfo buffer*/
  FilteredOut[0] = FiltOut ;//(CurrOut);
  ECG_bufCur++;
  ECG_bufStart++;

  if ( ECG_bufStart  == (FILTERORDER - 1))
  {
    ECG_bufStart = 0;
    ECG_bufCur = FILTERORDER - 1;
  }
}
/*********************************************************************************************************
**                                               **
**   Function Name : QRS_Algorithm_Interface                          **
**   Description -   This function is called by the main acquisition            **
**      thread at every samples read.                **
**             Before calling the process_buffer() the below check             **
**      has to be done. i.e. We have always received +2            **
**      samples before starting the processing  for each           **
**      samples. This function basically checks the                **
**      difference between the current  and  previous ECG          **
**      Samples using 1st & 2nd differentiation calculations.           **
**                                               **
**   Parameters  : - Lead II sample CurrSample                  **
**   Return    : None                                     **
*********************************************************************************************************/
void QRS_Calculate_Heart_Rate(int16_t CurrSample)
{
  static int16_t prev_data[32] = {0};
  int16_t i;
  long Mac = 0;
  prev_data[0] = CurrSample;

  for ( i = 31; i > 0; i--)
  {
    Mac += prev_data[i];
    prev_data[i] = prev_data[i - 1];
  }
  Mac += CurrSample;
  Mac = Mac >> 2;
  CurrSample = (int16_t) Mac;
  QRS_Second_Prev_Sample = QRS_Prev_Sample ;
  QRS_Prev_Sample = QRS_Current_Sample ;
  QRS_Current_Sample = QRS_Next_Sample ;
  QRS_Next_Sample = QRS_Second_Next_Sample ;
  QRS_Second_Next_Sample = CurrSample ;
  QRS_process_buffer();
}
/*********************************************************************************************************
**   Function Name : QRS_check_sample_crossing_threshold()                 **
**   Description -                                     **
**                                                **
**      This function computes duration of QRS peaks using              **
**      order differentiated input sample and computes             **
**      QRS_Current_Sample.After we process the data we can             **
**      Heart rate. It mutes comptation in case of leads off.           **
**                                                **
**   Parameters  - Scaled Result                                 **
**   Global variables - QRS_Heart_Rate_new  and HR_flag                        **
**   Return variables - None                     **
*********************************************************************************************************/
static void QRS_check_sample_crossing_threshold( uint16_t scaled_result)
{
  /* array to hold the sample indexes S1,S2,S3 etc */
  static uint16_t s_array_index = 0 ;
  static uint16_t m_array_index = 0 ;
  static unsigned char threshold_crossed = false ;
  static uint16_t maxima_search = 0 ;
  static unsigned char peak_detected = false ;
  static uint16_t skip_window = 0 ;
  static long maxima_sum = 0 ;
  static unsigned int peak = 0;
  static unsigned int sample_sum = 0;
  static unsigned int nopeak = 0;
  uint16_t Max = 0 ;
  uint16_t HRAvg;

  if ( true == threshold_crossed  )
  {
    /*
    Once the sample value crosses the threshold check for the
    maxima value till MAXIMA_SEARCH_WINDOW samples are received
    */
    sample_count ++ ;
    maxima_search ++ ;

    if ( scaled_result > peak )
      peak = scaled_result ;

    if ( maxima_search >= MAXIMA_SEARCH_WINDOW )
    {
      // Store the maxima values for each peak
      maxima_sum += peak ;
      maxima_search = 0 ;
      threshold_crossed = false ;
      peak_detected = true ;
    }
  }
  else if ( true == peak_detected )
  {
    // Once the sample value goes below the threshold
    // skip the samples untill the SKIP WINDOW criteria is meet
    sample_count ++ ;
    skip_window ++ ;

    if ( skip_window >= MINIMUM_SKIP_WINDOW )
    {
      skip_window = 0 ;
      peak_detected = false ;
    }

    if ( m_array_index == MAX_PEAK_TO_SEARCH )
    {
      sample_sum = sample_sum / (MAX_PEAK_TO_SEARCH - 1);
      HRAvg =  (uint16_t) sample_sum  ;
      // Compute HR without checking LeadOffStatus
      ecg_heart_rate = (uint16_t) 60 *  SAMPLING_RATE;
      ecg_heart_rate =  ecg_heart_rate / HRAvg ;

      if (ecg_heart_rate > 250)
        ecg_heart_rate = 250 ;

      /* Setting the Current HR value in the ECG_Info structure*/
      maxima_sum =  maxima_sum / MAX_PEAK_TO_SEARCH;
      Max = (int16_t) maxima_sum ;
      /*  calculating the new QRS_Threshold based on the maxima obtained in 4 peaks */
      maxima_sum = Max * 7;
      maxima_sum = maxima_sum / 10;
      QRS_Threshold_New = (int16_t)maxima_sum;
      /* Limiting the QRS Threshold to be in the permissible range*/
      if (QRS_Threshold_New > (4 * QRS_Threshold_Old))
        QRS_Threshold_New = QRS_Threshold_Old;

      sample_count = 0 ;
      s_array_index = 0 ;
      m_array_index = 0 ;
      maxima_sum = 0 ;
      sample_index[0] = 0 ;
      sample_index[1] = 0 ;
      sample_index[2] = 0 ;
      sample_index[3] = 0 ;
      Start_Sample_Count_Flag = 0;
      sample_sum = 0;
    }
  }
  else if ( scaled_result > QRS_Threshold_New )
  {
    // if the sample value crosses the threshold then store the sample index
    Start_Sample_Count_Flag = 1;
    sample_count ++ ;
    m_array_index++;
    threshold_crossed = true ;
    peak = scaled_result ;
    nopeak = 0;

    //!!!!!!!!!!!!!!!!!!!!
    npeakflag = 1;
    //!!!!!!!!!!!!!!!!!!!!
    //  storing sample index
    sample_index[ s_array_index ] = sample_count ;
    if ( s_array_index >= 1 )
      sample_sum += sample_index[ s_array_index ] - sample_index[ s_array_index - 1 ] ;
    s_array_index ++ ;
  }
  else if (( scaled_result < QRS_Threshold_New ) && (Start_Sample_Count_Flag == 1))
  {
    sample_count ++ ;
    nopeak++;

    if (nopeak > (3 * SAMPLING_RATE))
    {
      sample_count = 0 ;
      s_array_index = 0 ;
      m_array_index = 0 ;
      maxima_sum = 0 ;
      sample_index[0] = 0 ;
      sample_index[1] = 0 ;
      sample_index[2] = 0 ;
      sample_index[3] = 0 ;
      Start_Sample_Count_Flag = 0;
      peak_detected = false ;
      sample_sum = 0;
      first_peak_detect = false;
      nopeak = 0;
      ecg_heart_rate = 0;
    }
  }
  else
  {
    nopeak++;
    if (nopeak > (3 * SAMPLING_RATE))
    {
      // Reset heart rate computation sate variable 
      // in case of no peak found in 3 seconds 
      sample_count = 0 ;
      s_array_index = 0 ;
      m_array_index = 0 ;
      maxima_sum = 0 ;
      sample_index[0] = 0 ;
      sample_index[1] = 0 ;
      sample_index[2] = 0 ;
      sample_index[3] = 0 ;
      Start_Sample_Count_Flag = 0;
      peak_detected = false ;
      sample_sum = 0;
      first_peak_detect = false;
      nopeak = 0;
      ecg_heart_rate = 0;
    }
  }
}
/*********************************************************************************************************
**   Function Name : QRS_process_buffer()                             **
**   Description -                                     **
**                                                **
**      This function will be doing the first and second           **
**      order differentiation for   input sample,                  **
**      QRS_Current_Sample.After we process the data we can             **
**      fill the   QRS_Proc_Data_Buffer which is the input             **
**      for HR calculation Algorithm. This function is             **
**      called for each n sample.Once we have received 6s of            **
**      processed   data(i.e.Sampling rate*6) in the B4            **
**      buffer we will start the heart rate calculation for             **
**      first time and later we will do heart rate                 **
**      calculations once we receive the defined   number              **
**      of samples for the expected number of refresh              **
**      seconds.                               **
**                                                **
**   Parameters  - Nil                                      **
**   Return     - None                                     **
*********************************************************************************************************/
static void QRS_process_buffer(void)
{
  int16_t first_derivative = 0 ;
  int16_t scaled_result = 0 ;
  static int16_t Max = 0 ;
  /* calculating first derivative*/
  first_derivative = QRS_Next_Sample - QRS_Prev_Sample  ;
  
  /*taking the absolute value*/
  if (first_derivative < 0)
  {
    first_derivative = -(first_derivative);
  }

  scaled_result = first_derivative;

  if ( scaled_result > Max )
    Max = scaled_result ;

  QRS_B4_Buffer_ptr++;

  if (QRS_B4_Buffer_ptr ==  TWO_SEC_SAMPLES)
  {
    QRS_Threshold_Old = ((Max * 7) / 10 ) ;
    QRS_Threshold_New = QRS_Threshold_Old ;
   // if (Max > 70)
      first_peak_detect = true ;
    Max = 0;
    QRS_B4_Buffer_ptr = 0;
  }

  if ( true == first_peak_detect )
    QRS_check_sample_crossing_threshold(scaled_result) ;
}
static void Resp_FilterProcess(int16_t * RESP_WorkingBuff, int16_t * CoeffBuf, int16_t* FilterOut)
{
  int32_t acc=0;     // accumulator for MACs
  int  k;

  // perform the multiply-accumulate
  for ( k = 0; k < 161; k++ )
      acc += (int32_t)(*CoeffBuf++) * (int32_t)(*RESP_WorkingBuff--);

  // saturate the result
  if ( acc > 0x3fffffff )
      acc = 0x3fffffff;
  else if ( acc < -0x40000000 )
      acc = -0x40000000;

  // convert from Q30 to Q15
  *FilterOut = (int16_t)(acc >> 15);
}

void Filter_CurrentRESP_sample(int16_t CurrAqsSample, int16_t * FiltOut)
{
  static uint16_t bufStart=0, bufCur = FILTERORDER-1, FirstFlag = 1;
  int16_t temp1, temp2;//, RESPData;
  int16_t RESPData;
  /* Count variable*/
  uint16_t Cur_Chan;
  
  // int16_t FiltOut;
  temp1 = NRCOEFF * Pvev_DC_Sample;
  Pvev_DC_Sample = (CurrAqsSample  - Pvev_Sample) + temp1;
  Pvev_Sample = CurrAqsSample;
  temp2 = Pvev_DC_Sample;
  RESPData = (int16_t) temp2;
  RESPData = CurrAqsSample;
  /* Store the DC removed value in RESP_WorkingBuff buffer in millivolts range*/
  RESP_WorkingBuff[bufCur] = RESPData;
  Resp_FilterProcess(&RESP_WorkingBuff[bufCur],RespCoeffBuf,FiltOut);
  /* Store the DC removed value in Working buffer in millivolts range*/
  RESP_WorkingBuff[bufStart] = RESPData;
  /* Store the filtered out sample to the LeadInfo buffer*/
  bufCur++;
  bufStart++;

  if ( bufStart  >= (FILTERORDER-1))
  {
    bufStart=0;
    bufCur = FILTERORDER-1;
  }
}

void Calculate_RespRate(int16_t CurrSample,volatile uint8_t *respirationRate)
{
  static int16_t prev_data[64] ={0};
  char i;
  long Mac=0;
  prev_data[0] = CurrSample;

  for ( i=63; i > 0; i--)
  {
    Mac += prev_data[i];
    prev_data[i] = prev_data[i-1];
  }

  Mac += CurrSample;
  CurrSample = (int16_t) Mac >> 1;
  RESP_Second_Prev_Sample = RESP_Prev_Sample ;
  RESP_Prev_Sample = RESP_Current_Sample ;
  RESP_Current_Sample = RESP_Next_Sample ;
  RESP_Next_Sample = RESP_Second_Next_Sample ;
  RESP_Second_Next_Sample = CurrSample;// << 3 ;
  Respiration_Rate_Detection(RESP_Second_Next_Sample,respirationRate);
}

static void Respiration_Rate_Detection(int16_t Resp_wave,volatile uint8_t *respirationRate)
{
  static uint16_t skipCount = 0, SampleCount = 0,TimeCnt=0, SampleCountNtve=0, PtiveCnt =0,NtiveCnt=0 ;
  static int16_t MinThreshold = 0x7FFF, MaxThreshold = 0x8000, PrevSample = 0, PrevPrevSample = 0, PrevPrevPrevSample =0;
  static int16_t MinThresholdNew = 0x7FFF, MaxThresholdNew = 0x8000, AvgThreshold = 0;
  static unsigned char startCalc=0, PtiveEdgeDetected=0, NtiveEdgeDetected=0, peakCount = 0;
  static uint16_t PeakCount[8];
  SampleCount++;
  SampleCountNtve++;
  TimeCnt++;

  if (Resp_wave < MinThresholdNew)
    MinThresholdNew = Resp_wave;

  if (Resp_wave > MaxThresholdNew)
    MaxThresholdNew = Resp_wave;

  if (SampleCount > 1000)
    SampleCount =0;
  
  if (SampleCountNtve > 1000)
    SampleCountNtve =0;

  if ( startCalc == 1)
  {
    if (TimeCnt >= 500)
    {
      TimeCnt =0;

      if ( (MaxThresholdNew - MinThresholdNew) > 400)
      {
        MaxThreshold = MaxThresholdNew;
        MinThreshold =  MinThresholdNew;
        AvgThreshold = MaxThreshold + MinThreshold;
        AvgThreshold = AvgThreshold >> 1;
      }
      else
      {
        startCalc = 0;
        Respiration_Rate = 0;
      }
    }

    PrevPrevPrevSample = PrevPrevSample;
    PrevPrevSample = PrevSample;
    PrevSample = Resp_wave;

    if ( skipCount == 0)
    {
      if (PrevPrevPrevSample < AvgThreshold && Resp_wave > AvgThreshold)
      {
        if ( SampleCount > 40 &&  SampleCount < 700)
        {
          PtiveEdgeDetected = 1;
          PtiveCnt = SampleCount;
          skipCount = 4;
        }
        SampleCount = 0;
      }

      if (PrevPrevPrevSample < AvgThreshold && Resp_wave > AvgThreshold)
      {
        if ( SampleCountNtve > 40 &&  SampleCountNtve < 700)
        {
          NtiveEdgeDetected = 1;
          NtiveCnt = SampleCountNtve;
          skipCount = 4;
        }
        SampleCountNtve = 0;
      }

      if (PtiveEdgeDetected ==1 && NtiveEdgeDetected ==1)
      {
        PtiveEdgeDetected = 0;
        NtiveEdgeDetected =0;

        if (abs(PtiveCnt - NtiveCnt) < 5)
        {
          PeakCount[peakCount++] = PtiveCnt;
          PeakCount[peakCount++] = NtiveCnt;

          if( peakCount == 8)
          {
            peakCount = 0;
            PtiveCnt = PeakCount[0] + PeakCount[1] + PeakCount[2] + PeakCount[3] +
            PeakCount[4] + PeakCount[5] + PeakCount[6] + PeakCount[7];
            PtiveCnt = PtiveCnt >> 3;
            Respiration_Rate = 6000/PtiveCnt; // 60 * 125/SampleCount;
          }
        }
      }
    }
    else
      skipCount--;
  }
  else
  {
    TimeCnt++;
    if (TimeCnt >= 500)
    {
      TimeCnt = 0;
      if ( (MaxThresholdNew - MinThresholdNew) > 400)
      {
        startCalc = 1;
        MaxThreshold = MaxThresholdNew;
        MinThreshold =  MinThresholdNew;
        AvgThreshold = MaxThreshold + MinThreshold;
        AvgThreshold = AvgThreshold >> 1;
        PrevPrevPrevSample = Resp_wave;
        PrevPrevSample = Resp_wave;
        PrevSample = Resp_wave;
      }
    }
  }
  *respirationRate=(uint8_t)Respiration_Rate;
}
