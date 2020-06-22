/*---------------------------------------------------------------------------------
 FIXME
 The code here is NOT my code, it is from healthypi project, 
 Code here requires tidy up and testing!
---------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------- 
 heart rate variability (HRV)
---------------------------------------------------------------------------------*/
#include "ADS1292r.h"
#include "ecg_resp.h"
#include "firmware.h"

extern bool histogramReady;
extern bool hrvDataReady;

#define MAX                           20

unsigned int array[MAX];
int rear = -1;
int sqsum;
int hist[] = {0};
int k=0;
int count = 0;
int min_f=0;
int max_f=0;
int max_t=0;
int min_t=0;
int index_cnt = 0;
int data_count;
int status_size;
int number_of_samples = 0;

float sdnn;
float sdnn_f;
float rmssd;
float mean_f;
float rmssd_f;
float per_pnn;
float pnn_f=0;
float tri =0;

#define HISTGRM_DATA_SIZE             12*4
#define HISTGRM_CALC_TH               10

uint32_t heart_rate_histogram[HISTGRM_DATA_SIZE];

uint8_t respirationRate;
uint8_t histogram_percent_bin[HISTGRM_DATA_SIZE/4];
uint8_t hr_percent_count = 0;
uint8_t hrv_array[20];


volatile unsigned int RR;

int HRVMAX(unsigned int array[])
{  
  for(int i=0;i<MAX;i++)
  {
    if(array[i]>max_t)
      max_t = array[i];
  }
  return max_t;
}

int HRVMIN(unsigned int array[])
{   
  min_t = max_f;
  for(int i=0;i<MAX;i++)
  {
    if(array[i]< min_t)
      min_t = array[i]; 
  }
  return min_t;
}

float mean(unsigned int array[])
{ 
  int sum = 0;
  float mean_rr;
  for(int i=0;i<(MAX);i++)
    sum = sum + array[i];
  mean_rr = (((float)sum)/ MAX);  
  return mean_rr;
} 

float sdnn_ff(unsigned int array[])
{
  int sumsdnn = 0;
  int diff;
 
  for(int i=0;i<(MAX);i++)
  {
    diff = (array[i]-(mean_f));
    diff = diff*diff;
    sumsdnn = sumsdnn + diff;   
  }

  sdnn = (sqrt(sumsdnn/(MAX)));
  return   sdnn;
}

float pnn_ff(unsigned int array[])
{ 
  unsigned int pnn50[MAX];
  long l;

  count = 0;
  sqsum = 0;

  for(int i=0;i<(MAX-2);i++)
  {
    l = array[i+1] - array[i];       //array[] is unsigned integer 0-65535, l = -65535 ~ +65535
    pnn50[i]= abs(l);                //abs() return 0~65535
    sqsum = sqsum + (pnn50[i]*pnn50[i]);

    if(pnn50[i]>50)
      count = count + 1;    
  }
  per_pnn = ((float)count/MAX)*100;
  return per_pnn;
}

float rmssd_ff(unsigned int array[])
{
  unsigned int pnn50[MAX];
  long l;
  sqsum = 0;

  for(int i=0;i<(MAX-2);i++)
  {
    l = array[i+1] - array[i];       //array[] is unsigned integer 0-65535, l = -65535 ~ +65535
    pnn50[i]= abs(l);                //abs() return 0~65535
    sqsum = sqsum + (pnn50[i]*pnn50[i]);
  }

  rmssd = sqrt(sqsum/(MAX-1));
  return rmssd;
}

/*---------------------------------------------------------------------------------

---------------------------------------------------------------------------------*/
uint8_t* read_send_data(uint8_t peakvalue,uint8_t respirationRate)
{  
  int meanval;
  uint16_t sdnn;
  uint16_t pnn;
  uint16_t rmsd;
  RR = peakvalue;
  k++;

  if(rear == MAX-1)
  {
    for(int i=0;i<(MAX-1);i++)
    {
      array[i]=array[i+1];
    }

    array[MAX-1] = RR;   
  }
  else
  {     
    rear++;
    array[rear] = RR;
  }

  if(k>=MAX)
  { 
    max_f = HRVMAX(array);
    min_f = HRVMIN(array);
    mean_f = mean(array);
    sdnn_f = sdnn_ff(array);
    pnn_f = pnn_ff(array);
    rmssd_f=rmssd_ff(array);
  
    meanval = mean_f*100;
    sdnn= sdnn_f*100;
    pnn= pnn_f*100;
    rmsd=rmssd_f*100;

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
    hrvDataReady= true;
  }
}


void add_heart_rate_histogram(uint8_t hr)
{
  uint8_t index = hr/10;
  heart_rate_histogram[index-4]++;
  uint32_t sum = 0;

  if(hr_percent_count++ > HISTGRM_CALC_TH)
  {
    hr_percent_count = 0;
    
    for(int i = 0; i < HISTGRM_DATA_SIZE; i++)
    {
      sum += heart_rate_histogram[i];
    }

    if(sum != 0)
    {
      for(int i = 0; i < HISTGRM_DATA_SIZE/4; i++)
      {
        uint32_t percent = ((heart_rate_histogram[i] * 100) / sum);
        histogram_percent_bin[i] = percent;
      }
    }
    histogramReady = true;
  }
}
