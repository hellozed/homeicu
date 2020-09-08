/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#include <stdlib.h>

/****************************************************************/
/* Constants*/
/****************************************************************/

/*threshold = 0.7 * maxima*/
#define QRS_THRESHOLD_FRACTION	0.7					

#define FILTERORDER 				161

#define TRUE	1
#define FALSE	0

/* DC Removal Numerator Coeff*/
#define NRCOEFF (0.992)

/****************************************************************/
/* Global functions*/
/****************************************************************/

void RESP_Algorithm_Interface(short CurrSample);
void Resp_ProcessCurrSample(short *CurrAqsSample, short *FilteredOut);
void RESP_Algorithm_Interface(short CurrSample);
void ECG_FilterProcess(short * WorkingBuff, short * CoeffBuf, short * FilterOut);

/*  Pointer which points to the index in B4 buffer where the processed data*/
/*  has to be filled */

//static unsigned short RESP_B4_Buffer_ptr = 0 ;
/* Variable which will hold the calculated heart rate */

unsigned short Respiration_Rate = 0 ;

/* Variables to hold the sample data for calculating the 1st and 2nd */
/* differentiation                                                   */
int RESP_Second_Prev_Sample = 0 ;
int RESP_Prev_Sample = 0 ;
int RESP_Current_Sample = 0 ;
int RESP_Next_Sample = 0 ;
int RESP_Second_Next_Sample = 0 ;

/* Working Buffer Used for Filtering*/
short RESP_WorkingBuff[2 * FILTERORDER];
//extern unsigned short Resp_Rr_val;
#if (FILTERORDER == 161)

/*
short RespCoeffBuf[FILTERORDER] = {             

// Coeff for lowpass Fc=2Hz @ 500 SPS

       15,     16,     16,     17,     18,     19,     20,     22,     23,
       25,     27,     29,     32,     34,     37,     41,     44,     48,
       51,     56,     60,     64,     69,     74,     80,     85,     91,
       97,    103,    109,    116,    123,    130,    137,    144,    152,
      159,    167,    175,    183,    191,    199,    207,    216,    224,
      232,    241,    249,    257,    266,    274,    282,    290,    298,
      306,    313,    321,    328,    336,    343,    349,    356,    362,
      368,    374,    379,    385,    389,    394,    398,    402,    406,
      409,    412,    414,    416,    418,    419,    420,    421,    421,
      421,    420,    419,    418,    416,    414,    412,    409,    406,
      402,    398,    394,    389,    385,    379,    374,    368,    362,
      356,    349,    343,    336,    328,    321,    313,    306,    298,
      290,    282,    274,    266,    257,    249,    241,    232,    224,
      216,    207,    199,    191,    183,    175,    167,    159,    152,
      144,    137,    130,    123,    116,    109,    103,     97,     91,
       85,     80,     74,     69,     64,     60,     56,     51,     48,
       44,     41,     37,     34,     32,     29,     27,     25,     23,
       22,     20,     19,     18,     17,     16,     16,     15
      
};
*/

// Coeff for lowpass Fc=2Hz @ 125 SPS
short RespCoeffBuf[FILTERORDER] = 
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
#endif


/*********************************************************************************************************/
/*********************************************************************************************************
** Function Name : Resp_FilterProcess()                                  								**
** Description	  :                                                         							**
** 				The function process one sample filtering with 161 ORDER    							**
** 				FIR low pass filter with 2Hz .   														**
**                                                                          							**
** Parameters	  :                                                         							**
** 				- RESP_WorkingBuff	- In - input sample buffer              							**
** 				- CoeffBuf			- In - Co-eficients for FIR filter.     							**
** 				- FilterOut			- Out - Filtered output                 							**
** Return 		  : None                                                    							**
*********************************************************************************************************/

void Resp_FilterProcess(short * RESP_WorkingBuff, short * CoeffBuf, short* FilterOut)
{
	 short i, Val_Hi, Val_Lo;
	 short MACS;

	short RESHI = 0;
	short RESLO = 0;
	short MPYS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	short OP2 = *CoeffBuf++;                             // Load second operand
	
	for ( i = 0; i < FILTERORDER/10; i++)
	{
	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand
	  
	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand
	  
	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand
	  
	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand
	  
	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand
	  
	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand
	  
	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand
	  
	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand

	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand
	  
	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand
	  
	}

	 Val_Hi = RESHI << 1;                       // Q15 result
	 Val_Lo = RESLO >> 15;
	 Val_Lo &= 0x01;
	 *FilterOut = Val_Hi | Val_Lo; 	
}
/*********************************************************************************************************/

/*********************************************************************************************************
** Function Name : Resp_ProcessCurrSample()                                  							**
** Description	  :                                                         							**
** 				The function process one sample of data at a time and       							**
** 				which stores the filtered out sample in the Leadinfobuff.   							**
** 				The function does the following :-                          							**
**                                                                          							**
** 				- DC Removal of the current sample                          							**
** 				- Multi band FIR LPF with Notch at 50Hz filtering           							**
** Parameters	  :                                                         							**
** 				- RESP_WorkingBuff	- In - input sample buffer              							**
** 				- FilterOut			- Out - Filtered output                 							**
** Return 		  : None                                                    							**
*********************************************************************************************************/
void Resp_ProcessCurrSample(short *CurrAqsSample, short *FilteredOut)
{

 	static unsigned short bufStart=0, bufCur = FILTERORDER-1, FirstFlag = 1;
 	static short Pvev_DC_Sample, Pvev_Sample;
 	short temp1, temp2, RESPData;
 	
	/* Count variable*/
	unsigned short Cur_Chan;
	short FiltOut;

	if  ( FirstFlag )
	{
		for ( Cur_Chan =0 ; Cur_Chan < FILTERORDER; Cur_Chan++)
		{
			RESP_WorkingBuff[Cur_Chan] = 0;
		}

		Pvev_DC_Sample = 0;
		Pvev_Sample = 0;
		FirstFlag = 0;
	}
	temp1 = NRCOEFF * Pvev_DC_Sample;
	Pvev_DC_Sample = (CurrAqsSample[0]  - Pvev_Sample) + temp1;
	Pvev_Sample = CurrAqsSample[0];
	temp2 = Pvev_DC_Sample >> 2;
	RESPData = (short) temp2;

	/* Store the DC removed value in RESP_WorkingBuff buffer in millivolts range*/
	RESP_WorkingBuff[bufCur] = RESPData;
	ECG_FilterProcess(&RESP_WorkingBuff[bufCur],RespCoeffBuf,(short*)&FiltOut);
	/* Store the DC removed value in Working buffer in millivolts range*/
	RESP_WorkingBuff[bufStart] = RESPData;


	//FiltOut = RESPData[Cur_Chan];

	/* Store the filtered out sample to the LeadInfo buffer*/
	FilteredOut[0] = FiltOut ;//(CurrOut);

	bufCur++;
	bufStart++;
	if ( bufStart  == (FILTERORDER-1))
	{
		bufStart=0; 
		bufCur = FILTERORDER-1;
	}

	return ;
}
/*********************************************************************************************************/
/*********************************************************************************************************
**  Module Name	: Respiration_Rate_Detection()															**
**  Inputs      : Resp_wave                   		  													**
**  Output      : Respiration Rate																		**
**  Brief   	:																						**
**																										**
** 																										**
*********************************************************************************************************/
void Respiration_Rate_Detection(short Resp_wave)
{

	static unsigned short skipCount = 0, SampleCount = 0,TimeCnt=0, SampleCountNtve=0, PtiveCnt =0,NtiveCnt=0 ;
	static short MinThreshold = 0x7FFF, MaxThreshold = 0x8000, PrevSample = 0, PrevPrevSample = 0, PrevPrevPrevSample =0;
	static short MinThresholdNew = 0x7FFF, MaxThresholdNew = 0x8000, AvgThreshold = 0;
	static unsigned char startCalc=0, PtiveEdgeDetected=0, NtiveEdgeDetected=0, peakCount = 0;
	static unsigned short PeakCount[8];
	
	SampleCount++;
	SampleCountNtve++;
	TimeCnt++; 
	if (Resp_wave < MinThresholdNew) MinThresholdNew = Resp_wave;
	if (Resp_wave > MaxThresholdNew) MaxThresholdNew = Resp_wave;
	
	if (SampleCount > 800)
	{
		SampleCount =0;
	}
	if (SampleCountNtve > 800)
	{
		SampleCountNtve =0;
	}
	

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
//						Respiration_Rate = 6000/SampleCount;	// 60 * 100/SampleCount;
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
						Respiration_Rate = 6000/PtiveCnt;	// 60 * 100/SampleCount;
					}
				}
			}
		}
		else
		{
			skipCount--;
		}
		
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
}

/*********************************************************************************************************
**                                                                       								**
** 	Function Name : RESP_Algorithm_Interface                              								**
** 	Description -   This function is called by the main acquisition      								**
** 					thread at every samples read.  						 								**
**                  Before calling the process_buffer() the below check  								**
** 					has to be done. i.e. We have always received +2      								**
** 					samples before starting the processing  for each     								**
** 					samples. This function basically checks the          								**
** 					difference between the current  and  previous ECG    								**
** 					Samples using 1st & 2nd differentiation calculations.								**
**                                                                       								**
** 	Parameters  : - Respiration CurrSample						     								**
** 	Return		: None                                                   								**
*********************************************************************************************************/
void RESP_Algorithm_Interface(short CurrSample)
{
//	static FILE *fp = fopen("RESPData.txt", "w");
	static short prev_data[64] ={0};
	static unsigned char Decimeter = 0;
	char i;
	long Mac=0;
	prev_data[0] = CurrSample;
	for ( i=63; i > 0; i--)
	{
		Mac += prev_data[i];
		prev_data[i] = prev_data[i-1];

	}
	Mac += CurrSample;
//	Mac = Mac;
	CurrSample = (short) Mac >> 1;
	RESP_Second_Prev_Sample = RESP_Prev_Sample ;
	RESP_Prev_Sample = RESP_Current_Sample ;
	RESP_Current_Sample = RESP_Next_Sample ;
	RESP_Next_Sample = RESP_Second_Next_Sample ;
	RESP_Second_Next_Sample = CurrSample;// << 3 ;
//	fprintf(fp,"%d\n", CurrSample);
	Decimeter++;
	//Resp_Rr_val = RESP_Second_Next_Sample;
	if ( Decimeter == 5)
	{
		Decimeter = 0;
//		RESP_process_buffer();
		Respiration_Rate_Detection(RESP_Second_Next_Sample);
	}
}
/*********************************************************************************************************/
