#ifndef ADS1x9x_ECG_PROCESSING_H_
#define ADS1x9x_ECG_PROCESSING_H_

/****************************************************************/
/* Constants*/
/****************************************************************/

#define MAX_PEAK_TO_SEARCH 				5
#define MAXIMA_SEARCH_WINDOW			40
#define MINIMUM_SKIP_WINDOW				50

#define SAMPLING_RATE					500
#define TWO_SEC_SAMPLES  				2 * SAMPLING_RATE

/*threshold = 0.7 * maxima*/
#define QRS_THRESHOLD_FRACTION	0.7					

#define MAXCHAN						2
#define FILTERORDER 				161

#define TRUE	1
#define FALSE	0

#define MUSCLE_ARTIFACT_FILTER		1
#define NOTCHFILTERSEL				1		// 0 - 50 Hz Notch filter
											// 1 - 60 Hz Notch filter

/* DC Removal Numerator Coeff*/
#define NRCOEFF (0.992)

/****************************************************************/
/* Global functions*/
/****************************************************************/

void QRS_Algorithm_Interface(short CurrSample);
void ECG_FilterProcess(short * WorkingBuff, short * CoeffBuf, short* FilterOut);
void ADS1x9x_Filtered_ECG(void);

//#define SPS 500


#endif /*ADS1x9x_ECG_PROCESSING_H_*/
