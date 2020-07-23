#ifndef ADS1x9x_RESP_PROCESSING_H_
#define ADS1x9x_RESP_PROCESSING_H_

/****************************************************************/
/* Constants*/
/****************************************************************/

#define RESP_MAX_PEAK_TO_SEARCH 			5
#define RESP_MAXIMA_SEARCH_WINDOW			8
#define RESP_MINIMUM_SKIP_WINDOW			80

#define RESP_SAMPLING_RATE				100
#define RESP_TWO_SEC_SAMPLES  			2 * RESP_SAMPLING_RATE

/*threshold = 0.7 * maxima*/
#define QRS_THRESHOLD_FRACTION	0.7					

#define MAXCHAN						2
#define FILTERORDER 				161

#define TRUE	1
#define FALSE	0

/* DC Removal Numerator Coeff*/
#define NRCOEFF (0.992)

/****************************************************************/
/* Global functions*/
/****************************************************************/

void RESP_Algorithm_Interface(short CurrSample);


//#define SPS 500

void Resp_ProcessCurrSample(short *CurrAqsSample, short *FilteredOut);
void RESP_Algorithm_Interface(short CurrSample);

#endif /*ADS1x9x_RESP_PROCESSING_H_*/
