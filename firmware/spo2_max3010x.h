/*************************************************** 
 This is a library written for the Maxim MAX3010X Optical Smoke Detector
 It should also work with the MAX30102. However, the MAX30102 does not have a Green LED.

 These sensors use I2C to communicate, as well as a single (optional)
 interrupt line that is not currently supported in this driver.
 
 Written by Peter Jansen and Nathan Seidle (SparkFun)
 BSD license, all text above must be included in any redistribution.
 *****************************************************/

#pragma once

// Status Registers
static const uint8_t MAX3010X_INTSTAT1      =		0x00;
static const uint8_t MAX3010X_INTSTAT2      =		0x01;
static const uint8_t MAX3010X_INTENABLE1    =		0x02;
static const uint8_t MAX3010X_INTENABLE2    =		0x03;

// FIFO Registers
static const uint8_t MAX3010X_FIFOWRITEPTR  = 	0x04;
static const uint8_t MAX3010X_FIFOOVERFLOW  = 	0x05;
static const uint8_t MAX3010X_FIFOREADPTR   = 	0x06;
static const uint8_t MAX3010X_FIFODATA      =		0x07;

// Configuration Registers
static const uint8_t MAX3010X_FIFOCONFIG    = 	0x08;
static const uint8_t MAX3010X_MODECONFIG    = 	0x09;
static const uint8_t MAX3010X_PARTICLECONFIG= 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX3010X_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX3010X_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX3010X_LED3_PULSEAMP = 	0x0E;
static const uint8_t MAX3010X_LED_PROX_AMP  = 	0x10;
static const uint8_t MAX3010X_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX3010X_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX3010X_DIETEMPINT    = 	0x1F;
static const uint8_t MAX3010X_DIETEMPFRAC   = 	0x20;
static const uint8_t MAX3010X_DIETEMPCONFIG = 	0x21;

// Proximity Function Registers
static const uint8_t MAX3010X_PROXINTTHRESH = 	0x30;

// Part ID Registers
static const uint8_t MAX3010X_REVISIONID    = 	0xFE;
static const uint8_t MAX3010X_PARTID        = 	0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX3010X Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX3010X_INT_A_FULL_MASK   =		(byte)~0b10000000;
static const uint8_t MAX3010X_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX3010X_INT_A_FULL_DISABLE= 	0x00;

static const uint8_t MAX3010X_INT_DATA_RDY_MASK   = (byte)~0b01000000;
static const uint8_t MAX3010X_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX3010X_INT_DATA_RDY_DISABLE= 0x00;

static const uint8_t MAX3010X_INT_ALC_OVF_MASK    = (byte)~0b00100000;
static const uint8_t MAX3010X_INT_ALC_OVF_ENABLE  = 0x20;
static const uint8_t MAX3010X_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX3010X_INT_PROX_INT_MASK   = (byte)~0b00010000;
static const uint8_t MAX3010X_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX3010X_INT_PROX_INT_DISABLE= 0x00;

static const uint8_t MAX3010X_INT_DIE_TEMP_RDY_MASK   = (byte)~0b00000010;
static const uint8_t MAX3010X_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX3010X_INT_DIE_TEMP_RDY_DISABLE= 0x00;

static const uint8_t MAX3010X_SAMPLEAVG_MASK  =	(byte)~0b11100000;
static const uint8_t MAX3010X_SAMPLEAVG_1     = 	0x00;
static const uint8_t MAX3010X_SAMPLEAVG_2     = 	0x20;
static const uint8_t MAX3010X_SAMPLEAVG_4     = 	0x40;
static const uint8_t MAX3010X_SAMPLEAVG_8     = 	0x60;
static const uint8_t MAX3010X_SAMPLEAVG_16    = 	0x80;
static const uint8_t MAX3010X_SAMPLEAVG_32    = 	0xA0;

static const uint8_t MAX3010X_ROLLOVER_MASK   = 	0xEF;
static const uint8_t MAX3010X_ROLLOVER_ENABLE =   0x10;
static const uint8_t MAX3010X_ROLLOVER_DISABLE=   0x00;

static const uint8_t MAX3010X_A_FULL_MASK     = 	0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX3010X_SHUTDOWN_MASK   = 	0x7F;
static const uint8_t MAX3010X_SHUTDOWN        = 	0x80;
static const uint8_t MAX3010X_WAKEUP          = 	0x00;

static const uint8_t MAX3010X_RESET_MASK      = 	0xBF;
static const uint8_t MAX3010X_RESET           = 	0x40;

static const uint8_t MAX3010X_MODE_MASK       = 	0xF8;
static const uint8_t MAX3010X_MODE_REDONLY    = 	0x02;
static const uint8_t MAX3010X_MODE_REDIRONLY  = 	0x03;
static const uint8_t MAX3010X_MODE_MULTILED   = 	0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX3010X_ADCRANGE_MASK   = 	0x9F;
static const uint8_t MAX3010X_ADCRANGE_2048   = 	0x00;
static const uint8_t MAX3010X_ADCRANGE_4096   = 	0x20;
static const uint8_t MAX3010X_ADCRANGE_8192   = 	0x40;
static const uint8_t MAX3010X_ADCRANGE_16384  = 	0x60;

static const uint8_t MAX3010X_SAMPLERATE_MASK =   0xE3; //(byte)~0b0001 1100;
static const uint8_t MAX3010X_SAMPLERATE_50   = 	0x00; //         0000 0000
static const uint8_t MAX3010X_SAMPLERATE_100  = 	0x04; //         0000 0100 
static const uint8_t MAX3010X_SAMPLERATE_200  = 	0x08; //         0000 1000
static const uint8_t MAX3010X_SAMPLERATE_400  = 	0x0C; //         0000 1100
static const uint8_t MAX3010X_SAMPLERATE_800  = 	0x10;
static const uint8_t MAX3010X_SAMPLERATE_1000 =   0x14;
static const uint8_t MAX3010X_SAMPLERATE_1600 =   0x18;
static const uint8_t MAX3010X_SAMPLERATE_3200 =   0x1C;

static const uint8_t MAX3010X_PULSEWIDTH_MASK =   0xFC;
static const uint8_t MAX3010X_PULSEWIDTH_69   = 	0x00;
static const uint8_t MAX3010X_PULSEWIDTH_118  = 	0x01;
static const uint8_t MAX3010X_PULSEWIDTH_215  = 	0x02;
static const uint8_t MAX3010X_PULSEWIDTH_411  = 	0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX3010X_SLOT1_MASK      = 	0xF8;
static const uint8_t MAX3010X_SLOT2_MASK      = 	0x8F;
static const uint8_t MAX3010X_SLOT3_MASK      = 	0xF8;
static const uint8_t MAX3010X_SLOT4_MASK      = 	0x8F;

static const uint8_t SLOT_NONE = 				0x00;
static const uint8_t SLOT_RED_LED = 		0x01;
static const uint8_t SLOT_IR_LED = 			0x02;
static const uint8_t SLOT_GREEN_LED = 	0x03;
static const uint8_t SLOT_NONE_PILOT = 	0x04;
static const uint8_t SLOT_RED_PILOT =		0x05;
static const uint8_t SLOT_IR_PILOT = 		0x06;
static const uint8_t SLOT_GREEN_PILOT = 0x07;

static const uint8_t MAX_30105_EXPECTEDPARTID = 0x15;


#if ((SPO2_TYPE==OXI_MAX30101)||(SPO2_TYPE==OXI_MAX30102))
  #define MAX3010X_I2C_ADDRESS_W          0x57 //7-bit I2C Address
  #define MAX3010X_I2C_ADDRESS_R          0x57 //7-bit I2C Address
#elif (SPO2_TYPE==OXI_MAXM86161)
  #define MAX3010X_I2C_ADDRESS_W          0xC4 //7-bit I2C Address
  #define MAX3010X_I2C_ADDRESS_R          0xC5 //7-bit I2C Address
#else // not build for 3010X chip 
  #define MAX3010X_I2C_ADDRESS_W          0x00 
  #define MAX3010X_I2C_ADDRESS_R          0x00 
#endif 

#define I2C_BUFFER_LENGTH 32

class MAX3010X {
 public: 
  MAX3010X(void);

  boolean begin(TwoWire &wirePort, 
                uint8_t i2c_write_addr,
                uint8_t i2c_read_addr);

  uint32_t getRed(void); //Returns immediate red value
  uint32_t getIR(void); //Returns immediate IR value
  uint32_t getGreen(void); //Returns immediate green value
  bool safeCheck(uint8_t maxTimeToCheck); //Given a max amount of time, check for new data

  // Configuration
  void softReset();
  void shutDown(); 
  void wakeUp(); 

  void setLEDMode(uint8_t mode);

  void setADCRange(uint8_t adcRange);
  void setSampleRate(uint8_t sampleRate);
  void setPulseWidth(uint8_t pulseWidth);

  void setPulseAmplitudeRed(uint8_t value);
  void setPulseAmplitudeIR(uint8_t value);
  void setPulseAmplitudeGreen(uint8_t value);
  void setPulseAmplitudeProximity(uint8_t value);

  void setProximityThreshold(uint8_t threshMSB);

  //Multi-led configuration mode (page 22)
  void enableSlot(uint8_t slotNumber, uint8_t device); //Given slot number, assign a device to slot
  void disableSlots(void);
  
  // Data Collection

  //Interrupts (page 13, 14)
  uint8_t getINT1(void); //Returns the main interrupt group
  uint8_t getINT2(void); //Returns the temp ready interrupt
  void enableAFULL(void); //Enable/disable individual interrupts
  void disableAFULL(void);
  void enableDATARDY(void);
  void disableDATARDY(void);
  void enableALCOVF(void);
  void disableALCOVF(void);
  void enablePROXINT(void);
  void disablePROXINT(void);
  void enableDIETEMPRDY(void);
  void disableDIETEMPRDY(void);

  //FIFO Configuration (page 18)
  void setFIFOAverage(uint8_t samples);
  void enableFIFORollover();
  void disableFIFORollover();
  void setFIFOAlmostFull(uint8_t samples);
  
  //FIFO Reading
  uint16_t check(void); //Checks for new data and fills FIFO
  uint8_t available(void); //Tells caller how many new samples are available (head - tail)
  void nextSample(void); //Advances the tail of the sense array
  uint32_t getFIFORed(void); //Returns the FIFO sample pointed to by tail
  uint32_t getFIFOIR(void); //Returns the FIFO sample pointed to by tail
  uint32_t getFIFOGreen(void); //Returns the FIFO sample pointed to by tail

  uint8_t getWritePointer(void);
  uint8_t getReadPointer(void);
  void clearFIFO(void); //Sets the read/write pointers to zero

  //Proximity Mode Interrupt Threshold
  void setPROXINTTHRESH(uint8_t val);

  // Die Temperature
  float readTemperature();

  // Setup the IC with user selectable settings
  void setup(byte powerLevel = 0x1F, byte sampleAverage = 4, byte ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);

  // Low-level I2C communication
  uint8_t readRegister8(uint8_t address, uint8_t reg);
  void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);

 private:
  TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
  uint8_t _i2c_read_addr;
  uint8_t _i2c_write_addr;

  //activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
  byte activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO
  
  uint8_t revisionID; 

  void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
 
  #define STORAGE_SIZE 250  //Each long is 4 bytes so limit this to fit on your micro

  typedef struct Record
  {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    uint32_t green[STORAGE_SIZE];
    byte head;
    byte tail;
  } sense_struct; //This is our circular buffer of readings from the sensor

  sense_struct sense;
};
