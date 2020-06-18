//////////////////////////////////////////////////////////////////////////////////////////
//
//   Arduino Library for ADS1292R Shield/Breakout
//
//   Copyright (c) 2017 ProtoCentral
//
//   This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   Requires g4p_control graphing library for processing.  Built on V4.1
//   Downloaded from Processing IDE Sketch->Import Library->Add Library->G4P Install
//
/////////////////////////////////////////////////////////////////////////////////////////
#ifndef ads1292r_h
#define ads1292r_h

#define CONFIG_SPI_MASTER_DUMMY 0xFF

// Register Read Commands
#define RREG 0x20   //Read n nnnn registers starting at address r rrrr
                    //first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define WREG 0x40   //Write n nnnn registers starting at address r rrrr
                    //first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define START 0x08  //Start/restart (synchronize) conversions
#define STOP 0x0A   //Stop conversion
#define RDATAC 0x10 //Enable Read Data Continuous mode.

//This mode is the default mode at power-up.
#define SDATAC 0x11 //Stop Read Data Continuously mode
#define RDATA 0x12  //Read data by command; supports multiple read back.

//register address
#define ADS1292_REG_ID 0x00
#define ADS1292_REG_CONFIG1 0x01
#define ADS1292_REG_CONFIG2 0x02
#define ADS1292_REG_LOFF 0x03
#define ADS1292_REG_CH1SET 0x04
#define ADS1292_REG_CH2SET 0x05
#define ADS1292_REG_RLDSENS 0x06
#define ADS1292_REG_LOFFSENS 0x07
#define ADS1292_REG_LOFFSTAT 0x08
#define ADS1292_REG_RESP1 0x09
#define ADS1292_REG_RESP2 0x0A

typedef struct ads1292r_Record
{
  signed long raw_ecg;
  signed long raw_resp;
  uint32_t status_reg;
} ads1292r_data;

void ads1292r_interrupt_handler(void);
extern portMUX_TYPE ads1292Mux;

class ads1292r
{
public:
  boolean getAds1292r_Data_if_Available(const int data_ready, const int chip_select, ads1292r_data *data_struct);
  static void Init(const int chip_select, const int pwdn_pin, const int start_pin);
  static void Init();
  static void Reset(const int pwdn_pin);
  static void Reset();
  static void Reg_Write(unsigned char READ_WRITE_ADDRESS, unsigned char DATA, const int chip_select);
  static void Reg_Write(unsigned char READ_WRITE_ADDRESS, unsigned char DATA);
  static void SPI_Command_Data(unsigned char data_in, const int chip_select);
  static void SPI_Command_Data(unsigned char data_in);
  static void Disable_Start(const int start_pin);
  static void Disable_Start();
  static void Enable_Start(const int start_pin);
  static void Enable_Start();
  static void Hard_Stop(const int start_pin);
  static void Hard_Stop();
  static void Start_Data_Conv_Command(const int chip_select);
  static void Start_Data_Conv_Command();
  static void Soft_Stop(const int chip_select);
  static void Soft_Stop();
  static void Start_Read_Data_Continuous(const int chip_select);
  static void Start_Read_Data_Continuous();
  static void Stop_Read_Data_Continuous(const int chip_select);
  static void Stop_Read_Data_Continuous();
  static char *Read_Data(const int chip_select);
  static char *Read_Data();
};

#endif
