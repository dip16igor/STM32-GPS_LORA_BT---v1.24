/* �������� ��� ����������� "����������������" �� ������ MESH LORA ����
 * ������� ������ - NMEA � GPS/GLONASS ���������
 * ����� ����� ������� - MESH ���� � ��������� �����������
 * ��� ������ ���������� ������������ ��������, ����������� ����� BT HM-10
 * ���������� - STM32F103C8T6
 * ����� ���������� - Coocox v.1.7.8 + GCC 5.3 + StdPeriphLib v.3.5.0
 * ������� - ST-Link v2 � ������������, ������������ �� SWD, ���� ���� CoLinkEx
 * USART1 - ����� � ���������� �� PC
 * USART2 - ����� � BLE HM-10 (���� XM-10, ��� ������)
 * USART3 - ����� � GPS SIM-33ELA
 * SPI1 - ����� � LORA ������� (RFM68)
 * SPI2 -
 * I2C1 - OLED ������� SSD1306
 * I2C2 -
 * ADC1 ch0 - ��������� ���������� ������������
 * ADC1 ch1 - ��������� ���������� ������� ���������� �������
 *
 *  14.05.2016	v0.1	UART, ������� NMEA
 *  02.06.2016	v0.2	��������� �������
 *  11.06.2016	v0.3	��������� SPI, ����������� RFM98
 *  18.12.2016  v1.0    ������� �� ����� ����� v0.1
 *  15.02.2017  v1.1	������� ������� � Dropbox, ������������ ����������
 *  20.02.2017  v1.2   	�������� I2C � OLED ������� SSD1306
 *  21.02.2017	v1.3	�������� ADC
 *  23.02.2017  v1.4	��������� ���� ��������� ������� RTC
 *  08.03.2017	v1.5	�������� ������ GSA ��������� (DOP)
 *  10.03.2017	v1.6 	��������� ������� ���������� �� 1PPS, ������������� RTC
 *  21.03.2017  v1.7 	�������� LORA ���������
 *  29.03.2017  v1.8 	�������� BLE ������ XC-10 �� ���� CC41-A
 *  28.06.2017	v1.9	����������� ����� ��� ��������� GPS � RTC
 *  25.06.2017 	v1.10	����� �� RTC, ������� �� ������� ������
 *  01.08.2017  v1.11	�������� ������� ����� GLONASS ��������� 
 *  06.08.2017	v1.12	�������� ������� �� �����������
 *  30.11.2017	v1.13	����� ������� �� ������ ������, ���� ���� ��������� ��������, ��������� ������
 *  03.12.2017	v1.14	�������� � ����� ���������, ������������
 *  09.12.2017	v1.15   ���������� ��������� PER
 *  11.12.2017 	v1.16	��������� ������������� ��� ���������� ���������� ����� �������
 *  16.12.2017	v1.17	�������� ������ ���������� ������������ �� �������, ������� ��� � ��������� ������� ������ �����������
 *  17.12.2017	v1.18	I2C ��� ������� ����� DMA
 *	30.12.2017	v1.19	��������� PER ��� ����������, ��������� ������������� ���������� �������
 *	02.01.2018	v1.20	����� � ���� ������� �������� � GPS �� ���������, ��������� ���������� ������� �� ��������
 *  04.05.2018	v1.21   ������� ����� ���������� �� �������, ������ �����������.
 *  10.06.2018	v1.22	������� �� GCC 7.0
 *  01.09.2018	v1.23	�������� Release ����� ����������� �������. ��� ����� � ���� ��� ������� ���������� ������ SCREEN
 *  08.09.2018	v1.24	��� �������� ����������� ������� �� ���� �������� ������� �������. � ����� ������ ������� ���� ��������� 1 ���� ������� 0..255
 *  todo
 *  		�������� ��������� ����������� �������� � ��� STM32
 *  		//���������� ���������� ������������ ��� ������
 *  		�������� ���������� ����������� �������� �����������
 *  		��������� ��� � ���������� ����������� ��� LORA_TX_TIME = 0 ��� �������������� ���������� 1PPS
 *  		�������� � ������� ������ ������ ���� ��� ����������� �������� ����� 00 �����
 *  		����������� � ������������ ������� BLE
 *  		// �������� ����������� ���������� ��������� � ����������
 *  		//����������� ������������� ���������� ������
 *  		� ����������� ��������� ����� �� ���������� ���������  LORA
 *  		������ ����������� ���������������� ������� ( ���� � ����� 2. ������������ ����������� �������� ����� �������������� �������, �� �� ������)
 */

#define LORA_TX_TIME 	3		// ����� ������� ��� �������� (1-9)  3 - 03 13 23 33 43 53 03 � ��.  <----------------------------------------------------------------------------------------- TX TIME -----------------
#define VersionMajor	1		// ������ �������� v1.24
#define VersionMinor	24
#define RFPowerDef		2		// �������� LORA ����������� (�� 2 �� 17 dBm - �� 1,6 �� 50,1 mW)
#define LcdOffTimeout 	180000	// ����� �� ���������� ������, ��

//  ����� ���������� ���������� � USART1
//#define DebugGpsToUart1				// ����� ���������� ���������� �� GPS
//#define Debug1PPSMeasurementToUART	// ����� ���������� ���������� �� ��������� ������������ 1PPS
//#define DebugLORAToUART				// ����� ���������� ���������� � ������ ����������� LORA

#include "stm32f10x.h"			// ����
#include "stm32f10x_flash.h"	// ������ � flash ������� ��������
#include "stm32f10x_gpio.h"		// ����� �����-������
#include "stm32f10x_rcc.h"		// ������� ������������
#include "stm32f10x_tim.h"		// �������
#include "stm32f10x_usart.h"	// ���������� USART
#include "stm32f10x_spi.h"		// SPI
#include "stm32f10x_adc.h"		// ADC
#include "stm32f10x_exti.h"		// ������� ���������� EXTI
#include "stm32f10x_dma.h"		// DMA ( ������������ ��� i2c ������� )
#include "misc.h"				// ����������
#include "stdio.h"				// printf  ��������� rprintfInit()
#include "string.h"				// ������ �� ��������
#include <stdlib.h>				// ���� ������, ��������� � ��.
#include <math.h>				// ����������
#include <stdbool.h>			// ����������� ���� bool

#include "sx1276.h"				// ������� LORA �������
#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"
#include "platform.h"
#include "radio.h"

#include "ssd1306.h" 			// OLED SSD1306

// ��������� �������
void init						(void);
void RCC_Configuration 			(void);
void DMA_Configuration			(void);
void NVIC_Configuration 		(void);
void PortIO_Configuration		(void);
void USART1_Configuration		(void);
void USART2_Configuration		(void);
void USART2_Configuration115200	(void);
void USART3_Configuration		(void);
void USART3_Configuration115200	(void);
void RTC_Configuration			(void);
void TIM1_Configuration			(void);
void TIM2_Configuration			(void);
void TIM3_Configuration			(void);
void TIM4_Configuration			(void);
void SPI1_Configuration			(void);
void ADC1_Configuration			(void);
void EXTI_Configuration			(void);

void USART_SendByte 		(USART_TypeDef* USARTx, u16 Data);
void USART1_SendByte 		(char byte);
void USART2_SendByte 		(char byte);
void USART3_SendByte 		(char byte);

u8 USART_Scanf				(u32 value);
void Time_Display			(u32 TimeVar);
void Time_Display2			(u32 TimeVar); 	// ��� ������

void DelayMS				(u16 delay);	// ����������� �������� � �� �� SysTick

void OnVcc1 				(void); 	// �������� �������
void OffVcc1 				(void); 	// ��������� �������

void NRESET_on				(void);		// RESET SIM33ELA
void NRESET_off				(void);

void OnLED0 				(void);
void OffLED0 				(void);
void CplLED0				(void);

void OnLED1 				(void);
void OffLED1 				(void);
void CplLED1				(void);

void OnTest					(void);
void OffTest				(void);
void CplTest				(void);

void OnResetLORA			(void);
void OffResetLORA			(void);
void OnNssLORA				(void);
void OffNssLORA				(void);

void LoraProcess			(void);
void LoraWriteReg			(u8 Addr, u8 Reg);
u8	 LoraReadReg			(u8 Addr);
void LoraWriteBuffer 		(u8 Addr, u8 *buffer, u8 size);
void LoraReadBuffer 		(u8 Addr, u8 *buffer, u8 size);
void LoraWriteFIFO 			(u8 *buffer, u8 size);
void LoraReadFIFO 			(u8 *buffer, u8 size);

void OnResetBLE 			(void);
void OffResetBLE 			(void);

u16  GetSamplesADC1			(void); // ������� ��� todo ����������
void OnDivGND1 				(void);
void OffDivGND1				(void);
void OnDivGND2 				(void);
void OffDivGND2				(void);

void Display1				(void); // ����� ���������� �� �����
void Display2				(void); // ���������� ������������, ���������
void Display3				(void); // ����� RTC
void Display4				(void); // ���������� � ���������
void Display5				(void); // ���������� � ��������� � ����������� ����
void Display6				(void); // ���������� � ���������, ������������ � ������� � ���������� DOP
void Display7				(void); // ���������� � ������ � u32
void Display8				(void); // ���� ��������� ������� 1PPS
void Display9				(void); // �������� LORA
void Display10				(void); // �������� ����������
void Display11				(void); // ������� �������
void Display12				(void); // ��������� �������� �������
void Display13				(void); // PER �� ������� �����
void Display14				(void); // ���������� ���������� ����� �������
void Display15				(void); // ������ ������� �� ������ � ����������� ����

void Display1Release		(void); // �������� �����
void Display2Release		(void); // ���������� ����� �������
void Display3Release		(void); // ���������� GPS

void OLED_Putc1				(char c);
void OLED_Putc2				(char c);
void OLED_Putc3				(char c);

void InitMeasVoltage		(void); // �������������� ���������� ������� ���������� ����������� ����������
void MeasVoltage			(void); // ��������� ���������� ������������ � �������

void ScanKeyPowerOff 		(void); // ���������� ������� ������ PowerOff
void ScanKeySOS 			(void); // ���������� ������� ������ SOS

void ReceiveGPS				(void); // ������ � ������������� ������ �� GPS
void Measurement1PPS		(void);	// ��������� � ���������� ������� ������� 1PPS
void Average1PPS			(void);	// ���������� ���������� �������� ������� 1PPS

void ClearGPSBuffer			(void);	// ������� ������ UART GPS

void ClearBLEBuffer			(void);	// ������� ������ UART BLE

// ------------- NMEA PARCER ------------------------------
u8  NMEA_Parser_GGA			(void);
u8  NMEA_Parser_ZDA			(void);
u8	NMEA_Parser_GSV_GPS		(void);
u8 	NMEA_Parser_GSV_GLO		(void);
u8	NMEA_Parser_GSA_GPS		(void);
u8	NMEA_Parser_GSA_GLO		(void);

u8	ControlCheckSum			(u16 StartIndex);

u32 GetGPSTimeSec 			(void); // ���������� ������� GPS �� ZDA ���������. ����� �������
u32 GetGPSTimeMilliSec 		(void); // ���������� ������� GPS �� ZDA ���������. ������������

// ---------------------------- #define ---------------------------------------------

#define GPSBUFLENGTH  (1000)       // ����� ��������, ����� ������ ��� ������ ������� (GSV �������� �������)
#define RESERVE (0)   // ����� �������� ���� ��������� GPGGA_Struct, ���� ��������� ������ ��� ��� ������ �������������
#define MAXSAT (64)   // ������������ ���������� ���������, GPS + �������, ��� ������� ���������� ������� ���������
#define PARSAT (4)    // �����, ���� �����, ������, SNR, ��� ������� ���������� ������� ���������
#define ONESAT "nnn"  // ������������ ����� ������ ���� ������� ���������� ������� ���������

#define USE_SX1276_RADIO	// ����� ����������� ��� ���������� LORA
#define BUFFER_SIZE	256		// Define the payload size here LORA
#define BW 250  	  		// ������ ������ ������� ��� ������� ������ �� �������, ���. LORA

#define Terminal			USART1_SendByte
#define BleUsart			USART2_SendByte
#define GpsUsart 			USART3_SendByte
#define OLED_font1 			OLED_Putc1  // ����� 7x10 ��������
#define OLED_font2 			OLED_Putc2  // 		11x18
#define OLED_font3 			OLED_Putc3  // 		16x26

#define BufferUSART1Size 	100 	// ������ ��������� ������ USART1
#define BufferUSART2Size 	100 	// ������ ��������� ������ USART2
#define BufferUSART3Size 	100 	// ������ ��������� ������ USART3
#define BaudRateUSART1		115200	// �������� USART1  - ����� � ��, ������� ����� ��������
#define BaudRateUSART2		9600	// �������� USART2  - ����� � BLE, ������������� �� 115200
#define BaudRateUSART3		9600	// �������� USART3  - ����� � GPS, ������������� �� 115200

#define VaccPowerOff		3000 	// ���������� ������������, ��� ������� ������� �����������
#define VaccAttention		3100 	// ���������� ������������, ��� ������� ��� �������� �������� �������������� � ���������� ������ ���������� �����������

#define STOP 				for(;;){} // ����������� ������ ����, ��� �������

// ----------------------- ���������� ���������� -----------------------------------------------
// ----------------- USART, TIM ---------------------------------------------------
u8				NumberDev = LORA_TX_TIME;			// ����� ����������		

ErrorStatus 	HSEStartUpStatus;

volatile u8 	triggerUSART1Interrupt = 0;
volatile u16 	counterUSART1Buffer;
volatile char	BufferUSART1RX[BufferUSART1Size+1] = {'\0'}; // �������� ����� USART1

volatile u8 	triggerUSART2Interrupt = 0;
volatile u16 	counterUSART2Buffer;
volatile char	BufferUSART2RX[BufferUSART2Size+1] = {'\0'}; // �������� ����� USART2

volatile u8 	triggerUSART3Interrupt = 0;
volatile u16 	counterUSART3Buffer;
//volatile char	BufferUSART3RX[BufferUSART3Size+1] = {'\0'}; // �������� ����� USART3

volatile u32 	timer1ms, timer1s; 		// ���������� ���������� �������

volatile u8		triggerTIM2Interrupt;   // ���� ���������� �� ������ ������� �� BLE ������
volatile u8		triggerTIM3Interrupt;   // ���� ���������� �� ������ ������� �� GPS ������

volatile 		u32 SecRTC	;			// ������� ����� RTC � ��������
u32 			SecRTCold;				// ���������� �������� �������. ��� ���� ���������� ������������
u32 			sec_RTC_to_display ;	// ������� �� ������� ��� �����������
u32 			sec_GPS_to_display ;	// ������� �� GPS ��� �����������

u32 i, j; 	// ������� ������ ����������

// --------------------------- ���������� GPS ��������� ----------------------------
char 	GPSBuf[GPSBUFLENGTH];	// ����� UART ��� ������� �� GPS ������
volatile u16 GPSBufWrPoint; 	// ����� �� ���������, � ��������, ����� ������ ����� ������
u16	GSVStartPos = 0; 			// ��� ���������� ������������ ������ � NMEA_Parser_GSV(), �� ���������� ������
//u16 GSVStartPosGLO = 0;		// � �������� ���������, ������ ��� ���������� ���������������� �� ����� ���������� ���������������� ������� GSVStartPos
u8 	SatNumGPS = 0;				// ���������� ��������� GPS
u8 	SatNumGLO = 0;				// ���������� ��������� GLONASS
u8 	SatFix = 0;	 				// ���������� ���������, ������������ � ������� �� GGA ������������������
u8 	SatFixGPS = 0;	 			// ���������� ��������� GPS, ������������ � ������� �� GSA
u8 	SatFixGLO = 0;	 			// ���������� ��������� GLONASS, ������������ � ������� �� GSA
u8 	GSVNumStringGPS	 = 0; 		// ���������� GSV ����� �� GPS 
u8 	GSVNumStringGLO	 = 0;		// ���������� GSV ����� �� GLONASS

struct GPGGA_Struct  		// ��������� � ������� GPS-�������-������
{
  char Time [sizeof("hhmmss.ss")+RESERVE];      	// �����
  char Latitude [sizeof("xxxx.yyyy")+RESERVE];    	// ������
  char NS;                            				// �����-��
  char Longitude[sizeof("xxxxx.yyyy")+RESERVE]; 	// �������
  char EW;                            				// �����-������
  char ReceiverMode;                    			// ����� ������ ���������
  char SatelliteNum [sizeof("nn")+RESERVE];     	// ���������� ��������� � �������
  char Altitude [sizeof("aaaaa.a")+RESERVE];    	// ������
  char Year[sizeof("yyyy")+RESERVE];          		// ���
  char Month[sizeof("mm")+RESERVE];         		// �����
  char Day[sizeof("dd")+RESERVE];           		// �����
  char LocalTimeHr [sizeof("+hh")+RESERVE];     	// �������� �� ������� �����, ����
  char LocalTimeMn [sizeof("mm")+RESERVE];      	// �������� �� ������� �����, ������
  char Sats [MAXSAT][PARSAT][sizeof(ONESAT)];   	// ������ ���������� ������� ��������� GPS 
  char SatsGLO [MAXSAT][PARSAT][sizeof(ONESAT)];   	// ������ ���������� ������� ��������� GLONASS
};

struct GPGGA_Struct GPSFixData; 	// � ���� ���������� � ����� ��������
u8  SatLevelsGPS[40]; 				// ������ S/N �� ��������� GPS
u8  SatIDGPS 	[40]; 				// ������ ������� ��������� GPS
u8  SatLevelsGLO	[40];			// ������ S/N �� ��������� GLONASS
u8  SatIDGLO 	[40]; 				// ������ ������� ��������� GLONASS
u16 SatSummGPS, SatSummUsedGPS;		// ����� S/N ������� � ������������ ��������� GPS
u16 SatSummGLO, SatSummUsedGLO;		// ����� S/N ������� � ������������ ��������� GLONASS
u8	SatUsedGPS	[12];				// ������ ������� �������������� � ������� ��������� GPS
u8	SatUsedGLO	[12];				// ������ ������� �������������� � ������� ��������� GLONASS
u8	SatNumUsed;						// ���������� �������������� � ������� ���������
u16	PDOP, VDOP, HDOP;				// ������������ ��������� �������� ����������� ��������� ( x100 )
u8 	Fix;							// ����� ������ GPS ��������� - 2D/3D/��� �������
u8	AutoSel_2D3D;					// 1 - �������������� ����� 2D/3D, 0 - ������
u32 GPSTimeSec;						// ���������� ������ ������� ����� �� ZDA ���������
u32 GPSTimeMilliSec;				// ���������� ����������� �� ZDA ���������
u32 GPSLongitude, GPSLatitude;		// ������� ������ � �������
u8  GPS_N_S, GPS_E_W ;				// 1 - �����, 0 - ��; 1 - �����, 0 - ������
u32 GPSAltitude;					// ������� ������
u16 TTFF;							// ����� �� ������ �������� ��������������
// --------------------------------------------------------------------------------
char c1;
u8 size;
char m;

int tempData = 0;
char test [50];

bool triggerTTFF ;		// ������� ��������� ������� ������ �������� ��������������
bool flagTX;			// ���� ������� �������� ������ �� 433
bool flagTXDisplay;
bool flagRXDisplay;
//----------------------------- ���������� LORA ------------------------------------------
u8 	LoraTxTime  = LORA_TX_TIME; // ����� ������� ��� �������� � ����

s8 	RFPower = RFPowerDef;		// �������� �����������
u16 SPI1read;

u8  LORAbufferRX 	[256];
u8  LORAbufferTX 	[256];

static u16 BufferSize = BUFFER_SIZE;			// RF buffer size
//static u8 Buffer[BUFFER_SIZE];				// RF buffer
tRadioDriver *Radio = NULL;						// ��������� RadioDriver , ��������� �� �������

u8 temp1, temp2;
u32 temp3;

/*!
 * SX1276 LoRa registers variable
 */
//extern tSX1276LR* SX1276LR;
extern  u8 RFLRState;

u16 countPacket, countPacketOld, Power;
u32 countOK, countERR;
u32 PER1, PER2;
//u32 PER;
u8 trigger_PER;
u8 CRCerror;
double FreqError;		// ������ ��������� ������� �� �������
s32 FREQ_ERROR;
bool flagRX_OK;			// ���� ���������� ����� ������ � �������� ������

volatile u16 counterRXDisplay; 	// ������� ������� ����������� RX �� �������
volatile u16 counterTXDisplay; 	// ������� ������� ����������� TX �� �������
bool triggerOffLED0;			// �������� ��� ������������ ���������� �����������
bool triggerOffLED1;
volatile u16 counterLed0;		// ������� ������� ������� ������ ����������
volatile u16 counterLed1;		// ������� ������� ������� �������� ����������


u32 LoraResult ;			// ��������� �������� ���������  LORA ����������
//u32 LoraCounter ;			// �������� �������

typedef struct LoraRXstruct_t 		// ��������� ���������� ��������� ������� �� ������ �����
{
	s16		RSSI;		// ���������� �������� ������ ������� RSSI � �������� ������
	s32 	FreqError;	// ������������� �������� �� ������� ��������� ������
	s8		SNR;		// ����������� ������/��� ��� ��������� ������
	s8		Power;		// �������� ��������� �����������, ���������� � ������
	u8 		CounterTxP;	// ������� ���������� �������. �������� todo
}LoraRXstruct;

LoraRXstruct LoraRX [10]; 	// ������ �������� ���������� ������ �� ������� �� ������
s16 LoraRssiMax = -130	;	// ������������ ������� ������� �� �������� �������, ��� ����������
u16 counterMaxRssiReset ;	// ������� ��� �������������� ������ ������������� ������ RSSI
u16 counterRssiReset [10];	// ������ ��������� ��� ���� ������, ��� ������ ������ RSSI �� �������

u8 NumberTransmitted;		// ����� �����, �� �������� ������� �������
s8 RFPowerTransmitted;  	// �������� �����������, �� �������� ������� �������

u32	TimeSecFirstRX [10];	// ������ ������ ����� �� ������� ����� ��� ������ ������ � ������ ���. ��� ���������� PER
u16 CountRX	[10];			// �������� �������� ������� �� ������. ��� ���������� PER
u16 PER [10];				// ������ ���������� PER ��� ������� �����

//u16 timeDelta; // ��������
#pragma pack (push, 1) 		// �������� ������� �������� ������������ �������� �� ���������� ���� �����������, ������� ������������ = 1 ����

typedef struct datastruct_t // ��������� ������ ������ �����
{	// 6 32-��������� ����, 24 ����
	u8	Number;			// ����� ����� � �������� ��������� ������� (+ �����)
	u8  NS_EW;			// �-� / �-�
	u16 Vacc;			// ���������� ������������
	u32 TimeSec;		// ������� ����� UTC
	u32 Longitude;		// �������
	u32 Latitude;		// ������
	u32 Altitude;		// ������
	u32 reserv;			// ������  (NO FIX, 2D, 3D)
}DATASTRUCT;  			// ���������� ����� ��� ���������� � ���� ���������

#pragma pack (pop)			// ������� ������ �� ������� ����������� ����� �����������


// DATASTRUCT TX_str ;

//DATASTRUCT  TX_Packet [10] ; 	//

// �������� �������
// ����� ������ �� 10-�� �������� ���� DATASTRUCT
DATASTRUCT structBuffer [10];

u32 TimeNew	;				// ����� ����� ������� �� ������� �� ������
u32 TimesOld	[10];		// ������ ������ ������ ������� �� ������� �� ������

// ���-�� � �������������:
//char *inBuffer = (char*)structBuffer; // ��������� �� ���������� ���� char. structBuffer ����� ���������� � ����� ����
// ������� ����� ������ � inBuffer [] ������� ����������� ������ �������� ���� DATASTRUCT (?)

//------------------------------ ���������� ADC --------------------------------------
u16 adc1, adc2, adcIntRef;
u32 Vacc1, Vbat1, Vref1, VaccSumm, VbatSumm, VrefSumm; 	// ������� ���������� ������������ � ���������, � ��
#define Average1	1000									// ������ �������� ��� ���������� ���������� ����������
u16 Vacc [Average1];									// ������� ��� ���������� ���������� �������
//u16 Vbat [Average1];
u16 Vref [Average1];
u16 VaccAvg, VrefAvg;			// ����������� ��������
s8 VrefDev, VaccDev;			// ���������� �� ������������ ��������, � %
bool ChargeTrigger; 			// ������� ������ ������������, ��� ���������

u8 VaccLog [128]; 		// ������ ���������� ������������ ��� �������
u8 counterVaccLog; 		// ������� �������� ������� � ����
u32 TimeStartLogVcc; 	// ����� ������ ������ � ���
u32 TimeLogVcc; 		// ������������ ������ � ���
bool triggerTimeStartLogVcc ; // ������� ��� ����������� ������� ������ ������ ����
u8 counterVaccosc;		// ������� ��� ������������� ����������� ����������
s8 VaccOsc [100];		// ������ ��� ������������� ����������� ����������
//----------------------- ������, ����� ----------------------------------------------
bool trigger1, trigger2;				// �������� ������������ ������� ������
u8	screen = 1;							// ����� ������ ��� �����������
#define  PowerOffTimeout 100			// ������� ��� ����������� ������� ������ SCREEN
u16	counterPowerOff = PowerOffTimeout; 	// ������� �������� ���������� ������� �������
u32 counterLcdOff = LcdOffTimeout;		// ������� �������� ���������� �������, ��

//------------------ ������, ��������� ������� 1PPS ------------------------------
#define  AVG_TIM4 			30 			// ��������� 1PPS �� 30 ���������

volatile u8 counter_TIM4_OVR; 			// ������� ������������ ������� 4
volatile u8 counter_TIM1_OVR; 			// ������� ������������ ������� 1
volatile u8 counter_TIM4_fix;			// �������� �������� ������ ������� �� ������ ������� ���������� 1PPS
volatile u16 TIM4_fix;					// �������� ������� � ������ ������� ���������� 1PPS
volatile bool flag_1PPS_Update;  		// ���� ���������� �������� ������� 1PPS
volatile bool flag_1PPS_timeout;		// 1PPS �����������
u16 averageTIM4 [AVG_TIM4];				// ������ ��� ���������� �������� ������� 1PPS
u16 averageTIM4_temp;					// ����������� �������� ������� 1PPS
u16 averageTIM4_ready = 37200;			// ����������� �������� ������� 1PPS ����� 35 ����������
volatile bool flag_1PPS_fail;			// ���� ������ ��������� ������� 1PPS, ������ ������� ��������� ��� �������
volatile bool flag_1PPS_AVG_OK = 0;		// ���� ���������� ����������� ���������� 1PPS
volatile bool flag_sync_stop = 0;		// ��������� ���� ��� ��������� ����� ������� �������
volatile u8 counter_AVG_1PPS = 0;		// ������� ���������� 1PPS
u8 i_avg = 0; 							// ������� ����������
u32 summ = 0;							// ����� ��� ����������
volatile bool flag_1PPS_pulse;			// ���� ��������������� � ���������� �� 1PPS ��� �� ������� 1

u32 sec_div;							// ������� �� ������� ������ ������� ����� �� 60

double distance1_1_2, distance1_1_3, distance1_2_3;		// ���������� ����� �������
double distance2_1_2, distance2_1_3, distance2_2_3;		// ���������� ����� �������
double latt_1, latt_2, latt_3, latt_4;  // ������  ������ � ��������
double long_1, long_2, long_3, long_4; 	// ������� ���� ������ � ��������
double grad, rad;



// ���������� I2C1
__IO uint8_t Tx_Idx1=0 ;
extern __IO uint32_t NumbOfBytes1;
extern uint8_t Buffer_Tx1[];
extern uint8_t Address;

#define DebugMode	0		// ���������� ������ �������
#define ReleaseMode	1		// ����������� ������ �������
u8 ScreenMode = DebugMode;	// ����� ������ ���������� �� ����� - ������ ������ ��� �����������
u8 CounterTxPacket;			// ������� ���������� � ���� �������. ��� �������� ����������� ������� �� ����, ��������

//--------------------------------- MAIN -----------------------------------------------------------------------------------------------------------------------------
int  main(void)
{
	init(); 				// ������������� �����������, ���������, ������� �������

	Radio -> StartRx();		// ��������� ���������� �� �����
//============================================================================================
	for (;;) 	// ����������� ����
	{
		
		Measurement1PPS (); 	// ��������� ������� ������� 1PPS
	
		Average1PPS ();			// ����������

		ReceiveGPS(); 			// �������� ������ ������� �� GPS � �������



// -- ���������� ���������� ����� �������  ----------- 55�10.7014  55107014     61�24.0958  6124058

//		structBuffer[1].Latitude = 55450000 ;
//		structBuffer[1].Longitude = 61370000 ;
//
//		structBuffer[2].Latitude = 55460000 ;
//		structBuffer[2].Longitude = 61380000 ;

//latt_2 = ( ( (GPSLatitude / 1000000) + ( (GPSLatitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// ���������� � ��������
//long_2 = ( ( (GPSLongitude / 1000000) + ( (GPSLongitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// ���������� � ��������

		latt_1 = ( ( (structBuffer[1].Latitude / 1000000) + ( (structBuffer[1].Latitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// ���������� � ��������
		latt_2 = ( ( (structBuffer[2].Latitude / 1000000) + ( (structBuffer[2].Latitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// ���������� � ��������
		latt_3 = ( ( (structBuffer[3].Latitude / 1000000) + ( (structBuffer[3].Latitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// ���������� � ��������
		latt_4 = ( ( (structBuffer[4].Latitude / 1000000) + ( (structBuffer[4].Latitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// ���������� � ��������
		//rad = (grad * M_PI) / 180;							// ���������� � ��������

		long_1 = ( ( (structBuffer[1].Longitude / 1000000) + ( (structBuffer[1].Longitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// ���������� � ��������
		long_2 = ( ( (structBuffer[2].Longitude / 1000000) + ( (structBuffer[2].Longitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// ���������� � ��������
		long_3 = ( ( (structBuffer[3].Longitude / 1000000) + ( (structBuffer[3].Longitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// ���������� � ��������
		long_4 = ( ( (structBuffer[4].Longitude / 1000000) + ( (structBuffer[4].Longitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// ���������� � ��������

		//distance1 = sin( latt_2 );


		distance1_1_2 = 6372795 * acos ( sin (latt_1) * sin (latt_2) +cos (latt_1) * cos (latt_2) * cos (long_1 - long_2) )  ; // ���������� ����� 1 � 2 �������. ��������� sin � �������� . �������� � ������
		distance1_2_3 = 6372795 * acos ( sin (latt_2) * sin (latt_3) +cos (latt_2) * cos (latt_3) * cos (long_2 - long_3) )  ; // ���������� ����� 2 � 3 �������. ��������� sin � �������� . �������� � ������
		distance1_1_3 = 6372795 * acos ( sin (latt_1) * sin (latt_3) +cos (latt_1) * cos (latt_3) * cos (long_1 - long_3) )  ; // ���������� ����� 1 � 3 �������. ��������� sin � �������� . �������� � ������



		if ( LORA_TX_TIME == 1)  // ���� ���� ���� - �1
		{
			latt_1 = ( ( (GPSLatitude / 1000000) + ( (GPSLatitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;				// ���������� �������� �������������� � ��������
			long_1 = ( ( (GPSLongitude / 1000000) + ( (GPSLongitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;			// ���������� � ��������
			distance2_1_2 = 6372795 * acos ( sin (latt_1) * sin (latt_2) +cos (latt_1) * cos (latt_2) * cos (long_1 - long_2) )  ; 	// ���������� ����� ������� ��������������� � ��������� �������� �� ������� �����. ��������� sin � �������� . �������� � ������
			distance2_1_3 = 6372795 * acos ( sin (latt_1) * sin (latt_3) +cos (latt_1) * cos (latt_3) * cos (long_1 - long_3) )  ; 	// ���������� ����� ������� ��������������� � ��������� �������� �� ������� �����. ��������� sin � �������� . �������� � ������
		}

		if ( LORA_TX_TIME == 2)  // ���� ���� ���� - �2
		{
			latt_2 = ( ( (GPSLatitude / 1000000) + ( (GPSLatitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;				// ���������� �������� �������������� � ��������
			long_2 = ( ( (GPSLongitude / 1000000) + ( (GPSLongitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;			// ���������� � ��������
			distance2_1_2 = 6372795 * acos ( sin (latt_1) * sin (latt_2) +cos (latt_1) * cos (latt_2) * cos (long_1 - long_2) )  ; 	// ���������� ����� ������� ��������������� � ��������� �������� �� ������� �����. ��������� sin � �������� . �������� � ������
			distance2_2_3 = 6372795 * acos ( sin (latt_2) * sin (latt_3) +cos (latt_2) * cos (latt_3) * cos (long_2 - long_3) )  ; 	// ���������� ����� ������� ��������������� � ��������� �������� �� ������� �����. ��������� sin � �������� . �������� � ������
		}

		if ( LORA_TX_TIME == 3)  // ���� ���� ���� - �3
		{
			latt_3 = ( ( (GPSLatitude / 1000000) + ( (GPSLatitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;				// ���������� �������� �������������� � ��������
			long_3 = ( ( (GPSLongitude / 1000000) + ( (GPSLongitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;			// ���������� � ��������
			distance2_1_3 = 6372795 * acos ( sin (latt_1) * sin (latt_3) +cos (latt_1) * cos (latt_3) * cos (long_1 - long_3) )  ; 	// ���������� ����� ������� ��������������� � ��������� �������� �� ������� �����. ��������� sin � �������� . �������� � ������
			distance2_2_3 = 6372795 * acos ( sin (latt_2) * sin (latt_3) +cos (latt_2) * cos (latt_3) * cos (long_2 - long_3) )  ; 	// ���������� ����� ������� ��������������� � ��������� �������� �� ������� �����. ��������� sin � �������� . �������� � ������
		}


//-------------------------------------- LORA TX --------------------------------------------

		if ( flagTX == 1)  // ������ ���� ������� ��������
			{
				flagTX = 0;

		 // --- ��������� ��������� ������  ��� �������� ������� ------
				structBuffer[NumberDev].Number = NumberDev;  		// U8 ���� Number (����� �������) ��������� structBuffer � ������� ������� �������
				structBuffer[NumberDev].NS_EW = 0x00;				// U8 ������� ������-�����, �����-��. �������������� �����
				structBuffer[NumberDev].Vacc = VaccAvg; 			// U16 ���������� ������������ ������� �������
				structBuffer[NumberDev].TimeSec = SecRTC;			// U32 ������� ����� � �������� c ������ �����
				structBuffer[NumberDev].Longitude = GPSLongitude;	// U32 ������� �����
				structBuffer[NumberDev].Latitude = GPSLatitude;		// U32 ������ �����
				structBuffer[NumberDev].Altitude = GPSAltitude;		// U32 ������ �����
				structBuffer[NumberDev].reserv = Fix;				// U32 ������ �������� Fix - ����� ������ GPS ��������� 1- ��� �������, 2- 2D, 3- 3D

				for (i = 0; i < 10; i ++ )  // ��������� ����� ��������. 10 �� 24 �����
				{
					LORAbufferTX[24 * i + 0] = structBuffer[i].Number;			// 8 ���
					LORAbufferTX[24 * i + 1] = structBuffer[i].NS_EW;			// 8 ���
					LORAbufferTX[24 * i + 2] = structBuffer[i].Vacc >> 8;		// 16 ���
					LORAbufferTX[24 * i + 3] = structBuffer[i].Vacc;
					LORAbufferTX[24 * i + 4] = structBuffer[i].TimeSec >> 24;	// 32 ����
					LORAbufferTX[24 * i + 5] = structBuffer[i].TimeSec >> 16;
					LORAbufferTX[24 * i + 6] = structBuffer[i].TimeSec >> 8;
					LORAbufferTX[24 * i + 7] = structBuffer[i].TimeSec;
					LORAbufferTX[24 * i + 8] = structBuffer[i].Longitude >> 24;	// 32 ����
					LORAbufferTX[24 * i + 9] = structBuffer[i].Longitude >> 16;
					LORAbufferTX[24 * i + 10] = structBuffer[i].Longitude >> 8;
					LORAbufferTX[24 * i + 11] = structBuffer[i].Longitude;
					LORAbufferTX[24 * i + 12] = structBuffer[i].Latitude >> 24;	// 32 ����
					LORAbufferTX[24 * i + 13] = structBuffer[i].Latitude >> 16;
					LORAbufferTX[24 * i + 14] = structBuffer[i].Latitude >> 8;
					LORAbufferTX[24 * i + 15] = structBuffer[i].Latitude;
					LORAbufferTX[24 * i + 16] = structBuffer[i].Altitude >> 24;	// 32 ����
					LORAbufferTX[24 * i + 17] = structBuffer[i].Altitude >> 16;
					LORAbufferTX[24 * i + 18] = structBuffer[i].Altitude >> 8;
					LORAbufferTX[24 * i + 19] = structBuffer[i].Altitude;
					LORAbufferTX[24 * i + 20] = structBuffer[i].reserv >> 24;	// 32 ����
					LORAbufferTX[24 * i + 21] = structBuffer[i].reserv >> 16;
					LORAbufferTX[24 * i + 22] = structBuffer[i].reserv >> 8;
					LORAbufferTX[24 * i + 23] = structBuffer[i].reserv;
				}

				LORAbufferTX[240] = NumberDev;  // � ����� ������� ��������� ����� ����������, ������� ��������
				LORAbufferTX[241] = RFPower;	// � �������� �����������
				LORAbufferTX[242] = CounterTxPacket++;			// ������� ������� todo

				Radio->SetTxPacket( LORAbufferTX, 255 ); 	// �������� ������ ( ����� ���������� ������ 243 ���� )


				counterLed1 = 800;
				OnLED1();					// �������� ������� ��������� �� 800 ��
				counterTXDisplay = 800;		// ������ TX 800 �� ��� ���� �� ���������� ����������
// ----------------------------------------- BLE -----------------------------------------------------------------
				rprintfInit(BleUsart);  // ����� � BLE NumberDev
				printf("\n\r\n\r #%d TxPacket at ", NumberDev);
				Time_Display (structBuffer[NumberDev].TimeSec);

	#ifdef DebugLORAToUART
		rprintfInit (Terminal); 	// ������������� printf (). ����� � ��������
		printf("\n\r\n\r SetTxPacket! \n\r");
	#endif
			}

//---------------------------------------------------------------------------------------------------------------------------------------------
	    LoraProcess();  	// ��������� ��������� ��������������

//		if (counterRXDisplay == 0) // ������� ���������
//		{
//			if (triggerOffLED0 == 0)
//			{
//				triggerOffLED0 = 1;
//				OffLED0(); 		// ����� ����� ��������� ����������
//			}
//		}
//		else
//		{
//			triggerOffLED0 = 0;
//		}
//
//		if (counterTXDisplay == 0) // ������� ���������
//		{
//			if (triggerOffLED1 == 0)
//			{
//				triggerOffLED1 = 1;
//				OffLED1(); 		// ����� ������� ��������� ����������
//			}
//		}
//		else
//		{
//			triggerOffLED1 = 0;
//		}


// -------------------------------------- LORA RX -------------------------------- �������� ������ �������� �� ������ ��������� �� ����������� --------------
		if ( flagRX_OK == 1) 		// �� ����������� ������ ����� �����
		{
			flagRX_OK = 0; 			// �������� ����, ������������ ����������

			for (i=0; i<10; i++) 	// ������ �������
			{
				TimeNew  = (LORAbufferRX[24 * i + 4]<<24) + (LORAbufferRX[24 * i + 5]<<16) + (LORAbufferRX[24 * i + 6]<<8) + LORAbufferRX[24 * i + 7]; // ����� � ����� �������

				// ���������� �� ������� ���������
				if (TimeNew  > TimesOld [i]) // ���� ������ ����� ������, ��
				{
					TimesOld [i] = TimeNew; // ��������� �����

					structBuffer[i].Number = 	LORAbufferRX[24 * i + 0]; // ��������� ��� ��������� � ���������
					structBuffer[i].NS_EW = 	LORAbufferRX[24 * i + 1];
					structBuffer[i].Vacc = 		(LORAbufferRX[24 * i + 2]<<8) + LORAbufferRX[24 * i + 3];
					structBuffer[i].TimeSec = 	(LORAbufferRX[24 * i + 4]<<24) + (LORAbufferRX[24 * i + 5]<<16) + (LORAbufferRX[24 * i + 6]<<8) + LORAbufferRX[24 * i + 7]; // ��� �� TimeNew
					structBuffer[i].Longitude = (LORAbufferRX[24 * i + 8]<<24) + (LORAbufferRX[24 * i + 9]<<16) + (LORAbufferRX[24 * i + 10]<<8) + LORAbufferRX[24 * i + 11];
					structBuffer[i].Latitude =  (LORAbufferRX[24 * i + 12]<<24) + (LORAbufferRX[24 * i + 13]<<16) + (LORAbufferRX[24 * i + 14]<<8) + LORAbufferRX[24 * i + 15];
					structBuffer[i].Altitude =  (LORAbufferRX[24 * i + 16]<<24) + (LORAbufferRX[24 * i + 17]<<16) + (LORAbufferRX[24 * i + 18]<<8) + LORAbufferRX[24 * i + 19];
					structBuffer[i].reserv = 	(LORAbufferRX[24 * i + 20]<<24) + (LORAbufferRX[24 * i + 21]<<16) + (LORAbufferRX[24 * i + 22]<<8) + LORAbufferRX[24 * i + 23];
				}
			}

			NumberTransmitted = LORAbufferRX[240]; 								// �� ������ ������ ������ �������
			LoraRX [NumberTransmitted].Power = LORAbufferRX[241]; 				// � ����� ��������� �� ��������
			LoraRX [NumberTransmitted].FreqError = (s32)FreqError; 				// ���������� �������� ������� ��� ������ �������
			LoraRX [NumberTransmitted].RSSI = (s16)SX1276LoRaGetPacketRssi();	// ���������� ������� �������
			LoraRX [NumberTransmitted].SNR = SX1276GetPacketSnr();				// ���������� ����������� ������-��� � ������
			LoraRX [NumberTransmitted].CounterTxP = LORAbufferRX[242]; 			// ������� ���������� ������� todo

			counterRssiReset [NumberTransmitted] = 11000; 	// ������� ������ ��� ������ LoraRssi �� ����� �� 11 ������



			LoraRssiMax = -130;  // ����� �� �������
			for (i=0; i<10; i++) // ����� ����������� ������ RSSI
			{
				if ( (LoraRX [i].RSSI != 0) && (LoraRX [i].RSSI > LoraRssiMax) )
				{
					LoraRssiMax = LoraRX [i].RSSI; 	// ����� ������������ ������

					counterMaxRssiReset = 11000; 	// ������� ������ ��� ������ LoraRssiMax �� 11 ������
				}

			}


			if (SecRTC > 0) // ���� ����� SecRTC ��� ����������
				{
					if ( TimeSecFirstRX [NumberTransmitted] == 0)		// ���� �� ����� ����� ��� �� �����������
						TimeSecFirstRX [NumberTransmitted] = SecRTC;	// ���������� ����� ������� ������ ������� �� ����������� ����� ��� ���������� PER

					CountRX [NumberTransmitted] ++ ; 	// ������� ������� ������� ���� ������� �� ������� �����
				}

			// ���������� PER ���� ��� ������ �����
			for (i=0; i<10; i++)  // �������� �� ���� ������
			{
				if ( TimeSecFirstRX [i] > 0) // ���� ��� ���� ������� �������
				{	// ��������� PER ��� ������� �� ������ � ���� � ���� 100.00 % (������ �� 100)
					PER [i] =10000 - ((u32)CountRX [i] * 10000) / ( (SecRTC - TimeSecFirstRX [i])  / 10 + 1);
				}
			}
// ------------------------------------------------------------ BLE --------------------------------------------------------------------------------------
			rprintfInit(BleUsart);  // ����� � BLE

			printf("\n\r\n\r #%d RSSI: %d dBm  FreqEr: %d Hz  SNR: %d dB",NumberTransmitted, LoraRX [NumberTransmitted].RSSI, LoraRX [NumberTransmitted].FreqError, LoraRX [NumberTransmitted].SNR);
			if (PER [NumberTransmitted] != 65535)
				printf(" PER: %d.%02d%%", PER[NumberTransmitted]/100, PER[NumberTransmitted]%100);

			printf("\n\r Time: ");
			Time_Display (structBuffer[NumberTransmitted].TimeSec);
			printf(" Latt: %d Long: %d \n\r",  structBuffer[NumberTransmitted].Latitude, structBuffer[NumberTransmitted].Longitude);

			temp3 = distance1_1_2 * 10;
			printf (" 1-2: %d.%d m ", temp3 / 10,  temp3 % 10 ); // ���������� ������

			temp3 = distance1_1_3 * 10;
			printf ("1-3: %d.%d m ", temp3 / 10,  temp3 % 10 ); // ���������� ������

			temp3 = distance1_2_3 * 10;
			printf ("2-3: %d.%d m ", temp3 / 10,  temp3 % 10 ); // ���������� ������
		}
// -------------------------------------------------------------------------------------------------------------------------------------------------------

		for ( i = 0; i < 10; i ++) // �������� �� ���� ��������� ������ RSSI
		{
			if ( counterRssiReset[i] == 0)
				LoraRX [i].RSSI = 0;  	// ���������� RSSI �� ��������� ��������

		}

		// ����� ���������� ������ ������� LORA
		if (counterMaxRssiReset == 0) // ������ ���������
		{
			LoraRssiMax = -130; // ����� �� �������
		}

		// ���������� PER ��� ���, ���� ����� �� ������ �������
		for (i=0; i<10; i++)  // �������� �� ���� ������
		{
			if ( TimeSecFirstRX [i] > 0) // ���� ��� ���� ������� �������
			{	// ��������� PER ��� ������� �� ������ � ���� � ���� 100.00 % (������ �� 100)
				PER [i] =10000 - ((u32)CountRX [i] * 10000) / ( (SecRTC - TimeSecFirstRX [i])  / 10 + 1) ;
			}
		}


	    MeasVoltage();		// ��������� ���������� ������������, ������� � �������, �������� ������

	    if (VaccAvg < VaccPowerOff)	// ���� ���������� ������������ ������ ������
	   	{
	    	NRESET_off();  	// ��������� GPS
	        OnResetBLE();	// ����� BLE
	        OnResetLORA();	// ����� LORA

	        Display2(); 	// ���������� ������������ � �������

			while ( DMA_GetFlagStatus(DMA1_FLAG_TC6) == 0  )	// ���� ���� ���������� ������� ���������� �� DMA
			{}
//OffLED1();
			// Disable the DMA1 Channel 6
			DMA_Cmd(DMA1_Channel6, DISABLE);
			// Clear the DMA Transfer complete flag
			DMA_ClearFlag(DMA1_FLAG_TC6);

			// EV8_2: Wait until BTF is set before programming the STOP
			while ((I2C1->SR1 & 0x00004) != 0x000004);
			// Program the STOP
			I2C1->CR1 |= 0x0200 ; //CR1_STOP_Set;
			// Make sure that the STOP bit is cleared by Hardware
			while ((I2C1->CR1&0x200) == 0x200);

//OnLED1();
			SSD1306_UpdateScreenDMA();	  // ������ ������ ���������� ������ �� �����������


			while (1)
			{
				ScanKeyPowerOff();	// ��������� ������� ������
				ScanKeySOS();
			}

	        STOP
	    	//OffVcc1(); 		// ��������� �������
	   	}



	    ScanKeyPowerOff();	// ��������� ������� ������
	    ScanKeySOS();

	    if (ScreenMode == ReleaseMode)
	    {
	    	switch (screen)	// ������������ ���������� �� OLED �������
				{
				case 1:
					Display1Release(); // �������� �����
					break;
				case 2:
					Display2Release(); // ���������� ����� �������
					break;
				case 3:
					Display3Release(); // ���������� GPS
					break;

				default:
					screen = 1;
				}
	    }
	    else
	    {
	    switch (screen)	// ������������ ���������� �� OLED �������
			{
			case 1:
				Display1(); // ���������� GPS
				break;
			case 2:
				Display2(); // ���������� ������������ � �������
				break;
			case 3:
				Display3(); // ���� ��������� ������� RTC
				break;
			case 4:
				Display4(); // ������� ������� �� ��������� � ���� ID S/N dB
				break;
			case 5:
				Display5(); // ������� ������� �� ��������� � ����������� ����
				break;
			case 6:
				Display6(); // ���������� � ���������, ������������ � ������� � ���������� DOP
				break;
			case 7:
				Display7(); // ����������, ������ � u32
				break;
			case 8:
				Display8(); // ��������� ������� 1PPS
				break;
			case 9:
				Display9(); // ������ � ��������� LORA
				break;
			case 10:
				Display10(); // ������ � ��������� LORA ����������
				break;
			case 11:
				Display11(); // ������ � ��������� LORA �������
				break;
			case 12:
				Display12(); // ��������� �������� �������
				break;
			case 13:
				Display13(); // PER �� ������� �� ������
				break;
			case 14:
				Display14(); // ���������� ����� �������
				break;
			case 15:
				Display15(); // ������ ������� �� ������ � ����������� ����
				break;
			}
	    }


//OnLED0();
		//if ( counterLcdOff > 0)
		//{
				// DMA end of transfer  ����������� �������� � I2C �� DMA
			if ( DMA_GetFlagStatus(DMA1_FLAG_TC6) == 1  )
			{
	//OffLED1();
				// Disable the DMA1 Channel 6
				DMA_Cmd(DMA1_Channel6, DISABLE);
				// Clear the DMA Transfer complete flag
				DMA_ClearFlag(DMA1_FLAG_TC6);

				// EV8_2: Wait until BTF is set before programming the STOP
				while ((I2C1->SR1 & 0x00004) != 0x000004);
				// Program the STOP
				I2C1->CR1 |= 0x0200 ; //CR1_STOP_Set;
				// Make sure that the STOP bit is cleared by Hardware
				while ((I2C1->CR1&0x200) == 0x200);

	//OnLED1();
				if ( counterLcdOff > 0)
					SSD1306_UpdateScreenDMA();	  // ������ ������ ���������� ������ �� �����������
				else
					SSD1306_OFF ();
			}
		//}
//OffLED0();


	} // END of MAIN LOOP
//=====================================================================================================
}

/************************************************************************************************************************************************************************
*************************************************************/
// ----------- ������������� ����������������, ���������
void init				(void)
{
	RCC_Configuration(); 	// ��������� ����� ������������
	SysTick_Config(72000-1);// ��������� ������ ������ ������ ��
	PortIO_Configuration();	// ��������� ������
OnVcc1();		// ���������� 1 �� �������� ��������������
	USART1_Configuration();	// ��������� USART1
	USART2_Configuration();	// ��������� USART2
	USART3_Configuration();	// ��������� USART3
	TIM1_Configuration();	// ��������� Timer1 - ������ ��������� ������ ������������� ��� ���������� ���������� ������� LORA ����. ���������������� �� 1PPS �� GPS ������
	TIM2_Configuration();	// ��������� Timer2 - ������ ��������� ������� ��� ������ ������� �� usart2
    TIM3_Configuration();	// ��������� Timer3 - ������ ��������� ������� ��� ������ ������� �� usart3
    TIM4_Configuration();	// ��������� Timer4 - ������ �������� ������ ������� 1PPS, ��� ������������� LORA ���� � ��������� ������� 1PPS �� GPS ������
    SPI1_Configuration();	// ��������� SPI1
	NVIC_Configuration();	// ��������� ����������
	ADC1_Configuration();	// ��������� ���
	EXTI_Configuration();	// ��������� �������� ���������� �� PB12
	DMA_Configuration ();	// ��������� DMA ��� I2C �������

	DelayMS(100);			// ����� ��� ���������� ������������� �������

	SSD1306_Init();  		// ������������� �������

//DelayMS(200);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // ��������� ���������� �� ������ �����
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // ��������� ���������� �� ������ �����
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // ��������� ���������� �� ������ �����

	rprintfInit (Terminal); // ������������� printf (). ����� � ��������
	printf("\n\r\n\r Power ON ! ");
	printf("\n\r RESET! USART1\n\r");

	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_Fill(SSD1306_COLOR_BLACK);

	SSD1306_GotoXY(4, 53);
	printf("#%d", NumberDev);
	SSD1306_GotoXY(63, 53);
	printf(" SW v%d.%d", VersionMajor, VersionMinor);

	SSD1306_UpdateScreen();

	InitMeasVoltage();

/*	SSD1306_GotoXY(0, 0);
	printf(" %d", Vref[0]);
	SSD1306_GotoXY(0, 10);
	printf(" %d", Vref[1]);
	SSD1306_GotoXY(0, 20);
	printf(" %d", Vacc[2]);
	SSD1306_GotoXY(0, 30);
	printf(" %d", Vref[3]);
	SSD1306_GotoXY(0, 40);
	printf(" %d", Vref[4]);
	SSD1306_GotoXY(0, 50);
	printf(" %d", Vref[5]);
	SSD1306_UpdateScreen();

//	STOP

	DelayMS (4000);
*/

	rprintfInit (OLED_font2); 	// ����� �� OLED �������
	SSD1306_GotoXY(3, 7);

	for (j=0; j<1; j++) // 20 ������ ��������� ���������� ������������ � Vcc
	{
		MeasVoltage();
		OffLED1();// �������

		SSD1306_GotoXY(32, 0);
		printf("%d.%02d V ", VaccAvg/1000, (VaccAvg%1000)/10 ); // ���������� � ��������� 10 ��

		// ���������
		u8 X_bat = 32;
		u8 Y_bat = 21;
		SSD1306_DrawRectangle(0+X_bat, 0+Y_bat, 59, 25, SSD1306_COLOR_WHITE );
		SSD1306_DrawFilledRectangle(60+X_bat, 4+Y_bat, 2, 17, SSD1306_COLOR_WHITE );

		if (VaccAvg >= 3500)	SSD1306_DrawFilledRectangle(X_bat+3, 			Y_bat+3, 11, 19, SSD1306_COLOR_WHITE );
		if (VaccAvg >= 3690)	SSD1306_DrawFilledRectangle(X_bat+3+14,		 	Y_bat+3, 11, 19, SSD1306_COLOR_WHITE );
		if (VaccAvg >= 3780)	SSD1306_DrawFilledRectangle(X_bat+3+14+14, 		Y_bat+3, 11, 19, SSD1306_COLOR_WHITE );
		if (VaccAvg >= 3850)	SSD1306_DrawFilledRectangle(X_bat+3+14+14+14, 	Y_bat+3, 11, 19, SSD1306_COLOR_WHITE );
		SSD1306_UpdateScreen();
	}

	DelayMS(2000);

	if (VaccAvg < VaccAttention) // ���������� � ������� ����������
	{
		OffLED0();
		OnLED1();
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		SSD1306_GotoXY(4, 5);
		SSD1306_Puts("LOW BATTERY", &Font_11x18, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(58, 27);
		SSD1306_Puts("!", &Font_16x26, SSD1306_COLOR_WHITE);

		rprintfInit (OLED_font1); 	// ����� �� OLED �������
		SSD1306_GotoXY(4, 53);
		printf("%d mV ", VaccAvg);

		SSD1306_UpdateScreen();

		DelayMS(4000);

		OffVcc1();		// ��������� �������

		STOP  // ����������� ����, ���� ���� �������� ������
	}


//SSD1306_DrawRectangle(0, 0, 127, 64, SSD1306_COLOR_WHITE );		// �����
//OnLED0();
//	SSD1306_UpdateScreenDMA();
////OffLED0();
////OnLED1();
//// DMA end of transfer  ����������� �������� � I2C �� DMA
//while ( DMA_GetFlagStatus(DMA1_FLAG_TC6) == 0  )
//{
//}
////{
////OffLED1();
//// Disable the DMA1 Channel 6
//DMA_Cmd(DMA1_Channel6, DISABLE);
//// Clear the DMA Transfer complete flag
//DMA_ClearFlag(DMA1_FLAG_TC6);
//
//// EV8_2: Wait until BTF is set before programming the STOP
//while ((I2C1->SR1 & 0x00004) != 0x000004);
////OnLED0();
//// Program the STOP
//I2C1->CR1 |= 0x0200 ; //CR1_STOP_Set;
//// Make sure that the STOP bit is cleared by Hardware
//while ((I2C1->CR1&0x200) == 0x200);

//OffLED0();
//STOP

	//DelayMS(2000);
	OffVcc1();		// ��������� �������
	DelayMS(10);	// ����� ����� ����� �����������

/*	for ( i=0; i<100; i=i+2 ) // �������� ���������
	{
		SSD1306_DrawFilledRectangle(14, 40, i, 5, SSD1306_COLOR_WHITE ); 	// ���������
		SSD1306_DrawRectangle(12, 38, 102, 9, SSD1306_COLOR_WHITE );		// �����
		SSD1306_DrawLine(13+20, 40, 13+20, 45 ,SSD1306_COLOR_BLACK);		// �������
		SSD1306_DrawLine(13+21, 40, 13+21, 45 ,SSD1306_COLOR_BLACK);
		SSD1306_DrawLine(13+40, 40, 13+40, 45 ,SSD1306_COLOR_BLACK);
		SSD1306_DrawLine(13+41, 40, 13+41, 45 ,SSD1306_COLOR_BLACK);
		SSD1306_DrawLine(13+60, 40, 13+60, 45 ,SSD1306_COLOR_BLACK);
		SSD1306_DrawLine(13+61, 40, 13+61, 45 ,SSD1306_COLOR_BLACK);
		SSD1306_DrawLine(13+80, 40, 13+80, 45 ,SSD1306_COLOR_BLACK);
		SSD1306_DrawLine(13+81, 40, 13+81, 45 ,SSD1306_COLOR_BLACK);

		SSD1306_UpdateScreen();
		//DelayMS(1);
	}*/
//OffLED0();
	OnVcc1();		// ���������� 1 �� �������� ��������������

	OnLED0(); 		// ��� ����� ���������. ����� ��������

	if ( GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 0 )  		// ���� ������ SCREEN ������ �� ����� ��������� �������
		{
		ScreenMode = ReleaseMode; 	// ����������� ������ �������
		}


	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("Batt check..  ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();

//	SSD1306_GotoXY(0, 10);
//	SSD1306_Puts("Off  RTC....  ", &Font_7x10, SSD1306_COLOR_WHITE);
//	SSD1306_UpdateScreen();

   // DelayMS(100);
    SSD1306_Puts("OK", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();

// ------------------------------ GPS -----------------------------
	NRESET_on();  // �������� GPS

	rprintfInit (Terminal); // ������������� printf (). ����� � ��������
	printf("\n\r GPS power ON  ... OK");

	SSD1306_GotoXY(0, 10);
	SSD1306_Puts("GPS setup...  ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();

	DelayMS(800); // ����� ��� �������� GPS ������

/////////////// test GPS boudrate ////////////////
	ClearGPSBuffer();  				// ������� ��������� ������ GPS

	rprintfInit (GpsUsart);  		// ����������� ����� �� USART3 - � ������ GPS
	printf("$PMTK000*32\r\n\n"); 	// ������� TEST , ����� ������ ���� $PMTK001,0,3*30\r\n

	rprintfInit (Terminal); 		// ������������� printf (). ����� � ��������
	printf("\n\r GPS TEST 9600 ...  \n\r");
	DelayMS(200); 					// �������� �� ����� �� GPS ������

//if ( triggerTIM3Interrupt == 1 ) 		// ��������  - ������ ������� �� GPS ������
//{
	for (i=0; i < 20; i++) 	// ����� ������ � ��������
		{
		if (GPSBuf[i] == '\0' || GPSBuf[i] == '\r' || GPSBuf[i] == '\n') break; 	// ������� ������ �� ������� 0 ��� \r � ������
		USART_SendByte(USART1, GPSBuf[i]);
		}

	if (GPSBuf[0] == '$' && GPSBuf[1] == 'P' && GPSBuf[2] == 'M' && GPSBuf[3] == 'T' && GPSBuf[4] == 'K' && GPSBuf[5] == '0'
			&& GPSBuf[6] == '0' && GPSBuf[7] == '1' && GPSBuf[8] == ',' && GPSBuf[9] == '0' && GPSBuf[10] == ','
					&& GPSBuf[11] == '3' && GPSBuf[12] == '*' && GPSBuf[13] == '3' && GPSBuf[14] == '0' ) // �������� 9600
		{
			printf("\tOK  ");
//			printf("\n\r Set to 115200 ");
//			rprintfInit (GpsUsart);  				// ����������� ����� �� USART3 - � ������ GPS
//			printf("$PMTK251,115200*1F\r\n\n"); 	// ��������� �������� USART - 115200 ���
//			//printf("$PMTK251,9600*17\r\n\n);	//	9600 ���
//			DelayMS(100);
		}
	else
		printf("\tNO! ");

	printf("\n\r Set to 115200 ");          // �������������� ������������, �������� �������� �� ������
	rprintfInit (GpsUsart);  				// ����������� ����� �� USART3 - � ������ GPS
	printf("$PMTK251,115200*1F\r\n\n"); 	// ��������� �������� USART - 115200 ���
	DelayMS(100);

	rprintfInit (Terminal); 		// ������������� printf (). ����� � ��������
	printf("\n\r GPS TEST 115200..\n\r ");

	USART3_Configuration115200();  	// ����������� �������� USART3 �� 115200

	ClearGPSBuffer();  				// ������� ��������� ������ GPS

	rprintfInit (GpsUsart);  		// ����������� ����� �� USART3 - � ������ GPS
	printf("$PMTK000*32\r\n\n"); 	// ������� TEST , ����� ������ ���� $PMTK001,0,3*30\r\n

	DelayMS(200); 					// �������� �� ����� �� GPS ������
	rprintfInit (Terminal); 		// ������������� printf (). ����� � ��������

	for (i=0; i < 20; i++) 	// ����� ������ � ��������
		{
		if (GPSBuf[i] == '\0' || GPSBuf[i] == '\r' || GPSBuf[i] == '\n') break; 	// ������� ������ �� ������� 0 � ������
		USART_SendByte(USART1, GPSBuf[i]);
		}

	if (GPSBuf[0] == '$' && GPSBuf[1] == 'P' && GPSBuf[2] == 'M' && GPSBuf[3] == 'T' && GPSBuf[4] == 'K' && GPSBuf[5] == '0'
			&& GPSBuf[6] == '0' && GPSBuf[7] == '1' && GPSBuf[8] == ',' && GPSBuf[9] == '0' && GPSBuf[10] == ','
					&& GPSBuf[11] == '3' && GPSBuf[12] == '*' && GPSBuf[13] == '3' && GPSBuf[14] == '0' ) // �������� 115200
		printf("\tOK  ");
	else
		printf("\tNO! ");

	//	triggerTIM3Interrupt = 0; 	// ���������� ���� ������� ������ � ������
	//}

	ClearGPSBuffer();  				// ������� ��������� ������ GPS

    printf("\n\r GPS setup    ...\t");



    rprintfInit ( GpsUsart );  				// ����������� ����� �� USART3 - � ������ GPS

    printf("$PMTK300,1000,0,0,0,0*1C\r\n\n"); 	// ��������� �� ������ ������� �� GPS 1 ���
    //printf("$PMTK300,10000,0,0,0,0*2C"); 	// ��������� �� ������ ������� �� GPS 10 ���
    DelayMS(50);			// ����� ����� ��� ������ �� ���������� ������� todo �������� �������� ������

    //printf("$PMTK353,1,0*36\r\n\n"); 	// ������ GPS
    //printf("$PMTK353,0,1*36\r\n\n"); 	// ������ GLONASS
    printf("$PMTK353,1,1*37\r\n\n"); 	// GPS � GLONASS
    DelayMS(70);			// ����� ����� ��� ������ �� ���������� ������� todo �������� �������� ������

    printf("$PMTK314,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,0*28\r\n\n"); // ��������� ��������� ���������� -  GGA, GSA, GSV, ZDA
    // printf("$PMTK314,-1,*04 	// ����� �������� �� ���������

    DelayMS(70);			// ����� ����� ��� ������ �� ���������� ������� todo �������� �������� ������

    rprintfInit ( Terminal );	// ����� � ��������
    printf("OK \r\n");
	SSD1306_Puts("OK ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
    DelayMS(500);


/*    printf("\r\n START test 1PPS: \r\n");
for (;;) // ���� ��������� ������� 1PPS
{
	rprintfInit ( Terminal );	// ����� � ��������

	if ( flag_1PPS_Update == 1)
	{
		trigger_1PPS_Update = 0;

		printf("Count: %d\tTIM4: %d \r\n", counter_TIM4_fix, TIM4_fix);
	}

	if ( flag_1PPS_timeout == 1)
	{
		flag_1PPS_timeout = 0;

		printf("1PPS timeout ! \r\n");
	}

	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_GotoXY(0, 0);
	printf ("count: %d ", counter_TIM4_fix);
	SSD1306_GotoXY(0, 10);
	printf ("TIM4 : %d ", TIM4_fix);

	SSD1306_UpdateScreen();

} // STOP HERE*/
//////////////////////////////////////////

// ---------------- BLE -------------------------------
    rprintfInit ( Terminal );	// ����� � ��������
    printf(" BLE setup     ...");
    SSD1306_GotoXY(0, 20);
	SSD1306_Puts("BLE setup...  ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();

    OnResetBLE();	// �����
    DelayMS(50);
    OffResetBLE(); 	// ��������� BLE ������
    DelayMS(500);


    ClearBLEBuffer();		// ������� ��������� ������

	DelayMS(20);
	rprintfInit ( BleUsart ); // ����� � USART BLE ������

    printf("AT+BAUD8");		// ��������� �������� UART ���������� BLE ������ �� 115200. BAUD4-9600
    DelayMS(1);
    USART2_SendByte(0x0D); 	// \r\n
    USART2_SendByte(0x0A); 	//

    DelayMS(200);
    for (i=0; i < BufferUSART2Size; i++)
		{
		if (BufferUSART2RX[i] == '\0') break;
		USART_SendByte(USART1, BufferUSART2RX[i]); // terminal
		}
//STOP
    USART1_SendByte(0x0D); 	// \r\n
    USART1_SendByte(0x0A); 	//

    DelayMS(20);
    USART2_Configuration115200();  // ����������� �������� USART2 �� 115200

    ClearBLEBuffer();		// ������� ��������� ������

    DelayMS(20);
    printf("AT+NAMEtrack%d\r\n\n", NumberDev);		// ��������� �������� ������
    DelayMS(100);

    for (i=0; i < BufferUSART2Size; i++)
		{
		if (BufferUSART2RX[i] == '\0') break;
		USART_SendByte(USART1, BufferUSART2RX[i]); // terminal
		}

    SSD1306_Puts("OK ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
    rprintfInit ( Terminal );	// ����� � ��������
    printf(" BLE setup   ...\tOK\r\n");
//STOP
//    if (BufferUSART2RX[0] == 'O' && BufferUSART2RX[1] == 'K')
//    {
//        rprintfInit ( Terminal );	// ����� � ��������
//        printf("OK \r\n");
//    	SSD1306_Puts("OK ", &Font_7x10, SSD1306_COLOR_WHITE);
//    	SSD1306_UpdateScreen();
//        DelayMS(500);
//    }
//    else
//    {
//		rprintfInit ( Terminal );	// ����� � ��������
//		printf("ER! \r\n");
//		SSD1306_Puts("ER! ", &Font_7x10, SSD1306_COLOR_WHITE);
//		SSD1306_UpdateScreen();
//		DelayMS(5000);
//    }

//    for (;;) // test SEND to BLE USART
//    	{
//			temp1 ++;
//
//	//    	DelayMS(5000);
//	//      rprintfInit(BleUsart);
//	//    	printf(" BLE test #%d \n\r", temp1);
//
//			if (triggerUSART1Interrupt == 1 ) // ������ ���� �� ���������
//			{
//				DelayMS(100); // �����, ���� ��� �������
//
//				for (i=0; i < BufferUSART1Size; i++) // ������� �� ��������� � BLE
//					{
//					if (BufferUSART1RX[i] == '\0') break;
//					USART_SendByte(USART1, BufferUSART1RX[i]); // terminal ���
//					USART_SendByte(USART2, BufferUSART1RX[i]); // to BLE
//					}
//
//				counterUSART1Buffer = 0;
//				triggerUSART1Interrupt = 0;
//			}
//
//			if ( triggerTIM2Interrupt == 1 ) 	// ��������  - ������ ������� �� BLE ������
//			{
//			OnLED0();
//			//CplLED0();
//				for (i=0; i < BufferUSART2Size; i++)
//					{
//					if (BufferUSART2RX[i] == '\0') break;
//					USART_SendByte(USART1, BufferUSART2RX[i]); // terminal
//					//USART_SendByte(USART2, BufferUSART2RX[i]); // back
//					}
//
//				triggerTIM2Interrupt = 0; 	// ���������� ���� ������� ������ � ������
//			OffLED0();
//			}
//
//    	}


// --------------------------------------------- LORA -------------------------------
    //OffResetLORA 	();			//  RESET LORA � 1 ��������� ������

    rprintfInit ( Terminal );	// ����� � ��������
    printf(" LORA setup  ...\t");
    SSD1306_GotoXY(0, 30);
	SSD1306_Puts("LORA setup..  ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();

	Radio = RadioDriverInit();  // ������������� ��������� , ����������� �������
	Radio->Init( );				// ������������� ���������������, ��������� ��������� , ������� � ��

	SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST ); // �������� ����� PABoost
	//SX1276LoRaSetPa20dBm( true );
	SX1276LoRaSetRFFrequency(434000000);	// �������, ��
	SX1276LoRaSetRFPower( RFPower );		// ��������, ���
	//SX1276LoRaSetSpreadingFactor ( 11 );
	SX1276LoRaSetPreambleLength( 8 ); 		// ������ ��������� 6-65535 8+14 = 12 �������� ��������� (��� �� ��������� LoRA)

//	for (i=0; i<256; i++)					// ������� ������ �������� LORA ����������
//	{
//		LORAbuffer[i]= 0;
//	}

	SX1276Write ( REG_LR_MODEMCONFIG3, 0b00000100 ); // todo ALC on
	SX1276Write ( REG_LR_LNA, 0b00000000 );

	DelayMS(100);
    rprintfInit ( Terminal );	// ����� � ��������
    printf("OK \r\n");
	SSD1306_Puts("OK ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
    DelayMS(500);

//OnLED0();
    rprintfInit ( Terminal );	// ����� � ��������
 /*   Radio->Process( );
    Radio->Process( );
    Radio->Process( );
    //printf("RFLRState = %d\n\r", RFLRState);
    printf("SetTxPacket!\n\r");

OnLED0();
Radio->SetTxPacket( LORAbuffer, 255 ); // �������� ������
*/
//for (;;)
//	 Radio->Process( );
//STOP
    for (i = 0; i< 10; i++)  // ��������� ������ PER - �������������� ��������
    {
    	PER [i] = 65535;
    }

////////////////////////////////////////////////////////// TX test /////////////////////////////////////////////////////////////////
/*	for (i=0; i<256; i++)
	{
		LORAbuffer[i]= 1;
	}

	for(;;) // Transmit LOOP
		{
			//CplLED0();
			Radio->Process( );
			if (RFLRState != 0)
				//OnLED0();
				//printf("RFLRState = %d\n\r", RFLRState);
			if (RFLRState == RFLR_STATE_TX_DONE)
			{
				OffLED0();

				DelayMS(3000);

				temp1 ++ ;
				for (i=0; i<256; i++)					// ������� ������ ������ LORA ����������
				{
					LORAbuffer[i]= temp1;
				}

//				i = i + 3;
//				if (i >=18 )
//					i=2;
//
//				countPacket ++;
//				Power = i;
//				LORAbuffer [0] = countPacket>>8;
//				LORAbuffer [1] = countPacket;
//				LORAbuffer [2] = 0;
//				LORAbuffer [3] = Power;
//				LORAbuffer [4] = 0;

//				SX1276LoRaSetRFPower( Power );				// ��������, ���

//				printf("TX#%03d, power = %02d dBm\n\r", countPacket, SX1276LoRaGetRFPower() );
				Radio->SetTxPacket( LORAbuffer, 255 ); 	// �������� ������
				OnLED0();
			}
		}//STOP
*/
////////////////////////////////////////////////////////// RX test ///////////////////////////////////////////////////////////////////////////////////////

/*

SX1276LoRaSetPreambleLength( 28 ); // 6-65535


for (i=2; i<256; i++)
{
	LORAbuffer2[i]= i;
}

SX1276Write ( REG_LR_MODEMCONFIG3, 0b00000100 ); // todo ALC on
SX1276Write ( REG_LR_LNA, 0b00000000 );

 Radio -> StartRx();
// SX1276LoRaSetRFState( RFLR_STATE_RX_INIT );
// Radio -> Process();
 printf(" StartRx \n\r");


 printf(" 0x26: 0x%02X  \n\r", LoraReadReg(0x26) );
 printf(" 0x0C: 0x%02X  \n\r", LoraReadReg(0x0C) );

// OnTest();
 trigger_PER = 0;
/////////////////////////////
 while (1) // Receive
 {
 	 rprintfInit ( Terminal );	// ����� � ��������
     DelayMS(1);
	 switch ( Radio -> Process())
	 {
	 case RF_BUSY:
		 break;
	 case RF_RX_TIMEOUT:
		 printf(" RF_RX_TIMEOUT \n\r");
		 break;
	 case RF_RX_DONE:
		 Radio -> GetRxPacket( LORAbuffer, (uint16_t*)&BufferSize );

		 countOK ++; // ������� ������ ���������� �������

		 if (CRCerror == 1) // ������ CRC
		 {
			 CRCerror = 0;
			 printf(" CRC ER! " );

			 countERR ++;

//				 if (trigger_PER == 0)
//					 {
//					 countERR = 0;
//					 countOK = 0;
//					 PER = 0;
//					 trigger_PER = 1;
//					 }
		 }

		 PER = (countERR * 10000 / countOK) ;
		 PER1 = PER / 100;
		 PER2 = PER % 100;

//			 printf(" 0x%02X", LoraReadReg(0x14) );
//			 printf("%02X ", LoraReadReg(0x15) );
//			 printf(" 0x%02X", LoraReadReg(0x16) );
//			 printf("%02X  ", LoraReadReg(0x17) );
//			 printf("Freq Err=0x%02X", LoraReadReg(0x28) );
//			 printf("%02X", LoraReadReg(0x29) );
//			 printf("%02X ", LoraReadReg(0x2A) );

		 FREQ_ERROR =( (LoraReadReg(0x28) << 28) + (LoraReadReg(0x29) << 20) + (LoraReadReg(0x2A) << 12) ) ;

		// FreqError =  ((FREQ_ERROR * (2^24) ) / 32e6 ) * (BW / 500) ; // ������ ������ �� �������
		 FreqError = (FREQ_ERROR/4096) / 3.815;


		// printf(" 0x10: 0x%02X ", LoraReadReg(0x10) );
		// printf(" 0x0D: 0x%02X  \n\r", LoraReadReg(0x0D) );
		 printf(" TX#%03d  P:%02d dBm ", LORAbuffer[0], LORAbuffer[1]);
		// printf("\n\r RSSI: %d dBm", (s32)(SX1276LoRaReadRssi()) );
		 printf(" RSSI:%d dBm ", (s32)(SX1276LoRaGetPacketRssi()) );
		 //printf("\n\r Preamble:  %d\n\r",  SX1276LoRaGetPreambleLength () );
		// printf(" Gain: %d ", SX1276GetPacketRxGain() );
		 printf(" SNR:%d dB ", SX1276GetPacketSnr() );

		// printf(" Preamble: %d ",  SX1276LoRaGetPreambleLength () );
		// printf(" NbTrigPeaks: %d \n\r",  SX1276LoRaGetNbTrigPeaks () );

		 printf(" PER:%02d.%02d%% ", PER1, PER2 );
		 printf(" %d Hz", (s32)FreqError );
		// printf(" err=%d summ=%d ", countERR, countOK );

		 countPacket = LORAbuffer[0];
		// printf("\n\r new=%d old=%d old+1=%d  ", countPacket, countPacketOld, (u8)(countPacketOld+1) );
		 if ( countPacket != (u8)(countPacketOld+1) )
		 	 {
			 printf(" c.ER! " );
			 countPacketOld = LORAbuffer[0];
		 	 }
		 else
			 printf(" c.OK! " );

		 printf(" Data." );
		 countPacketOld = LORAbuffer[0];

		 LORAbuffer[0]=0; LORAbuffer[1]=0;
		 LORAbuffer2[0]=0; LORAbuffer2[1]=0;
		 if( strncmp( ( const char* )LORAbuffer, ( const char* )LORAbuffer2, 255 ) == 0 ) // ���������� ������
			 {
			 printf("OK! \n\r" );
			 }
		 else
			 {
			 printf("ER! \n\r" );
			 }

		 break;
	 }
	 Display9	();
 }
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	DelayMS(400);

	ClearGPSBuffer();			// ������� ��������� ������ GPS
	triggerTIM3Interrupt = 0; 	// ���������� ���� ������� ������ � ������
//OffLED1();

	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_DrawRectangle(0, 0, 127, 64, SSD1306_COLOR_WHITE );		// �����
//OnLED1();
	SSD1306_UpdateScreenDMA();
	counterLcdOff = LcdOffTimeout; // ������� ������

OffLED0();
}

// ----------- ������������ ������� ������ POWER OFF / SCREEN
void ScanKeyPowerOff 	(void)
{
		if ( GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 0 )  		// ������ ������
		{
			DelayMS(10);											// �������� ������ ��������
			if ( GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 0 ) 	// �������� ��� ���, ������������� ���� �������
			{
				if(trigger1 == 0) 									// ������� ��� ������������ �������
				{
					trigger1 = 1;

					if (counterLcdOff == 0) // ����� ��������
					{
						counterLcdOff = LcdOffTimeout; // ������� ������
						// �������� �����
						SSD1306_ON ();
						SSD1306_UpdateScreenDMA();	  // ������ ������ ���������� ������ �� �����������

					}
					else	// ����� �������
					{
						counterLcdOff = LcdOffTimeout; // ������� ������

						screen ++;
						if (screen > 15)
							screen = 1;
					}
				}
				else
				{
					counterPowerOff -- ;	// ������� �������� ���������� �������
					if (counterPowerOff == 0)
					{
						OffVcc1(); 			// ��������� �������
					}

				}


			}
		}
		else 									// ������ ��������
		{
			trigger1 = 0; 						// ����� �������� ������������ �������
			counterPowerOff = PowerOffTimeout; 	// �������� �� ���������� ������� �������� 3 ���
		}
}

// ----------- ������������ ������� ������ SOS
void ScanKeySOS 		(void)
{
	if ( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0 )  		// ������ ������
	{
		DelayMS(10);											// �������� ������ ��������
		if ( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0 ) 	// �������� ��� ���, ������������� ���� �������
		{
			if(trigger2 == 0) 									// ������� ��� ������������ �������
			{
				trigger2 = 1;


			}
		}
	}
	else 														// ������ ��������
	{
		trigger2 = 0; 											// ����� �������� ������������ �������
	}

}

// ----------- �������� ������� ���1
u16  GetSamplesADC1		(void) // ������� ���
{
	while(ADC_GetFlagStatus( ADC1, ADC_FLAG_EOC) == RESET ); // ���� ���� ����� ��������������
	ADC_ClearFlag( ADC1, ADC_FLAG_EOC );		// ���������� ����
	return ( ADC_GetConversionValue(ADC1) );
}


void InitMeasVoltage		(void)
{
	u16 ii;	//�������

// �������, ��� ����� ������ ��������� adcIntRef �� ����������  = 4095
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1, ADC_SampleTime_239Cycles5 ); // ���������� �� ����� ��� ���������� �������� �������� ���������� 1.2 �
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);								// start ONE conversion
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);				// wait end of conversion
	adcIntRef = ADC_GetConversionValue(ADC1)  ;	// ��� ���, ��������������� ���������� �� ����� 1.2 � ��� ������� ������� ���������� ��� (��� �� ���������� �������)

// �������� ���������� ������� ���������� 1.2 � todo ��������� ��������� ������ �������
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1, ADC_SampleTime_239Cycles5 ); // ���������� �� ����� ��� ���������� �������� �������� ���������� 1.2 �

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);								// start ONE conversion
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);				// wait end of conversion

	adcIntRef = ADC_GetConversionValue(ADC1)  ;	// ��� ���, ��������������� ���������� �� ����� 1.2 � ��� ������� ������� ���������� ��� (��� �� ���������� �������)

// �������� ���������� ������������
	OnDivGND1();  // �������� ������������ �� �����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 ); // ���� 0 - ���������� ������������ ����� ��������

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);								// start ONE conversion
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);				// wait end of conversion

	adc1 = ADC_GetConversionValue(ADC1);								// get value
	OffDivGND1();  // �������� ������������ � �������

// ���������� ����������
	Vacc1 = ( 4030 * 1200 * adc1 / adcIntRef ) / 1000;		// 4030/1000 - � ������ �������� ����������

	Vref1 = ( 4095 * 1200 / adcIntRef );

	for (ii=0; ii<Average1; ii++) // ���������� ������� ����������� ����������
	{
		Vacc[ii] = Vacc1;
		Vref[ii] = Vref1;
	}

	Vref[4] = adcIntRef;
	Vref[5] = adc1 ;
}


// ----------- ��������� ���������� ������������ � �������
void MeasVoltage		(void)
{
	u16 ii;	//�������
// �������� ���������� ������� ���������� 1.2 � todo ��������� ��������� ������ �������
		ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1, ADC_SampleTime_239Cycles5 ); // ���������� �� ����� ��� ���������� �������� �������� ���������� 1.2 �
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);								// start ONE conversion
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);				// wait end of conversion
		adcIntRef = ADC_GetConversionValue(ADC1)  ;	// ��� ���, ��������������� ���������� �� ����� 1.2 � ��� ������� ������� ���������� ��� (��� �� ���������� �������)

// �������� ���������� ������������
		OnDivGND1();  // �������� ������������ �� �����
		ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 ); // ���� 0 - ���������� ������������ ����� ��������
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);								// start ONE conversion
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);				// wait end of conversion
		adc1 = ADC_GetConversionValue(ADC1)  ;								// get value
		OffDivGND1();  // �������� ������������ � �������

// ���������� ����������
		Vacc1 = ( 4030 * 1200 * adc1 / adcIntRef ) / 1000;		// 4030/1000 - � ������ �������� ����������
		Vref1 = ( 4095 * 1200 / adcIntRef );
// ���������� ���������� �������
		for (ii=Average1-1; ii>=1; ii--) // ����� ����
		{
			Vacc[ii] = Vacc[ii-1];
			Vref[ii] = Vref[ii-1];
		}
		Vacc[0] = Vacc1;	// ������� �������� - � ������� ������� �������
		Vref[0] = Vref1;

		VaccSumm = 0;
		VrefSumm = 0;
		for (ii=0; ii<Average1; ii++) //����������� �����
		{
			VaccSumm += Vacc[ii];
			VrefSumm += Vref[ii];
		}
		VaccAvg = VaccSumm / Average1; // ��������� �������
		VrefAvg = VrefSumm / Average1;

		VrefDev = VrefAvg * 100 / 3300 - 100; // ���������� �� �������� � %
		VaccDev = VaccAvg * 100 / 3700 - 100; // todo ���������� 2.7V = 0%, 3,7V = 100%

		if ( (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0) && (ChargeTrigger == 0) )  		// ������� ����� ������������
		{
			ChargeTrigger = 1;
			OnLED1();				// ����������� ��������� �������� ����������
		}
		if ( (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 1) && (ChargeTrigger == 1) )  		// ����� ����������
		{
			ChargeTrigger = 0;
			OffLED1();				// ����������� ���������� �������� ����������
		}

//		if ( flag_1PPS_AVG_OK == 1 ) // ���� �������� ����� � ����� ����������
//		{
//			if ( triggerTimeStartLogVcc == 0)
//				{
//				triggerTimeStartLogVcc = 1;
//				TimeStartLogVcc = GPSTimeSec + 1;	// ���������� ����� ������ ������ ���� ���������� ������������
//				}
//		}

		// ������ � ���
		//TimeLogVcc = SecRTC - TimeStartLogVcc ;  // �����, ��������� � ������ ������ ����
		TimeLogVcc = timer1s;	// �����, ��������� �� ��������� �������

		if ( TimeLogVcc != SecRTCold)
		{
			SecRTCold = TimeLogVcc ;

			if ( TimeLogVcc % 600 == 0 ) // ����� ������� �� 10 ��� ��� �������
			{
				VaccLog [counterVaccLog] = (VaccAvg - 3000) / 25;	// �������� � ������� ������� ( ����� 48 ������� �� 25 �� )
				counterVaccLog ++ ;
				if (counterVaccLog > 127)
					counterVaccLog = 127;
			}
		}


}

// ������� �� ������� ������� � �����
void Display1	(void)
{
	u8 X_bat, Y_bat;
	u8 X_gps_ico, Y_gps_ico;
	u8 X_gps_lev, Y_gps_lev;
	u8 X_lora_ico, Y_lora_ico;
	u8 X_lora_lev, Y_lora_lev;

	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_Fill(SSD1306_COLOR_BLACK);

	// ���������
	X_bat = 111;
	Y_bat = 0;
	SSD1306_DrawRectangle(0+X_bat, 0+Y_bat, 14, 7, SSD1306_COLOR_WHITE );
	SSD1306_DrawFilledRectangle(15+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
	if (VaccAvg >= 3500)	SSD1306_DrawFilledRectangle(2+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
	if (VaccAvg >= 3690)	SSD1306_DrawFilledRectangle(X_bat+2+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
	if (VaccAvg >= 3780)	SSD1306_DrawFilledRectangle(X_bat+2+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
	if (VaccAvg >= 3850)	SSD1306_DrawFilledRectangle(X_bat+2+3+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
	//////////////////////////////////////////////////////////////////////

	// GPS
	X_gps_ico=0;
	Y_gps_ico=0;
	SSD1306_DrawFilledRectangle(X_gps_ico, Y_gps_ico, 1, 6, SSD1306_COLOR_WHITE );
	SSD1306_DrawCircle(X_gps_ico+6, Y_gps_ico+3, 3, SSD1306_COLOR_WHITE);
	SSD1306_DrawCircle(X_gps_ico+6, Y_gps_ico+3, 2, SSD1306_COLOR_WHITE);
	SSD1306_DrawFilledRectangle(X_gps_ico+11, Y_gps_ico, 1, 6, SSD1306_COLOR_WHITE );
	SSD1306_DrawLine(X_gps_ico+2, Y_gps_ico+2, 2, 4, SSD1306_COLOR_WHITE);
	SSD1306_DrawLine(X_gps_ico+10, Y_gps_ico+2, 10, 4, SSD1306_COLOR_WHITE);
	//////////////////////////////////////////////////////////

	// GPS level
	X_gps_lev=15;
	Y_gps_lev=0;
	if (SatFix >= 1)	SSD1306_DrawFilledRectangle(X_gps_lev, Y_gps_lev+6, 2, 1, SSD1306_COLOR_WHITE );
	if (SatFix >= 3)	SSD1306_DrawFilledRectangle(X_gps_lev+4, Y_gps_lev+4, 2, 3, SSD1306_COLOR_WHITE );
	if (SatFix >= 5)	SSD1306_DrawFilledRectangle(X_gps_lev+8, Y_gps_lev+2, 2, 5, SSD1306_COLOR_WHITE );
	if (SatFix >= 8)	SSD1306_DrawFilledRectangle(X_gps_lev+12, Y_gps_lev+0, 2, 7, SSD1306_COLOR_WHITE );
	////////////////////////////////////////////
	SSD1306_GotoXY(31, 0);
	printf ("%d", SatFix);
	//SSD1306_Puts("12", &Font_7x10, SSD1306_COLOR_WHITE);

	//
	//������� LORA
	X_lora_ico = 50;
	Y_lora_ico = 0;
	SSD1306_DrawFilledRectangle(X_lora_ico, Y_lora_ico, 1, 7, SSD1306_COLOR_WHITE );
	SSD1306_DrawLine(X_lora_ico+2, Y_lora_ico+2, X_lora_ico+4, Y_lora_ico, SSD1306_COLOR_WHITE);
	SSD1306_DrawLine(X_lora_ico+2, Y_lora_ico+3, X_lora_ico+5, Y_lora_ico, SSD1306_COLOR_WHITE);
	SSD1306_DrawLine(X_lora_ico-1, Y_lora_ico+2, X_lora_ico-3, Y_lora_ico, SSD1306_COLOR_WHITE);
	SSD1306_DrawLine(X_lora_ico-1, Y_lora_ico+3, X_lora_ico-4, Y_lora_ico, SSD1306_COLOR_WHITE);
	// LORA level
	X_lora_lev=55;
	Y_lora_lev=0;
	if ( (LoraRssiMax >  -130) )	SSD1306_DrawFilledRectangle(X_lora_lev, Y_lora_lev+6, 2, 1, SSD1306_COLOR_WHITE );
	if ( (LoraRssiMax >= -110) )	SSD1306_DrawFilledRectangle(X_lora_lev+4, Y_lora_lev+4, 2, 3, SSD1306_COLOR_WHITE );
	if ( (LoraRssiMax >= -90) )		SSD1306_DrawFilledRectangle(X_lora_lev+8, Y_lora_lev+2, 2, 5, SSD1306_COLOR_WHITE );
	if ( (LoraRssiMax >= -70) )		SSD1306_DrawFilledRectangle(X_lora_lev+12, Y_lora_lev+0, 2, 7, SSD1306_COLOR_WHITE );

	SSD1306_GotoXY(73, 0);
	Time_Display2( SecRTC ); 		// ���������� ����� RTC ��� ������

	SSD1306_GotoXY(0, 10);
	printf ("UTC:%c%c:%c%c:%c%c.%c%c",GPSFixData.Time[0],GPSFixData.Time[1],GPSFixData.Time[2],GPSFixData.Time[3],GPSFixData.Time[4],
			GPSFixData.Time[5], GPSFixData.Time[7], GPSFixData.Time[8]);

	SSD1306_GotoXY(112, 10);
	if (counterTXDisplay > 0)
		printf("TX");

	SSD1306_GotoXY(112, 20);
	if (counterRXDisplay > 0)
		printf("RX");

	SSD1306_GotoXY(0, 20);
	printf ("SATs:");
//	for (i=0; i<2; i++)
//		{
//		if (GPSFixData.SatelliteNum[i] == '\0') break;
//		OLED_Putc1 (GPSFixData.SatelliteNum[i]);
//		}
	printf ("%d", SatFix);					// ���������� ���������, ����������� � �������
	printf ("/%d", SatNumGPS+SatNumGLO);  	// ���������� ��������� ��� �������� ���������

//	SSD1306_GotoXY(72, 20);
//	printf ("P:%d ", countPacket);

	SSD1306_GotoXY(0, 30);
	printf ("Vacc: %d.%03d V ", VaccAvg/1000, VaccAvg%1000);

	SSD1306_GotoXY(96, 30);
	if ( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0 )  		// ���� �����
	{
		printf ("CHRG!");
	}

	SSD1306_GotoXY(0, 40);
	printf ("La: ");
	for (i=0; i<9; i++)									// DDMM.MMM
		{
		if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
			OLED_Putc1 ( GPSFixData.Latitude[i]);
		else
			OLED_Putc1 ('-');
		}
	if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
		printf ("'%c ", GPSFixData.NS );
	else
		printf ("--");

	SSD1306_GotoXY(0, 50);
	printf ("Lo: ");
		if (GPSFixData.Longitude[0] == '0') 			// �������� ���������� ����
		{
			for (i=1; i<10; i++)
				{
				if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
					OLED_Putc1 ( GPSFixData.Longitude[i]);	// DDMM.MMM
				else
					OLED_Putc1 ('-');
				}
		}
		else
		{
			for (i=0; i<10; i++)						// DDDMM.MMM
				{
				if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
					OLED_Putc1 ( GPSFixData.Longitude[i]);
				else
					OLED_Putc1 ('-');
				}
		}
	if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
		printf ("'%c ", GPSFixData.EW );
	else
		printf ("--");

	SSD1306_GotoXY(112, 50);
	printf("#%d", LORA_TX_TIME);

}
// ���������� ������������, �������, �������
void Display2	(void)
{
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_Fill(SSD1306_COLOR_BLACK);

	SSD1306_GotoXY(0, 0);
	printf ("%d.%03d V ", VaccAvg/1000, VaccAvg%1000 ); // ����������� ���������� ������������

//	SSD1306_GotoXY(7, 10);
//	Time_Display( TimeStartLogVcc ); 	// ���������� �����
//	SSD1306_GotoXY(60, 10);
//	printf ("%d  ", counterVaccLog );
//	Time_Display( SecRTC ); 			// ���������� �����

	SSD1306_GotoXY(60, 0);
	Time_Display( TimeLogVcc ); 		// ���������� �����, ������� ������� ���


	SSD1306_DrawLine(0, 15, 0, 63, SSD1306_COLOR_WHITE);	// ��� Y - ����������
	SSD1306_DrawLine(0, 63, 127, 63, SSD1306_COLOR_WHITE);	// ��� X - �����

	for (i = 0; i < 22; i++) // ������� ������� , ����� 1 ���, �� 6 ��������, 1 ������� - 10 ���
	{
		SSD1306_DrawLine(i*6, 60, i*6, 63, SSD1306_COLOR_WHITE);
	}

	for (i = 0; i < 6; i++) // ������� ���������� , ����� 0,2 �, �� 8 ��������, 1 ������� - 25 ��
	{
		SSD1306_DrawLine(1, 15 + i * 8, 3, 15 + i * 8, SSD1306_COLOR_WHITE);
	}

	for (i = 1; i<= 127; i++) // ������ ������
	{
		if (VaccLog[i] == 0) // ���� �������� 0, �� �� ������
			break;

		SSD1306_DrawLine(i-1+1, 63 - VaccLog[i-1], i+1, 63 - VaccLog[i], SSD1306_COLOR_WHITE);
	}


	SSD1306_DrawLine(127, 12, 127, 52, SSD1306_COLOR_WHITE);	// ��� Y - ���������� ����������
	for (i = 0; i < 5; i++) // ������� ���������� , ����� 10 ��, �� 10 ��������, 1 ������� - 1 ��
	{
		SSD1306_DrawLine(125, 12 + i * 10, 126, 12 + i * 10, SSD1306_COLOR_WHITE);
	}

	if (counterVaccosc == 5) // ���������� ������������� ������ 5 ������
	{
		counterVaccosc = 0;

		for (i = 99; i > 0; i--)
			VaccOsc [i] = VaccOsc [i-1];

		VaccOsc[0] = Vacc1 - VaccAvg;
	}
	else
		counterVaccosc ++;

	for (i = 1 ; i < 100; i++)	// ������ ����������� ����������
	{
		SSD1306_DrawLine(20 + i-1, 32 - VaccOsc[i-1], 20 + i, 32 - VaccOsc[i], SSD1306_COLOR_WHITE);
	}

/*	if (counterVaccosc == 99) 			// ������ ���������� Vacc ��������� �� 100
	{
		counterVaccosc = 0;
		for (i = 0 ; i < Average1; i++)	// ��������� ������ �������������
		{
			VaccOsc [i] = Vacc [i] - VaccAvg;
		}
	}
	else
		counterVaccosc ++;

	for (i = 1 ; i < Average1; i++)	// ������ ����������� ����������
	{
		SSD1306_DrawLine(20 + i-1, 32 - VaccOsc[i-1], 20 + i, 32 - VaccOsc[i], SSD1306_COLOR_WHITE);
	}*/

//	SSD1306_GotoXY(0, 0);
//	printf ("Vacc %d.%03d V ", Vacc1/1000, Vacc1%1000);
//
//	SSD1306_GotoXY(0, 10);
//	printf ("Vref %d.%03d V ", Vref1/1000, Vref1%1000);
//
//	SSD1306_GotoXY(0, 20);
//	printf ("Vbat %d.%03d V ", Vbat1/1000, Vbat1%1000);
//
//	SSD1306_GotoXY(0, 30);
//	printf ("Vacc %d.%03d V %d %% ", VaccAvg/1000, VaccAvg%1000, VaccDev);
//
//	SSD1306_GotoXY(0, 40);
//	printf ("Vref %d.%03d V %d %%", VrefAvg/1000, VrefAvg%1000, VrefDev);
//
//	SSD1306_GotoXY(0, 50);
//	printf ("Vbat %d.%03d V %d %% ", VbatAvg/1000, VbatAvg%1000, VbatDev);
//
//	SSD1306_GotoXY(96, 0);
//	if ( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0 )  		// ���� �����
//	{
//		printf ("CHRG!");
//	}
//	if ( GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == 1 )  		// USB ���������
//	{ // �� �������� ��-�� ����������� � JTAG
//		SSD1306_GotoXY(96, 10);
//		printf ("USB");
//	}

}
// ����� RTC � GPS
void Display3 	(void)
{
	rprintfInit (OLED_font1); 	// ����� �� OLED �������
	SSD1306_Fill(SSD1306_COLOR_BLACK);

	SSD1306_GotoXY(110, 0);
	printf ("*%d", flagTXDisplay);

	SSD1306_GotoXY(0, 0);
	printf ("RTC: ");
	//SSD1306_GotoXY(0, 10);
	Time_Display( sec_RTC_to_display );

	SSD1306_GotoXY(0, 10);
	printf ("GPS: ");
	//SSD1306_GotoXY(0, 30);
	Time_Display( sec_GPS_to_display );		// ������� �� ����� HH:MM:SS - �����, ���������� �� ������ GPS ������� �����
	printf (".%d", GPSTimeMilliSec );		// � ������������
	//printf ("%c%c:%c%c:%c%c.%c%c",GPSFixData.Time[0],GPSFixData.Time[1],GPSFixData.Time[2],GPSFixData.Time[3],GPSFixData.Time[4],
	//		GPSFixData.Time[5], GPSFixData.Time[7], GPSFixData.Time[8]);

	SSD1306_GotoXY(0, 20);

	printf ("Date: ");
	for (i=0; i<2; i++)
		{
		OLED_Putc1(GPSFixData.Day[i]);
		}
	OLED_Putc1('.');
	for (i=0; i<2; i++)
		{
		OLED_Putc1(GPSFixData.Month[i]);
		}
	OLED_Putc1('.');
	for (i=0; i<4; i++)
		{
		OLED_Putc1(GPSFixData.Year[i]);
		}
	SSD1306_GotoXY(0, 30);
	printf ("Sec RTC: %d", SecRTC );
	SSD1306_GotoXY(0, 40);
	printf ("Sec GPS: %d", GPSTimeSec);

	SSD1306_GotoXY(0, 50);
	printf ("Sat:%d/%d", SatNumGPS, SatFix);  	// ���������� ��������� ��� �������� ���������
	SSD1306_GotoXY(80, 50);
	printf ("Fix: %d", Fix);

}
// ������� ������� �� ��������� � ���� ID S/N dB
void Display4 	(void)
{
	rprintfInit (OLED_font1); 	// ����� �� OLED �������
	SSD1306_Fill(SSD1306_COLOR_BLACK);

//	SSD1306_GotoXY(105, 0);
//	printf ("%d", SatNum);  	// ���������� ��������� ��� �������� ���������
//	SSD1306_GotoXY(105, 10);
//	printf ("%d", SatFix);		// ���������� ���������, ����������� � �������

	SSD1306_GotoXY(0, 0);
	for (i=0; i<6; i++)		//	������ ������ ������ 6 ��������� GPS
	{
	//	if (i<6)
	//	{
			SSD1306_GotoXY(0, 10 * i);

			for (j=0; j<3; j++)
			{
				if ( GPSFixData.Sats[i][0][j] == '\0' )
					break;
				SSD1306_Putc (GPSFixData.Sats[i][0][j], &Font_7x10, SSD1306_COLOR_WHITE ); // ����� ��������
			}

			SSD1306_GotoXY(18, 10 * i);

			for (j=0; j<3; j++)
			{
				if ( GPSFixData.Sats[i][3][j] == '\0' )
					break;
				SSD1306_Putc (GPSFixData.Sats[i][3][j], &Font_7x10, SSD1306_COLOR_WHITE ); // SNR ��������
			}
			printf ("dB");
	//	}
	}
	for (i=0; i<6; i++)		//	������ ������ ������ 6 ��������� GLONASS
		//else if (i<12) // ������� �������� 12 ���������
		{
			SSD1306_GotoXY(55, 10 * i);

			for (j=0; j<3; j++)
			{
				if ( GPSFixData.SatsGLO[i][0][j] == '\0' )
					break;
				SSD1306_Putc (GPSFixData.SatsGLO[i][0][j], &Font_7x10, SSD1306_COLOR_WHITE );	// ����� ��������
			}

			SSD1306_GotoXY(73, 10 * i);

			for (j=0; j<3; j++)
			{
				if ( GPSFixData.SatsGLO[i][3][j] == '\0' )
					break;
				SSD1306_Putc (GPSFixData.SatsGLO[i][3][j], &Font_7x10, SSD1306_COLOR_WHITE );	// SNR ��������
			}
			printf ("dB");
		}
	//	else {
	//		break;
	//	}

	//}

}
// ������� ������� �� ��������� � ����������� ����
void Display5	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	for (i=0; i<SatNumGPS; i++) 	// ���������� ��� ������� �������� GPS
	{
		SatIDGPS [i] = (GPSFixData.Sats[i][0][0] - 0x30) * 10 + (GPSFixData.Sats[i][0][1] - 0x30) * 1 ;

		for (j=0; j<3; j++)
		{
			if ( GPSFixData.Sats[i][3][j] == '\0' ) 	// ������� ������� ���� � ����� S/N
				break;
		}

		if (j==3) 		// ��� �����
		{
			SatLevelsGPS [i] = (GPSFixData.Sats[i][3][0] - 0x30) * 100 + (GPSFixData.Sats[i][3][1] - 0x30) * 10 + (GPSFixData.Sats[i][3][2] - 0x30) * 1 ;
		}
		else if (j==2) 	// ��� �����
		{
			SatLevelsGPS [i] = (GPSFixData.Sats[i][3][0] - 0x30) * 10 + (GPSFixData.Sats[i][3][1] - 0x30) * 1 ;
		}
		else if (j==1) 	// ���� �����
		{
			SatLevelsGPS [i] = GPSFixData.Sats[i][3][0] - 0x30 ;
		}
//printf ("%d %d;", i, j);
	}

	SatSummGPS = 0; 			// ����� ������� �������� �� ��������� GPS
	for (i=0; i<SatNumGPS; i++)
	{
		SatSummGPS += SatLevelsGPS[i];
	}

	// ---------------------------- GLONASS �������� -------------------------------------------
	for (i=0; i<SatNumGLO; i++) 	// ���������� ��� ������� �������� GPS
		{
			SatIDGLO [i] = (GPSFixData.SatsGLO[i][0][0] - 0x30) * 10 + (GPSFixData.SatsGLO[i][0][1] - 0x30) * 1 ;

			for (j=0; j<3; j++)
			{
				if ( GPSFixData.SatsGLO[i][3][j] == '\0' ) 	// ������� ������� ���� � ����� S/N
					break;
			}

			if (j==3) 		// ��� �����
			{
				SatLevelsGLO [i] = (GPSFixData.SatsGLO[i][3][0] - 0x30) * 100 + (GPSFixData.SatsGLO[i][3][1] - 0x30) * 10 + (GPSFixData.SatsGLO[i][3][2] - 0x30) * 1 ;
			}
			else if (j==2) 	// ��� �����
			{
				SatLevelsGLO [i] = (GPSFixData.SatsGLO[i][3][0] - 0x30) * 10 + (GPSFixData.SatsGLO[i][3][1] - 0x30) * 1 ;
			}
			else if (j==1) 	// ���� �����
			{
				SatLevelsGLO [i] = GPSFixData.SatsGLO[i][3][0] - 0x30 ;
			}
	//printf ("%d %d;", i, j);
		}

		SatSummGLO = 0; 			 // ����� ������� �������� �� ��������� GLONASS
		for (i=0; i<SatNumGLO; i++)
		{
			SatSummGLO += SatLevelsGLO[i];
		}
	//---------------------------------------------------------------------------------------------

	//rprintfInit (OLED_font1); 	// ����� �� OLED �������
	SSD1306_GotoXY(0, 0);
	printf ("GPS:%d/%d", SatFixGPS, SatNumGPS );  		// ���������� ��������� ��� �������� ���������
	SSD1306_GotoXY(66, 0);
	printf ("GLO:%d/%d", SatFixGLO, SatNumGLO );  	// ���������� ��������� ��� �������� ���������

	// ������� ���� �������������� ��������� GPS � ������ ������
	SatSummUsedGPS = 0;
	for (i=0; i<SatNumGPS; i++)
	{
		for (j=0; j<12; j++) // ���������� ��� �������� �� �������, ���������� ������
		{
			if ( SatUsedGPS [j] == SatIDGPS [i] )
			{
				SatSummUsedGPS += SatLevelsGPS[i];  		// ����� S/N ���������, ������������ � �������
				SSD1306_DrawFilledRectangle(0 + i*4, 63-SatLevelsGPS[i], 2, SatLevelsGPS[i], SSD1306_COLOR_WHITE);  // ��������, ������������ � ������� - �������� �����
			}
			else
				SSD1306_DrawRectangle(0 + i*4, 63-SatLevelsGPS[i], 2, SatLevelsGPS[i], SSD1306_COLOR_WHITE);		  // ��������, �������������� � ������� - ������� ����������������
		}
	}
	// ������� ���� �������������� ��������� GLONASS � ������ ������
	SatSummUsedGLO = 0;
	for (i=0; i<SatNumGLO; i++)
	{
		for (j=0; j<12; j++) // ���������� ��� �������� �� �������, ���������� ������
		{
			if ( SatUsedGLO [j] == SatIDGLO [i] )
			{
				SatSummUsedGLO += SatLevelsGLO[i];  		// ����� S/N ���������, ������������ � �������
				SSD1306_DrawFilledRectangle(66 + i*4, 63-SatLevelsGLO[i], 2, SatLevelsGLO[i], SSD1306_COLOR_WHITE);  // ��������, ������������ � ������� - �������� �����
			}
			else
				SSD1306_DrawRectangle(66 + i*4, 63-SatLevelsGLO[i], 2, SatLevelsGLO[i], SSD1306_COLOR_WHITE);		  // ��������, �������������� � ������� - ������� ����������������
		}
	}

	SSD1306_GotoXY(0, 10);
	printf ("S=%d/%d", SatSummUsedGPS, SatSummGPS);

	SSD1306_GotoXY(66, 10);
	printf ("S=%d/%d", SatSummUsedGLO, SatSummGLO);

	SSD1306_GotoXY(101, 20);
	printf ("%d", SatSummUsedGPS + SatSummUsedGLO);

}
// PDOP, HDOP, VDOP ���������
void Display6	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_GotoXY(0, 0);
	printf ("Auto2D3D: %d ", AutoSel_2D3D);
	SSD1306_GotoXY(0, 10);
	printf ("Fix : %d", Fix);
	SSD1306_GotoXY(0, 20);
	printf ("PDOP: %d.%d", PDOP/100, PDOP%100);
	SSD1306_GotoXY(0, 30);
	printf ("HDOP: %d.%d", HDOP/100, HDOP%100);
	SSD1306_GotoXY(0, 40);
	printf ("VDOP: %d.%d", VDOP/100, VDOP%100);

	SSD1306_GotoXY(0, 50);
	//SSD1306_GotoXY(60, 20);
	printf ("TTFF: %d s", TTFF);
//	for (i=0; i<12; i++)
//	{
//		if ( SatUsed[i] != 0)
//			printf ("%d ",  SatUsed [i]);
//	}

}
// ���������� � ������
void Display7	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);

	SSD1306_GotoXY(0, 0);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	printf ("  Coordinates: ");

	rprintfInit (OLED_font2); 	// ����� �� OLED �������

	SSD1306_GotoXY(0, 10);
	//printf ("La: ");
	for (i=0; i<9; i++)									// DDMM.MMM
		{
			if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
				OLED_Putc2 ( GPSFixData.Latitude[i]);
			else
				OLED_Putc2 ('-');
		}
	if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
		printf ("'%c ", GPSFixData.NS );
	else
		printf ("--");

	SSD1306_GotoXY(0, 27);
	//printf ("Lo: ");
		if (GPSFixData.Longitude[0] == '0') 			// �������� ���������� ����
		{
			for (i=1; i<10; i++)
				{
				if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
					OLED_Putc2 ( GPSFixData.Longitude[i]);	// DDMM.MMM
				else
					OLED_Putc2 ('-');
				}
		}
		else
		{
			for (i=0; i<10; i++)						// DDDMM.MMM
				{
				if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
					OLED_Putc2 ( GPSFixData.Longitude[i]);
				else
					OLED_Putc2 ('-');
				}
		}
	if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
		printf ("'%c ", GPSFixData.EW );
	else
		printf ("--");

	rprintfInit (OLED_font1); 	// ����� �� OLED �������
	SSD1306_GotoXY(0, 50);
	printf ("Alt: ");					// ������
	SSD1306_GotoXY(30, 44);
	for (i=0; i<5; i++)
		{
		if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
			OLED_Putc2 ( GPSFixData.Altitude[i]);
		else
			OLED_Putc2 ('-');
		}
	printf (" m");

}
/*void Display7	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_GotoXY(0, 0);
	printf ("Lat : %d %d", GPSLatitude, GPS_N_S);
	SSD1306_GotoXY(0, 10);
	printf ("Long: %d %d", GPSLongitude, GPS_E_W);
	SSD1306_GotoXY(0, 20);
	printf ("Alt : %d ", GPSAltitude);

	SSD1306_GotoXY(0, 30);
	printf ("La: ");
	for (i=0; i<9; i++)									// DDMM.MMM
		{
			if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
				OLED_Putc1 ( GPSFixData.Latitude[i]);
			else
				OLED_Putc1 ('-');
		}
	if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
		printf ("'%c ", GPSFixData.NS );
	else
		printf ("--");

	SSD1306_GotoXY(0, 40);
	printf ("Lo: ");
		if (GPSFixData.Longitude[0] == '0') 			// �������� ���������� ����
		{
			for (i=1; i<10; i++)
				{
				if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
					OLED_Putc1 ( GPSFixData.Longitude[i]);	// DDMM.MMM
				else
					OLED_Putc1 ('-');
				}
		}
		else
		{
			for (i=0; i<10; i++)						// DDDMM.MMM
				{
				if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
					OLED_Putc1 ( GPSFixData.Longitude[i]);
				else
					OLED_Putc1 ('-');
				}
		}
	if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
		printf ("'%c ", GPSFixData.EW );
	else
		printf ("--");

	SSD1306_GotoXY(0, 50);
	printf ("Alt: ");					// ������
	for (i=0; i<5; i++)
		{
		if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
			OLED_Putc1 ( GPSFixData.Altitude[i]);
		else
			OLED_Putc1 ('-');
		}

}*/
// ��������� ������������ 1PPS
void Display8	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_GotoXY(0, 0);
	printf ("count: %d ", counter_TIM4_fix);
	SSD1306_GotoXY(0, 10);
	printf ("TIM4 : %d ", TIM4_fix);
	SSD1306_GotoXY(0, 20);
	printf ("AVG  : %d ", averageTIM4_temp);
	SSD1306_GotoXY(0, 30);
	printf ("cAvg : %d ", counter_AVG_1PPS);
	SSD1306_GotoXY(0, 40);
	printf ("Fail:  %d ", flag_1PPS_fail);
	SSD1306_GotoXY(0, 50);
	printf ("AvgOK: %d ", flag_1PPS_AVG_OK);

}
// �������� LORA
void Display9	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_GotoXY(0, 0);
	printf (" -LORA receiver-  ");
	SSD1306_GotoXY(0, 20);
	printf("RSSI:%d dBm ", (s32)(SX1276LoRaGetPacketRssi()) );
	SSD1306_GotoXY(112, 10);
	if (counterRXDisplay > 0)
		printf("RX");
	SSD1306_GotoXY(0, 30);
	printf("SNR:%d dB ", SX1276GetPacketSnr() );
	SSD1306_GotoXY(0, 30);
	//printf("PER:%02d.%02d%% ", PER1, PER2 );
	SSD1306_GotoXY(0, 40);
	printf("Offset: %d Hz", (s32)FreqError );
	SSD1306_GotoXY(0, 50);
	printf ("RX/ERR: %d / %d ", countOK - countERR, countERR);

}
// �������� ����������
void Display10	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_GotoXY(0, 0);
	//printf ("  Coordinates: ");
	SSD1306_GotoXY(0, 0);
	printf("[1]:%d %d", structBuffer[1].Latitude, structBuffer[1].Vacc );
	SSD1306_GotoXY(0, 10);
	printf("    %d %d", structBuffer[1].Longitude, structBuffer[1].reserv );

	SSD1306_GotoXY(0, 20);
	printf("[2]:%d %d", structBuffer[2].Latitude, structBuffer[2].Vacc );
	SSD1306_GotoXY(0, 30);
	printf("    %d %d", structBuffer[2].Longitude, structBuffer[2].reserv );

	SSD1306_GotoXY(0, 40);
	printf("[3]:%d %d", structBuffer[3].Latitude, structBuffer[3].Vacc );
	SSD1306_GotoXY(0, 50);
	printf("    %d %d", structBuffer[3].Longitude, structBuffer[3].reserv );

}
// �������� ������� �������
void Display11	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_GotoXY(0, 0);
	printf (" Times: ");
	SSD1306_GotoXY(64, 0);
	Time_Display( SecRTC ); 		// ���������� ����� RTC

	rprintfInit (OLED_font2); 		// ����� �� OLED �������
	//SSD1306_GotoXY(0, 10);

	//printf("[0]:%d", TimesOld[0] );
	SSD1306_GotoXY(0, 10);
	printf("1 " );
	SSD1306_GotoXY(32, 10);
	Time_Display (TimesOld[1]);

	SSD1306_GotoXY(0, 27);
	printf("2 " );
	SSD1306_GotoXY(32, 27);
	Time_Display (TimesOld[2]);

	SSD1306_GotoXY(0, 44);
	printf("3 " );
	SSD1306_GotoXY(32, 44);
	Time_Display (TimesOld[3]);

	//SSD1306_GotoXY(0, 50);
	//printf("[4]:%d", TimesOld[4] );

}
// ��������� �������� ��������
void Display12	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_GotoXY(0, 0);
	printf ("N P FreqE RSSI SNR");
	for(i=1; i<4; i++)
	{
		SSD1306_GotoXY(0, 10*(i+1));
		printf("%d", i);
		SSD1306_GotoXY(16, 10*(i+1));
		printf("%d", LoraRX [i].Power);
		SSD1306_GotoXY(34, 10*(i+1));
		printf("%d", LoraRX [i].FreqError);
		SSD1306_GotoXY(74, 10*(i+1));
		printf("%d", LoraRX [i].RSSI);
		SSD1306_GotoXY(106, 10*(i+1));
		printf("%d", LoraRX [i].SNR );
	}

	//SSD1306_GotoXY(0, 50);
	//printf("RssiMax = %d", LoraRssiMax );

}
// PER �� ������� �����
void Display13	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_GotoXY(0, 0);
	printf ("N RSSI Count  PER");
	for(i=1; i<4; i++)
	{
		SSD1306_GotoXY(0, 10*(i+1));
		printf("%d", i);
		SSD1306_GotoXY(14, 10*(i+1));
		printf("%d", LoraRX [i].RSSI);
		//printf("%d", TimeSecFirstRX[i]);
		SSD1306_GotoXY(49, 10*(i+1));
		printf("%d", CountRX[i]);
		SSD1306_GotoXY(85, 10*(i+1));
		if (PER [i] != 65535)
			printf("%d.%02d%%", PER[i]/100, PER[i]%100);
		//else
			//printf("---%%", PER[i]);
		//SSD1306_GotoXY(106, 10*(i+1));
		//printf("%d", LoraRX [i].SNR );
	}

//	SSD1306_GotoXY(0, 50);
//	printf("Delta = %d  ", timeDelta );
//	SSD1306_DrawPixel (127, 63, SSD1306_COLOR_WHITE );
//	SSD1306_DrawPixel (127, 0, SSD1306_COLOR_WHITE );
//	SSD1306_DrawPixel (0, 0, SSD1306_COLOR_WHITE );
//	SSD1306_DrawPixel (0, 63, SSD1306_COLOR_WHITE );

}
// �������������
void Display14	(void)
{
	u32 temp;

	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_GotoXY(0, 0);
	printf ("    Distance: ");

	rprintfInit (OLED_font2); 	// ����� �� OLED �������
	SSD1306_GotoXY(0, 10);
	temp = distance1_1_2 * 10;
	printf ("1-2:%d.%d m", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
	SSD1306_GotoXY(0, 27);
	temp = distance1_1_3 * 10;
	printf ("1-3:%d.%d m", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
	SSD1306_GotoXY(0, 44);
	temp = distance1_2_3 * 10;
	printf ("2-3:%d.%d m", temp / 10,  temp % 10 ); // ���������� �� ����� � ������

}
// ������ ������� �� ������ � ����������� ����
void Display15	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_GotoXY(0, 0);
	printf ("      RSSI:    dBm");


	for (i=1; i<6; i++) // ����� � 1 �� 5
	{
		SSD1306_GotoXY(0, 10*i);
		printf ("[%d]", i);

		if (LoraRX [i].RSSI != 0) 	// ���� �����-�� ���������� ��������
		{
			if ( LoraRX [i].RSSI >= -140) // ���� ������� ������ -140, ������
				{
					if ( LoraRX [i].RSSI > -55 )
						SSD1306_DrawFilledRectangle( 20, 10*i+2, 85, 5, SSD1306_COLOR_WHITE );  // ��������, �����������
					else
						SSD1306_DrawFilledRectangle( 20, 10*i+2, 85 + (55 + LoraRX [i].RSSI) , 5, SSD1306_COLOR_WHITE );  // ������� RSSI ( �� -55 �� -140 ��� )
				}

			if ( LoraRX [i].RSSI <= -100) // ���� 3 ����� � �����
				SSD1306_GotoXY(106-7, 10*i);
			else
				SSD1306_GotoXY(113-7, 10*i); // ���� 2 ����� � �����

			printf ("%d", LoraRX [i].RSSI);
		}

	}

}

// �������� �����
void Display1Release	(void)
{
	u32 temp;
	u32 temp2;
	u8 X_bat, Y_bat;
	u16 VaccAvgTemp;

	SSD1306_Fill(SSD1306_COLOR_BLACK);
	temp2 = sec_div % 10 ;

	// ���������
	X_bat = 0;
	Y_bat = 0;
	SSD1306_DrawRectangle(0+X_bat, 0+Y_bat, 14, 7, SSD1306_COLOR_WHITE );
	SSD1306_DrawFilledRectangle(15+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
	if (VaccAvg >= 3500)	SSD1306_DrawFilledRectangle(2+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
	if (VaccAvg >= 3690)	SSD1306_DrawFilledRectangle(X_bat+2+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
	if (VaccAvg >= 3780)	SSD1306_DrawFilledRectangle(X_bat+2+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
	if (VaccAvg >= 3850)	SSD1306_DrawFilledRectangle(X_bat+2+3+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );

//------------------------------------------------------------------ 1 ------------------------------------------------------------------------------------------
	if (NumberDev == 1)
	{
		rprintfInit (OLED_font2);

		SSD1306_GotoXY(0, 10);
		if ( temp2 == 2)
			printf ("2>");
		else
			printf ("2 ");

		if (LoraRX [2].RSSI != 0) 	// ���� �����-�� ���������� ��������
			{
				if ( LoraRX [2].RSSI >= -140) // ���� ������� ������ -140, ������
					{
						if ( LoraRX [2].RSSI > -55 )
							SSD1306_DrawFilledRectangle( 24, 21, 85, 5, SSD1306_COLOR_WHITE );  // ��������, �����������
						else
						{
							SSD1306_DrawRectangle( 24, 21, 85, 5, SSD1306_COLOR_WHITE );
							SSD1306_DrawFilledRectangle( 24, 21, 85 + (55 + LoraRX [2].RSSI) , 5, SSD1306_COLOR_WHITE );  // ������� RSSI ( �� -55 �� -140 ��� )
						}
					}

				if ( LoraRX [2].RSSI <= -100) // ���� 3 ����� � �����
					SSD1306_GotoXY(18, 11);
				else
					SSD1306_GotoXY(25, 11); // ���� 2 ����� � �����

				rprintfInit (OLED_font1);
				printf ("%d dBm ", LoraRX [2].RSSI );
			}
		else
			{
				SSD1306_GotoXY(25, 0+11); //
				rprintfInit (OLED_font1);
				printf ("Fail!");
			}

		if (LoraRX [2].Power > 0)
		{
			SSD1306_GotoXY(80, 11); // �������� �����������
			printf("P%d", LoraRX [2].Power);
			SSD1306_GotoXY(113, 11+10); // ������� �������
			printf("%02X", LoraRX [2].CounterTxP);
		}
		rprintfInit (OLED_font1);

		SSD1306_GotoXY(25, 0);
		if (distance1_1_2 < 100000)
			{
			temp = distance1_1_2 * 10;
			printf ("%d.%dm", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
			}
		else
			{
			printf ("---.-m" ); // ���������� �� ����������
			}

		SSD1306_GotoXY(71, 0);
		Time_Display (TimesOld[2]);

		// ���������
		if ( structBuffer[2].Vacc != 0)
		{
			X_bat = 110;
			Y_bat = 11;
			SSD1306_DrawRectangle(0+X_bat, 0+Y_bat, 14, 7, SSD1306_COLOR_WHITE );
			SSD1306_DrawFilledRectangle(15+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
			VaccAvgTemp = structBuffer[2].Vacc;
			if (VaccAvgTemp >= 3500)	SSD1306_DrawFilledRectangle(2+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3690)	SSD1306_DrawFilledRectangle(X_bat+2+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3780)	SSD1306_DrawFilledRectangle(X_bat+2+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3850)	SSD1306_DrawFilledRectangle(X_bat+2+3+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
		}

		// ------------------------------------------
		rprintfInit (OLED_font2);

		SSD1306_GotoXY(0, 32+10);
		if ( temp2 == 3)
			printf ("3>");
		else
			printf ("3 ");

		if (LoraRX [3].RSSI != 0) 	// ���� �����-�� ���������� ��������
			{
				if ( LoraRX [3].RSSI >= -140) // ���� ������� ������ -140, ������
					{
						if ( LoraRX [3].RSSI > -55 )
							SSD1306_DrawFilledRectangle( 24, 31+21, 85, 5, SSD1306_COLOR_WHITE );  // ��������, �����������
						else
						{
							SSD1306_DrawRectangle( 24, 31+21, 85, 5, SSD1306_COLOR_WHITE );
							SSD1306_DrawFilledRectangle( 24, 31+21, 85 + (55 + LoraRX [3].RSSI) , 5, SSD1306_COLOR_WHITE );  // ������� RSSI ( �� -55 �� -140 ��� )
						}
					}

				if ( LoraRX [3].RSSI <= -100) // ���� 3 ����� � �����
					SSD1306_GotoXY(18, 31+11);
				else
					SSD1306_GotoXY(25, 31+11); // ���� 2 ����� � �����

				rprintfInit (OLED_font1);
				printf ("%d dBm", LoraRX [3].RSSI);
			}
		else
			{
				SSD1306_GotoXY(25, 31+11); //
				rprintfInit (OLED_font1);
				printf ("Fail!");
			}

		if (LoraRX [3].Power > 0)
		{
			SSD1306_GotoXY(80, 31+11); // �������� �����������
			printf("P%d", LoraRX [3].Power);
			SSD1306_GotoXY(113, 31+11+10); // ������� �������
			printf("%02X", LoraRX [3].CounterTxP);
		}

		rprintfInit (OLED_font1);

		SSD1306_GotoXY(25, 31+0);
		if (distance1_1_3 < 100000)
			{
			temp = distance1_1_3 * 10;
			printf ("%d.%dm", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
			}
		else
			{
			printf ("---.-m" ); // ���������� �� ����������
			}
		SSD1306_GotoXY(71, 31+0);
		Time_Display (TimesOld[3]);

		// ���������
		if ( structBuffer[3].Vacc != 0)
		{
			X_bat = 110;
			Y_bat = 31+11;
			SSD1306_DrawRectangle(0+X_bat, 0+Y_bat, 14, 7, SSD1306_COLOR_WHITE );
			SSD1306_DrawFilledRectangle(15+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
			VaccAvgTemp = structBuffer[3].Vacc;
			if (VaccAvgTemp >= 3500)	SSD1306_DrawFilledRectangle(2+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3690)	SSD1306_DrawFilledRectangle(X_bat+2+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3780)	SSD1306_DrawFilledRectangle(X_bat+2+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3850)	SSD1306_DrawFilledRectangle(X_bat+2+3+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
		}

	}

//------------------------------------------------------------------ 2 ------------------------------------------------------------------------------------------
	if (NumberDev == 2)
	{
		rprintfInit (OLED_font2);

		SSD1306_GotoXY(0, 10);
		if ( temp2 == 1)
			printf ("1>");
		else
			printf ("1 ");

		if (LoraRX [1].RSSI != 0) 	// ���� �����-�� ���������� ��������
			{
				if ( LoraRX [1].RSSI >= -140) // ���� ������� ������ -140, ������
					{
						if ( LoraRX [1].RSSI > -55 )
							SSD1306_DrawFilledRectangle( 24, 21, 85, 5, SSD1306_COLOR_WHITE );  // ��������, �����������
						else
						{
							SSD1306_DrawRectangle( 24, 21, 85, 5, SSD1306_COLOR_WHITE );
							SSD1306_DrawFilledRectangle( 24, 21, 85 + (55 + LoraRX [1].RSSI) , 5, SSD1306_COLOR_WHITE );  // ������� RSSI ( �� -55 �� -140 ��� )
						}
					}

				if ( LoraRX [1].RSSI <= -100) // ���� 3 ����� � �����
					SSD1306_GotoXY(18, 11);
				else
					SSD1306_GotoXY(25, 11); // ���� 2 ����� � �����

				rprintfInit (OLED_font1);
				printf ("%d dBm ", LoraRX [1].RSSI );
			}
		else
			{
				SSD1306_GotoXY(25, 0+11); //
				rprintfInit (OLED_font1);
				printf ("Fail!");
			}

		if (LoraRX [1].Power > 0)
		{
			SSD1306_GotoXY(80, 11); // �������� �����������
			printf("P%d", LoraRX [1].Power);
			SSD1306_GotoXY(113, 11+10); // ������� �������
			printf("%02X", LoraRX [1].CounterTxP);
		}

		rprintfInit (OLED_font1);

		SSD1306_GotoXY(25, 0);
		if (distance1_1_2 < 100000)
			{
			temp = distance1_1_2 * 10;
			printf ("%d.%dm", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
			}
		else
			{
			printf ("---.-m" ); // ���������� �� ����������
			}

		SSD1306_GotoXY(71, 0);
		Time_Display (TimesOld[1]);

		// ���������
		if ( structBuffer[1].Vacc != 0)
		{
			X_bat = 110;
			Y_bat = 11;
			SSD1306_DrawRectangle(0+X_bat, 0+Y_bat, 14, 7, SSD1306_COLOR_WHITE );
			SSD1306_DrawFilledRectangle(15+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
			VaccAvgTemp = structBuffer[1].Vacc;
			if (VaccAvgTemp >= 3500)	SSD1306_DrawFilledRectangle(2+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3690)	SSD1306_DrawFilledRectangle(X_bat+2+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3780)	SSD1306_DrawFilledRectangle(X_bat+2+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3850)	SSD1306_DrawFilledRectangle(X_bat+2+3+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
		}

		// ---------------------------------------------
		rprintfInit (OLED_font2);

		SSD1306_GotoXY(0, 32+10);
		if ( temp2 == 3)
			printf ("3>");
		else
			printf ("3 ");

		if (LoraRX [3].RSSI != 0) 	// ���� �����-�� ���������� ��������
			{
				if ( LoraRX [3].RSSI >= -140) // ���� ������� ������ -140, ������
					{
						if ( LoraRX [3].RSSI > -55 )
							SSD1306_DrawFilledRectangle( 24, 31+21, 85, 5, SSD1306_COLOR_WHITE );  // ��������, �����������
						else
						{
							SSD1306_DrawRectangle( 24, 31+21, 85, 5, SSD1306_COLOR_WHITE );
							SSD1306_DrawFilledRectangle( 24, 31+21, 85 + (55 + LoraRX [3].RSSI) , 5, SSD1306_COLOR_WHITE );  // ������� RSSI ( �� -55 �� -140 ��� )
						}
					}

				if ( LoraRX [3].RSSI <= -100) // ���� 3 ����� � �����
					SSD1306_GotoXY(18, 31+11);
				else
					SSD1306_GotoXY(25, 31+11); // ���� 2 ����� � �����

				rprintfInit (OLED_font1);
				printf ("%d dBm", LoraRX [3].RSSI);
			}
		else
			{
				SSD1306_GotoXY(25, 31+11); //
				rprintfInit (OLED_font1);
				printf ("Fail!");
			}

		if (LoraRX [3].Power > 0)
		{
			SSD1306_GotoXY(80, 31+11); // �������� �����������
			printf("P%d", LoraRX [3].Power);
			SSD1306_GotoXY(113, 31+11+10); // ������� �������
			printf("%02X", LoraRX [3].CounterTxP);
		}

		rprintfInit (OLED_font1);

		SSD1306_GotoXY(25, 31+0);
		if (distance1_2_3 < 100000)
			{
			temp = distance1_2_3 * 10;
			printf ("%d.%dm", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
			}
		else
			{
			printf ("---.-m" ); // ���������� �� ����������
			}
		SSD1306_GotoXY(71, 31+0);
		Time_Display (TimesOld[3]);

		// ���������
		if ( structBuffer[3].Vacc != 0)
		{
			X_bat = 110;
			Y_bat = 31+11;
			SSD1306_DrawRectangle(0+X_bat, 0+Y_bat, 14, 7, SSD1306_COLOR_WHITE );
			SSD1306_DrawFilledRectangle(15+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
			VaccAvgTemp = structBuffer[3].Vacc;
			if (VaccAvgTemp >= 3500)	SSD1306_DrawFilledRectangle(2+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3690)	SSD1306_DrawFilledRectangle(X_bat+2+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3780)	SSD1306_DrawFilledRectangle(X_bat+2+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3850)	SSD1306_DrawFilledRectangle(X_bat+2+3+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
		}
	}

//------------------------------------------------------------------ 3 ------------------------------------------------------------------------------------------
	if (NumberDev == 3)
	{
		rprintfInit (OLED_font2);

		SSD1306_GotoXY(0, 10);
		if ( temp2 == 1)
			printf ("1>");
		else
			printf ("1 ");

		if (LoraRX [1].RSSI != 0) 	// ���� �����-�� ���������� ��������
			{
				if ( LoraRX [1].RSSI >= -140) // ���� ������� ������ -140, ������
					{
						if ( LoraRX [1].RSSI > -55 )
							SSD1306_DrawFilledRectangle( 24, 21, 85, 5, SSD1306_COLOR_WHITE );  // ��������, �����������
						else
						{
							SSD1306_DrawRectangle( 24, 21, 85, 5, SSD1306_COLOR_WHITE );
							SSD1306_DrawFilledRectangle( 24, 21, 85 + (55 + LoraRX [1].RSSI) , 5, SSD1306_COLOR_WHITE );  // ������� RSSI ( �� -55 �� -140 ��� )
						}
					}

				if ( LoraRX [1].RSSI <= -100) // ���� 3 ����� � �����
					SSD1306_GotoXY(18, 11);
				else
					SSD1306_GotoXY(25, 11); // ���� 2 ����� � �����

				rprintfInit (OLED_font1);
				printf ("%d dBm ", LoraRX [1].RSSI );
			}
		else
			{
				SSD1306_GotoXY(25, 0+11); //
				rprintfInit (OLED_font1);
				printf ("Fail!");
			}

		if (LoraRX [1].Power > 0)
		{
			SSD1306_GotoXY(80, 11); // �������� �����������
			printf("P%d", LoraRX [1].Power);
			SSD1306_GotoXY(113, 11+10); // ������� �������
			printf("%02X", LoraRX [1].CounterTxP);
		}
		rprintfInit (OLED_font1);

		SSD1306_GotoXY(25, 0);
		if (distance1_1_3 < 100000)
			{
			temp = distance1_1_3 * 10;
			printf ("%d.%dm", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
			}
		else
			{
			printf ("---.-m" ); // ���������� �� ����������
			}

		SSD1306_GotoXY(71, 0);
		Time_Display (TimesOld[1]);

		// ���������
		if ( structBuffer[1].Vacc != 0)
		{
			X_bat = 110;
			Y_bat = 11;
			SSD1306_DrawRectangle(0+X_bat, 0+Y_bat, 14, 7, SSD1306_COLOR_WHITE );
			SSD1306_DrawFilledRectangle(15+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
			VaccAvgTemp = structBuffer[1].Vacc;
			if (VaccAvgTemp >= 3500)	SSD1306_DrawFilledRectangle(2+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3690)	SSD1306_DrawFilledRectangle(X_bat+2+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3780)	SSD1306_DrawFilledRectangle(X_bat+2+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3850)	SSD1306_DrawFilledRectangle(X_bat+2+3+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
		}
		rprintfInit (OLED_font2);

		//---------------------------------------------------------------
		SSD1306_GotoXY(0, 32+6);
		if ( temp2 == 2)
			printf ("2>");
		else
			printf ("2 ");

		if (LoraRX [2].RSSI != 0) 	// ���� �����-�� ���������� ��������
			{
				if ( LoraRX [2].RSSI >= -140) // ���� ������� ������ -140, ������
					{
						if ( LoraRX [2].RSSI > -55 )
							SSD1306_DrawFilledRectangle( 24, 31+21, 85, 5, SSD1306_COLOR_WHITE );  // ��������, �����������
						else
						{
							SSD1306_DrawRectangle( 24, 31+21, 85, 5, SSD1306_COLOR_WHITE );
							SSD1306_DrawFilledRectangle( 24, 31+21, 85 + (55 + LoraRX [2].RSSI) , 5, SSD1306_COLOR_WHITE );  // ������� RSSI ( �� -55 �� -140 ��� )
						}
					}

				if ( LoraRX [2].RSSI <= -100) // ���� 3 ����� � �����
					SSD1306_GotoXY(18, 31+11);
				else
					SSD1306_GotoXY(25, 31+11); // ���� 2 ����� � �����

				rprintfInit (OLED_font1);
				printf ("%d dBm", LoraRX [2].RSSI);
			}
		else
			{
				SSD1306_GotoXY(25, 31+11); //
				rprintfInit (OLED_font1);
				printf ("Fail!");
			}

		if (LoraRX [2].Power > 0)
		{
			SSD1306_GotoXY(80, 31+11); // �������� �����������
			printf("P%d", LoraRX [2].Power);
			SSD1306_GotoXY(113, 31+11+10); // ������� �������
			printf("%02X", LoraRX [2].CounterTxP);
		}

		rprintfInit (OLED_font1);

		SSD1306_GotoXY(25, 31+0);
		if (distance1_2_3 < 100000)
			{
			temp = distance1_2_3 * 10;
			printf ("%d.%dm", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
			}
		else
			{
			printf ("---.-m" ); // ���������� �� ����������
			}
		SSD1306_GotoXY(71, 31+0);
		Time_Display (TimesOld[2]);

		// ���������
		if ( structBuffer[2].Vacc != 0)
		{
			X_bat = 110;
			Y_bat = 31+11;
			SSD1306_DrawRectangle(0+X_bat, 0+Y_bat, 14, 7, SSD1306_COLOR_WHITE );
			SSD1306_DrawFilledRectangle(15+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
			VaccAvgTemp = structBuffer[2].Vacc;
			if (VaccAvgTemp >= 3500)	SSD1306_DrawFilledRectangle(2+X_bat, 2+Y_bat, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3690)	SSD1306_DrawFilledRectangle(X_bat+2+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3780)	SSD1306_DrawFilledRectangle(X_bat+2+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
			if (VaccAvgTemp >= 3850)	SSD1306_DrawFilledRectangle(X_bat+2+3+3+3, Y_bat+2, 1, 3, SSD1306_COLOR_WHITE );
		}
	}


}

// ���������� ����� �������
void Display2Release	(void)
{
	u32 temp;

	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_GotoXY(0, 0);
	printf ("    Distance: ");

	rprintfInit (OLED_font2); 	// ����� �� OLED �������
	SSD1306_GotoXY(0, 10);
	temp = distance1_1_2 * 10;
	printf ("1-2:%d.%d m", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
	SSD1306_GotoXY(0, 27);
	temp = distance1_1_3 * 10;
	printf ("1-3:%d.%d m", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
	SSD1306_GotoXY(0, 44);
	temp = distance1_2_3 * 10;
	printf ("2-3:%d.%d m", temp / 10,  temp % 10 ); // ���������� �� ����� � ������

}

// ���������� � ���������
void Display3Release	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	for (i=0; i<SatNumGPS; i++) 	// ���������� ��� ������� �������� GPS
	{
		SatIDGPS [i] = (GPSFixData.Sats[i][0][0] - 0x30) * 10 + (GPSFixData.Sats[i][0][1] - 0x30) * 1 ;

		for (j=0; j<3; j++)
		{
			if ( GPSFixData.Sats[i][3][j] == '\0' ) 	// ������� ������� ���� � ����� S/N
				break;
		}

		if (j==3) 		// ��� �����
		{
			SatLevelsGPS [i] = (GPSFixData.Sats[i][3][0] - 0x30) * 100 + (GPSFixData.Sats[i][3][1] - 0x30) * 10 + (GPSFixData.Sats[i][3][2] - 0x30) * 1 ;
		}
		else if (j==2) 	// ��� �����
		{
			SatLevelsGPS [i] = (GPSFixData.Sats[i][3][0] - 0x30) * 10 + (GPSFixData.Sats[i][3][1] - 0x30) * 1 ;
		}
		else if (j==1) 	// ���� �����
		{
			SatLevelsGPS [i] = GPSFixData.Sats[i][3][0] - 0x30 ;
		}
//printf ("%d %d;", i, j);
	}

	SatSummGPS = 0; 			// ����� ������� �������� �� ��������� GPS
	for (i=0; i<SatNumGPS; i++)
	{
		SatSummGPS += SatLevelsGPS[i];
	}

	// ---------------------------- GLONASS �������� -------------------------------------------
	for (i=0; i<SatNumGLO; i++) 	// ���������� ��� ������� �������� GPS
		{
			SatIDGLO [i] = (GPSFixData.SatsGLO[i][0][0] - 0x30) * 10 + (GPSFixData.SatsGLO[i][0][1] - 0x30) * 1 ;

			for (j=0; j<3; j++)
			{
				if ( GPSFixData.SatsGLO[i][3][j] == '\0' ) 	// ������� ������� ���� � ����� S/N
					break;
			}

			if (j==3) 		// ��� �����
			{
				SatLevelsGLO [i] = (GPSFixData.SatsGLO[i][3][0] - 0x30) * 100 + (GPSFixData.SatsGLO[i][3][1] - 0x30) * 10 + (GPSFixData.SatsGLO[i][3][2] - 0x30) * 1 ;
			}
			else if (j==2) 	// ��� �����
			{
				SatLevelsGLO [i] = (GPSFixData.SatsGLO[i][3][0] - 0x30) * 10 + (GPSFixData.SatsGLO[i][3][1] - 0x30) * 1 ;
			}
			else if (j==1) 	// ���� �����
			{
				SatLevelsGLO [i] = GPSFixData.SatsGLO[i][3][0] - 0x30 ;
			}
	//printf ("%d %d;", i, j);
		}

		SatSummGLO = 0; 			 // ����� ������� �������� �� ��������� GLONASS
		for (i=0; i<SatNumGLO; i++)
		{
			SatSummGLO += SatLevelsGLO[i];
		}
	//---------------------------------------------------------------------------------------------

	//rprintfInit (OLED_font1); 	// ����� �� OLED �������
	SSD1306_GotoXY(0, 0);
	printf ("GPS:%d/%d", SatFixGPS, SatNumGPS );  		// ���������� ��������� ��� �������� ���������
	SSD1306_GotoXY(66, 0);
	printf ("GLO:%d/%d", SatFixGLO, SatNumGLO );  	// ���������� ��������� ��� �������� ���������

	// ������� ���� �������������� ��������� GPS � ������ ������
	SatSummUsedGPS = 0;
	for (i=0; i<SatNumGPS; i++)
	{
		for (j=0; j<12; j++) // ���������� ��� �������� �� �������, ���������� ������
		{
			if ( SatUsedGPS [j] == SatIDGPS [i] )
			{
				SatSummUsedGPS += SatLevelsGPS[i];  		// ����� S/N ���������, ������������ � �������
				SSD1306_DrawFilledRectangle(0 + i*4, 63-SatLevelsGPS[i], 2, SatLevelsGPS[i], SSD1306_COLOR_WHITE);  // ��������, ������������ � ������� - �������� �����
			}
			else
				SSD1306_DrawRectangle(0 + i*4, 63-SatLevelsGPS[i], 2, SatLevelsGPS[i], SSD1306_COLOR_WHITE);		  // ��������, �������������� � ������� - ������� ����������������
		}
	}
	// ������� ���� �������������� ��������� GLONASS � ������ ������
	SatSummUsedGLO = 0;
	for (i=0; i<SatNumGLO; i++)
	{
		for (j=0; j<12; j++) // ���������� ��� �������� �� �������, ���������� ������
		{
			if ( SatUsedGLO [j] == SatIDGLO [i] )
			{
				SatSummUsedGLO += SatLevelsGLO[i];  		// ����� S/N ���������, ������������ � �������
				SSD1306_DrawFilledRectangle(66 + i*4, 63-SatLevelsGLO[i], 2, SatLevelsGLO[i], SSD1306_COLOR_WHITE);  // ��������, ������������ � ������� - �������� �����
			}
			else
				SSD1306_DrawRectangle(66 + i*4, 63-SatLevelsGLO[i], 2, SatLevelsGLO[i], SSD1306_COLOR_WHITE);		  // ��������, �������������� � ������� - ������� ����������������
		}
	}

	//SSD1306_GotoXY(0, 10);
	//printf ("S=%d/%d", SatSummUsedGPS, SatSummGPS);

	//SSD1306_GotoXY(66, 10);
	//printf ("S=%d/%d", SatSummUsedGLO, SatSummGLO);

	SSD1306_GotoXY(0, 10);
	printf ("SN:%ddB", SatSummUsedGPS + SatSummUsedGLO);
	SSD1306_GotoXY(66, 10);
	printf ("TF:%d s", TTFF);

}
/*void Display14	(void)
{
	u32 temp;

	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// ����� �� OLED �������

//	SSD1306_GotoXY(0, 0);
//	printf (" sin(90) * 100 =");
//	SSD1306_GotoXY(0, 20);
//	printf(" %d", (s32) distance2);
//	SSD1306_GotoXY(0, 40);
//	printf(" %d", (s32) (1000 * sin( 3.1415/4 ) ) );

	SSD1306_GotoXY(0, 0);
	//printf ("   Distance 1-2 " );

//	printf (" Latit = %d ", structBuffer[2].Latitude );
//	SSD1306_GotoXY(0, 10);
//	printf (" grad = %d ", (u32) (grad*1000000.0) );
//	SSD1306_GotoXY(0, 20);
//	printf (" rad = %d ", (u32)(rad*1000000.0) );


	if (counterMaxRssiReset > 0) // ������ ���� �����
	{
		SSD1306_GotoXY(10, 0);
		rprintfInit (OLED_font2); 	// ����� �� OLED �������

		if ( structBuffer[1].Latitude == 0 || structBuffer[2].Latitude == 0 ) // ���� ���������� ������ �� ������ �� ����������
			printf ("--- m" );
		else
		{
			temp = distance1 * 10;
			printf ("%d.%d m", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
		}

		SSD1306_GotoXY(10, 20);
		rprintfInit (OLED_font1); 	// ����� �� OLED �������

		if ( structBuffer[1].Latitude == 0 || structBuffer[2].Latitude == 0 ) // ���� ���������� ������ �� ������ �� ����������
			printf ("--- m" );
		else
		{
			temp = distance2 * 10;
			printf ("%d.%d m", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
		}

	}
	else // ���� �� ����� , ���������� ���������� �� �������� �������������� �� ��������� �������� �����
	{
		SSD1306_GotoXY(10, 0);
		rprintfInit (OLED_font1); 	// ����� �� OLED �������

		if ( structBuffer[1].Latitude == 0 || structBuffer[2].Latitude == 0 ) // ���� ���������� ������ �� ������ �� ����������
			printf ("--- m" );
		else
		{
			temp = distance1 * 10;
			printf ("%d.%d m", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
		}

		SSD1306_GotoXY(10, 15);
		rprintfInit (OLED_font2); 	// ����� �� OLED �������

		if ( structBuffer[1].Latitude == 0 || structBuffer[2].Latitude == 0 ) // ���� ���������� ������ �� ������ �� ����������
			printf ("--- m" );
		else
		{
			temp = distance2 * 10;
			printf ("%d.%d m", temp / 10,  temp % 10 ); // ���������� �� ����� � ������
		}
	}

	rprintfInit (OLED_font1); 	// ����� �� OLED �������

	SSD1306_GotoXY(112, 10);
	if (counterTXDisplay > 0)
		printf("TX");

	SSD1306_GotoXY(112, 20);
	if (counterRXDisplay > 0)
		printf("RX");

	SSD1306_GotoXY(75, 35);
	if (LoraRssiMax != -130)
		printf("%d dBm", LoraRssiMax);

	//counter_AVG_1PPS

	if ( flag_1PPS_AVG_OK == 0)
	{ // ��������� ���������� 1PPS
	SSD1306_DrawRectangle (0, 58, 127, 5, SSD1306_COLOR_WHITE);
	SSD1306_DrawFilledRectangle (0, 58, (127 * counter_AVG_1PPS) / AVG_TIM4 , 5, SSD1306_COLOR_WHITE);

	SSD1306_GotoXY(0, 47);
	printf("AVR 1PPS: %d  ", counter_AVG_1PPS );
	}
	else
	{
		SSD1306_GotoXY(0, 50);
		printf ("Fix: %d", Fix);
	}


}
*/

void OLED_Putc1	(char c)  // ����� ������� �� �������. ����� 7x10 ��������
{
	SSD1306_Putc ( c, &Font_7x10, SSD1306_COLOR_WHITE );
}
void OLED_Putc2	(char c)  // ����� ������� �� �������. ����� 11x18 ��������
{
	SSD1306_Putc ( c, &Font_11x18, SSD1306_COLOR_WHITE );
}
void OLED_Putc3	(char c)  // ����� ������� �� �������. ����� 16x26 ��������
{
	SSD1306_Putc ( c, &Font_16x26, SSD1306_COLOR_WHITE );
}

// -----------------  ����� � ������� ������ � GPS ������
void ReceiveGPS (void)
{
	u8 scan;	// ������� ��� ������ GSV �������������������
	
	if ( triggerTIM3Interrupt == 1 ) 	// ��������  - ������ ������� �� GPS ������
	{
		triggerTIM3Interrupt = 0; 		// ���������� ���� ������� ������ � ������

    	countPacket ++ ;


		rprintfInit (Terminal);
	#ifdef DebugGpsToUart1
		printf("\n\r\n\r ------- NEW DATA ----------------------------------\n\r");
	#endif
		// ���� ����� ����������, �������� ������ � ������������ NMEA ���������
	//printf ("\n\r GPSBufWrPoint: %d ", GPSBufWrPoint); 	// ������� ��������� ��������� ������ � �����
		GPSBufWrPoint = 0; 				// ����� � 0, ���������� ��������� ������� � ������ ������

		NMEA_Parser_GGA();						// GGA - �������� ���������, ������ ����������
		NMEA_Parser_GSA_GPS();					// GSA - ������ GPS ��������� � �������, DOP

		NMEA_Parser_GSA_GLO();					// GSA - ������ GLONASS ��������� � �������

		NMEA_Parser_ZDA();      				// ZDA - ������ ����� � ����
		GPSTimeSec = GetGPSTimeSec(); 			// ������� �������, ������� ������� �����
		GPSTimeMilliSec	= GetGPSTimeMilliSec(); // ������� �������, ������������

		GSVStartPos = 0;
		SatNumGPS = 0;

	//	printf("\n\r ------- GPS --------------------------------------");
		NMEA_Parser_GSV_GPS();	// ������ ������ ������, ������� ����� ���������� GSV-GPS �����
		for (scan = 0; scan < GSVNumStringGPS-1; scan ++ )
			{
				NMEA_Parser_GSV_GPS();	// ��������� �������
			}

		GSVStartPos = 0;
		SatNumGLO = 0;

	//	printf("\n\r\n\r ------- Glonass ----------------------------------");
		NMEA_Parser_GSV_GLO ();	// ������ ������ ������, ������� ����� ���������� GSV-GLONASS �����
		for (scan = 0; scan < GSVNumStringGLO-1; scan ++ )
			{
				NMEA_Parser_GSV_GLO();	// ��������� �������
			}

	//  ������� ���������� � ������ �� ������ � u32
		if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
		{
			GPSLongitude = (GPSFixData.Longitude[0]-0x30)*100000000 + (GPSFixData.Longitude[1]-0x30)*10000000 + (GPSFixData.Longitude[2]-0x30)*1000000 +
					(GPSFixData.Longitude[3]-0x30)*100000 + (GPSFixData.Longitude[4]-0x30)*10000 + (GPSFixData.Longitude[6]-0x30)*1000 +
					(GPSFixData.Longitude[7]-0x30)*100 + (GPSFixData.Longitude[8]-0x30)*10  + (GPSFixData.Longitude[9]-0x30)*1 ;

			GPSLatitude = (GPSFixData.Latitude[0]-0x30)*10000000 + (GPSFixData.Latitude[1]-0x30)*1000000 + (GPSFixData.Latitude[2]-0x30)*100000 +
					(GPSFixData.Latitude[3]-0x30)*10000 + (GPSFixData.Latitude[5]-0x30)*1000 + (GPSFixData.Latitude[6]-0x30)*100 +
					(GPSFixData.Latitude[7]-0x30)*10 + (GPSFixData.Latitude[8]-0x30)*1 ;

			if (GPSFixData.NS == 'N')			GPS_N_S = 1 ; 	// ������� ��������-����� ������
				else							GPS_N_S = 0 ;

			if (GPSFixData.EW == 'E')			GPS_E_W = 0 ; 	// ��������-��������� �������
				else							GPS_E_W = 1 ;

			GPSAltitude = (GPSFixData.Altitude[0]-0x30)*1000 + (GPSFixData.Altitude[1]-0x30)*100 + (GPSFixData.Altitude[2]-0x30)*10 +
					(GPSFixData.Altitude[4]-0x30)*1;// + (GPSFixData.Altitude[4]-0x30)*10 + (GPSFixData.Altitude[6]-0x30)*1 ;
			}

	#ifdef DebugGpsToUart1
		printf("\n\r ------- BUFFER ----------------------------------\n\r");
		for (i=0; i < GPSBUFLENGTH; i++)
			{
			if (GPSBuf[i] == '\0') break;
			USART_SendByte(USART1, GPSBuf[i]);
			}

		printf("\n\r ------- DATA DECODED ----------------------------------\n\r");
		printf ("\n\r Year: \t\t");
		for (i=0; i<4; i++)
			{
			USART_SendByte(USART1, GPSFixData.Year[i]);
			}
		printf ("\n\r Month: \t\t");
		for (i=0; i<2; i++)
			{
			USART_SendByte(USART1, GPSFixData.Month[i]);
			}
		printf ("\n\r Day: \t\t");
		for (i=0; i<2; i++)
			{
			USART_SendByte(USART1, GPSFixData.Day[i]);
			}
		printf ("\n\r UTC Time: \t%c%c:%c%c:%c%c.%c%c",GPSFixData.Time[0],GPSFixData.Time[1],GPSFixData.Time[2],GPSFixData.Time[3],GPSFixData.Time[4],
				GPSFixData.Time[5], GPSFixData.Time[7], GPSFixData.Time[8]);
		//for (i=0; i<9; i++)
		//	{
		//	USART_SendByte(USART1, GPSFixData.Time[i]);
		//	}
		printf ("\n\r LocalTimeHr:\t");
		for (i=0; i<3; i++)
			{
			USART_SendByte(USART1, GPSFixData.LocalTimeHr[i]);
			}
		printf ("\n\r LocalTimeMn:\t");
		for (i=0; i<2; i++)
			{
			USART_SendByte(USART1, GPSFixData.LocalTimeMn[i]);
			}
		printf ("\n\r Altitude: \t\t");
		for (i=0; i<5; i++)
			{
			USART_SendByte(USART1, GPSFixData.Altitude[i]);
			}
		printf (" m\n\r Latitude: \t\t");
		for (i=0; i<9; i++)
			{
			USART_SendByte(USART1, GPSFixData.Latitude[i]);
			}
		printf (" %c\n\r Longitude: \t", GPSFixData.NS );
			for (i=0; i<10; i++)
			{
			USART_SendByte(USART1, GPSFixData.Longitude[i]);
			}
		printf (" %c\n\r ReceiverMode: \t", GPSFixData.EW);
		for (i=0; i<1; i++)
			{
			USART_SendByte(USART1, GPSFixData.ReceiverMode);
			}
		if (GPSFixData.ReceiverMode == '0')
			printf ("\t (NO solution)");
		if (GPSFixData.ReceiverMode == '1')
			printf ("\t (StandAlone)");
		if (GPSFixData.ReceiverMode == '2')
			printf ("\t (DGPS)");
	#endif
	//printf ("\n\r Tracked SATs: \t");
		j=0;
		for (i=0; i<2; i++)
			{
			if (GPSFixData.SatelliteNum[i] == '\0') break;
			//USART_SendByte(USART1, GPSFixData.SatelliteNum[i]);
			j++;		// ������� ����
			}
		if (j == 1) 								// ���� �����
			SatFix = GPSFixData.SatelliteNum[0] - 0x30;
		else if (j == 2)							// ��� �����
			SatFix = (GPSFixData.SatelliteNum[0] - 0x30) * 10 + (GPSFixData.SatelliteNum[1] - 0x30);			// �� ASCII ���� � �����
		else
			SatFix = 0;

	#ifdef DebugGpsToUart1
		printf ("\n\r GGA Fix SATs: \t%d ", SatFix);
		printf ("\n\r GSV GPS SATs: \t%d", SatNumGPS);
		printf ("\n\r GSV GLO SATs: \t%d", SatNumGLO);
		printf ("\n\r GSA SatFixGPS: \t%d", SatFixGPS);
		printf ("\n\r GSA SatFixGLO: \t%d", SatFixGLO);
	#endif
	//	for (i=0; i<SatNum; i++)
	//	{
		//	printf ("\n\r %d\tSAT# %c%c\tElev:%c%c�\tAzim:%c%c%c\t SNR:%c%c dB",i+1, GPSFixData.Sats[i][0][0], GPSFixData.Sats[i][0][1], GPSFixData.Sats[i][1][0], GPSFixData.Sats[i][1][1],
		//							GPSFixData.Sats[i][2][0], GPSFixData.Sats[i][2][1], GPSFixData.Sats[i][2][2], GPSFixData.Sats[i][3][0], GPSFixData.Sats[i][3][1]   );
	//	}

		if ( (Fix == 2 || Fix == 3) ) 	// ���� ���� 2D ��� 3D �������
		{
			if (triggerTTFF == 0)		// ����������� ��������� TTFF
			{
				triggerTTFF = 1;
				TTFF = countPacket;
			}
		}

		for (i=0; i<GPSBUFLENGTH; i++) // ������� ������
			{
			GPSBuf[i] = '\0';
			}
	}

}

//  ------------------------ MNEA PARCER ---- GPS GSA ------------------------------------
u8	NMEA_Parser_GSA_GPS	(void) // ������ DOP GPS
{
  u8 CommaPos = 0;
  u8 n = 0;
  u16 i, StartPos = 0, StartDOP=0;    // ������ ������

  while (!( (GPSBuf[StartPos] == '$')
		&&(GPSBuf[StartPos+1] == 'G')
		&&(GPSBuf[StartPos+2] == 'P')
		&&(GPSBuf[StartPos+3] == 'G')
		&&(GPSBuf[StartPos+4] == 'S')
		&&(GPSBuf[StartPos+5] == 'A'))
		&& StartPos < (GPSBUFLENGTH - sizeof("$GNGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.1,2.2,3.3*cs")+1))
	StartPos++;
//printf ("\n\r GSA Parser: "); // todo
//printf ("\n\r StartPos = %d", StartPos); // todo
  // ������ ������ ������� ������ �������, ����������� �����
  if (StartPos > GPSBUFLENGTH - sizeof("$GNGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.1,2.2,3.3*cs")-1) // ������ ������� ������, ������� ��� �� ������ �� �����
	  {
//printf ("\n\rGSA offset ERROR!");
	  //return 0;
	  }

  if (ControlCheckSum(StartPos) == 0) // �������� ����������� ����� CRC
 	  {
//printf ("\n\r GSA CRC ERROR!"); // todo
 	  //return 0; // �������� ����������� ����� todo
 	  }
  else
   	  {
//printf ("\n\r GSA CRC OK"); // todo
 	  }

  if (GPSBuf[StartPos+7] == 'A')
	  AutoSel_2D3D = 1;	// �������������� ����� ������ 2D/3D
  else  if (GPSBuf[StartPos+7] == 'M')
	  AutoSel_2D3D = 0;	// ������ ����� ������ 2D/3D
  else
	  AutoSel_2D3D = 2; // �� �������������� ��������

  if (GPSBuf[StartPos+9] == '1')
	  Fix = 1;			// ��� �������
  else if (GPSBuf[StartPos+9] == '2')
	  Fix = 2;			// 2D - �� ���������� ������
  else if (GPSBuf[StartPos+9] == '3')
	  Fix = 3;			// 3D - ������ � ���������� ����������
  else
	  Fix = 0;			// �� �������������� ��������

  for (i=0; i<12; i++)  // ������� ������� ������� ������������ ���������
  {
	  SatUsedGPS [i] = 0;
  }

//printf ("\n\r");
//for (i = StartPos+10; i < GPSBUFLENGTH ; i++)   // ���� ����� ������
//{
//	if (GPSBuf[i] == '$') break;
//	USART_SendByte(USART1, GPSBuf[i]);
//}

  for (i = StartPos+10; i < GPSBUFLENGTH ; i++)   // ���� ��� ����� ������ ��� 12-� �������
    {
	  if ( (GPSBuf[i] == ',') && (GPSBuf[i+1] == '0') && (GPSBuf[i+2] == ',')  ) 	// ���� ��� ��������
      {
    	i+=1 ; 			// ���������� ���� ����� ��������
        CommaPos ++; 	// ���������� ������� �������
//printf ("\n\r ,0, i: %d C:%d ",i, CommaPos); // todo
      }
      else
      {
    	SatUsedGPS [n] = (GPSBuf[i+1]-0x30)*10 + (GPSBuf[i+2]-0x30); 	// ����� ��������

    	CommaPos ++; 	// ���������� ������� �������
    	n++ ;			// ������� ���������
    	i+=2 ; 			// ������� �� ��������� ��������
//printf ("\n\r i: %d C:%d ",i, CommaPos); // todo
      }
	  SatFixGPS = n; 	// ���������� ��������� GPS, ��������� ��� ��������

      if (CommaPos >= 12) // �������� 12-� �������
      {
//printf ("\n\r break (C>=12)");
      	break;
      }
    }
  StartDOP = i+1; // ��������� �������, ����� ������� ���������� DOP ���������

//printf ("\n\r Used SATs: ");
//for (i=0; i<12; i++)  // ������� ������� ������� ������������ ���������
//{
//  printf ("%d ", SatUsed [i] ); 	// todo
//}

//printf ("\n\rStartDOP:%d ", StartDOP); 	// todo
//printf ("\n\r");
//for (i = StartDOP; i < GPSBUFLENGTH ; i++)   // ����� ������ � ��������
//{
//	if (GPSBuf[i] == '$') break;
//	USART_SendByte(USART1, GPSBuf[i]);
//}

  	i = 0; // ����� ��������

	if ( (GPSBuf[StartDOP+i] == ',') && (GPSBuf[StartDOP+i+1] == '0') && (GPSBuf[StartDOP+i+2] == ',')  ) 	// ���� ��� ��������
	{
		PDOP = 0;
		i += 2;
	}
	else
	{
		PDOP = (GPSBuf[StartDOP+i+1]-0x30)*100 + (GPSBuf[StartDOP+i+3]-0x30)*10 + (GPSBuf[StartDOP+i+4]-0x30)*1 ; 	// PDOP*100
		i += 5;
	}
//printf ("\n\rPDOP:%d ", PDOP); // todo

	if ( (GPSBuf[StartDOP+i] == ',') && (GPSBuf[StartDOP+i+1] == '0') && (GPSBuf[StartDOP+i+2] == ',')  ) 	// ���� ��� ��������
	{
		HDOP = 0;
		i += 2;
	}
	else
	{
		HDOP = (GPSBuf[StartDOP+i+1]-0x30)*100 + (GPSBuf[StartDOP+i+3]-0x30)*10 + (GPSBuf[StartDOP+i+4]-0x30)*1; 	// HDOP*100
		i += 5;
	}
//printf ("\n\rHDOP:%d ", HDOP); // todo

	if ( (GPSBuf[StartDOP+i] == ',') && (GPSBuf[StartDOP+i+1] == '0') && (GPSBuf[StartDOP+i+2] == '*')  ) 	// ���� ��� ��������
	{
		VDOP = 0;
		//i += 2;
	}
	else
	{
		VDOP = (GPSBuf[StartDOP+i+1]-0x30)*100 + (GPSBuf[StartDOP+i+3]-0x30)*10 + (GPSBuf[StartDOP+i+4]-0x30)*1; 	// VDOP*100
		//i += 4;
	}
//printf ("\n\rVDOP:%d ", VDOP); // todo

  //HDOP = (GPSBuf[StartDOP+4]-0x30)*10 + (GPSBuf[StartDOP+4+2]-0x30) ; 	// HDOP*10
  //VDOP = (GPSBuf[StartDOP+8]-0x30)*10 + (GPSBuf[StartDOP+8+2]-0x30) ; 	// VDOP*10

  return 1;

}

//  ------------------------ GLONASS GSA -----------------------------
u8	NMEA_Parser_GSA_GLO	(void) // ������ DOP GPS
{
  u8 CommaPos = 0;
  u8 n = 0;
  u16 i, StartPos = 0, StartDOP=0;    // ������ ������

  while (!( (GPSBuf[StartPos] == '$')
		&&(GPSBuf[StartPos+1] == 'G')
		&&(GPSBuf[StartPos+2] == 'L')
		&&(GPSBuf[StartPos+3] == 'G')
		&&(GPSBuf[StartPos+4] == 'S')
		&&(GPSBuf[StartPos+5] == 'A'))
		&& StartPos < (GPSBUFLENGTH - sizeof("$GNGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.1,2.2,3.3*cs")+1))
	StartPos++;
//printf ("\n\r GSA Parser: "); // todo
//printf ("\n\r StartPos = %d", StartPos); // todo
  // ������ ������ ������� ������ �������, ����������� �����
  if (StartPos > GPSBUFLENGTH - sizeof("$GNGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.1,2.2,3.3*cs")-1) // ������ ������� ������, ������� ��� �� ������ �� �����
	  {
//printf ("\n\rGSA offset ERROR!");
	  //return 0;
	  }

  if (ControlCheckSum(StartPos) == 0) // �������� ����������� ����� CRC
 	  {
//printf ("\n\r GSA CRC ERROR!"); // todo
 	  //return 0; // �������� ����������� ����� todo
 	  }
  else
   	  {
//printf ("\n\r GSA CRC OK"); // todo
 	  }

 /* if (GPSBuf[StartPos+7] == 'A')
	  AutoSel_2D3D = 1;	// �������������� ����� ������ 2D/3D
  else  if (GPSBuf[StartPos+7] == 'M')
	  AutoSel_2D3D = 0;	// ������ ����� ������ 2D/3D
  else
	  AutoSel_2D3D = 2; // �� �������������� ��������

  if (GPSBuf[StartPos+9] == '1')
	  Fix = 1;			// ��� �������
  else if (GPSBuf[StartPos+9] == '2')
	  Fix = 2;			// 2D - �� ���������� ������
  else if (GPSBuf[StartPos+9] == '3')
	  Fix = 3;			// 3D - ������ � ���������� ����������
  else
	  Fix = 0;			// �� �������������� ��������
*/

  for (i=0; i<12; i++)  // ������� ������� ������� ������������ ���������
  {
	  SatUsedGLO [i] = 0;
  }

//printf ("\n\r");
//for (i = StartPos+10; i < GPSBUFLENGTH ; i++)   // ���� ����� ������
//{
//	if (GPSBuf[i] == '$') break;
//	USART_SendByte(USART1, GPSBuf[i]);
//}

  for (i = StartPos+10; i < GPSBUFLENGTH ; i++)   // ���� ��� ����� ������ ��� 12-� �������
    {
	  if ( (GPSBuf[i] == ',') && (GPSBuf[i+1] == '0') && (GPSBuf[i+2] == ',')  ) 	// ���� ��� ��������
      {
    	i+=1 ; 			// ���������� ���� ����� ��������
        CommaPos ++; 	// ���������� ������� �������
//printf ("\n\r ,0, i: %d C:%d ",i, CommaPos); // todo
      }
      else
      {
    	SatUsedGLO [n] = (GPSBuf[i+1]-0x30)*10 + (GPSBuf[i+2]-0x30); 	// ����� ��������

    	CommaPos ++; 	// ���������� ������� �������
    	n++ ;			// ������� ���������
    	i+=2 ; 			// ������� �� ��������� ��������
//printf ("\n\r i: %d C:%d ",i, CommaPos); // todo
      }
	  SatFixGLO = n; 	// ���������� ��������� GLONASS, ��������� ��� ��������

      if (CommaPos >= 12) // �������� 12-� �������
      {
//printf ("\n\r break (C>=12)");
      	break;
      }
    }
  StartDOP = i+1; // ��������� �������, ����� ������� ���������� DOP ���������

//printf ("\n\r Used SATs: ");
//for (i=0; i<12; i++)  // ������� ������� ������� ������������ ���������
//{
//  printf ("%d ", SatUsed [i] ); 	// todo
//}

//printf ("\n\rStartDOP:%d ", StartDOP); 	// todo
//printf ("\n\r");
//for (i = StartDOP; i < GPSBUFLENGTH ; i++)   // ����� ������ � ��������
//{
//	if (GPSBuf[i] == '$') break;
//	USART_SendByte(USART1, GPSBuf[i]);
//}

  	i = 0; // ����� ��������

	if ( (GPSBuf[StartDOP+i] == ',') && (GPSBuf[StartDOP+i+1] == '0') && (GPSBuf[StartDOP+i+2] == ',')  ) 	// ���� ��� ��������
	{
		//PDOP = 0;
		i += 2;
	}
	else
	{
		//PDOP = (GPSBuf[StartDOP+i+1]-0x30)*100 + (GPSBuf[StartDOP+i+3]-0x30)*10 + (GPSBuf[StartDOP+i+4]-0x30)*1 ; 	// PDOP*100
		i += 5;
	}
//printf ("\n\rPDOP:%d ", PDOP); // todo

	if ( (GPSBuf[StartDOP+i] == ',') && (GPSBuf[StartDOP+i+1] == '0') && (GPSBuf[StartDOP+i+2] == ',')  ) 	// ���� ��� ��������
	{
		//HDOP = 0;
		i += 2;
	}
	else
	{
		//HDOP = (GPSBuf[StartDOP+i+1]-0x30)*100 + (GPSBuf[StartDOP+i+3]-0x30)*10 + (GPSBuf[StartDOP+i+4]-0x30)*1; 	// HDOP*100
		i += 5;
	}
//printf ("\n\rHDOP:%d ", HDOP); // todo

	if ( (GPSBuf[StartDOP+i] == ',') && (GPSBuf[StartDOP+i+1] == '0') && (GPSBuf[StartDOP+i+2] == '*')  ) 	// ���� ��� ��������
	{
		//VDOP = 0;
		//i += 2;
	}
	else
	{
		//VDOP = (GPSBuf[StartDOP+i+1]-0x30)*100 + (GPSBuf[StartDOP+i+3]-0x30)*10 + (GPSBuf[StartDOP+i+4]-0x30)*1; 	// VDOP*100
		//i += 4;
	}
//printf ("\n\rVDOP:%d ", VDOP); // todo

  //HDOP = (GPSBuf[StartDOP+4]-0x30)*10 + (GPSBuf[StartDOP+4+2]-0x30) ; 	// HDOP*10
  //VDOP = (GPSBuf[StartDOP+8]-0x30)*10 + (GPSBuf[StartDOP+8+2]-0x30) ; 	// VDOP*10

  return 1;

}

u8 	NMEA_Parser_GGA	(void)
{
  u8 CommaPos = 0;
  u8 k = 0;
  u16 i,StartPos = 0;    // ������ ������

  while (!( (GPSBuf[StartPos] == '$')
        &&(GPSBuf[StartPos+3] == 'G')
        &&(GPSBuf[StartPos+4] == 'G')
        &&(GPSBuf[StartPos+5] == 'A'))
        && StartPos < 7)
    StartPos++;
//printf ("\n\r GGA Parser: "); // todo
//printf ("\n\r StartPos = %d", StartPos); // todo
  // ������ ������ ������� ������ �������, ����������� �����
  if (StartPos>5)
	  {
	 // printf ("\n\rStartPos>5\n\r");
	  return 0;
	  }

  if (ControlCheckSum(StartPos) == 0)
	  {
//printf ("\n\r GGA CRC ERROR!"); // todo
	  //return 0; // �������� ����������� ����� todo
	  }
  else
  	  {
//printf ("\n\r GGA CRC OK"); // todo
	  }

  memset(&GPSFixData, 0, sizeof(GPSFixData));   // ������� ���������

  for (i = StartPos+6; i < GPSBUFLENGTH ; i++)   // ���� ��� ����� ������ ��� 10-� �������
  {
    if (GPSBuf[i] == ',')
    {
      CommaPos++; // ���������� ������� �������
      k=0;
    }
    else
      switch (CommaPos) // ���������� ���, ��� ������ ����������� ��������. �������������� ������ (��� ������� ������) �� �������� ����� � ����������� ���������� USART1_IRQHandler()
      {
        // ����� ������ ������� ���� �����, �� �� ������� ��� ����� �� ZDA ���������, ��������� � �����
        case 2: if(k < sizeof(GPSFixData.Latitude)-1) GPSFixData.Latitude[k++]=GPSBuf[i]; break;
        case 3: if(k < sizeof(GPSFixData.NS)) GPSFixData.NS=GPSBuf[i]; break;
        case 4: if(k < sizeof(GPSFixData.Longitude)-1) GPSFixData.Longitude[k++]=GPSBuf[i]; break;
        case 5: if(k < sizeof(GPSFixData.EW)) GPSFixData.EW=GPSBuf[i]; break;
        case 6: if(k < sizeof(GPSFixData.ReceiverMode))  GPSFixData.ReceiverMode=GPSBuf[i]; break;
        case 7: if(k < sizeof(GPSFixData.SatelliteNum)-1) GPSFixData.SatelliteNum[k++]=GPSBuf[i]; break;
        case 9: if(k < sizeof(GPSFixData.Altitude)-1) GPSFixData.Altitude[k++]=GPSBuf[i]; break;
        case 10: return 1; break;
        default: break;
      }
  }
  return 1;
}

u8  NMEA_Parser_ZDA	(void)
{
  u8 CommaPos = 0;
  u8 k;              		// ������ �������
  u16 i,StartPos = 0;    	// ������ ������

  while (!( (GPSBuf[StartPos] == '$')
        &&(GPSBuf[StartPos+3] == 'Z')
        &&(GPSBuf[StartPos+4] == 'D')
        &&(GPSBuf[StartPos+5] == 'A'))
        && StartPos < (GPSBUFLENGTH - sizeof("$GNZDA,hhmmss.ss,dd,mm,yyyy,+hh,mm*cs")+1))
    StartPos++;
//printf ("\n\r ZGA Parser: "); // todo
//printf ("\n\r StartPos = %d", StartPos); // todo
  // ������ ������ ������� ������ �������, ����������� ���� �����
  if (StartPos > (GPSBUFLENGTH - sizeof("$GNZDA,hhmmss.ss,dd,mm,yyyy,+hh,mm*cs")-1))
	  {
//printf ("\n\r ZDA offset ERROR!"); // todo
	  return 0;
	  }

  if (ControlCheckSum(StartPos) == 0) // �������� ����������� �����
  {
//printf ("\n\r ZDA CRC ERROR!"); // todo
  return 0; // �������� ����������� �����
  }
//printf ("\n\r ZDA CRC OK"); // todo

  for (i = StartPos+5; i < GPSBUFLENGTH ; i++)   // ���� ��� ����� ������ ��� 7-� �������
  {
    if (GPSBuf[i] == ',')
    {
      CommaPos++; // ���������� ������� �������
      k=0;
    }
    else
      switch (CommaPos) // ���������� ���, ��� ������ ����������� ��������. �������������� ������ (��� ������� ������) �� �������� ����� � ����������� ���������� USART1_IRQHandler()
      {
        case 1: if(k < sizeof(GPSFixData.Time)-1) GPSFixData.Time[k++]=GPSBuf[i]; break;
        case 2: if(k < sizeof(GPSFixData.Day)-1) GPSFixData.Day[k++]=GPSBuf[i]; break;
        case 3: if(k < sizeof(GPSFixData.Month)-1) GPSFixData.Month[k++]=GPSBuf[i]; break;
        case 4: if(k < sizeof(GPSFixData.Year)-1) GPSFixData.Year[k++]=GPSBuf[i];  break;
        case 5: if(k < sizeof(GPSFixData.LocalTimeHr)-1) GPSFixData.LocalTimeHr[k++]=GPSBuf[i]; break;
        case 6: if(k < sizeof(GPSFixData.LocalTimeMn)-1) GPSFixData.LocalTimeMn[k++]=GPSBuf[i]; break;
        case 7: return 1; break;
        default: break;
      }
  }
  return 1;
}

u32 GetGPSTimeSec 	(void) 		// �������� ����� �������� ZDA ���������
{
	u32 THH = 0, TMM = 0, TSS = 0 ;
	THH = (GPSFixData.Time[0]-0x30)*10 + (GPSFixData.Time[1]-0x30);		// ���� �� ZDA ���������
	TMM = (GPSFixData.Time[2]-0x30)*10 + (GPSFixData.Time[3]-0x30);		// ������ �� ZDA ���������
	TSS = (GPSFixData.Time[4]-0x30)*10 + (GPSFixData.Time[5]-0x30);		// ������� �� ZDA ���������
	//TmSmS = (GPSFixData.Time[7]-0x30)*10 + (GPSFixData.Time[8]-0x30);	// ������� ����� ������ �� ZDA ���������

	return((THH*3600 + TMM*60 + TSS)); // ���������� ����� ����� ������ �� ������� �����, ��� ������� �����
}
u32 GetGPSTimeMilliSec 	(void) 	// �������� ����� �������� ZDA ���������
{
	u32  TmSmS;
	TmSmS = (GPSFixData.Time[7]-0x30)*10 + (GPSFixData.Time[8]-0x30);	// ������� ����� ������ �� ZDA ���������
	return( TmSmS ) ;   // ���������� ������� �����
}

u8  NMEA_Parser_GSV_GPS	(void)
{
  u8 CommaPos = 0;
  u8 k = 0;              // ������ �������
  u16 i = 0;

  while (!( (GPSBuf[GSVStartPos] == '$')
		&&(GPSBuf[GSVStartPos+2] == 'P') // ������ GPS ��������
        &&(GPSBuf[GSVStartPos+3] == 'G')
        &&(GPSBuf[GSVStartPos+4] == 'S')
        &&(GPSBuf[GSVStartPos+5] == 'V'))
        && GSVStartPos < (GPSBUFLENGTH - sizeof("$GPGSV,n,n,nn,11,22,333,44,11,22,333,44,11,22,333,44,11,22,333,44*cs")+1))
    GSVStartPos++;  // ���������� ������ ��������� GSV ������ � �������

  // ������ ������ ������� ������ �������, ����������� ���
  if (GSVStartPos > (GPSBUFLENGTH - sizeof("$GPGSV,n,n,nn,11,22,333,44,11,22,333,44,11,22,333,44,11,22,333,44*cs")-1))
	  {
//printf ("\n\r return 0 offset so big "); // todo
	  return 0;
	  }
	  
  GSVNumStringGPS = GPSBuf[GSVStartPos+7] - 0x30 ; // ���������� GSV-GPS ����� � �������

  if (ControlCheckSum(GSVStartPos) == 0)
	  {
//printf ("\t\tGSV CRC error! "); // todo
	 // return 0;  // �������� ����������� �����
	  }

  for (i = GSVStartPos+5; i < GPSBUFLENGTH ; i++)    // ���� ��� ����� ������ ��� * � �������� ������
  {
    if (GPSBuf[i] == '*')
    {
      GSVStartPos = i;		// �������� ������ �� ������� ������
      SatNumGPS++;				// ���������� ����� �������� ��������
      //printf ("\n\rGPSSatNum(*): \t%d \tstart: %d", SatNum, GSVStartPos); // todo
      return 1;
    }

    if (GPSBuf[i] == ',')
    {
      if ((CommaPos == 7) || (CommaPos == 11) || (CommaPos == 15) || (CommaPos == 19))
	  {
	    SatNumGPS++;  	// ���������� ����� �������� ��������
//printf ("\n\rGPSSatNum(,): \t%d \tstart: %d", SatNum, GSVStartPos); // todo
	  }
      CommaPos++; k=0;  	// ���������� ������� �������
    }
    else
      switch (CommaPos) // ���������� ���, ��� ������ ����������� ��������. �������������� ������ (��� ������� ������) �� �������� ����� � ����������� ���������� USART1_IRQHandler()
      { // ����� ������ ���� ������� ���� ���������� �� ����� ���������� ���������, �� ����� ���������� GSV ������� � � ������ ���� ���������� GSV �������. ��� ���������� ��� �� �����
        case 4: case 8: case 12: case 16:  if(k < sizeof(ONESAT)-1) GPSFixData.Sats[SatNumGPS][0][k++]=GPSBuf[i]; break;  // ����� ��������. case 4: case 8: case 12: case 16: - ������ �������,
//		����� ������� ���� ��������������� ����������. � ������ GSV ������� ���� ���������� � �������� 4-� ���������, ������� 20-� ������� ����� �� ������������
        case 5: case 9: case 13: case 17:  if(k < sizeof(ONESAT)-1) GPSFixData.Sats[SatNumGPS][1][k++]=GPSBuf[i]; break;  // ���� �����
        case 6: case 10: case 14: case 18: if(k < sizeof(ONESAT)-1) GPSFixData.Sats[SatNumGPS][2][k++]=GPSBuf[i]; break;  // ������
        case 7: case 11: case 15: case 19: if(k < sizeof(ONESAT)-1) GPSFixData.Sats[SatNumGPS][3][k++]=GPSBuf[i]; break;  // ������� �������, SNR
        default: break;
      }
  }
  return 1;
}

u8  NMEA_Parser_GSV_GLO (void)
{
  u8 CommaPos = 0;
  u8 k = 0;              // ������ �������
  u16 i = 0;

  while (!( (GPSBuf[GSVStartPos] == '$')
		&&(GPSBuf[GSVStartPos+2] == 'L') 	// ������ GLONASS ��������
        &&(GPSBuf[GSVStartPos+3] == 'G')
        &&(GPSBuf[GSVStartPos+4] == 'S')
        &&(GPSBuf[GSVStartPos+5] == 'V'))
        && GSVStartPos < (GPSBUFLENGTH - sizeof("$GPGSV,n,n,nn,11,22,333,44,11,22,333,44,11,22,333,44,11,22,333,44*cs")+1))
    GSVStartPos++;

  // ������ ������ ������� ������ �������, ����������� ���
  if (GSVStartPos > (GPSBUFLENGTH - sizeof("$GPGSV,n,n,nn,11,22,333,44,11,22,333,44,11,22,333,44,11,22,333,44*cs")-1))
	  {
//printf ("\n\r return 0 offset so big "); // todo
	  return 0;
	  }

  GSVNumStringGLO = GPSBuf[GSVStartPos+7] - 0x30 ; // ���������� GSV-GLONASS ����� � �������

  if (ControlCheckSum(GSVStartPos) == 0)
	  {
//printf ("\t\tGSV CRC error! "); // todo
	 // return 0;  // �������� ����������� �����
	  }

  for (i = GSVStartPos+5; i < GPSBUFLENGTH ; i++)    // ���� ��� ����� ������ ��� * � �������� ������
  {
    if (GPSBuf[i] == '*')
    {
      GSVStartPos = i;
      SatNumGLO ++;
	  //printf ("\n\rGLOSatNum(*): \t%d \tstart: %d ", SatNumGLO, GSVStartPos); // todo
      return 1;
    }

    if (GPSBuf[i] == ',')
    {
      if ((CommaPos == 7) || (CommaPos == 11) || (CommaPos == 15) || (CommaPos == 19))
    	  {
    	  	  SatNumGLO ++;  // ���������� ����� �������� ��������
    	  	  //printf ("\n\rGLOSatNum(,): \t%d \tstart: %d ", SatNumGLO, GSVStartPos); // todo
    	  }
      CommaPos++; k=0;  // ���������� ������� �������
    }
    else
      switch (CommaPos) // ���������� ���, ��� ������ ����������� ��������. �������������� ������ (��� ������� ������) �� �������� ����� � ����������� ���������� USART1_IRQHandler()
      { // ����� ������ ���� ������� ���� ���������� �� ����� ���������� ���������, �� ����� ���������� GSV ������� � � ������ ���� ���������� GSV �������. ��� ���������� ��� �� �����
        case 4: case 8: case 12: case 16:  if(k < sizeof(ONESAT)-1) GPSFixData.SatsGLO[SatNumGLO][0][k++]=GPSBuf[i]; break;  // ����� ��������. case 4: case 8: case 12: case 16: - ������ �������, ����� ������� ���� ��������������� ����������. � ������ GSV ������� ���� ���������� � �������� 4-� ���������, ������� 20-� ������� ����� �� ������������
        case 5: case 9: case 13: case 17:  if(k < sizeof(ONESAT)-1) GPSFixData.SatsGLO[SatNumGLO][1][k++]=GPSBuf[i]; break;  // ���� �����
        case 6: case 10: case 14: case 18: if(k < sizeof(ONESAT)-1) GPSFixData.SatsGLO[SatNumGLO][2][k++]=GPSBuf[i]; break;  // ������
        case 7: case 11: case 15: case 19: if(k < sizeof(ONESAT)-1) GPSFixData.SatsGLO[SatNumGLO][3][k++]=GPSBuf[i]; break;  // ������� �������, SNR
        default: break;
      }
  }
  return 1;
}

u8  ControlCheckSum 	(u16 StartIndex)
{
  u8  CheckSum = 0, MessageCheckSum = 0;   // ����������� �����
  u16 i = StartIndex+1;                	// ��������� �� ���� ��� ������ �� ������� $

  while (GPSBuf[i]!='*')
  {
    CheckSum^=GPSBuf[i];
    if (++i == GPSBUFLENGTH) return 0; 			// �� ������ ������� ����������� �����
  }

  if (GPSBuf[++i]>0x40) MessageCheckSum=(GPSBuf[i]-0x37)<<4; 	// ������� ��� ���������� ��������� ����������
  else                  MessageCheckSum=(GPSBuf[i]-0x30)<<4;  	// � ����������������� �����, �������������� � ���� ASCII
  if (GPSBuf[++i]>0x40) MessageCheckSum+=(GPSBuf[i]-0x37);
  else                  MessageCheckSum+=(GPSBuf[i]-0x30);

  if (MessageCheckSum != CheckSum) return 0;	// ������������ �������� ����������� �����

  return 1; 	// ��� ��
}

//-------------------- ���������, ���������� � �������� ������� ������� 1PPS -------------------
void Measurement1PPS	(void)
{
	if ( flag_1PPS_Update == 1 )	// ���������� ��������
	{
		flag_1PPS_Update = 0;  // ����� ����� ��� ������������ ������������ ����������

		if ( (TIM4_fix >= 37000) && (TIM4_fix <= 37400) && (counter_TIM4_fix >= 90) && (counter_TIM4_fix <= 92)  )
			{ // ������ 1PPS � �������� �����

				for (i_avg = AVG_TIM4-1; i_avg > 0; i_avg -- )	// ����� ������ � ������� ������
				{
					averageTIM4 [i_avg] = averageTIM4 [i_avg-1] ;
				}
				averageTIM4 [0] = TIM4_fix; // ������� �������� - � 0 ������ �������

				summ = 0;
				for (i_avg = 0; i_avg < AVG_TIM4; i_avg ++ ) // ����� ��������� �������
				{
					summ = summ + averageTIM4 [i_avg] ;
				}
				averageTIM4_temp = summ / AVG_TIM4 ;		// �������� �������

				flag_1PPS_fail = 0;

				counter_AVG_1PPS ++;
				if ((counter_AVG_1PPS >= AVG_TIM4+2) )//&& (flag_sync_stop == 0) ) 			// ���������� ���������� ����� ����� ������� +2
				{
					counter_AVG_1PPS = AVG_TIM4+2;
					averageTIM4_ready = averageTIM4_temp; 	// �������� ������� �������� ����� 32 ����������
					flag_1PPS_AVG_OK = 1; 					// ���� ���������� �������� �������� 1PPS
				}
//					if (counter_AVG_1PPS >= AVG_TIM4+5) 		// ���������� ���������� ����� ����� ������� +5
//					{
//						counter_AVG_1PPS = AVG_TIM4+5;
//						flag_sync_stop = 1; 					// ���� ��� ����� �������������
//					}

			}
			else // ���� ��� ��������� ������� 1PPS
			{
				flag_1PPS_fail = 1;
				counter_TIM4_fix = 0;
				//flag_1PPS_AVG_OK = 0;
				counter_AVG_1PPS = 0;
			}


#ifdef	Debug1PPSMeasurementToUART
		rprintfInit (Terminal); // ������������� printf (). ����� � ��������
		for (i=0; i<10; i++)
		{
			printf("%d\t", averageTIM4[i] );
		}
		printf("\tc: %d\tTemp: %d\tReady: %d\tfail: %d\tOK: %d\tSt: %d\r\n", counter_AVG_1PPS, averageTIM4_temp, averageTIM4_ready, flag_1PPS_fail, flag_1PPS_AVG_OK, flag_sync_stop );
#endif
	}	
	
}

// --------------------- ���������� ���������� �������	 ������� 1PPS  --------------------------
void Average1PPS		(void)
{
//--------------------------------- ������� 1PPS -----------------------------------------
	   if (flag_1PPS_pulse == 1) 	// ������ ������� 1PPS �� ������� ��� �� �������� ����������
	   {
//OnLED0();
		   flag_1PPS_pulse = 0;

			if (flag_1PPS_AVG_OK == 0)		// ���� ����� ������ ������� �������� ������� 1PPS ��� �� ���� �� ��������
			   	{
				OnLED0();				//
				counterLed0 = 500; 		// �������� ����� ��������� �� 500 ��
			   	}
			else
				{

				}

			sec_GPS_to_display = GPSTimeSec + 1 ; 	// ������� �� GPS ��� �����������, ����� �� ����
			sec_RTC_to_display = SecRTC ;			// ������� �� ������� 1 ��� �����������

			sec_div = SecRTC % 60;		// ���������� ����� ������� �������, ����������� ����� ���������� �����
			if ( sec_div == 0+LoraTxTime || sec_div == 10+LoraTxTime || sec_div == 20+LoraTxTime || sec_div == 30+LoraTxTime || sec_div == 40+LoraTxTime || sec_div == 50+LoraTxTime ) // �������� ������� ��� ��������
			{
				flagTX = 1;			// �������� �� ����������� ���������
				//flagTXDisplay = 1;	// ���� ��� �����������
			}
		//	if ( sec_div == 0+LoraTxTime+1 || sec_div == 10+LoraTxTime+1 || sec_div == 20+LoraTxTime+1 || sec_div == 30+LoraTxTime+1 || sec_div == 40+LoraTxTime+1 || sec_div == 50+LoraTxTime+1 ) // �������� ������� ��� ��������
		//	{
		//		flagTXDisplay = 0;	// ����� ���� ��� ����������� �� ��������� ������� todo ���������� �� ������ sysclk
		//	}
//OffLED0();
		}

}

// --------------------- LORA ----------------------------------------------------------------------
void LoraProcess		(void)
{
	switch ( Radio -> Process())
			 {
	 	 	 case RF_IDLE:
	 	 		 break;

			 case RF_BUSY:
				 break;

			 case RF_TX_DONE:
 	 	 	 	 counterTXDisplay = 0;		// TX ������ �� ������ � ����� ������� ���������
 	 	 	 	 Radio -> StartRx();		// ��������� ���������� �� �����

			#ifdef DebugLORAToUART
				rprintfInit (Terminal); // ������������� printf (). ����� � ��������
				printf(" RF_TX_DONE StartRX \n\r");
			#endif
				 break;

			 case RF_RX_TIMEOUT:
			#ifdef DebugLORAToUART
				rprintfInit (Terminal); // ������������� printf (). ����� � ��������
				printf(" RF_RX_TIMEOUT \n\r");
			#endif	 
				 break;

			 case RF_RX_DONE: //---------------------------------------------------------------- ������ ����� -------------------------------------------------------

				// counterRXDisplay = 500;	// ������ RX 500 ��
				 countOK ++; // ������� ������ ���������� �������


				 if (CRCerror == 1) // ������ CRC
				 {
					CRCerror = 0;  // ���������� ������, ���� �������

					//OnLED0();				// ������ �����
					//counterLed0 = 100; 		// �������� ����� ��������� �� 100 ��

				#ifdef DebugLORAToUART 
					 rprintfInit (Terminal); // ������������� printf (). ����� � ��������
					 printf("\n\r\n\r CRC ER! Noise received " );
				#endif
					 countERR ++; // ���������� ������ �������
				 }
				 else   // ����� ������ � ��� ������ CRC
				 {
					Radio -> GetRxPacket( LORAbufferRX, (u16*)&BufferSize ); // �������� ������ � ����� ������ LORAbufferRX

					OnLED0();				// ������ ���������� �����
					counterLed0 = 500; 		// �������� ����� ��������� �� 500 ��
					counterRXDisplay = 500;	// ������ RX 500 ��

					flagRX_OK = 1; // ���� ���������� ����� ������ � ������

					FREQ_ERROR =( (LoraReadReg(0x28) << 28) + (LoraReadReg(0x29) << 20) + (LoraReadReg(0x2A) << 12) ) ;

					// FreqError =  ((FREQ_ERROR * (2^24) ) / 32e6 ) * (BW / 500) ; // ������ ������ �� �������
					FreqError = (FREQ_ERROR/4096) / 3.815; // �������� � ��


				#ifdef DebugLORAToUART
					 rprintfInit (Terminal); // ������������� printf (). ����� � ��������
					 printf("\n\r\n\r RSSI:%d dBm ", (s32)(SX1276LoRaGetPacketRssi()) );
					// printf("\n\r Preamble:  %d\n\r",  SX1276LoRaGetPreambleLength () );
					// printf(" Gain: %d ", SX1276GetPacketRxGain() );
					 printf(" SNR:%d dB ", SX1276GetPacketSnr() );

					// printf(" Preamble: %d ",  SX1276LoRaGetPreambleLength () );
					// printf(" NbTrigPeaks: %d \n\r",  SX1276LoRaGetNbTrigPeaks () );

					 printf(" PER:%02d.%02d%% ", PER1, PER2 );
					 printf(" Offset:%d Hz", (s32)FreqError );
					// printf(" err=%d summ=%d ", countERR, countOK );

				/*            if (CRCerror == 1) // ������ CRC
					 {
						 CRCerror = 0;
						 printf(" CRC ER! " );

						 countERR ++;

				//               if (trigger_PER == 0)
				//                   {
				//                   countERR = 0;
				//                   countOK = 0;
				//                   PER = 0;
				//                   trigger_PER = 1;
				//                   }
					 }*/

					printf("\n\r RX: [\t");
					for (i = 0; i< 240; i++)
					{
						 printf("%02x ", LORAbufferRX[i] ) ;
						if ( i == 23 || i == 47 || i == 71 || i == 95 || i == 119 || i == 143 || i == 71 || i == 167 || i == 191 || i == 215 ) // 10 ����� �� 24 �����
						{
							printf("\n\r\t");
						}
					}
					printf("  ]\r\n");

					printf(" Number of payload bytes \t\t0x13: 0x%02X  \n\r", LoraReadReg(0x13) ); // Number of payload bytes of latest packet received
					printf(" Number of valid headers received \t0x14-15: %d  \n\r", (LoraReadReg(0x14) << 8) + LoraReadReg(0x15) ); // Number of valid headers received
					printf(" Number of valid packets received \t0x16-17: %d  \n\r", (LoraReadReg(0x16) << 8) + LoraReadReg(0x17) ); // Number of valid packets received

				#endif


				 }


		//	 PER = (countERR * 10000 / countOK) ;
		//	 PER1 = PER / 100;
		//	 PER2 = PER % 100;




            break;
			 }

}

// ������ � FIFO ����� size ����
void LoraWriteFIFO (u8 *buffer, u8 size)
{
	LoraWriteBuffer (0, buffer, size);
}

// ������ �� FIFO ������ size ����
void LoraReadFIFO (u8 *buffer, u8 size)
{
	LoraReadBuffer (0, buffer, size);
}

// BURST ������ size ��������� ����� ������� � ������ addr
void LoraWriteBuffer (u8 Addr, u8 *buffer, u8 size)
{
	u8 i;
	OnNssLORA();

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ���� ���� ����������� ����� ��������
	SPI_I2S_SendData(SPI1, Addr | 0x80);
	for( i = 0; i < size; i++ )
		{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ���� ���� ����������� ����� ��������
		SPI_I2S_SendData(SPI1, buffer [i]);
		}
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);

	OffNssLORA();
}

// BURST ������ size ��������� ����� ������� � ������ addr
void LoraReadBuffer (u8 Addr, u8 *buffer, u8 size)
{
	u8 i;
	OnNssLORA();

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 		// ���� ���� ����������� ����� ��������
	SPI_I2S_SendData(SPI1, Addr & 0x7F);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 		// ���� ���� ���� ����������� �������
	SPI_I2S_ReceiveData (SPI1); 											// ������ ���� ��� ������ ����� ������

	for( i = 0; i < size; i++ )
		{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 	// ���� ���� ����������� ����� ��������
		SPI_I2S_SendData(SPI1, 0x00);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 	// ���� ���� ���� ����������� �������
		buffer [i] = SPI_I2S_ReceiveData (SPI1);
		}

	OffNssLORA();
}

// ������ ��������
void LoraWriteReg	(u8 Addr, u8 Reg)
{
	OnNssLORA();

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ���� ���� ����������� ����� ��������
	SPI_I2S_SendData(SPI1, Addr | 0x80);
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY) == SET); 	// todo ���������� ���������!
	SPI_I2S_SendData(SPI1, Reg);
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY) == SET);

	OffNssLORA();
}
// ������ ��������
u8   LoraReadReg	(u8 Addr)
{
	OnNssLORA();

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ���� ���� ����������� ����� ��������
	SPI_I2S_SendData(SPI1, Addr);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);  	// todo ���������� ���������!
	SPI_I2S_ReceiveData (SPI1);										// ��� ������� ����� SPI_I2S_FLAG_RXNE
	SPI_I2S_SendData(SPI1, 0x00);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); // ���� ���� ���� ����������� �������

	OffNssLORA();
	return (SPI_I2S_ReceiveData (SPI1));
}

/**
  * @brief  Gets numeric values from the hyperterminal.
  */
u8 	 USART_Scanf(u32 value)
{
    u32 index = 0;
    u32 tmp[2] = {0, 0};

    while (index < 2)  // ���� ����� ���� ���� (����) �� ��������� � USART1 .
    {
        /* Loop until RXNE = 1 */
        while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
        {}
        tmp[index++] = (USART_ReceiveData(USART1));
        if ((tmp[index - 1] < 0x30) || (tmp[index - 1] > 0x39))
        {
            printf("\n\rPlease enter valid number between 0 and 9");
            index--;
        }
    }
    /* Calculate the Corresponding value */
    index = (tmp[1] - 0x30) + ((tmp[0] - 0x30) * 10);
    /* Checks */
    if (index > value) // ����� ������ ���� �� ����� value
    {
        printf("\n\rPlease enter valid number between 0 and %d", value);
        return 0xFF;
    }
    return index;
}

///////////////////////////////
void USART3_SendByte 		(char byte)
{
	USART_SendByte(USART3, byte);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void USART2_SendByte 		(char byte)
{
	USART_SendByte(USART2, byte);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void USART1_SendByte 		(char byte)
{
	USART_SendByte(USART1, byte);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void USART_SendByte (USART_TypeDef* USARTx, u16 Data)
{
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);// ���� ���� ����������� ����� ��������
	USART_SendData ( USARTx, Data); //
}
////////////// ������� ��������� ������ GPS ///////////////////////////////////////////////////////////////////////
void ClearGPSBuffer			(void)
{
	for (i=0; i<GPSBUFLENGTH; i++) 	// ������� ������
		{
		GPSBuf[i] = '\0'; 			// ��������� ������
		}
	GPSBufWrPoint = 0; 				// ����� � 0, ���������� ��������� ������� � ������ ������
}
////////////// ������� ��������� ������ BLE ///////////////////////////////////////////////////////////////////////
void ClearBLEBuffer			(void)
{
	for (i=0; i<BufferUSART2Size; i++) 	// ������� ������
		{
		BufferUSART2RX[i] = '\0'; 			// ��������� ������
		}
	counterUSART2Buffer = 0; 				// ����� � 0, ���������� ��������� ������� � ������ ������
}

/**
  * @brief  Displays the current time.
  * @param  TimeVar: RTC counter value.
  * @retval None
  */
void Time_Display(u32 TimeVar)
{
    u32 THH = 0, TMM = 0, TSS = 0;

    /* Compute  hours */
    THH = TimeVar / 3600 % 24;
    /* Compute minutes */
    TMM = (TimeVar % 3600) / 60;
    /* Compute seconds */
    TSS = (TimeVar % 3600) % 60;

    printf("%02d:%02d:%02d", THH, TMM, TSS);
}
void Time_Display2(u32 TimeVar)
{
    u32 THH = 0, TMM = 0, TSS = 0;

    /* Compute  hours */
    THH = TimeVar / 3600 % 24;
    /* Compute minutes */
    TMM = (TimeVar % 3600) / 60;
    /* Compute seconds */
    //TSS = (TimeVar % 3600) % 60;

    printf("%02d:%02d", THH, TMM );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void OnVcc1 	(void) // �������� �������
{
	  GPIO_InitTypeDef        GPIO_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ; //     Vcc1ON   �����
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_SetBits(GPIOB, GPIO_Pin_13);  // �� ������ 1
}
void OffVcc1 	(void) // ��������� �������
{
	  GPIO_InitTypeDef        GPIO_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ; //     Vcc1ON   ����, ������ ���������
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIOB, GPIO_Pin_13);
}

void NRESET_on	(void) // 1
{ GPIO_SetBits(GPIOB, GPIO_Pin_2); }
void NRESET_off	(void) // 0
{ GPIO_ResetBits(GPIOB, GPIO_Pin_2); }

void OnLED0 	(void) // �������� ��������� LED0  ------ �����
{ GPIO_SetBits(GPIOA, GPIO_Pin_11); } // LED0 ���
void OffLED0 	(void) // �������� ��������� LED0
{ GPIO_ResetBits(GPIOA, GPIO_Pin_11); } // LED0 ���
void CplLED0	(void) // ����������� ���������
{	GPIOA->ODR ^= GPIO_Pin_11; }

void OnLED1 	(void) // �������� ��������� LED1  -------- ������� ---
{ GPIO_SetBits(GPIOA, GPIO_Pin_12); } // LED0 ���
void OffLED1 	(void) // �������� ��������� LED1
{ GPIO_ResetBits(GPIOA, GPIO_Pin_12); } // LED1 ���
void CplLED1	(void) // ����������� ���������
{	GPIOA->ODR ^= GPIO_Pin_12; }

void OnTest 	(void) // Test = 1	�� �� ������� ���������
{ GPIO_SetBits(GPIOA, GPIO_Pin_12); } //
void OffTest 	(void) // Test = 0
{ GPIO_ResetBits(GPIOA, GPIO_Pin_12); } //
void CplTest	(void)	// ����������� Test
{ GPIOA->ODR ^= GPIO_Pin_12; }

void OnResetLORA (void) //  = 0
{ GPIO_ResetBits(GPIOB, GPIO_Pin_4); } //
void OffResetLORA(void) //  = 1
{ GPIO_SetBits(GPIOB, GPIO_Pin_4); } //

void OnNssLORA 	(void) //  = 0
{ GPIO_ResetBits(GPIOA, GPIO_Pin_4); } //
void OffNssLORA	(void) //  = 1
{ GPIO_SetBits(GPIOA, GPIO_Pin_4); } //

void OnResetBLE (void) //  = 1
{ GPIO_ResetBits(GPIOB, GPIO_Pin_4); } //
void OffResetBLE(void) //  = 0
{ GPIO_SetBits(GPIOB, GPIO_Pin_4); } //

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void OnDivGND1 	(void) // ���������� �������� ���������� � �����
{
	  GPIO_InitTypeDef        GPIO_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ; 	//   �����
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIOB, GPIO_Pin_15);  			// �� ������ 0
}
void OffDivGND1	(void) // ��������� ��������
{
	  GPIO_InitTypeDef        GPIO_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ; 	 //  ����, ������ ���������
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_SetBits(GPIOB, GPIO_Pin_15);
}
void OnDivGND2 	(void) // ���������� �������� ���������� � �����
{
	  GPIO_InitTypeDef        GPIO_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ; 	//   �����
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIOB, GPIO_Pin_14);  			// �� ������ 0
}
void OffDivGND2	(void) // ��������� ��������
{
	  GPIO_InitTypeDef        GPIO_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ; 	 //    ����, ������ ���������
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_SetBits(GPIOB, GPIO_Pin_14);
}


// ����������� �������� � �� �� SysTick
void DelayMS				( u16 delay)
{
    timer1ms = 0;
    while (timer1ms < delay) {} // ���� delay ��
}

// ��� ��������� Lora
u32  GET_TICK_COUNT ( void )
{
	return ( timer1ms );
}

/* --------------- �������� ������ ����������� ---------------------------
 *
 */
void PortIO_Configuration 	(void) // ��������� ������
{
	GPIO_InitTypeDef        GPIO_InitStructure;

  /* Disable the Serial Wire Jtag Debug Port SWJ-DP */
 // GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);  // �� ��������� JTAG
 //  PA.13 (JTMS/SWDAT), PA.14 (JTCK/SWCLK) and PA.15 (JTDI)

//  �������� test pin �� PA12 �� USB ���������� todo
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ; 			// �����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA, GPIO_Pin_12);
//  �������� ����� ��������� �� PA11 �� USB ����������  todo
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 			// �����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA, GPIO_Pin_11);

// ��������� ������. ���� ������� USB_ON
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 			// ���� ������� USB_ON
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		// todo ��������, ������������ � JTAG
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
// ��������� ������. ���� ������� STAT �� ���������� ������ ������������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ; 			// ���� ������� STAT
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		// ������ ������� - ���� �����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

// ��������� ������. ���� ������� 1PPS  PB12
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ; 			// ���� ������� Timemark
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

// ��������� ������. ���� ������ SOS  PA15
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ; 			// ������ SOS ����, �������� �����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

// ��������� ������.������ ������� PB8
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ; 			// ������ ����/�����   ����, �������� �����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		// ���������� ������� ���������� ���������� ������, ������������ ������� - ��������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

// ����� ��� ��� ��������� ���������� ������������ � ���������  PA0 � PA1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;   // ���������� ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

// ��������� ������. ���������� ���������� ���������� ��� ���
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15 ; // ����, ������ ���������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 	// ���������� �������� ������� ����������, ��������� - ����� � 0
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

// ��������� ������. ���������� ��������, ���������-���������� ��������������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ; 				// Vcc1ON   ����, ������ ���������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 	// ���������� ������� ������� ����������, ��������� - ����� � 1
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_13);

// ------------------ HM10 BLE ������ �� CC2540 --- ��������� � USART2 -------------------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ; 				// BRESET   �����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_4); 						// �����


//------------------- SIM33ELA----------------- ��������� � USART3 ------------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 				// NRESET   �����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_2); 						// �����

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ; 				// TIMEMARK ����. ���������� ����������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_12);

//	------------- LORA ������ ----------------------------------
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ; 				// RESET
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_0); 						// RESET � 0 - ������ � ������

// -------------��������� SPI1 --------------------------------
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5  | GPIO_Pin_7; // SCK � MOSI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//GPIO_SetBits(GPIOB, GPIO_Pin_5); // �� �������� � alternate function push-pull
	//GPIO_SetBits(GPIOB, GPIO_Pin_7);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 				// MISO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 			// ���� � ��������� �����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ; 				// NSS, ����������� ������ CS ��� SIM33ELA
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ; 				// DIO0 ���� � ��������� ����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; 				// DIO5 ���� � ��������� ����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

// ----------------- USART1 ----------------------------------------------------
	/* Configure USART1 Rx (PA10) as input floating                         */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* Configure USART1 Tx (PA9) as alternate function push-pull            */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

// ----------------- USART2 ----------------------------------------------------
	/* Configure USART2 Rx (PA3) as input floating                         */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* Configure USART2 Tx (PA2) as alternate function push-pull            */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

// ----------------- USART3 ----------------------------------------------------
	/* Configure USART3 Rx (PB11) as input floating                         */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* Configure USART3 Tx (PB10) as alternate function push-pull            */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
*******************************************************************************/
void RCC_Configuration 		(void)
{
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);

    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }

  RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3
		  | RCC_APB1Periph_PWR | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3  | RCC_APB1Periph_TIM4, ENABLE);

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
		  | RCC_APB2Periph_GPIOD |  RCC_APB2Periph_USART1  |  RCC_APB2Periph_AFIO | RCC_APB2Periph_SPI1 | RCC_APB2Periph_TIM1 , ENABLE); //

  /* Enable the DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

}

void DMA_Configuration		(void)
{
// ������������� DMA ��� i2c  OLED �������
	DMA_InitTypeDef  I2CDMA_InitStructure;

	DMA_DeInit(DMA1_Channel6);  // ����� DMA1_Channel6 ?
	I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)0x40005410;  // ����� i2c1 �����������
	I2CDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;   /* This parameter will be configured durig communication */
	I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    /* This parameter will be configured durig communication */
	I2CDMA_InitStructure.DMA_BufferSize = 0xFFFF;            /* This parameter will be configured durig communication */
	I2CDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	I2CDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	I2CDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	I2CDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	I2CDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	I2CDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	I2CDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel6, &I2CDMA_InitStructure);
}

// ��������� ���1
void ADC1_Configuration		(void)
{
	ADC_InitTypeDef 		ADC_InitStructure;

//clock for ADC (max 14MHz --> 72/6=12MHz)
	RCC_ADCCLKConfig (RCC_PCLK2_Div6);
// enable ADC system clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

// define ADC config
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	// we work in continuous sampling mode
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;

	ADC_TempSensorVrefintCmd(ENABLE); // �������� ���������� ������ ����������� � �����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 ); // define regular conversion config
	ADC_Init ( ADC1, &ADC_InitStructure);	//set config of ADC1

// enable ADC
	ADC_Cmd (ADC1,ENABLE);	//enable ADC1

//	ADC calibration (optional, but recommended at power on)
	ADC_ResetCalibration(ADC1);	// Reset previous calibration
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);	// Start new calibration (ADC must be off at that time)
	while(ADC_GetCalibrationStatus(ADC1));

// start conversion
	ADC_Cmd (ADC1,ENABLE);	//enable ADC1
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	// start conversion (will be endless as we are in continuous mode)

}

// ��������� SPI1
void SPI1_Configuration		(void)
{
	SPI_InitTypeDef spi1;

	SPI_I2S_DeInit(SPI1);

	SPI_StructInit(&spi1);

	spi1.SPI_Mode = SPI_Mode_Master;
	spi1.SPI_DataSize = SPI_DataSize_8b;
	spi1.SPI_NSS = SPI_NSS_Soft;
	spi1.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_Init(SPI1,&spi1);
	SPI_Cmd(SPI1,ENABLE);
}

// ��������� USART1
void USART1_Configuration(void)
{
      USART_InitTypeDef USART_InitStructure;

      /* USART1 configured as follow:
            - BaudRate = BaudRateUSART1 baud
            - Word Length = 8 Bits
            - One Stop Bit
            - No parity
            - Hardware flow control disabled (RTS and CTS signals)
            - Receive and transmit enabled
            - USART Clock disabled
            - USART CPOL: Clock is active low
            - USART CPHA: Data is captured on the middle
            - USART LastBit: The clock pulse of the last data bit is not output to
                             the SCLK pin
      */
      USART_InitStructure.USART_BaudRate            = BaudRateUSART1;
      USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits            = USART_StopBits_1;
      USART_InitStructure.USART_Parity              = USART_Parity_No ;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
      USART_Init(USART1, &USART_InitStructure);

      USART_Cmd(USART1, ENABLE);
}

// ��������� USART2 BLE
void USART2_Configuration(void)
{
      USART_InitTypeDef USART_InitStructure;

      /* USART2 configured as follow:
            - BaudRate = BaudRateUSART2 baud
            - Word Length = 8 Bits
            - One Stop Bit
            - No parity
            - Hardware flow control disabled (RTS and CTS signals)
            - Receive and transmit enabled
            - USART Clock disabled
            - USART CPOL: Clock is active low
            - USART CPHA: Data is captured on the middle
            - USART LastBit: The clock pulse of the last data bit is not output to
                             the SCLK pin
      */
      USART_InitStructure.USART_BaudRate            = BaudRateUSART2;
      USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits            = USART_StopBits_1;
      USART_InitStructure.USART_Parity              = USART_Parity_No ;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
      USART_Init(USART2, &USART_InitStructure);

      USART_Cmd(USART2, ENABLE);
}

// ��������� USART2 BLE 115200 ���
void USART2_Configuration115200(void)
{
      USART_InitTypeDef USART_InitStructure;

      /* USART3 configured as follow:
            - BaudRate = BaudRateUSART3 baud
            - Word Length = 8 Bits
            - One Stop Bit
            - No parity
            - Hardware flow control disabled (RTS and CTS signals)
            - Receive and transmit enabled
            - USART Clock disabled
            - USART CPOL: Clock is active low
            - USART CPHA: Data is captured on the middle
            - USART LastBit: The clock pulse of the last data bit is not output to
                             the SCLK pin
      */
      USART_InitStructure.USART_BaudRate            = 115200;
      USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits            = USART_StopBits_1;
      USART_InitStructure.USART_Parity              = USART_Parity_No ;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
      USART_Init(USART2, &USART_InitStructure);

      USART_Cmd(USART2, ENABLE);
}

// ��������� USART3 GPS 9600 ���
void USART3_Configuration(void)
{
      USART_InitTypeDef USART_InitStructure;

      /* USART3 configured as follow:
            - BaudRate = BaudRateUSART3 baud
            - Word Length = 8 Bits
            - One Stop Bit
            - No parity
            - Hardware flow control disabled (RTS and CTS signals)
            - Receive and transmit enabled
            - USART Clock disabled
            - USART CPOL: Clock is active low
            - USART CPHA: Data is captured on the middle
            - USART LastBit: The clock pulse of the last data bit is not output to
                             the SCLK pin
      */
      USART_InitStructure.USART_BaudRate            = BaudRateUSART3;
      USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits            = USART_StopBits_1;
      USART_InitStructure.USART_Parity              = USART_Parity_No ;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
      USART_Init(USART3, &USART_InitStructure);

      USART_Cmd(USART3, ENABLE);
}

// ��������� USART3 GPS 115200 ���
void USART3_Configuration115200(void)
{
      USART_InitTypeDef USART_InitStructure;

      /* USART3 configured as follow:
            - BaudRate = BaudRateUSART3 baud
            - Word Length = 8 Bits
            - One Stop Bit
            - No parity
            - Hardware flow control disabled (RTS and CTS signals)
            - Receive and transmit enabled
            - USART Clock disabled
            - USART CPOL: Clock is active low
            - USART CPHA: Data is captured on the middle
            - USART LastBit: The clock pulse of the last data bit is not output to
                             the SCLK pin
      */
      USART_InitStructure.USART_BaudRate            = 115200;
      USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits            = USART_StopBits_1;
      USART_InitStructure.USART_Parity              = USART_Parity_No ;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
      USART_Init(USART3, &USART_InitStructure);

      USART_Cmd(USART3, ENABLE);
}

// ��������� ���������� � ���������� ������� �������� ����������
void NVIC_Configuration(void)
{
	 NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
    /* Configure one bit for preemption priority */
   // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_SetPriority(SysTick_IRQn, 0x03); 		// ��������� ���������� �� ���������� �������

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USART2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USART3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	/* Enable the EXTI Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn ; 				// ���������� �� �������� ������� 1PPS �� PB12 todo
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00 ;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE ;
	NVIC_Init(&NVIC_InitStructure);

	//enable tim1 irq
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//enable tim2 irq
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//enable tim3 irq
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//enable tim4 irq
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//enable I2C1_EV_IRQn irq
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//enable I2C1_ER_IRQn irq
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

// ��������� �������1. ������ ��������� ������, ���������� 1PPS ��� ��� ����������
void TIM1_Configuration		(void) //
{
	TIM_TimeBaseInitTypeDef base_timer;

	TIM_TimeBaseStructInit(&base_timer);
/* �������� ����������� ��� TIM_Prescaler + 1, ������� �������� 1 */
	base_timer.TIM_Prescaler = 12-1; 	// ���� ABP1 36 Mhz, �� APB1TIM = 72. Prescaler ����� �� 12 , �������� ������� ������� 6 ���
	base_timer.TIM_Period = 0xFFFF;		// ARR - Auto Reload Register. �� ������ �������� ������� ����� �� ������������			//
	base_timer.TIM_CounterMode = TIM_CounterMode_Up; 	// ������� �����
	TIM_TimeBaseInit(TIM1, &base_timer);
	//TIM4->CR1 |= TIM_CR1_OPM; 						// ����� ������ ��������

/* ������� ���  ���������� */
	//TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update); 	// ����� �������� TIM_FLAG_Update
  // ��������� ���������� �� ���������� (� ������ ������ - �� ������������) �������� ������� TIM1.

	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  // ��������� ��������� ���������� �� ������������ �������� ������� TIM1.

	//TIM_Cmd(TIM1, ENABLE);  			// ������ �������
	NVIC_EnableIRQ(TIM1_UP_IRQn);	 	// ���������� ��-�� ������������ �������

}

// ��������� �������2 Basic ������ ����������� �� ������ ����� �� USART2 �� BLE ������
void TIM2_Configuration		(void)
{
	TIM_TimeBaseInitTypeDef base_timer;

	TIM_TimeBaseStructInit(&base_timer);
/* �������� ����������� ��� TIM_Prescaler + 1, ������� �������� 1 */
	base_timer.TIM_Prescaler = 36000 - 1; 			// ���� ABP1 36 Mhz, �� APB1TIM = 72
	base_timer.TIM_Period = 30*2-1; 				// �������� � ��  (ms*2-1)
	base_timer.TIM_CounterMode = TIM_CounterMode_Up;
	base_timer.TIM_RepetitionCounter = 0;
	base_timer.TIM_ClockDivision = TIM_CKD_DIV1; 	// �������� ������� �������
	TIM_TimeBaseInit(TIM2, &base_timer);
	TIM2->CR1 |= TIM_CR1_OPM; 		// ����� ������ ��������

/* ������� ���  ���������� */
    TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update); // ����� �������� TIM_FLAG_Update

  // ��������� ���������� �� ���������� (� ������ ������ - �� ������������) �������� ������� TIM2.

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  // ��������� ��������� ���������� �� ������������ �������� ������� TIM2.

	NVIC_EnableIRQ(TIM2_IRQn);	// TIM2_IRQn

}

// ��������� �������3 Basic  ������ ����������� �� ������ ����� �� USART3 �� GPS ������ SIM33ELA,
void TIM3_Configuration		(void) //  ���� ����� ����� ��������� 10 ��, ���������� ����������
{
	TIM_TimeBaseInitTypeDef base_timer;

	TIM_TimeBaseStructInit(&base_timer);
/* �������� ����������� ��� TIM_Prescaler + 1, ������� �������� 1 */
	base_timer.TIM_Prescaler = 36000 - 1; 			// ���� ABP1 36 Mhz, �� APB1TIM = 72. ��� ���� 1 kHz .
	base_timer.TIM_Period = 10*2-1;					// �������� � ��  (ms*2-1)
	base_timer.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &base_timer);
	TIM3->CR1 |= TIM_CR1_OPM; 						// ����� ������ ��������

/* ������� ���  ���������� */
	TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update); 	// ����� �������� TIM_FLAG_Update
  // ��������� ���������� �� ���������� (� ������ ������ - �� ������������) �������� ������� TIM3.

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  // ��������� ��������� ���������� �� ������������ �������� ������� TIM3.

	NVIC_EnableIRQ(TIM3_IRQn);	//TIM3_IRQn
}

// ��������� �������4 Basic
void TIM4_Configuration		(void) //
{
	TIM_TimeBaseInitTypeDef base_timer;

	TIM_TimeBaseStructInit(&base_timer);
/* �������� ����������� ��� TIM_Prescaler + 1, ������� �������� 1 */
	base_timer.TIM_Prescaler = 12-1; 	// ���� ABP1 36 Mhz, �� APB1TIM = 72. Prescaler ����� �� 12 , �������� ������� ������� 6 ���
	base_timer.TIM_Period = 0xFFFF;			// ARR - Auto Reload Register. �� ������ �������� ������� ����� �� ������������			//
	base_timer.TIM_CounterMode = TIM_CounterMode_Up; 	// ������� �����
	TIM_TimeBaseInit(TIM4, &base_timer);
	//TIM4->CR1 |= TIM_CR1_OPM; 						// ����� ������ ��������

/* ������� ���  ���������� */
	//TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update); 	// ����� �������� TIM_FLAG_Update
  // ��������� ���������� �� ���������� (� ������ ������ - �� ������������) �������� ������� TIM4.

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  // ��������� ��������� ���������� �� ������������ �������� ������� TIM4.

	TIM_Cmd(TIM4, ENABLE);  		// ������ �������
	NVIC_EnableIRQ(TIM4_IRQn);	 	// TIM4_IRQn


}

void EXTI_Configuration		(void) // ��������� ���������� �� 1PPS �������
{
	EXTI_InitTypeDef EXTI_InitStructure ;

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12); // �������� ���������� - PB12
	EXTI_InitStructure.EXTI_Line = EXTI_Line12 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt ;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising ;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE ;
	EXTI_Init (&EXTI_InitStructure) ;
}

/******************************************************************************/
/*  -------------- STM32F10x Peripherals Interrupt Handlers ---------------   */
/******************************************************************************/

void SysTick_Handler(void) 		// ���������� �� ���������� �������, ���������� ������ ������������
{
	u8 nn;
	timer1ms ++; // ������� ��
//	if (timer1ms == 1000000000) // ����� �������� ������ �������� �����������
//		timer1ms = 0;
	if ((timer1ms % 1000) == 0) // ���� ������� ��� ������� �� 1000
		timer1s ++; // ������� c�����

	if (counterRXDisplay > 0) // ������� ��������� RX �� �������
	{
		counterRXDisplay -- ;
	}

	if (counterTXDisplay > 0) // ������� ��������� TX �� �������
	{
		counterTXDisplay -- ;
	}

	if (counterMaxRssiReset > 0) 	// ������� ������ ������������� ������ RSSI
	{
		counterMaxRssiReset -- ;
	}

	for (nn = 0; nn < 10; nn++) 	// �������� ������ ������ RSSI
		if (counterRssiReset[nn] > 0)
			counterRssiReset[nn] -- ;

	if (counterLed0 > 0) 	// ������� ���������� LED0 - ������
	{
		counterLed0 -- ;
		if (counterLed0 == 0)
			OffLED0();
	}

	if (counterLed1 > 0) 	// ������� ���������� LED1 - ��������
	{
		counterLed1 -- ;
		if (counterLed1 == 0)
			OffLED1();
	}

	if (counterLcdOff > 0)
	{
		counterLcdOff -- ;
	}

}

/**
  * @brief  This function handles USART1 global interrupt request.
  * ��������� ����� �������� NUM+1
  * � ����� ������ � ������ ������� \0
  * ��� ������� ������ � ������ ����������� ���� triggerUSART1Interupt
  */
void USART1_IRQHandler(void)  	// �������� �� ��
{
	u8 TempDataUSART1;

    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)	//  �������� ����� ���������� RXNE - �� ������ �����
	{
		TempDataUSART1 = USART_ReceiveData(USART1);		// ������ ���� �� ������
		if(counterUSART1Buffer == BufferUSART1Size)		// ����� � ����� �� �����
		{
			BufferUSART1RX[counterUSART1Buffer] = TempDataUSART1;
			counterUSART1Buffer = 0;
		}
		else
		{
			BufferUSART1RX[counterUSART1Buffer++] = TempDataUSART1;
		}
		BufferUSART1RX[counterUSART1Buffer] = '\0';		// ����� ������ � ����� ����� NULL \0

		triggerUSART1Interrupt = 1; 					// ���������� ����
	}
}

/**
  * @brief  This function handles USART2 global interrupt request.
  * @param  None
  * @retval None
  * ��������� ����� �������� NUM+1
  * � ����� ������ � ������ ������� \0
  * ��� ������� ������ � ������ ����������� ���� triggerUSART2Interupt
  */
void USART2_IRQHandler(void) 	// ���������� �� ������ ����� �� BLE ������
{
	u8 TempDataUSART2;

    if ((USART2->SR & USART_FLAG_RXNE) != (u16)RESET)	//  �������� ����� ���������� RXNE - �� ������ �����
	{
    	TIM_Cmd(TIM2, ENABLE);  // ������ �������
    	TIM2 -> CNT = 0;		// ����� �������� �������

		TempDataUSART2 = USART_ReceiveData(USART2);		// ������ ���� �� ������
		if(counterUSART2Buffer == BufferUSART2Size)		// ����� � ����� �� �����
		{
			BufferUSART2RX[counterUSART2Buffer] = TempDataUSART2;
			counterUSART2Buffer = 0;
		}
		else
		{
			BufferUSART2RX[counterUSART2Buffer++] = TempDataUSART2;
		}
		BufferUSART2RX[counterUSART2Buffer] = '\0';		// ����� ������ � ����� ����� NULL \0

		triggerUSART2Interrupt = 1; 					// ���������� ����
	}
}

/**
  * @brief  This function handles USART3 global interrupt request.
  * @param  None
  * @retval None
  * ��������� ����� �������� NUM+1
  * � ����� ������ � ������ ������� \0
  * ��� ������� ������ � ������ ����������� ���� triggerUSART3Interupt
  */
void USART3_IRQHandler(void) 	// ���������� �� ������ ����� �� GPS ������
{
//	u8 TempDataUSART2;
//    if ((USART2->SR & USART_FLAG_RXNE) != (u16)RESET)	//  �������� ����� ���������� RXNE - �� ������ �����
//	{
//		TempDataUSART2 = USART_ReceiveData(USART2);		// ������ ���� �� ������
//		if(counterUSART2Buffer == BufferUSART2Size)		// ����� � ����� �� �����
//		{
//			BufferUSART2RX[counterUSART2Buffer] = TempDataUSART2;
//			counterUSART2Buffer = 0;
//		}
//		else
//		{
//			BufferUSART2RX[counterUSART2Buffer++] = TempDataUSART2;
//		}
//		BufferUSART2RX[counterUSART2Buffer] = '\0';		// ����� ������ � ����� ����� NULL \0
//
//		triggerUSART2Interrupt = 1; 					// ���������� ����
//	}

//--------------------------------- NMEA RECEIVER ------------------------------------
// ��������� ������ � 0 ������ ����������� ��������
  volatile char UsartTemp = 0;
 // volatile char m;

  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // ��������� ���� ����������
  {
  	TIM_Cmd(TIM3, ENABLE);  // ������ �������
  	TIM3 -> CNT = 0;		// ����� �������� �������.. ���� ����� , ����� �������

	UsartTemp = USART_ReceiveData(USART3);
	GPSBuf[GPSBufWrPoint] = UsartTemp; // todo ��������� ���������� ������ � ������, ����� �������� CRC

	if ((GPSBufWrPoint > 1) && (GPSBufWrPoint < GPSBUFLENGTH))  // �������� ���������� ������ ������, ����� ��� ���������� �������������� �� ��������� �� �������
	{
	  if ((GPSBuf[GPSBufWrPoint-1] == ',') && (GPSBuf[GPSBufWrPoint] == ','))
	  { // �������� ������������������ ���� ���� ",," ������������ � ",0,", ��� ����� ��������� �������������� �������� � ������ 0
		GPSBuf[GPSBufWrPoint]   = '0';
		GPSBuf[GPSBufWrPoint+1] = ',';
		GPSBufWrPoint++;
	  }
	  if ((GPSBuf[GPSBufWrPoint-1] == ',') && (GPSBuf[GPSBufWrPoint] == '*'))
	  { // �������� ������������������ ���� ���� ",*" ������������ � ",0*", ��� ����� ��������� �������������� �������� � ������ 0
		GPSBuf[GPSBufWrPoint]   = '0';
		GPSBuf[GPSBufWrPoint+1] = '*';
		GPSBufWrPoint++;
	  }
	}

	if (GPSBufWrPoint < GPSBUFLENGTH)
	  GPSBufWrPoint++;  // ���������� ��������� �����
//	else
//	{ // ���� ����� ����������, �������� ������ � ������������ NMEA ���������
//printf ("\n\r GPSBufWrPoint1: %d \n\r\n\r", GPSBufWrPoint);
//	  GPSBufWrPoint = 0;
//	  if (NMEA_Parser_GGA() == 1) // GGA - �������� ���������, ������ ����������
//	  {
//printf ("\n\r NMEA_Parser_GGA = 1:\n\r");
//		NMEA_Parser_ZDA();      // ZDA ������ ����� � ����
//
//		GSVStartPos = 0;
//		SatNum = 0;
//		if (NMEA_Parser_GSV() == 1) // GSV ������ ������ ���������, ������� ����������. ��� ���������� ����� ������������ ��� ������������ �����������
//		{
//		for (m = 0; m <=6; m++) // GSV ������� �������� ��������, ������ ��� � ������ ������� ������ ���������� �������� � 4-� ���������, � �� �������� 25
//		  NMEA_Parser_GSV();
//		}
//	  }
//	}
  }



}

// ���������� ���������� �� Timer 1 - ������������
void TIM1_UP_IRQHandler(void)
{
 // ��������� �� ���������� �� ������������ �������� ������� TIM1.
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
  {
	/* ������� ��� ��������������� ���������� */
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // ����� �������� TIM_FLAG_Update
	counter_TIM1_OVR ++ ;  						// ������� ������������ �������

	if ( (counter_TIM1_OVR == 91) && (flag_1PPS_AVG_OK == 1) ) 				//
	  {

	  TIM1->ARR = averageTIM4_ready;			// ��� ��������� ������������ ARR ��������� �� averageTIM4_ready
	  }

	if ( counter_TIM1_OVR == 92 ) 				// ���� �������� ������ ���� ��� ������� 1PPS, ��� ���������� ������
	  {
//OnLED0();
		counter_TIM1_OVR = 0;
	    TIM1->ARR = 0xFFFF;			// ��� ��������� ������������ ARR ��������� �� 0xFFFF
	    flag_1PPS_pulse = 1;		// ������� 1PPS �� ������� 1
	    SecRTC ++;					// ��������� �������
//OffLED0();
	  }
  }
}

// ���������� ���������� Timer2 - ������ ������� �� BLE ������
void TIM2_IRQHandler()
{
 // ��������� �� ���������� �� ������������ �������� ������� TIM2.
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
	/* ������� ��� ��������������� ���������� */
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // ����� �������� TIM_FLAG_Update
	triggerTIM2Interrupt = 1; 					// ���������� ����, ������� ������� �� BLE ������
	counterUSART2Buffer = 0;					// �������� ������� ���� USART ������
  }
}

// ���������� ���������� Timer3 - ������ ������� �� GPS ������
void TIM3_IRQHandler()
{
// ��������� �� ���������� �� ������������ �������� ������� TIM3.
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    /* ������� ��� ��������������� ���������� */
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // ����� �������� TIM_FLAG_Update
    triggerTIM3Interrupt = 1; 					// ���������� ����, ������� ������� �� GPS ������
  }
}

// ���������� ���������� Timer4
void TIM4_IRQHandler() // ������� ������������� ������ 10,92 ��, �� 1 ������� ��������� �� 91
{
// ��������� �� ���������� �� ������������ �������� ������� TIM4.
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
	  /* ������� ��� ��������������� ���������� */
	  TIM_ClearITPendingBit(TIM4, TIM_IT_Update); // ����� �������� TIM_FLAG_Update

//CplLED0(); // todo

	  counter_TIM4_OVR ++ ;  		// ������� ������������ �������
	  if ( counter_TIM4_OVR >= 92 ) // ����� ��������� ��� ������� ����� ����� ���������� 1PPS
		  {
		  counter_TIM4_OVR = 0;
		  counter_TIM4_fix = 0;
		  TIM4_fix = 0;
		  flag_1PPS_timeout = 1;
		  flag_1PPS_fail = 1;
		  }

  }
}

// ���������� �� ��������� ������ 1PPS
void EXTI15_10_IRQHandler (void) // ���������� ���������� �� ������ 10-15
{
	if (EXTI_GetITStatus(EXTI_Line12))			// �������� ����� ����������
	{
//OnLED0();
		EXTI_ClearITPendingBit (EXTI_Line12); 	// ����� ����� ����������

		TIM4_fix = TIM4->CNT; 						// ��������� �������� �������
		counter_TIM4_fix = counter_TIM4_OVR; 		// ��������� �������� �������� ������ �������

		TIM4->CNT = 0; 				// ����� �������
		counter_TIM4_OVR = 0; 		// ����� ��������

		flag_1PPS_Update = 1; 		// ���������� �������� 1PPS
		flag_1PPS_pulse = 1;		// ������� ������

		// -- ������������� �������1 ----
		if (flag_1PPS_AVG_OK == 1)		// ���� ����� ������ ������� �������� ������� 1PPS ��� �������� ������ ���� ���
		{
			TIM_Cmd(TIM1, DISABLE);  	// ���� ������� todo �������� ����� ��
			TIM1->CNT = 0; 				// ����� �������
			TIM1->ARR = 0xFFFF;			// ��� ��������� ������������ ARR ��������� �� 0xFFFF
			counter_TIM1_OVR = 0;
			TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update); 	// ����� �������� TIM_FLAG_Update
			TIM_Cmd(TIM1, ENABLE);  	// ������ �������
			//
			SecRTC = GPSTimeSec + 1;	// ������� ������� ������� ����� ������� GPS + 1
		}
//OffLED0();
	}
}

void I2C1_EV_IRQHandler(void)
{
	__IO uint32_t SR1Register =0;
	__IO uint32_t SR2Register =0;

OnLED1();

    /* Read the I2C1 SR1 and SR2 status registers */
	// ��������� ��� �������� SR1 � SR2
    SR1Register = I2C1->SR1;
    SR2Register = I2C1->SR2;

    /* If SB = 1, I2C1 master sent a START on the bus: EV5) */
    // ���������� �� ������� START �� �������
    if ((SR1Register &0x0001) == 0x0001)
    {
        /* Send the slave address for transmssion or for reception (according to the configured value
            in the write master write routine */
        I2C1->DR = Address;	// �������� ����� ���������� 		extern uint8_t Address
        SR1Register = 0;	// ���������� ���������� ��������� ������� I2C
        SR2Register = 0;
    }
    /* If I2C1 is Master (MSL flag = 1) */
    // �������� ������
    if ((SR2Register &0x0001) == 0x0001)
    {
        /* If ADDR = 1, EV6 */  // ����� ��� ������, ���������� ������ ���� ������
        if ((SR1Register &0x0002) == 0x0002)
        {
            /* Write the first data in case the Master is Transmitter */

			/* Initialize the Transmit counter */
			Tx_Idx1 = 0;  						//__IO uint8_t
			/* Write the first data in the data register */
			I2C1->DR = Buffer_Tx1[Tx_Idx1++];
			/* Decrement the number of bytes to be written */
			NumbOfBytes1--;						// extern __IO uint32_t NumbOfBytes1;
			/* If no further data to be sent, disable the I2C BUF IT
			in order to not have a TxE  interrupt */
			if (NumbOfBytes1 == 0) // ���� ��� ������ �������� - ��������� ���������� I2C_IT_BUF
			{
				I2C1->CR2 &= (uint16_t)~I2C_IT_BUF;
			}
            SR1Register = 0;
            SR2Register = 0;
        }
        /* Master transmits the remaing data: from data2 until the last one.  */
        /* If TXE is set */ // ��������� ����� �� ������� ��� ��������
        if ((SR1Register &0x0084) == 0x0080)
        {
            /* If there is still data to write */
            if (NumbOfBytes1!=0)
            {
                /* Write the data in DR register */
                I2C1->DR = Buffer_Tx1[Tx_Idx1++]; // extern uint8_t Buffer_Tx1[]
                /* Decrement the number of data to be written */
                NumbOfBytes1--;
                /* If  no data remains to write, disable the BUF IT in order
                to not have again a TxE interrupt. */
                if (NumbOfBytes1 == 0)
                {
                    /* Disable the BUF IT */
                    I2C1->CR2 &= (uint16_t)~I2C_IT_BUF;
                }
            }
            SR1Register = 0;
            SR2Register = 0;
        }
        /* If BTF and TXE are set (EV8_2), program the STOP */
        // ��������� STOP
        if ((SR1Register &0x0084) == 0x0084)
        {
            /* Program the STOP */
            I2C1->CR1 |= I2C_CR1_STOP; //0x0200 ; //
            /* Disable EVT IT In order to not have again a BTF IT */
            I2C1->CR2 &= (uint16_t)~I2C_IT_EVT;
            SR1Register = 0;
            SR2Register = 0;
        }


    }

OffLED1();
}

void I2C1_ER_IRQHandler(void)	// ���������� ���������� �� ������ I2C
{
    __IO uint32_t SR1Register =0;

OnLED0();
//OnLED1();

    /* Read the I2C1 status register */
    SR1Register = I2C1->SR1;
    /* If AF = 1 */
    if ((SR1Register & 0x0400) == 0x0400)
    {
        I2C1->SR1 &= 0xFBFF;
        SR1Register = 0;
    }
    /* If ARLO = 1 */
    if ((SR1Register & 0x0200) == 0x0200)
    {
        I2C1->SR1 &= 0xFBFF;
        SR1Register = 0;
    }
    /* If BERR = 1 */
    if ((SR1Register & 0x0100) == 0x0100)
    {
        I2C1->SR1 &= 0xFEFF;
        SR1Register = 0;
    }

    /* If OVR = 1 */

    if ((SR1Register & 0x0800) == 0x0800)
    {
        I2C1->SR1 &= 0xF7FF;
        SR1Register = 0;
    }
}
