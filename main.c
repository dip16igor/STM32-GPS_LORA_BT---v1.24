/* Прошивка для контроллера "радиопеленгатора" на основе MESH LORA сети
 * Входные данные - NMEA с GPS/GLONASS приемника
 * Связь между точками - MESH сеть с временным разделением
 * Для вывода информации используется смартфон, подключеный через BT HM-10
 * Контроллер - STM32F103C8T6
 * Среда разработки - Coocox v.1.7.8 + GCC 5.3 + StdPeriphLib v.3.5.0
 * Отладка - ST-Link v2 с Алиэкспресса, подключенный по SWD, либо клон CoLinkEx
 * USART1 - связь с терминалом на PC
 * USART2 - связь с BLE HM-10 (клон XM-10, без кварца)
 * USART3 - связь с GPS SIM-33ELA
 * SPI1 - связь с LORA модулем (RFM68)
 * SPI2 -
 * I2C1 - OLED дисплей SSD1306
 * I2C2 -
 * ADC1 ch0 - измерение напряжения аккумулятора
 * ADC1 ch1 - измерение напряжения батареи резервного питания
 *
 *  14.05.2016	v0.1	UART, парсинг NMEA
 *  02.06.2016	v0.2	добавлены таймеры
 *  11.06.2016	v0.3	добавлены SPI, радиомодуль RFM98
 *  18.12.2016  v1.0    переход на синие платы v0.1
 *  15.02.2017  v1.1	перенос проекта в Dropbox, причесывание исходников
 *  20.02.2017  v1.2   	добавлен I2C и OLED дисплей SSD1306
 *  21.02.2017	v1.3	добавлен ADC
 *  23.02.2017  v1.4	добавлены часы реального времени RTC
 *  08.03.2017	v1.5	добавлен парсер GSA сентенции (DOP)
 *  10.03.2017	v1.6 	добавлено внешнее прерывание от 1PPS, синхронизация RTC
 *  21.03.2017  v1.7 	добавлен LORA трансивер
 *  29.03.2017  v1.8 	добавлен BLE модуль XC-10 на чипе CC41-A
 *  28.06.2017	v1.9	исправление багов при настройке GPS и RTC
 *  25.06.2017 	v1.10	отказ от RTC, переход на обычный таймер
 *  01.08.2017  v1.11	добавлен парсинг строк GLONASS спутников 
 *  06.08.2017	v1.12	передача посылок по радиоканалу
 *  30.11.2017	v1.13	прием посылок от других маяков, фикс бага некоторых дисплеев, индикация заряда
 *  03.12.2017	v1.14	передача и прием координат, ретрансляция
 *  09.12.2017	v1.15   корректное измерение PER
 *  11.12.2017 	v1.16	добавлена тригонометрия для вычислений расстояний между маяками
 *  16.12.2017	v1.17	добавлен график напряжения аккумулятора от времени, выявлен баг с джиттером времени работы передатчика
 *  17.12.2017	v1.18	I2C для дисплея через DMA
 *	30.12.2017	v1.19	измерение PER еще корректнее, добавлена осциллограмма напряжения батареи
 *	02.01.2018	v1.20	время в логе батареи изменено с GPS на системное, добавлено выключение дисплея по таймауту
 *  04.05.2018	v1.21   улучшен вывод информации на дисплей, мелкие исправления.
 *  10.06.2018	v1.22	переход на GCC 7.0
 *  01.09.2018	v1.23	Добавлен Release режим отображения экранов. Для входа в него при запуске удерживать кнопку SCREEN
 *  08.09.2018	v1.24	Для контроля прохождения посылок по сети добавлен счетчик пакетов. В конце каждой посылки маяк добавляет 1 байт счетчик 0..255
 *  todo
 *  		добавить измерение температуры датчиком в АЦП STM32
 *  		//отображать напряжение аккумулятора при старте
 *  		добавить адаптивную регулировку мощности передатчика
 *  		пофиксить баг с включением передатчика при LORA_TX_TIME = 0 при первоначальном усреднении 1PPS
 *  		добавить в счетчик секунд полную дату для корректного перехода через 00 часов
 *  		разобраться с нестабильной работой BLE
 *  		// измерить погрешность вычисления координат и расстояний
 *  		//исследовать эффективность спиральных антенн
 *  		и зависимость дальности связи от параметров модуляции  LORA
 *  		иногда неправильно инициализируется дисплей ( чаще у маяка 2. Исправляется добавлением задержки перед инициализацией дисплея, но не всегда)
 */

#define LORA_TX_TIME 	3		// номер секунды для передачи (1-9)  3 - 03 13 23 33 43 53 03 и тд.  <----------------------------------------------------------------------------------------- TX TIME -----------------
#define VersionMajor	1		// версия прошивки v1.24
#define VersionMinor	24
#define RFPowerDef		2		// мощность LORA трансиверов (от 2 до 17 dBm - от 1,6 до 50,1 mW)
#define LcdOffTimeout 	180000	// время до выключения экрана, мс

//  вывод отладочной информации в USART1
//#define DebugGpsToUart1				// вывод отладочной информации от GPS
//#define Debug1PPSMeasurementToUART	// вывод отладочной информации об измерении длительности 1PPS
//#define DebugLORAToUART				// вывод отладочной информации о работе радиомодуля LORA

#include "stm32f10x.h"			// ядро
#include "stm32f10x_flash.h"	// работа с flash памятью программ
#include "stm32f10x_gpio.h"		// порты ввода-вывода
#include "stm32f10x_rcc.h"		// система тактирования
#include "stm32f10x_tim.h"		// таймеры
#include "stm32f10x_usart.h"	// интерфейсы USART
#include "stm32f10x_spi.h"		// SPI
#include "stm32f10x_adc.h"		// ADC
#include "stm32f10x_exti.h"		// Внешние прерывания EXTI
#include "stm32f10x_dma.h"		// DMA ( используется для i2c дисплея )
#include "misc.h"				// прерывания
#include "stdio.h"				// printf  добавлено rprintfInit()
#include "string.h"				// работа со строками
#include <stdlib.h>				// типы данных, константы и тд.
#include <math.h>				// математика
#include <stdbool.h>			// определение типа bool

#include "sx1276.h"				// драйвер LORA модулей
#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"
#include "platform.h"
#include "radio.h"

#include "ssd1306.h" 			// OLED SSD1306

// прототипы функций
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
void Time_Display2			(u32 TimeVar); 	// без секунд

void DelayMS				(u16 delay);	// блокирующая задержка в мс на SysTick

void OnVcc1 				(void); 	// Включить питание
void OffVcc1 				(void); 	// Выключить питание

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

u16  GetSamplesADC1			(void); // выборка АЦП todo доработать
void OnDivGND1 				(void);
void OffDivGND1				(void);
void OnDivGND2 				(void);
void OffDivGND2				(void);

void Display1				(void); // вывод информации на экран
void Display2				(void); // напряжения аккумулятора, батарейки
void Display3				(void); // время RTC
void Display4				(void); // информация о спутниках
void Display5				(void); // информация о спутниках в графическом виде
void Display6				(void); // информация о спутниках, используемых в решении и параметрах DOP
void Display7				(void); // координаты и высота в u32
void Display8				(void); // тест измерения периода 1PPS
void Display9				(void); // приемник LORA
void Display10				(void); // принятые координаты
void Display11				(void); // времена посылок
void Display12				(void); // параметры принятых посылок
void Display13				(void); // PER от каждого маяка
void Display14				(void); // вычисление расстояния между маяками
void Display15				(void); // уровни сигнала от маяков в графическом виде

void Display1Release		(void); // основной экран
void Display2Release		(void); // расстояния между маяками
void Display3Release		(void); // информация GPS

void OLED_Putc1				(char c);
void OLED_Putc2				(char c);
void OLED_Putc3				(char c);

void InitMeasVoltage		(void); // первоначальное заполнение массива усреднения измеренными значениями
void MeasVoltage			(void); // измерения напряжений аккумулятора и батареи

void ScanKeyPowerOff 		(void); // обработчик нажатия кнопки PowerOff
void ScanKeySOS 			(void); // обработчик нажатия кнопки SOS

void ReceiveGPS				(void); // разбор и декодирование данных от GPS
void Measurement1PPS		(void);	// измерение и усреднение периода сигнала 1PPS
void Average1PPS			(void);	// усреднение измеренных значений периода 1PPS

void ClearGPSBuffer			(void);	// очистка буфера UART GPS

void ClearBLEBuffer			(void);	// очистка буфера UART BLE

// ------------- NMEA PARCER ------------------------------
u8  NMEA_Parser_GGA			(void);
u8  NMEA_Parser_ZDA			(void);
u8	NMEA_Parser_GSV_GPS		(void);
u8 	NMEA_Parser_GSV_GLO		(void);
u8	NMEA_Parser_GSA_GPS		(void);
u8	NMEA_Parser_GSA_GLO		(void);

u8	ControlCheckSum			(u16 StartIndex);

u32 GetGPSTimeSec 			(void); // вычисление времени GPS из ZDA сентенции. Целые секунды
u32 GetGPSTimeMilliSec 		(void); // вычисление времени GPS из ZDA сентенции. Миллисекунды

// ---------------------------- #define ---------------------------------------------

#define GPSBUFLENGTH  (1000)       // Буфер побольше, чтобы влезли все нужные посылки (GSV особенно длинная)
#define RESERVE (0)   // Можно удлинить поля структуры GPGGA_Struct, если изменится формат или при другой необходимости
#define MAXSAT (64)   // Максимальное количество спутников, GPS + ГЛОНАСС, для массива параметров видимых спутников
#define PARSAT (4)    // Номер, угол места, азимут, SNR, для массива параметров видимых спутников
#define ONESAT "nnn"  // Максимальная длина одного поля массива параметров видимых спутников

#define USE_SX1276_RADIO	// выбор радиомодуля для библиотеки LORA
#define BUFFER_SIZE	256		// Define the payload size here LORA
#define BW 250  	  		// ширина полосы сигнала для расчета ошибки по частоте, кГц. LORA

#define Terminal			USART1_SendByte
#define BleUsart			USART2_SendByte
#define GpsUsart 			USART3_SendByte
#define OLED_font1 			OLED_Putc1  // шрифт 7x10 пикселей
#define OLED_font2 			OLED_Putc2  // 		11x18
#define OLED_font3 			OLED_Putc3  // 		16x26

#define BufferUSART1Size 	100 	// размер приемного буфера USART1
#define BufferUSART2Size 	100 	// размер приемного буфера USART2
#define BufferUSART3Size 	100 	// размер приемного буфера USART3
#define BaudRateUSART1		115200	// скорость USART1  - связь с ПК, отладка через терминал
#define BaudRateUSART2		9600	// скорость USART2  - связь с BLE, переключается на 115200
#define BaudRateUSART3		9600	// скорость USART3  - связь с GPS, переключается на 115200

#define VaccPowerOff		3000 	// напряжение аккумулятора, при котором питание отключается
#define VaccAttention		3100 	// напряжение аккумулятора, при котором при загрузке выдается предупреждение и дальнейшая работа устройства блокируется

#define STOP 				for(;;){} // бесконечный пустой цикл, для отладки

// ----------------------- Глобальные переменные -----------------------------------------------
// ----------------- USART, TIM ---------------------------------------------------
u8				NumberDev = LORA_TX_TIME;			// номер устройства		

ErrorStatus 	HSEStartUpStatus;

volatile u8 	triggerUSART1Interrupt = 0;
volatile u16 	counterUSART1Buffer;
volatile char	BufferUSART1RX[BufferUSART1Size+1] = {'\0'}; // приемный буфер USART1

volatile u8 	triggerUSART2Interrupt = 0;
volatile u16 	counterUSART2Buffer;
volatile char	BufferUSART2RX[BufferUSART2Size+1] = {'\0'}; // приемный буфер USART2

volatile u8 	triggerUSART3Interrupt = 0;
volatile u16 	counterUSART3Buffer;
//volatile char	BufferUSART3RX[BufferUSART3Size+1] = {'\0'}; // приемный буфер USART3

volatile u32 	timer1ms, timer1s; 		// переменные системного таймера

volatile u8		triggerTIM2Interrupt;   // флаг прерывания по приему посылки от BLE модуля
volatile u8		triggerTIM3Interrupt;   // флаг прерывания по приему посылки от GPS модуля

volatile 		u32 SecRTC	;			// текущее время RTC в секундах
u32 			SecRTCold;				// предыдущее значение времени. Для лога напряжения аккумулятора
u32 			sec_RTC_to_display ;	// секунды от таймера для отображения
u32 			sec_GPS_to_display ;	// секунды от GPS для отображения

u32 i, j; 	// счетчик общего назначения

// --------------------------- переменные GPS приемника ----------------------------
char 	GPSBuf[GPSBUFLENGTH];	// Буфер UART для посылок от GPS модуля
volatile u16 GPSBufWrPoint; 	// Буфер не кольцевой, а линейный, нужна только точка записи
u16	GSVStartPos = 0; 			// Эти переменные используются только в NMEA_Parser_GSV(), но обнуляются только
//u16 GSVStartPosGLO = 0;		// В основной программе, потому эти переменные инкрементируются во время нескольких последовательных вызовов GSVStartPos
u8 	SatNumGPS = 0;				// Количество спутников GPS
u8 	SatNumGLO = 0;				// Количество спутников GLONASS
u8 	SatFix = 0;	 				// Количество спутников, используемых в решении из GGA последовательности
u8 	SatFixGPS = 0;	 			// Количество спутников GPS, используемых в решении из GSA
u8 	SatFixGLO = 0;	 			// Количество спутников GLONASS, используемых в решении из GSA
u8 	GSVNumStringGPS	 = 0; 		// Количество GSV строк по GPS 
u8 	GSVNumStringGLO	 = 0;		// Количество GSV строк по GLONASS

struct GPGGA_Struct  		// Структура с данными GPS-ГЛОНАСС-модуля
{
  char Time [sizeof("hhmmss.ss")+RESERVE];      	// Время
  char Latitude [sizeof("xxxx.yyyy")+RESERVE];    	// Широта
  char NS;                            				// Север-Юг
  char Longitude[sizeof("xxxxx.yyyy")+RESERVE]; 	// Долгота
  char EW;                            				// Запад-Восток
  char ReceiverMode;                    			// Режим работы приемника
  char SatelliteNum [sizeof("nn")+RESERVE];     	// Количество спутников в решении
  char Altitude [sizeof("aaaaa.a")+RESERVE];    	// Высота
  char Year[sizeof("yyyy")+RESERVE];          		// Год
  char Month[sizeof("mm")+RESERVE];         		// Месяц
  char Day[sizeof("dd")+RESERVE];           		// Число
  char LocalTimeHr [sizeof("+hh")+RESERVE];     	// Поправка на местное время, часы
  char LocalTimeMn [sizeof("mm")+RESERVE];      	// Поправка на местное время, минуты
  char Sats [MAXSAT][PARSAT][sizeof(ONESAT)];   	// Массив параметров видимых спутников GPS 
  char SatsGLO [MAXSAT][PARSAT][sizeof(ONESAT)];   	// Массив параметров видимых спутников GLONASS
};

struct GPGGA_Struct GPSFixData; 	// С этой структурой и будем работать
u8  SatLevelsGPS[40]; 				// уровни S/N со спутников GPS
u8  SatIDGPS 	[40]; 				// номера видимых спутников GPS
u8  SatLevelsGLO	[40];			// уровни S/N со спутников GLONASS
u8  SatIDGLO 	[40]; 				// номера видимых спутников GLONASS
u16 SatSummGPS, SatSummUsedGPS;		// сумма S/N видимых и используемых спутников GPS
u16 SatSummGLO, SatSummUsedGLO;		// сумма S/N видимых и используемых спутников GLONASS
u8	SatUsedGPS	[12];				// массив номеров использованных в решении спутников GPS
u8	SatUsedGLO	[12];				// массив номеров использованных в решении спутников GLONASS
u8	SatNumUsed;						// количество использованных в решении спутников
u16	PDOP, VDOP, HDOP;				// коэффициенты ухудшения точности определения координат ( x100 )
u8 	Fix;							// режим работы GPS приемника - 2D/3D/нет решения
u8	AutoSel_2D3D;					// 1 - автоматический выбор 2D/3D, 0 - ручной
u32 GPSTimeSec;						// количество секунд текущих суток из ZDA сентенции
u32 GPSTimeMilliSec;				// количество миллисекунд из ZDA сентенции
u32 GPSLongitude, GPSLatitude;		// текущие широта и долгота
u8  GPS_N_S, GPS_E_W ;				// 1 - север, 0 - юг; 1 - запад, 0 - восток
u32 GPSAltitude;					// текущая высота
u16 TTFF;							// время до первой фиксации местоположения
// --------------------------------------------------------------------------------
char c1;
u8 size;
char m;

int tempData = 0;
char test [50];

bool triggerTTFF ;		// триггер измерения времени первой фиксации местоположения
bool flagTX;			// флаг запуска передачи буфера на 433
bool flagTXDisplay;
bool flagRXDisplay;
//----------------------------- переменные LORA ------------------------------------------
u8 	LoraTxTime  = LORA_TX_TIME; // номер секунды для передачи в эфир

s8 	RFPower = RFPowerDef;		// мощность передатчика
u16 SPI1read;

u8  LORAbufferRX 	[256];
u8  LORAbufferTX 	[256];

static u16 BufferSize = BUFFER_SIZE;			// RF buffer size
//static u8 Buffer[BUFFER_SIZE];				// RF buffer
tRadioDriver *Radio = NULL;						// структура RadioDriver , указатели на функции

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
double FreqError;		// ошибка принятого сигнала по частоте
s32 FREQ_ERROR;
bool flagRX_OK;			// флаг готовности новых данных в приемном буфере

volatile u16 counterRXDisplay; 	// счетчик времени отображения RX на дисплее
volatile u16 counterTXDisplay; 	// счетчик времени отображения TX на дисплее
bool triggerOffLED0;			// триггеры для однократного выключения светодиодов
bool triggerOffLED1;
volatile u16 counterLed0;		// счетчик времени горения Синего светодиода
volatile u16 counterLed1;		// счетчик времени горения Красного светодиода


u32 LoraResult ;			// состояние автомата состояний  LORA трансивера
//u32 LoraCounter ;			// тестовый счетчик

typedef struct LoraRXstruct_t 		// структура параметров принятого сигнала от одного маяка
{
	s16		RSSI;		// измеренное значение уровня сигнала RSSI в принятом пакете
	s32 	FreqError;	// относительное смещение по частоте принятого пакета
	s8		SNR;		// соотношение сигнал/шум для принятого пакета
	s8		Power;		// мощность излучения передатчика, передается в пакете
	u8 		CounterTxP;	// счетчик переданных пакетов. Временно todo
}LoraRXstruct;

LoraRXstruct LoraRX [10]; 	// массив структур параметров приема от каждого из маяков
s16 LoraRssiMax = -130	;	// максимальный уровень сигнала из принятых пакетов, для индикатора
u16 counterMaxRssiReset ;	// счетчик для периодического сброса максимального уровня RSSI
u16 counterRssiReset [10];	// массив счетчиков для всех маяков, для сброса уровня RSSI по времени

u8 NumberTransmitted;		// номер маяка, от которого принята посылка
s8 RFPowerTransmitted;  	// мощность передатчика, от которого принята посылка

u32	TimeSecFirstRX [10];	// массив времен когда от данного маяка был принят сигнал в первый раз. Для вычисления PER
u16 CountRX	[10];			// счетчики принятых посылок от маяков. Для вычисления PER
u16 PER [10];				// массив измерненых PER для каждого маяка

//u16 timeDelta; // временно
#pragma pack (push, 1) 		// помещаем текущее значение выравнивания упаковки во внутренний стек компилятора, текущее выравнивание = 1 байт

typedef struct datastruct_t // структура данных одного маяка
{	// 6 32-разрядных слов, 24 байт
	u8	Number;			// номер маяка и мощность принятого сигнала (+ флаги)
	u8  NS_EW;			// с-ю / в-з
	u16 Vacc;			// напряжение аккумулятора
	u32 TimeSec;		// текущее время UTC
	u32 Longitude;		// Долгота
	u32 Latitude;		// широта
	u32 Altitude;		// высота
	u32 reserv;			// резерв  (NO FIX, 2D, 3D)
}DATASTRUCT;  			// Определяем новый тип переменных и саму структуру

#pragma pack (pop)			// удаляем запись из вершины внутреннего стека компилятора


// DATASTRUCT TX_str ;

//DATASTRUCT  TX_Packet [10] ; 	//

// приемная сторона
// новый массив из 10-ти структур типа DATASTRUCT
DATASTRUCT structBuffer [10];

u32 TimeNew	;				// новое время посылки от каждого из маяков
u32 TimesOld	[10];		// массив старых времен посылок от каждого из маяков

// где-то в инициализации:
//char *inBuffer = (char*)structBuffer; // указатель на переменную типа char. structBuffer также приводится к этому типу
// записав поток данных в inBuffer [] получим заполненный массив структур типа DATASTRUCT (?)

//------------------------------ переменные ADC --------------------------------------
u16 adc1, adc2, adcIntRef;
u32 Vacc1, Vbat1, Vref1, VaccSumm, VbatSumm, VrefSumm; 	// текущее напряжение аккумулятора и батарейки, в мВ
#define Average1	1000									// размер массивов для усреднения измеренных напряжений
u16 Vacc [Average1];									// массивы для усреднения скользящим средним
//u16 Vbat [Average1];
u16 Vref [Average1];
u16 VaccAvg, VrefAvg;			// усредненные значения
s8 VrefDev, VaccDev;			// отклонение от номинального значения, в %
bool ChargeTrigger; 			// триггер заряда аккумулятора, для индикации

u8 VaccLog [128]; 		// данные напряжения аккумулятора для графика
u8 counterVaccLog; 		// счетчик текущего отсчета в логе
u32 TimeStartLogVcc; 	// время начала записи в лог
u32 TimeLogVcc; 		// длительность записи в лог
bool triggerTimeStartLogVcc ; // триггер для определения времени начала записи лога
u8 counterVaccosc;		// счетчик для осциллограммы мгновенного напряжения
s8 VaccOsc [100];		// массив для осциллограммы мгновенного напряжения
//----------------------- кнопки, экран ----------------------------------------------
bool trigger1, trigger2;				// триггеры однократного нажатия кнопок
u8	screen = 1;							// номер экрана для отображения
#define  PowerOffTimeout 100			// таймаут для длительного нажатия кнопки SCREEN
u16	counterPowerOff = PowerOffTimeout; 	// счетчик задержки выключения питания кнопкой
u32 counterLcdOff = LcdOffTimeout;		// счетчик задержки выключения дисплея, мс

//------------------ таймер, измерение периода 1PPS ------------------------------
#define  AVG_TIM4 			30 			// усреднять 1PPS по 30 значениям

volatile u8 counter_TIM4_OVR; 			// счетчик переполнений таймера 4
volatile u8 counter_TIM1_OVR; 			// счетчик переполнений таймера 1
volatile u8 counter_TIM4_fix;			// значение счетчика циклов таймера на момент прихода прерывания 1PPS
volatile u16 TIM4_fix;					// значение таймера в момент прихода прерывания 1PPS
volatile bool flag_1PPS_Update;  		// флаг обновления значения периода 1PPS
volatile bool flag_1PPS_timeout;		// 1PPS отсутствует
u16 averageTIM4 [AVG_TIM4];				// массив для усреднения значения периода 1PPS
u16 averageTIM4_temp;					// усредненное значение таймера 1PPS
u16 averageTIM4_ready = 37200;			// усредненное значение таймера 1PPS после 35 усреднений
volatile bool flag_1PPS_fail;			// флаг ошибки измерения периода 1PPS, период слишком маленький или большой
volatile bool flag_1PPS_AVG_OK = 0;		// флаг готовности результатов усреднения 1PPS
volatile bool flag_sync_stop = 0;		// временный флаг для измерений ухода частоты таймера
volatile u8 counter_AVG_1PPS = 0;		// счетчик усреднений 1PPS
u8 i_avg = 0; 							// счетчик усреднений
u32 summ = 0;							// сумма для усреднения
volatile bool flag_1PPS_pulse;			// флаг устанавливается в прерывании от 1PPS или от таймера 1

u32 sec_div;							// остаток от деления секунд текущих суток на 60

double distance1_1_2, distance1_1_3, distance1_2_3;		// расстояние между маяками
double distance2_1_2, distance2_1_3, distance2_2_3;		// расстояние между маяками
double latt_1, latt_2, latt_3, latt_4;  // широты  маяков в градусах
double long_1, long_2, long_3, long_4; 	// долготы двух маяков в градусах
double grad, rad;



// переменные I2C1
__IO uint8_t Tx_Idx1=0 ;
extern __IO uint32_t NumbOfBytes1;
extern uint8_t Buffer_Tx1[];
extern uint8_t Address;

#define DebugMode	0		// Расширеный список экранов
#define ReleaseMode	1		// Сокращенный список экранов
u8 ScreenMode = DebugMode;	// Режим вывода информации на экран - полный список или сокращенный
u8 CounterTxPacket;			// Счетчик переданных в эфир пакетов. Для контроля прохождения пакетов по сети, временно

//--------------------------------- MAIN -----------------------------------------------------------------------------------------------------------------------------
int  main(void)
{
	init(); 				// инициализация контроллера, периферии, внешних модулей

	Radio -> StartRx();		// включение трансивера на прием
//============================================================================================
	for (;;) 	// бесконечный цикл
	{
		
		Measurement1PPS (); 	// Измерение периода сигнала 1PPS
	
		Average1PPS ();			// Усреднение

		ReceiveGPS(); 			// Проверка приема посылки от GPS и парсинг



// -- вычисление расстояния между маяками  ----------- 55°10.7014  55107014     61°24.0958  6124058

//		structBuffer[1].Latitude = 55450000 ;
//		structBuffer[1].Longitude = 61370000 ;
//
//		structBuffer[2].Latitude = 55460000 ;
//		structBuffer[2].Longitude = 61380000 ;

//latt_2 = ( ( (GPSLatitude / 1000000) + ( (GPSLatitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// координата в радианах
//long_2 = ( ( (GPSLongitude / 1000000) + ( (GPSLongitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// координата в радианах

		latt_1 = ( ( (structBuffer[1].Latitude / 1000000) + ( (structBuffer[1].Latitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// координата в радианах
		latt_2 = ( ( (structBuffer[2].Latitude / 1000000) + ( (structBuffer[2].Latitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// координата в радианах
		latt_3 = ( ( (structBuffer[3].Latitude / 1000000) + ( (structBuffer[3].Latitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// координата в радианах
		latt_4 = ( ( (structBuffer[4].Latitude / 1000000) + ( (structBuffer[4].Latitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// координата в радианах
		//rad = (grad * M_PI) / 180;							// координата в радианах

		long_1 = ( ( (structBuffer[1].Longitude / 1000000) + ( (structBuffer[1].Longitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// координата в радианах
		long_2 = ( ( (structBuffer[2].Longitude / 1000000) + ( (structBuffer[2].Longitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// координата в радианах
		long_3 = ( ( (structBuffer[3].Longitude / 1000000) + ( (structBuffer[3].Longitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// координата в радианах
		long_4 = ( ( (structBuffer[4].Longitude / 1000000) + ( (structBuffer[4].Longitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;		// координата в радианах

		//distance1 = sin( latt_2 );


		distance1_1_2 = 6372795 * acos ( sin (latt_1) * sin (latt_2) +cos (latt_1) * cos (latt_2) * cos (long_1 - long_2) )  ; // Расстояние между 1 и 2 маяками. Аргументы sin в радианах . Значение в метрах
		distance1_2_3 = 6372795 * acos ( sin (latt_2) * sin (latt_3) +cos (latt_2) * cos (latt_3) * cos (long_2 - long_3) )  ; // Расстояние между 2 и 3 маяками. Аргументы sin в радианах . Значение в метрах
		distance1_1_3 = 6372795 * acos ( sin (latt_1) * sin (latt_3) +cos (latt_1) * cos (latt_3) * cos (long_1 - long_3) )  ; // Расстояние между 1 и 3 маяками. Аргументы sin в радианах . Значение в метрах



		if ( LORA_TX_TIME == 1)  // если этот маяк - №1
		{
			latt_1 = ( ( (GPSLatitude / 1000000) + ( (GPSLatitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;				// координата текущего местоположения в радианах
			long_1 = ( ( (GPSLongitude / 1000000) + ( (GPSLongitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;			// координата в радианах
			distance2_1_2 = 6372795 * acos ( sin (latt_1) * sin (latt_2) +cos (latt_1) * cos (latt_2) * cos (long_1 - long_2) )  ; 	// Расстояние между текущим местоположением и последним принятым от другого маяка. Аргументы sin в радианах . Значение в метрах
			distance2_1_3 = 6372795 * acos ( sin (latt_1) * sin (latt_3) +cos (latt_1) * cos (latt_3) * cos (long_1 - long_3) )  ; 	// Расстояние между текущим местоположением и последним принятым от другого маяка. Аргументы sin в радианах . Значение в метрах
		}

		if ( LORA_TX_TIME == 2)  // если этот маяк - №2
		{
			latt_2 = ( ( (GPSLatitude / 1000000) + ( (GPSLatitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;				// координата текущего местоположения в радианах
			long_2 = ( ( (GPSLongitude / 1000000) + ( (GPSLongitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;			// координата в радианах
			distance2_1_2 = 6372795 * acos ( sin (latt_1) * sin (latt_2) +cos (latt_1) * cos (latt_2) * cos (long_1 - long_2) )  ; 	// Расстояние между текущим местоположением и последним принятым от другого маяка. Аргументы sin в радианах . Значение в метрах
			distance2_2_3 = 6372795 * acos ( sin (latt_2) * sin (latt_3) +cos (latt_2) * cos (latt_3) * cos (long_2 - long_3) )  ; 	// Расстояние между текущим местоположением и последним принятым от другого маяка. Аргументы sin в радианах . Значение в метрах
		}

		if ( LORA_TX_TIME == 3)  // если этот маяк - №3
		{
			latt_3 = ( ( (GPSLatitude / 1000000) + ( (GPSLatitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;				// координата текущего местоположения в радианах
			long_3 = ( ( (GPSLongitude / 1000000) + ( (GPSLongitude % 1000000) / 10000.0 ) / 60.0 ) * M_PI ) / 180 ;			// координата в радианах
			distance2_1_3 = 6372795 * acos ( sin (latt_1) * sin (latt_3) +cos (latt_1) * cos (latt_3) * cos (long_1 - long_3) )  ; 	// Расстояние между текущим местоположением и последним принятым от другого маяка. Аргументы sin в радианах . Значение в метрах
			distance2_2_3 = 6372795 * acos ( sin (latt_2) * sin (latt_3) +cos (latt_2) * cos (latt_3) * cos (long_2 - long_3) )  ; 	// Расстояние между текущим местоположением и последним принятым от другого маяка. Аргументы sin в радианах . Значение в метрах
		}


//-------------------------------------- LORA TX --------------------------------------------

		if ( flagTX == 1)  // поднят флаг запуска передачи
			{
				flagTX = 0;

		 // --- заполняем структуру данных  для текущего прибора ------
				structBuffer[NumberDev].Number = NumberDev;  		// U8 поле Number (номер прибора) структуры structBuffer с номером данного прибора
				structBuffer[NumberDev].NS_EW = 0x00;				// U8 признак Восток-Запад, Север-Юг. Дополнительные флаги
				structBuffer[NumberDev].Vacc = VaccAvg; 			// U16 напряжение аккумулятора данного прибора
				structBuffer[NumberDev].TimeSec = SecRTC;			// U32 текущее время в секундах c начала суток
				structBuffer[NumberDev].Longitude = GPSLongitude;	// U32 долгота места
				structBuffer[NumberDev].Latitude = GPSLatitude;		// U32 широта места
				structBuffer[NumberDev].Altitude = GPSAltitude;		// U32 высота места
				structBuffer[NumberDev].reserv = Fix;				// U32 резерв Временно Fix - режим работы GPS приемника 1- нет решения, 2- 2D, 3- 3D

				for (i = 0; i < 10; i ++ )  // заполняем буфер передачи. 10 по 24 байта
				{
					LORAbufferTX[24 * i + 0] = structBuffer[i].Number;			// 8 бит
					LORAbufferTX[24 * i + 1] = structBuffer[i].NS_EW;			// 8 бит
					LORAbufferTX[24 * i + 2] = structBuffer[i].Vacc >> 8;		// 16 бит
					LORAbufferTX[24 * i + 3] = structBuffer[i].Vacc;
					LORAbufferTX[24 * i + 4] = structBuffer[i].TimeSec >> 24;	// 32 бита
					LORAbufferTX[24 * i + 5] = structBuffer[i].TimeSec >> 16;
					LORAbufferTX[24 * i + 6] = structBuffer[i].TimeSec >> 8;
					LORAbufferTX[24 * i + 7] = structBuffer[i].TimeSec;
					LORAbufferTX[24 * i + 8] = structBuffer[i].Longitude >> 24;	// 32 бита
					LORAbufferTX[24 * i + 9] = structBuffer[i].Longitude >> 16;
					LORAbufferTX[24 * i + 10] = structBuffer[i].Longitude >> 8;
					LORAbufferTX[24 * i + 11] = structBuffer[i].Longitude;
					LORAbufferTX[24 * i + 12] = structBuffer[i].Latitude >> 24;	// 32 бита
					LORAbufferTX[24 * i + 13] = structBuffer[i].Latitude >> 16;
					LORAbufferTX[24 * i + 14] = structBuffer[i].Latitude >> 8;
					LORAbufferTX[24 * i + 15] = structBuffer[i].Latitude;
					LORAbufferTX[24 * i + 16] = structBuffer[i].Altitude >> 24;	// 32 бита
					LORAbufferTX[24 * i + 17] = structBuffer[i].Altitude >> 16;
					LORAbufferTX[24 * i + 18] = structBuffer[i].Altitude >> 8;
					LORAbufferTX[24 * i + 19] = structBuffer[i].Altitude;
					LORAbufferTX[24 * i + 20] = structBuffer[i].reserv >> 24;	// 32 бита
					LORAbufferTX[24 * i + 21] = structBuffer[i].reserv >> 16;
					LORAbufferTX[24 * i + 22] = structBuffer[i].reserv >> 8;
					LORAbufferTX[24 * i + 23] = structBuffer[i].reserv;
				}

				LORAbufferTX[240] = NumberDev;  // в конце посылки добавляем номер устройства, которое передает
				LORAbufferTX[241] = RFPower;	// и мощность передатчика
				LORAbufferTX[242] = CounterTxPacket++;			// счетчик пакетов todo

				Radio->SetTxPacket( LORAbufferTX, 255 ); 	// передача пакета ( можно передавать только 243 байт )


				counterLed1 = 800;
				OnLED1();					// зажигаем КРАСНЫЙ светодиод на 800 мс
				counterTXDisplay = 800;		// рисуем TX 800 мс или пока не произойдет прерывание
// ----------------------------------------- BLE -----------------------------------------------------------------
				rprintfInit(BleUsart);  // вывод в BLE NumberDev
				printf("\n\r\n\r #%d TxPacket at ", NumberDev);
				Time_Display (structBuffer[NumberDev].TimeSec);

	#ifdef DebugLORAToUART
		rprintfInit (Terminal); 	// инициализация printf (). Вывод в терминал
		printf("\n\r\n\r SetTxPacket! \n\r");
	#endif
			}

//---------------------------------------------------------------------------------------------------------------------------------------------
	    LoraProcess();  	// обработка состояний радиоприемника

//		if (counterRXDisplay == 0) // счетчик обнулился
//		{
//			if (triggerOffLED0 == 0)
//			{
//				triggerOffLED0 = 1;
//				OffLED0(); 		// гасим СИНИЙ светодиод однократно
//			}
//		}
//		else
//		{
//			triggerOffLED0 = 0;
//		}
//
//		if (counterTXDisplay == 0) // счетчик обнулился
//		{
//			if (triggerOffLED1 == 0)
//			{
//				triggerOffLED1 = 1;
//				OffLED1(); 		// гасим КРАСНЫЙ светодиод однократно
//			}
//		}
//		else
//		{
//			triggerOffLED1 = 0;
//		}


// -------------------------------------- LORA RX -------------------------------- собираем массив структур из пакета принятого по радиоканалу --------------
		if ( flagRX_OK == 1) 		// по радиоканалу принят новый пакет
		{
			flagRX_OK = 0; 			// сбросили флаг, обрабатываем однократно

			for (i=0; i<10; i++) 	// парсим времена
			{
				TimeNew  = (LORAbufferRX[24 * i + 4]<<24) + (LORAbufferRX[24 * i + 5]<<16) + (LORAbufferRX[24 * i + 6]<<8) + LORAbufferRX[24 * i + 7]; // время в новой посылке

				// сравниваем со старыми временами
				if (TimeNew  > TimesOld [i]) // если данные более свежие, то
				{
					TimesOld [i] = TimeNew; // обновляем время

					structBuffer[i].Number = 	LORAbufferRX[24 * i + 0]; // обновляем все параметры в структуре
					structBuffer[i].NS_EW = 	LORAbufferRX[24 * i + 1];
					structBuffer[i].Vacc = 		(LORAbufferRX[24 * i + 2]<<8) + LORAbufferRX[24 * i + 3];
					structBuffer[i].TimeSec = 	(LORAbufferRX[24 * i + 4]<<24) + (LORAbufferRX[24 * i + 5]<<16) + (LORAbufferRX[24 * i + 6]<<8) + LORAbufferRX[24 * i + 7]; // оно же TimeNew
					structBuffer[i].Longitude = (LORAbufferRX[24 * i + 8]<<24) + (LORAbufferRX[24 * i + 9]<<16) + (LORAbufferRX[24 * i + 10]<<8) + LORAbufferRX[24 * i + 11];
					structBuffer[i].Latitude =  (LORAbufferRX[24 * i + 12]<<24) + (LORAbufferRX[24 * i + 13]<<16) + (LORAbufferRX[24 * i + 14]<<8) + LORAbufferRX[24 * i + 15];
					structBuffer[i].Altitude =  (LORAbufferRX[24 * i + 16]<<24) + (LORAbufferRX[24 * i + 17]<<16) + (LORAbufferRX[24 * i + 18]<<8) + LORAbufferRX[24 * i + 19];
					structBuffer[i].reserv = 	(LORAbufferRX[24 * i + 20]<<24) + (LORAbufferRX[24 * i + 21]<<16) + (LORAbufferRX[24 * i + 22]<<8) + LORAbufferRX[24 * i + 23];
				}
			}

			NumberTransmitted = LORAbufferRX[240]; 								// от какого номера пришла посылка
			LoraRX [NumberTransmitted].Power = LORAbufferRX[241]; 				// с какой мощностью он передает
			LoraRX [NumberTransmitted].FreqError = (s32)FreqError; 				// измеренное смещение частоты при приеме посылки
			LoraRX [NumberTransmitted].RSSI = (s16)SX1276LoRaGetPacketRssi();	// измеренный уровень сигнала
			LoraRX [NumberTransmitted].SNR = SX1276GetPacketSnr();				// измеренное соотношение сигнал-шум в пакете
			LoraRX [NumberTransmitted].CounterTxP = LORAbufferRX[242]; 			// счетчик переданных пакетов todo

			counterRssiReset [NumberTransmitted] = 11000; 	// заводим таймер для сброса LoraRssi от маяка на 11 секунд



			LoraRssiMax = -130;  // сброс на минимум
			for (i=0; i<10; i++) // поиск наибольшего уровня RSSI
			{
				if ( (LoraRX [i].RSSI != 0) && (LoraRX [i].RSSI > LoraRssiMax) )
				{
					LoraRssiMax = LoraRX [i].RSSI; 	// нашли максимальный сигнал

					counterMaxRssiReset = 11000; 	// заводим таймер для сброса LoraRssiMax на 11 секунд
				}

			}


			if (SecRTC > 0) // если время SecRTC уже определено
				{
					if ( TimeSecFirstRX [NumberTransmitted] == 0)		// если до этого время еще не сохранялось
						TimeSecFirstRX [NumberTransmitted] = SecRTC;	// запоминаем время первого приема сигнала от конкретного маяка для вычисления PER

					CountRX [NumberTransmitted] ++ ; 	// считаем сколько посылок было принято от каждого маяка
				}

			// вычисление PER если был принят пакет
			for (i=0; i<10; i++)  // проходим по всем маякам
			{
				if ( TimeSecFirstRX [i] > 0) // если уже были приняты посылки
				{	// вычисляем PER для каждого из маяков в сети в виде 100.00 % (делить на 100)
					PER [i] =10000 - ((u32)CountRX [i] * 10000) / ( (SecRTC - TimeSecFirstRX [i])  / 10 + 1);
				}
			}
// ------------------------------------------------------------ BLE --------------------------------------------------------------------------------------
			rprintfInit(BleUsart);  // вывод в BLE

			printf("\n\r\n\r #%d RSSI: %d dBm  FreqEr: %d Hz  SNR: %d dB",NumberTransmitted, LoraRX [NumberTransmitted].RSSI, LoraRX [NumberTransmitted].FreqError, LoraRX [NumberTransmitted].SNR);
			if (PER [NumberTransmitted] != 65535)
				printf(" PER: %d.%02d%%", PER[NumberTransmitted]/100, PER[NumberTransmitted]%100);

			printf("\n\r Time: ");
			Time_Display (structBuffer[NumberTransmitted].TimeSec);
			printf(" Latt: %d Long: %d \n\r",  structBuffer[NumberTransmitted].Latitude, structBuffer[NumberTransmitted].Longitude);

			temp3 = distance1_1_2 * 10;
			printf (" 1-2: %d.%d m ", temp3 / 10,  temp3 % 10 ); // расстояние метрах

			temp3 = distance1_1_3 * 10;
			printf ("1-3: %d.%d m ", temp3 / 10,  temp3 % 10 ); // расстояние метрах

			temp3 = distance1_2_3 * 10;
			printf ("2-3: %d.%d m ", temp3 / 10,  temp3 % 10 ); // расстояние метрах
		}
// -------------------------------------------------------------------------------------------------------------------------------------------------------

		for ( i = 0; i < 10; i ++) // проходим по всем счетчикам сброса RSSI
		{
			if ( counterRssiReset[i] == 0)
				LoraRX [i].RSSI = 0;  	// сбрасываем RSSI на начальное значение

		}

		// сброс индикатора уровня сигнала LORA
		if (counterMaxRssiReset == 0) // таймер обнулился
		{
			LoraRssiMax = -130; // сброс на минимум
		}

		// вычисление PER еще раз, если пакет не пришел вовремя
		for (i=0; i<10; i++)  // проходим по всем маякам
		{
			if ( TimeSecFirstRX [i] > 0) // если уже были приняты посылки
			{	// вычисляем PER для каждого из маяков в сети в виде 100.00 % (делить на 100)
				PER [i] =10000 - ((u32)CountRX [i] * 10000) / ( (SecRTC - TimeSecFirstRX [i])  / 10 + 1) ;
			}
		}


	    MeasVoltage();		// измерение напряжений аккумулятора, питания и батареи, проверка заряда

	    if (VaccAvg < VaccPowerOff)	// если напряжение аккумулятора меньше порога
	   	{
	    	NRESET_off();  	// выключаем GPS
	        OnResetBLE();	// сброс BLE
	        OnResetLORA();	// сброс LORA

	        Display2(); 	// напряжения аккумулятора и батареи

			while ( DMA_GetFlagStatus(DMA1_FLAG_TC6) == 0  )	// ждем пока закончится текущая транзакция по DMA
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
			SSD1306_UpdateScreenDMA();	  // запуск нового обновления экрана из видеобуфера


			while (1)
			{
				ScanKeyPowerOff();	// обработка нажатий кнопок
				ScanKeySOS();
			}

	        STOP
	    	//OffVcc1(); 		// выключаем питание
	   	}



	    ScanKeyPowerOff();	// обработка нажатий кнопок
	    ScanKeySOS();

	    if (ScreenMode == ReleaseMode)
	    {
	    	switch (screen)	// переключение информации на OLED дисплее
				{
				case 1:
					Display1Release(); // основной экран
					break;
				case 2:
					Display2Release(); // расстояния между маяками
					break;
				case 3:
					Display3Release(); // информация GPS
					break;

				default:
					screen = 1;
				}
	    }
	    else
	    {
	    switch (screen)	// переключение информации на OLED дисплее
			{
			case 1:
				Display1(); // информация GPS
				break;
			case 2:
				Display2(); // напряжения аккумулятора и батареи
				break;
			case 3:
				Display3(); // часы реального времени RTC
				break;
			case 4:
				Display4(); // уровень сигнала со спутников в виде ID S/N dB
				break;
			case 5:
				Display5(); // уровень сигнала со спутников в графическом виде
				break;
			case 6:
				Display6(); // информация о спутниках, используемых в решении и параметрах DOP
				break;
			case 7:
				Display7(); // координаты, высота в u32
				break;
			case 8:
				Display8(); // измерение периода 1PPS
				break;
			case 9:
				Display9(); // данные с приемника LORA
				break;
			case 10:
				Display10(); // данные с приемника LORA координаты
				break;
			case 11:
				Display11(); // данные с приемника LORA времена
				break;
			case 12:
				Display12(); // параметры принятых посылок
				break;
			case 13:
				Display13(); // PER от каждого из маяков
				break;
			case 14:
				Display14(); // расстояние между маяками
				break;
			case 15:
				Display15(); // уровни сигнала от маяков в графическом виде
				break;
			}
	    }


//OnLED0();
		//if ( counterLcdOff > 0)
		//{
				// DMA end of transfer  Закончилась передача в I2C по DMA
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
					SSD1306_UpdateScreenDMA();	  // запуск нового обновления экрана из видеобуфера
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
// ----------- Инициализация микроконтроллера, настройка
void init				(void)
{
	RCC_Configuration(); 	// настройка схемы тактирования
	SysTick_Config(72000-1);// системный таймер тикает каждую мс
	PortIO_Configuration();	// настройка портов
OnVcc1();		// удерживаем 1 на линейных стабилизаторах
	USART1_Configuration();	// настройка USART1
	USART2_Configuration();	// настройка USART2
	USART3_Configuration();	// настройка USART3
	TIM1_Configuration();	// настройка Timer1 - таймер формирует сигнал синхронизации для временного разделения каналов LORA сети. Синхронизируется по 1PPS от GPS модуля
	TIM2_Configuration();	// настройка Timer2 - таймер формирует таймаут для приема посылки по usart2
    TIM3_Configuration();	// настройка Timer3 - таймер формирует таймаут для приема посылки по usart3
    TIM4_Configuration();	// настройка Timer4 - таймер измеряет период сигнала 1PPS, для синхронизации LORA сети в отсутсвии сигнала 1PPS от GPS модуля
    SPI1_Configuration();	// настройка SPI1
	NVIC_Configuration();	// настройка прерываний
	ADC1_Configuration();	// настройка АЦП
	EXTI_Configuration();	// настройка внешнего прерывания на PB12
	DMA_Configuration ();	// настройка DMA для I2C дисплея

	DelayMS(100);			// пауза для правильной инициализации дисплея

	SSD1306_Init();  		// инициализация дисплея

//DelayMS(200);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // включение прерывания по приему байта
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // включение прерывания по приему байта
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // включение прерывания по приему байта

	rprintfInit (Terminal); // инициализация printf (). Вывод в терминал
	printf("\n\r\n\r Power ON ! ");
	printf("\n\r RESET! USART1\n\r");

	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

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

	rprintfInit (OLED_font2); 	// вывод на OLED дисплей
	SSD1306_GotoXY(3, 7);

	for (j=0; j<1; j++) // 20 циклов измерения напряжений аккумулятора и Vcc
	{
		MeasVoltage();
		OffLED1();// красный

		SSD1306_GotoXY(32, 0);
		printf("%d.%02d V ", VaccAvg/1000, (VaccAvg%1000)/10 ); // показываем с точностью 10 мВ

		// батарейка
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

	if (VaccAvg < VaccAttention) // сравниваем с порогом напряжения
	{
		OffLED0();
		OnLED1();
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		SSD1306_GotoXY(4, 5);
		SSD1306_Puts("LOW BATTERY", &Font_11x18, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(58, 27);
		SSD1306_Puts("!", &Font_16x26, SSD1306_COLOR_WHITE);

		rprintfInit (OLED_font1); 	// вывод на OLED дисплей
		SSD1306_GotoXY(4, 53);
		printf("%d mV ", VaccAvg);

		SSD1306_UpdateScreen();

		DelayMS(4000);

		OffVcc1();		// выключаем питание

		STOP  // бесконечный цикл, ждем пока отпустят кнопку
	}


//SSD1306_DrawRectangle(0, 0, 127, 64, SSD1306_COLOR_WHITE );		// рамка
//OnLED0();
//	SSD1306_UpdateScreenDMA();
////OffLED0();
////OnLED1();
//// DMA end of transfer  Закончилась передача в I2C по DMA
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
	OffVcc1();		// выключаем питание
	DelayMS(10);	// пауза чтобы успел выключиться

/*	for ( i=0; i<100; i=i+2 ) // анимация включения
	{
		SSD1306_DrawFilledRectangle(14, 40, i, 5, SSD1306_COLOR_WHITE ); 	// градусник
		SSD1306_DrawRectangle(12, 38, 102, 9, SSD1306_COLOR_WHITE );		// рамка
		SSD1306_DrawLine(13+20, 40, 13+20, 45 ,SSD1306_COLOR_BLACK);		// деления
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
	OnVcc1();		// удерживаем 1 на линейных стабилизаторах

	OnLED0(); 		// Вкл синий светодиод. Пошла загрузка

	if ( GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 0 )  		// если кнопка SCREEN нажата во время включения питания
		{
		ScreenMode = ReleaseMode; 	// сокращенный список экранов
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
	NRESET_on();  // включаем GPS

	rprintfInit (Terminal); // инициализация printf (). Вывод в терминал
	printf("\n\r GPS power ON  ... OK");

	SSD1306_GotoXY(0, 10);
	SSD1306_Puts("GPS setup...  ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();

	DelayMS(800); // пауза для загрузки GPS модуля

/////////////// test GPS boudrate ////////////////
	ClearGPSBuffer();  				// очистка приемного буфера GPS

	rprintfInit (GpsUsart);  		// переключаем вывод на USART3 - в модуль GPS
	printf("$PMTK000*32\r\n\n"); 	// команда TEST , ответ должен быть $PMTK001,0,3*30\r\n

	rprintfInit (Terminal); 		// инициализация printf (). Вывод в терминал
	printf("\n\r GPS TEST 9600 ...  \n\r");
	DelayMS(200); 					// задержка на ответ от GPS модуля

//if ( triggerTIM3Interrupt == 1 ) 		// проверка  - пришла посылка от GPS модуля
//{
	for (i=0; i < 20; i++) 	// вывод буфера в терминал
		{
		if (GPSBuf[i] == '\0' || GPSBuf[i] == '\r' || GPSBuf[i] == '\n') break; 	// выводим только до первого 0 или \r в буфере
		USART_SendByte(USART1, GPSBuf[i]);
		}

	if (GPSBuf[0] == '$' && GPSBuf[1] == 'P' && GPSBuf[2] == 'M' && GPSBuf[3] == 'T' && GPSBuf[4] == 'K' && GPSBuf[5] == '0'
			&& GPSBuf[6] == '0' && GPSBuf[7] == '1' && GPSBuf[8] == ',' && GPSBuf[9] == '0' && GPSBuf[10] == ','
					&& GPSBuf[11] == '3' && GPSBuf[12] == '*' && GPSBuf[13] == '3' && GPSBuf[14] == '0' ) // скорость 9600
		{
			printf("\tOK  ");
//			printf("\n\r Set to 115200 ");
//			rprintfInit (GpsUsart);  				// переключаем вывод на USART3 - в модуль GPS
//			printf("$PMTK251,115200*1F\r\n\n"); 	// настройки скорости USART - 115200 бод
//			//printf("$PMTK251,9600*17\r\n\n);	//	9600 бод
//			DelayMS(100);
		}
	else
		printf("\tNO! ");

	printf("\n\r Set to 115200 ");          // принудительное переключение, проверка проходит не всегда
	rprintfInit (GpsUsart);  				// переключаем вывод на USART3 - в модуль GPS
	printf("$PMTK251,115200*1F\r\n\n"); 	// настройки скорости USART - 115200 бод
	DelayMS(100);

	rprintfInit (Terminal); 		// инициализация printf (). Вывод в терминал
	printf("\n\r GPS TEST 115200..\n\r ");

	USART3_Configuration115200();  	// переключаем скорость USART3 на 115200

	ClearGPSBuffer();  				// очистка приемного буфера GPS

	rprintfInit (GpsUsart);  		// переключаем вывод на USART3 - в модуль GPS
	printf("$PMTK000*32\r\n\n"); 	// команда TEST , ответ должен быть $PMTK001,0,3*30\r\n

	DelayMS(200); 					// задержка на ответ от GPS модуля
	rprintfInit (Terminal); 		// инициализация printf (). Вывод в терминал

	for (i=0; i < 20; i++) 	// вывод буфера в терминал
		{
		if (GPSBuf[i] == '\0' || GPSBuf[i] == '\r' || GPSBuf[i] == '\n') break; 	// выводим только до первого 0 в буфере
		USART_SendByte(USART1, GPSBuf[i]);
		}

	if (GPSBuf[0] == '$' && GPSBuf[1] == 'P' && GPSBuf[2] == 'M' && GPSBuf[3] == 'T' && GPSBuf[4] == 'K' && GPSBuf[5] == '0'
			&& GPSBuf[6] == '0' && GPSBuf[7] == '1' && GPSBuf[8] == ',' && GPSBuf[9] == '0' && GPSBuf[10] == ','
					&& GPSBuf[11] == '3' && GPSBuf[12] == '*' && GPSBuf[13] == '3' && GPSBuf[14] == '0' ) // скорость 115200
		printf("\tOK  ");
	else
		printf("\tNO! ");

	//	triggerTIM3Interrupt = 0; 	// сбрасываем флаг наличия данных в буфере
	//}

	ClearGPSBuffer();  				// очистка приемного буфера GPS

    printf("\n\r GPS setup    ...\t");



    rprintfInit ( GpsUsart );  				// переключаем вывод на USART3 - в модуль GPS

    printf("$PMTK300,1000,0,0,0,0*1C\r\n\n"); 	// настройка на период посылок от GPS 1 сек
    //printf("$PMTK300,10000,0,0,0,0*2C"); 	// настройка на период посылок от GPS 10 сек
    DelayMS(50);			// нужна пауза для ответа на предыдущую команду todo добавить проверку ответа

    //printf("$PMTK353,1,0*36\r\n\n"); 	// только GPS
    //printf("$PMTK353,0,1*36\r\n\n"); 	// только GLONASS
    printf("$PMTK353,1,1*37\r\n\n"); 	// GPS и GLONASS
    DelayMS(70);			// нужна пауза для ответа на предыдущую команду todo добавить проверку ответа

    printf("$PMTK314,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,0*28\r\n\n"); // настройки выводимой информации -  GGA, GSA, GSV, ZDA
    // printf("$PMTK314,-1,*04 	// сброс настроек на заводские

    DelayMS(70);			// нужна пауза для ответа на предыдущую команду todo добавить проверку ответа

    rprintfInit ( Terminal );	// вывод в терминал
    printf("OK \r\n");
	SSD1306_Puts("OK ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
    DelayMS(500);


/*    printf("\r\n START test 1PPS: \r\n");
for (;;) // тест измерения периода 1PPS
{
	rprintfInit ( Terminal );	// вывод в терминал

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
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

	SSD1306_GotoXY(0, 0);
	printf ("count: %d ", counter_TIM4_fix);
	SSD1306_GotoXY(0, 10);
	printf ("TIM4 : %d ", TIM4_fix);

	SSD1306_UpdateScreen();

} // STOP HERE*/
//////////////////////////////////////////

// ---------------- BLE -------------------------------
    rprintfInit ( Terminal );	// вывод в терминал
    printf(" BLE setup     ...");
    SSD1306_GotoXY(0, 20);
	SSD1306_Puts("BLE setup...  ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();

    OnResetBLE();	// сброс
    DelayMS(50);
    OffResetBLE(); 	// включение BLE модуля
    DelayMS(500);


    ClearBLEBuffer();		// очистка приемного буфера

	DelayMS(20);
	rprintfInit ( BleUsart ); // вывод в USART BLE модуля

    printf("AT+BAUD8");		// настройка скорости UART интерфейса BLE модуля на 115200. BAUD4-9600
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
    USART2_Configuration115200();  // переключаем скорость USART2 на 115200

    ClearBLEBuffer();		// очистка приемного буфера

    DelayMS(20);
    printf("AT+NAMEtrack%d\r\n\n", NumberDev);		// изменение названия модуля
    DelayMS(100);

    for (i=0; i < BufferUSART2Size; i++)
		{
		if (BufferUSART2RX[i] == '\0') break;
		USART_SendByte(USART1, BufferUSART2RX[i]); // terminal
		}

    SSD1306_Puts("OK ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
    rprintfInit ( Terminal );	// вывод в терминал
    printf(" BLE setup   ...\tOK\r\n");
//STOP
//    if (BufferUSART2RX[0] == 'O' && BufferUSART2RX[1] == 'K')
//    {
//        rprintfInit ( Terminal );	// вывод в терминал
//        printf("OK \r\n");
//    	SSD1306_Puts("OK ", &Font_7x10, SSD1306_COLOR_WHITE);
//    	SSD1306_UpdateScreen();
//        DelayMS(500);
//    }
//    else
//    {
//		rprintfInit ( Terminal );	// вывод в терминал
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
//			if (triggerUSART1Interrupt == 1 ) // пришел байт от терминала
//			{
//				DelayMS(100); // пауза, ждем всю посылку
//
//				for (i=0; i < BufferUSART1Size; i++) // команды от терминала к BLE
//					{
//					if (BufferUSART1RX[i] == '\0') break;
//					USART_SendByte(USART1, BufferUSART1RX[i]); // terminal эхо
//					USART_SendByte(USART2, BufferUSART1RX[i]); // to BLE
//					}
//
//				counterUSART1Buffer = 0;
//				triggerUSART1Interrupt = 0;
//			}
//
//			if ( triggerTIM2Interrupt == 1 ) 	// проверка  - пришла посылка от BLE модуля
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
//				triggerTIM2Interrupt = 0; 	// сбрасываем флаг наличия данных в буфере
//			OffLED0();
//			}
//
//    	}


// --------------------------------------------- LORA -------------------------------
    //OffResetLORA 	();			//  RESET LORA в 1 Включение модуля

    rprintfInit ( Terminal );	// вывод в терминал
    printf(" LORA setup  ...\t");
    SSD1306_GotoXY(0, 30);
	SSD1306_Puts("LORA setup..  ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();

	Radio = RadioDriverInit();  // инициализация структуры , определение функций
	Radio->Init( );				// инициализация радиоинтерфейса, начальные настройки , частота и тп

	SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST ); // включаем выход PABoost
	//SX1276LoRaSetPa20dBm( true );
	SX1276LoRaSetRFFrequency(434000000);	// Частота, Гц
	SX1276LoRaSetRFPower( RFPower );		// Мощность, дБм
	//SX1276LoRaSetSpreadingFactor ( 11 );
	SX1276LoRaSetPreambleLength( 8 ); 		// размер преамбулы 6-65535 8+14 = 12 символов преамбула (как по стандарту LoRA)

//	for (i=0; i<256; i++)					// очистка буфера передачи LORA трансивера
//	{
//		LORAbuffer[i]= 0;
//	}

	SX1276Write ( REG_LR_MODEMCONFIG3, 0b00000100 ); // todo ALC on
	SX1276Write ( REG_LR_LNA, 0b00000000 );

	DelayMS(100);
    rprintfInit ( Terminal );	// вывод в терминал
    printf("OK \r\n");
	SSD1306_Puts("OK ", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
    DelayMS(500);

//OnLED0();
    rprintfInit ( Terminal );	// вывод в терминал
 /*   Radio->Process( );
    Radio->Process( );
    Radio->Process( );
    //printf("RFLRState = %d\n\r", RFLRState);
    printf("SetTxPacket!\n\r");

OnLED0();
Radio->SetTxPacket( LORAbuffer, 255 ); // передача пакета
*/
//for (;;)
//	 Radio->Process( );
//STOP
    for (i = 0; i< 10; i++)  // заполняем массив PER - неопределенные значения
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
				for (i=0; i<256; i++)					// очистка буфера приема LORA трансивера
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

//				SX1276LoRaSetRFPower( Power );				// Мощность, дБм

//				printf("TX#%03d, power = %02d dBm\n\r", countPacket, SX1276LoRaGetRFPower() );
				Radio->SetTxPacket( LORAbuffer, 255 ); 	// передача пакета
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
 	 rprintfInit ( Terminal );	// вывод в терминал
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

		 countOK ++; // счетчик общего количества пакетов

		 if (CRCerror == 1) // ошибка CRC
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

		// FreqError =  ((FREQ_ERROR * (2^24) ) / 32e6 ) * (BW / 500) ; // расчет ошибки по частоте
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
		 if( strncmp( ( const char* )LORAbuffer, ( const char* )LORAbuffer2, 255 ) == 0 ) // сравниваем строки
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

	ClearGPSBuffer();			// очистка приемного буфера GPS
	triggerTIM3Interrupt = 0; 	// сбрасываем флаг наличия данных в буфере
//OffLED1();

	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_DrawRectangle(0, 0, 127, 64, SSD1306_COLOR_WHITE );		// рамка
//OnLED1();
	SSD1306_UpdateScreenDMA();
	counterLcdOff = LcdOffTimeout; // заводим таймер

OffLED0();
}

// ----------- обрабатываем нажатие кнопки POWER OFF / SCREEN
void ScanKeyPowerOff 	(void)
{
		if ( GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 0 )  		// кнопка нажата
		{
			DelayMS(10);											// задержка против дребезга
			if ( GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 0 ) 	// проверка еще раз, действительно было нажатие
			{
				if(trigger1 == 0) 									// триггер для однократного нажатия
				{
					trigger1 = 1;

					if (counterLcdOff == 0) // экран выключен
					{
						counterLcdOff = LcdOffTimeout; // заводим таймер
						// включаем экран
						SSD1306_ON ();
						SSD1306_UpdateScreenDMA();	  // запуск нового обновления экрана из видеобуфера

					}
					else	// экран включен
					{
						counterLcdOff = LcdOffTimeout; // заводим таймер

						screen ++;
						if (screen > 15)
							screen = 1;
					}
				}
				else
				{
					counterPowerOff -- ;	// счетчик задержки выключения питания
					if (counterPowerOff == 0)
					{
						OffVcc1(); 			// выключаем питание
					}

				}


			}
		}
		else 									// кнопка отпущена
		{
			trigger1 = 0; 						// сброс триггера однократного нажатия
			counterPowerOff = PowerOffTimeout; 	// задержка на выключение питания примерно 3 сек
		}
}

// ----------- обрабатываем нажатие кнопки SOS
void ScanKeySOS 		(void)
{
	if ( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0 )  		// кнопка нажата
	{
		DelayMS(10);											// задержка против дребезга
		if ( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0 ) 	// проверка еще раз, действительно было нажатие
		{
			if(trigger2 == 0) 									// триггер для однократного нажатия
			{
				trigger2 = 1;


			}
		}
	}
	else 														// кнопка отпущена
	{
		trigger2 = 0; 											// сброс триггера однократного нажатия
	}

}

// ----------- Получаем выборку АЦП1
u16  GetSamplesADC1		(void) // выборка АЦП
{
	while(ADC_GetFlagStatus( ADC1, ADC_FLAG_EOC) == RESET ); // ждем флаг конца преобразования
	ADC_ClearFlag( ADC1, ADC_FLAG_EOC );		// сбрасываем флаг
	return ( ADC_GetConversionValue(ADC1) );
}


void InitMeasVoltage		(void)
{
	u16 ii;	//счетчик

// костыль, без этого первое измерение adcIntRef не корректное  = 4095
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1, ADC_SampleTime_239Cycles5 ); // подключили ко входу АЦП внутренний источник опорного напряжения 1.2 В
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);								// start ONE conversion
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);				// wait end of conversion
	adcIntRef = ADC_GetConversionValue(ADC1)  ;	// код АЦП, соответствующий напряжению на входе 1.2 В при текущем опорном напряжении АЦП (оно же напряжение питания)

// Измеряем внутреннее опорное напряжение 1.2 В todo настроить измерение группы каналов
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1, ADC_SampleTime_239Cycles5 ); // подключили ко входу АЦП внутренний источник опорного напряжения 1.2 В

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);								// start ONE conversion
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);				// wait end of conversion

	adcIntRef = ADC_GetConversionValue(ADC1)  ;	// код АЦП, соответствующий напряжению на входе 1.2 В при текущем опорном напряжении АЦП (оно же напряжение питания)

// Измеряем напряжение аккумулятора
	OnDivGND1();  // делитель аккумулятора на земле
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 ); // Вход 0 - напряжение аккумулятора через делитель

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);								// start ONE conversion
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);				// wait end of conversion

	adc1 = ADC_GetConversionValue(ADC1);								// get value
	OffDivGND1();  // делитель аккумулятора в воздухе

// вычисление напряжений
	Vacc1 = ( 4030 * 1200 * adc1 / adcIntRef ) / 1000;		// 4030/1000 - с учетом делителя напряжения

	Vref1 = ( 4095 * 1200 / adcIntRef );

	for (ii=0; ii<Average1; ii++) // заполнение массива измеренными значениями
	{
		Vacc[ii] = Vacc1;
		Vref[ii] = Vref1;
	}

	Vref[4] = adcIntRef;
	Vref[5] = adc1 ;
}


// ----------- Измерение напряжений аккумулятора и батареи
void MeasVoltage		(void)
{
	u16 ii;	//счетчик
// Измеряем внутреннее опорное напряжение 1.2 В todo настроить измерение группы каналов
		ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1, ADC_SampleTime_239Cycles5 ); // подключили ко входу АЦП внутренний источник опорного напряжения 1.2 В
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);								// start ONE conversion
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);				// wait end of conversion
		adcIntRef = ADC_GetConversionValue(ADC1)  ;	// код АЦП, соответствующий напряжению на входе 1.2 В при текущем опорном напряжении АЦП (оно же напряжение питания)

// Измеряем напряжение аккумулятора
		OnDivGND1();  // делитель аккумулятора на земле
		ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 ); // Вход 0 - напряжение аккумулятора через делитель
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);								// start ONE conversion
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);				// wait end of conversion
		adc1 = ADC_GetConversionValue(ADC1)  ;								// get value
		OffDivGND1();  // делитель аккумулятора в воздухе

// вычисление напряжений
		Vacc1 = ( 4030 * 1200 * adc1 / adcIntRef ) / 1000;		// 4030/1000 - с учетом делителя напряжения
		Vref1 = ( 4095 * 1200 / adcIntRef );
// усреднение скользящим средним
		for (ii=Average1-1; ii>=1; ii--) // сдвиг окна
		{
			Vacc[ii] = Vacc[ii-1];
			Vref[ii] = Vref[ii-1];
		}
		Vacc[0] = Vacc1;	// текущее значение - в нулевой элемент массива
		Vref[0] = Vref1;

		VaccSumm = 0;
		VrefSumm = 0;
		for (ii=0; ii<Average1; ii++) //накапливаем сумму
		{
			VaccSumm += Vacc[ii];
			VrefSumm += Vref[ii];
		}
		VaccAvg = VaccSumm / Average1; // вычисляем среднее
		VrefAvg = VrefSumm / Average1;

		VrefDev = VrefAvg * 100 / 3300 - 100; // отклонение от номинала в %
		VaccDev = VaccAvg * 100 / 3700 - 100; // todo доработать 2.7V = 0%, 3,7V = 100%

		if ( (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0) && (ChargeTrigger == 0) )  		// начался заряд аккумулятора
		{
			ChargeTrigger = 1;
			OnLED1();				// однократное включение красного светодиода
		}
		if ( (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 1) && (ChargeTrigger == 1) )  		// заряд закончился
		{
			ChargeTrigger = 0;
			OffLED1();				// однократное выключение красного светодиода
		}

//		if ( flag_1PPS_AVG_OK == 1 ) // если спутники видны и время определено
//		{
//			if ( triggerTimeStartLogVcc == 0)
//				{
//				triggerTimeStartLogVcc = 1;
//				TimeStartLogVcc = GPSTimeSec + 1;	// запоминаем время старта записи лога напряжения аккумулятора
//				}
//		}

		// запись в лог
		//TimeLogVcc = SecRTC - TimeStartLogVcc ;  // время, прошедшее с начала записи лога
		TimeLogVcc = timer1s;	// время, прошедшее от включения прибора

		if ( TimeLogVcc != SecRTCold)
		{
			SecRTCold = TimeLogVcc ;

			if ( TimeLogVcc % 600 == 0 ) // время делится на 10 мин без остатка
			{
				VaccLog [counterVaccLog] = (VaccAvg - 3000) / 25;	// пересчет в деления графика ( всего 48 делений по 25 мВ )
				counterVaccLog ++ ;
				if (counterVaccLog > 127)
					counterVaccLog = 127;
			}
		}


}

// Выводим на дисплей графику и текст
void Display1	(void)
{
	u8 X_bat, Y_bat;
	u8 X_gps_ico, Y_gps_ico;
	u8 X_gps_lev, Y_gps_lev;
	u8 X_lora_ico, Y_lora_ico;
	u8 X_lora_lev, Y_lora_lev;

	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

	SSD1306_Fill(SSD1306_COLOR_BLACK);

	// батарейка
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
	//Антенна LORA
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
	Time_Display2( SecRTC ); 		// отображаем время RTC без секунд

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
	printf ("%d", SatFix);					// количество спутников, участвующих в решении
	printf ("/%d", SatNumGPS+SatNumGLO);  	// количество найденных при парсинге спутников

//	SSD1306_GotoXY(72, 20);
//	printf ("P:%d ", countPacket);

	SSD1306_GotoXY(0, 30);
	printf ("Vacc: %d.%03d V ", VaccAvg/1000, VaccAvg%1000);

	SSD1306_GotoXY(96, 30);
	if ( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0 )  		// идет заряд
	{
		printf ("CHRG!");
	}

	SSD1306_GotoXY(0, 40);
	printf ("La: ");
	for (i=0; i<9; i++)									// DDMM.MMM
		{
		if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
			OLED_Putc1 ( GPSFixData.Latitude[i]);
		else
			OLED_Putc1 ('-');
		}
	if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
		printf ("'%c ", GPSFixData.NS );
	else
		printf ("--");

	SSD1306_GotoXY(0, 50);
	printf ("Lo: ");
		if (GPSFixData.Longitude[0] == '0') 			// опускаем лидирующий ноль
		{
			for (i=1; i<10; i++)
				{
				if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
					OLED_Putc1 ( GPSFixData.Longitude[i]);	// DDMM.MMM
				else
					OLED_Putc1 ('-');
				}
		}
		else
		{
			for (i=0; i<10; i++)						// DDDMM.MMM
				{
				if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
					OLED_Putc1 ( GPSFixData.Longitude[i]);
				else
					OLED_Putc1 ('-');
				}
		}
	if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
		printf ("'%c ", GPSFixData.EW );
	else
		printf ("--");

	SSD1306_GotoXY(112, 50);
	printf("#%d", LORA_TX_TIME);

}
// Напряжения аккумулятора, батареи, питания
void Display2	(void)
{
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

	SSD1306_Fill(SSD1306_COLOR_BLACK);

	SSD1306_GotoXY(0, 0);
	printf ("%d.%03d V ", VaccAvg/1000, VaccAvg%1000 ); // усредненное напряжение аккумулятора

//	SSD1306_GotoXY(7, 10);
//	Time_Display( TimeStartLogVcc ); 	// отображаем время
//	SSD1306_GotoXY(60, 10);
//	printf ("%d  ", counterVaccLog );
//	Time_Display( SecRTC ); 			// отображаем время

	SSD1306_GotoXY(60, 0);
	Time_Display( TimeLogVcc ); 		// отображаем время, сколько пишется лог


	SSD1306_DrawLine(0, 15, 0, 63, SSD1306_COLOR_WHITE);	// ось Y - напряжение
	SSD1306_DrawLine(0, 63, 127, 63, SSD1306_COLOR_WHITE);	// ось X - время

	for (i = 0; i < 22; i++) // деления времени , через 1 час, по 6 пикселей, 1 пиксель - 10 мин
	{
		SSD1306_DrawLine(i*6, 60, i*6, 63, SSD1306_COLOR_WHITE);
	}

	for (i = 0; i < 6; i++) // деления напряжения , через 0,2 В, по 8 пикселей, 1 пиксель - 25 мВ
	{
		SSD1306_DrawLine(1, 15 + i * 8, 3, 15 + i * 8, SSD1306_COLOR_WHITE);
	}

	for (i = 1; i<= 127; i++) // рисуем график
	{
		if (VaccLog[i] == 0) // если значение 0, то не рисуем
			break;

		SSD1306_DrawLine(i-1+1, 63 - VaccLog[i-1], i+1, 63 - VaccLog[i], SSD1306_COLOR_WHITE);
	}


	SSD1306_DrawLine(127, 12, 127, 52, SSD1306_COLOR_WHITE);	// ось Y - мгновенное напряжение
	for (i = 0; i < 5; i++) // деления напряжения , через 10 мВ, по 10 пикселей, 1 пиксель - 1 мВ
	{
		SSD1306_DrawLine(125, 12 + i * 10, 126, 12 + i * 10, SSD1306_COLOR_WHITE);
	}

	if (counterVaccosc == 5) // обновление осциллограммы каждые 5 циклов
	{
		counterVaccosc = 0;

		for (i = 99; i > 0; i--)
			VaccOsc [i] = VaccOsc [i-1];

		VaccOsc[0] = Vacc1 - VaccAvg;
	}
	else
		counterVaccosc ++;

	for (i = 1 ; i < 100; i++)	// график мгновенного напряжения
	{
		SSD1306_DrawLine(20 + i-1, 32 - VaccOsc[i-1], 20 + i, 32 - VaccOsc[i], SSD1306_COLOR_WHITE);
	}

/*	if (counterVaccosc == 99) 			// массив усреднения Vacc сдвинулся на 100
	{
		counterVaccosc = 0;
		for (i = 0 ; i < Average1; i++)	// обновляем массив осциллограммы
		{
			VaccOsc [i] = Vacc [i] - VaccAvg;
		}
	}
	else
		counterVaccosc ++;

	for (i = 1 ; i < Average1; i++)	// график мгновенного напряжения
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
//	if ( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0 )  		// идет заряд
//	{
//		printf ("CHRG!");
//	}
//	if ( GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == 1 )  		// USB подключен
//	{ // не работает из-за пересечения с JTAG
//		SSD1306_GotoXY(96, 10);
//		printf ("USB");
//	}

}
// Время RTC и GPS
void Display3 	(void)
{
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей
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
	Time_Display( sec_GPS_to_display );		// выводим на экран HH:MM:SS - время, полученное из секунд GPS текущих суток
	printf (".%d", GPSTimeMilliSec );		// и миллисекунды
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
	printf ("Sat:%d/%d", SatNumGPS, SatFix);  	// количество найденных при парсинге спутников
	SSD1306_GotoXY(80, 50);
	printf ("Fix: %d", Fix);

}
// уровень сигнала со спутников в виде ID S/N dB
void Display4 	(void)
{
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей
	SSD1306_Fill(SSD1306_COLOR_BLACK);

//	SSD1306_GotoXY(105, 0);
//	printf ("%d", SatNum);  	// количество найденных при парсинге спутников
//	SSD1306_GotoXY(105, 10);
//	printf ("%d", SatFix);		// количество спутников, участвующих в решении

	SSD1306_GotoXY(0, 0);
	for (i=0; i<6; i++)		//	входят только первые 6 спутников GPS
	{
	//	if (i<6)
	//	{
			SSD1306_GotoXY(0, 10 * i);

			for (j=0; j<3; j++)
			{
				if ( GPSFixData.Sats[i][0][j] == '\0' )
					break;
				SSD1306_Putc (GPSFixData.Sats[i][0][j], &Font_7x10, SSD1306_COLOR_WHITE ); // номер спутника
			}

			SSD1306_GotoXY(18, 10 * i);

			for (j=0; j<3; j++)
			{
				if ( GPSFixData.Sats[i][3][j] == '\0' )
					break;
				SSD1306_Putc (GPSFixData.Sats[i][3][j], &Font_7x10, SSD1306_COLOR_WHITE ); // SNR спутника
			}
			printf ("dB");
	//	}
	}
	for (i=0; i<6; i++)		//	входят только первые 6 спутников GLONASS
		//else if (i<12) // выводим максимум 12 спутников
		{
			SSD1306_GotoXY(55, 10 * i);

			for (j=0; j<3; j++)
			{
				if ( GPSFixData.SatsGLO[i][0][j] == '\0' )
					break;
				SSD1306_Putc (GPSFixData.SatsGLO[i][0][j], &Font_7x10, SSD1306_COLOR_WHITE );	// номер спутника
			}

			SSD1306_GotoXY(73, 10 * i);

			for (j=0; j<3; j++)
			{
				if ( GPSFixData.SatsGLO[i][3][j] == '\0' )
					break;
				SSD1306_Putc (GPSFixData.SatsGLO[i][3][j], &Font_7x10, SSD1306_COLOR_WHITE );	// SNR спутника
			}
			printf ("dB");
		}
	//	else {
	//		break;
	//	}

	//}

}
// уровень сигнала со спутников в графическом виде
void Display5	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

	for (i=0; i<SatNumGPS; i++) 	// перебираем все видимые спутники GPS
	{
		SatIDGPS [i] = (GPSFixData.Sats[i][0][0] - 0x30) * 10 + (GPSFixData.Sats[i][0][1] - 0x30) * 1 ;

		for (j=0; j<3; j++)
		{
			if ( GPSFixData.Sats[i][3][j] == '\0' ) 	// считаем сколько цифр в числе S/N
				break;
		}

		if (j==3) 		// три цифры
		{
			SatLevelsGPS [i] = (GPSFixData.Sats[i][3][0] - 0x30) * 100 + (GPSFixData.Sats[i][3][1] - 0x30) * 10 + (GPSFixData.Sats[i][3][2] - 0x30) * 1 ;
		}
		else if (j==2) 	// две цифры
		{
			SatLevelsGPS [i] = (GPSFixData.Sats[i][3][0] - 0x30) * 10 + (GPSFixData.Sats[i][3][1] - 0x30) * 1 ;
		}
		else if (j==1) 	// одна цифра
		{
			SatLevelsGPS [i] = GPSFixData.Sats[i][3][0] - 0x30 ;
		}
//printf ("%d %d;", i, j);
	}

	SatSummGPS = 0; 			// сумма уровней сигналов со спутников GPS
	for (i=0; i<SatNumGPS; i++)
	{
		SatSummGPS += SatLevelsGPS[i];
	}

	// ---------------------------- GLONASS спутники -------------------------------------------
	for (i=0; i<SatNumGLO; i++) 	// перебираем все видимые спутники GPS
		{
			SatIDGLO [i] = (GPSFixData.SatsGLO[i][0][0] - 0x30) * 10 + (GPSFixData.SatsGLO[i][0][1] - 0x30) * 1 ;

			for (j=0; j<3; j++)
			{
				if ( GPSFixData.SatsGLO[i][3][j] == '\0' ) 	// считаем сколько цифр в числе S/N
					break;
			}

			if (j==3) 		// три цифры
			{
				SatLevelsGLO [i] = (GPSFixData.SatsGLO[i][3][0] - 0x30) * 100 + (GPSFixData.SatsGLO[i][3][1] - 0x30) * 10 + (GPSFixData.SatsGLO[i][3][2] - 0x30) * 1 ;
			}
			else if (j==2) 	// две цифры
			{
				SatLevelsGLO [i] = (GPSFixData.SatsGLO[i][3][0] - 0x30) * 10 + (GPSFixData.SatsGLO[i][3][1] - 0x30) * 1 ;
			}
			else if (j==1) 	// одна цифра
			{
				SatLevelsGLO [i] = GPSFixData.SatsGLO[i][3][0] - 0x30 ;
			}
	//printf ("%d %d;", i, j);
		}

		SatSummGLO = 0; 			 // сумма уровней сигналов со спутников GLONASS
		for (i=0; i<SatNumGLO; i++)
		{
			SatSummGLO += SatLevelsGLO[i];
		}
	//---------------------------------------------------------------------------------------------

	//rprintfInit (OLED_font1); 	// вывод на OLED дисплей
	SSD1306_GotoXY(0, 0);
	printf ("GPS:%d/%d", SatFixGPS, SatNumGPS );  		// количество найденных при парсинге спутников
	SSD1306_GotoXY(66, 0);
	printf ("GLO:%d/%d", SatFixGLO, SatNumGLO );  	// количество найденных при парсинге спутников

	// считаем сумм использованных спутников GPS и рисуем уровни
	SatSummUsedGPS = 0;
	for (i=0; i<SatNumGPS; i++)
	{
		for (j=0; j<12; j++) // перебираем все спутники из решения, сравниваем номера
		{
			if ( SatUsedGPS [j] == SatIDGPS [i] )
			{
				SatSummUsedGPS += SatLevelsGPS[i];  		// сумма S/N спутников, используемых в решении
				SSD1306_DrawFilledRectangle(0 + i*4, 63-SatLevelsGPS[i], 2, SatLevelsGPS[i], SSD1306_COLOR_WHITE);  // спутники, используемые в решении - сплошным белым
			}
			else
				SSD1306_DrawRectangle(0 + i*4, 63-SatLevelsGPS[i], 2, SatLevelsGPS[i], SSD1306_COLOR_WHITE);		  // спутники, неиспользуемые в решении - пустыми прямоугольниками
		}
	}
	// считаем сумм использованных спутников GLONASS и рисуем уровни
	SatSummUsedGLO = 0;
	for (i=0; i<SatNumGLO; i++)
	{
		for (j=0; j<12; j++) // перебираем все спутники из решения, сравниваем номера
		{
			if ( SatUsedGLO [j] == SatIDGLO [i] )
			{
				SatSummUsedGLO += SatLevelsGLO[i];  		// сумма S/N спутников, используемых в решении
				SSD1306_DrawFilledRectangle(66 + i*4, 63-SatLevelsGLO[i], 2, SatLevelsGLO[i], SSD1306_COLOR_WHITE);  // спутники, используемые в решении - сплошным белым
			}
			else
				SSD1306_DrawRectangle(66 + i*4, 63-SatLevelsGLO[i], 2, SatLevelsGLO[i], SSD1306_COLOR_WHITE);		  // спутники, неиспользуемые в решении - пустыми прямоугольниками
		}
	}

	SSD1306_GotoXY(0, 10);
	printf ("S=%d/%d", SatSummUsedGPS, SatSummGPS);

	SSD1306_GotoXY(66, 10);
	printf ("S=%d/%d", SatSummUsedGLO, SatSummGLO);

	SSD1306_GotoXY(101, 20);
	printf ("%d", SatSummUsedGPS + SatSummUsedGLO);

}
// PDOP, HDOP, VDOP параметры
void Display6	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

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
// координаты и высота
void Display7	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);

	SSD1306_GotoXY(0, 0);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

	printf ("  Coordinates: ");

	rprintfInit (OLED_font2); 	// вывод на OLED дисплей

	SSD1306_GotoXY(0, 10);
	//printf ("La: ");
	for (i=0; i<9; i++)									// DDMM.MMM
		{
			if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
				OLED_Putc2 ( GPSFixData.Latitude[i]);
			else
				OLED_Putc2 ('-');
		}
	if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
		printf ("'%c ", GPSFixData.NS );
	else
		printf ("--");

	SSD1306_GotoXY(0, 27);
	//printf ("Lo: ");
		if (GPSFixData.Longitude[0] == '0') 			// опускаем лидирующий ноль
		{
			for (i=1; i<10; i++)
				{
				if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
					OLED_Putc2 ( GPSFixData.Longitude[i]);	// DDMM.MMM
				else
					OLED_Putc2 ('-');
				}
		}
		else
		{
			for (i=0; i<10; i++)						// DDDMM.MMM
				{
				if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
					OLED_Putc2 ( GPSFixData.Longitude[i]);
				else
					OLED_Putc2 ('-');
				}
		}
	if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
		printf ("'%c ", GPSFixData.EW );
	else
		printf ("--");

	rprintfInit (OLED_font1); 	// вывод на OLED дисплей
	SSD1306_GotoXY(0, 50);
	printf ("Alt: ");					// высота
	SSD1306_GotoXY(30, 44);
	for (i=0; i<5; i++)
		{
		if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
			OLED_Putc2 ( GPSFixData.Altitude[i]);
		else
			OLED_Putc2 ('-');
		}
	printf (" m");

}
/*void Display7	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

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
			if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
				OLED_Putc1 ( GPSFixData.Latitude[i]);
			else
				OLED_Putc1 ('-');
		}
	if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
		printf ("'%c ", GPSFixData.NS );
	else
		printf ("--");

	SSD1306_GotoXY(0, 40);
	printf ("Lo: ");
		if (GPSFixData.Longitude[0] == '0') 			// опускаем лидирующий ноль
		{
			for (i=1; i<10; i++)
				{
				if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
					OLED_Putc1 ( GPSFixData.Longitude[i]);	// DDMM.MMM
				else
					OLED_Putc1 ('-');
				}
		}
		else
		{
			for (i=0; i<10; i++)						// DDDMM.MMM
				{
				if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
					OLED_Putc1 ( GPSFixData.Longitude[i]);
				else
					OLED_Putc1 ('-');
				}
		}
	if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
		printf ("'%c ", GPSFixData.EW );
	else
		printf ("--");

	SSD1306_GotoXY(0, 50);
	printf ("Alt: ");					// высота
	for (i=0; i<5; i++)
		{
		if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
			OLED_Putc1 ( GPSFixData.Altitude[i]);
		else
			OLED_Putc1 ('-');
		}

}*/
// измерение длительности 1PPS
void Display8	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

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
// приемник LORA
void Display9	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

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
// принятые координаты
void Display10	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

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
// принятые времена посылок
void Display11	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

	SSD1306_GotoXY(0, 0);
	printf (" Times: ");
	SSD1306_GotoXY(64, 0);
	Time_Display( SecRTC ); 		// отображаем время RTC

	rprintfInit (OLED_font2); 		// вывод на OLED дисплей
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
// параметры принятых послылок
void Display12	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

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
// PER от каждого маяка
void Display13	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

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
// тригонометрия
void Display14	(void)
{
	u32 temp;

	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

	SSD1306_GotoXY(0, 0);
	printf ("    Distance: ");

	rprintfInit (OLED_font2); 	// вывод на OLED дисплей
	SSD1306_GotoXY(0, 10);
	temp = distance1_1_2 * 10;
	printf ("1-2:%d.%d m", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
	SSD1306_GotoXY(0, 27);
	temp = distance1_1_3 * 10;
	printf ("1-3:%d.%d m", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
	SSD1306_GotoXY(0, 44);
	temp = distance1_2_3 * 10;
	printf ("2-3:%d.%d m", temp / 10,  temp % 10 ); // расстояние до маяка в метрах

}
// уровни сигнала от маяков в графическом виде
void Display15	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

	SSD1306_GotoXY(0, 0);
	printf ("      RSSI:    dBm");


	for (i=1; i<6; i++) // маяки с 1 по 5
	{
		SSD1306_GotoXY(0, 10*i);
		printf ("[%d]", i);

		if (LoraRX [i].RSSI != 0) 	// есть какое-то измеренное значение
		{
			if ( LoraRX [i].RSSI >= -140) // если уровень больше -140, рисуем
				{
					if ( LoraRX [i].RSSI > -55 )
						SSD1306_DrawFilledRectangle( 20, 10*i+2, 85, 5, SSD1306_COLOR_WHITE );  // максимум, ограничение
					else
						SSD1306_DrawFilledRectangle( 20, 10*i+2, 85 + (55 + LoraRX [i].RSSI) , 5, SSD1306_COLOR_WHITE );  // уровень RSSI ( от -55 до -140 дБм )
				}

			if ( LoraRX [i].RSSI <= -100) // если 3 цифры в числе
				SSD1306_GotoXY(106-7, 10*i);
			else
				SSD1306_GotoXY(113-7, 10*i); // если 2 цифры в числе

			printf ("%d", LoraRX [i].RSSI);
		}

	}

}

// основной экран
void Display1Release	(void)
{
	u32 temp;
	u32 temp2;
	u8 X_bat, Y_bat;
	u16 VaccAvgTemp;

	SSD1306_Fill(SSD1306_COLOR_BLACK);
	temp2 = sec_div % 10 ;

	// батарейка
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

		if (LoraRX [2].RSSI != 0) 	// есть какое-то измеренное значение
			{
				if ( LoraRX [2].RSSI >= -140) // если уровень больше -140, рисуем
					{
						if ( LoraRX [2].RSSI > -55 )
							SSD1306_DrawFilledRectangle( 24, 21, 85, 5, SSD1306_COLOR_WHITE );  // максимум, ограничение
						else
						{
							SSD1306_DrawRectangle( 24, 21, 85, 5, SSD1306_COLOR_WHITE );
							SSD1306_DrawFilledRectangle( 24, 21, 85 + (55 + LoraRX [2].RSSI) , 5, SSD1306_COLOR_WHITE );  // уровень RSSI ( от -55 до -140 дБм )
						}
					}

				if ( LoraRX [2].RSSI <= -100) // если 3 цифры в числе
					SSD1306_GotoXY(18, 11);
				else
					SSD1306_GotoXY(25, 11); // если 2 цифры в числе

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
			SSD1306_GotoXY(80, 11); // мощность передатчика
			printf("P%d", LoraRX [2].Power);
			SSD1306_GotoXY(113, 11+10); // счетчик пакетов
			printf("%02X", LoraRX [2].CounterTxP);
		}
		rprintfInit (OLED_font1);

		SSD1306_GotoXY(25, 0);
		if (distance1_1_2 < 100000)
			{
			temp = distance1_1_2 * 10;
			printf ("%d.%dm", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
			}
		else
			{
			printf ("---.-m" ); // расстояние не определено
			}

		SSD1306_GotoXY(71, 0);
		Time_Display (TimesOld[2]);

		// батарейка
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

		if (LoraRX [3].RSSI != 0) 	// есть какое-то измеренное значение
			{
				if ( LoraRX [3].RSSI >= -140) // если уровень больше -140, рисуем
					{
						if ( LoraRX [3].RSSI > -55 )
							SSD1306_DrawFilledRectangle( 24, 31+21, 85, 5, SSD1306_COLOR_WHITE );  // максимум, ограничение
						else
						{
							SSD1306_DrawRectangle( 24, 31+21, 85, 5, SSD1306_COLOR_WHITE );
							SSD1306_DrawFilledRectangle( 24, 31+21, 85 + (55 + LoraRX [3].RSSI) , 5, SSD1306_COLOR_WHITE );  // уровень RSSI ( от -55 до -140 дБм )
						}
					}

				if ( LoraRX [3].RSSI <= -100) // если 3 цифры в числе
					SSD1306_GotoXY(18, 31+11);
				else
					SSD1306_GotoXY(25, 31+11); // если 2 цифры в числе

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
			SSD1306_GotoXY(80, 31+11); // мощность передатчика
			printf("P%d", LoraRX [3].Power);
			SSD1306_GotoXY(113, 31+11+10); // счетчик пакетов
			printf("%02X", LoraRX [3].CounterTxP);
		}

		rprintfInit (OLED_font1);

		SSD1306_GotoXY(25, 31+0);
		if (distance1_1_3 < 100000)
			{
			temp = distance1_1_3 * 10;
			printf ("%d.%dm", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
			}
		else
			{
			printf ("---.-m" ); // расстояние не определено
			}
		SSD1306_GotoXY(71, 31+0);
		Time_Display (TimesOld[3]);

		// батарейка
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

		if (LoraRX [1].RSSI != 0) 	// есть какое-то измеренное значение
			{
				if ( LoraRX [1].RSSI >= -140) // если уровень больше -140, рисуем
					{
						if ( LoraRX [1].RSSI > -55 )
							SSD1306_DrawFilledRectangle( 24, 21, 85, 5, SSD1306_COLOR_WHITE );  // максимум, ограничение
						else
						{
							SSD1306_DrawRectangle( 24, 21, 85, 5, SSD1306_COLOR_WHITE );
							SSD1306_DrawFilledRectangle( 24, 21, 85 + (55 + LoraRX [1].RSSI) , 5, SSD1306_COLOR_WHITE );  // уровень RSSI ( от -55 до -140 дБм )
						}
					}

				if ( LoraRX [1].RSSI <= -100) // если 3 цифры в числе
					SSD1306_GotoXY(18, 11);
				else
					SSD1306_GotoXY(25, 11); // если 2 цифры в числе

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
			SSD1306_GotoXY(80, 11); // мощность передатчика
			printf("P%d", LoraRX [1].Power);
			SSD1306_GotoXY(113, 11+10); // счетчик пакетов
			printf("%02X", LoraRX [1].CounterTxP);
		}

		rprintfInit (OLED_font1);

		SSD1306_GotoXY(25, 0);
		if (distance1_1_2 < 100000)
			{
			temp = distance1_1_2 * 10;
			printf ("%d.%dm", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
			}
		else
			{
			printf ("---.-m" ); // расстояние не определено
			}

		SSD1306_GotoXY(71, 0);
		Time_Display (TimesOld[1]);

		// батарейка
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

		if (LoraRX [3].RSSI != 0) 	// есть какое-то измеренное значение
			{
				if ( LoraRX [3].RSSI >= -140) // если уровень больше -140, рисуем
					{
						if ( LoraRX [3].RSSI > -55 )
							SSD1306_DrawFilledRectangle( 24, 31+21, 85, 5, SSD1306_COLOR_WHITE );  // максимум, ограничение
						else
						{
							SSD1306_DrawRectangle( 24, 31+21, 85, 5, SSD1306_COLOR_WHITE );
							SSD1306_DrawFilledRectangle( 24, 31+21, 85 + (55 + LoraRX [3].RSSI) , 5, SSD1306_COLOR_WHITE );  // уровень RSSI ( от -55 до -140 дБм )
						}
					}

				if ( LoraRX [3].RSSI <= -100) // если 3 цифры в числе
					SSD1306_GotoXY(18, 31+11);
				else
					SSD1306_GotoXY(25, 31+11); // если 2 цифры в числе

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
			SSD1306_GotoXY(80, 31+11); // мощность передатчика
			printf("P%d", LoraRX [3].Power);
			SSD1306_GotoXY(113, 31+11+10); // счетчик пакетов
			printf("%02X", LoraRX [3].CounterTxP);
		}

		rprintfInit (OLED_font1);

		SSD1306_GotoXY(25, 31+0);
		if (distance1_2_3 < 100000)
			{
			temp = distance1_2_3 * 10;
			printf ("%d.%dm", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
			}
		else
			{
			printf ("---.-m" ); // расстояние не определено
			}
		SSD1306_GotoXY(71, 31+0);
		Time_Display (TimesOld[3]);

		// батарейка
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

		if (LoraRX [1].RSSI != 0) 	// есть какое-то измеренное значение
			{
				if ( LoraRX [1].RSSI >= -140) // если уровень больше -140, рисуем
					{
						if ( LoraRX [1].RSSI > -55 )
							SSD1306_DrawFilledRectangle( 24, 21, 85, 5, SSD1306_COLOR_WHITE );  // максимум, ограничение
						else
						{
							SSD1306_DrawRectangle( 24, 21, 85, 5, SSD1306_COLOR_WHITE );
							SSD1306_DrawFilledRectangle( 24, 21, 85 + (55 + LoraRX [1].RSSI) , 5, SSD1306_COLOR_WHITE );  // уровень RSSI ( от -55 до -140 дБм )
						}
					}

				if ( LoraRX [1].RSSI <= -100) // если 3 цифры в числе
					SSD1306_GotoXY(18, 11);
				else
					SSD1306_GotoXY(25, 11); // если 2 цифры в числе

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
			SSD1306_GotoXY(80, 11); // мощность передатчика
			printf("P%d", LoraRX [1].Power);
			SSD1306_GotoXY(113, 11+10); // счетчик пакетов
			printf("%02X", LoraRX [1].CounterTxP);
		}
		rprintfInit (OLED_font1);

		SSD1306_GotoXY(25, 0);
		if (distance1_1_3 < 100000)
			{
			temp = distance1_1_3 * 10;
			printf ("%d.%dm", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
			}
		else
			{
			printf ("---.-m" ); // расстояние не определено
			}

		SSD1306_GotoXY(71, 0);
		Time_Display (TimesOld[1]);

		// батарейка
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

		if (LoraRX [2].RSSI != 0) 	// есть какое-то измеренное значение
			{
				if ( LoraRX [2].RSSI >= -140) // если уровень больше -140, рисуем
					{
						if ( LoraRX [2].RSSI > -55 )
							SSD1306_DrawFilledRectangle( 24, 31+21, 85, 5, SSD1306_COLOR_WHITE );  // максимум, ограничение
						else
						{
							SSD1306_DrawRectangle( 24, 31+21, 85, 5, SSD1306_COLOR_WHITE );
							SSD1306_DrawFilledRectangle( 24, 31+21, 85 + (55 + LoraRX [2].RSSI) , 5, SSD1306_COLOR_WHITE );  // уровень RSSI ( от -55 до -140 дБм )
						}
					}

				if ( LoraRX [2].RSSI <= -100) // если 3 цифры в числе
					SSD1306_GotoXY(18, 31+11);
				else
					SSD1306_GotoXY(25, 31+11); // если 2 цифры в числе

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
			SSD1306_GotoXY(80, 31+11); // мощность передатчика
			printf("P%d", LoraRX [2].Power);
			SSD1306_GotoXY(113, 31+11+10); // счетчик пакетов
			printf("%02X", LoraRX [2].CounterTxP);
		}

		rprintfInit (OLED_font1);

		SSD1306_GotoXY(25, 31+0);
		if (distance1_2_3 < 100000)
			{
			temp = distance1_2_3 * 10;
			printf ("%d.%dm", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
			}
		else
			{
			printf ("---.-m" ); // расстояние не определено
			}
		SSD1306_GotoXY(71, 31+0);
		Time_Display (TimesOld[2]);

		// батарейка
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

// расстояния между маяками
void Display2Release	(void)
{
	u32 temp;

	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

	SSD1306_GotoXY(0, 0);
	printf ("    Distance: ");

	rprintfInit (OLED_font2); 	// вывод на OLED дисплей
	SSD1306_GotoXY(0, 10);
	temp = distance1_1_2 * 10;
	printf ("1-2:%d.%d m", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
	SSD1306_GotoXY(0, 27);
	temp = distance1_1_3 * 10;
	printf ("1-3:%d.%d m", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
	SSD1306_GotoXY(0, 44);
	temp = distance1_2_3 * 10;
	printf ("2-3:%d.%d m", temp / 10,  temp % 10 ); // расстояние до маяка в метрах

}

// информация о спутниках
void Display3Release	(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

	for (i=0; i<SatNumGPS; i++) 	// перебираем все видимые спутники GPS
	{
		SatIDGPS [i] = (GPSFixData.Sats[i][0][0] - 0x30) * 10 + (GPSFixData.Sats[i][0][1] - 0x30) * 1 ;

		for (j=0; j<3; j++)
		{
			if ( GPSFixData.Sats[i][3][j] == '\0' ) 	// считаем сколько цифр в числе S/N
				break;
		}

		if (j==3) 		// три цифры
		{
			SatLevelsGPS [i] = (GPSFixData.Sats[i][3][0] - 0x30) * 100 + (GPSFixData.Sats[i][3][1] - 0x30) * 10 + (GPSFixData.Sats[i][3][2] - 0x30) * 1 ;
		}
		else if (j==2) 	// две цифры
		{
			SatLevelsGPS [i] = (GPSFixData.Sats[i][3][0] - 0x30) * 10 + (GPSFixData.Sats[i][3][1] - 0x30) * 1 ;
		}
		else if (j==1) 	// одна цифра
		{
			SatLevelsGPS [i] = GPSFixData.Sats[i][3][0] - 0x30 ;
		}
//printf ("%d %d;", i, j);
	}

	SatSummGPS = 0; 			// сумма уровней сигналов со спутников GPS
	for (i=0; i<SatNumGPS; i++)
	{
		SatSummGPS += SatLevelsGPS[i];
	}

	// ---------------------------- GLONASS спутники -------------------------------------------
	for (i=0; i<SatNumGLO; i++) 	// перебираем все видимые спутники GPS
		{
			SatIDGLO [i] = (GPSFixData.SatsGLO[i][0][0] - 0x30) * 10 + (GPSFixData.SatsGLO[i][0][1] - 0x30) * 1 ;

			for (j=0; j<3; j++)
			{
				if ( GPSFixData.SatsGLO[i][3][j] == '\0' ) 	// считаем сколько цифр в числе S/N
					break;
			}

			if (j==3) 		// три цифры
			{
				SatLevelsGLO [i] = (GPSFixData.SatsGLO[i][3][0] - 0x30) * 100 + (GPSFixData.SatsGLO[i][3][1] - 0x30) * 10 + (GPSFixData.SatsGLO[i][3][2] - 0x30) * 1 ;
			}
			else if (j==2) 	// две цифры
			{
				SatLevelsGLO [i] = (GPSFixData.SatsGLO[i][3][0] - 0x30) * 10 + (GPSFixData.SatsGLO[i][3][1] - 0x30) * 1 ;
			}
			else if (j==1) 	// одна цифра
			{
				SatLevelsGLO [i] = GPSFixData.SatsGLO[i][3][0] - 0x30 ;
			}
	//printf ("%d %d;", i, j);
		}

		SatSummGLO = 0; 			 // сумма уровней сигналов со спутников GLONASS
		for (i=0; i<SatNumGLO; i++)
		{
			SatSummGLO += SatLevelsGLO[i];
		}
	//---------------------------------------------------------------------------------------------

	//rprintfInit (OLED_font1); 	// вывод на OLED дисплей
	SSD1306_GotoXY(0, 0);
	printf ("GPS:%d/%d", SatFixGPS, SatNumGPS );  		// количество найденных при парсинге спутников
	SSD1306_GotoXY(66, 0);
	printf ("GLO:%d/%d", SatFixGLO, SatNumGLO );  	// количество найденных при парсинге спутников

	// считаем сумм использованных спутников GPS и рисуем уровни
	SatSummUsedGPS = 0;
	for (i=0; i<SatNumGPS; i++)
	{
		for (j=0; j<12; j++) // перебираем все спутники из решения, сравниваем номера
		{
			if ( SatUsedGPS [j] == SatIDGPS [i] )
			{
				SatSummUsedGPS += SatLevelsGPS[i];  		// сумма S/N спутников, используемых в решении
				SSD1306_DrawFilledRectangle(0 + i*4, 63-SatLevelsGPS[i], 2, SatLevelsGPS[i], SSD1306_COLOR_WHITE);  // спутники, используемые в решении - сплошным белым
			}
			else
				SSD1306_DrawRectangle(0 + i*4, 63-SatLevelsGPS[i], 2, SatLevelsGPS[i], SSD1306_COLOR_WHITE);		  // спутники, неиспользуемые в решении - пустыми прямоугольниками
		}
	}
	// считаем сумм использованных спутников GLONASS и рисуем уровни
	SatSummUsedGLO = 0;
	for (i=0; i<SatNumGLO; i++)
	{
		for (j=0; j<12; j++) // перебираем все спутники из решения, сравниваем номера
		{
			if ( SatUsedGLO [j] == SatIDGLO [i] )
			{
				SatSummUsedGLO += SatLevelsGLO[i];  		// сумма S/N спутников, используемых в решении
				SSD1306_DrawFilledRectangle(66 + i*4, 63-SatLevelsGLO[i], 2, SatLevelsGLO[i], SSD1306_COLOR_WHITE);  // спутники, используемые в решении - сплошным белым
			}
			else
				SSD1306_DrawRectangle(66 + i*4, 63-SatLevelsGLO[i], 2, SatLevelsGLO[i], SSD1306_COLOR_WHITE);		  // спутники, неиспользуемые в решении - пустыми прямоугольниками
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
	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

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


	if (counterMaxRssiReset > 0) // другой маяк виден
	{
		SSD1306_GotoXY(10, 0);
		rprintfInit (OLED_font2); 	// вывод на OLED дисплей

		if ( structBuffer[1].Latitude == 0 || structBuffer[2].Latitude == 0 ) // если координата одного из маяков не определена
			printf ("--- m" );
		else
		{
			temp = distance1 * 10;
			printf ("%d.%d m", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
		}

		SSD1306_GotoXY(10, 20);
		rprintfInit (OLED_font1); 	// вывод на OLED дисплей

		if ( structBuffer[1].Latitude == 0 || structBuffer[2].Latitude == 0 ) // если координата одного из маяков не определена
			printf ("--- m" );
		else
		{
			temp = distance2 * 10;
			printf ("%d.%d m", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
		}

	}
	else // маяк не виден , показываем расстояние от текущего местоположения до последней принятой точки
	{
		SSD1306_GotoXY(10, 0);
		rprintfInit (OLED_font1); 	// вывод на OLED дисплей

		if ( structBuffer[1].Latitude == 0 || structBuffer[2].Latitude == 0 ) // если координата одного из маяков не определена
			printf ("--- m" );
		else
		{
			temp = distance1 * 10;
			printf ("%d.%d m", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
		}

		SSD1306_GotoXY(10, 15);
		rprintfInit (OLED_font2); 	// вывод на OLED дисплей

		if ( structBuffer[1].Latitude == 0 || structBuffer[2].Latitude == 0 ) // если координата одного из маяков не определена
			printf ("--- m" );
		else
		{
			temp = distance2 * 10;
			printf ("%d.%d m", temp / 10,  temp % 10 ); // расстояние до маяка в метрах
		}
	}

	rprintfInit (OLED_font1); 	// вывод на OLED дисплей

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
	{ // градусник усреднения 1PPS
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

void OLED_Putc1	(char c)  // вывод символа на дисплей. Шрифт 7x10 пикселей
{
	SSD1306_Putc ( c, &Font_7x10, SSD1306_COLOR_WHITE );
}
void OLED_Putc2	(char c)  // вывод символа на дисплей. Шрифт 11x18 пикселей
{
	SSD1306_Putc ( c, &Font_11x18, SSD1306_COLOR_WHITE );
}
void OLED_Putc3	(char c)  // вывод символа на дисплей. Шрифт 16x26 пикселей
{
	SSD1306_Putc ( c, &Font_16x26, SSD1306_COLOR_WHITE );
}

// -----------------  прием и парсинг данных с GPS модуля
void ReceiveGPS (void)
{
	u8 scan;	// счетчик для поиска GSV последовательностей
	
	if ( triggerTIM3Interrupt == 1 ) 	// проверка  - пришла посылка от GPS модуля
	{
		triggerTIM3Interrupt = 0; 		// сбрасываем флаг наличия данных в буфере

    	countPacket ++ ;


		rprintfInit (Terminal);
	#ifdef DebugGpsToUart1
		printf("\n\r\n\r ------- NEW DATA ----------------------------------\n\r");
	#endif
		// Если буфер заполнился, начинаем искать и декодировать NMEA сентенции
	//printf ("\n\r GPSBufWrPoint: %d ", GPSBufWrPoint); 	// текущее положение указателя записи в буфер
		GPSBufWrPoint = 0; 				// сброс в 0, записываем следующую посылку с начала буфера

		NMEA_Parser_GGA();						// GGA - основная сентенция, выдает координаты
		NMEA_Parser_GSA_GPS();					// GSA - список GPS спутников в решении, DOP

		NMEA_Parser_GSA_GLO();					// GSA - список GLONASS спутников в решении

		NMEA_Parser_ZDA();      				// ZDA - выдает время и дату
		GPSTimeSec = GetGPSTimeSec(); 			// парсинг времени, секунды текущих суток
		GPSTimeMilliSec	= GetGPSTimeMilliSec(); // парсинг времени, миллисекунды

		GSVStartPos = 0;
		SatNumGPS = 0;

	//	printf("\n\r ------- GPS --------------------------------------");
		NMEA_Parser_GSV_GPS();	// парсим первую строку, смотрим общее количество GSV-GPS строк
		for (scan = 0; scan < GSVNumStringGPS-1; scan ++ )
			{
				NMEA_Parser_GSV_GPS();	// повторяем парсинг
			}

		GSVStartPos = 0;
		SatNumGLO = 0;

	//	printf("\n\r\n\r ------- Glonass ----------------------------------");
		NMEA_Parser_GSV_GLO ();	// парсим первую строку, смотрим общее количество GSV-GLONASS строк
		for (scan = 0; scan < GSVNumStringGLO-1; scan ++ )
			{
				NMEA_Parser_GSV_GLO();	// повторяем парсинг
			}

	//  текущие координаты и высота из строки в u32
		if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
		{
			GPSLongitude = (GPSFixData.Longitude[0]-0x30)*100000000 + (GPSFixData.Longitude[1]-0x30)*10000000 + (GPSFixData.Longitude[2]-0x30)*1000000 +
					(GPSFixData.Longitude[3]-0x30)*100000 + (GPSFixData.Longitude[4]-0x30)*10000 + (GPSFixData.Longitude[6]-0x30)*1000 +
					(GPSFixData.Longitude[7]-0x30)*100 + (GPSFixData.Longitude[8]-0x30)*10  + (GPSFixData.Longitude[9]-0x30)*1 ;

			GPSLatitude = (GPSFixData.Latitude[0]-0x30)*10000000 + (GPSFixData.Latitude[1]-0x30)*1000000 + (GPSFixData.Latitude[2]-0x30)*100000 +
					(GPSFixData.Latitude[3]-0x30)*10000 + (GPSFixData.Latitude[5]-0x30)*1000 + (GPSFixData.Latitude[6]-0x30)*100 +
					(GPSFixData.Latitude[7]-0x30)*10 + (GPSFixData.Latitude[8]-0x30)*1 ;

			if (GPSFixData.NS == 'N')			GPS_N_S = 1 ; 	// признак северная-южная широта
				else							GPS_N_S = 0 ;

			if (GPSFixData.EW == 'E')			GPS_E_W = 0 ; 	// западная-восточная долгота
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
			j++;		// счетчик цифр
			}
		if (j == 1) 								// одна цифра
			SatFix = GPSFixData.SatelliteNum[0] - 0x30;
		else if (j == 2)							// две цифры
			SatFix = (GPSFixData.SatelliteNum[0] - 0x30) * 10 + (GPSFixData.SatelliteNum[1] - 0x30);			// из ASCII кода в цифры
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
		//	printf ("\n\r %d\tSAT# %c%c\tElev:%c%c°\tAzim:%c%c%c\t SNR:%c%c dB",i+1, GPSFixData.Sats[i][0][0], GPSFixData.Sats[i][0][1], GPSFixData.Sats[i][1][0], GPSFixData.Sats[i][1][1],
		//							GPSFixData.Sats[i][2][0], GPSFixData.Sats[i][2][1], GPSFixData.Sats[i][2][2], GPSFixData.Sats[i][3][0], GPSFixData.Sats[i][3][1]   );
	//	}

		if ( (Fix == 2 || Fix == 3) ) 	// если есть 2D или 3D решение
		{
			if (triggerTTFF == 0)		// однократное измерение TTFF
			{
				triggerTTFF = 1;
				TTFF = countPacket;
			}
		}

		for (i=0; i<GPSBUFLENGTH; i++) // очистка буфера
			{
			GPSBuf[i] = '\0';
			}
	}

}

//  ------------------------ MNEA PARCER ---- GPS GSA ------------------------------------
u8	NMEA_Parser_GSA_GPS	(void) // Парсер DOP GPS
{
  u8 CommaPos = 0;
  u8 n = 0;
  u16 i, StartPos = 0, StartDOP=0;    // Начало строки

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
  // начало пакета слишком сильно смещено, отбрасываем пакет
  if (StartPos > GPSBUFLENGTH - sizeof("$GNGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.1,2.2,3.3*cs")-1) // начало слишком далеко, посылка уже не войдет до конца
	  {
//printf ("\n\rGSA offset ERROR!");
	  //return 0;
	  }

  if (ControlCheckSum(StartPos) == 0) // проверка контрольной суммы CRC
 	  {
//printf ("\n\r GSA CRC ERROR!"); // todo
 	  //return 0; // Проверка контрольной суммы todo
 	  }
  else
   	  {
//printf ("\n\r GSA CRC OK"); // todo
 	  }

  if (GPSBuf[StartPos+7] == 'A')
	  AutoSel_2D3D = 1;	// автоматический режим выбора 2D/3D
  else  if (GPSBuf[StartPos+7] == 'M')
	  AutoSel_2D3D = 0;	// ручной режим выбора 2D/3D
  else
	  AutoSel_2D3D = 2; // не действительное значение

  if (GPSBuf[StartPos+9] == '1')
	  Fix = 1;			// нет решения
  else if (GPSBuf[StartPos+9] == '2')
	  Fix = 2;			// 2D - не определена высота
  else if (GPSBuf[StartPos+9] == '3')
	  Fix = 3;			// 3D - высота и координаты определены
  else
	  Fix = 0;			// не действительное значение

  for (i=0; i<12; i++)  // очистка массива номеров используемых спутников
  {
	  SatUsedGPS [i] = 0;
  }

//printf ("\n\r");
//for (i = StartPos+10; i < GPSBUFLENGTH ; i++)   // Ждем конца буфера
//{
//	if (GPSBuf[i] == '$') break;
//	USART_SendByte(USART1, GPSBuf[i]);
//}

  for (i = StartPos+10; i < GPSBUFLENGTH ; i++)   // Ждем или конца буфера или 12-й запятой
    {
	  if ( (GPSBuf[i] == ',') && (GPSBuf[i+1] == '0') && (GPSBuf[i+2] == ',')  ) 	// поле без значений
      {
    	i+=1 ; 			// пропускаем ноль между запятыми
        CommaPos ++; 	// Наращиваем счетчик запятых
//printf ("\n\r ,0, i: %d C:%d ",i, CommaPos); // todo
      }
      else
      {
    	SatUsedGPS [n] = (GPSBuf[i+1]-0x30)*10 + (GPSBuf[i+2]-0x30); 	// номер спутника

    	CommaPos ++; 	// Наращиваем счетчик запятых
    	n++ ;			// счетчик спутников
    	i+=2 ; 			// переход на следующее значение
//printf ("\n\r i: %d C:%d ",i, CommaPos); // todo
      }
	  SatFixGPS = n; 	// количество спутников GPS, найденных при парсинге

      if (CommaPos >= 12) // достигли 12-й запятой
      {
//printf ("\n\r break (C>=12)");
      	break;
      }
    }
  StartDOP = i+1; // положение запятой, после которой начинаются DOP параметры

//printf ("\n\r Used SATs: ");
//for (i=0; i<12; i++)  // очистка массива номеров используемых спутников
//{
//  printf ("%d ", SatUsed [i] ); 	// todo
//}

//printf ("\n\rStartDOP:%d ", StartDOP); 	// todo
//printf ("\n\r");
//for (i = StartDOP; i < GPSBUFLENGTH ; i++)   // Вывод буфера в терминал
//{
//	if (GPSBuf[i] == '$') break;
//	USART_SendByte(USART1, GPSBuf[i]);
//}

  	i = 0; // сброс счетчика

	if ( (GPSBuf[StartDOP+i] == ',') && (GPSBuf[StartDOP+i+1] == '0') && (GPSBuf[StartDOP+i+2] == ',')  ) 	// поле без значений
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

	if ( (GPSBuf[StartDOP+i] == ',') && (GPSBuf[StartDOP+i+1] == '0') && (GPSBuf[StartDOP+i+2] == ',')  ) 	// поле без значений
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

	if ( (GPSBuf[StartDOP+i] == ',') && (GPSBuf[StartDOP+i+1] == '0') && (GPSBuf[StartDOP+i+2] == '*')  ) 	// поле без значений
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
u8	NMEA_Parser_GSA_GLO	(void) // Парсер DOP GPS
{
  u8 CommaPos = 0;
  u8 n = 0;
  u16 i, StartPos = 0, StartDOP=0;    // Начало строки

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
  // начало пакета слишком сильно смещено, отбрасываем пакет
  if (StartPos > GPSBUFLENGTH - sizeof("$GNGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.1,2.2,3.3*cs")-1) // начало слишком далеко, посылка уже не войдет до конца
	  {
//printf ("\n\rGSA offset ERROR!");
	  //return 0;
	  }

  if (ControlCheckSum(StartPos) == 0) // проверка контрольной суммы CRC
 	  {
//printf ("\n\r GSA CRC ERROR!"); // todo
 	  //return 0; // Проверка контрольной суммы todo
 	  }
  else
   	  {
//printf ("\n\r GSA CRC OK"); // todo
 	  }

 /* if (GPSBuf[StartPos+7] == 'A')
	  AutoSel_2D3D = 1;	// автоматический режим выбора 2D/3D
  else  if (GPSBuf[StartPos+7] == 'M')
	  AutoSel_2D3D = 0;	// ручной режим выбора 2D/3D
  else
	  AutoSel_2D3D = 2; // не действительное значение

  if (GPSBuf[StartPos+9] == '1')
	  Fix = 1;			// нет решения
  else if (GPSBuf[StartPos+9] == '2')
	  Fix = 2;			// 2D - не определена высота
  else if (GPSBuf[StartPos+9] == '3')
	  Fix = 3;			// 3D - высота и координаты определены
  else
	  Fix = 0;			// не действительное значение
*/

  for (i=0; i<12; i++)  // очистка массива номеров используемых спутников
  {
	  SatUsedGLO [i] = 0;
  }

//printf ("\n\r");
//for (i = StartPos+10; i < GPSBUFLENGTH ; i++)   // Ждем конца буфера
//{
//	if (GPSBuf[i] == '$') break;
//	USART_SendByte(USART1, GPSBuf[i]);
//}

  for (i = StartPos+10; i < GPSBUFLENGTH ; i++)   // Ждем или конца буфера или 12-й запятой
    {
	  if ( (GPSBuf[i] == ',') && (GPSBuf[i+1] == '0') && (GPSBuf[i+2] == ',')  ) 	// поле без значений
      {
    	i+=1 ; 			// пропускаем ноль между запятыми
        CommaPos ++; 	// Наращиваем счетчик запятых
//printf ("\n\r ,0, i: %d C:%d ",i, CommaPos); // todo
      }
      else
      {
    	SatUsedGLO [n] = (GPSBuf[i+1]-0x30)*10 + (GPSBuf[i+2]-0x30); 	// номер спутника

    	CommaPos ++; 	// Наращиваем счетчик запятых
    	n++ ;			// счетчик спутников
    	i+=2 ; 			// переход на следующее значение
//printf ("\n\r i: %d C:%d ",i, CommaPos); // todo
      }
	  SatFixGLO = n; 	// количество спутников GLONASS, найденных при парсинге

      if (CommaPos >= 12) // достигли 12-й запятой
      {
//printf ("\n\r break (C>=12)");
      	break;
      }
    }
  StartDOP = i+1; // положение запятой, после которой начинаются DOP параметры

//printf ("\n\r Used SATs: ");
//for (i=0; i<12; i++)  // очистка массива номеров используемых спутников
//{
//  printf ("%d ", SatUsed [i] ); 	// todo
//}

//printf ("\n\rStartDOP:%d ", StartDOP); 	// todo
//printf ("\n\r");
//for (i = StartDOP; i < GPSBUFLENGTH ; i++)   // Вывод буфера в терминал
//{
//	if (GPSBuf[i] == '$') break;
//	USART_SendByte(USART1, GPSBuf[i]);
//}

  	i = 0; // сброс счетчика

	if ( (GPSBuf[StartDOP+i] == ',') && (GPSBuf[StartDOP+i+1] == '0') && (GPSBuf[StartDOP+i+2] == ',')  ) 	// поле без значений
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

	if ( (GPSBuf[StartDOP+i] == ',') && (GPSBuf[StartDOP+i+1] == '0') && (GPSBuf[StartDOP+i+2] == ',')  ) 	// поле без значений
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

	if ( (GPSBuf[StartDOP+i] == ',') && (GPSBuf[StartDOP+i+1] == '0') && (GPSBuf[StartDOP+i+2] == '*')  ) 	// поле без значений
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
  u16 i,StartPos = 0;    // Начало строки

  while (!( (GPSBuf[StartPos] == '$')
        &&(GPSBuf[StartPos+3] == 'G')
        &&(GPSBuf[StartPos+4] == 'G')
        &&(GPSBuf[StartPos+5] == 'A'))
        && StartPos < 7)
    StartPos++;
//printf ("\n\r GGA Parser: "); // todo
//printf ("\n\r StartPos = %d", StartPos); // todo
  // начало пакета слишком сильно смещено, отбрасываем пакет
  if (StartPos>5)
	  {
	 // printf ("\n\rStartPos>5\n\r");
	  return 0;
	  }

  if (ControlCheckSum(StartPos) == 0)
	  {
//printf ("\n\r GGA CRC ERROR!"); // todo
	  //return 0; // Проверка контрольной суммы todo
	  }
  else
  	  {
//printf ("\n\r GGA CRC OK"); // todo
	  }

  memset(&GPSFixData, 0, sizeof(GPSFixData));   // Очищаем структуру

  for (i = StartPos+6; i < GPSBUFLENGTH ; i++)   // Ждем или конца буфера или 10-й запятой
  {
    if (GPSBuf[i] == ',')
    {
      CommaPos++; // Наращиваем счетчик запятых
      k=0;
    }
    else
      switch (CommaPos) // Пользуемся тем, что данные разделяются запятыми. Неопределенные данные (две запятые подряд) мы заменили нулем в обработчике прерывания USART1_IRQHandler()
      {
        // После первой запятой идет время, но мы возьмем его позже из ZDA сентенции, синхронно с датой
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
  u8 k;              		// Просто счетчик
  u16 i,StartPos = 0;    	// Начало строки

  while (!( (GPSBuf[StartPos] == '$')
        &&(GPSBuf[StartPos+3] == 'Z')
        &&(GPSBuf[StartPos+4] == 'D')
        &&(GPSBuf[StartPos+5] == 'A'))
        && StartPos < (GPSBUFLENGTH - sizeof("$GNZDA,hhmmss.ss,dd,mm,yyyy,+hh,mm*cs")+1))
    StartPos++;
//printf ("\n\r ZGA Parser: "); // todo
//printf ("\n\r StartPos = %d", StartPos); // todo
  // начало пакета слишком сильно смещено, отбрасываем этот пакет
  if (StartPos > (GPSBUFLENGTH - sizeof("$GNZDA,hhmmss.ss,dd,mm,yyyy,+hh,mm*cs")-1))
	  {
//printf ("\n\r ZDA offset ERROR!"); // todo
	  return 0;
	  }

  if (ControlCheckSum(StartPos) == 0) // Проверка контрольной суммы
  {
//printf ("\n\r ZDA CRC ERROR!"); // todo
  return 0; // Проверка контрольной суммы
  }
//printf ("\n\r ZDA CRC OK"); // todo

  for (i = StartPos+5; i < GPSBUFLENGTH ; i++)   // Ждем или конца буфера или 7-й запятой
  {
    if (GPSBuf[i] == ',')
    {
      CommaPos++; // Наращиваем счетчик запятых
      k=0;
    }
    else
      switch (CommaPos) // Пользуемся тем, что данные разделяются запятыми. Неопределенные данные (две запятые подряд) мы заменили нулем в обработчике прерывания USART1_IRQHandler()
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

u32 GetGPSTimeSec 	(void) 		// вызывать после парсинга ZDA сентенции
{
	u32 THH = 0, TMM = 0, TSS = 0 ;
	THH = (GPSFixData.Time[0]-0x30)*10 + (GPSFixData.Time[1]-0x30);		// часы из ZDA сентенции
	TMM = (GPSFixData.Time[2]-0x30)*10 + (GPSFixData.Time[3]-0x30);		// минуты из ZDA сентенции
	TSS = (GPSFixData.Time[4]-0x30)*10 + (GPSFixData.Time[5]-0x30);		// секунды из ZDA сентенции
	//TmSmS = (GPSFixData.Time[7]-0x30)*10 + (GPSFixData.Time[8]-0x30);	// дробная часть секунд из ZDA сентенции

	return((THH*3600 + TMM*60 + TSS)); // возвращаем целое число секунд за текущие сутки, без дробной части
}
u32 GetGPSTimeMilliSec 	(void) 	// вызывать после парсинга ZDA сентенции
{
	u32  TmSmS;
	TmSmS = (GPSFixData.Time[7]-0x30)*10 + (GPSFixData.Time[8]-0x30);	// дробная часть секунд из ZDA сентенции
	return( TmSmS ) ;   // возвращаем дробную часть
}

u8  NMEA_Parser_GSV_GPS	(void)
{
  u8 CommaPos = 0;
  u8 k = 0;              // Просто счетчик
  u16 i = 0;

  while (!( (GPSBuf[GSVStartPos] == '$')
		&&(GPSBuf[GSVStartPos+2] == 'P') // только GPS спутники
        &&(GPSBuf[GSVStartPos+3] == 'G')
        &&(GPSBuf[GSVStartPos+4] == 'S')
        &&(GPSBuf[GSVStartPos+5] == 'V'))
        && GSVStartPos < (GPSBUFLENGTH - sizeof("$GPGSV,n,n,nn,11,22,333,44,11,22,333,44,11,22,333,44,11,22,333,44*cs")+1))
    GSVStartPos++;  // определяем начало очередной GSV строки в посылке

  // начало пакета слишком сильно смещено, отбрасываем его
  if (GSVStartPos > (GPSBUFLENGTH - sizeof("$GPGSV,n,n,nn,11,22,333,44,11,22,333,44,11,22,333,44,11,22,333,44*cs")-1))
	  {
//printf ("\n\r return 0 offset so big "); // todo
	  return 0;
	  }
	  
  GSVNumStringGPS = GPSBuf[GSVStartPos+7] - 0x30 ; // количество GSV-GPS строк в посылке

  if (ControlCheckSum(GSVStartPos) == 0)
	  {
//printf ("\t\tGSV CRC error! "); // todo
	 // return 0;  // Проверка контрольной суммы
	  }

  for (i = GSVStartPos+5; i < GPSBUFLENGTH ; i++)    // Ждем или конца буфера или * в приемном буфере
  {
    if (GPSBuf[i] == '*')
    {
      GSVStartPos = i;		// сдвинули начало за текущую строку
      SatNumGPS++;				// Наращиваем номер текущего спутника
      //printf ("\n\rGPSSatNum(*): \t%d \tstart: %d", SatNum, GSVStartPos); // todo
      return 1;
    }

    if (GPSBuf[i] == ',')
    {
      if ((CommaPos == 7) || (CommaPos == 11) || (CommaPos == 15) || (CommaPos == 19))
	  {
	    SatNumGPS++;  	// Наращиваем номер текущего спутника
//printf ("\n\rGPSSatNum(,): \t%d \tstart: %d", SatNum, GSVStartPos); // todo
	  }
      CommaPos++; k=0;  	// Наращиваем счетчик запятых
    }
    else
      switch (CommaPos) // Пользуемся тем, что данные разделяются запятыми. Неопределенные данные (две запятые подряд) мы заменили нулем в обработчике прерывания USART1_IRQHandler()
      { // После первых трех запятых идет информация об общем количестве спутников, об общем количестве GSV посылок и о номере этой конкретной GSV посылки. Эта информация нам не нужна
        case 4: case 8: case 12: case 16:  if(k < sizeof(ONESAT)-1) GPSFixData.Sats[SatNumGPS][0][k++]=GPSBuf[i]; break;  // Номер спутника. case 4: case 8: case 12: case 16: - номера запятых,
//		после которых идет соответствующая информация. В каждой GSV посылке есть информация о максимум 4-х спутниках, поэтому 20-ю запятую можно не обрабатывать
        case 5: case 9: case 13: case 17:  if(k < sizeof(ONESAT)-1) GPSFixData.Sats[SatNumGPS][1][k++]=GPSBuf[i]; break;  // Угол места
        case 6: case 10: case 14: case 18: if(k < sizeof(ONESAT)-1) GPSFixData.Sats[SatNumGPS][2][k++]=GPSBuf[i]; break;  // Азимут
        case 7: case 11: case 15: case 19: if(k < sizeof(ONESAT)-1) GPSFixData.Sats[SatNumGPS][3][k++]=GPSBuf[i]; break;  // Уровень сигнала, SNR
        default: break;
      }
  }
  return 1;
}

u8  NMEA_Parser_GSV_GLO (void)
{
  u8 CommaPos = 0;
  u8 k = 0;              // Просто счетчик
  u16 i = 0;

  while (!( (GPSBuf[GSVStartPos] == '$')
		&&(GPSBuf[GSVStartPos+2] == 'L') 	// только GLONASS спутники
        &&(GPSBuf[GSVStartPos+3] == 'G')
        &&(GPSBuf[GSVStartPos+4] == 'S')
        &&(GPSBuf[GSVStartPos+5] == 'V'))
        && GSVStartPos < (GPSBUFLENGTH - sizeof("$GPGSV,n,n,nn,11,22,333,44,11,22,333,44,11,22,333,44,11,22,333,44*cs")+1))
    GSVStartPos++;

  // начало пакета слишком сильно смещено, отбрасываем его
  if (GSVStartPos > (GPSBUFLENGTH - sizeof("$GPGSV,n,n,nn,11,22,333,44,11,22,333,44,11,22,333,44,11,22,333,44*cs")-1))
	  {
//printf ("\n\r return 0 offset so big "); // todo
	  return 0;
	  }

  GSVNumStringGLO = GPSBuf[GSVStartPos+7] - 0x30 ; // количество GSV-GLONASS строк в посылке

  if (ControlCheckSum(GSVStartPos) == 0)
	  {
//printf ("\t\tGSV CRC error! "); // todo
	 // return 0;  // Проверка контрольной суммы
	  }

  for (i = GSVStartPos+5; i < GPSBUFLENGTH ; i++)    // Ждем или конца буфера или * в приемном буфере
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
    	  	  SatNumGLO ++;  // Наращиваем номер текущего спутника
    	  	  //printf ("\n\rGLOSatNum(,): \t%d \tstart: %d ", SatNumGLO, GSVStartPos); // todo
    	  }
      CommaPos++; k=0;  // Наращиваем счетчик запятых
    }
    else
      switch (CommaPos) // Пользуемся тем, что данные разделяются запятыми. Неопределенные данные (две запятые подряд) мы заменили нулем в обработчике прерывания USART1_IRQHandler()
      { // После первых трех запятых идет информация об общем количестве спутников, об общем количестве GSV посылок и о номере этой конкретной GSV посылки. Эта информация нам не нужна
        case 4: case 8: case 12: case 16:  if(k < sizeof(ONESAT)-1) GPSFixData.SatsGLO[SatNumGLO][0][k++]=GPSBuf[i]; break;  // Номер спутника. case 4: case 8: case 12: case 16: - номера запятых, после которых идет соответствующая информация. В каждой GSV посылке есть информация о максимум 4-х спутниках, поэтому 20-ю запятую можно не обрабатывать
        case 5: case 9: case 13: case 17:  if(k < sizeof(ONESAT)-1) GPSFixData.SatsGLO[SatNumGLO][1][k++]=GPSBuf[i]; break;  // Угол места
        case 6: case 10: case 14: case 18: if(k < sizeof(ONESAT)-1) GPSFixData.SatsGLO[SatNumGLO][2][k++]=GPSBuf[i]; break;  // Азимут
        case 7: case 11: case 15: case 19: if(k < sizeof(ONESAT)-1) GPSFixData.SatsGLO[SatNumGLO][3][k++]=GPSBuf[i]; break;  // Уровень сигнала, SNR
        default: break;
      }
  }
  return 1;
}

u8  ControlCheckSum 	(u16 StartIndex)
{
  u8  CheckSum = 0, MessageCheckSum = 0;   // Контрольная сумма
  u16 i = StartIndex+1;                	// Смещаемся на один шаг вправо от символа $

  while (GPSBuf[i]!='*')
  {
    CheckSum^=GPSBuf[i];
    if (++i == GPSBUFLENGTH) return 0; 			// Не найден признак контрольной суммы
  }

  if (GPSBuf[++i]>0x40) MessageCheckSum=(GPSBuf[i]-0x37)<<4; 	// Условие для корректной обработки десятичных
  else                  MessageCheckSum=(GPSBuf[i]-0x30)<<4;  	// и шестнадцатиричных чисел, представленных в коде ASCII
  if (GPSBuf[++i]>0x40) MessageCheckSum+=(GPSBuf[i]-0x37);
  else                  MessageCheckSum+=(GPSBuf[i]-0x30);

  if (MessageCheckSum != CheckSum) return 0;	// Неправильное значение контрольной суммы

  return 1; 	// Все ОК
}

//-------------------- измерение, усреднение и проверка периода сигнала 1PPS -------------------
void Measurement1PPS	(void)
{
	if ( flag_1PPS_Update == 1 )	// обновилось значение
	{
		flag_1PPS_Update = 0;  // сброс флага для однократного срабатывания усреднения

		if ( (TIM4_fix >= 37000) && (TIM4_fix <= 37400) && (counter_TIM4_fix >= 90) && (counter_TIM4_fix <= 92)  )
			{ // период 1PPS в пределах нормы

				for (i_avg = AVG_TIM4-1; i_avg > 0; i_avg -- )	// сдвиг данных в массиве вправо
				{
					averageTIM4 [i_avg] = averageTIM4 [i_avg-1] ;
				}
				averageTIM4 [0] = TIM4_fix; // текущее значение - в 0 ячейку массива

				summ = 0;
				for (i_avg = 0; i_avg < AVG_TIM4; i_avg ++ ) // сумма элементов массива
				{
					summ = summ + averageTIM4 [i_avg] ;
				}
				averageTIM4_temp = summ / AVG_TIM4 ;		// получили среднее

				flag_1PPS_fail = 0;

				counter_AVG_1PPS ++;
				if ((counter_AVG_1PPS >= AVG_TIM4+2) )//&& (flag_sync_stop == 0) ) 			// количество усреднений равно длине массива +2
				{
					counter_AVG_1PPS = AVG_TIM4+2;
					averageTIM4_ready = averageTIM4_temp; 	// получили среднее значение после 32 усреднений
					flag_1PPS_AVG_OK = 1; 					// флаг готовности среднего значения 1PPS
				}
//					if (counter_AVG_1PPS >= AVG_TIM4+5) 		// количество усреднений равно длине массива +5
//					{
//						counter_AVG_1PPS = AVG_TIM4+5;
//						flag_sync_stop = 1; 					// флаг для теста синхронизации
//					}

			}
			else // сбой при измерении периода 1PPS
			{
				flag_1PPS_fail = 1;
				counter_TIM4_fix = 0;
				//flag_1PPS_AVG_OK = 0;
				counter_AVG_1PPS = 0;
			}


#ifdef	Debug1PPSMeasurementToUART
		rprintfInit (Terminal); // инициализация printf (). Вывод в терминал
		for (i=0; i<10; i++)
		{
			printf("%d\t", averageTIM4[i] );
		}
		printf("\tc: %d\tTemp: %d\tReady: %d\tfail: %d\tOK: %d\tSt: %d\r\n", counter_AVG_1PPS, averageTIM4_temp, averageTIM4_ready, flag_1PPS_fail, flag_1PPS_AVG_OK, flag_sync_stop );
#endif
	}	
	
}

// --------------------- усреднение измеренных значени	 сигнала 1PPS  --------------------------
void Average1PPS		(void)
{
//--------------------------------- импульс 1PPS -----------------------------------------
	   if (flag_1PPS_pulse == 1) 	// пришел импульс 1PPS от таймера или от внешнего прерывания
	   {
//OnLED0();
		   flag_1PPS_pulse = 0;

			if (flag_1PPS_AVG_OK == 0)		// если после сброса среднее значение периода 1PPS еще ни разу не получено
			   	{
				OnLED0();				//
				counterLed0 = 500; 		// зажигаем синий светодиод на 500 мс
			   	}
			else
				{

				}

			sec_GPS_to_display = GPSTimeSec + 1 ; 	// секунды от GPS для отображения, сдвиг на одну
			sec_RTC_to_display = SecRTC ;			// секунды от таймера 1 для отображения

			sec_div = SecRTC % 60;		// определяем номер текущей секунды, отбрасываем целое количество минут
			if ( sec_div == 0+LoraTxTime || sec_div == 10+LoraTxTime || sec_div == 20+LoraTxTime || sec_div == 30+LoraTxTime || sec_div == 40+LoraTxTime || sec_div == 50+LoraTxTime ) // проверка времени для передачи
			{
				flagTX = 1;			// передача по радиоканалу разрешена
				//flagTXDisplay = 1;	// флаг для отображения
			}
		//	if ( sec_div == 0+LoraTxTime+1 || sec_div == 10+LoraTxTime+1 || sec_div == 20+LoraTxTime+1 || sec_div == 30+LoraTxTime+1 || sec_div == 40+LoraTxTime+1 || sec_div == 50+LoraTxTime+1 ) // проверка времени для передачи
		//	{
		//		flagTXDisplay = 0;	// гасим флаг для отображения на следующей секунде todo переделать на таймер sysclk
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
 	 	 	 	 counterTXDisplay = 0;		// TX больше не рисуем и гасим Красный светодиод
 	 	 	 	 Radio -> StartRx();		// включение трансивера на прием

			#ifdef DebugLORAToUART
				rprintfInit (Terminal); // инициализация printf (). Вывод в терминал
				printf(" RF_TX_DONE StartRX \n\r");
			#endif
				 break;

			 case RF_RX_TIMEOUT:
			#ifdef DebugLORAToUART
				rprintfInit (Terminal); // инициализация printf (). Вывод в терминал
				printf(" RF_RX_TIMEOUT \n\r");
			#endif	 
				 break;

			 case RF_RX_DONE: //---------------------------------------------------------------- принят пакет -------------------------------------------------------

				// counterRXDisplay = 500;	// рисуем RX 500 мс
				 countOK ++; // счетчик общего количества пакетов


				 if (CRCerror == 1) // ошибка CRC
				 {
					CRCerror = 0;  // обработали ошибку, флаг сброшен

					//OnLED0();				// Ложный прием
					//counterLed0 = 100; 		// зажигаем синий светодиод на 100 мс

				#ifdef DebugLORAToUART 
					 rprintfInit (Terminal); // инициализация printf (). Вывод в терминал
					 printf("\n\r\n\r CRC ER! Noise received " );
				#endif
					 countERR ++; // количество ложных приемов
				 }
				 else   // пакет принят и нет ошибки CRC
				 {
					Radio -> GetRxPacket( LORAbufferRX, (u16*)&BufferSize ); // копируем данные в буфер приема LORAbufferRX

					OnLED0();				// Принят правильный пакет
					counterLed0 = 500; 		// зажигаем синий светодиод на 500 мс
					counterRXDisplay = 500;	// рисуем RX 500 мс

					flagRX_OK = 1; // флаг готовности новых данных в буфере

					FREQ_ERROR =( (LoraReadReg(0x28) << 28) + (LoraReadReg(0x29) << 20) + (LoraReadReg(0x2A) << 12) ) ;

					// FreqError =  ((FREQ_ERROR * (2^24) ) / 32e6 ) * (BW / 500) ; // расчет ошибки по частоте
					FreqError = (FREQ_ERROR/4096) / 3.815; // значение в Гц


				#ifdef DebugLORAToUART
					 rprintfInit (Terminal); // инициализация printf (). Вывод в терминал
					 printf("\n\r\n\r RSSI:%d dBm ", (s32)(SX1276LoRaGetPacketRssi()) );
					// printf("\n\r Preamble:  %d\n\r",  SX1276LoRaGetPreambleLength () );
					// printf(" Gain: %d ", SX1276GetPacketRxGain() );
					 printf(" SNR:%d dB ", SX1276GetPacketSnr() );

					// printf(" Preamble: %d ",  SX1276LoRaGetPreambleLength () );
					// printf(" NbTrigPeaks: %d \n\r",  SX1276LoRaGetNbTrigPeaks () );

					 printf(" PER:%02d.%02d%% ", PER1, PER2 );
					 printf(" Offset:%d Hz", (s32)FreqError );
					// printf(" err=%d summ=%d ", countERR, countOK );

				/*            if (CRCerror == 1) // ошибка CRC
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
						if ( i == 23 || i == 47 || i == 71 || i == 95 || i == 119 || i == 143 || i == 71 || i == 167 || i == 191 || i == 215 ) // 10 строк по 24 байта
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

// запись в FIFO буфер size байт
void LoraWriteFIFO (u8 *buffer, u8 size)
{
	LoraWriteBuffer (0, buffer, size);
}

// чтение из FIFO буфера size байт
void LoraReadFIFO (u8 *buffer, u8 size)
{
	LoraReadBuffer (0, buffer, size);
}

// BURST запись size регистров сразу начиная с адреса addr
void LoraWriteBuffer (u8 Addr, u8 *buffer, u8 size)
{
	u8 i;
	OnNssLORA();

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ждем пока освободится буфер передачи
	SPI_I2S_SendData(SPI1, Addr | 0x80);
	for( i = 0; i < size; i++ )
		{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ждем пока освободится буфер передачи
		SPI_I2S_SendData(SPI1, buffer [i]);
		}
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);

	OffNssLORA();
}

// BURST чтение size регистров сразу начиная с адреса addr
void LoraReadBuffer (u8 Addr, u8 *buffer, u8 size)
{
	u8 i;
	OnNssLORA();

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 		// ждем пока освободится буфер передачи
	SPI_I2S_SendData(SPI1, Addr & 0x7F);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 		// ждем пока байт прочитается целиком
	SPI_I2S_ReceiveData (SPI1); 											// читаем байт для сброса флага чтения

	for( i = 0; i < size; i++ )
		{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 	// ждем пока освободится буфер передачи
		SPI_I2S_SendData(SPI1, 0x00);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 	// ждем пока байт прочитается целиком
		buffer [i] = SPI_I2S_ReceiveData (SPI1);
		}

	OffNssLORA();
}

// запись регистра
void LoraWriteReg	(u8 Addr, u8 Reg)
{
	OnNssLORA();

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ждем пока освободится буфер передачи
	SPI_I2S_SendData(SPI1, Addr | 0x80);
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY) == SET); 	// todo переделать правильно!
	SPI_I2S_SendData(SPI1, Reg);
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY) == SET);

	OffNssLORA();
}
// чтение регистра
u8   LoraReadReg	(u8 Addr)
{
	OnNssLORA();

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ждем пока освободится буфер передачи
	SPI_I2S_SendData(SPI1, Addr);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);  	// todo переделать правильно!
	SPI_I2S_ReceiveData (SPI1);										// для очистки флага SPI_I2S_FLAG_RXNE
	SPI_I2S_SendData(SPI1, 0x00);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); // ждем пока байт прочитается целиком

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

    while (index < 2)  // ждем ввода двух байт (цифр) из терминала в USART1 .
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
    if (index > value) // число должно быть не более value
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
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);// ждем пока освободится буфер передачи
	USART_SendData ( USARTx, Data); //
}
////////////// очистка приемного буфера GPS ///////////////////////////////////////////////////////////////////////
void ClearGPSBuffer			(void)
{
	for (i=0; i<GPSBUFLENGTH; i++) 	// очистка буфера
		{
		GPSBuf[i] = '\0'; 			// заполняем нулями
		}
	GPSBufWrPoint = 0; 				// сброс в 0, записываем следующую посылку с начала буфера
}
////////////// очистка приемного буфера BLE ///////////////////////////////////////////////////////////////////////
void ClearBLEBuffer			(void)
{
	for (i=0; i<BufferUSART2Size; i++) 	// очистка буфера
		{
		BufferUSART2RX[i] = '\0'; 			// заполняем нулями
		}
	counterUSART2Buffer = 0; 				// сброс в 0, записываем следующую посылку с начала буфера
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
void OnVcc1 	(void) // Включить питание
{
	  GPIO_InitTypeDef        GPIO_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ; //     Vcc1ON   ВЫХОД
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_SetBits(GPIOB, GPIO_Pin_13);  // на выходе 1
}
void OffVcc1 	(void) // Выключить питание
{
	  GPIO_InitTypeDef        GPIO_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ; //     Vcc1ON   вход, третье состояние
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIOB, GPIO_Pin_13);
}

void NRESET_on	(void) // 1
{ GPIO_SetBits(GPIOB, GPIO_Pin_2); }
void NRESET_off	(void) // 0
{ GPIO_ResetBits(GPIOB, GPIO_Pin_2); }

void OnLED0 	(void) // Включить светодиод LED0  ------ СИНИЙ
{ GPIO_SetBits(GPIOA, GPIO_Pin_11); } // LED0 ВКЛ
void OffLED0 	(void) // Включить светодиод LED0
{ GPIO_ResetBits(GPIOA, GPIO_Pin_11); } // LED0 ВКЛ
void CplLED0	(void) // переключить светодиод
{	GPIOA->ODR ^= GPIO_Pin_11; }

void OnLED1 	(void) // Включить светодиод LED1  -------- КРАСНЫЙ ---
{ GPIO_SetBits(GPIOA, GPIO_Pin_12); } // LED0 ВКЛ
void OffLED1 	(void) // Включить светодиод LED1
{ GPIO_ResetBits(GPIOA, GPIO_Pin_12); } // LED1 ВКЛ
void CplLED1	(void) // переключить светодиод
{	GPIOA->ODR ^= GPIO_Pin_12; }

void OnTest 	(void) // Test = 1	он же КРАСНЫЙ светодиод
{ GPIO_SetBits(GPIOA, GPIO_Pin_12); } //
void OffTest 	(void) // Test = 0
{ GPIO_ResetBits(GPIOA, GPIO_Pin_12); } //
void CplTest	(void)	// Переключить Test
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
void OnDivGND1 	(void) // Подключить делитель напряжения к земле
{
	  GPIO_InitTypeDef        GPIO_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ; 	//   ВЫХОД
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIOB, GPIO_Pin_15);  			// на выходе 0
}
void OffDivGND1	(void) // Выключить делитель
{
	  GPIO_InitTypeDef        GPIO_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ; 	 //  вход, третье состояние
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_SetBits(GPIOB, GPIO_Pin_15);
}
void OnDivGND2 	(void) // Подключить делитель напряжения к земле
{
	  GPIO_InitTypeDef        GPIO_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ; 	//   ВЫХОД
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIOB, GPIO_Pin_14);  			// на выходе 0
}
void OffDivGND2	(void) // Выключить делитель
{
	  GPIO_InitTypeDef        GPIO_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ; 	 //    вход, третье состояние
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_SetBits(GPIOB, GPIO_Pin_14);
}


// блокирующая задержка в мс на SysTick
void DelayMS				( u16 delay)
{
    timer1ms = 0;
    while (timer1ms < delay) {} // ждем delay мс
}

// для таймаутов Lora
u32  GET_TICK_COUNT ( void )
{
	return ( timer1ms );
}

/* --------------- Настрока портов контроллера ---------------------------
 *
 */
void PortIO_Configuration 	(void) // настройка портов
{
	GPIO_InitTypeDef        GPIO_InitStructure;

  /* Disable the Serial Wire Jtag Debug Port SWJ-DP */
 // GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);  // не отключаем JTAG
 //  PA.13 (JTMS/SWDAT), PA.14 (JTCK/SWCLK) and PA.15 (JTDI)

//  ВРЕМЕННО test pin на PA12 на USB интерфейсе todo
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ; 			// Выход
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA, GPIO_Pin_12);
//  ВРЕМЕННО синий светодиод на PA11 на USB интерфейсе  todo
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 			// Выход
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA, GPIO_Pin_11);

// настройка портов. Вход сигнала USB_ON
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 			// вход сигнала USB_ON
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		// todo заменить, пересекается с JTAG
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
// настройка портов. Вход сигнала STAT от микросхемы заряда аккумулятора
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ; 			// вход сигнала STAT
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		// низкий уровень - идет заряд
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

// настройка портов. Вход сигнала 1PPS  PB12
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ; 			// вход сигнала Timemark
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

// настройка портов. Вход кнопки SOS  PA15
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ; 			// Кнопка SOS вход, подтяжка вверх
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

// настройка портов.Кнопка питания PB8
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ; 			// Кнопка выкл/экран   вход, подтяжка вверх
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		// выключение питания длительным удержанием кнопки, переключение экранов - коротким
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

// Входы АЦП для измерения напряжения аккумулятора и батарейки  PA0 и PA1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;   // Аналоговый вход
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

// настройка портов. Управление делителями напряжения для АЦП
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15 ; // вход, третье состояние
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 	// выключение делителя третьим состоянием, включение - выход в 0
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

// настройка портов. Управление питанием, включение-выключение стабилизаторов
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ; 				// Vcc1ON   вход, третье состояние
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 	// выключение питания третьим состоянием, включение - выход в 1
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_13);

// ------------------ HM10 BLE модуль на CC2540 --- подключен к USART2 -------------------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ; 				// BRESET   выход
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_4); 						// СБРОС


//------------------- SIM33ELA----------------- подключен к USART3 ------------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 				// NRESET   выход
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_2); 						// СБРОС

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ; 				// TIMEMARK вход. Генерирует прерывание
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_12);

//	------------- LORA модуль ----------------------------------
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ; 				// RESET
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_0); 						// RESET в 0 - модуль в сбросе

// -------------настройка SPI1 --------------------------------
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5  | GPIO_Pin_7; // SCK и MOSI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//GPIO_SetBits(GPIOB, GPIO_Pin_5); // не работает в alternate function push-pull
	//GPIO_SetBits(GPIOB, GPIO_Pin_7);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 				// MISO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 			// вход с подтяжкой вверх
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ; 				// NSS, программный сигнал CS для SIM33ELA
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ; 				// DIO0 вход с подтяжкой вниз
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; 				// DIO5 вход с подтяжкой вниз
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
// инициализация DMA для i2c  OLED дисплея
	DMA_InitTypeDef  I2CDMA_InitStructure;

	DMA_DeInit(DMA1_Channel6);  // канал DMA1_Channel6 ?
	I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)0x40005410;  // адрес i2c1 контроллера
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

// Настройка АЦП1
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

	ADC_TempSensorVrefintCmd(ENABLE); // включили встроенный датчик температуры и опору
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

// Настройка SPI1
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

// Настройка USART1
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

// Настройка USART2 BLE
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

// Настройка USART2 BLE 115200 бод
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

// Настройка USART3 GPS 9600 бод
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

// Настройка USART3 GPS 115200 бод
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

// Настройка прерываний и размещение таблицы векторов прерываний
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

    NVIC_SetPriority(SysTick_IRQn, 0x03); 		// приоритет прерывания от системного таймера

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
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn ; 				// прерывание от внешнего сигнала 1PPS на PB12 todo
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

// Настройка Таймера1. Таймер формирует сигнал, заменяющий 1PPS при его отсутствии
void TIM1_Configuration		(void) //
{
	TIM_TimeBaseInitTypeDef base_timer;

	TIM_TimeBaseStructInit(&base_timer);
/* Делитель учитывается как TIM_Prescaler + 1, поэтому отнимаем 1 */
	base_timer.TIM_Prescaler = 12-1; 	// шина ABP1 36 Mhz, но APB1TIM = 72. Prescaler делит на 12 , тактовая частота таймера 6 МГц
	base_timer.TIM_Period = 0xFFFF;		// ARR - Auto Reload Register. До какого значения считать вверх до переполнения			//
	base_timer.TIM_CounterMode = TIM_CounterMode_Up; 	// считать вверх
	TIM_TimeBaseInit(TIM1, &base_timer);
	//TIM4->CR1 |= TIM_CR1_OPM; 						// режим одного импульса

/* Очищаем бит  прерывания */
	//TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update); 	// также работает TIM_FLAG_Update
  // Разрешаем прерывание по обновлению (в данном случае - по переполнению) счётчика таймера TIM1.

	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  // Разрешаем обработку прерывания по переполнению счётчика таймера TIM1.

	//TIM_Cmd(TIM1, ENABLE);  			// запуск таймера
	NVIC_EnableIRQ(TIM1_UP_IRQn);	 	// прерывание из-за переполнения таймера

}

// Настройка Таймера2 Basic Таймер запускается по приему байта по USART2 от BLE модуля
void TIM2_Configuration		(void)
{
	TIM_TimeBaseInitTypeDef base_timer;

	TIM_TimeBaseStructInit(&base_timer);
/* Делитель учитывается как TIM_Prescaler + 1, поэтому отнимаем 1 */
	base_timer.TIM_Prescaler = 36000 - 1; 			// шина ABP1 36 Mhz, но APB1TIM = 72
	base_timer.TIM_Period = 30*2-1; 				// задержка в мс  (ms*2-1)
	base_timer.TIM_CounterMode = TIM_CounterMode_Up;
	base_timer.TIM_RepetitionCounter = 0;
	base_timer.TIM_ClockDivision = TIM_CKD_DIV1; 	// делитель внешней частоты
	TIM_TimeBaseInit(TIM2, &base_timer);
	TIM2->CR1 |= TIM_CR1_OPM; 		// режим одного импульса

/* Очищаем бит  прерывания */
    TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update); // также работает TIM_FLAG_Update

  // Разрешаем прерывание по обновлению (в данном случае - по переполнению) счётчика таймера TIM2.

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  // Разрешаем обработку прерывания по переполнению счётчика таймера TIM2.

	NVIC_EnableIRQ(TIM2_IRQn);	// TIM2_IRQn

}

// Настройка Таймера3 Basic  Таймер запускается по приему байта по USART3 от GPS модуля SIM33ELA,
void TIM3_Configuration		(void) //  ждет паузу между посылками 10 мс, генерирует прерывание
{
	TIM_TimeBaseInitTypeDef base_timer;

	TIM_TimeBaseStructInit(&base_timer);
/* Делитель учитывается как TIM_Prescaler + 1, поэтому отнимаем 1 */
	base_timer.TIM_Prescaler = 36000 - 1; 			// шина ABP1 36 Mhz, но APB1TIM = 72. Нам надо 1 kHz .
	base_timer.TIM_Period = 10*2-1;					// задержка в мс  (ms*2-1)
	base_timer.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &base_timer);
	TIM3->CR1 |= TIM_CR1_OPM; 						// режим одного импульса

/* Очищаем бит  прерывания */
	TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update); 	// также работает TIM_FLAG_Update
  // Разрешаем прерывание по обновлению (в данном случае - по переполнению) счётчика таймера TIM3.

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  // Разрешаем обработку прерывания по переполнению счётчика таймера TIM3.

	NVIC_EnableIRQ(TIM3_IRQn);	//TIM3_IRQn
}

// Настройка Таймера4 Basic
void TIM4_Configuration		(void) //
{
	TIM_TimeBaseInitTypeDef base_timer;

	TIM_TimeBaseStructInit(&base_timer);
/* Делитель учитывается как TIM_Prescaler + 1, поэтому отнимаем 1 */
	base_timer.TIM_Prescaler = 12-1; 	// шина ABP1 36 Mhz, но APB1TIM = 72. Prescaler делит на 12 , тактовая частота таймера 6 МГц
	base_timer.TIM_Period = 0xFFFF;			// ARR - Auto Reload Register. До какого значения считать вверх до переполнения			//
	base_timer.TIM_CounterMode = TIM_CounterMode_Up; 	// считать вверх
	TIM_TimeBaseInit(TIM4, &base_timer);
	//TIM4->CR1 |= TIM_CR1_OPM; 						// режим одного импульса

/* Очищаем бит  прерывания */
	//TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update); 	// также работает TIM_FLAG_Update
  // Разрешаем прерывание по обновлению (в данном случае - по переполнению) счётчика таймера TIM4.

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  // Разрешаем обработку прерывания по переполнению счётчика таймера TIM4.

	TIM_Cmd(TIM4, ENABLE);  		// запуск таймера
	NVIC_EnableIRQ(TIM4_IRQn);	 	// TIM4_IRQn


}

void EXTI_Configuration		(void) // настройка прерывания от 1PPS сигнала
{
	EXTI_InitTypeDef EXTI_InitStructure ;

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12); // источник прерывания - PB12
	EXTI_InitStructure.EXTI_Line = EXTI_Line12 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt ;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising ;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE ;
	EXTI_Init (&EXTI_InitStructure) ;
}

/******************************************************************************/
/*  -------------- STM32F10x Peripherals Interrupt Handlers ---------------   */
/******************************************************************************/

void SysTick_Handler(void) 		// прерывание от системного таймера, вызывается каждую миллисекунду
{
	u8 nn;
	timer1ms ++; // счетчик мс
//	if (timer1ms == 1000000000) // сброс счетчика каждый миллиард миллисекунд
//		timer1ms = 0;
	if ((timer1ms % 1000) == 0) // если делится без остатка на 1000
		timer1s ++; // счетчик cекунд

	if (counterRXDisplay > 0) // счетчик видимости RX на дисплее
	{
		counterRXDisplay -- ;
	}

	if (counterTXDisplay > 0) // счетчик видимости TX на дисплее
	{
		counterTXDisplay -- ;
	}

	if (counterMaxRssiReset > 0) 	// счетчик сброса максимального уровня RSSI
	{
		counterMaxRssiReset -- ;
	}

	for (nn = 0; nn < 10; nn++) 	// счетчики сброса уровня RSSI
		if (counterRssiReset[nn] > 0)
			counterRssiReset[nn] -- ;

	if (counterLed0 > 0) 	// счетчик светодиода LED0 - Синего
	{
		counterLed0 -- ;
		if (counterLed0 == 0)
			OffLED0();
	}

	if (counterLed1 > 0) 	// счетчик светодиода LED1 - Красного
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
  * Кольцевой буфер размером NUM+1
  * В конце данных в буфере пишется \0
  * При наличии данных в буфере поднимается флаг triggerUSART1Interupt
  */
void USART1_IRQHandler(void)  	// терминал на ПК
{
	u8 TempDataUSART1;

    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)	//  проверка флага прерывания RXNE - по приему байта
	{
		TempDataUSART1 = USART_ReceiveData(USART1);		// читаем байт из буфера
		if(counterUSART1Buffer == BufferUSART1Size)		// пишем в буфер по кругу
		{
			BufferUSART1RX[counterUSART1Buffer] = TempDataUSART1;
			counterUSART1Buffer = 0;
		}
		else
		{
			BufferUSART1RX[counterUSART1Buffer++] = TempDataUSART1;
		}
		BufferUSART1RX[counterUSART1Buffer] = '\0';		// после данных в буфер пишем NULL \0

		triggerUSART1Interrupt = 1; 					// установили флаг
	}
}

/**
  * @brief  This function handles USART2 global interrupt request.
  * @param  None
  * @retval None
  * Кольцевой буфер размером NUM+1
  * В конце данных в буфере пишется \0
  * При наличии данных в буфере поднимается флаг triggerUSART2Interupt
  */
void USART2_IRQHandler(void) 	// прерывание по приему байта от BLE модуля
{
	u8 TempDataUSART2;

    if ((USART2->SR & USART_FLAG_RXNE) != (u16)RESET)	//  проверка флага прерывания RXNE - по приему байта
	{
    	TIM_Cmd(TIM2, ENABLE);  // запуск таймера
    	TIM2 -> CNT = 0;		// сброс счетчика таймера

		TempDataUSART2 = USART_ReceiveData(USART2);		// читаем байт из буфера
		if(counterUSART2Buffer == BufferUSART2Size)		// пишем в буфер по кругу
		{
			BufferUSART2RX[counterUSART2Buffer] = TempDataUSART2;
			counterUSART2Buffer = 0;
		}
		else
		{
			BufferUSART2RX[counterUSART2Buffer++] = TempDataUSART2;
		}
		BufferUSART2RX[counterUSART2Buffer] = '\0';		// после данных в буфер пишем NULL \0

		triggerUSART2Interrupt = 1; 					// установили флаг
	}
}

/**
  * @brief  This function handles USART3 global interrupt request.
  * @param  None
  * @retval None
  * Кольцевой буфер размером NUM+1
  * В конце данных в буфере пишется \0
  * При наличии данных в буфере поднимается флаг triggerUSART3Interupt
  */
void USART3_IRQHandler(void) 	// прерывание по приему байта от GPS модуля
{
//	u8 TempDataUSART2;
//    if ((USART2->SR & USART_FLAG_RXNE) != (u16)RESET)	//  проверка флага прерывания RXNE - по приему байта
//	{
//		TempDataUSART2 = USART_ReceiveData(USART2);		// читаем байт из буфера
//		if(counterUSART2Buffer == BufferUSART2Size)		// пишем в буфер по кругу
//		{
//			BufferUSART2RX[counterUSART2Buffer] = TempDataUSART2;
//			counterUSART2Buffer = 0;
//		}
//		else
//		{
//			BufferUSART2RX[counterUSART2Buffer++] = TempDataUSART2;
//		}
//		BufferUSART2RX[counterUSART2Buffer] = '\0';		// после данных в буфер пишем NULL \0
//
//		triggerUSART2Interrupt = 1; 					// установили флаг
//	}

//--------------------------------- NMEA RECEIVER ------------------------------------
// формируем данные с 0 вместо пропущенных значений
  volatile char UsartTemp = 0;
 // volatile char m;

  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // проверяем флаг прерывания
  {
  	TIM_Cmd(TIM3, ENABLE);  // запуск таймера
  	TIM3 -> CNT = 0;		// сброс счетчика таймера.. ждем паузы , конца посылки

	UsartTemp = USART_ReceiveData(USART3);
	GPSBuf[GPSBufWrPoint] = UsartTemp; // todo перенести заполнение нулями в парсер, после проверки CRC

	if ((GPSBufWrPoint > 1) && (GPSBufWrPoint < GPSBUFLENGTH))  // Проверка нахождения внутри буфера, чтобы при дальнейшем преобразовании не выскочить за границу
	{
	  if ((GPSBuf[GPSBufWrPoint-1] == ',') && (GPSBuf[GPSBufWrPoint] == ','))
	  { // Принятая последовательность байт вида ",," превращается в ",0,", тем самым превращая неопределенный параметр в четкий 0
		GPSBuf[GPSBufWrPoint]   = '0';
		GPSBuf[GPSBufWrPoint+1] = ',';
		GPSBufWrPoint++;
	  }
	  if ((GPSBuf[GPSBufWrPoint-1] == ',') && (GPSBuf[GPSBufWrPoint] == '*'))
	  { // Принятая последовательность байт вида ",*" превращается в ",0*", тем самым превращая неопределенный параметр в четкий 0
		GPSBuf[GPSBufWrPoint]   = '0';
		GPSBuf[GPSBufWrPoint+1] = '*';
		GPSBufWrPoint++;
	  }
	}

	if (GPSBufWrPoint < GPSBUFLENGTH)
	  GPSBufWrPoint++;  // Продолжаем заполнять буфер
//	else
//	{ // Если буфер заполнился, начинаем искать и декодировать NMEA сентенции
//printf ("\n\r GPSBufWrPoint1: %d \n\r\n\r", GPSBufWrPoint);
//	  GPSBufWrPoint = 0;
//	  if (NMEA_Parser_GGA() == 1) // GGA - основная сентенция, выдает координаты
//	  {
//printf ("\n\r NMEA_Parser_GGA = 1:\n\r");
//		NMEA_Parser_ZDA();      // ZDA выдает время и дату
//
//		GSVStartPos = 0;
//		SatNum = 0;
//		if (NMEA_Parser_GSV() == 1) // GSV выдает список спутников, видимых приемником. Эту информацию можно использовать для графического отображения
//		{
//		for (m = 0; m <=6; m++) // GSV посылки приходят группами, потому что в каждую посылку входит информация максимум о 4-х спутниках, а их примерно 25
//		  NMEA_Parser_GSV();
//		}
//	  }
//	}
  }



}

// обработчик прерывания от Timer 1 - переполнение
void TIM1_UP_IRQHandler(void)
{
 // произошло ли прерывание по переполнению счётчика таймера TIM1.
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
  {
	/* Очищаем бит обрабатываемого прерывания */
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // также работает TIM_FLAG_Update
	counter_TIM1_OVR ++ ;  						// счетчик перезагрузок таймера

	if ( (counter_TIM1_OVR == 91) && (flag_1PPS_AVG_OK == 1) ) 				//
	  {

	  TIM1->ARR = averageTIM4_ready;			// при следующем переполнении ARR изменится на averageTIM4_ready
	  }

	if ( counter_TIM1_OVR == 92 ) 				// сюда попадаем только если нет сигнала 1PPS, при автономной работе
	  {
//OnLED0();
		counter_TIM1_OVR = 0;
	    TIM1->ARR = 0xFFFF;			// при следующем переполнении ARR изменится на 0xFFFF
	    flag_1PPS_pulse = 1;		// импульс 1PPS от таймера 1
	    SecRTC ++;					// прибавили секунду
//OffLED0();
	  }
  }
}

// обработчик прерывания Timer2 - пришла посылка от BLE модуля
void TIM2_IRQHandler()
{
 // произошло ли прерывание по переполнению счётчика таймера TIM2.
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
	/* Очищаем бит обрабатываемого прерывания */
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // также работает TIM_FLAG_Update
	triggerTIM2Interrupt = 1; 					// установили флаг, принята посылка от BLE модуля
	counterUSART2Buffer = 0;					// сбросили счетчик байт USART буфера
  }
}

// обработчик прерывания Timer3 - пришла посылка от GPS модуля
void TIM3_IRQHandler()
{
// произошло ли прерывание по переполнению счётчика таймера TIM3.
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    /* Очищаем бит обрабатываемого прерывания */
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // также работает TIM_FLAG_Update
    triggerTIM3Interrupt = 1; 					// установили флаг, принята посылка от GPS модуля
  }
}

// обработчик прерывания Timer4
void TIM4_IRQHandler() // счетчик переполняется каждые 10,92 мс, за 1 секунду досчитает до 91
{
// произошло ли прерывание по переполнению счётчика таймера TIM4.
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
	  /* Очищаем бит обрабатываемого прерывания */
	  TIM_ClearITPendingBit(TIM4, TIM_IT_Update); // также работает TIM_FLAG_Update

//CplLED0(); // todo

	  counter_TIM4_OVR ++ ;  		// счетчик перезагрузок таймера
	  if ( counter_TIM4_OVR >= 92 ) // сброс счетчиков при большой паузе между импульсами 1PPS
		  {
		  counter_TIM4_OVR = 0;
		  counter_TIM4_fix = 0;
		  TIM4_fix = 0;
		  flag_1PPS_timeout = 1;
		  flag_1PPS_fail = 1;
		  }

  }
}

// прерывание от переднего фронта 1PPS
void EXTI15_10_IRQHandler (void) // обработчик прерываний от портов 10-15
{
	if (EXTI_GetITStatus(EXTI_Line12))			// проверка флага прерывания
	{
//OnLED0();
		EXTI_ClearITPendingBit (EXTI_Line12); 	// сброс флага прерывания

		TIM4_fix = TIM4->CNT; 						// фиксируем значение таймера
		counter_TIM4_fix = counter_TIM4_OVR; 		// фиксируем значение счетчика циклов таймера

		TIM4->CNT = 0; 				// сброс таймера
		counter_TIM4_OVR = 0; 		// сброс счетчика

		flag_1PPS_Update = 1; 		// обновилось значение 1PPS
		flag_1PPS_pulse = 1;		// импульс пришел

		// -- СИНХРОНИЗАЦИЯ таймера1 ----
		if (flag_1PPS_AVG_OK == 1)		// если после сброса среднее значение периода 1PPS уже получено хотябы один раз
		{
			TIM_Cmd(TIM1, DISABLE);  	// стоп таймера todo уточнить нужно ли
			TIM1->CNT = 0; 				// сброс таймера
			TIM1->ARR = 0xFFFF;			// при следующем переполнении ARR изменится на 0xFFFF
			counter_TIM1_OVR = 0;
			TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update); 	// также работает TIM_FLAG_Update
			TIM_Cmd(TIM1, ENABLE);  	// запуск таймера
			//
			SecRTC = GPSTimeSec + 1;	// текущая секунда таймера равна секунде GPS + 1
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
	// прочитали два регистра SR1 и SR2
    SR1Register = I2C1->SR1;
    SR2Register = I2C1->SR2;

    /* If SB = 1, I2C1 master sent a START on the bus: EV5) */
    // прерывание по событию START от мастера
    if ((SR1Register &0x0001) == 0x0001)
    {
        /* Send the slave address for transmssion or for reception (according to the configured value
            in the write master write routine */
        I2C1->DR = Address;	// посылаем адрес устройства 		extern uint8_t Address
        SR1Register = 0;	// сбрасываем переменные регистров статуса I2C
        SR2Register = 0;
    }
    /* If I2C1 is Master (MSL flag = 1) */
    // передаем данные
    if ((SR2Register &0x0001) == 0x0001)
    {
        /* If ADDR = 1, EV6 */  // адрес уже послан, отправляем первый байт данных
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
			if (NumbOfBytes1 == 0) // если все данные переданы - отключаем прерывание I2C_IT_BUF
			{
				I2C1->CR2 &= (uint16_t)~I2C_IT_BUF;
			}
            SR1Register = 0;
            SR2Register = 0;
        }
        /* Master transmits the remaing data: from data2 until the last one.  */
        /* If TXE is set */ // остальные байты из массива для передачи
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
        // формируем STOP
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

void I2C1_ER_IRQHandler(void)	// обработчик прерывания по ошибке I2C
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
