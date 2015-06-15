//*****************************************************************************
// Program:	 ROHM Sensor Platform Firmware for Q112 Lapis Microcontroller
//		 ROHM Semiconductor USA, LLC
//		 US Design Center
// Started:  July 8th, 2014
// Purpose:	 Firmware for Q112 for Sensor Platform Board 
// Updated:	 July 8th, 2014
//*****************************************************************************
//#define DebugSensor	16

// ============================= Sensor Platform Board Specs ============================== 
//	UART to USB/PC:
//		UART to FTDI => B0, B1
//		VBus Det => A2
//	Sensor Interface:
//		I2C => B5, B6
//		ADC => A0, A1
//		GPIO => B2, B3, B4, B7
//	LED Feedback Section:
//		LEDs = C0 to C7
//	Sensor Control Section:
//		DIP Switch = D0 to D3

//***** PREPROCESSOR DIRECTIVES ***********************************************
// INCLUDED FILES...
// Include Path: common;main;irq;timer;clock;tbc;pwm;uart;

	#include	<ML610112.H>	// Lapis Micro ML610Q112 on LaPi Development Board
	#include	<stdlib.h>		// General-purpose utilities
	#include 	<uart.h>		// UART Function Prototypes
	#include 	<common.h>		// Common Definitions
	#include 	<irq.h>			// IRQ Definitions
	#include 	<mcu.h>			// MCU Definition
	#include	<i2c.h>			// I2C Definition
	//#include 	<clock.h>		// Set System Clock API
	#include 	<tbc.h>			// Set TBC (Timer Based Clock) API
	#include 	<timer.h>		// Timer Macros & APIs
	//#include 	<main.h>		// Clear WDT API
	//#include	<ctype.h>		// Character classification and conversion 
	//#include	<errno.h>		// Error identifiers Library
	//#include	<float.h>		// Numerical limits for floating-point numbers
	//#include	<limits.h>		// Numerical limits for integers
	//#include	<math.h>		// Mathematical functions
	//#include	<muldivu8.h>	// Multiplication and Division accelerator
	//#include	<setjmp.h>		// Global jump (longjmp)
	//#include	<signal.h>		// Signal handling functions
	//#include	<stdarg.h>		// Variable numbers of arguments
	//#include	<stddef.h>		// Standard types and macros 
	#include	<stdio.h>		// I/O-related processing
	//#include	<string.h>		// Character string manipulation routines
	//#include	<yfuns.h>		// 
	//#include	<yvals.h>		// Called for by most Header Files

//===========================================================================
//   MACROS: 
//===========================================================================
#define WelcomeString	("\033[2J\033[1;1H"\
	"*********************************************\n\r"\
	"** Q112 Firmware - Sensor Platform EVK\n\r"\
	"** Revision    : REV00\n\r"\
	"** Release date: " __DATE__ " " __TIME__ "\n\r"\
	"** By          : ROHM Semiconductor USA, LLC\n\r"\
	"*********************************************\n\n\n\n\r"\
)

#define PRINTF(msg)		write(0, msg, sizeof(msg))

// ===== Peripheral setting.=====
#define HSCLK_KHZ	( 8000u )	// 8MHz = 8000kHz (will be multiplied by 1024 to give 8,192,000Hz)
#define FLG_SET		( 0x01u )

// SET DESIRED UART SETTINGS HERE! (Options in UART.h)
#define UART_BAUDRATE		( UART_BR_115200BPS) 	// Data Bits Per Second - Tested at rates from 2400bps to 512000bps!
//#define UART_BAUDRATE		( UART_BR_9600BPS) 	// Data Bits Per Second - Tested at rates from 2400bps to 512000bps!
#define UART_DATA_LENGTH	( UART_LG_8BIT )		// x-Bit Data
#define UART_PARITY_BIT		( UART_PT_NON )		// Parity
#define UART_STOP_BIT		( UART_STP_1BIT )		// x Stop-Bits
#define UART_LOGIC			( UART_NEG_POS )		// Desired Logic
#define UART_DIRECTION		( UART_DIR_LSB )		// LSB or MSB First
//#define _TBC_H_

/**
 * Sensor Interface Header 1
 */
#define SENINTF_HDR1_GPIO0(reg)		PB2##reg
#define SENINTF_HDR1_GPIO1(reg)	PB3##reg
#define SENINTF_HDR1_GPIO2(reg)		PB4##reg
#define SENINTF_HDR1_GPIO3(reg)		PB7##reg

/**
 * LED[7-0]
 */
#define LEDOUT(x)	PCD=x

//===========================================================================
//   STRUCTURES:    
//===========================================================================
static const tUartSetParam  _uartSetParam = {		// UART Parameters
	UART_BAUDRATE,								// Members of Structure...
	UART_DATA_LENGTH,							// Members of Structure...
	UART_PARITY_BIT,							// Members of Structure...
	UART_STOP_BIT,								// Members of Structure...
	UART_LOGIC,									// Members of Structure...
	UART_DIRECTION								// Members of Structure...
};

//===========================================================================
//   FUNCTION PROTOTYPES: 
//	Establishes the name and return type of a function and may specify the 
// 	types, formal parameter names and number of arguments to the function                                 
//===========================================================================
void main_clrWDT( void );			// no return value and no arguments
void Initialization( void );		// no return value and no arguments
void SetOSC( void );				// no return value and no arguments
void PortA_Low( void );				// no return value and no arguments
void PortB_Low( void );				// no return value and no arguments
void PortC_Low( void );				// no return value and no arguments
void PortD_Low( void );				// no return value and no arguments

//RTLU8: Low-level function
int write(int handle, unsigned char *buffer, unsigned int len);
int ADC_Read(unsigned char idx);
void I2C_Read(unsigned char slave_address, unsigned char *reg_address, unsigned char reg_address_size, unsigned char *buffer, unsigned char size);
void I2C_Write(unsigned char slave_address, unsigned char *reg_address, unsigned char reg_address_size, unsigned char *buffer, unsigned char size);

//UART and I2C Functions
void _funcUartFin( unsigned int size, unsigned char errStat );
void _funcI2CFin( unsigned int size, unsigned char errStat );
void main_reqNotHalt( void );
void _intUart( void );
void _intI2c( void );
void _intADC( void );
void NOPms( unsigned int ms );

unsigned char ReverseBits(unsigned char data);
unsigned char FlashLEDs(void);

void DeviceSelection(void); // Initializes port D for registering Sensor Control States
void SensorInitialization(void); 
  
void MainOp_Hall_Effect_Sensors_1();
void MainOp_Hall_Effect_Sensors_2();
void MainOp_Pressure_Sensor_3();			//new gabriel
void MainOp_Color_Sensor_4();
void MainOp_Ambient_Light_Sensor_5();
void MainOp_Ambient_Light_Sensor_6();
void MainOp_Ambient_Light_Sensor_7();
void MainOp_Ambient_Light_Sensor_8();
void MainOp_Ambient_Light_Sensor_9();
void MainOp_UV_Sensor_10();
void MainOp_ALS_PROX_Sensor_11();
void MainOp_KXG03_Sensor_12();
void MainOp_KX022();
void MainOp_KMX61();
void MainOp_Temperature_Sensor_20();
void MainOp_Temperature_Sensor_21();
void MainOp_Temperature_Sensor_22();
void MainOp_Temperature_Sensor_23();

void Init_Hall_Effect_Sensors_1();
void Init_Hall_Effect_Sensors_2();
void Init_Pressure_Sensor_3(); //48
void Init_Color_Sensor_4();		//49
void Init_Ambient_Light_Sensor_5();
void Init_Ambient_Light_Sensor_6();
void Init_Ambient_Light_Sensor_7();
void Init_Ambient_Light_Sensor_8();
void Init_Ambient_Light_Sensor_9();
void Init_UV_Sensor_10();
void Init_ALS_PROX_Sensor_11(); //50
void Init_KXG03_Sensor_12();	//52
void Init_KX022();
void Init_KMX61();
void Init_Temperature_Sensor_20();
void Init_Temperature_Sensor_21();
void Init_Temperature_Sensor_22();
void Init_Temperature_Sensor_23();

//*****************************************************************************
// GLOBALS...
// ADC, UART and I2C Variables
unsigned char	_flgUartFin;
unsigned char 	_flgI2CFin;
unsigned char	_flgADCFin;
unsigned char	_reqNotHalt;

union {
	unsigned char	_uchar;
	unsigned char	_ucharArr[6];
	unsigned int	_uint;
	unsigned int	_uintArr[3];
	int				_intArr[3];
	float			_float;
} uniRawSensorOut;

float flSensorOut[3];

/**
 * ANSI Escape Code
 */
#define ESC_SOL			"\r"	//end
#define ESC_NEWLINE		"\n\r"	//beg
#define ESC_PREVLINE	"\033[F" //end-cont new line
#define ESC_ERASE2END	"\033[J" //beg
#define ESC_CLRSCR		"\033[2J" //
#define ESC_STX			"\033[37F"
/**
 * Ambient Light Sensors
 */ 
// I2C device address of: BH1710FVC, BH1721FVC
const unsigned char BH17xxFVC_ADDR_1			= 0x23u;
// I2C device address of: BH1730FVC, BH1780GLI
const unsigned char BH17xxFVC_ADDR_2			= 0x29u;

const unsigned char BH17xxFVC_PWR_DOWN			= 0x00u;
const unsigned char BH17xxFVC_PWR_ON			= 0x01u;
/**
 * BH1710FVC
 */		
// Register addresses of BH1710FVC
const unsigned char BH1710FVC_RESET				= 0x07u;
const unsigned char BH1710FVC_CONT_H_RES_MOD	= 0x10u;
const unsigned char BH1710FVC_CONT_M_RES_MOD	= 0x13u;
const unsigned char BH1710FVC_CONT_L_RES_MOD	= 0x16u;
const unsigned char BH1710FVC_ONET_H_RES_MOD	= 0x20u;
const unsigned char BH1710FVC_ONET_M_RES_MOD	= 0x23u;
const unsigned char BH1710FVC_ONET_L_RES_MOD	= 0x26u;
/**
 * BH1721FVC
 */
// Register addresses of BH1721FVC
const unsigned char BH1721FVC_CONT_A_RES_MOD	= 0x10u;	// or 0x20u
const unsigned char BH1721FVC_CONT_H_RES_MOD	= 0x12u;	// or 0x22u
const unsigned char BH1721FVC_CONT_L_RES_MOD	= 0x13u;	// or 0x16u or 0x23u or 0x26u
/**
 * BH1730FVC
 */
// Register addresses of BH1730FVC
const unsigned char BH1730FVC_REG_CONTROL		= 0x80u;
const unsigned char BH1730FVC_REG_TIMING		= 0x81u;
const unsigned char BH1730FVC_REG_INTERRUPT		= 0x82u;
const unsigned char BH1730FVC_REG_THLLOW		= 0x83u;
const unsigned char BH1730FVC_REG_THLHIGH		= 0x84u;
const unsigned char BH1730FVC_REG_THHLOW		= 0x85u;
const unsigned char BH1730FVC_REG_THHHIGH		= 0x86u;
const unsigned char BH1730FVC_REG_GAIN			= 0x87u;
const unsigned char BH1730FVC_REG_ID			= 0x92u;
const unsigned char BH1730FVC_REG_DATA0LOW		= 0x94u;
const unsigned char BH1730FVC_REG_DATA0HIGH		= 0x95u;
const unsigned char BH1730FVC_REG_DATA1LOW		= 0x96u;
const unsigned char BH1730FVC_REG_DATA1HIGH		= 0x97u;
const unsigned char BH1730FVC_CMD_RESET_INT_OUT	= 0xe1u;
const unsigned char BH1730FVC_CMD_STOP_MIM		= 0xe2u;	// Stop manual integration mode.
const unsigned char BH1730FVC_CMD_START_MIM		= 0xe3u;	// Start manual integration mode.
const unsigned char BH1730FVC_CMD_SW_RESET		= 0xe4u;
/**
 * BH1780GLI
 */
// Register addresses of BH1780GLI
const unsigned char BH1780GLI_REG_CONTROL		= 0x80u;
const unsigned char BH1780GLI_REG_PART_ID		= 0x8au;
const unsigned char BH1780GLI_REG_MFG_ID		= 0x8bu;
const unsigned char BH1780GLI_REG_DATALOW		= 0x8cu;
const unsigned char BH1780GLI_REG_DATAHIGH		= 0x8du;

/**
 * ML8511 (UV Sensor)
 *		Vout 		: 2.2[V] @ 10[mW/cm2]
 *		Sensitivity	: 0.129[Vcm2/mW]
 */
#define	Voltage2UVIntensity(v)	(v-2.2f)/0.129f+10

/**
 * Temperature Sensors
 *		Vout		: v0[V] @ t0[°C]
 *		Sensitivity	: s[V/°C]
 */
#define Voltage2Temperature(v, v0, t0, s)	(v-(v0))/(s)+(t0)

/**
 * KX022 (± 2g/4g/8g Tri-axis Digital Accelerometer)
 */
// I2C device address of KX022
const unsigned char KX022_I2C_ADDR		= 0x1eu;
// Register addresses of KX022
const unsigned char KX022_XHPL			= 0x00u;
const unsigned char KX022_XHPH			= 0x01u;
const unsigned char KX022_YHPL			= 0x02u;
const unsigned char KX022_YHPH			= 0x03u;
const unsigned char KX022_ZHPL			= 0x04u;
const unsigned char KX022_ZHPH			= 0x05u;
const unsigned char KX022_XOUTL			= 0x06u;
const unsigned char KX022_XOUTH			= 0x07u;
const unsigned char KX022_YOUTL			= 0x08u;
const unsigned char KX022_YOUTH			= 0x09u;
const unsigned char KX022_ZOUTL			= 0x0au;
const unsigned char KX022_ZOUTH			= 0x0bu;
const unsigned char KX022_COTR			= 0x0cu;
const unsigned char KX022_WHO_AM_I		= 0x0fu;
const unsigned char KX022_TSCP			= 0x10u;
const unsigned char KX022_TSPP			= 0x11u;
const unsigned char KX022_INS1			= 0x12u;
const unsigned char KX022_INS2			= 0x13u;
const unsigned char KX022_INS3			= 0x14u;
const unsigned char KX022_STAT			= 0x15u;
const unsigned char KX022_INT_REL		= 0x17u;
const unsigned char KX022_CNTL1			= 0x18u;
const unsigned char KX022_CNTL2			= 0x19u;
const unsigned char KX022_CNTL3			= 0x1au;
const unsigned char KX022_ODCNTL		= 0x1bu;
const unsigned char KX022_INC1			= 0x1cu;
const unsigned char KX022_INC2			= 0x1du;
const unsigned char KX022_INC3			= 0x1eu;
const unsigned char KX022_INC4			= 0x1fu;
const unsigned char KX022_INC5			= 0x20u;
const unsigned char KX022_INC6			= 0x21u;
const unsigned char KX022_TILT_TIMER	= 0x22u;
const unsigned char KX022_WUFC			= 0x23u;
const unsigned char KX022_TDTRC			= 0x24u;
const unsigned char KX022_TDTC			= 0x25u;
const unsigned char KX022_TTH			= 0x26u;
const unsigned char KX022_TTL			= 0x27u;
const unsigned char KX022_FTD			= 0x28u;
const unsigned char KX022_STD			= 0x29u;
const unsigned char KX022_TLT			= 0x2au;
const unsigned char KX022_TWS			= 0x2bu;
const unsigned char KX022_ATH			= 0x30u;
const unsigned char KX022_TILT_ANGLE_LL	= 0x32u;
const unsigned char KX022_TILT_ANGLE_HL	= 0x33u;
const unsigned char KX022_HYST_SET		= 0x34u;
const unsigned char KX022_LP_CNTL		= 0x35u;
const unsigned char KX022_BUF_CNTL1		= 0x3au;
const unsigned char KX022_BUF_CNTL2		= 0x3bu;
const unsigned char KX022_BUF_STATUS_1	= 0x3cu;
const unsigned char KX022_BUF_STATUS_2	= 0x3du;
const unsigned char KX022_BUF_CLEAR		= 0x3eu;
const unsigned char KX022_BUF_READ		= 0x3fu;
const unsigned char KX022_SELF_TEST		= 0x60u;
// Configuration data
// Set accelerometer to stand-by mode(PC1=0), +/-2g - 16bits and enable tilt position function
const unsigned char KX022_CNTL1_CFGDAT		= 0x41u;
// Set Output Data Rate(ODR) to 50Hz
const unsigned char KX022_ODCNTL_CFGDAT		= 0x02u;
// Set Tilt Output Data Rate to 50Hz
const unsigned char KX022_CNTL3_CFGDAT		= 0xd8u;
// Set Tilt Timer to 20ms
const unsigned char KX022_TILT_TIMER_CFGDAT	= 0x01u;

/**
 * KMX61 (Digital Tri-axis Magnetometer/Tri-axis Accelerometer)
 */
// I2C device address of KMX61
const unsigned char KMX61_I2C_ADDR			= 0x0eu;
// Register addresses of KMX61
const unsigned char KMX61_WHO_AM_I			= 0x00u;
const unsigned char KMX61_INS1				= 0x01u;
const unsigned char KMX61_INS2				= 0x02u;
const unsigned char KMX61_STATUS_REG		= 0x03u;
const unsigned char KMX61_ACCEL_XOUT_L		= 0x0au;
const unsigned char KMX61_ACCEL_XOUT_H		= 0x0bu;
const unsigned char KMX61_ACCEL_YOUT_L		= 0x0cu;
const unsigned char KMX61_ACCEL_YOUT_H		= 0x0du;
const unsigned char KMX61_ACCEL_ZOUT_L		= 0x0eu;
const unsigned char KMX61_ACCEL_ZOUT_H		= 0x0fu;
const unsigned char KMX61_TEMP_OUT_L		= 0x10u;
const unsigned char KMX61_TEMP_OUT_H		= 0x11u;
const unsigned char KMX61_MAG_XOUT_L		= 0x12u;
const unsigned char KMX62_MAG_XOUT_L		= 0x10u;

const unsigned char KMX61_MAG_XOUT_H		= 0x13u;
const unsigned char KMX61_MAG_YOUT_L		= 0x14u;
const unsigned char KMX61_MAG_YOUT_H		= 0x15u;
const unsigned char KMX61_MAG_ZOUT_L		= 0x16u;
const unsigned char KMX61_MAG_ZOUT_H		= 0x17u;
const unsigned char KMX61_XOUT_HPF_L		= 0x18u;
const unsigned char KMX61_XOUT_HPF_H		= 0x19u;
const unsigned char KMX61_YOUT_HPF_L		= 0x1au;
const unsigned char KMX61_YOUT_HPF_H		= 0x1bu;
const unsigned char KMX61_ZOUT_HPF_L		= 0x1cu;
const unsigned char KMX61_ZOUT_HPF_H		= 0x1du;
const unsigned char KMX61_SN_1				= 0x24u;
const unsigned char KMX61_SN_2				= 0x25u;
const unsigned char KMX61_SN_3				= 0x26u;
const unsigned char KMX61_SN_4				= 0x27u;
const unsigned char KMX61_INL				= 0x28u;
const unsigned char KMX61_STBY_REG			= 0x29u;
const unsigned char KMX61_CNTL1				= 0x2au;
const unsigned char KMX61_CNTL2				= 0x2bu;
const unsigned char KMX61_ODCNTL			= 0x2cu;
const unsigned char KMX61_INC1				= 0x2du;
const unsigned char KMX61_INC2				= 0x2eu;
const unsigned char KMX61_INC3				= 0x2fu;
const unsigned char KMX61_COTR				= 0x3cu;
const unsigned char KMX62_CNTL2				= 0x3au;
const unsigned char KMX62_CNTL2_CFG			= 0x5fu;

const unsigned char KMX61_WUFTH				= 0x3du;
const unsigned char KMX61_WUFC				= 0x3eu;
const unsigned char KMX61_BTH				= 0x3fu;
const unsigned char KMX61_BTSC				= 0x40u;
const unsigned char KMX61_TEMP_EN_CNTL		= 0x4cu;
const unsigned char KMX61_SELF_TEST			= 0x60u;
const unsigned char KMX61_BUF_THRESH_H		= 0x76u;
const unsigned char KMX61_BUF_THRESH_L		= 0x77u;
const unsigned char KMX61_BUF_CTRL1			= 0x78u;
const unsigned char KMX61_BUF_CTRL2			= 0x79u;
const unsigned char KMX61_BUF_CLEAR			= 0x7au;
const unsigned char KMX61_BUF_STATUS_REG	= 0x7bu;
const unsigned char KMX61_BUF_STATUS_H		= 0x7cu;
const unsigned char KMX61_BUF_STATUS_L		= 0x7du;
const unsigned char KMX61_BUF_READ			= 0x7eu;
// Configuration data
// Disable self-test function
const unsigned char KMX61_SELF_TEST_CFGDAT = 0x0u;
// Disable Back to Sleep engine, Wake up engine and interrupt pin.
// Set operating mode is higher power mode and +/-8g - 14bit
const unsigned char KMX61_CNTL1_CFGDAT	= 0x13u;
// Set Output Data Rate at which the wake up (motion detection) is 0.781Hz
const unsigned char KMX61_CNTL2_CFGDAT	= 0x0u;
// Set Output Data Rate of accelerometer and magnetometer are 12.5Hz
const unsigned char KMX61_ODCNTL_CFGDAT	= 0x0u;
// Enable the Temperature output when the Magnetometer is on
const unsigned char KMX61_TEMP_EN_CNTL_CFGDAT = 0x01u;
// Set operating mode of the sample buffer is FIFO
const unsigned char KMX61_BUF_CTRL1_CFGDAT = 0x0u;
const unsigned char KMX61_BUF_CTRL2_CFGDAT = 0x0u;

/**
 * KXG03 (MEMS Sensor series Accelerometer & Gyro Sensor IC)
 */
 // I2C device address of KXG03
const unsigned char KXG03_I2C_ADDR			= 0x4e;	//could be 0x4f depends on ADDR pin

//Relevant register Addresses
const unsigned char KXG03_GYRO_XOUT_L	=	0x00;	
const unsigned char KXG03_GYRO_XOUT_H	=	0x01;	
const unsigned char KXG03_GYRO_YOUT_L	=	0x02;	
const unsigned char KXG03_GYRO_YOUT_H	=	0x03;	
const unsigned char KXG03_GYRO_ZOUT_L	=	0x04;	
const unsigned char KXG03_GYRO_ZOUT_H	=	0x05;	
const unsigned char KXG03_ACC_XOUT_L	=	0x06;	
const unsigned char KXG03_ACC_XOUT_H	=	0x07;	
const unsigned char KXG03_ACC_YOUT_L	=	0x08;	
const unsigned char KXG03_ACC_YOUT_H	=	0x09;	
const unsigned char KXG03_ACC_ZOUT_L	=	0x0A;	
const unsigned char KXG03_ACC_ZOUT_H	=	0x0B;	

//configuration register
const unsigned char KXG03_STBY_REG		=	0x29;	
const unsigned char KXG03_CTRL_REG1		=	0x2A;	
const unsigned char KXG03_CTRL_REG2		=	0x2B;	
const unsigned char KXG03_ODCTRL		=	0x2C;	

//configuration data
const unsigned char KXG03_STBY_SET			=	0x7f;
const unsigned char KXG03_CTRL_REG1_CFDAT	=	0x1c;	//RES =1 (high), +/-2g, RSEl = =/-2048		
const unsigned char KXG03_CTRL_REG2_CFDAT	=	0x06;	//OutDR = 50HZ
const unsigned char KXG03_ODCTRL_CFDAT		=	0x22;	//GYRBW = 40Hz 

 
/**
 * BM1383GLV (MEMS Sensor series Pressure Sensor IC)
 */
// I2C device address of BM1383GLV
const unsigned char BM1383GLV_I2C_ADDR			= 0x5du;

// Register addresses of BM1383GLV
const unsigned char BM1383GLV_ID 				= 0x10u;
const unsigned char BM1383GLV_RESET_CONTROL 	= 0x11u;
const unsigned char BM1383GLV_POWER_DOWN 		= 0x12u;
const unsigned char BM1383GLV_SLEEP 			= 0x13u;
const unsigned char BM1383GLV_MODE_CONTROL 		= 0x14u;
const unsigned char BM1383GLV_TEMPERATURE_MSB 	= 0x1au;	//data read starts here
const unsigned char BM1383GLV_TEMPERATURE_LSB 	= 0x1bu;
const unsigned char BM1383GLV_PRESSURE_MSB	  	= 0x1cu;
const unsigned char BM1383GLV_PRESSURE_LSB	  	= 0x1du;
//const unsigned char BM1383GLV_PRESSURE_LSB    	= 0x1eu;

//configuration data
const unsigned char BM1383GLV_POWER_ON			= 0x01u;
const unsigned char BM1383GLV_POWER_OFF			= 0x00u;
const unsigned char BM1383GLV_SLEEP_ACT   		= 0x01u;
const unsigned char BM1383GLV_SLEEP_SLP   		= 0x00u;
const unsigned char BM1383GLV_MODE_CONTROL_AVGN = 0x02u;	//12 ms delay
const unsigned char BM1383GLV_MODE_CONTROL_MODE = 0x04u;	//sampling 200ms
const unsigned char BM1383GLV_MODE_CONTROL_REG  = 0x44u;

/**
*	BH1745NUC Color Sensor	DATA
*/
//I2C device address of BH1745
const unsigned char BH1745_I2C_ADDR						= 0x39u; 

//Configuration register addresses of BM1383GLV
const unsigned char BH1745_SYSTEM_CONTROL				= 0x40u; 
const unsigned char BH1745_MODE_CONTROL1				= 0x41u; 
const unsigned char BH1745_MODE_CONTROL2				= 0x42u; 
const unsigned char BH1745_MODE_CONTROL3				= 0x43u; 
const unsigned char BH1745_PERSISTENCE	 				= 0x61u; 
const unsigned char BH1745_INTERRUPT	 				= 0x60u; 

//Register addresses of BM1383GLV
const unsigned char BH1745_MANUFACTURER_ID				= 0x92u; 
const unsigned char BH1745_RED_DATA_LSBs	 			= 0x50u; 
const unsigned char BH1745_RED_DATA_MSBs	 			= 0x51u; 
const unsigned char BH1745_GREEN_DATA_LSBs	 	 		= 0x52u; 
const unsigned char BH1745_GREEN_DATA_MSBs	 			= 0x53u; 
const unsigned char BH1745_BLUE_DATA_LSBs	 	 		= 0x54u; 
const unsigned char BH1745_BLUE_DATA_MSBs	 	 		= 0x55u;

/*
*	RPR0521
*/
//I2C device address of RPR0521
const unsigned char RPR0521_I2C_ADDR			= 0x38u; 

//Configuration register addresses of RPR0521
const unsigned char RPR0521_Prox_ModeCTR		= 0x41u;
const unsigned char RPR0521_ALS_ModeCTR			= 0x42u; //(0x02) defaults
const unsigned char RPR0521_PS_ModeCTR			= 0x43u;  //(0x01) defaults

//configuration data
const unsigned char RPR0521_Prox_ModeCTR_CFG	= 0xC6u;
const unsigned char RPR0521_ALS_ModeCTR_CFG		= 0x02u; //(0x02) defaults
const unsigned char RPR0521_PS_ModeCTR_CFG		= 0x21u;  //(0x01) defaults, PS GAIN ×4

//Register addresses of RPR0521
const unsigned char RPR0521_Manufact_ID			= 0x92u;
const unsigned char RPR0521_Prox_PS_LSB			= 0x44u; 
const unsigned char RPR0521_Prox_PS_MSB			= 0x45u; 
const unsigned char RPR0521_ALS0_LSB			= 0x46u; //visible light
const unsigned char RPR0521_ALS0_MSB			= 0x47u;  
const unsigned char RPR0521_ALS1_LSB			= 0x48u; //infrared light
const unsigned char RPR0521_ALS1_MSB			= 0x49u;



static unsigned char SensorPlatformSelection;
static unsigned char SensorIntializationFlag = 1;
static unsigned char LEDFlashFlag = 0;
static unsigned char LEDChangeFlag = 0;

//===========================================================================
//  	Start of MAIN FUNCTION
//===========================================================================
int main(void) 
{ 	
	Initialization(); //Ports, UART, Timers, Oscillator, Comparators, etc.

	PRINTF(WelcomeString);

#ifdef DebugSensor // Debug Initialization For devices
	SensorIntializationFlag = 1;
	SensorPlatformSelection = DebugSensor;
#endif
	
MainLoop:
	main_clrWDT();
	
#ifndef DebugSensor	//Debug Main Loop For Devices
	DeviceSelection(); 	// SensorPlatformSelection holds 8-bits of sensor type
#endif

	if(SensorIntializationFlag==1){	//Holds the SW Statement for Initializing Sensors
		SensorInitialization();
		SensorIntializationFlag = 0;
	}
	
	switch(SensorPlatformSelection){
		case 1:
			MainOp_Hall_Effect_Sensors_1(); // Refer to function description for list of sensors
			break;
		case 2:
			MainOp_Hall_Effect_Sensors_2(); // Refer to function description for list of sensors 
			break;
		
		case 48:
			
			MainOp_Temperature_Sensor_22();
			MainOp_UV_Sensor_10();
			MainOp_Hall_Effect_Sensors_2();
			MainOp_KMX61();
			
			MainOp_Pressure_Sensor_3();
			MainOp_ALS_PROX_Sensor_11();
			MainOp_Color_Sensor_4();

			MainOp_KX022();
			
			

			
			break;
		case 49:
			MainOp_Pressure_Sensor_3();
			break;
		case 50:
			MainOp_Color_Sensor_4();
			break;
		case 56:
			MainOp_ALS_PROX_Sensor_11();
			break;
		case 52:
			MainOp_KXG03_Sensor_12();
			break;
			
		case 5: 
			MainOp_Ambient_Light_Sensor_5(); // Refer to function description for list of sensors
			break;
		case 6:
			MainOp_Ambient_Light_Sensor_6(); // Refer to function description for list of sensors 
			break;
		case 7:
			MainOp_Ambient_Light_Sensor_7(); // Refer to function description for list of sensors 
			break;
		case 8:
			MainOp_Ambient_Light_Sensor_8(); // Refer to function description for list of sensors 
			break;
		case 9:
			MainOp_Ambient_Light_Sensor_9(); // Refer to function description for list of sensors 
			break;
		case 10:
			MainOp_UV_Sensor_10(); // Refer to function description for list of sensors 
			break;
		case 15:
			MainOp_KX022(); // Refer to function description for list of sensors 
			break;
		case 16:
			MainOp_KMX61(); // Refer to function description for list of sensors 
			break;
		case 20:
			MainOp_Temperature_Sensor_20(); // Refer to function description for list of sensors 
			break;
		case 21:
			MainOp_Temperature_Sensor_21(); // Refer to function description for list of sensors
			break;
		case 22:
			MainOp_Temperature_Sensor_22(); // Refer to function description for list of sensors 
			break;
		case 23:
			MainOp_Temperature_Sensor_23(); // Refer to function description for list of sensors
			break; 
		default:
			PRINTF(ESC_ERASE2END "No device connected." ESC_SOL);
			printf(ESC_ERASE2END "device selected = %lu[lx]" ESC_SOL, (unsigned long)SensorPlatformSelection);
			FlashLEDs();
			break;
	}
	
	HLT = 1;	//Wait time here depends on the WDT timing
	__asm("nop\n"); 
	__asm("nop\n");
	
	goto MainLoop;
}
//===========================================================================
//  	End of MAIN FUNCTION
//===========================================================================


//===========================================================================
//  	Start of Other Functions...
//===========================================================================
//==========================================================================
//	Initialize Micro to Desired State...
//===========================================================================
static void Initialization(void){

	//Initialize Peripherals	
	//BLKCON2 Control Bits...Manually Set 4/12/2013
	DSIO0 = 1; // 0=> Enables Synchronous Serial Port 0 (initial value).
	DUA0  = 0; // 0=> Enables the operation of UART0 (initial value).
	DUA1  = 1; // 0=> Enables Uart1 (initial value). 
	DI2C1 = 1; // 0=> Enables I2C bus Interface (Slave) (initial value).
	DI2C0 = 0; // 0=> Enables I2C bus Interface (Master) (initial value).	
	
	BLKCON4 = 0x00; // 0=> Enables SA-ADC
	BLKCON6 = 0x00; // (1=disables; 0=enables) the operation of Timers 8, 9, A, E, F.
	BLKCON7 = 0x0F; // (1=disables; 0=enables) the operation of PWM (PWMC, PWMD, PWME, PWMF

	// Port Initialize
	PortA_Low();	//Initialize all 3 Ports of Port A to GPIO-Low
	PortB_Low();	//Initialize all 8 Ports of Port B to GPIO-Low
	PortC_Low();	//Initialize all 8 Ports of Port C to GPIO-Low
	PortD_Low();	//Initialize all 6 Ports of Port D to input GPIO
	
	// Set Oscillator Rate
    SetOSC();
	
	// Settings for the ADC input (A0, A1)
	PA0DIR = 1;
	PA1DIR = 1;		//GPIO Input
	SACH0 = 1;		//This enables the ADC Channel 0 from A0 Pin
	SACH1 = 1;		//This enables the ADC Channel 1 from A1 Pin
	SALP = 0;		//Single Read or Continuous Read... Single = 0, Consecutive = 1
	
	// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
	// INTERRUPT SETUP...
	irq_di();	// Disable Interrupts
	irq_init();	// Initialize Interrupts (All Off and NO Requests)

	// INTERRUPT ENABLE REGISTERS...
	IE0 = IE1 = IE2 = IE3 = IE4 = IE5 = IE6 = IE7 = 0;
	// INTERRUPT REQUEST REGISTERS...
	IRQ0 = IRQ1 = IRQ2 = IRQ3 = IRQ4 = IRQ5 = IRQ6 = IRQ7 = 0;

	E2H = 0;	// E2H is the Enable flag for 2Hz TBC Interrupt (1=ENABLED)
				
	irq_setHdr((unsigned char)IRQ_NO_UA0INT, _intUart);
	EUA0 = 1; 	// EUA0 is the enable flag for the UART0 interrupt (1=ENABLED)
	
	irq_setHdr((unsigned char)IRQ_NO_I2CMINT, _intI2c);
	EI2CM = 1;
	QI2CM = 0;
	
	//Enable ADC Interrupts Handler
	irq_setHdr((unsigned char)IRQ_NO_SADINT, _intADC);
	ESAD = 1;
	QSAD = 0;
	
	/*
	//Set up xHz TBC Interrupt (Options: 128Hz, 32Hz, 16Hz, 2Hz)
	//(void)irq_setHdr( (unsigned char)IRQ_NO_T2HINT, TBC_ISR );  //Clear interrupt request flag
	
	// TBC...Set Ratio: : 1:1 => 1_1
	(void)tb_setHtbdiv( (unsigned char)TB_HTD_1_1 ); //Set the ratio of dividing frequency of the time base counter
		E2H = 0;	  // Enable x Hz TBC Interrupt (1=ENABLED)
		Q2H = 0;	  // Request flag for the time base counter x Hz interrupt	
	*/
	
	irq_ei(); // Enable Interrupts
	// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

	// WDT... This will be the triggering condition to return from halt mode
	// We will need to calibrate based on the timing of our loop
	// 0x00 = 125ms
	// 0x01 = 500ms
	// 0x02 = 2sec
	// 0x03 = 8sec
	// 0x04 = 23.4ms
	// 0x05 = 31.25ms
	// 0x06	= 62.5ms
	WDTMOD = 0x01; 	
	main_clrWDT(); 	// Clear WDT
	
	//I2C Initialization...
	//P20C0 = 1;	/* CMOS output */
	//P20C1 = 1;	
	//P20D = 1;		/* write protect enable */
	i2c_init(I2C_MOD_FST, (unsigned short)HSCLK_KHZ, I2C_SYN_OFF);
	
	//UART Initialization...
	uart_init((unsigned char)UART_CS_HSCLK,		/* Generator       */
			  (unsigned short)HSCLK_KHZ,		/* HSCLK frequency */
			   &_uartSetParam );				/* Param... 	 */
	uart_PortSet();
	
	//Write "Program Start" UART Control
	
}//End Initialization
//===========================================================================

/*******************************************************************************
	Routine Name	: write
	Form			: int write(int handle, unsigned char *buffer, unsigned int len)
	Parameters		: int handle
					  unsigned char *buffer
					  unsigned int len
	Return value	: int
	Initialization	: None.
	Description		: The write function writes len bytes of data from the area specified by buffer to UART0.
******************************************************************************/
int write(int handle, unsigned char *buffer, unsigned int len)
{
	_flgUartFin = 0; 
	uart_stop();
	uart_startSend(buffer, len, _funcUartFin); 
	while(_flgUartFin != 1)
	{
		main_clrWDT();
	}
	return len;
}

/*******************************************************************************
	Routine Name	: ADC_Read
	Form			: int ADC_Read()
	Parameters		: unsigned char idx
	Return value	: int
	Initialization	: None.
	Description		: Read ADC(idx) value
******************************************************************************/
int ADC_Read(unsigned char idx)
{
	_flgADCFin = 0;
	SADMOD0 = (unsigned char)(1<<idx);
	SARUN = 1;
	while(_flgADCFin == 0)
	{
		main_clrWDT();
	}
	switch(idx)
	{
		case 0:		return (SADR0H<<2|SADR0L>>6);
		case 1:		return (SADR1H<<2|SADR1L>>6);
		case 2:		return (SADR2H<<2|SADR2L>>6);
		case 3:		return (SADR3H<<2|SADR3L>>6);
		case 4:		return (SADR4H<<2|SADR4L>>6);
		case 5:		return (SADR5H<<2|SADR5L>>6);
		case 6:		return (SADR6H<<2|SADR6L>>6);
		case 7:		return (SADR7H<<2|SADR7L>>6);
		default:	return 0;
	}
}

/*******************************************************************************
	Routine Name	: I2C_Read
	Form			: void I2C_Read(unsigned char slave_address, unsigned char *address, unsigned char address_size, unsigned char *buffer, unsigned char size)
	Parameters		: unsigned char slave_address
					  unsigned char *address
					  unsigned char address_size
					  unsigned char *buffer
					  unsigned char size
	Return value	: void
	Initialization	: None.
	Description		: 
******************************************************************************/
void I2C_Read(unsigned char slave_address, unsigned char *reg_address, unsigned char reg_address_size, unsigned char *buffer, unsigned char size)
{
	_flgI2CFin = 0;
	i2c_stop();	
	i2c_startReceive(slave_address, reg_address, reg_address_size, buffer, size, (cbfI2c)_funcI2CFin);
	while(_flgI2CFin != 1)
	{
		main_clrWDT();
	}
}

/*******************************************************************************
	Routine Name	: I2C_Write
	Form			: void I2C_Write(unsigned char slave_address, unsigned char *address, unsigned char address_size, unsigned char *buffer, unsigned char size)
	Parameters		: unsigned char slave_address
					  unsigned char *address
					  unsigned char address_size
					  unsigned char *buffer
					  unsigned char size
	Return value	: void
	Initialization	: None.
	Description		: 
******************************************************************************/
void I2C_Write(unsigned char slave_address, unsigned char *reg_address, unsigned char reg_address_size, unsigned char *buffer, unsigned char size)
{
	_flgI2CFin = 0;
	i2c_stop();	
	i2c_startSend(slave_address, reg_address, reg_address_size, buffer, size, (cbfI2c)_funcI2CFin);
	while(_flgI2CFin != 1)
	{
		main_clrWDT();
	}
}

/*******************************************************************************
	Routine Name:	DeviceSelection
	Form:			void DeviceSelection( void )
	Parameters:		void
	Return value:	void
	Initialization: SensorPlatformSelection is zeroized.
	Description:	Configures the Port D0..D5 to input and stores their respective
					bits [0..5] to the global char SensorPlatformSelection.
******************************************************************************/
void DeviceSelection(void)
{  
	// SensorPlatformSelection_Temp = 0x00; 
	// SensorPlatformSelection_Temp |= PD0D;
	// SensorPlatformSelection_Temp |= PD1D<<1;
	// SensorPlatformSelection_Temp |= PD2D<<2;
	// SensorPlatformSelection_Temp |= PD3D<<3;
	// SensorPlatformSelection_Temp |= PD4D<<4; 
	// SensorPlatformSelection_Temp |= PD5D<<5; 
	
	// if(SensorPlatformSelection_Temp != SensorPlatformSelection){
		// SensorIntializationFlag = 1;
		// SensorPlatformSelection = SensorPlatformSelection_Temp;
	// }
	if(PDD!=SensorPlatformSelection)
	{
		SensorIntializationFlag = 1;
		SensorPlatformSelection = PDD;
	}
}

/*******************************************************************************
	Routine Name:	SensorInitialization
	Form:			void SensorInitialization( void )
	Parameters:		void
	Return value:	void
	Description:	Holds the SW Statement for Initializing the Sensors
******************************************************************************/
void SensorInitialization(void)
{  
	switch(SensorPlatformSelection){
		case 1:
			Init_Hall_Effect_Sensors_1(); // Refer to function description for list of sensors
			break;
		case 2:
			Init_Hall_Effect_Sensors_2(); // Refer to function description for list of sensors 
			break;
		case 48:			
			Init_Temperature_Sensor_22();
			Init_UV_Sensor_10();
			Init_Hall_Effect_Sensors_2();
			Init_KMX61();
			Init_Pressure_Sensor_3();
			Init_ALS_PROX_Sensor_11();
			Init_Color_Sensor_4();
			Init_KX022();			
			break;
		case 49:
			Init_Pressure_Sensor_3();
			break;
		case 50:
			Init_Color_Sensor_4();
			break;
		case 56:
			Init_ALS_PROX_Sensor_11();
			break;
		case 52:
			Init_KXG03_Sensor_12();
			break;
		case 5: 
			Init_Ambient_Light_Sensor_5(); // Refer to function description for list of sensors
			break;
		case 6:
			Init_Ambient_Light_Sensor_6(); // Refer to function description for list of sensors 
			break;
		case 7:
			Init_Ambient_Light_Sensor_7(); // Refer to function description for list of sensors 
			break;
		case 8:
			Init_Ambient_Light_Sensor_8(); // Refer to function description for list of sensors 
			break;
		case 9:
			Init_Ambient_Light_Sensor_9(); // Refer to function description for list of sensors 
			break;
		case 10:
			Init_UV_Sensor_10(); // Refer to function description for list of sensors 
			break;
		case 15:
			Init_KX022(); // Refer to function description for list of sensors 
			break;
		case 16:
			Init_KMX61(); // Refer to function description for list of sensors 
			break;
		case 20:
			Init_Temperature_Sensor_20(); // Refer to function description for list of sensors 
			break;
		case 21:
			Init_Temperature_Sensor_21(); // Refer to function description for list of sensors
			break;
		case 22:
			Init_Temperature_Sensor_22(); // Refer to function description for list of sensors 
			break;
		case 23:
			Init_Temperature_Sensor_23(); // Refer to function description for list of sensors
			break; 
		default:
			break;
	} 	 
}

/*******************************************************************************
	Routine Name:	MainOp_Hall_Effect_Sensors_1
	Form:			void MainOp_Hall_Effect_Sensors_1(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 1.
	Sensor Platform(s): Hall-Effect Sensor(s)			
						BU52004GUL
******************************************************************************/
void MainOp_Hall_Effect_Sensors_1()
{
	if(SENINTF_HDR1_GPIO0(D)==1 && SENINTF_HDR1_GPIO1(D)==1)
	{
		LEDOUT(0x0);	// Turn off all LEDs
		PRINTF(ESC_ERASE2END "BU52004GUL> Hall – No Mag Fields Detected." ESC_SOL);
	}
	else if(SENINTF_HDR1_GPIO0(D)==1 && SENINTF_HDR1_GPIO1(D)==0)
	{
		LEDOUT(0x02);	// Turn on LED1
		PRINTF(ESC_ERASE2END "BU52004GUL> Hall – North Mag Field Detected." ESC_SOL);
	}
	else if(SENINTF_HDR1_GPIO0(D)==0 && SENINTF_HDR1_GPIO1(D)==1)
	{
		LEDOUT(0x80);	// Turn on LED7
		PRINTF(ESC_ERASE2END "BU52004GUL> Hall – South Mag Field Detected." ESC_SOL);
	}
	else
	{
		LEDOUT(0x82);	// Turn on LED7 and LED1
		PRINTF( "BU52004GUL> Hall – Both Mag Fields Detected." ESC_NEWLINE);
	}
}

/*******************************************************************************
	Routine Name:	MainOp_Hall_Effect_Sensors_2
	Form:			void MainOp_Hall_Effect_Sensors_2(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 2.
	Sensor Platform(s): Hall-Effect Sensors
						BU52011HFV
******************************************************************************/
void MainOp_Hall_Effect_Sensors_2()
{
	if(SENINTF_HDR1_GPIO0(D)==0)
	{
		LEDOUT(0x80);	// Turn on LED7
		PRINTF( ESC_NEWLINE "BU52011HFV> \t\t\t\t= Mag Field Detected.        " ESC_NEWLINE );
	}
	else
	{
		LEDOUT(0x0);	// Turn off all LEDs
		PRINTF( ESC_NEWLINE "BU52011HFV> \t\t\t\t= No Mag Fields Detected.           " ESC_NEWLINE);
	}
}

/*******************************************************************************
	Routine Name:	MainOp_Pressure_Sensor_3()
	Form:			void Pressure_Sensor_3(void);
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor 3 Temperature and Pressure.
	Sensor Platform(s): Pressure and temperature sensor	
						BM1383GLV
******************************************************************************/
void MainOp_Pressure_Sensor_3()
{
	float temperature, pressure, var_int, var_frac,sign;
	
	I2C_Read(BM1383GLV_I2C_ADDR, &BM1383GLV_TEMPERATURE_MSB, 1, uniRawSensorOut._ucharArr, 5);
	
	sign = ((uniRawSensorOut._ucharArr[0] >> 7) == 1) ? -1: 1 ;
	var_frac = ( uniRawSensorOut._ucharArr[1] & 0x1f) * 0.03125 * sign  ; 		//0.03125 = 2^-5 
	var_int  = ((signed char)uniRawSensorOut._ucharArr[0] )<< 3 |(uniRawSensorOut._ucharArr[1] >> 5) ;
	temperature = (var_int + var_frac);///32 ;
	
	var_int  = (uniRawSensorOut._ucharArr[2]<<3) | (uniRawSensorOut._ucharArr[3] >> 5);
	var_frac = ((uniRawSensorOut._ucharArr[3] & 0x1f) << 6 | (uniRawSensorOut._ucharArr[4] >> 2)) * 0.00048828125;  //0.00048828125 = 2^-11
	pressure = (var_int + var_frac);   //question pending here...
	
	printf( ESC_NEWLINE "BM1383> Pressure \t\t\t= %3.4f[hPa], \n\r\tTemperature \t\t\t= %3.4f[C]" ESC_NEWLINE, pressure,temperature);
	//printf(ESC_ERASE2END "BM1383> Temperature = %l[lx]" ESC_SOL, (long)temperature);
}

/*******************************************************************************
	Routine Name:	MainOp_Color_Sensor_4
	Form:			void MainOp_Color_Sensor_4(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 5.
	Sensor Platform(s): Ambient Light Sensor
						
******************************************************************************/
void MainOp_Color_Sensor_4()
{
	int R,G,B,C;
	I2C_Read(BH1745_I2C_ADDR, &BH1745_RED_DATA_LSBs, 1, uniRawSensorOut._ucharArr, 4);		//ucharArr max 6 elens
	R	= 	((int)uniRawSensorOut._ucharArr[0] | ((int)uniRawSensorOut._ucharArr[1])<<8);
	G	= 	((int)uniRawSensorOut._ucharArr[2] | ((int)uniRawSensorOut._ucharArr[3])<<8);
	
	I2C_Read(BH1745_I2C_ADDR, &BH1745_BLUE_DATA_LSBs, 1, uniRawSensorOut._ucharArr, 4);		//ucharArr max 6 elens
	B	= 	((int)uniRawSensorOut._ucharArr[0] | ((int)uniRawSensorOut._ucharArr[1])<<8);
	C	 = 	((int)uniRawSensorOut._ucharArr[2] | ((int)uniRawSensorOut._ucharArr[3])<<8);
		
	printf( ESC_NEWLINE "BH1745> R \t\t\t\t= %8hu, \n\r\tG \t\t\t\t= %8hu, \n\r\tB \t\t\t\t= %8hu, \n\r\tC \t\t\t\t= %8hu" ESC_NEWLINE, (unsigned short)R, (unsigned short)G, (unsigned short)B, (unsigned short)C);

}
/*******************************************************************************
	Routine Name:	MainOp_Ambient_Light_Sensor_5
	Form:			void MainOp_Ambient_Light_Sensor_5(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 5.
	Sensor Platform(s): Ambient Light Sensor
						BH1620FVC
******************************************************************************/
void MainOp_Ambient_Light_Sensor_5()
{	
	uniRawSensorOut._uint = ADC_Read(0);
	// Viout = ADCVal x Vref / (2^10-1)
	flSensorOut[0] = uniRawSensorOut._uint*3.3f/1023;
	// Calculate illuminance (lx)
	//     - H-Gain mode(GC[2-1]=01): Viout = 0.57 x 10-6 x Ev x R1, Ev-max = 1000
	//     - M-Gain mode(GC[2-1]=10): Viout = 0.057 x 10-6 x Ev x R1, Ev-max = 10000
	//     - L-Gain mode(GC[2-1]=11): Viout = 0.0057 x 10-6 x Ev x R1, Ev-max = 100000
	switch(SENINTF_HDR1_GPIO1(D)<<1|SENINTF_HDR1_GPIO0(D))
	{
		case 1: // H-Gain mode
			flSensorOut[0] = flSensorOut[0]/(0.57e-6f*5.6e3f);
			break;
		case 2: // M-Gain mode
			flSensorOut[0] = flSensorOut[0]/(0.057e-6f*5.6e3f);
			break;
		case 3: // L-Gain mode
			flSensorOut[0] = flSensorOut[0]/(0.0057e-6f*5.6e3f);
			break;
		case 0:	// Shutdown
		default:
			flSensorOut[0] = 0;
			break;
	}
	// Scale for 10bits value to 8bits value
	LEDOUT(ReverseBits((unsigned char)(uniRawSensorOut._uint>>2)));
	printf(ESC_ERASE2END "BH1620FVC> Ambient Light = %lu[lx]" ESC_SOL, (unsigned long)flSensorOut[0]);
}

/*******************************************************************************
	Routine Name:	MainOp_Ambient_Light_Sensor_6
	Form:			void MainOp_Ambient_Light_Sensor_6(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 6.
	Sensor Platform(s): Ambient Light Sensor	
						BH1710FVC
******************************************************************************/
void MainOp_Ambient_Light_Sensor_6()
{
	// Wait to complete measurement
	//     - Max H-Resolution Mode Measurement Time: 180ms
	//     - Max M-Resolution Mode Measurement Time: 24ms
	//     - Max L-Resolution Mode Measurement Time: 4.5ms
	// NOPms(24);
	I2C_Read(BH17xxFVC_ADDR_1, NULL, 0, uniRawSensorOut._ucharArr, 2);
	
	// Calculate illuminance (lx)
	//     Measurement Accuracy (Typ. 1.2) = Sensor out / Actual Illuminance 
	flSensorOut[0] = (uniRawSensorOut._ucharArr[0]<<8|uniRawSensorOut._ucharArr[1])/1.2f;
	
	LEDOUT(ReverseBits(uniRawSensorOut._ucharArr[0]));
	printf(ESC_ERASE2END "BH1710FVC> Ambient Light = %lu[lx]" ESC_SOL, (unsigned long)flSensorOut[0]);
}

/*******************************************************************************
	Routine Name:	MainOp_Ambient_Light_Sensor_7
	Form:			void MainOp_Ambient_Light_Sensor_7(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 7.
	Sensor Platform(s): Ambient Light Sensor		
						BH1730FVC	
******************************************************************************/
void MainOp_Ambient_Light_Sensor_7()
{
	// Wait to complete measurement
	//     - Max Measurement Time is 150ms @ TIMING=0xDA
	// NOPms(150);
	
	// Start read data with start address is DATA0LOW
	I2C_Read(BH17xxFVC_ADDR_2, NULL, 0, uniRawSensorOut._ucharArr, 4);
	
	// Calculate illuminance (lx)
	//     - DATA1/DATA0<0.26: Lx = ( 1.290*DATA0 - 2.733*DATA1 ) / GAIN * 100ms / TIMING
	//     - DATA1/DATA0<0.55: Lx = ( 0.795*DATA0 - 0.859*DATA1 ) / GAIN * 100ms / TIMING
	//     - DATA1/DATA0<1.09: Lx = ( 0.510*DATA0 - 0.345*DATA1 ) / GAIN * 100ms / TIMING
	//     - DATA1/DATA0<2.13: Lx = ( 0.276*DATA0 - 0.130*DATA1 ) / GAIN * 100ms / TIMING
	flSensorOut[0] = (float)uniRawSensorOut._uintArr[1]/uniRawSensorOut._uintArr[0];
	if(flSensorOut[0]<0.26f)
		flSensorOut[0] = (1.290f*uniRawSensorOut._uintArr[0]-2.733f*uniRawSensorOut._uintArr[1])/1*100/218;
	else if(flSensorOut[0]<0.55f)
		flSensorOut[0] = (0.795f*uniRawSensorOut._uintArr[0]-0.859f*uniRawSensorOut._uintArr[1])/1*100/218;
	else if(flSensorOut[0]<1.09f)
		flSensorOut[0] = (0.510f*uniRawSensorOut._uintArr[0]-0.345f*uniRawSensorOut._uintArr[1])/1*100/218;
	else if(flSensorOut[0]<2.13f)
		flSensorOut[0] = (0.276f*uniRawSensorOut._uintArr[0]-0.130f*uniRawSensorOut._uintArr[1])/1*100/218;
	else
		flSensorOut[0] = 0;
	
	LEDOUT(ReverseBits(uniRawSensorOut._ucharArr[1]));
	printf(ESC_ERASE2END "BH1730FVC> Ambient Light = %lu[lx]" ESC_SOL, (unsigned long)flSensorOut[0]);
}

/*******************************************************************************
	Routine Name:	MainOp_Ambient_Light_Sensor_8
	Form:			void MainOp_Ambient_Light_Sensor_8(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 8.
	Sensor Platform(s): Ambient Light Sensor
						BH1721FVC
******************************************************************************/
void MainOp_Ambient_Light_Sensor_8()
{	
	// Wait to complete measurement
	//     - Max Auto/H-Resolution Mode Measurement Time: 180ms
	//     - Max L-Resolution Mode Measurement Time: 24ms
	// NOPms(180);
	I2C_Read(BH17xxFVC_ADDR_1, NULL, 0, uniRawSensorOut._ucharArr, 2);
	
	// Calculate illuminance (lx)
	//     Measurement Accuracy (Typ. 1.2) = Sensor out / Actual Illuminance 
	flSensorOut[0] = (uniRawSensorOut._ucharArr[0]<<8|uniRawSensorOut._ucharArr[1])/1.2f;
	
	LEDOUT(ReverseBits(uniRawSensorOut._ucharArr[0]));
	printf(ESC_ERASE2END "BH1721FVC> Ambient Light = %lu[lx]" ESC_SOL, (unsigned long)flSensorOut[0]);
}

/*******************************************************************************
	Routine Name:	MainOp_Ambient_Light_Sensor_9
	Form:			void MainOp_Ambient_Light_Sensor_9(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 9.
	Sensor Platform(s): Ambient Light Sensor	
						BH1780GLI
******************************************************************************/
void MainOp_Ambient_Light_Sensor_9()
{
	// Wait to complete measurement
	//     - Max Measurement Time is 250ms
	// NOPms(250);
	
	// Start read data with start address is DATALOW
	I2C_Read(BH17xxFVC_ADDR_2, NULL, 0, uniRawSensorOut._ucharArr, 2);
	
	// Calculate illuminance (lx)
	//     Measurement Accuracy (Typ. 1) = Sensor out / Actual Illuminance
	flSensorOut[0] = uniRawSensorOut._uint;
	
	LEDOUT(ReverseBits(uniRawSensorOut._ucharArr[1]));
	printf(ESC_ERASE2END "BH1780GLI> Ambient Light = %lu[lx]" ESC_SOL, (unsigned long)flSensorOut[0]);
}

/*******************************************************************************
	Routine Name:	MainOp_UV_Sensor_10
	Form:			void MainOp_UV_Sensor_10(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 10.
	Sensor Platform(s): UV Sensor
						ML8511
******************************************************************************/
void MainOp_UV_Sensor_10()
{
	uniRawSensorOut._uint = ADC_Read(1);
	// Vsenout = ADCVal x Vref / (2^10-1)
	flSensorOut[0] = uniRawSensorOut._uint*3.3f/1023;
	// Calculate UV Intensity (mW/cm2)
	flSensorOut[1] = Voltage2UVIntensity(flSensorOut[0]);
	// Scale for 10bits value to 8bits value
	LEDOUT(ReverseBits((unsigned char)(uniRawSensorOut._uint>>2)));
	printf( "\n\rML8511> UV Intensity \t\t\t= %.02f[mW/cm2]. \n\r\tVsenout \t\t\t= %.02f[V] \n\r" , flSensorOut[1], flSensorOut[0]);
}

/*******************************************************************************
	Routine Name:	MainOp_ALS_PROX_Sensor_11
	Form:			void MainOp_ALS_PROX_Sensor_11(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 10.
	Sensor Platform(s): RPR-0521
						
******************************************************************************/
void MainOp_ALS_PROX_Sensor_11()
{
	unsigned short Prox, ALS;
	I2C_Read(RPR0521_I2C_ADDR, &RPR0521_Prox_PS_LSB, 1, uniRawSensorOut._ucharArr, 8);
	Prox	= 	((unsigned short)uniRawSensorOut._ucharArr[0] | ((unsigned short)uniRawSensorOut._ucharArr[1])<<8);
	ALS		= 	((unsigned short)uniRawSensorOut._ucharArr[2] | ((unsigned short)uniRawSensorOut._ucharArr[3])<<8);
			
	printf( ESC_NEWLINE "RPR0521> Prox \t\t\t\t= %10lu[raw], \n\r\tALS \t\t\t\t= %10lu[raw]" ESC_NEWLINE, (long)Prox, (long)ALS);
}

/*******************************************************************************
	Routine Name:	MainOp_KXG03_Sensor_122
	Form:			void MainOp_KXG03_Sensor_12()
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 12.
	Sensor Platform(s): 3-axis accelerometer &3-axis Gyro	
						KXG03
******************************************************************************/
void MainOp_KXG03_Sensor_12()
{
	// Wait 20ms(50Hz) to data updated
	// NOPms(20);
	
	// Start read accelerometer output with start address is XOUT_L
	I2C_Read(KXG03_I2C_ADDR, &KXG03_GYRO_XOUT_L, 1, uniRawSensorOut._ucharArr, 6);
	
	// Calculate GYRO info : byte order X_L,X_H, Y_L,Y_H,Z_L,Z_H
	flSensorOut[0] = (float)uniRawSensorOut._intArr[0]*2048/32768.0f;	// GX
	flSensorOut[1] = (float)uniRawSensorOut._intArr[1]*2048/32768.0f;	// GY
	flSensorOut[2] = (float)uniRawSensorOut._intArr[2]*2048/32768.0f;	// GZ
	
	printf(ESC_ERASE2END "KXG03> GyroX_raw = %d, GyroX_scaled = %.05f[g], GyroY_raw = %d, GyroY_scaled = %.05f[g], GyroZ_raw = %d, GyroZ_scaled = %.05f[g]" ESC_SOL,
		uniRawSensorOut._intArr[0], flSensorOut[0], uniRawSensorOut._intArr[1], flSensorOut[1], uniRawSensorOut._intArr[2], flSensorOut[2]);
	
	// Start read Accelerometer position with start address is TSCP
	I2C_Read(KXG03_I2C_ADDR, &KXG03_ACC_XOUT_L, 1, uniRawSensorOut._ucharArr, 6);
	
	// Calculate acceleration
	flSensorOut[0] = (float)uniRawSensorOut._intArr[0]*2/2048.0f;	// AX
	flSensorOut[1] = (float)uniRawSensorOut._intArr[1]*2/2048.0f;	// AY
	flSensorOut[2] = (float)uniRawSensorOut._intArr[2]*2/2048.0f;	// AZ
	
	printf(ESC_ERASE2END "KX022> AccelX_raw = %d, AccelX_scaled = %.05f[g], AccelY_raw = %d, AccelY_scaled = %.05f[g], AccelZ_raw = %d, AccelZ_scaled = %.05f[g]" ESC_SOL,
		uniRawSensorOut._intArr[0], flSensorOut[0], uniRawSensorOut._intArr[1], flSensorOut[1], uniRawSensorOut._intArr[2], flSensorOut[2]);
	
}

/*******************************************************************************
	Routine Name:	MainOp_KX022
	Form:			void MainOp_KX022(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 15.
	Sensor Platform(s): 3-axis accelerometer	
						KX022
******************************************************************************/
void MainOp_KX022()
{
	// Wait 20ms(50Hz) to data updated
	// NOPms(20);
	
	// Start read accelerometer output with start address is XOUT_L
	I2C_Read(KX022_I2C_ADDR, &KX022_XOUTL, 1, uniRawSensorOut._ucharArr, 6);
	
	// Calculate acceleration
	flSensorOut[0] = (float)uniRawSensorOut._intArr[0]/16384.0f;	// X
	flSensorOut[1] = (float)uniRawSensorOut._intArr[1]/16384.0f;	// Y
	flSensorOut[2] = (float)uniRawSensorOut._intArr[2]/16384.0f;	// Y
	
	printf(ESC_NEWLINE "KX022> AccelX_raw \t\t\t= %d, \n\r\tAccelX_scaled \t\t\t= %.05f[g], \n\r\tAccelY_raw \t\t\t= %d, \n\r\tAccelY_scaled \t\t\t= %.05f[g], \n\r\tAccelZ_raw \t\t\t= %d, \n\r\tAccelZ_scaled \t\t\t= %.05f[g]" ESC_NEWLINE,
		uniRawSensorOut._intArr[0], flSensorOut[0], uniRawSensorOut._intArr[1], flSensorOut[1], uniRawSensorOut._intArr[2], flSensorOut[2]);
	
	// Start read tilt position with start address is TSCP
	I2C_Read(KX022_I2C_ADDR, &KX022_TSCP, 1, uniRawSensorOut._ucharArr, 2);

	switch(uniRawSensorOut._ucharArr[0])
	{
		case 0x01u:	LEDOUT(0x1u<<4); break;	// FU: Face-Up State (Z+). Turn on LED4
		case 0x02u:	LEDOUT(0x1u<<3); break;	// FD: Face-Down State (Z-). Turn on LED3
		case 0x04u:	LEDOUT(0x1u<<2); break;	// UP: Up State (Y+). Turn on LED2
		case 0x08u:	LEDOUT(0x1u<<5); break;	// DO: Down State (Y-). Turn on LED5
		case 0x10u:	LEDOUT(0x1u<<7); break;	// RI: Right State (X+). Turn on LED7
		case 0x20u:	LEDOUT(0x1u<<0); break;	// LE: Left State (X-). Turn on LED0
		default:	LEDOUT(0x00u); break;	// NONE
	}
}

/*******************************************************************************
	Routine Name:	MainOp_KMX61
	Form:			void MainOp_KMX61( void )
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 16.
	Sensor Platform(s): 6-Axis Accelerometer/Magnetometer
						KMX61
******************************************************************************/
void MainOp_KMX61()
{ 
	// Start read accelerometer output data with start address is ACCEL_XOUT_L
	I2C_Read(KMX61_I2C_ADDR, &KMX61_ACCEL_XOUT_L, 1, uniRawSensorOut._ucharArr, 6);
	uniRawSensorOut._intArr[0] = uniRawSensorOut._intArr[0]>>2;
	uniRawSensorOut._intArr[1] = uniRawSensorOut._intArr[1]>>2;
	uniRawSensorOut._intArr[2] = uniRawSensorOut._intArr[2]>>2;
	// Calculate acceleration.
	flSensorOut[0] = (float)uniRawSensorOut._intArr[0]/2048.0f;	// X
	flSensorOut[1] = (float)uniRawSensorOut._intArr[1]/2048.0f;	// Y
	flSensorOut[2] = (float)uniRawSensorOut._intArr[2]/2048.0f;	// Z
	printf( ESC_NEWLINE "KMX62> AccelX_raw \t\t\t= %10d, \n\r\tAccelX_scaled \t\t\t= %1.6f[g], \n\r\tAccelY_raw \t\t\t= %10d, \n\r\tAccelY_scaled \t\t\t= %1.6f[g], \n\r\tAccelZ_raw \t\t\t= %10d, \n\r\tAccelZ_scaled \t\t\t= %1.6f[g]" ESC_NEWLINE,
		uniRawSensorOut._intArr[0], flSensorOut[0], uniRawSensorOut._intArr[1], flSensorOut[1], uniRawSensorOut._intArr[2], flSensorOut[2]);
	
	if(-0.5f<flSensorOut[0] && flSensorOut[0]<0.5f && 0.5f<flSensorOut[1])
		LEDOUT(0x1u<<5);	// -30<angle(X,1g)<30 & angle(Y,1g)>30. Turn on LED5
	else if(0.5f<flSensorOut[0] && -0.5f<flSensorOut[1] && flSensorOut[1]<0.5f)
		LEDOUT(0x1u<<0);	// 30<angle(X,1g) & -30<angle(Y,1g)<30. Turn on LED0
	else if(-0.5f<flSensorOut[0] && flSensorOut[0]<0.5f && flSensorOut[1]<-0.5f)
		LEDOUT(0x1u<<2);	// -30<angle(X,1g)<30 & angle(Y,1g)<-30. Turn on LED2
	else if(flSensorOut[0]<-0.5f && -0.5f<flSensorOut[1] && flSensorOut[1]<0.5f)
		LEDOUT(0x1u<<7);	// angle(X,1g)<-30 & -30<angle(Y,1g)<30. Turn on LED7
	else if(flSensorOut[2]>0.5f)
		LEDOUT(0x1u<<4);	// angle(Z,1g)>30. Turn on LED4
	else if(flSensorOut[2]<-0.5f)
		LEDOUT(0x1u<<3);	// angle(Z,1g)<-30. Turn on LED3
	
	// Start read magnetometer output data with start address is MAG_XOUT_L
	I2C_Read(KMX61_I2C_ADDR, &KMX62_MAG_XOUT_L, 1, uniRawSensorOut._ucharArr, 6);
	uniRawSensorOut._intArr[0] = uniRawSensorOut._intArr[0]>>2;
	uniRawSensorOut._intArr[1] = uniRawSensorOut._intArr[1]>>2;
	uniRawSensorOut._intArr[2] = uniRawSensorOut._intArr[2]>>2;
	// Calculate magnetic
	flSensorOut[0] = (float)uniRawSensorOut._intArr[0]*0.146f;	// X
	flSensorOut[1] = (float)uniRawSensorOut._intArr[1]*0.146f;	// Y
	flSensorOut[2] = (float)uniRawSensorOut._intArr[2]*0.146f;	// Y
	printf("KMX62> MagX_raw = %d, \n\r\tMagX_scaled \t\t\t= %.05f[uT], \n\r\tMagY_raw \t\t\t= %d, \n\r\tMagY_scaled \t\t\t= %.05f[uT], \n\r\tMagZ_raw \t\t\t= %d, \n\r\tMagZ_scaled \t\t\t= %.05f[uT]" ESC_NEWLINE,
		uniRawSensorOut._intArr[0], flSensorOut[0], uniRawSensorOut._intArr[1], flSensorOut[1], uniRawSensorOut._intArr[2], flSensorOut[2]);
}

/*******************************************************************************
	Routine Name:	MainOp_Temperature_Sensor_20
	Form:			void MainOp_Temperature_Sensor_20(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 20.
	Sensor Platform(s): Temperature Sensors	
						BD1020HFV
******************************************************************************/
void MainOp_Temperature_Sensor_20()
{
	uniRawSensorOut._uint = ADC_Read(0);
	// Vsenout = ADCVal x Vref / (2^10-1)
	flSensorOut[0] = uniRawSensorOut._uint*3.3f/1023;
	// Calculate Temperature (°C)
	flSensorOut[1] = Voltage2Temperature(flSensorOut[0], 1.3f, 30.0f, -0.0082f);
	// Scale for 10bits value to 8bits value
	LEDOUT(((unsigned char)(uniRawSensorOut._uint>>2)));
	printf( ESC_NEWLINE "BD1020HFV> Temperature = %6.6f[°C]. Vsenout = %6.6f[V]" ESC_NEWLINE, flSensorOut[1], flSensorOut[0]);
}

/*******************************************************************************
	Routine Name:	MainOp_Temperature_Sensor_21
	Form:			void MainOp_Temperature_Sensor_21(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 21.
	Sensor Platform(s): Temperature Sensors
						BDJ0601HFV					
******************************************************************************/
void MainOp_Temperature_Sensor_21()
{
	uniRawSensorOut._uint = ADC_Read(0);
	// Vsenout = ADCVal x Vref / (2^10-1)
	flSensorOut[0] = uniRawSensorOut._uint*3.3f/1023;
	// Calculate Temperature (°C)
	flSensorOut[1] = Voltage2Temperature(flSensorOut[0], 1.3f, 30.0f, -0.0082f);
	// Scale for 10bits value to 7bits value
	LEDOUT(ReverseBits((unsigned char)(uniRawSensorOut._uint>>3)));
	printf(ESC_ERASE2END "BDJ0601HFV> Temperature = %.02f[°C]. Vsenout = %.02f[V]", flSensorOut[1], flSensorOut[0]);
	if(SENINTF_HDR1_GPIO0(D)==1)
		PRINTF(ESC_NEWLINE "BDJ0601HFV> Temperature Threshold Reached." ESC_PREVLINE);
	else
		PRINTF(ESC_NEWLINE "BDJ0601HFV> Temperature Threshold Not Reached." ESC_PREVLINE);
}

/*******************************************************************************
	Routine Name:	MainOp_Temperature_Sensor_22
	Form:			void MainOp_Temperature_Sensor_22(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 22.
	Sensor Platform(s): Temperature Sensors
						BDE0600G	
******************************************************************************/
void MainOp_Temperature_Sensor_22()
{
	uniRawSensorOut._uint = ADC_Read(0);
	// Vsenout = ADCVal x Vref / (2^10-1)
	flSensorOut[0] = uniRawSensorOut._uint*3.3f/1023;
	// Calculate Temperature (°C)
	flSensorOut[1] = Voltage2Temperature(flSensorOut[0], 1.753f, 30.0f, -0.01068f);
	// Scale for 10bits value to 7bits value
	LEDOUT(ReverseBits((unsigned char)(uniRawSensorOut._uint>>3)));
	printf(ESC_STX "BDE0600G> Temperature \t\t\t= %.02f[C]." ESC_NEWLINE, flSensorOut[1]);
	/*if(SENINTF_HDR1_GPIO0(D)==0)
		PRINTF(ESC_NEWLINE "BDE0600G> Temperature Threshold Reached." ESC_PREVLINE);
	else
		PRINTF(ESC_NEWLINE "BDE0600G> Temperature Threshold Not Reached." ESC_PREVLINE);*/
}

/*******************************************************************************
	Routine Name:	MainOp_Temperature_Sensor_23
	Form:			void MainOp_Temperature_Sensor_23(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 23.
	Sensor Platform(s): Temperature Sensors
						BDJ0550HFV
******************************************************************************/
void MainOp_Temperature_Sensor_23()
{
	uniRawSensorOut._uint = ADC_Read(0);
	// Vsenout = ADCVal x Vref / (2^10-1)
	flSensorOut[0] = uniRawSensorOut._uint*3.3f/1023;
	// Calculate Temperature (°C)
	flSensorOut[1] = Voltage2Temperature(flSensorOut[0], 1.3f, 30.0f, -0.0082f);
	// Scale for 10bits value to 7bits value
	LEDOUT(ReverseBits((unsigned char)(uniRawSensorOut._uint>>3)));
	printf(ESC_ERASE2END "BDJ0550HFV> Temperature = %.02f[°C]. Vsenout = %.02f[V]", flSensorOut[1], flSensorOut[0]);
	if(SENINTF_HDR1_GPIO0(D)==0)
		PRINTF(ESC_NEWLINE "BDJ0550HFV> Temperature Threshold Reached." ESC_PREVLINE);
	else
		PRINTF(ESC_NEWLINE "BDJ0550HFV> Temperature Threshold Not Reached." ESC_PREVLINE);
}

/*******************************************************************************
	Routine Name:	Init_Hall_Effect_Sensors_1
	Form:			void Init_Hall_Effect_Sensors_1(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 1.
	Sensor Platform(s): Hall-Effect Sensor(s)			
						BU52004GUL
******************************************************************************/
void Init_Hall_Effect_Sensors_1()
{
	// Configure pins GPIO0, GPIO1 of Sensor Interface Header 1 are input with a pull-up resistor
	SENINTF_HDR1_GPIO0(DIR) = 1;
	SENINTF_HDR1_GPIO1(DIR) = 1;
	
	SENINTF_HDR1_GPIO0(C0) = 0;
	SENINTF_HDR1_GPIO0(C1) = 1;
	SENINTF_HDR1_GPIO1(C0) = 0;
	SENINTF_HDR1_GPIO1(C1) = 1;
	
	SENINTF_HDR1_GPIO0(MD0) = 0;
	SENINTF_HDR1_GPIO0(MD1) = 0;
	SENINTF_HDR1_GPIO1(MD0) = 0;
	SENINTF_HDR1_GPIO1(MD1) = 0;
}

/*******************************************************************************
	Routine Name:	Init_Hall_Effect_Sensors_2
	Form:			void Init_Hall_Effect_Sensors_2(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Initialize Hall Effect Sensor 2
	Sensor Platform(s): Hall-Effect Sensors	
						BU52011HFV
******************************************************************************/
void Init_Hall_Effect_Sensors_2()
{
	// Configure pins GPIO0 of Sensor Interface Header 1 is input with a pull-up resistor
	SENINTF_HDR1_GPIO0(DIR) = 1;
	
	SENINTF_HDR1_GPIO0(C0) = 0;
	SENINTF_HDR1_GPIO0(C1) = 1;
	
	SENINTF_HDR1_GPIO0(MD0) = 0;
	SENINTF_HDR1_GPIO0(MD1) = 0;
	//
	SENINTF_HDR1_GPIO1(DIR) = 1;
	
	SENINTF_HDR1_GPIO1(C0) = 0;
	SENINTF_HDR1_GPIO1(C1) = 1;
	
	SENINTF_HDR1_GPIO1(MD0) = 0;
	SENINTF_HDR1_GPIO1(MD1) = 0;
}

/*******************************************************************************
	Routine Name:	Init_Pressure_Sensor_3()
	Form:			void Init_Pressure_Sensor_3()
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Initialize Pressure/Temp sensor
	Sensor Platform(s): Pressure/Temp Sensors	
						BM1383GLV
******************************************************************************/
void Init_Pressure_Sensor_3()
{
	I2C_Write(BM1383GLV_I2C_ADDR,  &BM1383GLV_POWER_DOWN,  1,  &BM1383GLV_POWER_ON, 1);
	I2C_Write(BM1383GLV_I2C_ADDR,  &BM1383GLV_SLEEP,  1,  &BM1383GLV_SLEEP_ACT, 1);
	I2C_Write(BM1383GLV_I2C_ADDR,  &BM1383GLV_MODE_CONTROL,  1,  &BM1383GLV_MODE_CONTROL_REG, 1);
}

/*******************************************************************************
	Routine Name:	Init_Color_Sensor_4()
	Form:			void Init_Color_Sensor_4()
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Initialize Pressure/Temp sensor
	Sensor Platform(s):Color Sensors	
						BH1745NUC
******************************************************************************/

void Init_Color_Sensor_4()
{
	I2C_Write(BH1745_I2C_ADDR, &BH1745_PERSISTENCE  , 1, 0x03, 1);		// see data sheet pg. 9
	I2C_Write(BH1745_I2C_ADDR, &BH1745_MODE_CONTROL1, 1, 0x00, 1);  	//000 : 160msec
	I2C_Write(BH1745_I2C_ADDR, &BH1745_MODE_CONTROL2, 1, 0x92, 1); 		//16x gain, RGBC_EN
	I2C_Write(BH1745_I2C_ADDR, &BH1745_MODE_CONTROL3, 1, 0x02, 1);		//default 0x02
}

/*******************************************************************************
	Routine Name:	Init_Ambient_Light_Sensor_5
	Form:			void Init_Ambient_Light_Sensor_5(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 5.
	Sensor Platform(s): Ambient Light Sensor
						BH1620FVC
******************************************************************************/
void Init_Ambient_Light_Sensor_5()
{
	// Configure pins GPIO0, GPIO1 of Sensor Interface Header 1 are input with a pull-up resistor
	SENINTF_HDR1_GPIO0(DIR) = 1;
	SENINTF_HDR1_GPIO1(DIR) = 1;
	
	SENINTF_HDR1_GPIO0(C0) = 0;
	SENINTF_HDR1_GPIO0(C1) = 1;
	SENINTF_HDR1_GPIO1(C0) = 0;
	SENINTF_HDR1_GPIO1(C1) = 1;
	
	SENINTF_HDR1_GPIO0(MD0) = 0;
	SENINTF_HDR1_GPIO0(MD1) = 0;
	SENINTF_HDR1_GPIO1(MD0) = 0;
	SENINTF_HDR1_GPIO1(MD1) = 0;
}

/*******************************************************************************
	Routine Name:	Init_Ambient_Light_Sensor_6
	Form:			void Init_Ambient_Light_Sensor_6(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 6.
	Sensor Platform(s): Ambient Light Sensor	
						BH1710FVC
******************************************************************************/
void Init_Ambient_Light_Sensor_6()
{
	// Power on sensor
	I2C_Write(BH17xxFVC_ADDR_1, NULL, 0, &BH17xxFVC_PWR_ON, 1);
	// Set Continuously M-Resolution Mode
	I2C_Write(BH17xxFVC_ADDR_1, NULL, 0, &BH1710FVC_CONT_M_RES_MOD, 1);
	// Wait to complete measurement
	//     - Max H-Resolution Mode Measurement Time: 180ms
	//     - Max M-Resolution Mode Measurement Time: 24ms
	//     - Max L-Resolution Mode Measurement Time: 4.5ms
	NOPms(24);
}

/*******************************************************************************
	Routine Name:	Init_Ambient_Light_Sensor_7
	Form:			void Init_Ambient_Light_Sensor_7(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 7.
	Sensor Platform(s): Ambient Light Sensor		
						BH1730FVC	
******************************************************************************/
void Init_Ambient_Light_Sensor_7()
{
	unsigned char cTmp = 0x0;
	
	// Disable interrupt function
	I2C_Write(BH17xxFVC_ADDR_2, &BH1730FVC_REG_INTERRUPT, 1, &cTmp, 1);
	// Set GAIN mode is X1
	I2C_Write(BH17xxFVC_ADDR_2, &BH1730FVC_REG_GAIN, 1, &cTmp, 1);
	// Set measurement time is 0xDA (102.6ms)
	cTmp = 0xdau;
	I2C_Write(BH17xxFVC_ADDR_2, &BH1730FVC_REG_TIMING, 1, &cTmp, 1);
	// Configure ADC measurement is continuous, Type0 and Type1 and start measurement
	cTmp = 0x3;
	I2C_Write(BH17xxFVC_ADDR_2, &BH1730FVC_REG_CONTROL, 1, &cTmp, 1);
	// Update value of Command register to read measurement value
	I2C_Write(BH17xxFVC_ADDR_2, &BH1730FVC_REG_DATA0LOW, 1, NULL, 0);
}

/*******************************************************************************
	Routine Name:	Init_Ambient_Light_Sensor_8
	Form:			void Init_Ambient_Light_Sensor_8(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 8.
	Sensor Platform(s): Ambient Light Sensor
						BH1721FVC
******************************************************************************/
void Init_Ambient_Light_Sensor_8()
{	
	// Power on sensor
	I2C_Write(BH17xxFVC_ADDR_1, NULL, 0, &BH17xxFVC_PWR_ON, 1);
	// Set Continuously M-Resolution Mode
	I2C_Write(BH17xxFVC_ADDR_1, NULL, 0, &BH1721FVC_CONT_A_RES_MOD, 1);
	// Wait to complete measurement
	//     - Max Auto/H-Resolution Mode Measurement Time: 180ms
	//     - Max L-Resolution Mode Measurement Time: 24ms
	NOPms(180);	
}

/*******************************************************************************
	Routine Name:	Init_Ambient_Light_Sensor_9
	Form:			void Init_Ambient_Light_Sensor_9(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 9.
	Sensor Platform(s): Ambient Light Sensor	
						BH1780GLI
******************************************************************************/
void Init_Ambient_Light_Sensor_9()
{
	uniRawSensorOut._uchar = 0x3;
	// Power on sensor
	I2C_Write(BH17xxFVC_ADDR_2, &BH1780GLI_REG_CONTROL, 1, &uniRawSensorOut._uchar, 1);
	// Update value of Command register to read measurement value
	I2C_Write(BH17xxFVC_ADDR_2, &BH1780GLI_REG_DATALOW, 1, NULL, 0);
}

/*******************************************************************************
	Routine Name:	Init_UV_Sensor_10
	Form:			void Init_UV_Sensor_10(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 10.
	Sensor Platform(s): UV-Sensor
						ML8511
******************************************************************************/
void Init_UV_Sensor_10()
{
	// Do nothing!
	// All configures (ADC0) are completed in Initialization() function.
}

/*******************************************************************************
	Routine Name:	Init_ALS_PROX_Sensor_11()
	Form:			void Init_ALS_PROX_Sensor_11()
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 9.
	Sensor Platform(s): Ambient Light Sensor/Prox Sensor	
						RPR-0521
******************************************************************************/

void Init_ALS_PROX_Sensor_11()
{
	I2C_Write(RPR0521_I2C_ADDR, &RPR0521_Prox_ModeCTR, 1, &RPR0521_Prox_ModeCTR_CFG, 1); 	        //0xC6: ALS=PS= enabled & 100ms 
	I2C_Write(RPR0521_I2C_ADDR, &RPR0521_PS_ModeCTR, 1, &RPR0521_PS_ModeCTR_CFG, 1); 	        //(0x01) defaults, PS GAIN ×4	
}

/*******************************************************************************
	Routine Name:	Init_KXG03
	Form:			void Init_KXG03_Sensor_12()
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 12.
	Sensor Platform(s): 3-axis accelerometer	 & 3-axis gyro
						KXG03
******************************************************************************/

void Init_KXG03_Sensor_12()
{
	I2C_Write(KXG03_I2C_ADDR, &KXG03_STBY_REG, 1, &KXG03_STBY_SET, 1);
	I2C_Write(KX022_I2C_ADDR, &KXG03_CTRL_REG1, 1, &KXG03_CTRL_REG1_CFDAT, 1);
	I2C_Write(KX022_I2C_ADDR, &KXG03_CTRL_REG2, 1, &KXG03_CTRL_REG2_CFDAT, 1);
	I2C_Write(KX022_I2C_ADDR, &KXG03_ODCTRL, 1, &KXG03_ODCTRL_CFDAT, 1);
	
	// Set accelerometer to operating mode (PC1=1)
	uniRawSensorOut._uchar = (unsigned char)(KXG03_STBY_SET&0xfc);
	I2C_Write(KX022_I2C_ADDR, &KX022_CNTL1, 1, &uniRawSensorOut._uchar, 1);
	
	
}

/*******************************************************************************
	Routine Name:	Init_KX022
	Form:			void Init_KX022(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 15.
	Sensor Platform(s): 3-axis accelerometer	
						KX022
******************************************************************************/
void Init_KX022()
{
	I2C_Write(KX022_I2C_ADDR, &KX022_CNTL1, 1, &KX022_CNTL1_CFGDAT, 1);
	I2C_Write(KX022_I2C_ADDR, &KX022_ODCNTL, 1, &KX022_ODCNTL_CFGDAT, 1);
	I2C_Write(KX022_I2C_ADDR, &KX022_CNTL3, 1, &KX022_CNTL3_CFGDAT, 1);
	I2C_Write(KX022_I2C_ADDR, &KX022_TILT_TIMER, 1, &KX022_TILT_TIMER_CFGDAT, 1);
	// Set accelerometer to operating mode (PC1=1)
	uniRawSensorOut._uchar = (unsigned char)(KX022_CNTL1_CFGDAT|0x80);
	I2C_Write(KX022_I2C_ADDR, &KX022_CNTL1, 1, &uniRawSensorOut._uchar, 1);
}

/*******************************************************************************
	Routine Name:	Init_KMX61
	Form:			void Init_KMX61( void )
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 16.
	Sensor Platform(s): 6-Axis Accelerometer/Magnetometer
						KMX61
******************************************************************************/
void Init_KMX61()
{ 

	I2C_Write(KMX61_I2C_ADDR, &KMX62_CNTL2, 1, &KMX62_CNTL2_CFG, 1);

/*
	// Set accelerometer and magnetometer to stand-by mode
	uniRawSensorOut._uchar = 0x03u;
	I2C_Write(KMX61_I2C_ADDR, &KMX61_STBY_REG, 1, &uniRawSensorOut._uchar, 1);
	// Configure
	I2C_Write(KMX61_I2C_ADDR, &KMX61_SELF_TEST, 1, &KMX61_SELF_TEST_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_CNTL1, 1, &KMX61_CNTL1_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_CNTL2, 1, &KMX61_CNTL2_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_ODCNTL, 1, &KMX61_ODCNTL_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_TEMP_EN_CNTL, 1, &KMX61_TEMP_EN_CNTL_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_BUF_CTRL1, 1, &KMX61_BUF_CTRL1_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_BUF_CTRL2, 1, &KMX61_BUF_CTRL2_CFGDAT, 1);
	// Set accelerometer and magnetometer to operating mode
	uniRawSensorOut._uchar = 0x0u;
	I2C_Write(KMX61_I2C_ADDR, &KMX61_STBY_REG, 1, &uniRawSensorOut._uchar, 1);*/
}

/*******************************************************************************
	Routine Name:	Temperature_Sensor_20
	Form:			void Temperature_Sensor_20(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 20.
	Sensor Platform(s): Temperature Sensors	
						BD1020HFV
******************************************************************************/
void Init_Temperature_Sensor_20()
{
	// Do nothing!
	// All configures (ADC0) are completed in Initialization() function.
}

/*******************************************************************************
	Routine Name:	Init_Temperature_Sensor_21
	Form:			void Init_Temperature_Sensor_21(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 21.
	Sensor Platform(s): Temperature Sensors
						BDJ0601HFV				******************************/
void Init_Temperature_Sensor_21()
{
	// Configure pins GPIO0 of Sensor Interface Header 1 is input with a pull-up resistor
	SENINTF_HDR1_GPIO0(DIR) = 1;
	
	SENINTF_HDR1_GPIO0(C0) = 0;
	SENINTF_HDR1_GPIO0(C1) = 1;
	
	SENINTF_HDR1_GPIO0(MD0) = 0;
	SENINTF_HDR1_GPIO0(MD1) = 0;
}

/*******************************************************************************
	Routine Name:	Init_Temperature_Sensor_22
	Form:			void Init_Temperature_Sensor_22(void)
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 22.
	Sensor Platform(s): Temperature Sensors
						BDE0600G	
******************************************************************************/
void Init_Temperature_Sensor_22()
{
	// Configure pins GPIO0 of Sensor Interface Header 1 is input with a pull-up resistor
	SENINTF_HDR1_GPIO0(DIR) = 1;
	
	SENINTF_HDR1_GPIO0(C0) = 0;
	SENINTF_HDR1_GPIO0(C1) = 1;
	
	SENINTF_HDR1_GPIO0(MD0) = 0;
	SENINTF_HDR1_GPIO0(MD1) = 0;
}

/*******************************************************************************
	Routine Name:	Init_Temperature_Sensor_23
	Form:			void Init_Temperature_Sensor_23( void )
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 23.
	Sensor Platform(s): Temperature Sensors
						BDJ0550HFV
******************************************************************************/
void Init_Temperature_Sensor_23()
{
	// Configure pins GPIO0 of Sensor Interface Header 1 is input with a pull-up resistor
	SENINTF_HDR1_GPIO0(DIR) = 1;
	
	SENINTF_HDR1_GPIO0(C0) = 0;
	SENINTF_HDR1_GPIO0(C1) = 1;
	
	SENINTF_HDR1_GPIO0(MD0) = 0;
	SENINTF_HDR1_GPIO0(MD1) = 0;
}

/*******************************************************************************
	Routine Name:	main_clrWDT
	Form:			void main_clrWDT( void )
	Parameters:		void
	Return value:	void
	Description:	clear WDT.
******************************************************************************/

void main_clrWDT( void )
{
	//How to clear the Watch Dog Timer:
	// => Write alternately 0x5A and 0xA5 into WDTCON register
	do {
		WDTCON = 0x5Au;
	} while (WDP != 1);
	WDTCON = 0xA5u;
}

/*******************************************************************************
	Routine Name:	_funcUartFin
	Form:			static void _funcUartFin( unsigned int size, unsigned char errStat )
	Parameters:		unsigned int size		 : 
				unsigned char errStat	 : 
	Return value:	void
	Description:	UART transmission completion callback function.
******************************************************************************/
static void _funcUartFin( unsigned int size, unsigned char errStat )
{
	uart_continue();					// Function in UART.c: process to continue send and receive...
	_flgUartFin = (unsigned char)FLG_SET;
	main_reqNotHalt();				// uncommented 5/2/2013
}

/*******************************************************************************
	Routine Name:	_funcI2CFin
	Form:			static void _funcUartFin( unsigned int size, unsigned char errStat )
	Parameters:		unsigned int size		 : 
				unsigned char errStat	 : 
	Return value:	void
	Description:	UART transmission completion callback function.
******************************************************************************/
static void _funcI2CFin( unsigned int size, unsigned char errStat )
{
	i2c_continue();					// Function in UART.c: process to continue send and receive...
	_flgI2CFin = (unsigned char)FLG_SET;
	main_reqNotHalt();				// uncommented 5/2/2013
}

/*******************************************************************************
	Routine Name:	_intI2c
	Form:			static void _intI2c( void )
	Parameters:		void
	Return value:	void
	Description:	I2C handler.
******************************************************************************/
static void _intI2c( void )
{
	i2c_continue();
	main_reqNotHalt();
}

/*******************************************************************************
	Routine Name:	_intADC
	Form:			static void _intADC( void )
	Parameters:		void
	Return value:	void
	Description:	I2C handler.
******************************************************************************/
static void _intADC( void )
{
	_flgADCFin = 1;
}

/*******************************************************************************
	Routine Name:	main_reqNotHalt
	Form:			void reqNotHalt( void )
	Parameters:		void
	Return value:	void
	Description:	request not halt.
******************************************************************************/
void main_reqNotHalt( void )
{
	_reqNotHalt = (unsigned char)FLG_SET;
}

/*******************************************************************************
	Routine Name:	_intUart
	Form:			static void _intUart( void )
	Parameters:		void
	Return value:	void
	Description:	UART handler.
******************************************************************************/
static void _intUart( void )
{
	uart_continue(); 	//in UART.c: process to continue send and receive...
}

//===========================================================================
//	OSC set
//===========================================================================
static void SetOSC(void){

	//FCON0: 			// xMHz PLL (3=1MHz; 2=2MHz; 1=4MHz; 0=8MHz)...
	SYSC0 = 0;			// Used to select the frequency of the HSCLK => 00=8.192MHz.
	SYSC1 = 0;

	OSCM1 = 1;			// 10 => Built-in PLL oscillation mode
	OSCM0 = 0;
   	
	ENOSC = 1;			//1=Enable High Speed Oscillator...
	SYSCLK = 1;			//1=HSCLK; 0=LSCLK 

	LPLL = 1;			//1=Enables the use of PLL oscillation - ADDED 4/30/2013

	__EI();			//INT enable
}
//===========================================================================

//===========================================================================
//	Clear All 3 Bits of Port A
//===========================================================================
void PortA_Low(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Set Pin Data...

	//Direction...	
	PA0DIR = 0;		// PortA Bit0 set to Output Mode...
	PA1DIR = 0;		// PortA Bit1 set to Output Mode...
	PA2DIR = 0;		// PortA Bit2 set to Output Mode...

	//I/O Type...
	PA0C1  = 1;		// PortA Bit0 set to CMOS Output...
	PA0C0  = 1;		
	PA1C1  = 1;		// PortA Bit1 set to CMOS Output...
	PA1C0  = 1;	
	PA2C1  = 1;		// PortA Bit2 set to CMOS Output...
	PA2C0  = 1;	

	//Purpose...
	PA0MD1  = 0;	// PortA Bit0 set to General Purpose Output...
	PA0MD0  = 0;	
	PA1MD1  = 0;	// PortA Bit1 set to General Purpose Output...
	PA1MD0  = 0;	
	PA2MD1  = 0;	// PortA Bit2 set to General Purpose Output...
	PA2MD0  = 0;	

	//Data...
	PA0D = 0;		// A.0 Output OFF....
	PA1D = 0;		// A.1 Output OFF....
	PA2D = 0;		// A.2 Output OFF....

	main_clrWDT(); 	// Clear WDT
}
//===========================================================================

//===========================================================================
//	Clear All 8 Bits of Port B
//===========================================================================
void PortB_Low(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Set Pin Data...

	//Direction...	
	PB0DIR = 0;		// PortB Bit0 set to Output Mode...
	PB1DIR = 0;		// PortB Bit1 set to Output Mode...
	PB2DIR = 0;		// PortB Bit2 set to Output Mode...
	PB3DIR = 0;		// PortB Bit3 set to Output Mode...
	PB4DIR = 0;		// PortB Bit4 set to Output Mode...
	PB5DIR = 0;		// PortB Bit5 set to Output Mode...
	PB6DIR = 0;		// PortB Bit6 set to Output Mode...
	PB7DIR = 0;		// PortB Bit7 set to Output Mode...

	//I/O Type...
	PB0C1  = 1;		// PortB Bit0 set to CMOS Output...
	PB0C0  = 1;		
	PB1C1  = 1;		// PortB Bit1 set to CMOS Output...
	PB1C0  = 1;	
	PB2C1  = 1;		// PortB Bit2 set to CMOS Output...
	PB2C0  = 1;	
	PB3C1  = 1;		// PortB Bit3 set to CMOS Output...
	PB3C0  = 1;		
	PB4C1  = 1;		// PortB Bit4 set to CMOS Output...
	PB4C0  = 1;	
	PB5C1  = 1;		// PortB Bit5 set to CMOS Output...
	PB5C0  = 1;	
	PB6C1  = 1;		// PortB Bit6 set to CMOS Output...
	PB6C0  = 1;	
	PB7C1  = 1;		// PortB Bit7 set to CMOS Output...
	PB7C0  = 1;	

	//Purpose...
	PB0MD1  = 0;	// PortB Bit0 set to General Purpose Output...
	PB0MD0  = 0;	
	PB1MD1  = 0;	// PortB Bit1 set to General Purpose Output...
	PB1MD0  = 0;	
	PB2MD1  = 0;	// PortB Bit2 set to General Purpose Output...
	PB2MD0  = 0;	
	PB3MD1  = 0;	// PortB Bit3 set to General Purpose Output...
	PB3MD0  = 0;	
	PB4MD1  = 0;	// PortB Bit4 set to General Purpose Output...
	PB4MD0  = 0;	
	PB5MD1  = 0;	// PortB Bit5 set to General Purpose Output...
	PB5MD0  = 0;
	PB6MD1  = 0;	// PortB Bit6 set to General Purpose Output...
	PB6MD0  = 0;	
	PB7MD1  = 0;	// PortB Bit7 set to General Purpose Output...
	PB7MD0  = 0;

	//Data...
	PB0D = 0;		// B.0 Output OFF....
	PB1D = 0;		// B.1 Output OFF....
	PB2D = 0;		// B.2 Output OFF....
	PB3D = 0;		// B.3 Output OFF....
	PB4D = 0;		// B.4 Output OFF....
	PB5D = 0;		// B.5 Output OFF....
	PB6D = 0;		// B.6 Output OFF....
	PB7D = 0;		// B.7 Output OFF....

	main_clrWDT(); 	// Clear WDT
}
//===========================================================================

//===========================================================================
//	Clear All 8 Bits of Port C
//===========================================================================
void PortC_Low(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Set Pin Data...

	//Direction...	
	PC0DIR = 0;		// PortC Bit0 set to Output Mode...
	PC1DIR = 0;		// PortC Bit1 set to Output Mode...
	PC2DIR = 0;		// PortC Bit2 set to Output Mode...
	PC3DIR = 0;		// PortC Bit3 set to Output Mode...
	PC4DIR = 0;		// PortC Bit4 set to Output Mode...
	PC5DIR = 0;		// PortC Bit5 set to Output Mode...
	PC6DIR = 0;		// PortC Bit6 set to Output Mode...
	PC7DIR = 0;		// PortC Bit7 set to Output Mode...

	//I/O Type...
	PC0C1  = 1;		// PortC Bit0 set to High-Impedance Output...
	PC0C0  = 1;		
	PC1C1  = 1;		// PortC Bit1 set to High-Impedance Output...
	PC1C0  = 1;	
	PC2C1  = 1;		// PortC Bit2 set to High-Impedance Output...
	PC2C0  = 1;	
	PC3C1  = 1;		// PortC Bit3 set to High-Impedance Output...
	PC3C0  = 1;		
	PC4C1  = 1;		// PortC Bit4 set to High-Impedance Output...
	PC4C0  = 1;	
	PC5C1  = 1;		// PortC Bit5 set to High-Impedance Output...
	PC5C0  = 1;	
	PC6C1  = 1;		// PortC Bit6 set to High-Impedance Output...
	PC6C0  = 1;	
	PC7C1  = 1;		// PortC Bit7 set to High-Impedance Output...
	PC7C0  = 1;	

	//Purpose...
	PC0MD1  = 0;	// PortC Bit0 set to General Purpose Output...
	PC0MD0  = 0;	
	PC1MD1  = 0;	// PortC Bit1 set to General Purpose Output...
	PC1MD0  = 0;	
	PC2MD1  = 0;	// PortC Bit2 set to General Purpose Output...
	PC2MD0  = 0;	
	PC3MD1  = 0;	// PortC Bit3 set to General Purpose Output...
	PC3MD0  = 0;	
	PC4MD1  = 0;	// PortC Bit4 set to General Purpose Output...
	PC4MD0  = 0;	
	PC5MD1  = 0;	// PortC Bit5 set to General Purpose Output...
	PC5MD0  = 0;
	PC6MD1  = 0;	// PortC Bit6 set to General Purpose Output...
	PC6MD0  = 0;	
	PC7MD1  = 0;	// PortC Bit7 set to General Purpose Output...
	PC7MD0  = 0;

	//Data...
	PC0D = 0;		// C.0 Output OFF....
	PC1D = 0;		// C.1 Output OFF....
	PC2D = 0;		// C.2 Output OFF....
	PC3D = 0;		// C.3 Output OFF....
	PC4D = 0;		// C.4 Output OFF....
	PC5D = 0;		// C.5 Output OFF....
	PC6D = 0;		// C.6 Output OFF....
	PC7D = 0;		// C.7 Output OFF....

	main_clrWDT(); 	// Clear WDT

}
//===========================================================================

//===========================================================================
//	Clear All 6 Bits of Port D
//===========================================================================
void PortD_Low(void){

	//Carl's Notes...

	//Step 1: Set Pin Direction...
	//Step 2: Set Pin I/O Type...
	//Step 3: Set Pin Data...

	//Direction...	
	PD0DIR = 1;		// PortD Bit0 set to Input Mode...
	PD1DIR = 1;		// PortD Bit1 set to Input Mode...
	PD2DIR = 1;		// PortD Bit2 set to Input Mode...
	PD3DIR = 1;		// PortD Bit3 set to Input Mode...
	PD4DIR = 1;		// PortD Bit4 set to Input Mode...
	PD5DIR = 1;		// PortD Bit5 set to Input Mode...

	//I/O Type...
	PD0C1= 1;		// PortD Bit0 set to High-impedance input...
	PD0C0= 1;		
	PD1C1= 1;		// PortD Bit1 set to High-impedance input...
	PD1C0= 1;	
	PD2C1= 1;		// PortD Bit2 set to High-impedance input...
	PD2C0= 1;	
	PD3C1= 1;		// PortD Bit3 set to High-impedance input...
	PD3C0= 1;		
	PD4C1= 1;		// PortD Bit4 set to High-impedance input...
	PD4C0= 1;	
	PD5C1= 1;		// PortD Bit5 set to High-impedance input...
	PD5C0= 1;	

	//Data...
	PD0D = 0;		// D.0 Input OFF....
	PD1D = 0;		// D.1 Input OFF....
	PD2D = 0;		// D.2 Input OFF....
	PD3D = 0;		// D.3 Input OFF....
	PD4D = 0;		// D.4 Input OFF....
	PD5D = 0;		// D.5 Input OFF....

	main_clrWDT(); 	// Clear WDT
}
//===========================================================================

/*******************************************************************************
	Routine Name:	NOPms
	Form:			void NOP1000( unsigned int ms )
	Parameters:		unsigned int sec = "Number of seconds where the device is not doing anything"
	Return value:	void
	Description:	NOP for x seconds. Uses HTB* clock (512kHz) and timer 8+9 (max 0xFFFF)
					*(HTBCLK = 1/16 * HSCLK = (1/16)*8192kHz = 512kHz, see HTBDR to change if we need an even smaller increment timer...)
					1/(512kHz) * 0xFFFF = 127ms
					
******************************************************************************/
void NOPms( unsigned int ms )
{
	unsigned int timerThres;
	unsigned char TimeFlag;
	unsigned int TempSec;
	unsigned int timer;
	unsigned int timertest;

	TempSec = ms;
	TimeFlag = 0;

	tm_init(TM_CH_NO_AB);
	tm_setABSource(TM_CS_HTBCLK);
	tm_setABData(0xffff);

	if(ms < 128){
		timerThres = 0x1FF * ms;
		TimeFlag = 0;
	}
	if(ms == 128){
		timerThres = 0xFFFF;
		TimeFlag = 0;
	}
	if(ms > 128){
		while(TempSec > 128){
			TempSec -= 128;
			TimeFlag++;
		}
		if(TempSec != 0){
			timerThres = 0x1FF * TempSec;
		}
		else{
			timerThres = 0xFFFF;
			TimeFlag--;
		}
	}

TimerRestart:
	main_clrWDT();	
	//tm_restart89();	//using LSCLK, the maximum delay time we have is ~2 secs
	tm_startAB();
	timer = tm_getABCounter();
	while(timer < timerThres){
		timer = tm_getABCounter();
		timertest = timer;
	}
	if(TimeFlag !=0){
		tm_stopAB();
		TimeFlag--;
		timerThres = 0xFFFF;
		goto TimerRestart;
	}
}
	
/*******************************************************************************
	Routine Name:	ReverseBits
	Form:			unsigned char ReverseBits(unsigned char data)
	Parameters:		unsigned char data
	Return value:	unsigned char
	Description:	Reverse bits order of data
******************************************************************************/	
unsigned char ReverseBits(unsigned char data)
{
__asm("\n\
	MOV r1,r0\n\
	MOV r0,#0\n\
	MOV r2,#8\n\
_ReverseBits_loop:\n\
	SLL r0,#1\n\
	SRL r1,#1\n\
	BGE _ReverseBits_next\n\
	OR r0,#1\n\
_ReverseBits_next:\n\
	ADD	r2,	#0ffh\n\
	CMP	r2,	#00h\n\
	BGT _ReverseBits_loop\n\
");
}

/*******************************************************************************
	Routine Name:	FlashLEDs
	Form:			unsigned char FlashLEDs(unsigned char data)
	Parameters:		unsigned char data
	Return value:	unsigned char
	Description:	Flash LEDs instead of always ON
******************************************************************************/	
unsigned char FlashLEDs(void)
{
	if(LEDFlashFlag == 0){
		if(LEDChangeFlag == 0){
			LEDOUT(0xAA);
			LEDChangeFlag = 1;
		}
		else{
			LEDOUT(0x55);
			LEDChangeFlag = 0;
		}
	}
	else{
		LEDOUT(0x00);
	}
	LEDFlashFlag++;
	if(LEDFlashFlag >= 4){
		LEDFlashFlag = 0;
	}
}