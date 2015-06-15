/*
 * @brief Blinky example using SysTick and interrupt
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

//Kris's Notes: this is the application that you are using to merge in all elements required for
//  			reading the sensors on the sensor platform board

//Kris's Notes: Update on 6/8/2015 - This will add all sensor on shield to NXP Platform

#include "board.h"
#include "romapi_adc.h"
#include <stdlib.h>

/** @defgroup PERIPH_BLINKY_5410X Simple blinky example
 * @ingroup EXAMPLES_PERIPH_5410X
 * @include "periph\blinky\readme.txt"
 */

/**
 * @}
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* 400KHz I2C bit-rate */
#define I2C_BITRATE         (400000)

/** I2C interface setup */
#define I2C_ADDR_7BIT_DALS		(0x23)
#define I2C_ADDR_7BIT_ALSPROX	(0x38)
#define I2C_ADDR_7BIT_KMX62     (0x0E)
#define I2C_ADDR_7BIT_PRESSURE  (0x5D)
#define I2C_ADDR_7BIT_COLOR     (0x39)
#define I2C_ADDR_7BIT_KX122     (0x1E)
#define LPC_I2C_PORT         	LPC_I2C0
#define LPC_I2CM_CLOCK          SYSCON_CLOCK_I2C0
#define LPC_I2CM_RESET          RESET_I2C0

/* ROM driver handle for I2C master */
static ROM_I2CM_HANDLE_T i2cmHandle;

/* I2C driver context area */
static uint32_t drvData[16];

// ADC Setup
/* Get 32 samples for 3 channels CH0, CH3 and CH4 */
#define NUM_SAMPLES_A    1
#define NUM_CHANNELS_A    3
#define SAMPLE_COUNT_A    1	/* Run 16 times */

/* SEQ_A enables channels 0, 3 and 4; Uses software trigger; doesn't use BURST */
#define ADC_SEQ_A_CONFIG  \
	TRIG_SOFT |		/* Software trigger for SEQ_A */ \
	TRIG_POL_POS |	/* UM recommends this for S/W trigger */ \
	MODE_EOS |		/* Event generated after Sequence done */ \
	ENABLE_CH(3) | ENABLE_CH(4) | ENABLE_CH(11) /* Associate channels 0, 2 and 3 to SEQ_A */ //Note: Channels equate to "pins"
	//ENABLE_CH(0) | ENABLE_CH(3) | ENABLE_CH(4)	/* Associate channels 0, 2 and 3 to SEQ_A */

/* General ADC configuration */
#define ADC_CONFIG \
	MODE_SYNC |	/* Enable synchronous mode */ \
	RESOL_12BIT |	/* Use 12-Bit resolution */	\
	SAMPLE_TIME(0)	/* No extra clocks */

/* Buffer pointers */
static uint16_t buff_A[NUM_SAMPLES_A][NUM_CHANNELS_A];	/* Memory to hold samples for SEQ_A */
//static uint16_t buff_B[NUM_SAMPLES_B][NUM_CHANNELS_B];	/* Memory to hold samples for SEQ_B */
static uint16_t(*volatile pBufA)[NUM_CHANNELS_A];	/* Pointer to current active SEQ_A buffer */
//static uint16_t(*volatile pBufB)[NUM_CHANNELS_B];	/* Pointer to current active SEQ_B buffer */

/* ADC Driver context memory */
#define RAMBLOCK_H          60
static uint32_t  start_of_ram_block0[RAMBLOCK_H];

/* ADC ROM Driver Handle */
static ADC_HANDLE_T *hADC;

//Blinky Variables
#define TICKRATE_HZ (10)	/* 10 ticks per second */
int LED_Counter = 0;
int nextColor = 0;
volatile uint32_t msTicks;

//ADC Setup Variables
int Analog_ALS = 0;
int Analog_Temp = 0;
int Analog_UV = 0;
float Analog_ALS_Ret = 0;
float Analog_Temp_Ret = 0;
float Analog_UV_Ret = 0;

//Digital ALS Variables
int ALS_Return;
float ALS_Return_Conv;

//Digital Accel+Mag Sensor Variables
int16_t Accel_raw_X = 0;
int16_t Accel_raw_Y = 0;
int16_t Accel_raw_Z = 0;
float Accel_X = 0;
float Accel_Y = 0;
float Accel_Z = 0;
int16_t Mag_raw_X = 0;
int16_t Mag_raw_Y = 0;
int16_t Mag_raw_Z = 0;
float Mag_X = 0;
float Mag_Y = 0;
float Mag_Z = 0;

//ALS+PROX Sensor Variables
int16_t RPR0521_PS = 0;
int16_t RPR0521_ALS0 = 0;
int16_t RPR0521_ALS1 = 0;
float RPR0521_ALS_DataRatio = 0;
float RPR0521_ALS_OUT = 0;

//PRessure Sensor Variables
int16_t Pressure_TempRaw = 0;
float Pressure_TempOut = 0;

int16_t Pressure_PresRaw_Inte = 0;
int16_t Pressure_PresRaw_Deci = 0;
float Pressure_PresRaw_Deci_F = 0;
float Pressure_PresOut = 0;

//COLOR Sensor Variables
int16_t BH1745_REDOUT = 0;
int16_t BH1745_GRNOUT = 0;
int16_t BH1745_BLUOUT = 0;

//KX122 Sensor Variables
int16_t KX122_X_RAW = 0;
int16_t KX122_Y_RAW = 0;
int16_t KX122_Z_RAW = 0;
float KX122_X_OUT = 0;
float KX122_Y_OUT = 0;
float KX122_Z_OUT = 0;


//Hall Sensor Setup Variables
int Hall0 = 0;
int Hall1 = 0;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
uint8_t rx[10], tx[4];
uint32_t actualRate;
ROM_I2CM_XFER_T mXfer;
uint32_t memSize, *devMem;
ROM_I2CM_INIT_T i2cmInit;

/*****************************************************************************
 * Private functions
 ****************************************************************************/
static void Delay (uint32_t dlyTicks);
static int adcrom_init(void);
static int adcrom_config(void);
static void adcrom_regcb(void);
static void adcrom_startstop(ADC_HANDLE_T hADC, ADC_CBINDEX_T idx, void *arg);
static void ADC_init(void);
static void I2C_init(void);
//static void I2C_init_DALS(void);
static void I2C_init_ALSPROX(void);
static void I2C_init_KMX62(void);
static void I2C_init_PRESSURE(void);
static void I2C_init_COLOR(void);
static void I2C_init_KX122(void);
static void Init_I2C_PinMux(void);
static void errorOut(char *errStr);
static void BlinkyLoop(void);
static void BlinkyInit(void);
static void HallInit(void);

void SysTick_Handler(void);


/**
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
int main(void)
{
	SystemCoreClockUpdate();
	Board_Init();
	printf("Application Init Started!\n\r");

	//----- Blinky Init -----
	BlinkyInit();
	printf("Blinky Init Done...!\n\r");

	//----- ADC Init -----
	ADC_init();
	printf("ADC Init Done...!\n\r");

	//----- I2C Init -----
	I2C_init();
	//I2C_init_DALS();	//Init Digital ALS
	I2C_init_KMX62();	//Init Digital Accel+Mag Sensor
	I2C_init_ALSPROX();	//Init RPR-0521
	I2C_init_PRESSURE();
	I2C_init_COLOR();
	I2C_init_KX122();

	printf("I2C Init Done...!\n\r");

	//----- Initialize Digital Input for Hall Sensor -----
	HallInit();
	printf("Dig Input Init Done...!\n\r");

	printf("Application Init Done... Starting While Loop... \n\r\n\r");
	while (1) {
		//__WFI();		//Wait for interrupt... not sure exactly what this does... maybe it is required when while(1) is blank, but interrupts still fire if I remove this...

		//Initial Blinky Application
		BlinkyLoop();
		printf("ROHM Multi-Sensor Shield Outputs - Using NXP LPC54102\n\r");

		// ----- Start ADC Reads for Relevant Devices (pins A0, A1, A2) -----
		if (ROM_ADC_StartConversion(hADC, ADC_SEQ_A, &buff_A[0][0], NUM_SAMPLES_A * NUM_CHANNELS_A) != LPC_OK) {
			DEBUGSTR("ERROR: Starting conversion in SEQ_A\r\n");
		}
		/* Wait for buffer to be filled up with required number of samples */
		if (ROM_ADC_Handler(hADC, ADC_EV_SEQ_A_POLL) == LPC_OK) {
			pBufA = buff_A;

			//Convert Raw Data Values into LUX Values
			Analog_ALS = pBufA[0][0];
			Analog_ALS_Ret = (float)Analog_ALS * (0.25246) ;	//Result in LX

			Analog_Temp = pBufA[0][1];
			Analog_Temp_Ret = (float)Analog_Temp * (3.3/4095);	//3.3V/4095
			Analog_Temp_Ret = (Analog_Temp_Ret - 1.753)/(-0.01068) + 30;

			Analog_UV = pBufA[0][2];
			Analog_UV_Ret = (float)Analog_UV *(3.3/4095);
			Analog_UV_Ret = (Analog_UV_Ret - 2.2)/(0.129)+10;

			//printf("%u, %u, %u \n\r", Analog_ALS, Analog_Temp, Analog_UV);
			//printf("Analog ALS: %0.3f lx \n\r", Analog_ALS_Ret);
			printf("BDE0600G: %0.3f DegC \n\r", Analog_Temp_Ret);
			printf("ML8511: %0.3f mW/cm2 \n\r", Analog_UV_Ret);
		}

		//----- Check Status of the Hall Sensor Outputs -----
		Hall0 = Chip_GPIO_ReadPortBit(LPC_GPIO, 1, 6);
		Hall1 = Chip_GPIO_ReadPortBit(LPC_GPIO, 1, 14);
			//This is an omnipolar, single output Hall Sensor... Any Mag field will trigger, but only one Pin shows this trigger...
			//Thus, only Hall1 is used to detect Mag Field...
		if(Hall0 == 0)
		{
			printf("BU52014, Hall = South Mag Field Detected!\n\r");		//1,6
		}
		else if(Hall1 == 0)
		{
			printf("BU52014, Hall = North Mag Field Detected!\n\r");		//1,6
		}
		else
		{
			printf("BU52014, Hall = Mag Field Not Detected!\n\r");
		}
		//printf("Hall0 = %u\n\r", Hall0);		//1,6
		//printf("Hall1 = %u\n\r", Hall1);		//1,6

		// ----- Start I2C Read from Digital ALS Device -----
		/*
		mXfer.slaveAddr = I2C_ADDR_7BIT_DALS;
		tx[0] = 0;
		mXfer.txBuff = tx;
		mXfer.rxBuff = rx;
		mXfer.txSz = 0;
		mXfer.rxSz = 2;
		mXfer.flags = ROM_I2CM_FLAG_BLOCKING;
		// Start transfer and wait for completion
		ROM_I2CM_Transfer(i2cmHandle, &mXfer);
		// Check status of the transfer
		if (mXfer.status != LPC_OK) {
			errorOut("Error during I2CM transfer\r\n");
		}
		// Format the I2C output
		ALS_Return = rx[0]<<8 | rx[1];
		ALS_Return_Conv = (float)ALS_Return / 1.2;
		DEBUGOUT("Digital ALS = %0.2f lx\r\n", ALS_Return_Conv);
		*/


		// ----- Start I2C Read from Accel Portion of KMX61 Device -----
		//Write address, Max - write 1 btye, read 8 bytes back
		mXfer.slaveAddr = I2C_ADDR_7BIT_KMX62;
		tx[0] = 0x0A;
		mXfer.txBuff = tx;
		mXfer.rxBuff = rx;
		mXfer.txSz = 1;
		mXfer.rxSz = 6;
		mXfer.flags = ROM_I2CM_FLAG_BLOCKING;
		/* Start transfer and wait for completion */
		ROM_I2CM_Transfer(i2cmHandle, &mXfer);
		/* Check status of the transfer */
		if (mXfer.status != LPC_OK) {
			errorOut("Error during I2CM transfer\r\n");
		}
		// Format the I2C output
		Accel_raw_X = rx[1]<<8 | rx[0];
		Accel_raw_Y = rx[3]<<8 | rx[2];
		Accel_raw_Z = rx[5]<<8 | rx[4];
		// Convert I2C output
		Accel_X = (float)Accel_raw_X/4096/2;
		Accel_Y = (float)Accel_raw_Y/4096/2;
		Accel_Z = (float)Accel_raw_Z/4096/2;
		//Print to Screen
		DEBUGOUT("KMX62, Accel (X,Y,Z) = %0.2f, %0.2f, %0.2f g\r\n", Accel_X, Accel_Y, Accel_Z);

		// ----- Start I2C Read from Mag Portion of KMX61 Device -----
		/* Write address, Max - write 1 btye, read 8 bytes back */
		mXfer.slaveAddr = I2C_ADDR_7BIT_KMX62;
		tx[0] = 0x10;
		mXfer.txBuff = tx;
		mXfer.rxBuff = rx;
		mXfer.txSz = 1;
		mXfer.rxSz = 6;
		mXfer.flags = ROM_I2CM_FLAG_BLOCKING;
		/* Start transfer and wait for completion */
		ROM_I2CM_Transfer(i2cmHandle, &mXfer);
		/* Check status of the transfer */
		if (mXfer.status != LPC_OK) {
			errorOut("Error during I2CM transfer\r\n");
		}
		// Format the I2C output
		Mag_raw_X = rx[1]<<8 | rx[0];
		Mag_raw_Y = rx[3]<<8 | rx[2];
		Mag_raw_Z = rx[5]<<8 | rx[4];
		// Convert I2C output
		Mag_X = (float)Mag_raw_X/0.584;
		Mag_Y = (float)Mag_raw_Y/0.584;
		Mag_Z = (float)Mag_raw_Z/0.584;
		//Print to Screen
		DEBUGOUT("KMX62, Mag (X,Y,Z) = %0.2f, %0.2f, %0.2f uT\r\n", Mag_X, Mag_Y, Mag_Z);


		// ----- Start I2C Read from RPR-0521  -----
		//Write address, Max - write 1 btye, read 8 bytes back
		mXfer.slaveAddr = I2C_ADDR_7BIT_ALSPROX;
		tx[0] = 0x44;
		mXfer.txBuff = tx;
		mXfer.rxBuff = rx;
		mXfer.txSz = 1;
		mXfer.rxSz = 6;
		mXfer.flags = ROM_I2CM_FLAG_BLOCKING;
		/* Start transfer and wait for completion */
		ROM_I2CM_Transfer(i2cmHandle, &mXfer);
		/* Check status of the transfer */
		if (mXfer.status != LPC_OK) {
			errorOut("Error during I2CM transfer\r\n");
		}
		// Format the I2C output
		RPR0521_PS = rx[1]<<8 | rx[0];
		RPR0521_ALS0 = rx[3]<<8 | rx[2];
		RPR0521_ALS1 = rx[5]<<8 | rx[4];
		// Convert I2C output
		RPR0521_ALS_DataRatio = (float)RPR0521_ALS1/(float)RPR0521_ALS0;

		if(RPR0521_ALS_DataRatio < 0.595){
			RPR0521_ALS_OUT = (1.682*(float)RPR0521_ALS0 - 1.877*(float)RPR0521_ALS1);
		}
		else if(RPR0521_ALS_DataRatio < 1.015){
			RPR0521_ALS_OUT = (0.644*(float)RPR0521_ALS0 - 0.132*(float)RPR0521_ALS1);
		}
		else if(RPR0521_ALS_DataRatio < 1.352){
			RPR0521_ALS_OUT = (0.756*(float)RPR0521_ALS0 - 0.243*(float)RPR0521_ALS1);
		}
		else if(RPR0521_ALS_DataRatio < 3.053){
			RPR0521_ALS_OUT = (0.766*(float)RPR0521_ALS0 - 0.25*(float)RPR0521_ALS1);
		}
		else{
			RPR0521_ALS_OUT = 0;
		}

		//Print to Screen
		DEBUGOUT("RPR-0521 (PROX) = %u ADC Counts\r\n", RPR0521_PS);
		DEBUGOUT("RPR-0521  (ALS) = %0.2f Lux\r\n", RPR0521_ALS_OUT);



		// ----- Start I2C Read from PRESSURE Sensor  -----
		//Write address, Max - write 1 btye, read 8 bytes back
		mXfer.slaveAddr = I2C_ADDR_7BIT_PRESSURE;
		tx[0] = 0x1A;
		mXfer.txBuff = tx;
		mXfer.rxBuff = rx;
		mXfer.txSz = 1;
		mXfer.rxSz = 5;
		mXfer.flags = ROM_I2CM_FLAG_BLOCKING;
		/* Start transfer and wait for completion */
		ROM_I2CM_Transfer(i2cmHandle, &mXfer);
		/* Check status of the transfer */
		if (mXfer.status != LPC_OK) {
			errorOut("Error during I2CM transfer\r\n");
		}

		Pressure_TempRaw = rx[0]<<8 | rx[1];
		Pressure_TempOut = (float)Pressure_TempRaw/32;

		Pressure_PresRaw_Inte = (rx[2]<<3) | (rx[3]>>5);
		Pressure_PresRaw_Deci = ((rx[3] && 0x1F)<<6) | (rx[4]>>2);
		Pressure_PresRaw_Deci_F = (float)Pressure_PresRaw_Deci*0.00048828125;
		Pressure_PresOut = (float)Pressure_PresRaw_Inte+(float)Pressure_PresRaw_Deci_F;

		DEBUGOUT("BM1382 (TEMP) = %0.2f DegC\r\n", Pressure_TempOut);
		DEBUGOUT("BM1382 (PRES) = %0.2f HPa\r\n", Pressure_PresOut);


		// ----- Start I2C Read from COLOR Sensor  -----
		//Write address, Max - write 1 btye, read 8 bytes back
		mXfer.slaveAddr = I2C_ADDR_7BIT_COLOR;
		tx[0] = 0x50;
		mXfer.txBuff = tx;
		mXfer.rxBuff = rx;
		mXfer.txSz = 1;
		mXfer.rxSz = 6;
		mXfer.flags = ROM_I2CM_FLAG_BLOCKING;
		/* Start transfer and wait for completion */
		ROM_I2CM_Transfer(i2cmHandle, &mXfer);
		/* Check status of the transfer */
		if (mXfer.status != LPC_OK) {
			errorOut("Error during I2CM transfer\r\n");
		}

		BH1745_REDOUT = rx[1]<<8 | rx[0];
		BH1745_GRNOUT = rx[3]<<8 | rx[2];
		BH1745_BLUOUT = rx[5]<<8 | rx[4];

		DEBUGOUT("BH1745 (R,G,B) = %u, %u, %u ADC Counts\r\n", BH1745_REDOUT, BH1745_GRNOUT, BH1745_BLUOUT);


		// ----- Start I2C Read from KX122 Sensor  -----
		//Write address, Max - write 1 btye, read 8 bytes back
		mXfer.slaveAddr = I2C_ADDR_7BIT_KX122;
		tx[0] = 0x06;
		mXfer.txBuff = tx;
		mXfer.rxBuff = rx;
		mXfer.txSz = 1;
		mXfer.rxSz = 6;
		mXfer.flags = ROM_I2CM_FLAG_BLOCKING;
		/* Start transfer and wait for completion */
		ROM_I2CM_Transfer(i2cmHandle, &mXfer);
		/* Check status of the transfer */
		if (mXfer.status != LPC_OK) {
			errorOut("Error during I2CM transfer\r\n");
		}

		KX122_X_RAW = rx[1]<<8 | rx[0];
		KX122_Y_RAW = rx[3]<<8 | rx[2];
		KX122_Z_RAW = rx[5]<<8 | rx[4];

		KX122_X_OUT = (float)KX122_X_RAW/16384;
		KX122_Y_OUT = (float)KX122_Y_RAW/16384;
		KX122_Z_OUT = (float)KX122_Z_RAW/16384;

		DEBUGOUT("KX122 (X,Y,Z) = %0.2f, %0.2f, %0.2f g\r\n", KX122_X_OUT, KX122_Y_OUT, KX122_Z_OUT);

		printf("\n\r");	//Extra Line Space to show new sample Data
		Delay(2000);	//Wait 2 Seconds before running again
	}
	return 0;	//should never reach this point due to above while loop...
}

static void Delay (uint32_t dlyTicks)  {
	  uint32_t curTicks;

	// Let's re-start the SysTick timer each time we delay, just so we know
	// for sure we don't overflow. Move this out of the delay function for
	// more accurate small delays.
  SysTick_Config(48000000/1000); // we're operating at 48 MHz
  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks); // wait here until our time has come...
}

static void adcrom_regcb(void)
{
	ROM_ADC_RegisterCB(hADC, ADC_STOP_SEQ_A, adcrom_startstop);	/* SEQ_A Stop */
	ROM_ADC_RegisterCB(hADC, ADC_START_SEQ_A, adcrom_startstop);/* SEQ_A Start */
	ROM_ADC_RegisterCB(hADC, ADC_STOP_SEQ_B, adcrom_startstop);	/* SEQ_B Stop */
	ROM_ADC_RegisterCB(hADC, ADC_START_SEQ_B, adcrom_startstop);/* SEQ_B start */
}

/* Call-Back function for ADC Start/Stop event */
static void adcrom_startstop(ADC_HANDLE_T hADC, ADC_CBINDEX_T idx, void *arg)
{
	switch (idx) {
	case ADC_START_SEQ_A:
		//DEBUGSTR("Sequence A Started!\r\n");
		break;

	case ADC_START_SEQ_B:
		//DEBUGSTR("Sequence B Started!\r\n");
		break;

	case ADC_STOP_SEQ_A:
		//DEBUGSTR("Sequence A Stopped!\r\n");
		break;

	case ADC_STOP_SEQ_B:
		//DEBUGSTR("Sequence B Stopped!\r\n");
		break;

	default:
		break;
	}
}

static void ADC_PinMuxSetup(void)
{
#if defined(BOARD_NXP_LPCXPRESSO_54102)
	/* Enable PININT1, which will trigger SEQ_B */
	/*	//This needs to be commented out as it messes with I2C's interurpt pin...
	Chip_PININT_Init(LPC_PININT);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 24, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_DIGITAL_EN | IOCON_GPIO_MODE);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, 0, 24);
	LPC_INMUX->PINTSEL[PININTSELECT1] = 24;
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH(PININTSELECT1));
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH(PININTSELECT1));
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH(PININTSELECT1));
	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);
	NVIC_EnableIRQ(PIN_INT1_IRQn);
	*/

	/* All pins to inactive, neither pull-up nor pull-down. */
	//Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 29, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	//Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 30, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	//Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 31, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 0, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);	//
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 1, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);	//
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 2, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 3, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 4, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 5, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	//Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 6, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 7, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 8, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);	//

#else
#warning "No ADC setup for this example"
#endif
}

/* Initialize the ADC ROM Driver */
static int adcrom_init(void)
{
	volatile int size_in_bytes;

	ADC_PinMuxSetup();
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_PD_ADC0 | SYSCON_PDRUNCFG_PD_VDDA_ENA | SYSCON_PDRUNCFG_PD_VREFP);
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_ADC0);

	Chip_Clock_SetADCClockSource(SYSCON_ADCCLKSELSRC_MAINCLK);
	Chip_Clock_SetADCClockDiv(0x1);

	size_in_bytes =  ROM_ADC_GetMemSize();

	if (RAMBLOCK_H < (size_in_bytes / 4)) {
		return 1;
	}

	hADC = ROM_ADC_Init(start_of_ram_block0, LPC_ADC_BASE, 0);

	return 0;
}

/* Configure ADC ROM Driver and peripheral */
static int adcrom_config(void)
{
	ADC_CFG_T cfg = {
		ADC_SEQ_A_CONFIG,
		ADC_CONFIG,
		0	/* Divider will be calculated during run time */
	};

	cfg.clkDiv = 0xFF;

	/* Configure the ADC */
	ROM_ADC_Configure(hADC, &cfg);

	/* Calibrate the ADC */
	if (ROM_ADC_Calibrate(hADC, Chip_Clock_GetSystemClockRate()) != LPC_OK) {
		DEBUGSTR("ERROR: Calibrating ADC\r\n");
		while (1) {}
	}
	DEBUGSTR("ADC Initialized and Calibrated successfully!\r\n");

	/* Channel configurations */
	ROM_ADC_ConfigureCh(hADC, 0, ADC_CH_THRES_DATA | ADC_CH_THRES_SEL1 | ADC_CH_THRES_CROSSING);
	ROM_ADC_ConfigureCh(hADC, 1, ADC_CH_THRES_DATA);

	return 0;
}

static void ADC_init(void)
{
	/* Initialize the PinMux and setup the memory for ROM driver */
	if (adcrom_init()) {
		return 1;
	}
	/* Configure the ADC */
	if (adcrom_config()) {
		return 1;
	}
	adcrom_regcb();	/* Register call-back functions */
}

static void I2C_init(void)
{
	/* Setup I2C pin muxing, enable I2C clock and reset I2C peripheral */
	Init_I2C_PinMux();
	Chip_Clock_EnablePeriphClock(LPC_I2CM_CLOCK);
	Chip_SYSCON_PeriphReset(LPC_I2CM_RESET);

	/* Get needed size for driver context memory */
	memSize = ROM_I2CM_GetMemSize();
	if (memSize > sizeof(drvData)) {
		errorOut("Can't allocate memory for I2C driver context\r\n");
	}
	devMem = drvData;	/* Or just use malloc(memSize) */

	/* Initialize driver */
	i2cmInit.pUserData = NULL;
	i2cmInit.base = (uint32_t) LPC_I2C_PORT;
	i2cmHandle = ROM_I2CM_Init(devMem, &i2cmInit);
	if (i2cmHandle == NULL) {
		/* Error initializing I2C */
		errorOut("Error initializing ROM\r\n");
	}

	/* Set I2C clock rate */
	actualRate = ROM_I2CM_SetClockRate(i2cmHandle,
									   Chip_Clock_GetAsyncSyscon_ClockRate(), I2C_BITRATE);
	DEBUGOUT("Actual I2C master rate = %dHz\r\n", actualRate);
}

/*
static void I2C_init_DALS(void)
{
	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_DALS;
	tx[0] = 0x01;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 1;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	//Start transfer and wait for completion
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	// Check status of the transfer
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("DALS - Init0 Sent\r\n");

	mXfer.slaveAddr = I2C_ADDR_7BIT_DALS;
	tx[0] = 0x10;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 1;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	// Start transfer and wait for completion
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	//Check status of the transfer
	//DEBUGOUT("I2C transfer Init1 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("DALS - Init1 Sent\r\n");
}
*/

static void I2C_init_ALSPROX(void)
{
	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_ALSPROX;
	tx[0] = 0x41;
	tx[1] = 0xE6;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	/* Check status of the transfer */
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("ALSPROX - Init1 Sent\r\n");

	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_ALSPROX;
	tx[0] = 0x42;
	tx[1] = 0x03;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	/* Check status of the transfer */
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("ALSPROX - Init2 Sent\r\n");

	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_ALSPROX;
	tx[0] = 0x43;
	tx[1] = 0x20;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	/* Check status of the transfer */
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("ALSPROX - Init3 Sent\r\n");
}

static void I2C_init_PRESSURE(void)
{
	//----- Initialize Pressure Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_PRESSURE;
	tx[0] = 0x12;
	tx[1] = 0x01;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	/* Check status of the transfer */
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("Pressure - PWRDOWN Sent\r\n");

	//----- Initialize Pressure Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_PRESSURE;
	tx[0] = 0x13;
	tx[1] = 0x01;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	/* Check status of the transfer */
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("Pressure - SLEEP Sent\r\n");

	//----- Initialize Pressure Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_PRESSURE;
	tx[0] = 0x14;
	tx[1] = 0xC4;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	/* Check status of the transfer */
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("Pressure - Mode Control Sent\r\n");
}
static void I2C_init_COLOR(void)
{
	//----- Initialize Pressure Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_COLOR;
	tx[0] = 0x61;
	tx[1] = 0x03;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	/* Check status of the transfer */
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("Color - INIT1 Sent\r\n");

	//----- Initialize Pressure Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_COLOR;
	tx[0] = 0x41;
	tx[1] = 0x00;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	/* Check status of the transfer */
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("Color - INIT2 Sent\r\n");

	//----- Initialize Pressure Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_COLOR;
	tx[0] = 0x42;
	tx[1] = 0x92;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	/* Check status of the transfer */
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("Color - INIT3 Sent\r\n");

	//----- Initialize Pressure Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_COLOR;
	tx[0] = 0x43;
	tx[1] = 0x02;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	/* Check status of the transfer */
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("Color - INIT4 Sent\r\n");
}
static void I2C_init_KX122(void)
{
	//----- Initialize Pressure Sensor -----
		mXfer.slaveAddr = I2C_ADDR_7BIT_KX122;
		tx[0] = 0x18;
		tx[1] = 0x41;
		mXfer.txBuff = tx;
		mXfer.rxBuff = rx;
		mXfer.txSz = 2;
		mXfer.rxSz = 0;
		mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

		/* Start transfer and wait for completion */
		ROM_I2CM_Transfer(i2cmHandle, &mXfer);

		/* Check status of the transfer */
		//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
		if (mXfer.status != LPC_OK) {
			errorOut("Error during I2CM transfer\r\n");
		}
		printf("KX122 - INIT1 Sent\r\n");

		//----- Initialize Pressure Sensor -----
		mXfer.slaveAddr = I2C_ADDR_7BIT_KX122;
		tx[0] = 0x1B;
		tx[1] = 0x02;
		mXfer.txBuff = tx;
		mXfer.rxBuff = rx;
		mXfer.txSz = 2;
		mXfer.rxSz = 0;
		mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

		/* Start transfer and wait for completion */
		ROM_I2CM_Transfer(i2cmHandle, &mXfer);

		/* Check status of the transfer */
		//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
		if (mXfer.status != LPC_OK) {
			errorOut("Error during I2CM transfer\r\n");
		}
		printf("KX122 - INIT2 Sent\r\n");

		//----- Initialize Pressure Sensor -----
		mXfer.slaveAddr = I2C_ADDR_7BIT_KX122;
		tx[0] = 0x1A;
		tx[1] = 0xD8;
		mXfer.txBuff = tx;
		mXfer.rxBuff = rx;
		mXfer.txSz = 2;
		mXfer.rxSz = 0;
		mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

		/* Start transfer and wait for completion */
		ROM_I2CM_Transfer(i2cmHandle, &mXfer);

		/* Check status of the transfer */
		//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
		if (mXfer.status != LPC_OK) {
			errorOut("Error during I2CM transfer\r\n");
		}
		printf("KX122 - INIT3 Sent\r\n");

		//----- Initialize Pressure Sensor -----
		mXfer.slaveAddr = I2C_ADDR_7BIT_KX122;
		tx[0] = 0x22;
		tx[1] = 0x01;
		mXfer.txBuff = tx;
		mXfer.rxBuff = rx;
		mXfer.txSz = 2;
		mXfer.rxSz = 0;
		mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

		/* Start transfer and wait for completion */
		ROM_I2CM_Transfer(i2cmHandle, &mXfer);

		/* Check status of the transfer */
		//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
		if (mXfer.status != LPC_OK) {
			errorOut("Error during I2CM transfer\r\n");
		}
		printf("KX122 - INIT4 Sent\r\n");

		//----- Initialize Pressure Sensor -----
		mXfer.slaveAddr = I2C_ADDR_7BIT_KX122;
		tx[0] = 0x18;
		tx[1] = 0xC1;
		mXfer.txBuff = tx;
		mXfer.rxBuff = rx;
		mXfer.txSz = 2;
		mXfer.rxSz = 0;
		mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

		/* Start transfer and wait for completion */
		ROM_I2CM_Transfer(i2cmHandle, &mXfer);

		/* Check status of the transfer */
		//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
		if (mXfer.status != LPC_OK) {
			errorOut("Error during I2CM transfer\r\n");
		}
		printf("KX122 - INIT5 Sent\r\n");
}


static void I2C_init_KMX62(void)
{

	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_KMX62;
	tx[0] = 0x3A;
	tx[1] = 0x5F;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	//Start transfer and wait for completion
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	// Check status of the transfer
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("ACCEL - Init1 Sent\r\n");



	/*
	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_KMX62;
	tx[0] = 0x29;
	tx[1] = 0x03;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	//Start transfer and wait for completion
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	// Check status of the transfer
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("ACCEL - Init1 Sent\r\n");

	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_KMX62;
	tx[0] = 0x60;
	tx[1] = 0x00;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	//Start transfer and wait for completion
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	//Check status of the transfer
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("ACCEL - Init2 Sent\r\n");

	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_KMX62;
	tx[0] = 0x2A;
	tx[1] = 0x13;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	//Start transfer and wait for completion
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	//Check status of the transfer
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("ACCEL - Init3 Sent\r\n");

	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_KMX62;
	tx[0] = 0x2B;
	tx[1] = 0x00;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	// Start transfer and wait for completion
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	// Check status of the transfer
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("ACCEL - Init4 Sent\r\n");

	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_KMX62;
	tx[0] = 0x2C;
	tx[1] = 0x00;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	//Start transfer and wait for completion
	//ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	//Check status of the transfer
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("ACCEL - Init5 Sent\r\n");

	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_KMX62;
	tx[0] = 0x4C;
	tx[1] = 0x01;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	//Start transfer and wait for completion
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	// Check status of the transfer
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("ACCEL - Init6 Sent\r\n");

	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_KMX62;
	tx[0] = 0x78;
	tx[1] = 0x00;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	//Start transfer and wait for completion
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	//Check status of the transfer
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("ACCEL - Init7 Sent\r\n");

	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_KMX62;
	tx[0] = 0x79;
	tx[1] = 0x00;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	// Start transfer and wait for completion
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	// Check status of the transfer
	// DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("ACCEL - Init8 Sent\r\n");

	//----- Initialize Digital ALS Sensor -----
	mXfer.slaveAddr = I2C_ADDR_7BIT_KMX62;
	tx[0] = 0x29;
	tx[1] = 0x00;
	mXfer.txBuff = tx;
	mXfer.rxBuff = rx;
	mXfer.txSz = 2;
	mXfer.rxSz = 0;
	mXfer.flags = ROM_I2CM_FLAG_BLOCKING;

	//Start transfer and wait for completion
	ROM_I2CM_Transfer(i2cmHandle, &mXfer);

	//Check status of the transfer
	//DEBUGOUT("I2C transfer Init0 completed: status = %x\r\n", mXfer.status);
	if (mXfer.status != LPC_OK) {
		errorOut("Error during I2CM transfer\r\n");
	}
	printf("ACCEL - Init9 Sent\r\n");
	*/
}

/* Initializes pin muxing for I2C interface - note that SystemInit() may
   already setup your pin muxing at system startup */
static void Init_I2C_PinMux(void)
{
#if defined(BOARD_NXP_LPCXPRESSO_54102)
	/* Connect the I2C_SDA and I2C_SCL signals to port pins */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, (IOCON_FUNC1 | IOCON_DIGITAL_EN));	/* I2C0 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 24, (IOCON_FUNC1 | IOCON_DIGITAL_EN));	/* I2C0 */

#else
	/* Configure your own I2C pin muxing here if needed */
#warning "No I2C pin muxing defined"
#endif
}

/* Display error string and spin */
static void errorOut(char *errStr)
{
	DEBUGOUT(errStr);
	//while (1) {}
}

static void BlinkyInit(void)
{
	Board_LED_Set(0, false);
	Board_LED_Set(1, false);
	Board_LED_Set(2, false);
	//----- Init Timer for Blinky and "sleep function" -----
	/* Enable SysTick Timer */
	SysTick_Config(48000000/1000); 	// we're operating at 48 MHz
									//SysTick_Config(SystemCoreClock / TICKRATE_HZ);
}

static void BlinkyLoop(void)
{
	if(LED_Counter == 0){
		if(nextColor == 0){
			Board_LED_Toggle(0);
			nextColor = 1;
		}
		else{
			Board_LED_Toggle(0);
			LED_Counter++;
			nextColor = 0;
		}
	}
	else if(LED_Counter == 1){
		if(nextColor == 0){
			Board_LED_Toggle(1);
			nextColor = 1;
		}
		else{
			Board_LED_Toggle(1);
			LED_Counter++;
			nextColor = 0;
		}
	}
	else{
		if(nextColor == 0){
			Board_LED_Toggle(2);
			nextColor = 1;
		}
		else{
			Board_LED_Toggle(2);
			LED_Counter++;
			nextColor = 0;
		}
	}
	if(LED_Counter >= 3){
		LED_Counter = 0;
	}
}

static void HallInit(void)
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 6, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_DIGITAL_EN | IOCON_GPIO_MODE);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 14, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_DIGITAL_EN | IOCON_GPIO_MODE);
}

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	msTicks++;
}
