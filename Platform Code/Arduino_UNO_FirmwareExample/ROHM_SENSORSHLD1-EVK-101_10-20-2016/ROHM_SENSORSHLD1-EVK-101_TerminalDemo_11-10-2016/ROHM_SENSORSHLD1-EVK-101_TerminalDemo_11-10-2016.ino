/*------------------------------------------------------------------------------
 ROHM Multi-Sensor Shield Board - Sensor Output Application
 
    Copyright (C) 2016 ROHM USDC Applications Engineering Team

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    ---Description---
     This program reads the value of the connected ROHM Sensors on the Multi-sensor Shield and 
     returns the sensor output through the arduino's serial port terminal (115200 Baud)

     ---Debugging Definitions---
     Using the default code will return all sensor values back for all sensors on the shield.
     You can remove sensor returns by commenting out the appropriate #defines for the sensors you do not want to return data for
     Note: using the #define keywords is also a good way to find where a particular sensor is being manupulated

   ---Demo Mode Definitions---
   There are two key parameters that this sample application can run, CSVOutput and SensorSamplePeriod
  
   CSVOutput can be used to format the UART output to the terminal application.
    If CSVOutput is NOT defined or commented out, the application will return the data to the terminal in a manner that is easy to read.
      This Option is good for live demos of the shield or when looking at live data values of the sensor
    If CSVOutput is defined, the application will return the data to the terminal in CSV format to be saved by a terminal application.
      When logging, we recommend resarting the board, when you start logging in order to get the header information correct
      See the document "ROHM_SENSORSHLD1-EVK-101_LoggingManual_2016-11-10.pdf" for additional information
      
    SensorSamplePeriod can be used to adjust the timing between each sensor read.
      This value adds a delay to the end of the main loop
      This value can be formatter in ms
      This value applies to both CSVOutput modes and will operate regardless of the mode set
   
   ---Rework Required on the Arduino UNO Board ---
   Confirmed on Arduino UNO Board
   Rework Required:
     1. The Arduino UNO's I2C input pins are routed to A4 and A5 on the Arduino UNO board.
         Thus, to make sure this does not conflict on our board, we need to remove jumper pins connected to A4/A5
         AKA. REMOVE R31 and R32
     2. Similarly above, since we removed the connection for A4, we need to connect ADC1 (or UV Sensor output) to the Arduino.
         Thus, this code expects for A4 to be re-routed to A0.
         We do this by removing R27 and by routing the top resistor pad of R31 to the bottom resistor pad of R27 (
 
First Revision Posted to Git on 15 June 2015
 by ROHM USDC Applications Engineering Team
Updated 10 November 2016

------------------------------------------------------------------------------*/

// ----- Debugging Definitions -----
#define AnalogTemp  //BDE0600G
#define AnalogUV    //ML8511
#define HallSen     //BU52014
#define KMX62       //KMX62
  //#define Pressure    //BM1383AGLV  //Use this definition witn non-yellow stickered SHLD1 board
#define PressureOld    //BM1383GLV  //Use this definition with yellow stickered SHLD1 board
#define ALSProx     //RPR-0521
#define Color       //BH1745
#define KX122       //KX122
#define KXG03       //KXG03
#define MagField    //BM1422

// ----- Demo Mode Definitions -----
//#define CSVOutput
#define SensorSamplePeriod  500 //in ms

// ----- Included Files -----
//#include <Wire.h>         //Default I2C Library

#define SCL_PIN 5  //A5       //Note that if you are using the I2C based sensors, you will need to download and
#define SCL_PORT PORTC    //install the "SoftI2CMaster" as "Wire" does not support repeated start...
#define SDA_PIN 4  //A4         //References:
#define SDA_PORT PORTC    //  http://playground.arduino.cc/Main/SoftwareI2CLibrary

#include <SoftI2CMaster.h>  //  https://github.com/felias-fogg/SoftI2CMaster
#define I2C_TIMEOUT 1000  // Sets Clock Stretching up to 1sec
#define I2C_FASTMODE 1    // Sets 400kHz operating speed

// ----- Globals -----

//ADC Globals - Analog ALS, Temp, UV
#ifdef AnalogTemp
int ADCpin_AnalogTemp = A2;
#endif

#ifdef AnalogUV
int ADCpin_AnalogUV = A0;
#endif

int sensorValue = 0;
float sensorConvert = 0;
unsigned int j;
unsigned int lineClear = 0;
double intervalTime = 0;

//Digital Input Globals - Hall Sensor
#ifdef HallSen
int Hall_Out0 = 0;
int Hall_Out1 = 1;
#endif

//I2C globals (using SoftI2CMaster libary) - MEMs Kionix Sensor 
int I2C_check = 0;

#ifdef KMX62
int KMX62_DeviceAddress = 0x1C;  //this is the 8bit address, 7bit address = 0x0E
//Accel Portion
int MEMS_Accel_Xout_highByte = 0;
int MEMS_Accel_Xout_lowByte = 0;
int MEMS_Accel_Yout_highByte = 0;
int MEMS_Accel_Yout_lowByte = 0;
int MEMS_Accel_Zout_highByte = 0;
int MEMS_Accel_Zout_lowByte = 0;
int MEMS_Accel_Xout = 0;
int MEMS_Accel_Yout = 0;
int MEMS_Accel_Zout = 0;
float MEMS_Accel_Conv_Xout = 0;
float MEMS_Accel_Conv_Yout = 0;
float MEMS_Accel_Conv_Zout = 0;
//Mag Sensor Portion
int MEMS_Mag_Xout_highByte = 0;
int MEMS_Mag_Xout_lowByte = 0;
int MEMS_Mag_Yout_highByte = 0;
int MEMS_Mag_Yout_lowByte = 0;
int MEMS_Mag_Zout_highByte = 0;
int MEMS_Mag_Zout_lowByte = 0;
int MEMS_Mag_Xout = 0;
int MEMS_Mag_Yout = 0;
int MEMS_Mag_Zout = 0;
float MEMS_Mag_Conv_Xout = 0;
float MEMS_Mag_Conv_Yout = 0;
float MEMS_Mag_Conv_Zout = 0;
#endif

#ifdef Pressure
int BM1383_DeviceAddress = 0xBA;  //this is the 8bit address, 7bit address = 0x5D
int BM1383_Temp_highByte = 0;
int BM1383_Temp_lowByte = 0;
int BM1383_Pres_highByte = 0;
int BM1383_Pres_lowByte = 0;
int BM1383_Pres_leastByte = 0;

int BM1383_Temp_Out = 0;
float BM1383_Temp_Conv_Out = 0;
float BM1383_Var = 0;
float BM1383_Deci = 0;
float BM1383_Pres_Conv_Out = 0;
#endif

#ifdef PressureOld
int BM1383_DeviceAddress = 0xBA;  //this is the 8bit address, 7bit address = 0x5D
int BM1383_Temp_highByte = 0;
int BM1383_Temp_lowByte = 0;
int BM1383_Pres_highByte = 0;
int BM1383_Pres_lowByte = 0;
int BM1383_Pres_leastByte = 0;

int BM1383_Temp_Out = 0;
float BM1383_Temp_Conv_Out = 0;
float BM1383_Var = 0;
float BM1383_Deci = 0;
float BM1383_Pres_Conv_Out = 0;
#endif

#ifdef ALSProx
int RPR0521_DeviceAddress = 0x70;  //this is the 8bit address, 7bit address = 0x5D
int RPR0521_PS_HB = 0;
int RPR0521_PS_LB = 0;
int RPR0521_ALS_D0_HB = 0;
int RPR0521_ALS_D0_LB = 0;
int RPR0521_ALS_D1_HB = 0;
int RPR0521_ALS_D1_LB = 0;

int RPR0521_PS_RAWOUT = 0;
float RPR0521_PS_OUT = 0;
unsigned int RPR0521_ALS_D0_RAWOUT = 0;
unsigned int RPR0521_ALS_D1_RAWOUT = 0;
float RPR0521_ALS_DataRatio = 0;
float RPR0521_ALS_OUT = 0;
#endif

#ifdef Color
unsigned int BH1745_DeviceAddress = 0x72;  //this is the 8bit address, 7bit address = 0x4E
unsigned int BH1745_RED_LB = 0;
unsigned int BH1745_RED_HB = 0;
unsigned int BH1745_GRN_LB = 0;
unsigned int BH1745_GRN_HB = 0;
unsigned int BH1745_BLU_LB = 0;
unsigned int BH1745_BLU_HB = 0;

unsigned int BH1745_RED_OUT = 0;
unsigned int BH1745_GRN_OUT = 0;
unsigned int BH1745_BLU_OUT = 0;
#endif

#ifdef KX122
int KX122_DeviceAddress = 0x3C;  //this is the 8bit address, 7bit address = 0x1E
int KX122_Accel_X_LB = 0;
int KX122_Accel_X_HB = 0;
int KX122_Accel_Y_LB = 0;
int KX122_Accel_Y_HB = 0;
int KX122_Accel_Z_LB = 0;
int KX122_Accel_Z_HB = 0;
int KX122_Accel_X_RawOUT = 0;
int KX122_Accel_Y_RawOUT = 0;
int KX122_Accel_Z_RawOUT = 0;
float KX122_Accel_X_OUT = 0;
float KX122_Accel_Y_OUT = 0;
float KX122_Accel_Z_OUT = 0;
#endif


#ifdef KXG03
int         i = 11;
int         t = 1;
short int   aveX = 0;
short int   aveX2 = 0;
short int   aveX3 = 0;
short int   aveY = 0;
short int   aveY2 = 0;
short int   aveY3 = 0;
short int   aveZ = 0;
short int   aveZ2 = 0;
short int   aveZ3 = 0;
int KXG03_DeviceAddress = 0x9E;  //this is the 8bit address, 7bit address = 0x4F
int KXG03_Gyro_X_LB = 0;
int KXG03_Gyro_X_HB = 0;
int KXG03_Gyro_Y_LB = 0;
int KXG03_Gyro_Y_HB = 0;
int KXG03_Gyro_Z_LB = 0;
int KXG03_Gyro_Z_HB = 0;
float KXG03_Gyro_X = 0;
float KXG03_Gyro_Y = 0;                               
float KXG03_Gyro_Z = 0;
short int KXG03_Gyro_X_RawOUT = 0;
short int KXG03_Gyro_Y_RawOUT = 0; 
short int KXG03_Gyro_Z_RawOUT = 0;
short int KXG03_Gyro_X_RawOUT2 = 0;
short int KXG03_Gyro_Y_RawOUT2 = 0; 
short int KXG03_Gyro_Z_RawOUT2 = 0; 
int KXG03_Accel_X_LB = 0;
int KXG03_Accel_X_HB = 0;
int KXG03_Accel_Y_LB = 0;
int KXG03_Accel_Y_HB = 0;
int KXG03_Accel_Z_LB = 0;
int KXG03_Accel_Z_HB = 0; 
float KXG03_Accel_X = 0;
float KXG03_Accel_Y = 0;                               
float KXG03_Accel_Z = 0;  
short int KXG03_Accel_X_RawOUT = 0;
short int KXG03_Accel_Y_RawOUT = 0;
short int KXG03_Accel_Z_RawOUT = 0; 
#endif

#ifdef MagField
unsigned int BM1422_DeviceAddress = 0x1E;
short int BM1422_Mag_X_LB = 0;
short int BM1422_Mag_X_HB = 0;
short int BM1422_Mag_Y_LB = 0;
short int BM1422_Mag_Y_HB = 0;
short int BM1422_Mag_Z_LB = 0;
short int BM1422_Mag_Z_HB = 0;
short int BM1422_Mag_X_RawOUT = 0;
short int BM1422_Mag_Y_RawOUT = 0; 
short int BM1422_Mag_Z_RawOUT = 0;
float BM1422_Mag_X = 0;
float BM1422_Mag_Y = 0;                               
float BM1422_Mag_Z = 0;  
#endif

void setup()
{
  //Wire.begin();        // start I2C functionality
  Serial.begin(115200);  // start serial port at 9600 bps
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  I2C_check = i2c_init();
  if(I2C_check == false){
     while(1){
       Serial.write("I2C Init Failed (SDA or SCL may not be pulled up!");
       Serial.write(0x0A); //Print Line Feed
       Serial.write(0x0D); //Print Carrage Return
       delay(500);
     }
  }

  pinMode(13, OUTPUT);   //Setup for the LED on Board
  
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  //pinMode(13, INPUT_PULLUP);
 
 //----- Start Initialization for KMX62 Digital Accel/Mag Sensor -----
 #ifdef KMX62
 //KMX62 Init Sequence
  // 1. Standby Register (0x29), write 0x03 (Turn Off)
  // 2. Self Test Register (0x60), write 0x00
  // 3. Control Register 1 (0x2A), write 0x13
  // 4. Control Register 2 (0x2B), write 0x00
  // 5. ODCNTL Register (0x2C), write 0x00
  // 6. Temp EN Register (0x4C), write 0x01
  // 7. Buffer CTRL Register 1 (0x78), write 0x00
  // 8. Buffer CTRL Register 2 (0x79), write 0x00
  // 9. Standby Register (0x29), write 0x0u

  //KMX62 Init Sequence
  // 1. CNTL2 (0x3A), write (0x5F): 4g, Max RES, EN temp mag and accel
  
  i2c_start(KMX62_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x3A);
  i2c_write(0x5F);
  i2c_stop();
 #endif
 //----- END Initialization for KMX62 Digital Accel/Mag Sensor -----

 //----- Start Initialization for BM1383A Digital Pressure Sensor -----
 #ifdef Pressure
  //BM1383GLV Init Sequence
  // 1. PWR_DOWN (0x12), write (0x01)
  // 1. SLEEP (0x13), write (0x01)
  // 2. Mode Control (0x14), write (0xC4)
  
  i2c_start(BM1383_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x12);
  i2c_write(0x01);
  i2c_stop();
  
  i2c_start(BM1383_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x13);
  i2c_write(0x01);
  i2c_stop();
  
  i2c_start(BM1383_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x14);
  i2c_write(0xC4);
  i2c_stop();
 #endif
  //----- END Initialization for BM1383A Digital Pressure Sensor -----

//----- Start Initialization for BM1383 Digital Pressure Sensor -----
 #ifdef PressureOld
  //BM1383GLV Init Sequence
  // 1. PWR_DOWN (0x12), write (0x01)
  // 1. SLEEP (0x13), write (0x01)
  // 2. Mode Control (0x14), write (0xC4)
  
  i2c_start(BM1383_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x12);
  i2c_write(0x01);
  i2c_stop();
  
  i2c_start(BM1383_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x13);
  i2c_write(0x01);
  i2c_stop();
  
  i2c_start(BM1383_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x14);
  i2c_write(0xC4);
  i2c_stop();
 #endif
  //----- END Initialization for BM1383 Digital Pressure Sensor -----
  
   //----- Start Initialization for RPR-0521 ALS/PROX Sensor -----
#ifdef ALSProx
  //RPR-0521 Init Sequence
  // 1. Mode Control (0x41), write (0xC6): ALS EN, PS EN, 100ms measurement for ALS and PS, PS_PULSE=1
  // 2. ALS_PS_CONTROL (0x42), write (0x03): LED Current = 200mA
  // 3. PERSIST (0x43), write (0x20): PS Gain x4  
  
  i2c_start(RPR0521_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x41);
  i2c_write(0xE6);
  i2c_stop();
  
  i2c_start(RPR0521_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x42);
  i2c_write(0x03);
  i2c_stop();
  
  i2c_start(RPR0521_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x43);
  i2c_write(0x20);
  i2c_stop();
#endif  
  //----- END Initialization for RPR-0521 ALS/PROX Sensor -----

  //----- Start Initialization for BH1745 Color Sensor -----
#ifdef Color
  //BH1745 Init Sequence
  // 1. Persistence (0x61), write (0x03)
  // 2. Mode Control 1 (0x41), write (0x00)
  // 3. Mode Control 2 (0x42), write (0x92)
  // 4. Mode Control 3 (0x43), write (0x02)
  
  i2c_start(BH1745_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x61);
  i2c_write(0x03);
  i2c_stop();
  
  i2c_start(BH1745_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x41);
  i2c_write(0x00);
  i2c_stop();
  
  i2c_start(BH1745_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x42);
  i2c_write(0x92);
  i2c_stop();
  
  i2c_start(BH1745_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x43);
  i2c_write(0x02);
  i2c_stop();

#endif
  //----- END Initialization for BH1745 Color Sensor -----
  
  //----- Start Initialization for KX122 Accel Sensor -----  
#ifdef KX122
  //1. CNTL1 (0x18) loaded with 0x40 (Set high resolution bit to 1)
  //2. CNTL1 (0x18) loaded with 0xC0 (Enable bit on)
  
  i2c_start(KX122_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x18);
  i2c_write(0x40);
  i2c_stop();
  
  i2c_start(KX122_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x18);
  i2c_write(0xC0);
  i2c_stop();

#endif
  //----- END Initialization for KX122 Accel Sensor -----
  
  //----- Start Initialization for KXG03 Gyro Sensor -----
#ifdef KXG03
  //1. STBY REG (0x43) loaded with 0xEF

  i2c_start(KXG03_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x43);
  i2c_write(0x00);
  i2c_stop();

  //i2c_start(KXG03_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  //i2c_write(0x41);
  //i2c_write(0x06);
  //i2c_stop();

  //i2c_start(KXG03_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  //i2c_write(0x43);
  //i2c_write(0xED);
  //i2c_stop();

  /*
  //2. STBY REG (0x43) loaded with 0xEF

  i2c_start(KXG03_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x43);
  i2c_write(0xEF);
  i2c_stop();

  i2c_start(KXG03_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x3E);
  i2c_write(0xD6);
  i2c_stop();

  i2c_start(KXG03_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x40);
  i2c_write(0x00);
  i2c_stop();

  i2c_start(KXG03_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x43);
  i2c_write(0xEE);
  i2c_stop();
  */

#endif
  //----- END Initialization for KXG03 Gyro Sensor -----  

#ifdef MagField
  i2c_start(BM1422_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x1B);
  i2c_write(0xC0);
  i2c_stop();
  
  i2c_start(BM1422_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x5C);
  i2c_write(0x00);
  i2c_stop();

  i2c_start(BM1422_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x5D);
  i2c_write(0x00);
  i2c_stop();

/*
  i2c_start(BM1422_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x1C);
  i2c_write(0x08);
  i2c_stop();
*/

  i2c_start(BM1422_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x1D);
  i2c_write(0x40);
  i2c_stop();
#endif

  //Prepare Logging Header
  #ifdef CSVOutput
  Serial.write("Sample Time (s),");
  
    #ifdef AnalogTemp
    Serial.write("BDE0600G Temp (degrees C),");
    #endif
    
    #ifdef AnalogUV
    Serial.write("ML8511 UV Sensor (mW/cm2),");
    #endif
    
    #ifdef HallSen
    Serial.write("BU52011HFV South Detect (0 = mag present; 1 = no Mag field),BU52011HFV North Detect (0 = mag present; 1 = no Mag field),");
    #endif
    
    #ifdef KMX62
    Serial.write("KMX62 Accel Xout (g),KMX62 Accel Yout (g),KMX62 Accel Zout (g),KMX62 Mag Xout (uT),KMX62 Mag Yout (uT),KMX62 Mag Zout (uT),");
    #endif
    
    #ifdef Pressure
    Serial.write("BM1383A Temp (degrees C),BM1383A Pressure (hPa),");
    #endif
    
    #ifdef PressureOld
    Serial.write("BM1383 Temp (degrees C),BM1383 Pressure (hPa),");
    #endif
    
    #ifdef ALSProx
    Serial.write("RPR-0521 Proximity (ADC Count),RPR-0521 Ambient Light Sensor (Lx),");
    #endif 
    
    #ifdef Color
    Serial.write("BH1745 Color RED (ADC Count),BH1745 Color GREEN (ADC Count),BH1745 Color BLUE (ADC Count),");
    #endif
    
    #ifdef KX122
    Serial.write("KX122 Accel Xout (g),KX122 Accel Yout (g),KX122 Accel Zout (g),");
    #endif
    
    #ifdef KXG03
    Serial.write("KXG03 Gyro X (deg/sec),KXG03 Gyro Y (deg/sec),KXG03 Gyro Z (deg/sec),");
    #endif
    
    #ifdef MagField
    Serial.write("BM1422 Mag Xout (uT),BM1422 Mag Yout (uT),BM1422 Mag Zout (uT),");
    #endif
  
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  #endif

}

void loop()
{
  /* //How to use the Ardiino LED
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(250);              // wait for 250ms
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(250);              // wait for 250ms
  */

  #ifndef CSVOutput
  //Clear the Display to make data easier to read
  for(j = 0; j<lineClear; j++){
    Serial.write("\033[F");
    Serial.write("\033[J");
  }
  lineClear = 0;
  #endif

  #ifdef CSVOutput
  Serial.print(intervalTime);
  Serial.write(",");
  #endif
  
  //---------- Start Code for Reading BDE0600G Analog Temperature Sensor ----------
  #ifdef AnalogTemp

  //----- Start ADC Read from Port A2 ----
  //Notes on Arduino ADC
  //Arduino uses an 5V ADC Reference Voltage; thus, we will need to scale this 
  //to 3.3V Levels to safely operate the sensors
  //5V = 1024, 3.3V = 670
  sensorValue = analogRead(ADCpin_AnalogTemp);
  
  /* //Uncomment this for Raw Values
  Serial.write("ADC Raw Value Return = ");
  Serial.print(sensorValue);
  Serial.write(0x0A); //Print Line Feed
  Serial.write(0x0D); //Print Carrage Return
  */
  
  //Calculations for Analog Temp Sensor - BDE0600G
  // Temperature Sensitivity = -10.68mV/degC
  // Temperature Sensitivity = -0.01068V/degC
  // Temperature Sensitivity = -93.63degC/V
  // Temp Known Point = 1.753V @ 30DegC
  
  // Math:  ADC_Voltage = (sensorValue / 670) * 3.3V              //Convert to V
  //        ADC_Voltage = sensorValue * (3.3V/670)
  //        ADC_Voltage = sensorValue * 0.004925
  //        Temperature = (ADC_Voltage - 1.753)/(-0.01068) + 30

  
  sensorConvert = (float)sensorValue * 0.004925;
  sensorConvert = (sensorConvert - 1.753)/(-0.01068)+30;
  
  #ifndef CSVOutput
  Serial.write("BDE0600G Temp = ");
  Serial.print(sensorConvert);
  Serial.write(" degC");
  Serial.write(0x0A); //Print Line Feed
  Serial.write(0x0D); //Print Carrage Return
  lineClear += 1;
  #endif

  #ifdef CSVOutput
  Serial.print(sensorConvert);
  Serial.write(",");
  #endif
  
  #endif
  //---------- End Code for Reading BDE0600G Analog Temperature Sensor ----------
  
  //---------- Start Code for Reading ML8511 Analog UV Sensor ----------
  #ifdef AnalogUV

  //----- Start ADC Read from Port A0 ----
  //Notes on Arduino ADC
  //Arduino uses an 5V ADC Reference Voltage; thus, we will need to scale this 
  //to 3.3V Levels to safely operate the sensors
  //5V = 1024, 3.3V = 670
  sensorValue = analogRead(ADCpin_AnalogUV);
  
  /* //Uncomment this for Raw Values
  Serial.write("ADC Raw Value Return = ");
  Serial.print(sensorValue);
  Serial.write(0x0A); //Print Line Feed
  Serial.write(0x0D); //Print Carrage Return
  */
  
  //Calculations for UV Sensor - ML8511
  //Known Point = 2.2V @ 10mW/cm2
  //Rate = 0.129
  
  // Math:  ADC_Voltage = (sensorValue / 670) * 3.3V              //Convert to V
  //        ADC_Voltage = sensorValue * (3.3V/670)
  //        ADC_Voltage = sensorValue * 0.004925
  //        UV Intensity = (ADC_Voltage - 2.2)/(0.129) + 10   //Conver to Temperature in DegC

  sensorConvert = (float)sensorValue * 0.004925;
  sensorConvert = (sensorConvert - 2.2)/(0.129)+10;

  #ifndef CSVOutput
  Serial.write("ML8511 UV Sensor = ");
  Serial.print(sensorConvert);
  Serial.write(" mW/cm2");
  Serial.write(0x0A); //Print Line Feed
  Serial.write(0x0D); //Print Carrage Return
  lineClear += 1;
  #endif

  #ifdef CSVOutput
  Serial.print(sensorConvert);
  Serial.write(",");
  #endif
  
  #endif
  //---------- End Code for Reading ML8511 Analog UV Sensor ----------
  
  //---------- Start Code for Reading BU52011HFV Hall Sensor ----------
  #ifdef HallSen
  //Hardware Connection
  //For the Hall Sensor, we only need to monitor digital inputs for OUT1 and OUT2 Pins of the Hall Sensor
  //Connect Pin1 of Hall Sensor Header U111 to Arduino Pin2
  //Connect Pin5 of Hall Sensor Header U111 to Arduino Pin3
  Hall_Out0 = digitalRead(2);

  #ifndef CSVOutput
  Serial.write("BU52011HFV South Detect = ");
  Serial.print(Hall_Out0);
  Serial.write(0x0A); //Print Line Feed
  Serial.write(0x0D); //Print Carrage Return
  lineClear += 1;
  #endif

  #ifdef CSVOutput
  Serial.print(Hall_Out0);
  Serial.write(",");
  #endif
  
  Hall_Out1 = digitalRead(3);

  #ifndef CSVOutput
  Serial.write("BU52011HFV North Detect = ");
  Serial.print(Hall_Out1);
  Serial.write(0x0A); //Print Line Feed
  Serial.write(0x0D); //Print Carrage Return
  lineClear += 1;
  #endif

  #ifdef CSVOutput
  Serial.print(Hall_Out1);
  Serial.write(",");
  #endif
  
  #endif
  //---------- End Code for Reading BU52011HFV Hall Sensor ----------
  
  //---------- Start Code for Reading KMX62 Kionix Accelerometer+Mag Sensor ----------  
  #ifdef KMX62
  // -- Notes on Arduino I2C Connection --
  //I2C is built in using using the "wire" library
  //Uno, Ethernet  A4 (SDA), A5 (SCL) [UNO - THIS is the platform we are using]
  //Mega2560          20 (SDA), 21 (SCL)
  //Leonardo          2  (SDA), 3  (SCL)
  //Due                 20 (SDA), 21 (SCL), SDA1, SCL1
  
  // -- Notes on ROHM KMX62 Accel/Mag Sensor --
  //Device Address = 0x0Eu
  //12 Bit Return Value
  
  //Intialization Routines (See Setup function above)
  // 1. Standby Register (0x29), write 0x03 (Turn Off)
  // 2. Self Test Register (0x60), write 0x00
  // 3. Control Register 1 (0x2A), write 0x13
  // 4. Control Register 2 (0x2B), write 0x00
  // 5. ODCNTL Register (0x2C), write 0x00
  // 6. Temp EN Register (0x4C), write 0x01
  // 7. Buffer CTRL Register 1 (0x78), write 0x00
  // 8. Buffer CTRL Register 2 (0x79), write 0x00
  // 9. Standby Register (0x29), write 0x0u
  
  //Main Loop Routines
  // 1. Read 6 Bytes starting from address 0x0A.  These will be the accelerometer output. [0][1]...[5]
  // 2. Xout = ([1]<<6) | ([0]>>2)
  // 3. Yout = ([3]<<6) | ([2]>>2)
  // 4. Zout = ([5]<<6) | ([4]>>2)
  // 5. Read 6 Bytes starting from addres 0x12. These will be the magnetometer output. [0][1]...[5]  
  // 6. Xout = ([1]<<6) | ([0]>>2)
  // 7. Yout = ([3]<<6) | ([2]>>2)
  // 8. Zout = ([5]<<6) | ([4]>>2)
  
  // Start Getting Data from Accel
  i2c_start(KMX62_DeviceAddress);
  i2c_write(0x0A);
  i2c_rep_start(KMX62_DeviceAddress | 1);  // Or-ed with "1" for read bit
  MEMS_Accel_Xout_lowByte = i2c_read(false);
  MEMS_Accel_Xout_highByte = i2c_read(false);
  MEMS_Accel_Yout_lowByte = i2c_read(false);
  MEMS_Accel_Yout_highByte = i2c_read(false);
  MEMS_Accel_Zout_lowByte = i2c_read(false);
  MEMS_Accel_Zout_highByte = i2c_read(true);
  i2c_stop();
  
  //Note: The highbyte and low byte return a 14bit value, dropping the two LSB in the Low byte.
  //      However, because we need the signed value, we will adjust the value when converting to "g"
  MEMS_Accel_Xout = (MEMS_Accel_Xout_highByte<<8) | (MEMS_Accel_Xout_lowByte);
  MEMS_Accel_Yout = (MEMS_Accel_Yout_highByte<<8) | (MEMS_Accel_Yout_lowByte);
  MEMS_Accel_Zout = (MEMS_Accel_Zout_highByte<<8) | (MEMS_Accel_Zout_lowByte);
  
  MEMS_Accel_Conv_Xout = (float)MEMS_Accel_Xout/8192;
  MEMS_Accel_Conv_Yout = (float)MEMS_Accel_Yout/8192;
  MEMS_Accel_Conv_Zout = (float)MEMS_Accel_Zout/8192;
  
  /*  //Uncomment if you want to see the Raw Sensor Output
  Serial.write("Digital MEMS Raw Accel Xout = ");
  Serial.print(MEMS_Accel_Xout);  
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("Digital MEMS Raw Accel Yout = ");
  Serial.print(MEMS_Accel_Yout);  
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return  
  Serial.write("Digital MEMS Raw Accel Zout = ");
  Serial.print(MEMS_Accel_Zout);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  */

  #ifndef CSVOutput
  Serial.write("KMX62 Accel Xout = ");
  Serial.print(MEMS_Accel_Conv_Xout);
  Serial.write(" g");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KMX62 Accel Yout = ");
  Serial.print(MEMS_Accel_Conv_Yout);
  Serial.write(" g");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return  
  Serial.write("KMX62 Accel Zout = ");
  Serial.print(MEMS_Accel_Conv_Zout);
  Serial.write(" g");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  lineClear += 3;
  #endif

  #ifdef CSVOutput
  Serial.print(MEMS_Accel_Conv_Xout);
  Serial.write(",");
  Serial.print(MEMS_Accel_Conv_Yout);
  Serial.write(",");
  Serial.print(MEMS_Accel_Conv_Zout);
  Serial.write(",");
  #endif
  
  // Start Getting Data from Mag Sensor
  i2c_start(KMX62_DeviceAddress);
  i2c_write(0x10);
  i2c_rep_start(KMX62_DeviceAddress | 1);  // Or-ed with "1" for read bit
  MEMS_Mag_Xout_lowByte = i2c_read(false);
  MEMS_Mag_Xout_highByte = i2c_read(false);
  MEMS_Mag_Yout_lowByte = i2c_read(false);
  MEMS_Mag_Yout_highByte = i2c_read(false);
  MEMS_Mag_Zout_lowByte = i2c_read(false);
  MEMS_Mag_Zout_highByte = i2c_read(true);
  i2c_stop();
  
  MEMS_Mag_Xout = (MEMS_Mag_Xout_highByte<<8) | (MEMS_Mag_Xout_lowByte);
  MEMS_Mag_Yout = (MEMS_Mag_Yout_highByte<<8) | (MEMS_Mag_Yout_lowByte);
  MEMS_Mag_Zout = (MEMS_Mag_Zout_highByte<<8) | (MEMS_Mag_Zout_lowByte);
  
  MEMS_Mag_Conv_Xout = (float)MEMS_Mag_Xout/27.30666619;
  MEMS_Mag_Conv_Yout = (float)MEMS_Mag_Yout/27.30666619;
  MEMS_Mag_Conv_Zout = (float)MEMS_Mag_Zout/27.30666619;
  
  /*  //Uncomment if you want to see the Raw Sensor Output  
  Serial.write("Digital MEMS Raw Mag Xout = ");
  Serial.print(MEMS_Mag_Xout);  
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("Digital MEMS Raw Mag Yout = ");
  Serial.print(MEMS_Mag_Yout);  
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return  
  Serial.write("Digital MEMS Raw Mag Zout = ");
  Serial.print(MEMS_Mag_Zout);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  lineClear += 3;
  */
  
  #ifndef CSVOutput
  Serial.write("KMX62 Mag Xout = ");
  Serial.print(MEMS_Mag_Conv_Xout);
  Serial.write(" uT");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KMX62 Mag Yout = ");
  Serial.print(MEMS_Mag_Conv_Yout);
  Serial.write(" uT");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return  
  Serial.write("KMX62 Mag Zout = ");
  Serial.print(MEMS_Mag_Conv_Zout);
  Serial.write(" uT");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  lineClear += 3;
  #endif

  #ifdef CSVOutput
  Serial.print(MEMS_Mag_Conv_Xout);
  Serial.write(",");
  Serial.print(MEMS_Mag_Conv_Yout);
  Serial.write(",");
  Serial.print(MEMS_Mag_Conv_Zout);
  Serial.write(",");
  #endif
  
  #endif
  //---------- END Code for Reading KMX62 Kionix Accelerometer+Mag Sensor ----------    
 
  //---------- Start Code for Reading BM1383A Pressure Sensor ----------  
  #ifdef Pressure
  
  // Start Getting Data from Pressure Sensor
  i2c_start(BM1383_DeviceAddress);
  i2c_write(0x1A);
  i2c_rep_start(BM1383_DeviceAddress | 1);  // Or-ed with "1" for read bit
  //For new version of Pressure Sensor (BM1383AGLV)
  BM1383_Pres_highByte = i2c_read(false);
  BM1383_Pres_lowByte = i2c_read(false);
  BM1383_Pres_leastByte = i2c_read(false);
  BM1383_Temp_highByte = i2c_read(false);
  BM1383_Temp_lowByte = i2c_read(true);
/*
  //For Old version of PRessure Sensor (BM1383GLV)
  BM1383_Temp_highByte = i2c_read(false);
  BM1383_Temp_lowByte = i2c_read(false);
  BM1383_Pres_highByte = i2c_read(false);
  BM1383_Pres_lowByte = i2c_read(false);
  BM1383_Pres_leastByte = i2c_read(true);
*/
  i2c_stop();
 
  BM1383_Temp_Out = (BM1383_Temp_highByte<<8) | (BM1383_Temp_lowByte);
  BM1383_Temp_Conv_Out = (float)BM1383_Temp_Out/32;
  
  BM1383_Var  = (BM1383_Pres_highByte<<3) | (BM1383_Pres_lowByte >> 5);
  BM1383_Deci = ((BM1383_Pres_lowByte & 0x1f) << 6 | ((BM1383_Pres_leastByte >> 2)));
  BM1383_Deci = (float)BM1383_Deci* 0.00048828125;  //0.00048828125 = 2^-11
  BM1383_Pres_Conv_Out = (BM1383_Var + BM1383_Deci);   //question pending here...

  #ifndef CSVOutput
  Serial.write("BM1383 (Temp) = ");
  Serial.print(BM1383_Temp_Conv_Out);
  Serial.write(" degC");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("BM1383 (Pres) = ");
  Serial.print(BM1383_Pres_Conv_Out);
  Serial.write(" hPa");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  lineClear += 2;
  #endif

  #ifdef CSVOutput
  Serial.print(BM1383_Temp_Conv_Out);
  Serial.write(",");
  Serial.print(BM1383_Pres_Conv_Out);
  Serial.write(",");
  #endif
  
  #endif
  //---------- END Code for Reading BM1383A Pressure Sensor ----------    

  //---------- Start Code for Reading BM1383 Pressure Sensor ----------  
  #ifdef PressureOld
  
  // Start Getting Data from Pressure Sensor
  i2c_start(BM1383_DeviceAddress);
  i2c_write(0x1A);
  i2c_rep_start(BM1383_DeviceAddress | 1);  // Or-ed with "1" for read bit
  //For Old version of PRessure Sensor (BM1383GLV)
  BM1383_Temp_highByte = i2c_read(false);
  BM1383_Temp_lowByte = i2c_read(false);
  BM1383_Pres_highByte = i2c_read(false);
  BM1383_Pres_lowByte = i2c_read(false);
  BM1383_Pres_leastByte = i2c_read(true);

  i2c_stop();
 
  BM1383_Temp_Out = (BM1383_Temp_highByte<<8) | (BM1383_Temp_lowByte);
  BM1383_Temp_Conv_Out = (float)BM1383_Temp_Out/32;
  
  BM1383_Var  = (BM1383_Pres_highByte<<3) | (BM1383_Pres_lowByte >> 5);
  BM1383_Deci = ((BM1383_Pres_lowByte & 0x1f) << 6 | ((BM1383_Pres_leastByte >> 2)));
  BM1383_Deci = (float)BM1383_Deci* 0.00048828125;  //0.00048828125 = 2^-11
  BM1383_Pres_Conv_Out = (BM1383_Var + BM1383_Deci);   //question pending here...

  #ifndef CSVOutput
  Serial.write("BM1383 (Temp) = ");
  Serial.print(BM1383_Temp_Conv_Out);
  Serial.write(" degC");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("BM1383 (Pres) = ");
  Serial.print(BM1383_Pres_Conv_Out);
  Serial.write(" hPa");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  lineClear += 2;
  #endif
  
  #ifdef CSVOutput
  Serial.print(BM1383_Temp_Conv_Out);
  Serial.write(",");
  Serial.print(BM1383_Pres_Conv_Out);
  Serial.write(",");
  #endif
  
  #endif
  //---------- END Code for Reading BM1383 Pressure Sensor ----------

   //----- Start Code for Reading RPR-0521 ALS/PROX Sensor -----
  #ifdef ALSProx

  // Start Getting Data from ALS/PROX Sensor
  i2c_start(RPR0521_DeviceAddress);
  i2c_write(0x42);
  i2c_rep_start(RPR0521_DeviceAddress | 1);  // Or-ed with "1" for read bit
  RPR0521_PS_LB = i2c_read(false);
  RPR0521_PS_HB = i2c_read(false);
  RPR0521_PS_LB = i2c_read(false);
  RPR0521_PS_HB = i2c_read(false);
  RPR0521_ALS_D0_LB = i2c_read(false);
  RPR0521_ALS_D0_HB = i2c_read(false);
  RPR0521_ALS_D1_LB = i2c_read(false);
  RPR0521_ALS_D1_HB = i2c_read(true);
  i2c_stop();

  RPR0521_PS_RAWOUT = (RPR0521_PS_HB<<8) | (RPR0521_PS_LB);
  RPR0521_ALS_D0_RAWOUT = (RPR0521_ALS_D0_HB<<8) | (RPR0521_ALS_D0_LB);
  RPR0521_ALS_D1_RAWOUT = (RPR0521_ALS_D1_HB<<8) | (RPR0521_ALS_D1_LB);
  RPR0521_ALS_DataRatio = (float)RPR0521_ALS_D1_RAWOUT / (float)RPR0521_ALS_D0_RAWOUT;
  
  if(RPR0521_ALS_DataRatio < 0.595){
    RPR0521_ALS_OUT = (1.682*(float)RPR0521_ALS_D0_RAWOUT - 1.877*(float)RPR0521_ALS_D1_RAWOUT);
  }
  else if(RPR0521_ALS_DataRatio < 1.015){
    RPR0521_ALS_OUT = (0.644*(float)RPR0521_ALS_D0_RAWOUT - 0.132*(float)RPR0521_ALS_D1_RAWOUT);
  }
  else if(RPR0521_ALS_DataRatio < 1.352){
    RPR0521_ALS_OUT = (0.756*(float)RPR0521_ALS_D0_RAWOUT - 0.243*(float)RPR0521_ALS_D1_RAWOUT);
  }
  else if(RPR0521_ALS_DataRatio < 3.053){
    RPR0521_ALS_OUT = (0.766*(float)RPR0521_ALS_D0_RAWOUT - 0.25*(float)RPR0521_ALS_D1_RAWOUT);
  }
  else{
    RPR0521_ALS_OUT = 0;
  }

  #ifndef CSVOutput
  Serial.write("RPR-0521 (Prox) = ");
  Serial.print(RPR0521_PS_RAWOUT);
  Serial.write(" ADC Counts");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("RPR-0521 (ALS)= ");
  Serial.print(RPR0521_ALS_OUT);
  Serial.write(" lx");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  lineClear += 2;
  #endif

  #ifdef CSVOutput
  Serial.print(RPR0521_PS_RAWOUT);
  Serial.write(",");
  Serial.print(RPR0521_ALS_OUT);
  Serial.write(",");
  #endif
  
  #endif  
  //----- END Code for Reading RPR-0521 ALS/PROX Sensor -----

  //----- START Code for Reading Color Sensor -----
  #ifdef Color

  // Start Getting Data from COLOR Sensor
  i2c_start(BH1745_DeviceAddress);
  i2c_write(0x50);
  i2c_rep_start(BH1745_DeviceAddress | 1);  // Or-ed with "1" for read bit
  BH1745_RED_LB = i2c_read(false);
  BH1745_RED_HB = i2c_read(false);
  BH1745_GRN_LB = i2c_read(false);
  BH1745_GRN_HB = i2c_read(false);
  BH1745_BLU_LB = i2c_read(false);
  BH1745_BLU_HB = i2c_read(true);
  i2c_stop();

  BH1745_RED_OUT = (BH1745_RED_HB<<8) | (BH1745_RED_LB);
  BH1745_GRN_OUT = (BH1745_GRN_HB<<8) | (BH1745_GRN_LB);
  BH1745_BLU_OUT = (BH1745_BLU_HB<<8) | (BH1745_BLU_LB);

  #ifndef CSVOutput
  Serial.write("BH1745 (Red) = ");
  Serial.print(BH1745_RED_OUT);
  Serial.write(" ADC Counts");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("BH1745 (Green) = ");
  Serial.print(BH1745_GRN_OUT);
  Serial.write(" ADC Counts");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("BH1745 (Blue) = ");
  Serial.print(BH1745_BLU_OUT);
  Serial.write(" ADC Counts");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  lineClear += 3;
  #endif

  #ifdef CSVOutput
  Serial.print(BH1745_RED_OUT);
  Serial.write(",");
  Serial.print(BH1745_GRN_OUT);
  Serial.write(",");
  Serial.print(BH1745_BLU_OUT);
  Serial.write(",");
  #endif
  
  #endif
  //----- END Code for Reading Color Sensor -----
  
  //----- START Code for Reading KX122 Accel Sensor -----
#ifdef KX122
  i2c_start(KX122_DeviceAddress);
  i2c_write(0x06);
  i2c_rep_start(KX122_DeviceAddress | 1);  // Or-ed with "1" for read bit
  KX122_Accel_X_LB = i2c_read(false);
  KX122_Accel_X_HB = i2c_read(false);
  KX122_Accel_Y_LB = i2c_read(false);
  KX122_Accel_Y_HB = i2c_read(false);
  KX122_Accel_Z_LB = i2c_read(false);
  KX122_Accel_Z_HB = i2c_read(true);
  i2c_stop();

  KX122_Accel_X_RawOUT = (KX122_Accel_X_HB<<8) | (KX122_Accel_X_LB);
  KX122_Accel_Y_RawOUT = (KX122_Accel_Y_HB<<8) | (KX122_Accel_Y_LB);
  KX122_Accel_Z_RawOUT = (KX122_Accel_Z_HB<<8) | (KX122_Accel_Z_LB);

  KX122_Accel_X_OUT = (float)KX122_Accel_X_RawOUT / 16384;
  KX122_Accel_Y_OUT = (float)KX122_Accel_Y_RawOUT / 16384;
  KX122_Accel_Z_OUT = (float)KX122_Accel_Z_RawOUT / 16384;

  #ifndef CSVOutput
  Serial.write("KX122 (X) = ");
  Serial.print(KX122_Accel_X_OUT);
  Serial.write(" g");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KX122 (Y) = ");
  Serial.print(KX122_Accel_Y_OUT);
  Serial.write(" g");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KX122 (Z) = ");
  Serial.print(KX122_Accel_Z_OUT);
  Serial.write(" g");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  lineClear += 3;
  #endif

  #ifdef CSVOutput
  Serial.print(KX122_Accel_X_OUT);
  Serial.write(",");
  Serial.print(KX122_Accel_Y_OUT);
  Serial.write(",");
  Serial.print(KX122_Accel_Z_OUT);
  Serial.write(",");
  #endif

#endif
  //----- END Code for Reading KX122 Accel Sensor -----
  
  //----- START Code for Reading KXG03 Gyro Sensor -----
#ifdef KXG03

  // Calibration for offset data
if (t == 1){
  int i = 11;
  while(--i){
  i2c_start(KXG03_DeviceAddress);
  i2c_write(0x02);
  i2c_rep_start(KXG03_DeviceAddress | 1);
  KXG03_Gyro_X_LB = i2c_read(false);
  KXG03_Gyro_X_HB = i2c_read(false);
  KXG03_Gyro_Y_LB = i2c_read(false);
  KXG03_Gyro_Y_HB = i2c_read(false);
  KXG03_Gyro_Z_LB = i2c_read(false);
  KXG03_Gyro_Z_HB = i2c_read(true);
  i2c_stop();

  KXG03_Gyro_X_RawOUT = (KXG03_Gyro_X_HB<<8) | (KXG03_Gyro_X_LB);
  KXG03_Gyro_Y_RawOUT = (KXG03_Gyro_Y_HB<<8) | (KXG03_Gyro_Y_LB);
  KXG03_Gyro_Z_RawOUT = (KXG03_Gyro_Z_HB<<8) | (KXG03_Gyro_Z_LB);

  aveX = KXG03_Gyro_X_RawOUT;
  aveY = KXG03_Gyro_Y_RawOUT;
  aveZ = KXG03_Gyro_Z_RawOUT;
  aveX2 = aveX2 + aveX;
  aveY2 = aveY2 + aveY;
  aveZ2 = aveZ2 + aveZ; 
  }

  aveX3 = aveX2 / 10;
  aveY3 = aveY2 / 10;
  aveZ3 = aveZ2 / 10;

  t = 0;

  #ifndef CSVOutput
  Serial.write("KXG03 Gyro (X) = 0");
  Serial.write(" deg/sec");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KXG03 Gyro (Y) = 0");
  Serial.write(" deg/sec");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KXG03 Gyro (Z) = 0");
  Serial.write(" deg/sec");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  lineClear += 3;
  #endif

  #ifdef CSVOutput
  Serial.write("0,");
  Serial.write("0,");
  Serial.write("0,");
  #endif

}

  // Start Getting Data from KXG03 Sensor
else {
  i2c_start(KXG03_DeviceAddress);
  i2c_write(0x02);
  i2c_rep_start(KXG03_DeviceAddress | 1);
  KXG03_Gyro_X_LB = i2c_read(false);
  KXG03_Gyro_X_HB = i2c_read(false);
  KXG03_Gyro_Y_LB = i2c_read(false);
  KXG03_Gyro_Y_HB = i2c_read(false);
  KXG03_Gyro_Z_LB = i2c_read(false);
  KXG03_Gyro_Z_HB = i2c_read(true);
  i2c_stop();
  
  KXG03_Gyro_X_RawOUT = (KXG03_Gyro_X_HB<<8) | (KXG03_Gyro_X_LB);
  KXG03_Gyro_Y_RawOUT = (KXG03_Gyro_Y_HB<<8) | (KXG03_Gyro_Y_LB);
  KXG03_Gyro_Z_RawOUT = (KXG03_Gyro_Z_HB<<8) | (KXG03_Gyro_Z_LB);

  KXG03_Gyro_X_RawOUT2 = KXG03_Gyro_X_RawOUT - aveX3;
  KXG03_Gyro_Y_RawOUT2 = KXG03_Gyro_Y_RawOUT - aveY3;
  KXG03_Gyro_Z_RawOUT2 = KXG03_Gyro_Z_RawOUT - aveZ3;

  //Scale Data
  KXG03_Gyro_X = (float)KXG03_Gyro_X_RawOUT2 * 0.007813 + 0.000004;
  KXG03_Gyro_Y = (float)KXG03_Gyro_Y_RawOUT2 * 0.007813 + 0.000004;
  KXG03_Gyro_Z = (float)KXG03_Gyro_Z_RawOUT2 * 0.007813 + 0.000004; 

  #ifndef CSVOutput
  Serial.write("KXG03 Gyro (X) = ");
  Serial.print(KXG03_Gyro_X);
  Serial.write(" deg/sec");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KXG03 Gyro (Y) = ");
  Serial.print(KXG03_Gyro_Y);
  Serial.write(" deg/sec");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KXG03 Gyro (Z) = ");
  Serial.print(KXG03_Gyro_Z);
  Serial.write(" deg/sec");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  lineClear += 3;
  #endif

  #ifdef CSVOutput
  Serial.print(KXG03_Gyro_X);
  Serial.write(",");
  Serial.print(KXG03_Gyro_Y);
  Serial.write(",");
  Serial.print(KXG03_Gyro_Z);
  Serial.write(",");
  #endif
  
  /*
  //Accel Data 
  i2c_start(KXG03_DeviceAddress);
  i2c_write(0x08);
  i2c_rep_start(KXG03_DeviceAddress | 1);
  KXG03_Accel_X_LB = i2c_read(false);
  KXG03_Accel_X_HB = i2c_read(false);
  KXG03_Accel_Y_LB = i2c_read(false);
  KXG03_Accel_Y_HB = i2c_read(false);
  KXG03_Accel_Z_LB = i2c_read(false);
  KXG03_Accel_Z_HB = i2c_read(true);
  i2c_stop();

  KXG03_Accel_X_RawOUT = (KXG03_Accel_X_HB<<8) | (KXG03_Accel_X_LB);
  KXG03_Accel_Y_RawOUT = (KXG03_Accel_Y_HB<<8) | (KXG03_Accel_Y_LB);
  KXG03_Accel_Z_RawOUT = (KXG03_Accel_Z_HB<<8) | (KXG03_Accel_Z_LB);

  //Scale Data
  KXG03_Accel_X = (float)KXG03_Accel_X_RawOUT * 0.000061 + 0.000017;
  KXG03_Accel_Y = (float)KXG03_Accel_Y_RawOUT * 0.000061 + 0.000017;
  KXG03_Accel_Z = (float)KXG03_Accel_Z_RawOUT * 0.000061 + 0.000017; 

  Serial.write("KXG03 ACC (X) = ");
  Serial.print(KXG03_Accel_X);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KXG03 ACC (Y) = ");
  Serial.print(KXG03_Accel_Y);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KXG03 ACC (Z) = ");
  Serial.print(KXG03_Accel_Z);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  */

  aveX2 = 0;
  aveY2 = 0;
  aveZ2 = 0;
}

#endif
  //----- END Code for Reading KXG03 Gyro Sensor -----  

#ifdef MagField
  i2c_start(BM1422_DeviceAddress);
  i2c_write(0x10);
  i2c_rep_start(BM1422_DeviceAddress | 1);
  BM1422_Mag_X_LB = i2c_read(false);
  BM1422_Mag_X_HB = i2c_read(false);
  BM1422_Mag_Y_LB = i2c_read(false);
  BM1422_Mag_Y_HB = i2c_read(false);
  BM1422_Mag_Z_LB = i2c_read(false);
  BM1422_Mag_Z_HB = i2c_read(true);
  i2c_stop();
  
  BM1422_Mag_X_RawOUT = (BM1422_Mag_X_HB<<8) | (BM1422_Mag_X_LB);
  BM1422_Mag_Y_RawOUT = (BM1422_Mag_Y_HB<<8) | (BM1422_Mag_Y_LB);
  BM1422_Mag_Z_RawOUT = (BM1422_Mag_Z_HB<<8) | (BM1422_Mag_Z_LB);
  
  BM1422_Mag_X = BM1422_Mag_X_RawOUT * 0.042;
  BM1422_Mag_Y = BM1422_Mag_Y_RawOUT * 0.042;
  BM1422_Mag_Z = BM1422_Mag_Z_RawOUT * 0.042;

  #ifndef CSVOutput
  Serial.write("BM1422 Mag (X) = ");
  Serial.print(BM1422_Mag_X);
  Serial.write("uT");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("BM1422 Mag (Y) = ");
  Serial.print(BM1422_Mag_Y);
  Serial.write("uT");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("BM1422 Mag (Z) = ");
  Serial.print(BM1422_Mag_Z);
  Serial.write("uT");
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  lineClear += 3;
  #endif

  #ifdef CSVOutput
  Serial.print(BM1422_Mag_X);
  Serial.write(",");
  Serial.print(BM1422_Mag_Y);
  Serial.write(",");
  Serial.print(BM1422_Mag_Z);
  Serial.write(",");
  #endif
  
  #endif

  #ifdef CSVOutput
  intervalTime += (double)SensorSamplePeriod/1000;
  Serial.write(0x0A); //Print Line Feed
  Serial.write(0x0D); //Print Carrage Return
  #endif

  delay(SensorSamplePeriod);  //Add some Loop Delay
}

void I2C_CheckACK()
{
  if(I2C_check == false){
     while(1){
       Serial.write("No ACK!");
       Serial.write(0x0A); //Print Line Feed
       Serial.write(0x0D); //Print Carrage Return
       delay(500);
     }
  }
}
