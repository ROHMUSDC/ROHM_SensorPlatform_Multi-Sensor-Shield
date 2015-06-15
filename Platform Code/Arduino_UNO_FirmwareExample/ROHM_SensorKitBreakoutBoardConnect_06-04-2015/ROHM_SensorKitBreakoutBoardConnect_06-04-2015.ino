/*------------------------------------------------------------------------------
 ROHM Sensor Platform Breakout Board Sensor Return Application
 
 This program reads the value of the connected ROHM Sensors and 
 returns the sensor output using the serial port
 
 Created 13 January 2015
 by ROHM USDC Applications Engineering Team
 
 Revision 01: Changed the pin functionality of the board to work with the Arduino "shield" board 
------------------------------------------------------------------------------*/

// ----- Debugging Definitions -----
#define AnalogTemp
#define AnalogUV
#define HallSen
#define KMX62
#define Pressure
#define ALSProx
#define Color
#define KX022

// ----- Included Files -----
//#include <Wire.h>         //Default I2C Library

/*  //Original Definition
#define SCL_PIN 4  //A5       //Note that if you are using the Accel/Mag Sensor, you will need to download and
#define SCL_PORT PORTD    //install the "SoftI2CMaster" as "Wire" does not support repeated start...
#define SDA_PIN 5  //A4         //References:
#define SDA_PORT PORTD    //  http://playground.arduino.cc/Main/SoftwareI2CLibrary
*/
#define SCL_PIN 5  //A5       //Note that if you are using the Accel/Mag Sensor, you will need to download and
#define SCL_PORT PORTC    //install the "SoftI2CMaster" as "Wire" does not support repeated start...
#define SDA_PIN 4  //A4         //References:
#define SDA_PORT PORTC    //  http://playground.arduino.cc/Main/SoftwareI2CLibrary

#include <SoftI2CMaster.h>  //  https://github.com/felias-fogg/SoftI2CMaster
#define I2C_TIMEOUT 1000  // Sets Clock Stretching up to 1sec
#define I2C_FASTMODE 1    // Sets 400kHz operating speed

// ----- Globals -----

//ADC Globals - Analog ALS, Temp, UV
int ADCpin_AnalogTemp = A2;
int ADCpin_AnalogUV = A0;
int sensorValue = 0;
float sensorConvert = 0;

//Digital Input Globals - Hall Sensor
int Hall_Out0 = 0;
int Hall_Out1 = 1;

//I2C globals (using SoftI2CMaster libary) - MEMs Kionix Sensor 
int I2C_check = 0;
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
int RPR0521_ALS_D0_RAWOUT = 0;
int RPR0521_ALS_D1_RAWOUT = 0;
float RPR0521_ALS_DataRatio = 0;
float RPR0521_ALS_OUT = 0;
#endif

#ifdef Color
int BH1745_DeviceAddress = 0x72;  //this is the 8bit address, 7bit address = 0x4E
int BH1745_RED_LB = 0;
int BH1745_RED_HB = 0;
int BH1745_GRN_LB = 0;
int BH1745_GRN_HB = 0;
int BH1745_BLU_LB = 0;
int BH1745_BLU_HB = 0;

int BH1745_RED_OUT = 0;
int BH1745_GRN_OUT = 0;
int BH1745_BLU_OUT = 0;
#endif

#ifdef KX022
int KX022_DeviceAddress = 0x3C;  //this is the 8bit address, 7bit address = 0x1E
int KX022_Accel_X_LB = 0;
int KX022_Accel_X_HB = 0;
int KX022_Accel_Y_LB = 0;
int KX022_Accel_Y_HB = 0;
int KX022_Accel_Z_LB = 0;
int KX022_Accel_Z_HB = 0;
int KX022_Accel_X_RawOUT = 0;
int KX022_Accel_Y_RawOUT = 0;
int KX022_Accel_Z_RawOUT = 0;
float KX022_Accel_X_OUT = 0;
float KX022_Accel_Y_OUT = 0;
float KX022_Accel_Z_OUT = 0;
#endif

void setup()
{
  //Wire.begin();        // start I2C functionality
  Serial.begin(9600);  // start serial port at 9600 bps
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
 
 //----- Start Initialization for KMX061 Digital Accel/Mag Sensor -----
 #ifdef KMX62
 //KMX61 Init Sequence
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
 //----- END Initialization for KMX061 Digital Accel/Mag Sensor -----

 //----- Start Initialization for BM1383 Digital Pressure Sensor -----
 #ifdef Pressure
  //BM1382GLV Init Sequence
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
  // 1. Mode Control (0x41), write (0xC6): ALS EN, PS EN, 100ms measurement for ALS and PS
  
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
  //----- END Initialization for BH175 Color Sensor -----
  
  //----- Start Initialization for KX022 Color Sensor -----  
  #ifdef KX022
  //1. CNTL1 (0x18) loaded with 0x41
  //2. ODCNTL (0x1B) loaded with 0x02
  //3. CNTL3 (0x1A) loaded with 0xD8
  //4. TILT_TIMER (0x22) loaded with 0x01
  
  i2c_start(KX022_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x18);
  i2c_write(0x41);
  i2c_stop();
  
  i2c_start(KX022_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x1B);
  i2c_write(0x02);
  i2c_stop();
  
  i2c_start(KX022_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x1A);
  i2c_write(0xD8);
  i2c_stop();

  i2c_start(KX022_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x22);
  i2c_write(0x01);
  i2c_stop();
  
  i2c_start(KX022_DeviceAddress);  //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(0x18);
  i2c_write(0xC1);
  i2c_stop();

  #endif

  //----- END Initialization for KX022 Color Sensor -----  
}

void loop()
{
  
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(250);              // wait for 250ms
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(250);              // wait for 250ms

  //---------- Start Code for Reading BDE0600G Analog Temperature Sensor ----------
  #ifdef AnalogTemp

  //----- Start ADC Read from Port A1 ----
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
  Serial.write("BDE0600G Temp = ");
  Serial.print(sensorConvert);
  Serial.write(" degC");
  Serial.write(0x0A); //Print Line Feed
  Serial.write(0x0D); //Print Carrage Return
  #endif
  //---------- End Code for Reading BDE0600G Analog Temperature Sensor ----------
  
  //---------- Start Code for Reading ML8511 Analog UV Sensor ----------
  #ifdef AnalogUV

  //----- Start ADC Read from Port A2 ----
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
  Serial.write("ML8511 UV Sensor = ");
  Serial.print(sensorConvert);
  Serial.write(" mW/cm2");
  Serial.write(0x0A); //Print Line Feed
  Serial.write(0x0D); //Print Carrage Return
  #endif
  //---------- End Code for Reading ML8511 Analog UV Sensor ----------
  
  //---------- Start Code for Reading BU52011HFV Hall Sensor ----------
  #ifdef HallSen
  //Hardware Connection
  //For the Hall Sensor, we only need to monitor digital inputs for OUT1 and OUT2 Pins of the Hall Sensor
  //Connect Pin1 of Hall Sensor Header U111 to Arduino Pin7
  //Connect Pin2 of Hall Sensor Header U111 to Arduino Pin8
  Hall_Out0 = digitalRead(2);
  Serial.write("BU52011HFV South Detect = ");
  Serial.print(Hall_Out0);
  Serial.write(0x0A); //Print Line Feed
  Serial.write(0x0D); //Print Carrage Return
  
  Hall_Out1 = digitalRead(3);
  Serial.write("BU52011HFV North Detect = ");
  Serial.print(Hall_Out1);
  Serial.write(0x0A); //Print Line Feed
  Serial.write(0x0D); //Print Carrage Return
  #endif
  //---------- End Code for Reading BU52011HFV Hall Sensor ----------
  
  //---------- Start Code for Reading KMX62 Kionix Accelerometer+Mag Sensor ----------  
  #ifdef KMX62
  // -- Notes on Arduino I2C Connection --
  //I2C is built in using using the "wire" library
  //Uno, Ethernet	A4 (SDA), A5 (SCL) [UNO - THIS is the platform we are using]
  //Mega2560	        20 (SDA), 21 (SCL)
  //Leonardo	        2  (SDA), 3  (SCL)
  //Due	                20 (SDA), 21 (SCL), SDA1, SCL1
  
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
  
  //Note: Conversion to G is as follows:
  //      Axis_ValueInG = MEMS_Accel_axis / 1024
  //      However, since we did not remove the LSB previously, we need to divide by 4 again
  //      Thus, we will divide the output by 4095 (1024*4) to convert and cancel out the LSB
  MEMS_Accel_Conv_Xout = (float)MEMS_Accel_Xout/4096/2;
  MEMS_Accel_Conv_Yout = (float)MEMS_Accel_Yout/4096/2;
  MEMS_Accel_Conv_Zout = (float)MEMS_Accel_Zout/4096/2;
  
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

  Serial.write("KMX62 Accel Xout = ");
  Serial.print(MEMS_Accel_Conv_Xout);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KMX62 Accel Yout = ");
  Serial.print(MEMS_Accel_Conv_Yout);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return  
  Serial.write("KMX62 Accel Zout = ");
  Serial.print(MEMS_Accel_Conv_Zout);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return  
  
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
  
  //Note: The highbyte and low byte return a 14bit value, dropping the two LSB in the Low byte.
  //      However, because we need the signed value, we will adjust the value when converting to "g"
  MEMS_Mag_Xout = (MEMS_Mag_Xout_highByte<<8) | (MEMS_Mag_Xout_lowByte);
  MEMS_Mag_Yout = (MEMS_Mag_Yout_highByte<<8) | (MEMS_Mag_Yout_lowByte);
  MEMS_Mag_Zout = (MEMS_Mag_Zout_highByte<<8) | (MEMS_Mag_Zout_lowByte);
  
  //Note: Conversion to G is as follows:
  //      Axis_ValueInG = MEMS_Accel_axis / 1024
  //      However, since we did not remove the LSB previously, we need to divide by 4 again
  //      Thus, we will divide the output by 4095 (1024*4) to convert and cancel out the LSB
  MEMS_Mag_Conv_Xout = (float)MEMS_Mag_Xout*0.146;
  MEMS_Mag_Conv_Yout = (float)MEMS_Mag_Yout*0.146;
  MEMS_Mag_Conv_Zout = (float)MEMS_Mag_Zout*0.146;
  
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
  */

  Serial.write("KMX62 Mag Xout = ");
  Serial.print(MEMS_Mag_Conv_Xout);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KMX62 Mag Yout = ");
  Serial.print(MEMS_Mag_Conv_Yout);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return  
  Serial.write("KMX62 Mag Zout = ");
  Serial.print(MEMS_Mag_Conv_Zout);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return  
  
  #endif
  //---------- END Code for Reading KMX62 Kionix Accelerometer+Mag Sensor ----------    
 
  //---------- Start Code for Reading BM1383 Pressure Sensor ----------  
  #ifdef Pressure
  
  // Start Getting Data from Pressure Sensor
  i2c_start(BM1383_DeviceAddress);
  i2c_write(0x1A);
  i2c_rep_start(BM1383_DeviceAddress | 1);  // Or-ed with "1" for read bit
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
 
  Serial.write("BM1383 (Temp) = ");
  Serial.print(BM1383_Temp_Conv_Out);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("BM1383 (Pres) = ");
  Serial.print(BM1383_Pres_Conv_Out);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  
   #endif
  //---------- END Code for Reading KMX62 BM1383 Pressure Sensor ----------    

   //----- Start Reading for RPR-0521 ALS/PROX Sensor -----
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
  Serial.write("RPR-0521 (Prox) = ");
  Serial.print(RPR0521_PS_RAWOUT);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("RPR-0521 (ALS)= ");
  Serial.print(RPR0521_ALS_OUT);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  
  #endif  
  //----- END Reading for RPR-0521 ALS/PROX Sensor -----

  //----- START Reading for Color Sensor -----
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
  
  Serial.write("BH1745 (Red) = ");
  Serial.print(BH1745_RED_OUT);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("BH1745 (Green) = ");
  Serial.print(BH1745_GRN_OUT);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("BH1745 (Blue) = ");
  Serial.print(BH1745_BLU_OUT);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  
  #endif
  //----- END Reading for Color Sensor -----

#ifdef KX022
  i2c_start(KX022_DeviceAddress);
  i2c_write(0x06);
  i2c_rep_start(KX022_DeviceAddress | 1);  // Or-ed with "1" for read bit
  KX022_Accel_X_LB = i2c_read(false);
  KX022_Accel_X_HB = i2c_read(false);
  KX022_Accel_Y_LB = i2c_read(false);
  KX022_Accel_Y_HB = i2c_read(false);
  KX022_Accel_Z_LB = i2c_read(false);
  KX022_Accel_Z_HB = i2c_read(true);
  i2c_stop();

  KX022_Accel_X_RawOUT = (KX022_Accel_X_HB<<8) | (KX022_Accel_X_LB);
  KX022_Accel_Y_RawOUT = (KX022_Accel_Y_HB<<8) | (KX022_Accel_Y_LB);
  KX022_Accel_Z_RawOUT = (KX022_Accel_Z_HB<<8) | (KX022_Accel_Z_LB);

  KX022_Accel_X_OUT = (float)KX022_Accel_X_RawOUT / 16384;
  KX022_Accel_Y_OUT = (float)KX022_Accel_Y_RawOUT / 16384;
  KX022_Accel_Z_OUT = (float)KX022_Accel_Z_RawOUT / 16384;
  
  Serial.write("KX122 (X) = ");
  Serial.print(KX022_Accel_X_OUT);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KX122 (Y) = ");
  Serial.print(KX022_Accel_Y_OUT);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return
  Serial.write("KX122 (Z) = ");
  Serial.print(KX022_Accel_Z_OUT);
  Serial.write(0x0A);  //Print Line Feed
  Serial.write(0x0D);  //Print Carrage Return

#endif



  Serial.write(0x0A); //Print Line Feed
  Serial.write(0x0D); //Print Carrage Return
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
