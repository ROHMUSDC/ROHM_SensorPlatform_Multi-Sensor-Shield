/* 
Sample Program Description: 
	This Program will enable to Multi-Tech Dragonfly platform to utilize ROHM's Multi-sensor Shield Board.
    This program will initialize all sensors on the shield and then read back the sensor data.
    Data will then be output to the UART Debug Terminal every 1 second.

Sample Program Author: 
	ROHM USDC

Additional Resources:
    ROHM Sensor Shield GitHub Repository: https://github.com/ROHMUSDC/ROHM_SensorPlatform_Multi-Sensor-Shield
*/
#include "mbed.h"

//Macros for checking each of the different Sensor Devices
#define AnalogTemp  //BDE0600
#define AnalogUV    //ML8511
#define HallSensor  //BU52011
#define RPR0521     //RPR0521
#define KMX62       //KMX61, Accel/Mag         
#define COLOR       //BH1745
#define KX022       //KX022, Accel Only
#define Pressure    //BM1383

//Define Pins for I2C Interface
I2C i2c(I2C_SDA, I2C_SCL);
bool        RepStart = true;
bool        NoRepStart = false;

//Define Sensor Variables
#ifdef AnalogTemp
AnalogIn    BDE0600_Temp(PC_4); //Mapped to A2
uint16_t    BDE0600_Temp_value;
float       BDE0600_output;
#endif

#ifdef AnalogUV
AnalogIn    ML8511_UV(PC_1);    //Mapped to A4
uint16_t    ML8511_UV_value;
float       ML8511_output;
#endif

#ifdef HallSensor
DigitalIn   Hall_GPIO0(PC_8);
DigitalIn   Hall_GPIO1(PB_5);
int         Hall_Return1;
int         Hall_Return0;
#endif

#ifdef RPR0521
int         RPR0521_addr_w = 0x70;          //7bit addr = 0x38, with write bit 0
int         RPR0521_addr_r = 0x71;          //7bit addr = 0x38, with read bit 1
char        RPR0521_ModeControl[2] = {0x41, 0xE6};  
char        RPR0521_ALSPSControl[2] = {0x42, 0x03};
char        RPR0521_Persist[2] = {0x43, 0x20};
char        RPR0521_Addr_ReadData = 0x44;
char        RPR0521_Content_ReadData[6];
int         RPR0521_PS_RAWOUT = 0;
float       RPR0521_PS_OUT = 0;
int         RPR0521_ALS_D0_RAWOUT = 0;
int         RPR0521_ALS_D1_RAWOUT = 0;
float       RPR0521_ALS_DataRatio = 0;
float       RPR0521_ALS_OUT = 0;
#endif

#ifdef KMX62
int         KMX62_addr_w = 0x1C;          //7bit addr = 0x38, with write bit 0
int         KMX62_addr_r = 0x1D;          //7bit addr = 0x38, with read bit 1
char        KMX62_CNTL2[2] = {0x3A, 0x5F};
char        KMX62_Addr_Accel_ReadData = 0x0A;
char        KMX62_Content_Accel_ReadData[6];
char        KMX62_Addr_Mag_ReadData = 0x10;
char        KMX62_Content_Mag_ReadData[6];
short int   MEMS_Accel_Xout = 0;
short int   MEMS_Accel_Yout = 0;
short int   MEMS_Accel_Zout = 0;
double      MEMS_Accel_Conv_Xout = 0;
double      MEMS_Accel_Conv_Yout = 0;
double      MEMS_Accel_Conv_Zout = 0;
short int   MEMS_Mag_Xout = 0;
short int   MEMS_Mag_Yout = 0;
short int   MEMS_Mag_Zout = 0;
float       MEMS_Mag_Conv_Xout = 0;
float       MEMS_Mag_Conv_Yout = 0;
float       MEMS_Mag_Conv_Zout = 0;
#endif

#ifdef COLOR
int         BH1745_addr_w = 0x72;   //write 
int         BH1745_addr_r = 0x73;   //read 
char        BH1745_persistence[2] = {0x61, 0x03};
char        BH1745_mode1[2] = {0x41, 0x00};
char        BH1745_mode2[2] = {0x42, 0x92};
char        BH1745_mode3[2] = {0x43, 0x02};
char        BH1745_Content_ReadData[6];
char        BH1745_Addr_color_ReadData = 0x50;
int         BH1745_Red;
int         BH1745_Blue;
int         BH1745_Green;
#endif

#ifdef KX022
int         KX022_addr_w = 0x3C;   //write 
int         KX022_addr_r = 0x3D;   //read 
char        KX022_Accel_CNTL1[2] = {0x18, 0x41};
char        KX022_Accel_ODCNTL[2] = {0x1B, 0x02};
char        KX022_Accel_CNTL3[2] = {0x1A, 0xD8};
char        KX022_Accel_TILT_TIMER[2] = {0x22, 0x01};
char        KX022_Accel_CNTL2[2] = {0x18, 0xC1};
char        KX022_Content_ReadData[6];
char        KX022_Addr_Accel_ReadData = 0x06;           
float       KX022_Accel_X;
float       KX022_Accel_Y;                               
float       KX022_Accel_Z;
short int   KX022_Accel_X_RawOUT = 0;
short int   KX022_Accel_Y_RawOUT = 0;
short int   KX022_Accel_Z_RawOUT = 0;
int         KX022_Accel_X_LB = 0;
int         KX022_Accel_X_HB = 0;
int         KX022_Accel_Y_LB = 0;
int         KX022_Accel_Y_HB = 0;
int         KX022_Accel_Z_LB = 0;
int         KX022_Accel_Z_HB = 0;
#endif

#ifdef Pressure
int         Press_addr_w = 0xBA;   //write 
int         Press_addr_r = 0xBB;   //read 
char        PWR_DOWN[2] = {0x12, 0x01};
char        SLEEP[2] = {0x13, 0x01};
char        Mode_Control[2] = {0x14, 0xC4};
char        Press_Content_ReadData[6];
char        Press_Addr_ReadData =0x1A;
int         BM1383_Temp_highByte;
int         BM1383_Temp_lowByte;
int         BM1383_Pres_highByte;
int         BM1383_Pres_lowByte;
int         BM1383_Pres_leastByte; 
short int   BM1383_Temp_Out;
float       BM1383_Temp_Conv_Out;
float       BM1383_Pres_Conv_Out;
float       BM1383_Var;
float       BM1383_Deci;
#endif

int main() {
    
//Initialization Section Start! **********************************************************
    
    //Initialize I2C Devices **********************************************************
    
#ifdef RPR0521
    i2c.write(RPR0521_addr_w, &RPR0521_ModeControl[0], 2, false);
    i2c.write(RPR0521_addr_w, &RPR0521_ALSPSControl[0], 2, false);
    i2c.write(RPR0521_addr_w, &RPR0521_Persist[0], 2, false);
#endif

#ifdef KMX62
    i2c.write(KMX62_addr_w, &KMX62_CNTL2[0], 2, false);
#endif

#ifdef COLOR
    i2c.write(BH1745_addr_w, &BH1745_persistence[0], 2, false);
    i2c.write(BH1745_addr_w, &BH1745_mode1[0], 2, false);
    i2c.write(BH1745_addr_w, &BH1745_mode2[0], 2, false);
    i2c.write(BH1745_addr_w, &BH1745_mode3[0], 2, false);
#endif

#ifdef KX022
    i2c.write(KX022_addr_w, &KX022_Accel_CNTL1[0], 2, false);
    i2c.write(KX022_addr_w, &KX022_Accel_ODCNTL[0], 2, false);
    i2c.write(KX022_addr_w, &KX022_Accel_CNTL3[0], 2, false);
    i2c.write(KX022_addr_w, &KX022_Accel_TILT_TIMER[0], 2, false);
    i2c.write(KX022_addr_w, &KX022_Accel_CNTL2[0], 2, false);
#endif

#ifdef Pressure
    i2c.write(Press_addr_w, &PWR_DOWN[0], 2, false);
    i2c.write(Press_addr_w, &SLEEP[0], 2, false);
    i2c.write(Press_addr_w, &Mode_Control[0], 2, false);
#endif
//End Initialization Section **********************************************************

//Begin Main Loop **********************************************************
    while (1) {
        
        // ADC Routines *******************************************************
        #ifdef AnalogTemp
        BDE0600_Temp_value = BDE0600_Temp.read_u16();
        
        BDE0600_output = (float)BDE0600_Temp_value * (float)0.000050354; //(value * (3.3V/65535))
        BDE0600_output = (BDE0600_output-(float)1.753)/((float)-0.01068) + (float)30;
        
        printf("BDE0600 Analog Temp Sensor Data:\r\n");
        printf(" Temp = %.2f C\r\n", BDE0600_output);
        #endif

        #ifdef AnalogUV
        ML8511_UV_value = ML8511_UV.read_u16();
        ML8511_output = (float)ML8511_UV_value * (float)0.000050354; //(value * (3.3V/65535))   //Note to self: when playing with this, a negative value is seen... Honestly, I think this has to do with my ADC converstion...
        ML8511_output = (ML8511_output-(float)2.2)/((float)0.129) + 10;                           // Added +5 to the offset so when inside (aka, no UV, readings show 0)... this is the wrong approach... and the readings don't make sense... Fix this.
        
        printf("ML8511 Analog UV Sensor Data:\r\n");
        printf(" UV = %.1f mW/cm2\r\n", ML8511_output);
        #endif

        // GPI Routines *******************************************************
        #ifdef HallSensor
        Hall_Return0 = Hall_GPIO0;
        Hall_Return1 = Hall_GPIO1;
        
        printf("BU52011 Hall Switch Sensor Data:\r\n");
        printf(" South Detect = %d\r\n", Hall_Return0);
        printf(" North Detect = %d\r\n", Hall_Return1);     
        #endif
        
        // I2C Routines *******************************************************
        #ifdef COLOR
        //Read color data from the IC
        i2c.write(BH1745_addr_w, &BH1745_Addr_color_ReadData, 1, RepStart);
        i2c.read(BH1745_addr_r, &BH1745_Content_ReadData[0], 6, NoRepStart);
        
        //separate all data read into colors
        BH1745_Red = (BH1745_Content_ReadData[1]<<8) | (BH1745_Content_ReadData[0]);
        BH1745_Green = (BH1745_Content_ReadData[3]<<8) | (BH1745_Content_ReadData[2]);
        BH1745_Blue = (BH1745_Content_ReadData[5]<<8) | (BH1745_Content_ReadData[4]);
        
        //Output Data into UART 
        printf("BH1745 COLOR Sensor Data:\r\n");
        printf(" Red   = %d ADC Counts\r\n",BH1745_Red);
        printf(" Green = %d ADC Counts\r\n",BH1745_Green);
        printf(" Blue  = %d ADC Counts\r\n",BH1745_Blue);
        #endif
        
        #ifdef RPR0521       //als digital
        i2c.write(RPR0521_addr_w, &RPR0521_Addr_ReadData, 1, RepStart);
        i2c.read(RPR0521_addr_r, &RPR0521_Content_ReadData[0], 6, NoRepStart);
        
        RPR0521_PS_RAWOUT = (RPR0521_Content_ReadData[1]<<8) | (RPR0521_Content_ReadData[0]);
        RPR0521_ALS_D0_RAWOUT = (RPR0521_Content_ReadData[3]<<8) | (RPR0521_Content_ReadData[2]);
        RPR0521_ALS_D1_RAWOUT = (RPR0521_Content_ReadData[5]<<8) | (RPR0521_Content_ReadData[4]);
        RPR0521_ALS_DataRatio = (float)RPR0521_ALS_D1_RAWOUT / (float)RPR0521_ALS_D0_RAWOUT;
         
        if(RPR0521_ALS_DataRatio < (float)0.595){
            RPR0521_ALS_OUT = ((float)1.682*(float)RPR0521_ALS_D0_RAWOUT - (float)1.877*(float)RPR0521_ALS_D1_RAWOUT);
        }
        else if(RPR0521_ALS_DataRatio < (float)1.015){
            RPR0521_ALS_OUT = ((float)0.644*(float)RPR0521_ALS_D0_RAWOUT - (float)0.132*(float)RPR0521_ALS_D1_RAWOUT);
        }
        else if(RPR0521_ALS_DataRatio < (float)1.352){
            RPR0521_ALS_OUT = ((float)0.756*(float)RPR0521_ALS_D0_RAWOUT - (float)0.243*(float)RPR0521_ALS_D1_RAWOUT);
        }
        else if(RPR0521_ALS_DataRatio < (float)3.053){
            RPR0521_ALS_OUT = ((float)0.766*(float)RPR0521_ALS_D0_RAWOUT - (float)0.25*(float)RPR0521_ALS_D1_RAWOUT);
        }
        else{
            RPR0521_ALS_OUT = 0;
        }
        printf("RPR-0521 ALS/PROX Sensor Data:\r\n");
        printf(" ALS = %0.2f lx\r\n", RPR0521_ALS_OUT);
        printf(" PROX= %u ADC Counts\r\n", RPR0521_PS_RAWOUT);
        #endif
        
        #ifdef KMX62
        //Read Accel Portion from the IC
        i2c.write(KMX62_addr_w, &KMX62_Addr_Accel_ReadData, 1, RepStart);
        i2c.read(KMX62_addr_r, &KMX62_Content_Accel_ReadData[0], 6, NoRepStart);

        //Note: The highbyte and low byte return a 14bit value, dropping the two LSB in the Low byte.
        //      However, because we need the signed value, we will adjust the value when converting to "g"
        MEMS_Accel_Xout = (KMX62_Content_Accel_ReadData[1]<<8) | (KMX62_Content_Accel_ReadData[0]);
        MEMS_Accel_Yout = (KMX62_Content_Accel_ReadData[3]<<8) | (KMX62_Content_Accel_ReadData[2]);
        MEMS_Accel_Zout = (KMX62_Content_Accel_ReadData[5]<<8) | (KMX62_Content_Accel_ReadData[4]);
          
        //Note: Conversion to G is as follows:
        //      Axis_ValueInG = MEMS_Accel_axis / 1024
        //      However, since we did not remove the LSB previously, we need to divide by 4 again
        //      Thus, we will divide the output by 4096 (1024*4) to convert and cancel out the LSB
        MEMS_Accel_Conv_Xout = ((float)MEMS_Accel_Xout/4096/2);
        MEMS_Accel_Conv_Yout = ((float)MEMS_Accel_Yout/4096/2);
        MEMS_Accel_Conv_Zout = ((float)MEMS_Accel_Zout/4096/2);

        //Read Mag portion from the IC
        i2c.write(KMX62_addr_w, &KMX62_Addr_Mag_ReadData, 1, RepStart);
        i2c.read(KMX62_addr_r, &KMX62_Content_Mag_ReadData[0], 6, NoRepStart);

        //Note: The highbyte and low byte return a 14bit value, dropping the two LSB in the Low byte.
        //      However, because we need the signed value, we will adjust the value when converting to "g"
        MEMS_Mag_Xout = (KMX62_Content_Mag_ReadData[1]<<8) | (KMX62_Content_Mag_ReadData[0]);
        MEMS_Mag_Yout = (KMX62_Content_Mag_ReadData[3]<<8) | (KMX62_Content_Mag_ReadData[2]);
        MEMS_Mag_Zout = (KMX62_Content_Mag_ReadData[5]<<8) | (KMX62_Content_Mag_ReadData[4]);
        
        //Note: Conversion to G is as follows:
        //      Axis_ValueInG = MEMS_Accel_axis / 1024
        //      However, since we did not remove the LSB previously, we need to divide by 4 again
        //      Thus, we will divide the output by 4095 (1024*4) to convert and cancel out the LSB
        MEMS_Mag_Conv_Xout = (float)MEMS_Mag_Xout/4096*(float)0.146;
        MEMS_Mag_Conv_Yout = (float)MEMS_Mag_Yout/4096*(float)0.146;
        MEMS_Mag_Conv_Zout = (float)MEMS_Mag_Zout/4096*(float)0.146; 

        // Return Data to UART
        printf("KMX62 Accel+Mag Sensor Data:\r\n");
        printf(" AccX= %0.2f g\r\n", MEMS_Accel_Conv_Xout);
        printf(" AccY= %0.2f g\r\n", MEMS_Accel_Conv_Yout);
        printf(" AccZ= %0.2f g\r\n", MEMS_Accel_Conv_Zout);
        printf(" MagX= %0.2f g\r\n", MEMS_Mag_Conv_Xout);
        printf(" MagY= %0.2f g\r\n", MEMS_Mag_Conv_Yout);
        printf(" MagZ= %0.2f g\r\n", MEMS_Mag_Conv_Zout);
        #endif
     
        #ifdef KX022
        //Read KX022 Portion from the IC
        i2c.write(KX022_addr_w, &KX022_Addr_Accel_ReadData, 1, RepStart);
        i2c.read(KX022_addr_r, &KX022_Content_ReadData[0], 6, NoRepStart);
                    
        //Format Data
        KX022_Accel_X_RawOUT = (KX022_Content_ReadData[1]<<8) | (KX022_Content_ReadData[0]);
        KX022_Accel_Y_RawOUT = (KX022_Content_ReadData[3]<<8) | (KX022_Content_ReadData[2]);
        KX022_Accel_Z_RawOUT = (KX022_Content_ReadData[5]<<8) | (KX022_Content_ReadData[4]);       

        //Scale Data
        KX022_Accel_X = (float)KX022_Accel_X_RawOUT / 16384;
        KX022_Accel_Y = (float)KX022_Accel_Y_RawOUT / 16384;
        KX022_Accel_Z = (float)KX022_Accel_Z_RawOUT / 16384;
        
        //Return Data through UART
        printf("KX022 Accelerometer Sensor Data: \r\n");
        printf(" AccX= %0.2f g\r\n", KX022_Accel_X);
        printf(" AccY= %0.2f g\r\n", KX022_Accel_Y);
        printf(" AccZ= %0.2f g\r\n", KX022_Accel_Z);
        #endif
     
        #ifdef Pressure
        i2c.write(Press_addr_w, &Press_Addr_ReadData, 1, RepStart);
        i2c.read(Press_addr_r, &Press_Content_ReadData[0], 6, NoRepStart);
        
        BM1383_Temp_Out = (Press_Content_ReadData[0]<<8) | (Press_Content_ReadData[1]);
        BM1383_Temp_Conv_Out = (float)BM1383_Temp_Out/32;
        
        BM1383_Var  = (Press_Content_ReadData[2]<<3) | (Press_Content_ReadData[3] >> 5);
        BM1383_Deci = ((Press_Content_ReadData[3] & 0x1f) << 6 | ((Press_Content_ReadData[4] >> 2)));
        BM1383_Deci = (float)BM1383_Deci* (float)0.00048828125;  //0.00048828125 = 2^-11
        BM1383_Pres_Conv_Out = (BM1383_Var + BM1383_Deci);   //question pending here...
        
        printf("BM1383 Pressure Sensor Data:\r\n");
        printf(" Temperature= %0.2f C\r\n", BM1383_Temp_Conv_Out);
        printf(" Pressure   = %0.2f hPa\r\n", BM1383_Pres_Conv_Out);
        #endif
        //*********************************************************************

        printf("\r\n");
        wait(1);    //Wait before re-gathering sensor data
    }
}