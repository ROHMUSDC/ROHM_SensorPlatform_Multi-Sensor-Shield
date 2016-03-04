# ROHM Sensor Shield Public Repository
* Applicable Part Number: ROHM_SENSORSHLD0-EVK-101, ROHM_SENSORSHLD1-EVK-101
* Description:  A single board containing all ROHM Sensors with a common platform pin out (standard Arduino header)
* Developer: USDC Applications Engineering Team, ROHM Semiconductor
* Date Started: March 2015
* Related Projects: https://github.com/ROHMUSDC/ROHMSensorPlatformEVK

----
### ROHM Sensor Shield Included Sensors
* ROHM_SENSORSHLD1-EVK-101
  * Core Sensors:
    * ROHM BDE0600G – Analog Temperature Sensor
    * ROHM BM1383AGLV – Digital Barometric Pressure Sensor
    * ROHM BU52014HFV – Hall Switch Sensor (Omnipolar with Polarity Discrimination)
    * ROHM BM1422GMV – Magnetometer Sensor
    * KIONIX KX122 – Digital Accelerometer
    * KIONIX KMX62 – Digital Magnetometer and Accelerometer
    * KIONIX KXG03 – Digital Gyroscope and Accelerometer
    * LAPIS ML8511A – Analog UV Sensor
    * LAPIS ML8523 – Digital UV Sensor with UV-A and UV-B Filtering
    * ROHM RPR-0521 – Digital Ambient Light Sensor and Proximity Sensor
    * ROHM BH1745 – Digital Color Sensor
  * Special Functions:
    * KNOWLES SPM0423HD4H-WB – Digital Microphone
      * Primarily for use with NXP MCU Lineup
    * KIONIX KX123-1039, KX124-1050 – Accelerometer
      * For four corner Accelerometer algorithm development
      * Difference between 1039 and 1050 is the I2C register address scheme to control all 4 accelerometers using a single I2C master

----
### Repository Contents
* "Documentation" Folder
  * "Sensor Shield General Datasheet" Folder - Contains the General Datasheet for this Shield Board
  * "Sensor Datasheets" Folder - Contains Datasheets for sensors on the Shield Board
* "HW Source Files" Folder 
  * "SENSORSHLD0-EVK-101 HW Design Files" Folder - Contains Schematic, BOM, Layout, Gerber Files for this revision of the Shield Board
  * "SENSORSHLD1-EVK-101 HW Design Files" Folder - Contains Schematic, BOM, Layout, Gerber Files for this revision of the shield Board
* "Platform Code" Folder
  * "Arduino_UNO_FirmwareExample" Folder - Provides documentation and example code for Arduino UNO Platform
  * "LAPIS_SensorPlatform_Firmware" Folder - Provides example code for ROHM's Sensor Platform Kit
  * "MultiTech_Dragonfly_FirmwareExample" Folder - Provides documentation and example code for MultiTech's Dragonfly
  * "Nordic_nRF51-DK_FirmwareExample" Folder - Provides documentation and example code for Nordic nRF51-DK
  * "NXP_LPCXpresso54102_BlinkyExample" Folder - Provides example code for NXP's LPCXpresso54102

----
### Current Supported Platforms
* Arduino UNO, Firmware Example and Documentation (This is our recommended starting point)
* LPCXpresso54102, Firmware Example Only
* Nordic Semiconductor nRF51-DK, Firmware Example and Documentation 
	*  Recommended for BTLE Low Power Sensor Applications
    *  MBED Repository Page: https://developer.mbed.org/teams/ROHMUSDC/code/Nordic_UART_TEMPLATE_ROHM/
* ROHM Sensor Platform Kit: Firmware Example  Only
	*  ROHM Sensor Platform Repository: https://github.com/ROHMUSDC/ROHMSensorPlatformEVK
* MultiTech Dragonfly and MTUDK2, Firmware Example and Documentation
	*  Recommended for Cellular Network Compatibility Applications
    *  MBED Repository Page: https://developer.mbed.org/teams/ROHMUSDC/code/MultiTech_DragonFly_ROHMSensorShield_Sam/

----
### FAQ
* Question:
	* What is the difference between ROHM_SENSORSHLD0-EVK-101 and ROHM_SENSORSHLD1-EVK-101?
* Answer:
	* Removed Erroneous Jumpers
	* Removed J5 to J11 and adjusted routing for J1 to J4
	* Added ROHM BM1422GMV Magnetometer, 1.8V level shifter, and 1.8V LDO (for Magnetometer usage)
	* Changed pressure sensor from BM1383GLV to BM1383AGLV.  (No HW change, but new PN has new FW I2C register mapping)
	* PN Change from ML8511 to ML8511A (only chip labelling change)
	* Kionix Corner Accelerometer: one PN, KX123-1050, has been adjusted to the KX124-1050
	
----
### DISCLAIMER
This Technical Data is protected under copyright laws.

ROHM hereby grants you a nonexclusive, nontransferable license to use this Technical Data 
as long as you abide by the terms and conditions of this DISCLAIMER. 

However, you are not authorized to sell, loan, rent, lease, redistribute or license this Technical Data, 
in whole or in part, or in modified form, to anyone.

You may modify this Technical Data to suit your specific applications, 
but the rights to derivative works and said modifications shall belong to ROHM. 

You may make copies of this Technical Data as necessary for internal use only within your company.

ROHM shall not be responsible for ensuring proper application on all computer systems.
This Technical Data is provided AS IS. ROHM makes no guarantees, either implicitly or explicitly, 
regarding its usability, functionality, accuracy, merchantability, or fitness for a specific purpose.

In addition, you shall take full responsibility for the use of any information acquired from this Technical Data. 

ROHM does not assume warranty of any kind arising from the use of this Technical Data. ACCORDINGLY, 
IN NO EVENT SHALL ROHM BE RESPONSIBLE, WHETHER THROUGH CONTRACT OR TORT LIABILITY, 
FOR ANY DAMAGES (INCLUDING BUT NOT LIMITED TO LOST PROFITS OR OTHER INCIDENTAL, CONSEQUENTAL, 
OR PUNITIVE DAMAGES) ARISING OUT OF OR IN CONNECTION WITH THE USE OR APPLICATION OF THIS Technical Data.

Furthermore, this Technical Data is subject to change without notice.

Unless otherwise expressly provided herein, ROHM does not convey any license under patent rights 
or any other intellectual property rights (including those of third parties).

ROHM is not obligated to provide maintenance or support for the Technical Data.