/*
 * GENERATED FILE - DO NOT EDIT
 * (c) Code Red Technologies Ltd, 2008-2015
 * (c) NXP Semiconductors 2013-2015
 * Linker script memory definitions
 * Created from LinkMemoryTemplate
 * By LPCXpresso v7.8.0 [Build 426] [2015-05-28]  on Mon Jun 08 15:03:46 PDT 2015)
*/

MEMORY
{
  /* Define each memory region */
  MFlash512 (rx) : ORIGIN = 0x0, LENGTH = 0x80000 /* 512K bytes */
  Ram0_64 (rwx) : ORIGIN = 0x2000000, LENGTH = 0x10000 /* 64K bytes */
  Ram1_32 (rwx) : ORIGIN = 0x2010000, LENGTH = 0x8000 /* 32K bytes */
  Ram2_8 (rwx) : ORIGIN = 0x3400000, LENGTH = 0x2000 /* 8K bytes */


}
  /* Define a symbol for the top of each memory region */
  __top_MFlash512 = 0x0 + 0x80000;
  __top_Ram0_64 = 0x2000000 + 0x10000;
  __top_Ram1_32 = 0x2010000 + 0x8000;
  __top_Ram2_8 = 0x3400000 + 0x2000;



