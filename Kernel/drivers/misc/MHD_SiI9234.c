/*
 * MHD_SiI9234.c - Driver for Silicon Image MHD SiI9234 Transmitter driver
 *
 * Copyright 2010  Philju Lee (Daniel Lee)
 *
 * Based on preview driver from Silicon Image.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

static int MYDRV_MAJOR;

static int MHDDRV_MAJOR;

static bool tclkStable;
static bool mobileHdCableConnected;
static bool hdmiCableConnected;
static bool dsRxPoweredUp;
static byte tmdsPoweredUp;
static byte txPowerState;
static bool checkTclkStable;


/*=======================================================================================*/

void InitCBusRegs(void);
byte ReadIndexedRegister (byte PageNum, byte Offset);
bool delay_ms(int msec);
void MHD_HW_Reset(void);
void MHD_HW_Off(void);
void MHD_GPIO_INIT(void);

byte MHD_Bridge_detect(void);
void MHD_OUT_EN(void);
void MHD_INT_clear(void);

void I2C_WriteByte(byte deviceID, byte offset, byte value);
byte I2C_ReadByte(byte deviceID, byte offset);
byte ReadByteTPI(byte Offset);
void WriteByteTPI(byte Offset, byte Data);
void ReadModifyWriteTPI(byte Offset, byte Mask, byte Data); 
void WriteIndexedRegister (byte PageNum, byte Offset, byte Data);
static void sii9234_initializeStateVariables (void);

void sii9234_initial_registers_set(void);
void sii9234_enable_interrupts(void);
byte ReadByteCBUS(byte Offset);
void WriteByteCBUS(byte Offset, byte Data);
byte ReadIndexedRegister (byte PageNum, byte Offset);
void ReadModifyWriteIndexedRegister (byte PageNum, byte Offset, byte Mask, byte Data);
void TxPowerStateD3(void);
void DisableInterrupts(void);
void EnableTMDS(void);
static void TxPowerStateD0 (void); 
static void EnableInterrupts (void); 
void CheckTxFifoStable (void); 
void HotPlugService (void) ;
static void WakeUpFromD3 (void) ;
void ReadModifyWriteCBUS(byte Offset, byte Mask, byte Value) ;
static void OnMHDCableConnected (void) ;
void ForceUsbIdSwitchOpen (void) ;
static void ReleaseUsbIdSwitchOpen (void) ;
void DisableTMDS (void) ;
void OnDownstreamRxPoweredDown (void) ;
void OnHdmiCableDisconnected (void) ;
void OnDownstreamRxPoweredUp (void) ;
void OnHdmiCableConnected (void) ;

void sii9234_tpi_init(void);
void sii9234_register_init(void);
void sii9234_start_tpi(void);
void mhd_rx_connected(void);
void enable_mhd_tx(void);
void set_mhd_power_active_mode(void);
void mhd_tx_fifo_stable(void);
int MHD_Read_deviceID(void);

/*=======================================================================================*/


bool delay_ms(int msec)
{
  mdelay(msec);
  return 0;
}

void MHD_HW_Reset(void)
{
  printk("[SIMG]MHD_HW_Reset == Start == \n"); 
  SII9234_HW_Reset();
  printk("[SIMG] MHD_HW_Reset == End == \n");   
}

EXPORT_SYMBOL(MHD_HW_Reset);

void MHD_HW_Off(void)
{
	SII9234_HW_Off();
}
EXPORT_SYMBOL(MHD_HW_Off);

void MHD_GPIO_INIT(void)
{
	SII9234_GPIO_INIT();
}

EXPORT_SYMBOL(MHD_GPIO_INIT);

int MHD_HW_IsOn(void)
{
	return SII9234_HW_IsOn();
}
EXPORT_SYMBOL(MHD_HW_IsOn);


byte MHD_Bridge_detect(void)
{
  byte temp = 0;
	byte BridgeOn = 0;
	DisableInterrupts();
	msleep(180);
	if(!gpio_get_value(GPIO_ACCESSORY_INT)&& MHD_HW_IsOn())
	{
	temp = ReadIndexedRegister(INDEXED_PAGE_0, 0x09);
	if ((temp & RSEN) == 0x00) 
  {
   			BridgeOn = FALSE;
			//ReadModifyWriteTPI(0x79, SI_BIT_5 | SI_BIT_4, SI_BIT_4); //force HPD to 0
			ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x79, SI_BIT_5 | SI_BIT_4, SI_BIT_4);
		}	
		else
  		{	
  			BridgeOn = TRUE;
			//ReadModifyWriteTPI(0x79, BIT_5 | BIT_4, 0); //back to current state
		}	
		printk("[MHD] Bridge detect %x :: HPD %d\n",BridgeOn,gpio_get_value(GPIO_HDMI_HPD));
		//ReadModifyWriteTPI(0x79, SI_BIT_5 | SI_BIT_4, 0); //back to current state
		ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x79, SI_BIT_5 | SI_BIT_4, 0);
	}	
	MHD_INT_clear();
	EnableInterrupts();
	printk("[MHD]MHD_Bridge_detect -- \n");
	return BridgeOn; 
}
EXPORT_SYMBOL(MHD_Bridge_detect);


void MHD_OUT_EN(void)
{
	byte state , int_stat;
	int_stat = ReadIndexedRegister(INDEXED_PAGE_0,0x74);
	printk("[MHD]MHD_OUT_EN INT register value is: 0x%02x \n", int_stat);
	state = ReadIndexedRegister(INDEXED_PAGE_0, 0x81);
	printk("[MHD]MHD_OUT_EN register 0x81 value is: 0x%02x\n", state);
	
	if((state & 0x02) && (int_stat &0x01))
	{	
		printk("[MHD]MHD_OUT_EN :: enable output\n");		
 		ReadModifyWriteIndexedRegister(INDEXED_PAGE_0,0x80,SI_BIT_4,0x0);
		msleep(20);	
		ReadModifyWriteIndexedRegister(INDEXED_PAGE_0,0x80,SI_BIT_4,SI_BIT_4);
		msleep(60);
	
		set_mhd_power_active_mode();
 		mhd_tx_fifo_stable(); //fifo clear
	}
	MHD_INT_clear();
}

EXPORT_SYMBOL(MHD_OUT_EN);

void MHD_INT_clear(void)
{

	byte Int_state;
	
#if 0	
	Int_state= ReadByteTPI(TPI_INTERRUPT_STATUS_REG);
	
	WriteByteTPI (TPI_INTERRUPT_STATUS_REG, Int_state);	  // Clear this interrupt.	  




	WriteByteTPI(TPI_INTERRUPT_ENABLE_REG, 0x00);
	
	WriteIndexedRegister(INDEXED_PAGE_0, 0x78, 0x01);	// Enable 
#endif
	Int_state = ReadIndexedRegister(INDEXED_PAGE_0,0x74);
	WriteIndexedRegister(INDEXED_PAGE_0,0x74,Int_state);
}


EXPORT_SYMBOL(MHD_INT_clear);

void I2C_WriteByte(byte deviceID, byte offset, byte value)
{
	if(deviceID == 0x72)
		SII9234_i2c_write(SII9234_i2c_client,offset,value);
	else if(deviceID == 0x7A)
		SII9234_i2c_write(SII9234A_i2c_client,offset,value);
	else if(deviceID == 0x92)
		SII9234_i2c_write(SII9234B_i2c_client,offset,value);
	else if(deviceID == 0xC8)
		SII9234_i2c_write(SII9234C_i2c_client,offset,value);
	else
		printk("[MHL]I2C_WriteByte error %x\n",deviceID); 
}

byte I2C_ReadByte(byte deviceID, byte offset)
{
  byte number = 0;
  //printk("[MHL]I2C_ReadByte called ID%x Offset%x\n",deviceID,offset);
  	if(deviceID == 0x72)
		number = SII9234_i2c_read(SII9234_i2c_client,offset);
	else if(deviceID == 0x7A)
		number = SII9234_i2c_read(SII9234A_i2c_client,offset);
	else if(deviceID == 0x92)
		number = SII9234_i2c_read(SII9234B_i2c_client,offset);
	else if(deviceID == 0xC8)
		number = SII9234_i2c_read(SII9234C_i2c_client,offset);
	else
		printk("[MHL]I2C_ReadByte error %x\n",deviceID); 
	//printk("[MHL]I2C_ReadByte ID:%x Offset:%x data:%x\n",deviceID,offset,number); 
    return (number);
}

byte ReadByteTPI (byte Offset) 
{
	return I2C_ReadByte(TPI_SLAVE_ADDR, Offset);
}

void WriteByteTPI (byte Offset, byte Data) 
{
	I2C_WriteByte(TPI_SLAVE_ADDR, Offset, Data);
}

void ReadModifyWriteTPI(byte Offset, byte Mask, byte Data) 
{

	byte Temp;

	Temp = ReadByteTPI(Offset);		// Read the current value of the register.
	Temp &= ~Mask;					// Clear the bits that are set in Mask.
	Temp |= (Data & Mask);			// OR in new value. Apply Mask to Value for safety.
	WriteByteTPI(Offset, Temp);		// Write new value back to register.
}


void WriteIndexedRegister (byte PageNum, byte Offset, byte Data) 
{
	WriteByteTPI(TPI_INDEXED_PAGE_REG, PageNum);		// Indexed page
	WriteByteTPI(TPI_INDEXED_OFFSET_REG, Offset);		// Indexed register
	WriteByteTPI(TPI_INDEXED_VALUE_REG, Data);			// Write value
}


static void sii9234_initializeStateVariables (void) 
{

	tclkStable = FALSE;
	checkTclkStable = TRUE;
	tmdsPoweredUp = FALSE;
	mobileHdCableConnected = FALSE;
	hdmiCableConnected = FALSE;
	dsRxPoweredUp = FALSE;
}

void InitCBusRegs(void) 
{
	I2C_WriteByte(0xC8, 0x1F, 0x02); 			// Heartbeat Max Fail Enable
	I2C_WriteByte(0xC8, 0x07, DDC_XLTN_TIMEOUT_MAX_VAL | 0x06); 			// Increase DDC translation layer timer
	I2C_WriteByte(0xC8, 0x40, 0x03); 			// CBUS Drive Strength
	I2C_WriteByte(0xC8, 0x42, 0x06); 			// CBUS DDC interface ignore segment pointer
	I2C_WriteByte(0xC8, 0x36, 0x0C);
	//I2C_WriteByte(0xC8, 0x44, 0x02);
	I2C_WriteByte(0xC8, 0x3D, 0xFD);
	I2C_WriteByte(0xC8, 0x1C, 0x00);
	I2C_WriteByte(0xC8, 0x44, 0x00);
	I2C_WriteByte(0xC8, 0x09, 0x60);			// Enable PVC Xfer aborted / follower aborted

}

void sii9234_initial_registers_set(void)
{

	printk ("==[SIMG] sii9234_initial_registers_set Start ==\n");

#if 0 //old cable	
		// Power Up
		I2C_WriteByte(0x7A, 0x3D, 0x3F);			// Power up CVCC 1.2V core
		I2C_WriteByte(0x92, 0x11, 0x01);			// Enable TxPLL Clock
		I2C_WriteByte(0x92, 0x12, 0x15);			// Enable Tx Clock Path & Equalizer
		I2C_WriteByte(0x72, 0x08, 0x35);			// Power Up TMDS Tx Core	
	
	// Analog PLL Control
		I2C_WriteByte(0x92, 0x17, 0x03);			// PLL Calrefsel
		I2C_WriteByte(0x92, 0x1A, 0x20);			// VCO Cal
		I2C_WriteByte(0x92, 0x22, 0x8A);			// Auto EQ
		I2C_WriteByte(0x92, 0x23, 0x6A);			// Auto EQ
		I2C_WriteByte(0x92, 0x24, 0xAA);			// Auto EQ
		I2C_WriteByte(0x92, 0x25, 0xCA);			// Auto EQ
		I2C_WriteByte(0x92, 0x26, 0xEA);			// Auto EQ
		I2C_WriteByte(0x92, 0x4C, 0xA0);			// Manual zone control
		I2C_WriteByte(0x92, 0x4D, 0x00);			// PLL Mode Value
	
		I2C_WriteByte(0x72, 0x80, 0x14);			// Enable Rx PLL Clock Value	
		I2C_WriteByte(0x92, 0x45, 0x44);			// Rx PLL BW value from I2C
		I2C_WriteByte(0x92, 0x31, 0x0A);			// Rx PLL BW ~ 4MHz
		I2C_WriteByte(0x72, 0xA1, 0xFC);			// Disable internal Mobile HD driver	
		I2C_WriteByte(0x72, 0xA3, 0xFF);         //AMP
	  I2C_WriteByte(0x72, 0x2B, 0x01);			// Enable HDCP Compliance workaround  
	  I2C_WriteByte(0x72, 0x91, 0xE5);		// Skip RGND detection	
		I2C_WriteByte(0x72, 0xA5, 0x00);			// RGND Hysterisis.
	
	  
		I2C_WriteByte(0x72, 0x90, 0x27);			// Enable CBUS discovery
		//I2C_WriteByte(0x72, 0x05, ASR_VALUE);		// Enable Auto soft reset on SCDT = 0	
		I2C_WriteByte(0x72, 0x0D, 0x1C);			// HDMI Transcode mode enable
	
	  WriteByteTPI(TPI_ENABLE, 0x00);	
	
	  delay_ms(100); 
		WriteIndexedRegister(INDEXED_PAGE_0, 0xA0, 0x10);  
		delay_ms(100); 
	  ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_CONTROL_MASK, TMDS_OUTPUT_CONTROL_ACTIVE);  
#endif	
#if 0  //test pattern generate
	  WriteByteTPI(0xBC, 0x81); 
	
	

	  
		I2C_WriteByte(0x72, 0xBD, 0x01);		// Enable Auto soft reset on SCDT = 0	
		I2C_WriteByte(0x72, 0xBB, 0x1D);	  
#endif

#if 0 //new MHL cable
	  // Power Up
	  I2C_WriteByte(0x7A, 0x3D, 0x3F);			  // Power up CVCC 1.2V core
	//I2C_WriteByte(0x7A, 0x3D, 0x37);			  // Power up CVCC 1.2V core	  
	
	I2C_WriteByte(0x92, 0x11, 0x01);		  // Enable TxPLL Clock
	  I2C_WriteByte(0x92, 0x12, 0x15);			  // Enable Tx Clock Path & Equalizer
	
	  I2C_WriteByte(0x72, 0x08, 0x35);			  // Power Up TMDS Tx Core
	//I2C_WriteByte(0x72, 0x08, 0x37);			  // Power Up TMDS Tx Core	  
	
	  // Analog PLL Control
	  I2C_WriteByte(0x92, 0x17, 0x03);			  // PLL Calrefsel
	  I2C_WriteByte(0x92, 0x1A, 0x20);			  // VCO Cal
	  I2C_WriteByte(0x92, 0x22, 0x8A);			  // Auto EQ
	  I2C_WriteByte(0x92, 0x23, 0x6A);			  // Auto EQ
	  I2C_WriteByte(0x92, 0x24, 0xAA);			  // Auto EQ
	  I2C_WriteByte(0x92, 0x25, 0xCA);			  // Auto EQ
	  I2C_WriteByte(0x92, 0x26, 0xEA);			  // Auto EQ
	  I2C_WriteByte(0x92, 0x4C, 0xA0);			  // Manual zone control
	
	//I2C_WriteByte(0x92, 0x1C, 0x11); //daniel RX0_offset test 
	//I2C_WriteByte(0x92, 0x1D, 0x11); //daniel RX1_offset	  test		  
	//I2C_WriteByte(0x92, 0x1E, 0x11); //daniel RX2_offset	  test		  
	
	  I2C_WriteByte(0x92, 0x4D, 0x00);			  // PLL Mode Value
	  
	  I2C_WriteByte(0x72, 0x80, 0x14);			  // Enable Rx PLL Clock Value
	  //I2C_WriteByte(0x72, 0x80, 0x24);		  // Enable Rx PLL Clock Value	  
	//I2C_WriteByte(0x72, 0x80, 0x34);			  // Enable Rx PLL Clock Value	  
	
	  I2C_WriteByte(0x92, 0x45, 0x44);			  // Rx PLL BW value from I2C
	  I2C_WriteByte(0x92, 0x31, 0x0A);			  // Rx PLL BW ~ 4MHz
	  I2C_WriteByte(0x72, 0xA0, 0xD0);
	  I2C_WriteByte(0x72, 0xA1, 0xFC);			  // Disable internal Mobile HD driver
	
	  I2C_WriteByte(0x72, 0xA3, 0xFF);
	  I2C_WriteByte(0x72, 0x2B, 0x01);			  // Enable HDCP Compliance workaround
	
	  // CBUS & Discovery
	  //ReadModifyWriteTPI(0x90, BIT_3 | BIT_2, BIT_3);   // CBUS discovery cycle time for each drive and float = 150us
	
	  I2C_WriteByte(0x72, 0x91, 0xE5);		  // Skip RGND detection
	  I2C_WriteByte(0x72, 0x94, 0x66);			  // 1.8V CBUS VTH & GND threshold
	
	  //set bit 2 and 3, which is Initiator Timeout
	  //I2C_WriteByte(CBUS_SLAVE_ADDR, 0x31, I2C_ReadByte(CBUS_SLAVE_ADDR, 0x31) | 0x0c);
	
	  I2C_WriteByte(0x72, 0xA5, 0x00);			  // RGND Hysterisis.
	  I2C_WriteByte(0x72, 0x95, 0x31);			  // RGND & single discovery attempt (RGND blocking)
	  I2C_WriteByte(0x72, 0x96, 0x22);			  // use 1K and 2K setting
	
	  ReadModifyWriteTPI(0x95, SI_BIT_6, SI_BIT_6);	  // Force USB ID switch to open
	
	WriteByteTPI(0x92, 0x46);			  // Force MHD mode
	  WriteByteTPI(0x93, 0xDC); 			  // Disable CBUS pull-up during RGND measurement

	//old cable
//WriteByteTPI(0x92, 0x86);				// Force MHD mode
//WriteByteTPI(0x93, 0xCC);				// Disable CBUS pull-up during RGND measurement
	  
	  delay_ms(25);
	  ReadModifyWriteTPI(0x95, SI_BIT_6, 0x00);	  // Release USB ID switch
	
	  I2C_WriteByte(0x72, 0x90, 0x27);			  // Enable CBUS discovery
	
	  //InitCBusRegs();
	
	  I2C_WriteByte(0x72, 0x05, 0x04);		  // Enable Auto soft reset on SCDT = 0
	
	  I2C_WriteByte(0x72, 0x0D, 0x1C);			  // HDMI Transcode mode enable
	
	WriteByteTPI(TPI_ENABLE, 0x00);   
	
	delay_ms(100); 
	  WriteIndexedRegister(INDEXED_PAGE_0, 0xA0, 0x10);  
	  //WriteByteCBUS(0x07, DDC_XLTN_TIMEOUT_MAX_VAL | 0x0E);	  // Increase DDC translation layer timer (burst mode)
	  //WriteByteCBUS(0x47, 0x03);	
	  //WriteByteCBUS(0x21, 0x01); // Heartbeat Disable
	  
	  delay_ms(100); 
	ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_CONTROL_MASK, TMDS_OUTPUT_CONTROL_ACTIVE);	

#endif

#if 1  //0607 update
// Power Up
I2C_WriteByte(0x7A, 0x3D, 0x3F);			// Power up CVCC 1.2V core

I2C_WriteByte(0x92, 0x11, 0x01);			// Enable TxPLL Clock
I2C_WriteByte(0x92, 0x12, 0x15);			// Enable Tx Clock Path & Equalizer

I2C_WriteByte(0x72, 0x08, 0x35);			// Power Up TMDS Tx Core

I2C_WriteByte(0x92, 0x00, 0x00);			// SIMG: correcting HW default
I2C_WriteByte(0x92, 0x13, 0x60);			// SIMG: Set termination value
I2C_WriteByte(0x92, 0x14, 0xF0);			// SIMG: Change CKDT level
I2C_WriteByte(0x92, 0x4B, 0x06);			// SIMG: Correcting HW default

// Analog PLL Control
I2C_WriteByte(0x92, 0x17, 0x07);			// SIMG: PLL Calrefsel
I2C_WriteByte(0x92, 0x1A, 0x20);			// VCO Cal
I2C_WriteByte(0x92, 0x22, 0xE0);			// SIMG: Auto EQ
I2C_WriteByte(0x92, 0x23, 0xC0);			// SIMG: Auto EQ
I2C_WriteByte(0x92, 0x24, 0xA0);			// SIMG: Auto EQ
I2C_WriteByte(0x92, 0x25, 0x80);			// SIMG: Auto EQ
I2C_WriteByte(0x92, 0x26, 0x60);			// SIMG: Auto EQ
I2C_WriteByte(0x92, 0x27, 0x40);			// SIMG: Auto EQ
I2C_WriteByte(0x92, 0x28, 0x20);			// SIMG: Auto EQ
I2C_WriteByte(0x92, 0x29, 0x00);			// SIMG: Auto EQ

I2C_WriteByte(0x92, 0x4D, 0x02);			// SIMG: PLL Mode Value (order is important)
I2C_WriteByte(0x92, 0x4C, 0xA0);			// Manual zone control


I2C_WriteByte(0x72, 0x80, 0x14);			// Enable Rx PLL Clock Value

I2C_WriteByte(0x92, 0x31, 0x0B);			// SIMG: Rx PLL BW value from I2C BW ~ 4MHz
I2C_WriteByte(0x92, 0x45, 0x06);			// SIMG: DPLL Mode
I2C_WriteByte(0x72, 0xA0, 0x10);			// SIMG: Term mode
I2C_WriteByte(0x72, 0xA1, 0xFC);			// Disable internal Mobile HD driver

I2C_WriteByte(0x72, 0xA3, 0xEB);			// SIMG: Output Swing
I2C_WriteByte(0x72, 0xA6, 0x0C);			// SIMG: Swing Offset

I2C_WriteByte(0x72, 0x2B, 0x01);			// Enable HDCP Compliance workaround

// CBUS & Discovery
//ReadModifyWriteTPI(0x90, BIT_3 | BIT_2, BIT_3);	// CBUS discovery cycle time for each drive and float = 150us

I2C_WriteByte(0x72, 0x91, 0xE5);		// Skip RGND detection
I2C_WriteByte(0x72, 0x94, 0x66);			// 1.8V CBUS VTH & GND threshold

//set bit 2 and 3, which is Initiator Timeout
//I2C_WriteByte(CBUS_SLAVE_ADDR, 0x31, I2C_ReadByte(CBUS_SLAVE_ADDR, 0x31) | 0x0c);

I2C_WriteByte(0x72, 0xA5, 0x80);			// SIMG: RGND Hysterisis, 3x mode for Beast
I2C_WriteByte(0x72, 0x95, 0x31);			// RGND & single discovery attempt (RGND blocking)
I2C_WriteByte(0x72, 0x96, 0x22);			// use 1K and 2K setting

ReadModifyWriteTPI(0x95, SI_BIT_6, SI_BIT_6); 	// Force USB ID switch to open

//WriteByteTPI(0x92, 0x46);				// Force MHD mode
//WriteByteTPI(0x93, 0xDC);				// Disable CBUS pull-up during RGND measurement

//old cable
WriteByteTPI(0x92, 0x86);				// Force MHD mode
WriteByteTPI(0x93, 0xCC);				// Disable CBUS pull-up during RGND measurement


delay_ms(25);
ReadModifyWriteTPI(0x95, SI_BIT_6, 0x00);		// Release USB ID switch

I2C_WriteByte(0x72, 0x90, 0x27);			// Enable CBUS discovery

//InitCBusRegs();

I2C_WriteByte(0x72, 0x05, 0x04);		// Enable Auto soft reset on SCDT = 0

I2C_WriteByte(0x72, 0x0D, 0x1C);			// HDMI Transcode mode enable

WriteByteTPI(TPI_ENABLE, 0x00); 

delay_ms(100); 
WriteIndexedRegister(INDEXED_PAGE_0, 0xA0, 0x10);  
WriteByteCBUS(0x07, DDC_XLTN_TIMEOUT_MAX_VAL | 0x0E); 	// Increase DDC translation layer timer (burst mode)
WriteByteCBUS(0x47, 0x03);  
WriteByteCBUS(0x21, 0x01); // Heartbeat Disable

delay_ms(100); 
ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_CONTROL_MASK, TMDS_OUTPUT_CONTROL_ACTIVE);	

	
#endif
	printk ("==[SIMG] sii9234_initial_registers_set END ==\n");	
}



void sii9234_enable_interrupts(void)
{
	ReadModifyWriteTPI(TPI_INTERRUPT_ENABLE_REG, HOT_PLUG_EVENT_MASK, HOT_PLUG_EVENT_MASK);
	WriteIndexedRegister(INDEXED_PAGE_0, 0x75, SI_BIT_5);	// Enable   
}

byte ReadByteCBUS (byte Offset) 
{
	return I2C_ReadByte(CBUS_SLAVE_ADDR, Offset);
}


void WriteByteCBUS(byte Offset, byte Data) 
{
	I2C_WriteByte(CBUS_SLAVE_ADDR, Offset, Data);
}

byte ReadIndexedRegister (byte PageNum, byte Offset) 
{
	WriteByteTPI(TPI_INDEXED_PAGE_REG, PageNum);		// Indexed page
	WriteByteTPI(TPI_INDEXED_OFFSET_REG, Offset);		// Indexed register
	return ReadByteTPI(TPI_INDEXED_VALUE_REG);			// Return read value
}

void ReadModifyWriteIndexedRegister (byte PageNum, byte Offset, byte Mask, byte Data) 
{

	byte Temp;

	Temp = ReadIndexedRegister (PageNum, Offset);	// Read the current value of the register.
	Temp &= ~Mask;									// Clear the bits that are set in Mask.
	Temp |= (Data & Mask);							// OR in new value. Apply Mask to Value for safety.
	WriteByteTPI(TPI_INDEXED_VALUE_REG, Temp);		// Write new value back to register.
}


void TxPowerStateD3 (void) 
{

	ReadModifyWriteIndexedRegister(INDEXED_PAGE_1, 0x3D, SI_BIT_0, 0x00);
	printk("[SIMG] TX Power State D3\n");
	txPowerState = TX_POWER_STATE_D3;
}

void DisableInterrupts (void) 
{

	ReadModifyWriteTPI(TPI_INTERRUPT_ENABLE_REG, RECEIVER_SENSE_EVENT_MASK, 0x00);
}


void EnableTMDS (void) 
{

	printk("[SIMG] TMDS -> Enabled\n");
	ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_CONTROL_MASK, TMDS_OUTPUT_CONTROL_ACTIVE);
	tmdsPoweredUp = TRUE;
}

static void TxPowerStateD0 (void) 
{

	ReadModifyWriteTPI(TPI_DEVICE_POWER_STATE_CTRL_REG, TX_POWER_STATE_MASK, 0x00);
	TPI_DEBUG_PRINT(("[SIMG] TX Power State D0\n"));
	txPowerState = TX_POWER_STATE_D0;
}

static void EnableInterrupts (void) 
{

	//ReadModifyWriteTPI(TPI_INTERRUPT_ENABLE_REG, RECEIVER_SENSE_EVENT_MASK, RECEIVER_SENSE_EVENT_MASK);
	WriteIndexedRegister(INDEXED_PAGE_0, 0x78, 0x01);	// Enable 
}

void CheckTxFifoStable (void) 
{

	byte bTemp;

	bTemp = ReadIndexedRegister(INDEXED_PAGE_0, 0x3E);			
	if ((bTemp & (SI_BIT_7 | SI_BIT_6)) != 0x00) 
  {
		TPI_DEBUG_PRINT(("[SIMG] FIFO Overrun / Underrun\n"));
		WriteIndexedRegister(INDEXED_PAGE_0, 0x05, SI_BIT_4 | ASR_VALUE);	// Assert MHD FIFO Reset
		delay_ms(1);
		WriteIndexedRegister(INDEXED_PAGE_0, 0x05, ASR_VALUE);			// Deassert MHD FIFO Reset
	}
}
void HotPlugService (void) 
{

	DisableInterrupts();
	EnableTMDS();
	TxPowerStateD0();
	EnableInterrupts();
	CheckTxFifoStable();
}


static void WakeUpFromD3 (void) 
{

	TPI_DEBUG_PRINT(("[SIMG] Waking up...\n"));
  sii9234_tpi_init();
}


void ReadModifyWriteCBUS(byte Offset, byte Mask, byte Value) 
{
  byte Temp;

  Temp = ReadByteCBUS(Offset);
  Temp &= ~Mask;
  Temp |= (Value & Mask);
  WriteByteCBUS(Offset, Temp);
}

void OnMHDCableConnected (void) 
{

	TPI_DEBUG_PRINT (("[SIMG] MHD Connected\n"));

	if (txPowerState == TX_POWER_STATE_D3) 
  {
	    sii9234_start_tpi();
	    EnableInterrupts();
	    TxPowerStateD0();
	}

	mobileHdCableConnected = TRUE;

	WriteIndexedRegister(INDEXED_PAGE_0, 0xA0, 0x10);

	TPI_DEBUG_PRINT (("[SIMG] Setting DDC Burst Mode\n"));
	WriteByteCBUS(0x07, DDC_XLTN_TIMEOUT_MAX_VAL | 0x0E); 	// Increase DDC translation layer timer (burst mode)
	WriteByteCBUS(0x47, 0x03);  

 	WriteByteCBUS(0x21, 0x01); // Heartbeat Disable
 	
}


void ForceUsbIdSwitchOpen (void) 
{
	ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x90, SI_BIT_0, 0x00);				// Disable discovery
	ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x95, SI_BIT_6, SI_BIT_6);				// Force USB ID Switch
	WriteIndexedRegister(INDEXED_PAGE_0, 0x92, 0x46);							// Force MHD mode
	ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x79, SI_BIT_5 | SI_BIT_4, SI_BIT_4);		// Force HPD to 0 when not in MHD mode.
}


static void ReleaseUsbIdSwitchOpen (void) 
{
	delay_ms(25);
	ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x95, SI_BIT_6, 0x00);
	ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x90, SI_BIT_0, SI_BIT_0);				// Enable discovery
}

static void InitForceUsbIdSwitchOpen (void) 
{
	I2C_WriteByte(0x72, 0x90, 0x26);					// Disable CBUS discovery
	ReadModifyWriteTPI(0x95, SI_BIT_6, SI_BIT_6);				// Force USB ID switch to open
	ReadModifyWriteTPI(0x95, SI_BIT_6, SI_BIT_6);				// Force USB ID switch to open
  WriteByteTPI(0x92, 0x46);						// Force MHD mode
}


static void InitReleaseUsbIdSwitchOpen (void) 
{
	delay_ms(25);
	ReadModifyWriteTPI(0x95, SI_BIT_6, 0x00);				// Release USB ID switch
	ReadModifyWriteTPI(0x90, SI_BIT_0, SI_BIT_0);				// Enable CBUS discovery
}

void DisableTMDS (void) 
{

	TPI_DEBUG_PRINT(("[SIMG] TMDS -> Disabled\n"));
	ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_CONTROL_MASK, TMDS_OUTPUT_CONTROL_POWER_DOWN);
	tmdsPoweredUp = FALSE;
}

void OnDownstreamRxPoweredDown (void) 
{

	TPI_DEBUG_PRINT (("[SIMG] DSRX -> Powered Down\n"));

	dsRxPoweredUp = FALSE;
	DisableTMDS();
}

void OnHdmiCableDisconnected (void) 
{

	TPI_DEBUG_PRINT (("[SIMG] HDMI Disconnected\n"));

	hdmiCableConnected = FALSE;
	OnDownstreamRxPoweredDown();
}


void sii9234_tpi_init(void)
{
  MHD_HW_Reset();
  //return;
  if(!MHL_i2c_init)
  {
  	TPI_DEBUG_PRINT (("[MHD] I2C not ready\n"));
	return;
  }
  
  //sii9234_initial_registers_set();  //previous setting
  printk("[MHD]9234 init ++ \n");
  //DisableInterrupts();
  //MHD_INT_clear();
  sii9234_register_init();
  sii9234_start_tpi();
  //EnableInterrupts();
  mhd_rx_connected();
  enable_mhd_tx();
  set_mhd_power_active_mode();
  mhd_tx_fifo_stable(); //fifo clear
  
  printk("[MHD]9234 init -- \n");
}

EXPORT_SYMBOL(sii9234_tpi_init);

void sii9234_start_tpi(void)
{
	WriteByteTPI(TPI_ENABLE, 0x00);				// Write "0" to 72:C7 to start HW TPI mode
}

int MHD_Read_deviceID(void)
{
	
	byte devID = 0x00;
	word wID = 0x0000;
	
	devID = ReadIndexedRegister(0x00, 0x03);
	wID = devID;
	wID <<= 8;
	devID = ReadIndexedRegister(0x00, 0x02);
	wID |= devID;

	devID = ReadByteTPI(TPI_DEVICE_ID);

	printk ("SiI %04X\n", (int) wID);

	if (devID == SiI_DEVICE_ID) 
	{
		return TRUE;
	}

	printk ("Unsupported TX\n");
	return FALSE;

}

EXPORT_SYMBOL(MHD_Read_deviceID);


void mhd_tx_fifo_stable(void)
{
	byte tmp;

	tmp = ReadIndexedRegister(INDEXED_PAGE_0, 0x3E);
	if ((tmp & (SI_BIT_7 | SI_BIT_6)) != 0x00) 
	{
		WriteIndexedRegister(INDEXED_PAGE_0, 0x05, SI_BIT_4 | ASR_VALUE);	// Assert Mobile HD FIFO Reset
		delay_ms(1);
		WriteIndexedRegister(INDEXED_PAGE_0, 0x05, ASR_VALUE);			// Deassert Mobile HD FIFO Reset
	}
}

void enable_mhd_tx(void)
{
	ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_CONTROL_MASK, TMDS_OUTPUT_CONTROL_ACTIVE);
}

void set_mhd_power_active_mode(void)
{
	ReadModifyWriteTPI(TPI_DEVICE_POWER_STATE_CTRL_REG, TX_POWER_STATE_MASK, 0x00);
}

void mhd_rx_connected(void)
{
  WriteIndexedRegister(INDEXED_PAGE_0, 0xA0, 0x10); //TX termination enable
  WriteByteCBUS(0x07, DDC_XLTN_TIMEOUT_MAX_VAL | 0x0E); 	// Increase DDC translation layer timer (burst mode)
  WriteByteCBUS(0x47, 0x03);
  WriteByteCBUS(0x21, 0x01); // Heartbeat Disable  
}

void sii9234_register_init(void)
{
	// Power Up
	I2C_WriteByte(0x7A, 0x3D, 0x3F);			// Power up CVCC 1.2V core
	I2C_WriteByte(0x92, 0x11, 0x01);			// Enable TxPLL Clock
	I2C_WriteByte(0x92, 0x12, 0x15);			// Enable Tx Clock Path & Equalizer
	I2C_WriteByte(0x72, 0x08, 0x35);			// Power Up TMDS Tx Core

	I2C_WriteByte(0x92, 0x00, 0x00);			// SIMG: correcting HW default
	I2C_WriteByte(0x92, 0x13, 0x60);			// SIMG: Set termination value
	I2C_WriteByte(0x92, 0x14, 0xF0);			// SIMG: Change CKDT level
	I2C_WriteByte(0x92, 0x4B, 0x06);			// SIMG: Correcting HW default
	
	// Analog PLL Control
	I2C_WriteByte(0x92, 0x17, 0x07);			// SIMG: PLL Calrefsel
	I2C_WriteByte(0x92, 0x1A, 0x20);			// VCO Cal
	I2C_WriteByte(0x92, 0x22, 0xE0);			// SIMG: Auto EQ
	I2C_WriteByte(0x92, 0x23, 0xC0);			// SIMG: Auto EQ
	I2C_WriteByte(0x92, 0x24, 0xA0);			// SIMG: Auto EQ
	I2C_WriteByte(0x92, 0x25, 0x80);			// SIMG: Auto EQ
	I2C_WriteByte(0x92, 0x26, 0x60);			// SIMG: Auto EQ
	I2C_WriteByte(0x92, 0x27, 0x40);			// SIMG: Auto EQ
	I2C_WriteByte(0x92, 0x28, 0x20);			// SIMG: Auto EQ
	I2C_WriteByte(0x92, 0x29, 0x00);			// SIMG: Auto EQ

	I2C_WriteByte(0x92, 0x4D, 0x02);			// SIMG: PLL Mode Value (order is important)
	I2C_WriteByte(0x92, 0x4C, 0xA0);			// Manual zone control

	I2C_WriteByte(0x72, 0x80, 0x14);			// Enable Rx PLL Clock Value
	//I2C_WriteByte(0x72, 0x80, 0x34);

	I2C_WriteByte(0x92, 0x31, 0x0B);			// SIMG: Rx PLL BW value from I2C BW ~ 4MHz
	I2C_WriteByte(0x92, 0x45, 0x06);			// SIMG: DPLL Mode
	I2C_WriteByte(0x72, 0xA0, 0xD0);			// SIMG: Term mode
	I2C_WriteByte(0x72, 0xA1, 0xFC);			// Disable internal Mobile HD driver

	I2C_WriteByte(0x72, 0xA3, 0xFF);			// SIMG: Output Swing  default EB
	I2C_WriteByte(0x72, 0xA6, 0x0C);			// SIMG: Swing Offset

	I2C_WriteByte(0x72, 0x2B, 0x01);			// Enable HDCP Compliance workaround

	// CBUS & Discovery
	ReadModifyWriteTPI(0x90, SI_BIT_3 | SI_BIT_2, SI_BIT_3);	// CBUS discovery cycle time for each drive and float = 150us

	I2C_WriteByte(0x72, 0x91, 0xE5);		// Skip RGND detection
	
	I2C_WriteByte(0x72, 0x94, 0x66);			// 1.8V CBUS VTH & GND threshold

	//set bit 2 and 3, which is Initiator Timeout
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x31, I2C_ReadByte(CBUS_SLAVE_ADDR, 0x31) | 0x0c);

	// original 3x config
	I2C_WriteByte(0x72, 0xA5, 0x80);			// SIMG: RGND Hysterisis, 3x mode for Beast
	I2C_WriteByte(0x72, 0x95, 0x31);			// RGND & single discovery attempt (RGND blocking)
	I2C_WriteByte(0x72, 0x96, 0x22);			// use 1K and 2K setting

	ReadModifyWriteTPI(0x95, SI_BIT_6, SI_BIT_6);		// Force USB ID switch to open

	WriteByteTPI(0x92, 0x46);				// Force MHD mode
	WriteByteTPI(0x93, 0xDC);				// Disable CBUS pull-up during RGND measurement
		
	ReadModifyWriteTPI(0x79, SI_BIT_1 | SI_BIT_2, 0);        //daniel test...MHL_INT
		
	delay_ms(25);
	ReadModifyWriteTPI(0x95, SI_BIT_6, 0x00);		// Release USB ID switch

	I2C_WriteByte(0x72, 0x90, 0x27);			// Enable CBUS discovery

	InitCBusRegs();

	I2C_WriteByte(0x72, 0x05, ASR_VALUE); 		// Enable Auto soft reset on SCDT = 0

	I2C_WriteByte(0x72, 0x0D, 0x1C); 			// HDMI Transcode mode enable
}

