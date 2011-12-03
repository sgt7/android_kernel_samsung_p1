#include "tune_cmc623_value_VA.h"
#include "tune_cmc623_value_PLS.h"
#include "tune_cmc623_value_VA50.h"
#include "tune_cmc623_value_TN.h"
#include "tune_cmc623_value_FFS.h"
#include "tune_cmc623_value_LCDPLS.h"

mDNIe_data_type cmc623_Bypass[]= 
{
	{0x0000,0x0000},	//BANK 0
	{0x0001,0x0020},	//LABC
	{0x002c,0x0fff},	//DNR bypass {0x003C
	{0x002d,0x1900},	//DNR bypass {0x0a08
	{0x002e,0x0000},	//DNR bypass {0x1010
	{0x002f,0x00ff},	//DNR bypass {0x0400
	{0x003a,0x0000},	//HDTR off
//	{0x00B4,0x4640},	//CABC PWM set
	{0x0000,0x0001},	//BANK 1
	//{0x0020,0x0000},	//GAMMA bypass
	//{0x0021,0x2000},
	//{0x0022,0x2000},
	//{0x0023,0x2000},
	//{0x0024,0x2000},
	//{0x0025,0x2000},
	//{0x0026,0x2000},
	//{0x0027,0x2000},
	//{0x0028,0x2000},
	//{0x0029,0x2000},
	//{0x002A,0x2000},
	//{0x002B,0x2000},
	//{0x002C,0x2000},
	//{0x002D,0x2000},
	//{0x002E,0x2000},
	//{0x002F,0x2000},
	//{0x0030,0x2000},
	//{0x0031,0x2000},
	//{0x0032,0x2000},
	//{0x0033,0x2000},
	//{0x0034,0x2000},
	//{0x0035,0x2000},
	//{0x0036,0x2000},
	//{0x0037,0x2000},
	//{0x0038,0xFF00},
	//{0x0020,0x0001},
	{0x0000,0x0000},	//BANK 0

	{END_SEQ,0x0000},
};

mDNIe_data_type cmc623_Bypass_CABC[]= 
{
	//start 
	{0x0000,0x0000},	//BANK 0
	{0x0001,0x0030},	//LABC CABC
	{0x002c,0x0fff},	//DNR bypass {0x003C
	{0x002d,0x1900},	//DNR bypass {0x0a08
	{0x002e,0x0000},	//DNR bypass {0x1010
	{0x002f,0x00ff},	//DNR bypass {0x0400
	{0x003a,0x0000},	//HDTR off
	{0x006E,0x0000},	//CABC Fgain
	{0x006F,0x0000},
	{0x0070,0x0000},
	{0x0071,0x0000},
	{0x0072,0x2110},	//CABC Dgain
	{0x0073,0x2B14},
	{0x0074,0x1e2D},
	{0x0075,0x3F00},
//	{0x0076,0x3c50},	//PowerLUT
//	{0x0077,0x2d64},	//PowerLUT
//	{0x0078,0x3c32},	//PowerLUT
//	{0x0079,0x1e10},	//PowerLUT
//	{0x007a,0x3200},	//PowerLUT
	{0x007C,0x0002},	//Dynamic LCD
	{0x00B4,0x5640},	//CABC PWM set
	{0x0000,0x0001},	//BANK 1
	//{0x0020,0x0000},	//GAMMA bypass
	//{0x0021,0x2000},
	//{0x0022,0x2000},
	//{0x0023,0x2000},
	//{0x0024,0x2000},
	//{0x0025,0x2000},
	//{0x0026,0x2000},
	//{0x0027,0x2000},
	//{0x0028,0x2000},
	//{0x0029,0x2000},
	//{0x002A,0x2000},
	//{0x002B,0x2000},
	//{0x002C,0x2000},
	//{0x002D,0x2000},
	//{0x002E,0x2000},
	//{0x002F,0x2000},
	//{0x0030,0x2000},
	//{0x0031,0x2000},
	//{0x0032,0x2000},
	//{0x0033,0x2000},
	//{0x0034,0x2000},
	//{0x0035,0x2000},
	//{0x0036,0x2000},
	//{0x0037,0x2000},
	//{0x0038,0xFF00},
	//{0x0020,0x0001},
	{0x0000,0x0000},	//BANK 0

	{END_SEQ,0x0000},
};

typedef enum
{
	CMC_Bypass,
	CMC_Bypass_CABC,
	CMC_Video,
	CMC_Video_CABC,
	CMC_Camera,
	CMC_Camera_CABC,
	CMC_UI,
	CMC_UI_CABC,
	CMC_VT,
	CMC_VT_CABC,
	CMC_DMB,
	CMC_DMB_CABC,
	CMC_GALLERY,
	CMC_GALLERY_CABC,
}Cmc623_Value_Type;

mDNIe_data_type* cmc623_values[]=
{
	cmc623_Bypass,
	cmc623_Bypass,
	cmc623_Bypass,
	cmc623_Bypass,
	cmc623_Bypass,
	cmc623_Bypass,
	cmc623_Bypass,
	cmc623_Bypass,
	
	cmc623_Bypass_CABC,
	cmc623_Bypass_CABC,
	cmc623_Bypass_CABC,
	cmc623_Bypass_CABC,
	cmc623_Bypass_CABC,
	cmc623_Bypass_CABC,
	cmc623_Bypass_CABC,
	cmc623_Bypass_CABC,
	
	cmc623_Video_VA,
	cmc623_Video_PLS,
	cmc623_Video_VA50,
	cmc623_Video_TN,
	cmc623_Video_FFS,
	cmc623_Video_LCDPLS,
	cmc623_Video_PLS,
	cmc623_Video_PLS,
	
	cmc623_Video_CABC_VA,
	cmc623_Video_CABC_PLS,
	cmc623_Video_CABC_VA50,
	cmc623_Video_CABC_TN,
	cmc623_Video_CABC_FFS,
	cmc623_Video_CABC_LCDPLS,
	cmc623_Video_CABC_PLS,
	cmc623_Video_CABC_PLS,
	
	cmc623_Camera_VA,
	cmc623_Camera_PLS,
	cmc623_Camera_VA50,
	cmc623_Camera_TN,
	cmc623_Camera_FFS,
	cmc623_Camera_LCDPLS,
	cmc623_Camera_PLS,
	cmc623_Camera_PLS,
	
	cmc623_Camera_CABC_VA,
	cmc623_Camera_CABC_PLS,
	cmc623_Camera_CABC_VA50,
	cmc623_Camera_CABC_TN,
	cmc623_Camera_CABC_FFS,
	cmc623_Camera_CABC_LCDPLS,
	cmc623_Camera_CABC_PLS,
	cmc623_Camera_CABC_PLS,
	
	cmc623_UI_VA,
	cmc623_UI_PLS,
	cmc623_UI_VA50,
	cmc623_UI_TN,
	cmc623_UI_FFS,
	cmc623_UI_LCDPLS,
	cmc623_UI_PLS,
	cmc623_UI_PLS,
	
	cmc623_UI_CABC_VA,
	cmc623_UI_CABC_PLS,
	cmc623_UI_CABC_VA50,
	cmc623_UI_CABC_TN,
	cmc623_UI_CABC_FFS,
	cmc623_UI_CABC_LCDPLS,
	cmc623_UI_CABC_PLS,
	cmc623_UI_CABC_PLS,
	
	cmc623_VT_VA,
	cmc623_VT_PLS,
	cmc623_VT_VA50,
	cmc623_VT_TN,
	cmc623_VT_FFS,
	cmc623_VT_LCDPLS,
	cmc623_VT_PLS,
	cmc623_VT_PLS,
	
	cmc623_VT_CABC_VA,
	cmc623_VT_CABC_PLS,
	cmc623_VT_CABC_VA50,
	cmc623_VT_CABC_TN,
	cmc623_VT_CABC_FFS,
	cmc623_VT_CABC_LCDPLS,
	cmc623_VT_CABC_PLS,
	cmc623_VT_CABC_PLS,
	
	cmc623_DMB_VA,
	cmc623_DMB_PLS,
	cmc623_DMB_VA50,
	cmc623_DMB_TN,
	cmc623_DMB_FFS,
	cmc623_DMB_LCDPLS,
	cmc623_DMB_PLS,
	cmc623_DMB_PLS,
	
	cmc623_DMB_CABC_VA,
	cmc623_DMB_CABC_PLS,
	cmc623_DMB_CABC_VA50,
	cmc623_DMB_CABC_TN,
	cmc623_DMB_CABC_FFS,
	cmc623_DMB_CABC_LCDPLS,
	cmc623_DMB_CABC_PLS,
	cmc623_DMB_CABC_PLS,
	
	cmc623_GALLERY_VA,
	cmc623_GALLERY_PLS,
	cmc623_GALLERY_VA50,
	cmc623_GALLERY_TN,
	cmc623_GALLERY_FFS,
	cmc623_GALLERY_LCDPLS,
	cmc623_GALLERY_PLS,
	cmc623_GALLERY_PLS,
	
	cmc623_GALLERY_CABC_VA,
	cmc623_GALLERY_CABC_PLS,
	cmc623_GALLERY_CABC_VA50,
	cmc623_GALLERY_CABC_TN,
	cmc623_GALLERY_CABC_FFS,
	cmc623_GALLERY_CABC_LCDPLS,
	cmc623_GALLERY_CABC_PLS,
	cmc623_GALLERY_CABC_PLS,
};

u16 OVE_values[] = 
{
	0x0000,
	0x0000,
	0x6050,//video
	0x6050,//video cabc
	0x6050,//camera
	0x6050,//camera cabc
	0x4040,//ui
	0x3040,//ui cabc
	0x0000,//vt
	0x0000,//vt cabc
	0x6050,//dmb
	0x6050,//dmb cabc
	0x0000,//gallery
	0x0000,//gallery cabc
};

mDNIe_data_type* cmc623_white_val_values[]=
{
	cmc623_White_VA,
	cmc623_White_CABC_VA,
	cmc623_White_PLS,
	cmc623_White_CABC_PLS,
	cmc623_White_VA50,
	cmc623_White_CABC_VA50,
	cmc623_White_TN,
	cmc623_White_CABC_TN,
	cmc623_White_FFS,
	cmc623_White_CABC_FFS,
	cmc623_White_LCDPLS,
	cmc623_White_CABC_LCDPLS,
	cmc623_White_PLS,
	cmc623_White_CABC_PLS,
	cmc623_White_PLS,
	cmc623_White_CABC_PLS,
};

#if 0
mDNIe_data_type* cmc623_white_values[]=
{
	cmc623_White_Minus_2_VA,
	cmc623_White_Minus_2_CABC_VA,
	cmc623_White_Minus_2_PLS,
	cmc623_White_Minus_2_CABC_PLS,
//	cmc623_White_Minus_2_VA50,
//	cmc623_White_Minus_2_CABC_VA50,
//	cmc623_White_Minus_2_TN,
//	cmc623_White_Minus_2_CABC_TN,
//	cmc623_White_Minus_2_FFS,
//	cmc623_White_Minus_2_CABC_FFS,
	cmc623_White_Minus_1_VA,
	cmc623_White_Minus_1_CABC_VA,
	cmc623_White_Minus_1_PLS,
	cmc623_White_Minus_1_CABC_PLS,
//	cmc623_White_Minus_1_VA50,
//	cmc623_White_Minus_1_CABC_VA50,
//	cmc623_White_Minus_1_TN,
//	cmc623_White_Minus_1_CABC_TN,
//	cmc623_White_Minus_1_FFS,
//	cmc623_White_Minus_1_CABC_FFS,
	cmc623_White_0_VA,
	cmc623_White_0_CABC_VA,
	cmc623_White_0_PLS,
	cmc623_White_0_CABC_PLS,
//	cmc623_White_0_VA50,
//	cmc623_White_0_CABC_VA50,
//	cmc623_White_0_TN,
//	cmc623_White_0_CABC_TN,
//	cmc623_White_0_FFS,
//	cmc623_White_0_CABC_FFS,
	cmc623_White_Plus_1_VA,
	cmc623_White_Plus_1_CABC_VA,
	cmc623_White_Plus_1_PLS,
	cmc623_White_Plus_1_CABC_PLS,
//	cmc623_White_Plus_1_VA50,
//	cmc623_White_Plus_1_CABC_VA50,
//	cmc623_White_Plus_1_TN,
//	cmc623_White_Plus_1_CABC_TN,
//	cmc623_White_Plus_1_FFS,
//	cmc623_White_Plus_1_CABC_FFS,
	cmc623_White_Plus_2_VA,
	cmc623_White_Plus_2_CABC_VA,
	cmc623_White_Plus_2_PLS,
	cmc623_White_Plus_2_CABC_PLS,
//	cmc623_White_Plus_2_VA50,
//	cmc623_White_Plus_2_CABC_VA50,
//	cmc623_White_Plus_2_TN,
//	cmc623_White_Plus_2_CABC_TN,
//	cmc623_White_Plus_2_FFS,
//	cmc623_White_Plus_2_CABC_FFS,
};
#endif

mDNIe_data_type* cmc623_saturation_val_values[]=
{
	cmc623_Saturation_VA,
	cmc623_Saturation_CABC_VA,
	cmc623_Saturation_PLS,
	cmc623_Saturation_CABC_PLS,
	cmc623_Saturation_VA50,
	cmc623_Saturation_CABC_VA50,
	cmc623_Saturation_TN,
	cmc623_Saturation_CABC_TN,
	cmc623_Saturation_FFS,
	cmc623_Saturation_CABC_FFS,
	cmc623_Saturation_LCDPLS,
	cmc623_Saturation_CABC_LCDPLS,
	cmc623_Saturation_PLS,
	cmc623_Saturation_CABC_PLS,
	cmc623_Saturation_PLS,
	cmc623_Saturation_CABC_PLS,
};

#if 0
mDNIe_data_type* cmc623_saturation_values[]=
{
	cmc623_Saturation_Minus_2_VA,
	cmc623_Saturation_Minus_2_CABC_VA,
	cmc623_Saturation_Minus_2_PLS,
	cmc623_Saturation_Minus_2_CABC_PLS,
//	cmc623_Saturation_Minus_2_VA50,
//	cmc623_Saturation_Minus_2_CABC_VA50,
//	cmc623_Saturation_Minus_2_TN,
//	cmc623_Saturation_Minus_2_CABC_TN,
//	cmc623_Saturation_Minus_2_FFS,
//	cmc623_Saturation_Minus_2_CABC_FFS,
	cmc623_Saturation_Minus_1_VA,
	cmc623_Saturation_Minus_1_CABC_VA,
	cmc623_Saturation_Minus_1_PLS,
	cmc623_Saturation_Minus_1_CABC_PLS,
//	cmc623_Saturation_Minus_1_VA50,
//	cmc623_Saturation_Minus_1_CABC_VA50,
//	cmc623_Saturation_Minus_1_TN,
//	cmc623_Saturation_Minus_1_CABC_TN,
//	cmc623_Saturation_Minus_1_FFS,
//	cmc623_Saturation_Minus_1_CABC_FFS,
	cmc623_Saturation_0_VA,
	cmc623_Saturation_0_CABC_VA,
	cmc623_Saturation_0_PLS,
	cmc623_Saturation_0_CABC_PLS,
//	cmc623_Saturation_0_VA50,
//	cmc623_Saturation_0_CABC_VA50,
//	cmc623_Saturation_0_TN,
//	cmc623_Saturation_0_CABC_TN,
//	cmc623_Saturation_0_FFS,
//	cmc623_Saturation_0_CABC_FFS,
	cmc623_Saturation_Plus_1_VA,
	cmc623_Saturation_Plus_1_CABC_VA,
	cmc623_Saturation_Plus_1_PLS,
	cmc623_Saturation_Plus_1_CABC_PLS,
//	cmc623_Saturation_Plus_1_VA50,
//	cmc623_Saturation_Plus_1_CABC_VA50,
//	cmc623_Saturation_Plus_1_TN,
//	cmc623_Saturation_Plus_1_CABC_TN,
//	cmc623_Saturation_Plus_1_FFS,
//	cmc623_Saturation_Plus_1_CABC_FFS,
	cmc623_Saturation_Plus_2_VA,
	cmc623_Saturation_Plus_2_CABC_VA,
	cmc623_Saturation_Plus_2_PLS,
	cmc623_Saturation_Plus_2_CABC_PLS,
//	cmc623_Saturation_Plus_2_VA50,
//	cmc623_Saturation_Plus_2_CABC_VA50,
//	cmc623_Saturation_Plus_2_TN,
//	cmc623_Saturation_Plus_2_CABC_TN,
//	cmc623_Saturation_Plus_2_FFS,
//	cmc623_Saturation_Plus_2_CABC_FFS,
};
#endif

mDNIe_data_type* cmc623_black_values[]=
{
	cmc623_Black_Minus_4_VA, 		cmc623_Black_Minus_4_CABC_VA,
	cmc623_Black_Minus_4_PLS,		cmc623_Black_Minus_4_CABC_PLS,
	cmc623_Black_Minus_4_VA50,		cmc623_Black_Minus_4_CABC_VA50,
	cmc623_Black_Minus_4_TN,		cmc623_Black_Minus_4_CABC_TN,
	cmc623_Black_Minus_4_FFS,		cmc623_Black_Minus_4_CABC_FFS,
	cmc623_Black_Minus_4_LCDPLS,	cmc623_Black_Minus_4_CABC_LCDPLS,
	cmc623_Black_Minus_4_PLS,		cmc623_Black_Minus_4_CABC_PLS,
	cmc623_Black_Minus_4_PLS,		cmc623_Black_Minus_4_CABC_PLS,
	
	cmc623_Black_Minus_3_VA,		cmc623_Black_Minus_3_CABC_VA,
	cmc623_Black_Minus_3_PLS,		cmc623_Black_Minus_3_CABC_PLS,
	cmc623_Black_Minus_3_VA50,		cmc623_Black_Minus_3_CABC_VA50,
	cmc623_Black_Minus_3_TN,		cmc623_Black_Minus_3_CABC_TN,
	cmc623_Black_Minus_3_FFS,		cmc623_Black_Minus_3_CABC_FFS,
	cmc623_Black_Minus_3_LCDPLS,	cmc623_Black_Minus_3_CABC_LCDPLS,
	cmc623_Black_Minus_3_PLS,		cmc623_Black_Minus_3_CABC_PLS,
	cmc623_Black_Minus_3_PLS,		cmc623_Black_Minus_3_CABC_PLS,
	
	cmc623_Black_Minus_2_VA, 		cmc623_Black_Minus_2_CABC_VA,
	cmc623_Black_Minus_2_PLS,		cmc623_Black_Minus_2_CABC_PLS,
	cmc623_Black_Minus_2_VA50,		cmc623_Black_Minus_2_CABC_VA50,
	cmc623_Black_Minus_2_TN,		cmc623_Black_Minus_2_CABC_TN,
	cmc623_Black_Minus_2_FFS,		cmc623_Black_Minus_2_CABC_FFS,
	cmc623_Black_Minus_2_LCDPLS,	cmc623_Black_Minus_2_CABC_LCDPLS,
	cmc623_Black_Minus_2_PLS,		cmc623_Black_Minus_2_CABC_PLS,
	cmc623_Black_Minus_2_PLS,		cmc623_Black_Minus_2_CABC_PLS,
	
	cmc623_Black_Minus_1_VA,		cmc623_Black_Minus_1_CABC_VA,
	cmc623_Black_Minus_1_PLS,		cmc623_Black_Minus_1_CABC_PLS,
	cmc623_Black_Minus_1_VA50,		cmc623_Black_Minus_1_CABC_VA50,
	cmc623_Black_Minus_1_TN,		cmc623_Black_Minus_1_CABC_TN,
	cmc623_Black_Minus_1_FFS,		cmc623_Black_Minus_1_CABC_FFS,
	cmc623_Black_Minus_1_LCDPLS,	cmc623_Black_Minus_1_CABC_LCDPLS,
	cmc623_Black_Minus_1_PLS,		cmc623_Black_Minus_1_CABC_PLS,
	cmc623_Black_Minus_1_PLS,		cmc623_Black_Minus_1_CABC_PLS,
	
	cmc623_Black_0_VA,			cmc623_Black_0_CABC_VA,
	cmc623_Black_0_PLS,			cmc623_Black_0_CABC_PLS,
	cmc623_Black_0_VA50,		cmc623_Black_0_CABC_VA50,
	cmc623_Black_0_TN,			cmc623_Black_0_CABC_TN,
	cmc623_Black_0_FFS,			cmc623_Black_0_CABC_FFS,
	cmc623_Black_0_LCDPLS,		cmc623_Black_0_CABC_LCDPLS,
	cmc623_Black_0_PLS,			cmc623_Black_0_CABC_PLS,
	cmc623_Black_0_PLS,			cmc623_Black_0_CABC_PLS,
	
	cmc623_Black_Plus_1_VA,		cmc623_Black_Plus_1_CABC_VA,
	cmc623_Black_Plus_1_PLS,		cmc623_Black_Plus_1_CABC_PLS,
	cmc623_Black_Plus_1_VA50,		cmc623_Black_Plus_1_CABC_VA50,
	cmc623_Black_Plus_1_TN,		cmc623_Black_Plus_1_CABC_TN,
	cmc623_Black_Plus_1_FFS,		cmc623_Black_Plus_1_CABC_FFS,
	cmc623_Black_Plus_1_LCDPLS,	cmc623_Black_Plus_1_CABC_LCDPLS,
	cmc623_Black_Plus_1_PLS,		cmc623_Black_Plus_1_CABC_PLS,
	cmc623_Black_Plus_1_PLS,		cmc623_Black_Plus_1_CABC_PLS,
	
	cmc623_Black_Plus_2_VA,		cmc623_Black_Plus_2_CABC_VA,
	cmc623_Black_Plus_2_PLS,		cmc623_Black_Plus_2_CABC_PLS,
	cmc623_Black_Plus_2_VA50,		cmc623_Black_Plus_2_CABC_VA50,
	cmc623_Black_Plus_2_TN,		cmc623_Black_Plus_2_CABC_TN,
	cmc623_Black_Plus_2_FFS,		cmc623_Black_Plus_2_CABC_FFS,
	cmc623_Black_Plus_2_LCDPLS,	cmc623_Black_Plus_2_CABC_LCDPLS,
	cmc623_Black_Plus_2_PLS,		cmc623_Black_Plus_2_CABC_PLS,
	cmc623_Black_Plus_2_PLS,		cmc623_Black_Plus_2_CABC_PLS,
	
	cmc623_Black_Plus_3_VA,		cmc623_Black_Plus_3_CABC_VA,
	cmc623_Black_Plus_3_PLS,		cmc623_Black_Plus_3_CABC_PLS,
	cmc623_Black_Plus_3_VA50,		cmc623_Black_Plus_3_CABC_VA50,
	cmc623_Black_Plus_3_TN,		cmc623_Black_Plus_3_CABC_TN,
	cmc623_Black_Plus_3_FFS,		cmc623_Black_Plus_3_CABC_FFS,
	cmc623_Black_Plus_3_LCDPLS,	cmc623_Black_Plus_3_CABC_LCDPLS,
	cmc623_Black_Plus_3_PLS,		cmc623_Black_Plus_3_CABC_PLS,
	cmc623_Black_Plus_3_PLS,		cmc623_Black_Plus_3_CABC_PLS,
	
	cmc623_Black_Plus_4_VA,		cmc623_Black_Plus_4_CABC_VA,
	cmc623_Black_Plus_4_PLS,		cmc623_Black_Plus_4_CABC_PLS,
	cmc623_Black_Plus_4_VA50,		cmc623_Black_Plus_4_CABC_VA50,
	cmc623_Black_Plus_4_TN,		cmc623_Black_Plus_4_CABC_TN,
	cmc623_Black_Plus_4_FFS,		cmc623_Black_Plus_4_CABC_FFS,
	cmc623_Black_Plus_4_LCDPLS,	cmc623_Black_Plus_4_CABC_LCDPLS,
	cmc623_Black_Plus_4_PLS,		cmc623_Black_Plus_4_CABC_PLS,
	cmc623_Black_Plus_4_PLS,		cmc623_Black_Plus_4_CABC_PLS,
};
//////////////////////////////////////////////////
mDNIe_data_type cmc623_Color_Tempature_Normal_VA[]=
{
	{0x0000,0x0000},	//BANK 0
	{0x005B,0x0032},	//MCM
	
	{END_SEQ,0x0000},
};

mDNIe_data_type cmc623_Color_Tempature_Warm_VA[]=
{
	{0x0000,0x0000},	//BANK 0
	{0x005B,0x0028},	//MCM 4000K
	
	{END_SEQ,0x0000},
};

mDNIe_data_type cmc623_Color_Tempature_Cold_VA[]=
{
	{0x0000,0x0000},	//BANK 0
	{0x005B,0x0064},	//MCM 10000K
	
	{END_SEQ,0x0000},
};
