/*  $Date: 2007/08/07 16:06:00 $
 *  $Revision: 1.1 $ 
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2010 Bosch Sensortec GmbH
 * All Rights Reserved
 */

/*! \file bma020calib.c
    \brief This file contains all function implementatios for the BMA020/BMA150 calibration process
        
*/
#include <linux/fs.h>

#include "bma020.h"
#include "bma020calib.h"

/** calculates new offset in respect to acceleration data and old offset register values
  \param orientation pass orientation one axis needs to be absolute 1 the others need to be 0
  \param *offset_x takes the old offset value and modifies it to the new calculated one
  \param *offset_y takes the old offset value and modifies it to the new calculated one
  \param *offset_z takes the old offset value and modifies it to the new calculated one
 */
int bma020_calc_new_offset(bma020acc_t orientation, bma020acc_t accel, 
                                             unsigned short *offset_x, unsigned short *offset_y, unsigned short *offset_z)
{
   short old_offset_x, old_offset_y, old_offset_z;
   short new_offset_x, new_offset_y, new_offset_z;   

   unsigned char  calibrated =0;

   old_offset_x = *offset_x;
   old_offset_y = *offset_y;
   old_offset_z = *offset_z;
   
   printk("[%s] before: x = %d, y = %d, z = %d\n", __func__, accel.x, accel.y, accel.z);  
   
   accel.x = accel.x - (orientation.x * 256);   
   accel.y = accel.y - (orientation.y * 256);
   accel.z = accel.z - (orientation.z * 256);   
   
   printk("[%s] after: x = %d, y = %d, z = %d\n", __func__, accel.x, accel.y, accel.z);  
   	                                
   if ((accel.x > 4) | (accel.x < -4)) {           /* does x axis need calibration? */
	     
	 if ((accel.x <8) && accel.x > 0)              /* check for values less than quantisation of offset register */
	 	new_offset_x= old_offset_x -1;           
	 else if ((accel.x >-8) && (accel.x < 0))    
	   new_offset_x= old_offset_x +1;
     else 
       new_offset_x = old_offset_x - (accel.x/8);  /* calculate new offset due to formula */
     if (new_offset_x <0) 						/* check for register boundary */
	   new_offset_x =0;							/* <0 ? */
     else if (new_offset_x>1023)
	   new_offset_x=1023;                       /* >1023 ? */
     *offset_x = new_offset_x;     
	 calibrated = 1;
   }

   if ((accel.y > 4) | (accel.y<-4)) {            /* does y axis need calibration? */
	 if ((accel.y <8) && accel.y > 0)             /* check for values less than quantisation of offset register */
	 	new_offset_y= old_offset_y -1;
	 else if ((accel.y >-8) && accel.y < 0)
	   new_offset_y= old_offset_y +1;	    
     else 
       new_offset_y = old_offset_y - accel.y/8;  /* calculate new offset due to formula */
     
	 if (new_offset_y <0) 						/* check for register boundary */
	   new_offset_y =0;							/* <0 ? */
     else if (new_offset_y>1023)
       new_offset_y=1023;                       /* >1023 ? */
   
     *offset_y = new_offset_y;
     calibrated = 1;
   }

   if ((accel.z > 4) | (accel.z<-4)) {            /* does z axis need calibration? */
	 if ((accel.z <8) && accel.z > 0)             /* check for values less than quantisation of offset register */  
	 	new_offset_z= old_offset_z -1;
	 else if ((accel.z >-8) && accel.z < 0)
	   new_offset_z= old_offset_z +1;	     
     else 
       new_offset_z = old_offset_z - (accel.z/8);/* calculate new offset due to formula */
 
     if (new_offset_z <0) 						/* check for register boundary */
	   new_offset_z =0;							/* <0 ? */
     else if (new_offset_z>1023)
	   new_offset_z=1023;

	 *offset_z = new_offset_z;
	 calibrated = 1;
  }	   
  return calibrated;
}

/** reads out acceleration data and averages them, measures min and max
  \param orientation pass orientation one axis needs to be absolute 1 the others need to be 0
  \param num_avg numer of samples for averaging
  \param *min returns the minimum measured value
  \param *max returns the maximum measured value
  \param *average returns the average value
 */

int bma020_read_accel_avg(int num_avg, bma020acc_t *min, bma020acc_t *max, bma020acc_t *avg )
{
   long x_avg=0, y_avg=0, z_avg=0;   
   int comres=0;
   int i;
   bma020acc_t accel;		                /* read accel data */

   x_avg = 0; y_avg=0; z_avg=0;                  
   max->x = -512; max->y =-512; max->z = -512;
   min->x = 512;  min->y = 512; min->z = 512; 
     

	 for (i=0; i<num_avg; i++) {
	 	comres += bma020_read_accel_xyz(&accel);      /* read 10 acceleration data triples */
		accel.y = -accel.y;
		accel.z = -accel.z;
		
		if (accel.x>max->x)
			max->x = accel.x;
		if (accel.x<min->x) 
			min->x=accel.x;

		if (accel.y>max->y)
			max->y = accel.y;
		if (accel.y<min->y) 
			min->y=accel.y;

		if (accel.z>max->z)
			max->z = accel.z;
		if (accel.z<min->z) 
			min->z=accel.z;
		
		x_avg+= accel.x;
		y_avg+= accel.y;
		z_avg+= accel.z;

		bma020_pause(10);
     }
	 avg->x = x_avg /= num_avg;                             /* calculate averages, min and max values */
	 avg->y = y_avg /= num_avg;
	 avg->z = z_avg /= num_avg;
	 return comres;
}
	 

/** verifies the accerleration values to be good enough for calibration calculations
 \param min takes the minimum measured value
  \param max takes the maximum measured value
  \param takes returns the average value
  \return 1: min,max values are in range, 0: not in range
*/

int bma020_verify_min_max(bma020acc_t min, bma020acc_t max, bma020acc_t avg) 
{
	short dx, dy, dz;
	int ver_ok=1;
	
	dx =  max.x - min.x;    /* calc delta max-min */
	dy =  max.y - min.y;
	dz =  max.z - min.z;


	if (dx> 10 || dx<-10) 
	  ver_ok = 0;
	if (dy> 10 || dy<-10) 
	  ver_ok = 0;
	if (dz> 10 || dz<-10) 
	  ver_ok = 0;

	return ver_ok;
}	



/** overall calibration process. This function takes care about all other functions 
  \param orientation input for orientation [0;0;1] for measuring the device in horizontal surface up
  \param *tries takes the number of wanted iteration steps, this pointer returns the number of calculated steps after this routine has finished
  \return 1: calibration passed 2: did not pass within N steps 
*/
  
int bma020_calibrate(bma020acc_t orientation, int *tries)
{

	unsigned short offset_x, offset_y, offset_z;
	unsigned short old_offset_x, old_offset_y, old_offset_z;
	unsigned short changed_offset_x, changed_offset_y, changed_offset_z;
	int need_calibration=0, min_max_ok=0;	
	int ltries;
	int retry = 30;

	bma020acc_t min,max,avg;
	
	printk("[%s] +\n", __func__);

	bma020_set_range(BMA020_RANGE_2G);
	bma020_set_bandwidth(BMA020_BW_25HZ);

	bma020_set_ee_w(1);  
	
	bma020_get_offset(0, &offset_x);
	bma020_get_offset(1, &offset_y);
	bma020_get_offset(2, &offset_z);	

	old_offset_x = offset_x;
	old_offset_y = offset_y;
	old_offset_z = offset_z;
	ltries = *tries;
	
	printk("[%s] old offset_x = %d, offset_y = %d, offset_z = %d\n", __func__, old_offset_x, offset_y, offset_z);
	
	orientation.x = 0;
	orientation.y = 0;
	orientation.z = 1;

	do {
		bma020_read_accel_avg(10, &min, &max, &avg);  /* read acceleration data min, max, avg */

		min_max_ok = bma020_verify_min_max(min, max, avg);

		if(!min_max_ok)
		{
			retry--;
			if(retry <= 0)
				return (-1);
		}
		
		/* check if calibration is needed */
		if (min_max_ok)
			need_calibration = bma020_calc_new_offset(orientation, avg, &offset_x, &offset_y, &offset_z);		
		  
		if (*tries==0) /*number of maximum tries reached? */
			break;

		if (need_calibration) {   
			/* when needed calibration is updated in image */
			printk("[%s] need calibration. tries = %d. changed offset x = %d, y = %d, z = %d\n", __func__, *tries, offset_x, offset_y, offset_z);
			(*tries)--;
			bma020_set_offset(0, offset_x); 
   		 	bma020_set_offset(1, offset_y);
 	  	 	bma020_set_offset(2, offset_z);
			bma020_pause(20);
		}
		printk("\n");
    } while (need_calibration || !min_max_ok);

	        		
    if (*tries>0 && *tries < ltries) {
		printk("[%s] eeprom is updated. new offset x=%d, y=%d, z=%d\n", 
					__func__, offset_x, offset_y, offset_z);
		
		if (old_offset_x!= offset_x) 
			bma020_set_offset_eeprom(0, offset_x);

		if (old_offset_y!= offset_y) 
	   		bma020_set_offset_eeprom(1,offset_y);

		if (old_offset_z!= offset_z) 
	   		bma020_set_offset_eeprom(2, offset_z);
	}	
	
	printk("[%s] tries = %d\n", __func__, *tries);
		
	bma020_set_ee_w(0);	    
    bma020_pause(20);
	*tries = ltries - *tries;
	
	printk("[%s] -\n", __func__);

	return !need_calibration;
}



	

