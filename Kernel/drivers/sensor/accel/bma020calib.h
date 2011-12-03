/*  $Date: 2007/08/07 16:06:00 $
 *  $Revision: 1.1 $ 
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2010 Bosch Sensortec GmbH
 * All Rights Reserved
 */

/*! \file bma020calib.h
    \brief This file contains all function headers for the BMA020/BMA150 calibration process
        
*/

#ifndef __BMA020_CALIBRATION__
#define __BMA020_CALIBRATION__


#define BMA020_CALIBRATION_MAX_TRIES 10


/** calculates new offset in respect to acceleration data and old offset register values
  \param orientation pass orientation one axis needs to be absolute 1 the others need to be 0
  \param *offset_x takes the old offset value and modifies it to the new calculated one
  \param *offset_y takes the old offset value and modifies it to the new calculated one
  \param *offset_z takes the old offset value and modifies it to the new calculated one
 */

int bma020_calibrate(bma020acc_t, int *);

/** reads out acceleration data and averages them, measures min and max
  \param orientation pass orientation one axis needs to be absolute 1 the others need to be 0
  \param num_avg numer of samples for averaging
  \param *min returns the minimum measured value
  \param *max returns the maximum measured value
  \param *average returns the average value
 */
int bma020_read_accel_avg(int, bma020acc_t *, bma020acc_t *, bma020acc_t * );

/** verifies the accerleration values to be good enough for calibration calculations
 \param min takes the minimum measured value
  \param max takes the maximum measured value
  \param takes returns the average value
  \return 1: min,max values are in range, 0: not in range
*/
int bma020_verify_min_max(bma020acc_t , bma020acc_t , bma020acc_t );

/** bma020_calibration routine
  \param orientation pass orientation one axis needs to be absolute 1 the others need to be 0
  \param tries number of iterative passes
  \param *min, *max, *avg returns minimum, maximum and average offset 
 */
int bma020_calc_new_offset(bma020acc_t, bma020acc_t, unsigned short *, unsigned short *, unsigned short *);

/** overall calibration process. This function takes care about all other functions 
  \param orientation input for orientation [0;0;1] for measuring the device in horizontal surface up
  \param *tries takes the number of wanted iteration steps, this pointer returns the number of calculated steps after this routine has finished
  \return 1: calibration passed 2: did not pass within N steps 
*/
int bma020_store_calibration(unsigned short, unsigned short, unsigned short);


#endif // endif __BMA020_CALIBRATION__
