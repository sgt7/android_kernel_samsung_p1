
/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2010 Bosch Sensortec GmbH
 * All Rights Reserved
 */

#ifndef __BMA020_I2C_HEADER__
#define __BMA020_I2C_HEADER__

char  i2c_acc_bma020_read (u8, u8 *, unsigned int);
char  i2c_acc_bma020_write(u8 reg, u8 *val);
void i2c_acc_bma020_delay(unsigned int msec);

int  i2c_acc_bma020_init(void);
void i2c_acc_bma020_exit(void);

#endif
