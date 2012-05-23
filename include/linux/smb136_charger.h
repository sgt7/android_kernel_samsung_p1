/*
 *  Copyright (C) 2011 Samsung Electronics
 *  Ikkeun Kim <iks.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SMB136_CHARGER_H_
#define __SMB136_CHARGER_H_

struct smb136_charger_data {
	int (*charger_dev_register)(struct charger_device *chgdev);
	void (*charger_dev_unregister)(struct charger_device *chgdev);
	struct charger_device *chgdev;
};

#endif
