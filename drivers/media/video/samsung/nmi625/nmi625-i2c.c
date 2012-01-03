/*****************************************************************************
 Copyright(c) 2010 NMI Inc. All Rights Reserved
 
 File name : nmi625-i2c.c
 
 Description :  Generic I2C driver for NM625
 
 History : 
 ----------------------------------------------------------------------
 2010/05/17 	ssw		initial
*******************************************************************************/

#include "nmi625-i2c.h"

static struct i2c_driver nmi625_i2c_driver;
static struct i2c_client *nmi625_i2c_client = NULL;

int nmi625_init = 0;

struct nmi625_state{
	struct i2c_client	*client;	
};
struct nmi625_state *nmi625_state;


int nmi625_i2c_init(void)
{
	int res;

	printk("nmi625_i2c_init ENTER...\n");
	nmi625_i2c_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	
	if(nmi625_i2c_client== NULL) {
		printk("nmi625_i2c_client NULL...\n");
		return -ENOMEM;
	}

	if (res=i2c_add_driver(&nmi625_i2c_driver))
		pr_err("%s: Can't add nmi625 i2c drv\n", __func__);
	else
		pr_info("%s: Added nmi625 i2c drv\n", __func__);

	return res;
}


int nmi625_i2c_deinit(void)
{
	printk("nmi625_i2c_deinit ENTER...\n");

	i2c_del_driver(&nmi625_i2c_driver);

	return 0;
}


int nmi625_i2c_read(void *hDevice, unsigned short addr, unsigned char *data, unsigned short length) 
{
	int res;
	struct i2c_msg rmsg;

	rmsg.addr = addr;
	rmsg.flags = I2C_M_RD;
	rmsg.len = length;
	rmsg.buf = data;
		
	res = i2c_transfer(nmi625_i2c_client->adapter, &rmsg, 1);

	return 0;
}

int nmi625_i2c_write(void *hDevice, unsigned short addr, unsigned char *data, unsigned short length)
{
	int res;
	struct i2c_msg wmsg;

	if(length+1>I2C_MAX_SEND_LENGTH)
	{
		printk(".......error", __FUNCTION__);
		return -ENODEV;
	}
	wmsg.addr = addr;
	wmsg.flags = I2C_M_WR;
	wmsg.len = length;
	wmsg.buf = data;
	
	res = i2c_transfer(nmi625_i2c_client->adapter, &wmsg, 1);

	return 0;
}

void nmi625_i2c_read_chip_id(void)
{
	u8 cmd[16] = {0,};
	u8 cmd1[16] = {0,};
	struct i2c_msg rmsg;
	struct i2c_msg wmsg;

	cmd[0] = 0x80;
	cmd[1] = 0x00;
	cmd[2] = 0x64;
	cmd[3] = 0x00;
	cmd[4] = 0x00;
	cmd[5] = 0x04;

	wmsg.addr = 0x61;
	wmsg.flags = I2C_M_WR;
	wmsg.len = 6;
	wmsg.buf = cmd;

	printk("nmi625_i2c_read_chip_id()\n");
	i2c_transfer(nmi625_i2c_client->adapter, &wmsg, 1);

	printk("nmi625_i2c_client->addr (%08x)\n", (unsigned long)nmi625_i2c_client->addr);

	rmsg.addr = 0x61;
	rmsg.flags = I2C_M_RD;
	rmsg.len = 4;
	rmsg.buf = cmd1;
	printk("nmi625_i2c_client->adapter (%08x)\n", (unsigned long)nmi625_i2c_client->adapter);
	i2c_transfer(nmi625_i2c_client->adapter, &rmsg, 1);
	printk("Nmi 325 Chip Id (%02x)(%02x)(%02x)(%02x)\n", cmd1[0],cmd1[1],cmd1[2],cmd1[3]);
}

static int nmi625_i2c_remove(struct i2c_client *client)
{
	struct nmi625_state *nmi625 = i2c_get_clientdata(client);

	kfree(nmi625);
	return 0;
}

static int nmi625_i2c_probe(struct i2c_client *client,  const struct i2c_device_id *id)
{
	struct nmi625_state *nmi625;

	nmi625 = kzalloc(sizeof(struct nmi625_state), GFP_KERNEL);
	if (nmi625 == NULL) {		
		printk("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	nmi625->client = client;
	i2c_set_clientdata(client, nmi625);
	
	/* rest of the initialisation goes here. */
	
	printk("nmi625 attach success!!!\n");

	nmi625_i2c_client = client;

	nmi625_init = 1;
	
	return 0;
}


static const struct i2c_device_id nmi625_device_id[] = {
	{"nmi625", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, nmi625_device_id);


static struct i2c_driver nmi625_i2c_driver = {
	.driver = {
		.name = "nmi625",
		.owner = THIS_MODULE,
	},
	.probe	= nmi625_i2c_probe,
	.remove	= nmi625_i2c_remove,
	.id_table	= nmi625_device_id,
};


