// SPDX-License-Identifier: GPL-2.0-only
/*
 * amd_w1 - AMD 1Wire bus master driver
 *
 * Copyright (C) 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
 */

#include <linux/atomic.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>

#include <linux/w1.h>

/* 1-wire AMD IP definition */
#define AXIW1_IPID	0x10ee4453
/* Registers offset */
#define AXIW1_INST_REG	0x0
#define AXIW1_CTRL_REG	0x4
#define AXIW1_IRQE_REG	0x8
#define AXIW1_STAT_REG	0xC
#define AXIW1_DATA_REG	0x10
#define AXIW1_IPVER_REG	0x18
#define AXIW1_IPID_REG	0x1C
/* Instructions */
#define AXIW1_READBIT	0x00000C00
#define AXIW1_WRITEBIT	0x00000E00
#define AXIW1_READBYTE	0x00000D00
#define AXIW1_WRITEBYTE	0x00000F00
#define AXIW1_INITPRES	0x00000800
/* Status flag masks */
#define AXIW1_DONE	BIT(0)
#define AXIW1_READDATA	BIT(0)
#define AXIW1_READY	BIT(4)
#define AXIW1_PRESENCE	BIT(31)
#define AXIW1_MAJORVER_MASK	GENMASK(23, 8)
#define AXIW1_MAJORVER_SHIFT	8
#define AXIW1_MINORVER_MASK	GENMASK(7, 0)
/* Control flag */
#define AXIW1_GO	BIT(0)
#define AXI_CLEAR	0
#define AXI_RESET	BIT(31)
/* Interrupt Enable */
#define AXIW1_READY_IRQ_EN	BIT(4)
#define AXIW1_DONE_IRQ_EN	BIT(0)

#define AXIW1_TIMEOUT	msecs_to_jiffies(100)

#define DRIVER_NAME "amd_w1"

struct amd_w1_local {
	u32 ver_major;
	u32 ver_minor;
	struct device *dev;
	int irq;
	void __iomem *base_addr;
	wait_queue_head_t wait_queue;
	atomic_t flag;
};

/* Functions to write and read the W1 IP registers */
static inline void amd_w1_write_register(struct amd_w1_local *amd_w1_local,
					 u8 reg_offset, u32 val)
{
	iowrite32(val, amd_w1_local->base_addr + reg_offset);
};

static inline u32 amd_w1_read_register(struct amd_w1_local *amd_w1_local, u8 reg_offset)
{
	return ioread32(amd_w1_local->base_addr + reg_offset);
};

/* Wait for IRQ with timeout */
static inline int amd_w1_wait_irq_interruptible_timeout(struct amd_w1_local *amd_w1_local, u32 IRQ)
{
	int ret;

	/* Enable the IRQ requested and wait for flag to indicate it's been triggered */
	amd_w1_write_register(amd_w1_local, AXIW1_IRQE_REG, IRQ);
	ret = wait_event_interruptible_timeout(amd_w1_local->wait_queue,
					       atomic_read(&amd_w1_local->flag) != 0,
					       AXIW1_TIMEOUT);
	if (ret < 0) {
		dev_err(amd_w1_local->dev, "Wait IRQ Interrupted\n");
		return -EINTR;
	} else if (ret == 0) {
		dev_err(amd_w1_local->dev, "Wait IRQ Timeout\n");
		return -EBUSY;
	}

	/* Clear flag */
	atomic_set(&amd_w1_local->flag, 0);
	return 0;
}

static u8 amd_w1_touch_bit(void *data, u8 bit)
{
	struct amd_w1_local *amd_w1_local = data;
	u8 val = 0;

	/* Wait for READY signal to be 1 to ensure 1-wire IP is ready */
	while ((amd_w1_read_register(amd_w1_local, AXIW1_STAT_REG) & AXIW1_READY) == 0)
		amd_w1_wait_irq_interruptible_timeout(amd_w1_local, AXIW1_READY_IRQ_EN);

	if (bit)
		/* Read. Write read Bit command in register 0 */
		amd_w1_write_register(amd_w1_local, AXIW1_INST_REG, AXIW1_READBIT);

	else
		/* Write. Write tx Bit command in instruction register with bit to transmit */
		amd_w1_write_register(amd_w1_local, AXIW1_INST_REG,
				      (AXIW1_WRITEBIT + (bit & 0x01)));

	/* Write Go signal and clear control reset signal in control register */
	amd_w1_write_register(amd_w1_local, AXIW1_CTRL_REG, AXIW1_GO);

	/* Wait for done signal to be 1 */
	while ((amd_w1_read_register(amd_w1_local, AXIW1_STAT_REG) & AXIW1_DONE) != 1)
		amd_w1_wait_irq_interruptible_timeout(amd_w1_local, AXIW1_DONE_IRQ_EN);

	if (bit) /* If read, Retrieve data from register */
		val = (u8)(amd_w1_read_register(amd_w1_local, AXIW1_DATA_REG) & AXIW1_READDATA);

	/* Clear Go signal in register 1 */
	amd_w1_write_register(amd_w1_local, AXIW1_CTRL_REG, AXI_CLEAR);

	return val;
}

static u8 amd_w1_read_byte(void *data)
{
	struct amd_w1_local *amd_w1_local = data;
	u8 val = 0;

	/* Wait for READY signal to be 1 to ensure 1-wire IP is ready */
	while ((amd_w1_read_register(amd_w1_local, AXIW1_STAT_REG) & AXIW1_READY) == 0)
		amd_w1_wait_irq_interruptible_timeout(amd_w1_local, AXIW1_READY_IRQ_EN);

	/* Write read Byte command in instruction register*/
	amd_w1_write_register(amd_w1_local, AXIW1_INST_REG, AXIW1_READBYTE);
	/* Write Go signal and clear control reset signal in control register */
	amd_w1_write_register(amd_w1_local, AXIW1_CTRL_REG, AXIW1_GO);

	/* Wait for done signal to be 1 */
	while ((amd_w1_read_register(amd_w1_local, AXIW1_STAT_REG) & AXIW1_DONE) != 1)
		amd_w1_wait_irq_interruptible_timeout(amd_w1_local, AXIW1_DONE_IRQ_EN);

	/* Retrieve LSB bit in data register to get RX byte */
	val = (u8)(amd_w1_read_register(amd_w1_local, AXIW1_DATA_REG) & 0x000000FF);

	/* Clear Go signal in control register */
	amd_w1_write_register(amd_w1_local, AXIW1_CTRL_REG, AXI_CLEAR);

	return val;
}

static void amd_w1_write_byte(void *data, u8 val)
{
	struct amd_w1_local *amd_w1_local = data;

	/* Wait for READY signal to be 1 to ensure 1-wire IP is ready */
	while ((amd_w1_read_register(amd_w1_local, AXIW1_STAT_REG) & AXIW1_READY) == 0)
		amd_w1_wait_irq_interruptible_timeout(amd_w1_local, AXIW1_READY_IRQ_EN);

	/* Write tx Byte command in instruction register with bit to transmit */
	amd_w1_write_register(amd_w1_local, AXIW1_INST_REG, (AXIW1_WRITEBYTE + val));
	/* Write Go signal and clear control reset signal in register 1 */
	amd_w1_write_register(amd_w1_local, AXIW1_CTRL_REG, AXIW1_GO);

	/* Wait for done signal to be 1 */
	while ((amd_w1_read_register(amd_w1_local, AXIW1_STAT_REG) & AXIW1_DONE) != 1)
		amd_w1_wait_irq_interruptible_timeout(amd_w1_local, AXIW1_DONE_IRQ_EN);

	/* Clear Go signal in control register */
	amd_w1_write_register(amd_w1_local, AXIW1_CTRL_REG, AXI_CLEAR);
}

static u8 amd_w1_reset_bus(void *data)
{
	struct amd_w1_local *amd_w1_local = data;
	u8 val = 0;

	/* Reset 1-wire Axi IP */
	amd_w1_write_register(amd_w1_local, AXIW1_CTRL_REG, AXI_RESET);

	/* Wait for READY signal to be 1 to ensure 1-wire IP is ready */
	while ((amd_w1_read_register(amd_w1_local, AXIW1_STAT_REG) & AXIW1_READY) == 0)
		amd_w1_wait_irq_interruptible_timeout(amd_w1_local, AXIW1_READY_IRQ_EN);

	/* Write Initialization command in instruction register */
	amd_w1_write_register(amd_w1_local, AXIW1_INST_REG, AXIW1_INITPRES);
	/* Write Go signal and clear control reset signal in register 1 */
	amd_w1_write_register(amd_w1_local, AXIW1_CTRL_REG, AXIW1_GO);

	/* Wait for done signal to be 1 */
	while ((amd_w1_read_register(amd_w1_local, AXIW1_STAT_REG) & AXIW1_DONE) != 1)
		amd_w1_wait_irq_interruptible_timeout(amd_w1_local, AXIW1_DONE_IRQ_EN);

	/* Retrieve MSB bit in status register to get failure bit */
	if ((amd_w1_read_register(amd_w1_local, AXIW1_STAT_REG) & AXIW1_PRESENCE) != 0)
		val = 1;

	/* Clear Go signal in control register */
	amd_w1_write_register(amd_w1_local, AXIW1_CTRL_REG, AXI_CLEAR);

	return val;
}

/* 1-wire master structure */
static struct w1_bus_master amd_w1_master = {
	.touch_bit	= amd_w1_touch_bit,
	.read_byte	= amd_w1_read_byte,
	.write_byte	= amd_w1_write_byte,
	.reset_bus	= amd_w1_reset_bus,
};

/* Reset the 1-wire AXI IP. Put the IP in reset state and clear registers */
static void amd_w1_reset(struct amd_w1_local *amd_w1_local)
{
	amd_w1_write_register(amd_w1_local, AXIW1_CTRL_REG, AXI_RESET);
	amd_w1_write_register(amd_w1_local, AXIW1_INST_REG, AXI_CLEAR);
	amd_w1_write_register(amd_w1_local, AXIW1_IRQE_REG, AXI_CLEAR);
	amd_w1_write_register(amd_w1_local, AXIW1_STAT_REG, AXI_CLEAR);
	amd_w1_write_register(amd_w1_local, AXIW1_DATA_REG, AXI_CLEAR);
}

static irqreturn_t amd_w1_irq(int irq, void *lp)
{
	struct amd_w1_local *amd_w1_local = lp;

	/* Clear enables in IRQ enable register */
	amd_w1_write_register(amd_w1_local, AXIW1_IRQE_REG, AXI_CLEAR);
	/* Wake up the waiting queue */
	atomic_set(&amd_w1_local->flag, 1);
	wake_up_interruptible(&amd_w1_local->wait_queue);

	return IRQ_HANDLED;
}

static int amd_w1_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct amd_w1_local *lp;
	int rc = 0;
	int val;

	/* Get iospace for the device */
	lp = devm_kzalloc(dev, sizeof(*lp), GFP_KERNEL);
	if (!lp)
		return -ENOMEM;

	lp->dev = dev;
	dev_set_drvdata(dev, lp);

	lp->base_addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lp->base_addr))
		return PTR_ERR(lp->base_addr);

	/* Get IRQ for the device */
	lp->irq = platform_get_irq(pdev, 0);
	if (lp->irq <= 0)
		return lp->irq;

	/* Initialize wait queue and flag */
	init_waitqueue_head(&lp->wait_queue);

	rc = devm_request_irq(dev, lp->irq, &amd_w1_irq, IRQF_TRIGGER_HIGH, DRIVER_NAME, lp);
	if (rc) {
		dev_err(dev, "%s: Could not allocate interrupt %d.\n", __func__,
			lp->irq);
		return rc;
	}

	/* Verify IP presence in HW */
	if (amd_w1_read_register(lp, AXIW1_IPID_REG) != AXIW1_IPID) {
		dev_err(dev, "%s: AMD 1-wire IP not detected in hardware\n", __func__);
		return rc;
	}

	val = amd_w1_read_register(lp, AXIW1_IPVER_REG);

	/* Allow for future driver expansion supporting new hardware features
	 * This driver currently only supports hardware 1.x, but include logic
	 * to detect if a potentially incompatible future version is used
	 * by reading major version ID.  It is highly undesirable for new IP versions
	 * to break the API, but this code will at least allow for graceful failure
	 * should that happen.  Future new features can be enabled by hardware
	 * incrementing the minor version and augmenting the driver to detect capability
	 * using the minor version number
	 */

	lp->ver_major = (val & AXIW1_MAJORVER_MASK) >> AXIW1_MAJORVER_SHIFT;
	lp->ver_minor = (val & AXIW1_MINORVER_MASK);

	if (lp->ver_major != 1)	{
		dev_err(dev, "AMD AXI W1 Master version %u.%u is not supported by this driver",
			lp->ver_major, lp->ver_minor);
		return -ENODEV;
	}

	dev_info(dev, "AMD AXI W1 Master version %u.%u detected", lp->ver_major, lp->ver_minor);

	amd_w1_master.data = (void *)lp;
	amd_w1_reset(lp);
	rc = w1_add_master_device(&amd_w1_master);
	if (rc) {
		dev_err(dev, "%s: Could not add master device\n", __func__);
		amd_w1_reset(lp);
	}

	return 0;
}

static void amd_w1_remove(struct platform_device *pdev)
{
	w1_remove_master_device(&amd_w1_master);
}

static const struct of_device_id amd_w1_of_match[] = {
	{ .compatible = "amd,axi-1wire-master" },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, amd_w1_of_match);

static struct platform_driver amd_w1_driver = {
	.probe		= amd_w1_probe,
	.remove_new	= amd_w1_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = amd_w1_of_match,
	},

};
module_platform_driver(amd_w1_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Thomas Delev <thomas.delev@amd.com>");
MODULE_DESCRIPTION("Driver for AMD AXI 1 Wire  IP core");
