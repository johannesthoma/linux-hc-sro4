/* Precise measurements of time delta between sending a trigger signal
   to the HC-SRO4 distance sensor and receiving the echo signal from
   the sensor back. This has to be precise in the us range. We (for now)
   do this with local_irq_disable and polling (with timeout). 

   DO NOT attach your HC-SRO4's echo pin directly to the raspberry, since
   it runs with 5V while raspberry expects 3V on the GPIO inputs.

*/

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/types.h>


/*
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
*/

static int probe_hc_sro4(struct platform_device *pdev)
{
	printk(KERN_INFO "Hello world\n");

	return 0;
}

static int remove_hc_sro4(struct platform_device *pdev)
{
	printk(KERN_INFO "Hello world\n");

	return 0;
}

static struct platform_driver hc_sro4_device_driver = {
	.probe 		= probe_hc_sro4,
	.remove		= remove_hc_sro4,
	.driver	= {
		.name	= "HC_SRO4",
	},
};
 
module_platform_driver(hc_sro4_device_driver);
	
MODULE_AUTHOR("Johannes Thoma");
MODULE_DESCRIPTION("Distance measurement for the HC-SRO4 ultrasonic distance sensor for the raspberry pi");
MODULE_LICENSE("GPL");

