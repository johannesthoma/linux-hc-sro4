/* Precise measurements of time delta between sending a trigger signal
   to the HC-SRO4 distance sensor and receiving the echo signal from
   the sensor back. This has to be precise in the us range. We (for now)
   do this with local_irq_disable and polling (with timeout). 

   DO NOT attach your HC-SRO4's echo pin directly to the raspberry, since
   it runs with 5V while raspberry expects 3V on the GPIO inputs.

*/

#include <linux/kernel.h>
#include <linux/module.h>

static int __init init_hc_sro4(void)
{
	printk(KERN_INFO "Hello world\n");

	return 0;
}

postcore_initcall(init_hc_sro4);
	
MODULE_AUTHOR("Johannes Thoma");
MODULE_DESCRIPTION("Distance measurement for the HC-SRO4 ultrasonic distance sensor for the raspberry pi");
MODULE_LICENSE("GPL");

