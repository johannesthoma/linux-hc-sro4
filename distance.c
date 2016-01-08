/* Precise measurements of time delta between sending a trigger signal
   to the HC-SRO4 distance sensor and receiving the echo signal from
   the sensor back. This has to be precise in the us range. We (for now)
   do this with local_irq_disable and polling (with timeout). 

   DO NOT attach your HC-SRO4's echo pin directly to the raspberry, since
   it runs with 5V while raspberry expects 3V on the GPIO inputs.

*/

/* TODO: check this for measure / delete races */
/* TODO: sysfs interface */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timekeeping.h>

struct hc_sro4 {
	int gpio_echo;
	int gpio_trig;
	struct timeval time_triggered;
	volatile struct timeval time_echoed;
	mutex_t measurement_mutex;
	wait_queue_head_t wait_for_echo;
	long timeout;
	struct list_head list;
};


static LIST_HEAD hc_sro4_devices;

static int create_hc_sro4(int echo, int trig)
{
	struct hc_sro4 *new;

	new = kmalloc(sizeof(*new));
	if (new == NULL) {
		return -ENOMEM;
	}

	new->gpio_echo = echo;
	new->gpio_trig = trig;
	init_mutex(&new->measurement_mutex);
	init_waitqueue_head(&new->wait_for_echo);
	new->timeout = 10;   /* TODO: make configurable */

	list_add_tail(&new->list, &hc_sro4_devices);

	return 0;
}

static int destroy_hc_sro4(struct hc_sro4 *device)
{
	mutex_lock(&device->measurement_lock);
	list_del(&device->list);
	kfree(device);

	return 0;
}

static irqreturn_t echo_received_irq(int irq, void *data)
{
	struct hc_sro4 *device = (struct hc_sro4*) data;
	
	do_gettimeofday(&device->time_echoed);	
	wake_up(&device->wait_for_echo);
	
	return IRQ_HANDLED;
}

static int do_measurement(struct hc_sro4 *device)
{
	long timeout;

	if (!mutex_trylock(&device->measurement_mutex)) {
		return -EBUSY;
	}
/* TODO: register interrupt */

/* TODO: assert trig signal for 10usecs */
	udelay(10);

	do_gettimeofday(&device->time_triggered);	

	wait_event_interruptible_timeout(&device->wait_for_echo, 1, &device->timeout);

/* TODO: unregister interrupt */

	mutex_unlock(&device->measurement_mutex);

	return 0;
}

static int __init init_hc_sro4(void)
{
	printk(KERN_INFO "Hello world\n");

	return create_hc_sro4(23, 24);	/* TODO: dynamic */
}

module_init(init_hc_sro4);
	
MODULE_AUTHOR("Johannes Thoma");
MODULE_DESCRIPTION("Distance measurement for the HC-SRO4 ultrasonic distance sensor for the raspberry pi");
MODULE_LICENSE("GPL");

