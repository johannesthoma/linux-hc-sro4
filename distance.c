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
#include <linux/gpio/consumer.h>

struct hc_sro4 {
	int gpio_echo;
	int gpio_trig;
	struct gpio_desc *echo_desc;
	struct gpio_desc *trig_desc;
	struct timeval time_triggered;
	volatile struct timeval time_echoed;
	mutex_t measurement_mutex;
	wait_queue_head_t wait_for_echo;
	long timeout;
	struct list_head list;
};


static LIST_HEAD hc_sro4_devices;
static DECLARE_MUTEX(devices_mutex);

static int create_hc_sro4(int echo, int trig)
{
	struct hc_sro4 *new;
	int err;

	new = kmalloc(sizeof(*new));
	if (new == NULL) {
		return -ENOMEM;
	}

	new->gpio_echo = echo;
	new->gpio_trig = trig;
	new->echo_desc = gpio_to_desc(echo);
	if (new->echo_desc == NULL) {
		kfree(new);
		return -EINVAL;
	}
	new->trig_desc = gpio_to_desc(trig);
	if (new->trig_desc == NULL) {
		kfree(new);
		return -EINVAL;
	}

	err = gpiod_direction_input(new->echo_desc);
	if (err < 0) {
		kfree(new);
		return err;
	}
	err = gpiod_direction_output(new->trig_desc, 0);
	if (err < 0) {
		kfree(new);
		return err;
	}

	init_mutex(&new->measurement_mutex);
	init_waitqueue_head(&new->wait_for_echo);
	new->timeout = 10;   /* TODO: make configurable */

	mutex_lock(&devices_mutex);
	list_add_tail(&new->list, &hc_sro4_devices);
	mutex_unlock(&devices_mutex);

	return 0;
}

static int destroy_hc_sro4(struct hc_sro4 *device)
{
	mutex_lock(&devices_mutex);
	mutex_lock(&device->measurement_lock);
	list_del(&device->list);
	mutex_unlock(&devices_mutex);
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

static int do_measurement(struct hc_sro4 *device, unsigned long long *usecs_elapsed)
{
	long timeout;
	int irq;
	int err;
	int timeout;

/* ?? bledsinn must be held by caller .. */
	mutex_lock(&devices_mutex);
	if (!mutex_trylock(&device->measurement_mutex)) {
		return -EBUSY;
	}
	mutex_unlock(&devices_mutex);

/* TODO: if ACTIVE_LOW */

        ret = request_any_context_irq(irq, echo_received_irq, IRQF_SHARED | IRQF_TRIGGER_FALLING, "hc_sro4", device);
	if (ret < 0) 
		goto out_mutex;

        ret = gpiochip_lock_as_irq(desc->chip, gpio_chip_hwgpio(desc));
	if (ret < 0) 
		goto out_irq;

	gpiod_set_value(device->trig_desc, 1);
	udelay(10);
	gpiod_set_value(device->trig_desc, 0);	 /* TODO: ?? or leave it until echo reveiced.. */

	do_gettimeofday(&device->time_triggered);

	timeout = wait_event_interruptible_timeout(&device->wait_for_echo, 1, &device->timeout);
	if (timeout == 0)
		ret = -ETIMEDOUT;
	else if (timeout < 0)
		ret = timeout;
	else
		*usecs_elapsed = 
			(device->time_echoed.tv_sec - device->time_triggered.tv_sec) * 1000000 + 
			(device->time_echoed.tv_usec - device->time_triggered.tv_usec);

out_irq:
	free_irq(irq, device);
out_mutex:
	mutex_unlock(&device->measurement_mutex);

	return ret;
}

static int __init init_hc_sro4(void)
{
	printk(KERN_INFO "Hello world\n");

	return create_hc_sro4(23, 24);	/* TODO: dynamic via sysfs */
}

module_init(init_hc_sro4);
	
MODULE_AUTHOR("Johannes Thoma");
MODULE_DESCRIPTION("Distance measurement for the HC-SRO4 ultrasonic distance sensor for the raspberry pi");
MODULE_LICENSE("GPL");

