/* Precise measurements of time delta between sending a trigger signal
 * to the HC-SRO4 distance sensor and receiving the echo signal from
 * the sensor back. This has to be precise in the usecs range. We
 * use trigger interrupts to measure the signal, so no busy wait :)
 *
 * This supports an (in theory) unlimited number of HC-SRO4 devices.
 * To add a device, do a (as root):
 *
 *	# echo 23 24 1000 > /sys/class/distance-sensor/configure
 *
 * (23 is the trigger GPIO, 24 is the echo GPIO and 1000 is a timeout in
 *  milliseconds)
 *
 * Then a directory appears with a file measure in it. To measure, do a
 *
 *	# cat /sys/class/distance-sensor/distance_23_24/measure
 *
 * You'll receive the length of the echo signal in usecs. To convert (roughly)
 * to centimeters multiply by 17150 and divide by 1e6.
 *
 * To deconfigure the device, do a
 *
 *	# echo -23 24 > /sys/class/distance-sensor/configure
 *
 * (normally not needed).
 *
 * DO NOT attach your HC-SRO4's echo pin directly to the raspberry, since
 * it runs with 5V while raspberry expects 3V on the GPIO inputs.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timekeeping.h>
#include <linux/gpio/consumer.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/gpio/driver.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/iio/sw_trigger.h>

struct hc_sro4 {
	int gpio_trig;
	int gpio_echo;
	struct gpio_desc *trig_desc;
	struct gpio_desc *echo_desc;
	struct timeval time_triggered;
	struct timeval time_echoed;
	int echo_received;
	int device_triggered;
	struct mutex measurement_mutex;
	wait_queue_head_t wait_for_echo;
	unsigned long timeout;
	struct iio_sw_trigger swt;
};

static DEFINE_MUTEX(devices_mutex);

static struct hc_sro4 *create_hc_sro4(int trig, int echo, unsigned long timeout)
		/* must be called with devices_mutex held */
{
	struct hc_sro4 *new;
	int err;

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (new == NULL)
		return ERR_PTR(-ENOMEM);

	new->gpio_echo = echo;
	new->gpio_trig = trig;
	new->echo_desc = gpio_to_desc(echo);
	if (new->echo_desc == NULL) {
		kfree(new);
		return ERR_PTR(-EINVAL);
	}
	new->trig_desc = gpio_to_desc(trig);
	if (new->trig_desc == NULL) {
		kfree(new);
		return ERR_PTR(-EINVAL);
	}

	err = gpiod_direction_input(new->echo_desc);
	if (err < 0) {
		kfree(new);
		return ERR_PTR(err);
	}
	err = gpiod_direction_output(new->trig_desc, 0);
	if (err < 0) {
		kfree(new);
		return ERR_PTR(err);
	}
	gpiod_set_value(new->trig_desc, 0);

	mutex_init(&new->measurement_mutex);
	init_waitqueue_head(&new->wait_for_echo);
	new->timeout = timeout;

	return new;
}

static irqreturn_t echo_received_irq(int irq, void *data)
{
	struct hc_sro4 *device = (struct hc_sro4 *) data;
	int val;
	struct timeval irq_tv;

	do_gettimeofday(&irq_tv);

	if (!device->device_triggered)
		return IRQ_HANDLED;
	if (device->echo_received)
		return IRQ_HANDLED;

	val = gpiod_get_value(device->echo_desc);
	if (val == 1) {
		device->time_triggered = irq_tv;
	} else {
		device->time_echoed = irq_tv;
		device->echo_received = 1;
		wake_up_interruptible(&device->wait_for_echo);
	}

	return IRQ_HANDLED;
}

/* devices_mutex must be held by caller, so nobody deletes the device
 * before we lock it.
 */

static int do_measurement(struct hc_sro4 *device,
			  unsigned long long *usecs_elapsed)
{
	long timeout;
	int irq;
	int ret;

	if (!mutex_trylock(&device->measurement_mutex)) {
		mutex_unlock(&devices_mutex);
		return -EBUSY;
	}
	mutex_unlock(&devices_mutex);

	msleep(60);
		/* wait 60 ms between measurements.
		 * now, a while true ; do cat measure ; done should work
		 */

	irq = gpiod_to_irq(device->echo_desc);
	if (irq < 0)
		return -EIO;

	device->echo_received = 0;
	device->device_triggered = 0;

	ret = request_any_context_irq(irq, echo_received_irq,
		IRQF_SHARED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		"hc_sro4", device);

	if (ret < 0)
		goto out_mutex;

	gpiod_set_value(device->trig_desc, 1);
	udelay(10);
	device->device_triggered = 1;
	gpiod_set_value(device->trig_desc, 0);

	ret = gpiochip_lock_as_irq(gpiod_to_chip(device->echo_desc),
				   device->gpio_echo);
	if (ret < 0)
		goto out_irq;

	timeout = wait_event_interruptible_timeout(device->wait_for_echo,
				device->echo_received, device->timeout);

	if (timeout == 0)
		ret = -ETIMEDOUT;
	else if (timeout < 0)
		ret = timeout;
	else {
		*usecs_elapsed =
	(device->time_echoed.tv_sec - device->time_triggered.tv_sec) * 1000000 +
	(device->time_echoed.tv_usec - device->time_triggered.tv_usec);
		ret = 0;
	}
/* TODO: unlock_as_irq */
out_irq:
	free_irq(irq, device);
out_mutex:
	mutex_unlock(&device->measurement_mutex);

	return ret;
}

static ssize_t sysfs_do_measurement(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct hc_sro4 *sensor = dev_get_drvdata(dev);
	unsigned long long usecs_elapsed;
	int status;

	mutex_lock(&devices_mutex);
	status = do_measurement(sensor, &usecs_elapsed);

	if (status < 0)
		return status;

	return sprintf(buf, "%lld\n", usecs_elapsed);
}

DEVICE_ATTR(measure, 0444, sysfs_do_measurement, NULL);


static struct attribute *sensor_attrs[] = {
	&dev_attr_measure.attr,
	NULL,
};

static const struct attribute_group sensor_group = {
	.attrs = sensor_attrs
};

static const struct attribute_group *sensor_groups[] = {
	&sensor_group,
	NULL
};

static ssize_t hc_sro4_echo_pin_store(struct config_item *item,
				const char *buf, size_t len)
{
	printk(KERN_INFO "echo pin is %s\n", buf);
	return len;
}

CONFIGFS_ATTR_WO(hc_sro4_, echo_pin);

static struct configfs_attribute *hc_sro4_config_attrs[] = {
	&hc_sro4_attr_echo_pin,
	NULL
};

static struct config_item_type iio_hc_sro4_type = {
        .ct_owner = THIS_MODULE,
	.ct_attrs = hc_sro4_config_attrs
};

#if 0
static struct class hc_sro4_class = {
	.name = "distance-sensor",
	.owner = THIS_MODULE,
	.class_attrs = hc_sro4_class_attrs
};


static int match_device(struct device *dev, const void *data)
{
	return dev_get_drvdata(dev) == data;
}

static int remove_sensor(struct hc_sro4 *rip_sensor)
	/* must be called with devices_mutex held. */
{
	struct device *dev;

	dev = class_find_device(&hc_sro4_class, NULL, rip_sensor, match_device);
	if (dev == NULL)
		return -ENODEV;

	mutex_lock(&rip_sensor->measurement_mutex);
			/* wait until measurement has finished */
	kfree(rip_sensor);

	device_unregister(dev);
	put_device(dev);

	return 0;
}


static ssize_t sysfs_configure_store(struct class *class,
				struct class_attribute *attr,
				const char *buf, size_t len)
{
	int add = buf[0] != '-';
	const char *s = buf;
	int trig, echo, timeout;
	struct hc_sro4 *new_sensor, *rip_sensor;
	int err;

	if (buf[0] == '-' || buf[0] == '+')
		s++;

	if (add) {
		if (sscanf(s, "%d %d %d", &trig, &echo, &timeout) != 3)
			return -EINVAL;

		mutex_lock(&devices_mutex);
		if (find_sensor(trig, echo)) {
			mutex_unlock(&devices_mutex);
			return -EEXIST;
		}

		new_sensor = create_hc_sro4(trig, echo, timeout);
		mutex_unlock(&devices_mutex);
		if (IS_ERR(new_sensor))
			return PTR_ERR(new_sensor);

		device_create_with_groups(class, NULL, MKDEV(0, 0), new_sensor,
				sensor_groups, "distance_%d_%d", trig, echo);
	} else {
		if (sscanf(s, "%d %d", &trig, &echo) != 2)
			return -EINVAL;

		mutex_lock(&devices_mutex);
		rip_sensor = find_sensor(trig, echo);
		if (rip_sensor == NULL) {
			mutex_unlock(&devices_mutex);
			return -ENODEV;
		}
		err = remove_sensor(rip_sensor);
		mutex_unlock(&devices_mutex);
		if (err < 0)
			return err;
	}
	return len;
}

#endif

static int iio_trig_hc_sro4_set_state(struct iio_trigger *trig, bool state)
{
        struct iio_hrtimer_info *trig_info;

        trig_info = iio_trigger_get_drvdata(trig);

        if (state)
		printk(KERN_INFO "starting HC_SRO4\n");
        else
		printk(KERN_INFO "stopping HC_SRO4\n");

        return 0;
}


static const struct iio_trigger_ops iio_hc_sro4_trigger_ops = {
        .owner = THIS_MODULE,
        .set_trigger_state = iio_trig_hc_sro4_set_state,
};

static struct iio_sw_trigger *iio_trig_hc_sro4_probe(const char *name)
{
	struct hc_sro4 *hc_sro4;
	int ret;

printk(KERN_INFO "sensor add name = %s\n", name);

	hc_sro4 = create_hc_sro4(23, 24, 3000);  
		/* TODO: make this configurable via configfs */
	if (IS_ERR(hc_sro4))
		return ERR_PTR(PTR_ERR(hc_sro4));

printk(KERN_INFO "create sensor ok\n");

	hc_sro4->swt.trigger = iio_trigger_alloc("%s", name);
	if (!hc_sro4->swt.trigger) {
		ret = -ENOMEM;
		goto err_free_sensor;
	}
	iio_trigger_set_drvdata(hc_sro4->swt.trigger, hc_sro4);
	hc_sro4->swt.trigger->ops = &iio_hc_sro4_trigger_ops;
	hc_sro4->swt.trigger->dev.groups = sensor_groups;

	ret = iio_trigger_register(hc_sro4->swt.trigger);
	if (ret)
		goto err_free_trigger;

/*	kfree(hc_sro4);
	return ERR_PTR(-ENOMEM); */

printk(KERN_INFO "sensor before group init type name name = %s\n", name);
/* printk(KERN_INFO "group: %p\n", hc_sro4->swt.group); */
        iio_swt_group_init_type_name(&hc_sro4->swt, name, &iio_hc_sro4_type);

printk(KERN_INFO "sensor add ok\n");
	return &hc_sro4->swt;

err_free_trigger:
	iio_trigger_free(hc_sro4->swt.trigger);
err_free_sensor:
	kfree(hc_sro4);

	return ERR_PTR(ret);
}

static int iio_trig_hc_sro4_remove(struct iio_sw_trigger *swt)
{
	struct hc_sro4 *rip_sensor;

printk(KERN_INFO "sensor remove\n");
	rip_sensor = iio_trigger_get_drvdata(swt->trigger);
	
	iio_trigger_unregister(swt->trigger);

/* TODO: cancel measurement, wait for measurement to be finished */
	iio_trigger_free(swt->trigger);
	kfree(rip_sensor);

	return 0;
}

static const struct iio_sw_trigger_ops iio_trig_hc_sro4_ops = {
        .probe          = iio_trig_hc_sro4_probe,
        .remove         = iio_trig_hc_sro4_remove,
};

static struct iio_sw_trigger_type iio_trig_hc_sro4 = {
        .name = "hc-sro4",
        .owner = THIS_MODULE,
        .ops = &iio_trig_hc_sro4_ops,
};

module_iio_sw_trigger_driver(iio_trig_hc_sro4);

MODULE_AUTHOR("Johannes Thoma");
MODULE_DESCRIPTION("Distance measurement for the HC-SRO4 ultrasonic distance sensor");
MODULE_LICENSE("GPL");

