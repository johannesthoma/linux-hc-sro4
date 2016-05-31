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

#define DEFAULT_TIMEOUT 1000

struct hc_sro4 {
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
	struct timeval last_measurement;
};

static inline struct hc_sro4 *to_hc_sro4(struct config_item *item)
{
	struct iio_sw_trigger *trig = to_iio_sw_trigger(item);
	return container_of(trig, struct hc_sro4, swt);
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

static int do_measurement(struct hc_sro4 *device,
			  long long *usecs_elapsed)
{
	long timeout;
	int irq;
	int ret;
	struct timeval now;
	long long time_since_last_measurement;

	*usecs_elapsed = -1;

	if (!device->echo_desc || !device->trig_desc) {
		dev_dbg(&device->swt.trigger->dev, "Please configure GPIO pins first.\n");
		return -EINVAL;
	}
	if (!mutex_trylock(&device->measurement_mutex)) {
		return -EBUSY;
	}

	do_gettimeofday(&now);
	if (device->last_measurement.tv_sec || device->last_measurement.tv_usec)
		time_since_last_measurement =
	(now.tv_sec - device->last_measurement.tv_sec) * 1000000 +
	(now.tv_usec - device->last_measurement.tv_usec);
	else
		time_since_last_measurement = 60000;
	
	msleep(max(60 - time_since_last_measurement / 1000, (long long) 0));
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
				   desc_to_gpio(device->echo_desc));
	if (ret < 0)
		goto out_irq;

	timeout = wait_event_interruptible_timeout(device->wait_for_echo,
			device->echo_received, device->timeout * HZ / 1000);

	if (timeout == 0)
		ret = -ETIMEDOUT;
	else if (timeout < 0)
		ret = timeout;
	else {
		*usecs_elapsed =
	(device->time_echoed.tv_sec - device->time_triggered.tv_sec) * 1000000 +
	(device->time_echoed.tv_usec - device->time_triggered.tv_usec);
		ret = 0;
		do_gettimeofday(&device->last_measurement);
	}
	gpiochip_unlock_as_irq(gpiod_to_chip(device->echo_desc),
		               desc_to_gpio(device->echo_desc));
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
	long long usecs_elapsed;
	int status;

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

static ssize_t configure_pin(struct gpio_desc **desc, struct config_item *item,
			const char *buf, size_t len, enum gpiod_flags flags,
			struct device *dev)
{
	int err;
	int echo;

	if (*desc) {
		gpiod_put(*desc);
	}

	*desc = gpiod_get(dev, buf, flags);
	if (IS_ERR(*desc)) {
		err = PTR_ERR(*desc);
		*desc = NULL;

		if (err == -ENOENT) {	/* fallback: use GPIO numbers */
			if (sscanf(buf, "%d", &echo) != 1)
				return -ENOENT;
			*desc = gpio_to_desc(echo);
			if (*desc)
				return len;
			return -ENOENT;
		}
			
		return err;
	}
	return len;
}

static ssize_t hc_sro4_echo_pin_store(struct config_item *item,
				const char *buf, size_t len)
{
	struct hc_sro4 *sensor = to_hc_sro4(item);
	ssize_t ret;
	int err;

	ret = configure_pin(&sensor->echo_desc, item, buf, len, GPIOD_IN, 
	                    &sensor->swt.trigger->dev);

	if (ret >= 0 && sensor->echo_desc) {
	        err = gpiod_direction_input(sensor->echo_desc);
		if (err < 0)
			return err;
	}
	return ret;
}

static ssize_t hc_sro4_echo_pin_show(struct config_item *item,
				     char *buf)
{
	struct hc_sro4 *sensor = to_hc_sro4(item);
	if (sensor->echo_desc)
		return sprintf(buf, "%d\n", desc_to_gpio(sensor->echo_desc));
	return 0;
}

static ssize_t hc_sro4_trig_pin_store(struct config_item *item,
				const char *buf, size_t len)
{
	struct hc_sro4 *sensor = to_hc_sro4(item);
	ssize_t ret;
	int err;

	ret = configure_pin(&sensor->trig_desc, item, buf, len, GPIOD_OUT_LOW,
			    &sensor->swt.trigger->dev);

	if (ret >= 0 && sensor->trig_desc) {
	        err = gpiod_direction_output(sensor->trig_desc, 0);
		if (err >= 0)
			gpiod_set_value(sensor->trig_desc, 0);
		else
			return err;
	}
	return ret;
}


 
static ssize_t hc_sro4_trig_pin_show(struct config_item *item,
                                     char *buf)
{
        struct hc_sro4 *sensor = to_hc_sro4(item);
	if (sensor->trig_desc)
		return sprintf(buf, "%d\n", desc_to_gpio(sensor->trig_desc));
	return 0;
}

static ssize_t hc_sro4_timeout_store(struct config_item *item,
				const char *buf, size_t len)
{
	struct hc_sro4 *sensor = to_hc_sro4(item);
	unsigned long t;

	if (sscanf(buf, "%ld", &t) != 1)
		return -EINVAL;
	sensor->timeout = t;
	return len;
}


static ssize_t hc_sro4_timeout_show(struct config_item *item,
			            char *buf)
{
	struct hc_sro4 *sensor = to_hc_sro4(item);
	return sprintf(buf, "%ld\n", sensor->timeout);
}


static ssize_t hc_sro4_dev_name_show(struct config_item *item,
			             char *buf)
{
	struct hc_sro4 *sensor = to_hc_sro4(item);
	return sprintf(buf, "%s", dev_name(&sensor->swt.trigger->dev));
}


CONFIGFS_ATTR(hc_sro4_, echo_pin);
CONFIGFS_ATTR(hc_sro4_, trig_pin);
CONFIGFS_ATTR(hc_sro4_, timeout);
CONFIGFS_ATTR_RO(hc_sro4_, dev_name);

static struct configfs_attribute *hc_sro4_config_attrs[] = {
	&hc_sro4_attr_echo_pin,
	&hc_sro4_attr_trig_pin,
	&hc_sro4_attr_timeout,
	&hc_sro4_attr_dev_name,
	NULL
};

static struct config_item_type iio_hc_sro4_type = {
        .ct_owner = THIS_MODULE,
	.ct_attrs = hc_sro4_config_attrs
};

static int iio_trig_hc_sro4_set_state(struct iio_trigger *trig, bool state)
{
        struct iio_hrtimer_info *trig_info;

        trig_info = iio_trigger_get_drvdata(trig);

/* TODO: when is this function called? Powersafe? */
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
	struct hc_sro4 *sensor;
	int ret;

	sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return ERR_PTR(-ENOMEM);

	mutex_init(&sensor->measurement_mutex);
	init_waitqueue_head(&sensor->wait_for_echo);
	sensor->timeout = DEFAULT_TIMEOUT;

	sensor->swt.trigger = iio_trigger_alloc("%s", name);
	if (!sensor->swt.trigger) {
		ret = -ENOMEM;
		goto err_free_sensor;
	}
	iio_trigger_set_drvdata(sensor->swt.trigger, sensor);
	sensor->swt.trigger->ops = &iio_hc_sro4_trigger_ops;
	sensor->swt.trigger->dev.groups = sensor_groups;

	ret = iio_trigger_register(sensor->swt.trigger);
	if (ret)
		goto err_free_trigger;

        iio_swt_group_init_type_name(&sensor->swt, name, &iio_hc_sro4_type);
	return &sensor->swt;

err_free_trigger:
	iio_trigger_free(sensor->swt.trigger);
err_free_sensor:
	kfree(sensor);

	return ERR_PTR(ret);
}

static int iio_trig_hc_sro4_remove(struct iio_sw_trigger *swt)
{
	struct hc_sro4 *rip_sensor;

	rip_sensor = iio_trigger_get_drvdata(swt->trigger);
	
	iio_trigger_unregister(swt->trigger);

	/* Wait for measurement to be finished. */
	mutex_lock(&rip_sensor->measurement_mutex);

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

