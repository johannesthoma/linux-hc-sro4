/*
 * hc-sr04.c - Support for HC-SR04 ultrasonic range sensor
 *
 * Copyright (C) 2016 Johannes Thoma <johannes@johannesthoma.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/* Precise measurements of time delta between sending a trigger signal
 * to the HC-SR04 distance sensor and receiving the echo signal from
 * the sensor back. This has to be precise in the usecs range. We
 * use trigger interrupts to measure the signal, so no busy wait :)
 *
 * This supports an (in theory) unlimited number of HC-SR04 devices.
 * It uses IIO software triggers to interface with userland.
 *
 * To configure a device do a
 *
 *    mkdir /config/iio/triggers/hc-sr04/sensor0
 *
 * (you need to mount configfs to /config first)
 *
 * Then configure the ECHO and TRIG pins (this also accepts symbolic names
 * configured in the device tree)
 *
 *    echo 23 > /config/iio/triggers/hc-sr04/sensor0/trig_pin
 *    echo 24 > /config/iio/triggers/hc-sr04/sensor0/echo_pin
 *
 * Then you can measure distance with:
 *
 *    cat /sys/devices/trigger0/measure
 *
 * (trigger0 is the device name as reported by
 *  /config/iio/triggers/hc-sr04/sensor0/dev_name
 *
 * To convert to centimeters, multiply by 17150 and divide by 1000000 (air)
 *
 * DO NOT attach your HC-SR04's echo pin directly to the raspberry, since
 * it runs with 5V while raspberry expects 3V on the GPIO inputs.
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

enum hc_sr04_state {
	DEVICE_IDLE,
	DEVICE_TRIGGERED,
	DEVICE_ECHO_RECEIVED
};

struct hc_sr04 {
		/* the GPIOs of ECHO and TRIG */
	struct gpio_desc *trig_desc;
	struct gpio_desc *echo_desc;
		/* Used to measure length of ECHO signal */
	struct timeval time_triggered;
	struct timeval time_echoed;
		/* protects against starting multiple measurements */
	struct mutex measurement_mutex;
		/* Current state of measurement */
	enum hc_sr04_state state;
		/* Used by interrupt to wake measurement routine up */
	wait_queue_head_t wait_for_echo;
		/* timeout in ms, fail when no echo received within that time */
	unsigned long timeout;
		/* Our IIO interface */
	struct iio_sw_trigger swt;
		/* Used to compute device settle time */
	struct timeval last_measurement;
};

static inline struct hc_sr04 *to_hc_sr04(struct config_item *item)
{
	struct iio_sw_trigger *trig = to_iio_sw_trigger(item);

	return container_of(trig, struct hc_sr04, swt);
}

static irqreturn_t echo_received_irq(int irq, void *data)
{
	struct hc_sr04 *device = (struct hc_sr04 *)data;
	int val;
	struct timeval irq_tv;

	do_gettimeofday(&irq_tv);

	if (device->state != DEVICE_TRIGGERED)
		return IRQ_HANDLED;

	val = gpiod_get_value(device->echo_desc);
	if (val == 1) {
		device->time_triggered = irq_tv;
	} else {
		device->time_echoed = irq_tv;
		device->state = DEVICE_ECHO_RECEIVED;
		wake_up_interruptible(&device->wait_for_echo);
	}

	return IRQ_HANDLED;
}

static int do_measurement(struct hc_sr04 *device,
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
	if (!mutex_trylock(&device->measurement_mutex))
		return -EBUSY;

	do_gettimeofday(&now);
	if (device->last_measurement.tv_sec || device->last_measurement.tv_usec)
		time_since_last_measurement =
	(now.tv_sec - device->last_measurement.tv_sec) * 1000000 +
	(now.tv_usec - device->last_measurement.tv_usec);
	else
		time_since_last_measurement = 60000;

		/* wait 60 ms between measurements.
		 * now, a while true ; do cat measure ; done should work
		 */

	if (time_since_last_measurement < 60000 &&
	    time_since_last_measurement >= 0)
		msleep(60 - (int)time_since_last_measurement / 1000);

	irq = gpiod_to_irq(device->echo_desc);
	if (irq < 0)
		return -EIO;

	ret = request_any_context_irq(irq, echo_received_irq,
				      IRQF_SHARED | IRQF_TRIGGER_FALLING |
				      IRQF_TRIGGER_RISING,
				      "hc_sr04", device);

	if (ret < 0)
		goto out_mutex;

	gpiod_set_value(device->trig_desc, 1);
	usleep_range(10, 20);
	device->state = DEVICE_TRIGGERED;
	gpiod_set_value(device->trig_desc, 0);

	ret = gpiochip_lock_as_irq(gpiod_to_chip(device->echo_desc),
				   desc_to_gpio(device->echo_desc));
	if (ret < 0)
		goto out_irq;

	timeout = wait_event_interruptible_timeout(
			device->wait_for_echo,
			device->state == DEVICE_ECHO_RECEIVED,
			device->timeout * HZ / 1000);

	device->state = DEVICE_IDLE;

	if (timeout == 0) {
		ret = -ETIMEDOUT;
	} else if (timeout < 0) {
		ret = timeout;
	} else {
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
	struct hc_sr04 *sensor = dev_get_drvdata(dev);
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
			     const char *buf, size_t len, struct device *dev)
{
	int err;
	int echo;

	if (*desc)
		gpiod_put(*desc);

	*desc = gpiod_get(dev, buf, GPIOD_ASIS);
	if (IS_ERR(*desc)) {
		err = PTR_ERR(*desc);
		*desc = NULL;

		if (err == -ENOENT) {	/* fallback: use GPIO numbers */
			err = kstrtoint(buf, 10, &echo);
			if (err < 0)
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

static ssize_t hc_sr04_echo_pin_store(struct config_item *item,
				      const char *buf, size_t len)
{
	struct hc_sr04 *sensor = to_hc_sr04(item);
	ssize_t ret;
	int err;

	ret = configure_pin(&sensor->echo_desc, item, buf, len,
			    &sensor->swt.trigger->dev);

	if (ret >= 0 && sensor->echo_desc) {
		err = gpiod_direction_input(sensor->echo_desc);
		if (err < 0)
			return err;
	}
	return ret;
}

static ssize_t hc_sr04_echo_pin_show(struct config_item *item,
				     char *buf)
{
	struct hc_sr04 *sensor = to_hc_sr04(item);

	if (sensor->echo_desc)
		return sprintf(buf, "%d\n", desc_to_gpio(sensor->echo_desc));
	return 0;
}

static ssize_t hc_sr04_trig_pin_store(struct config_item *item,
				      const char *buf, size_t len)
{
	struct hc_sr04 *sensor = to_hc_sr04(item);
	ssize_t ret;
	int err;

	ret = configure_pin(&sensor->trig_desc, item, buf, len,
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

static ssize_t hc_sr04_trig_pin_show(struct config_item *item,
				     char *buf)
{
	struct hc_sr04 *sensor = to_hc_sr04(item);

	if (sensor->trig_desc)
		return sprintf(buf, "%d\n", desc_to_gpio(sensor->trig_desc));
	return 0;
}

static ssize_t hc_sr04_timeout_store(struct config_item *item,
				     const char *buf, size_t len)
{
	struct hc_sr04 *sensor = to_hc_sr04(item);
	unsigned long t;
	int ret;

	ret = kstrtol(buf, 10, &t);
	if (ret < 0)
		return ret;

	sensor->timeout = t;
	return len;
}

static ssize_t hc_sr04_timeout_show(struct config_item *item,
				    char *buf)
{
	struct hc_sr04 *sensor = to_hc_sr04(item);

	return sprintf(buf, "%ld\n", sensor->timeout);
}

static ssize_t hc_sr04_dev_name_show(struct config_item *item,
				     char *buf)
{
	struct hc_sr04 *sensor = to_hc_sr04(item);

	return sprintf(buf, "%s", dev_name(&sensor->swt.trigger->dev));
}

CONFIGFS_ATTR(hc_sr04_, echo_pin);
CONFIGFS_ATTR(hc_sr04_, trig_pin);
CONFIGFS_ATTR(hc_sr04_, timeout);
CONFIGFS_ATTR_RO(hc_sr04_, dev_name);

static struct configfs_attribute *hc_sr04_config_attrs[] = {
	&hc_sr04_attr_echo_pin,
	&hc_sr04_attr_trig_pin,
	&hc_sr04_attr_timeout,
	&hc_sr04_attr_dev_name,
	NULL
};

static struct config_item_type iio_hc_sr04_type = {
	.ct_owner = THIS_MODULE,
	.ct_attrs = hc_sr04_config_attrs
};

static int iio_trig_hc_sr04_set_state(struct iio_trigger *trig, bool state)
{
	return 0;
}

static const struct iio_trigger_ops iio_hc_sr04_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = iio_trig_hc_sr04_set_state,
};

static struct iio_sw_trigger *iio_trig_hc_sr04_probe(const char *name)
{
	struct hc_sr04 *sensor;
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
	sensor->swt.trigger->ops = &iio_hc_sr04_trigger_ops;
	sensor->swt.trigger->dev.groups = sensor_groups;

	ret = iio_trigger_register(sensor->swt.trigger);
	if (ret)
		goto err_free_trigger;

	iio_swt_group_init_type_name(&sensor->swt, name, &iio_hc_sr04_type);
	return &sensor->swt;

err_free_trigger:
	iio_trigger_free(sensor->swt.trigger);
err_free_sensor:
	kfree(sensor);

	return ERR_PTR(ret);
}

static int iio_trig_hc_sr04_remove(struct iio_sw_trigger *swt)
{
	struct hc_sr04 *rip_sensor;

	rip_sensor = iio_trigger_get_drvdata(swt->trigger);

	iio_trigger_unregister(swt->trigger);

	/* Wait for measurement to be finished. */
	mutex_lock(&rip_sensor->measurement_mutex);

	iio_trigger_free(swt->trigger);
	kfree(rip_sensor);

	return 0;
}

static const struct iio_sw_trigger_ops iio_trig_hc_sr04_ops = {
	.probe          = iio_trig_hc_sr04_probe,
	.remove         = iio_trig_hc_sr04_remove,
};

static struct iio_sw_trigger_type iio_trig_hc_sr04 = {
	.name = "hc-sr04",
	.owner = THIS_MODULE,
	.ops = &iio_trig_hc_sr04_ops,
};

module_iio_sw_trigger_driver(iio_trig_hc_sr04);

MODULE_AUTHOR("Johannes Thoma");
MODULE_DESCRIPTION("Distance measurement for the HC-SR04 ultrasonic distance sensor");
MODULE_LICENSE("GPL");

