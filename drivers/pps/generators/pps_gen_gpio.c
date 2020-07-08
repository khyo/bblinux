/*
 * pps_gen_gpio.c -- kernel GPIO PPS signal generator
 *
 * Copyright (C)  2009   Alexander Gordeev <lasaine@lvk.cs.msu.su>
 *                2018   Juan Solano <jsm@jsolano.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/kobject.h>    // Using kobjects for the sysfs bindings


#define DRVDESC "GPIO PPS signal generator"
MODULE_AUTHOR("Juan Solano <jsm@jsolano.com>");
MODULE_DESCRIPTION(DRVDESC);
MODULE_LICENSE("GPL");

#define GPIO_PULSE_WIDTH_DEF_NS 0  // (30 * NSEC_PER_USEC)    /* 30us */
#define GPIO_PULSE_WIDTH_MAX_NS (100 * NSEC_PER_USEC)   /* 100us */
#define SAFETY_INTERVAL_NS      0 // (10 * NSEC_PER_USEC)    /* 10us */

enum pps_gen_gpio_level {
	PPS_GPIO_LOW = 0,
	PPS_GPIO_HIGH
};

/* Module parameters. */
static unsigned int gpio_pulse_width_ns = GPIO_PULSE_WIDTH_DEF_NS;
MODULE_PARM_DESC(width, "Delay between setting and dropping the signal (ns)");
module_param_named(width, gpio_pulse_width_ns, uint, 0000);

/* Device private data structure. */
struct pps_gen_gpio_devdata {
	struct gpio_desc *pps_gpio;     /* GPIO port descriptor */
	struct hrtimer timer;
	long gpio_instr_time;           /* measured port write time (ns) */
};

/* Average of hrtimer interrupt latency. */
static long hrtimer_avg_latency = SAFETY_INTERVAL_NS;


static void pps_gen_gpio_startup(void);

static struct pps_gen_gpio_devdata* _gdevdata = 0;
static int gpiopps_started = 0;
static ssize_t gpiopps_store(struct kobject *kobj, struct kobj_attribute *attr,
                                   const char *buf, size_t count){
	sscanf(buf, "%du", &gpiopps_started);
	// sysfs_notify(kobj, NULL, attr->attr.name);
	if (gpiopps_started) {
		pps_gen_gpio_startup();
	}
	return count;
}

static ssize_t gpiopps_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
	int retval = sprintf(buf, "%d\n", gpiopps_started);
	return retval;
}

static struct kobj_attribute pps_attr_obj = __ATTR(gpiopps,  S_IWUSR | S_IRUGO, gpiopps_show, gpiopps_store);

static int gpiopps_init(void) {
	int result = 0;
	result = sysfs_create_file(kernel_kobj, &pps_attr_obj.attr);
	return result;
}


/* hrtimer event callback */
static enum hrtimer_restart hrtimer_callback(struct hrtimer *timer)
{
	unsigned long irq_flags;
	long hrtimer_latency, pps_error_ns, time_calc_ns;
	struct pps_gen_gpio_devdata *devdata =
		container_of(timer, struct pps_gen_gpio_devdata, timer);
	struct timespec ts_expire_req, ts_expire_real, ts_hrtimer_latency;

	/* We have to disable interrupts here. The idea is to prevent
	 * other interrupts on the same processor to introduce random
	 * lags while polling the clock; getnstimeofday() takes <1us on
	 * most machines while other interrupt handlers can take much
	 * more potentially.
	 *
	 * Note: approximate time with blocked interrupts =
	 * gpio_pulse_width_ns + SAFETY_INTERVAL_NS + average hrtimer latency
	 */
	local_irq_save(irq_flags);

	/* Get current timestamp and requested time to check if we are late. */	
	getnstimeofday(&ts_expire_real);
	ts_expire_req.tv_sec  = ts_expire_real.tv_sec;
	ts_expire_req.tv_nsec = 0;	
	if (ts_expire_real.tv_nsec > NSEC_PER_SEC/2) {
		ts_expire_req.tv_sec += 1;
	}
	ts_hrtimer_latency = timespec_sub(ts_expire_real, ts_expire_req);
	pps_error_ns = timespec_to_ns(&ts_hrtimer_latency);

	if (-50000 < pps_error_ns && pps_error_ns < 50000) {
		/* Assert PPS GPIO. */
		gpiod_set_value(devdata->pps_gpio, PPS_GPIO_HIGH);
		/* Deassert PPS GPIO. */
		gpiod_set_value(devdata->pps_gpio, PPS_GPIO_LOW);
	}
	local_irq_restore(irq_flags);


	getnstimeofday(&ts_hrtimer_latency);
	ts_hrtimer_latency = timespec_sub(ts_expire_real, ts_hrtimer_latency);
	time_calc_ns = timespec_to_ns(&ts_hrtimer_latency);

	ts_expire_req = ktime_to_timespec(hrtimer_get_softexpires(timer));
	ts_hrtimer_latency = timespec_sub(ts_expire_real, ts_expire_req);
	hrtimer_latency = timespec_to_ns(&ts_hrtimer_latency);

	if (-50000 < pps_error_ns && pps_error_ns < 50000) {
		/* Update the average hrtimer latency. */
		hrtimer_avg_latency =
			(3 * hrtimer_avg_latency + hrtimer_latency) / 4;
	}

	if (hrtimer_latency < -100000000 || hrtimer_latency > 100000000) {
		ts_expire_req.tv_sec = ts_expire_real.tv_sec + 1;
		pr_err("gpiopps resync! [%ld.%09ld] latency: %ld ns\n", 
			ts_expire_real.tv_sec, 
			ts_expire_real.tv_nsec, hrtimer_latency);
	}
	if (gpiopps_started == 2)
		pr_err("gpiopps [%ld.%09ld] latency: %ld, pps_error: %ld ns, %ld ns\n", 
			ts_expire_real.tv_sec, ts_expire_real.tv_nsec, hrtimer_latency, 
			pps_error_ns, time_calc_ns);
	
	/* Update the hrtimer expire time. */
	hrtimer_set_expires(timer,
			    ktime_set(ts_expire_req.tv_sec + 1, 
			    	      NSEC_PER_SEC - (hrtimer_avg_latency + devdata->gpio_instr_time)));

	return HRTIMER_RESTART;
}

/* Initial calibration of GPIO set instruction time. */
#define PPS_GEN_CALIBRATE_LOOPS 100
static void pps_gen_calibrate(struct pps_gen_gpio_devdata *devdata)
{	
	int i;
	struct timespec ts1, ts2, ts_delta;
	long time_acc = 0, time_dummy = 0;
	unsigned long irq_flags, irq_flags2;
	local_irq_save(irq_flags);
		
	for (i = 0; i < PPS_GEN_CALIBRATE_LOOPS; i++) {
		getnstimeofday(&ts1);
		local_irq_save(irq_flags2);
		getnstimeofday(&ts_delta);
		gpiod_set_value(devdata->pps_gpio, PPS_GPIO_LOW);
		ts_delta = timespec_sub(ts_delta, ts1);
		time_dummy += timespec_to_ns(&ts_delta);
		getnstimeofday(&ts2);
		ts_delta = timespec_sub(ts2, ts1);
		time_acc += timespec_to_ns(&ts_delta);
	}
	local_irq_restore(irq_flags);
	
	devdata->gpio_instr_time = time_acc / PPS_GEN_CALIBRATE_LOOPS;
	pr_info("PPS GPIO set takes %ldns, dummy: %ld\n", devdata->gpio_instr_time, time_dummy);
}

static ktime_t pps_gen_first_timer_event(struct pps_gen_gpio_devdata *devdata)
{
	struct timespec ts;

	getnstimeofday(&ts);
	/* First timer callback will be triggered between 1 and 2 seconds from
	 * now, synchronized to the tv_sec increment of the wall-clock time.
	 */
	return ktime_set(ts.tv_sec + 1, NSEC_PER_SEC - 20000);
}

static int pps_gen_gpio_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct pps_gen_gpio_devdata *devdata;

	/* Allocate space for device info. */
	devdata = devm_kzalloc(dev,
			       sizeof(struct pps_gen_gpio_devdata),
			       GFP_KERNEL);
	if (!devdata) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	/* There should be a single PPS generator GPIO pin defined in DT. */
	if (of_gpio_named_count(dev->of_node, "pps-gen-gpio") != 1) {
		dev_err(dev, "There should be exactly one pps-gen GPIO defined in DT\n");
		ret = -EINVAL;
		goto err_dt;
	}

	devdata->pps_gpio = devm_gpiod_get(dev, "pps-gen", GPIOD_OUT_LOW);
	if (IS_ERR(devdata->pps_gpio)) {
		ret = PTR_ERR(devdata->pps_gpio);
		dev_err(dev, "Cannot get PPS GPIO [%d]\n", ret);
		goto err_gpio_get;
	}

	platform_set_drvdata(pdev, devdata);

	ret = gpiod_direction_output(devdata->pps_gpio, PPS_GPIO_HIGH);
	if (ret < 0) {
		dev_err(dev, "Cannot configure PPS GPIO\n");
		goto err_gpio_dir;
	}
	
	_gdevdata = devdata;
	gpiopps_init();

	return 0;

err_gpio_dir:
	devm_gpiod_put(dev, devdata->pps_gpio);
err_gpio_get:
err_dt:
	devm_kfree(dev, devdata);
err_alloc:
	return ret;
}

void pps_gen_gpio_startup(void) {
	struct pps_gen_gpio_devdata *devdata;
	static uint8_t startup_ran = 0;
	if (startup_ran)
		return;

	startup_ran = 1;
	devdata = _gdevdata;

	pps_gen_calibrate(devdata);
	hrtimer_init(&devdata->timer, CLOCK_TAI, HRTIMER_MODE_ABS);
	devdata->timer.function = hrtimer_callback;
	hrtimer_start(&devdata->timer,
		      pps_gen_first_timer_event(devdata),
		      HRTIMER_MODE_ABS);
}

static int pps_gen_gpio_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pps_gen_gpio_devdata *devdata = platform_get_drvdata(pdev);

	devm_gpiod_put(dev, devdata->pps_gpio);
	hrtimer_cancel(&devdata->timer);
	return 0;
}

/* The compatible property here defined is searched for in the DT */
static const struct of_device_id pps_gen_gpio_dt_ids[] = {
	{ .compatible = "pps-gen-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pps_gen_gpio_dt_ids);

static struct platform_driver pps_gen_gpio_driver = {
	.driver			= {
		.name		= "pps_gen_gpio",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(pps_gen_gpio_dt_ids),
	},
	.probe			= pps_gen_gpio_probe,
	.remove			= pps_gen_gpio_remove,
};

static int __init pps_gen_gpio_init(void)
{
	pr_info(DRVDESC "\n");
	if (gpio_pulse_width_ns > GPIO_PULSE_WIDTH_MAX_NS) {
		pr_err("pps_gen_gpio: width value should be not greater than %ldns\n",
		       GPIO_PULSE_WIDTH_MAX_NS);
		return -EINVAL;
	}
	platform_driver_register(&pps_gen_gpio_driver);
	return 0;
}

static void __exit pps_gen_gpio_exit(void)
{
	pr_info("pps_gen_gpio: hrtimer average latency is %ldns\n",
		hrtimer_avg_latency);
	platform_driver_unregister(&pps_gen_gpio_driver);
}

module_init(pps_gen_gpio_init);
module_exit(pps_gen_gpio_exit);
