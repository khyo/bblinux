#include <linux/kobject.h>    // Using kobjects for the sysfs bindings


#define I2CSPEED_COUNT 3
static char i2cspeed_is_init = 0;
struct I2cSpeed {
	struct omap_i2c_dev *omap;
} static i2cdevs [I2CSPEED_COUNT];


static ssize_t i2c_speed_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	int idx;
	struct omap_i2c_dev *omap;
	u32 speeds[I2CSPEED_COUNT] = {0};

	for (idx = 0; idx < I2CSPEED_COUNT; idx++) {
		omap = i2cdevs[idx].omap;
		speeds[idx] = 0;
		if (!omap) { continue; }
		speeds[idx] = 48000000 / ((omap->pscstate + 1) * (omap->scllstate + 5 + omap->sclhstate + 7)); 
	}

	return sprintf(buf, "%d %d %d\n", speeds[0], speeds[1], speeds[2]);
}

static ssize_t i2c_speed_store(struct kobject *kobj, struct kobj_attribute *attr,
                             const char *buf, size_t count) {
	struct omap_i2c_dev *omap;
	int idx = -1;
	int speed = -1;
	int psc, sclh, scll;
	
	if (buf[0] == 'x') {
		sscanf(buf+1, "%d %d %d %d", &idx, &psc, &sclh, &scll);
		if (0 > idx || idx >= I2CSPEED_COUNT) { return count; }
		omap = i2cdevs[idx].omap;
		if (!omap) { return 0; }
		omap->pscstate = psc;
		omap->sclhstate = sclh;
		omap->scllstate = scll;
	} else {
		sscanf(buf, "%d %d", &idx, &speed);
		omap = i2cdevs[idx].omap;
		if (!omap) { return 0; }
		if (speed <= 200000) {
			omap->pscstate = 0xb;
			omap->sclhstate = 0xf;
			omap->scllstate = 0xd;
		} else {
			omap->pscstate = 0x4;
			omap->sclhstate = 0x3;
			omap->scllstate = 0x9;
		}
	}
	return count;
}

static struct kobj_attribute i2cspeed_attr_obj = 
	__ATTR(i2cspeed,  S_IWUSR | S_IRUGO, i2c_speed_show, i2c_speed_store);


static int i2cspeed_init(struct omap_i2c_dev *omap) {
	int result = 0;
	int i;
	
	if (!i2cspeed_is_init) {
		i2cspeed_is_init = 1;
		for (i = 0; i < I2CSPEED_COUNT; i++) {
			i2cdevs[i].omap = 0;
		}
		result = sysfs_create_file(kernel_kobj, &i2cspeed_attr_obj.attr);
	}

	
	// printk(KERN_ALERT "i2C speed: id: %d, nr %d, name: %s\n", omap->dev->id, omap->adapter.nr, omap->adapter.name);
	switch(omap->adapter.nr) {
		case 0:
			i2cdevs[0].omap = omap;
			break;
		case 1:
			i2cdevs[1].omap = omap;
			break;
		case 2:
			i2cdevs[2].omap = omap;
			break;
		default:
			result = -1;
	}
		
	if (result) {
		printk(KERN_ALERT "i2C speed: failed to create sysfs entry: %d\n", omap->adapter.nr);
	}
	return result;
}
