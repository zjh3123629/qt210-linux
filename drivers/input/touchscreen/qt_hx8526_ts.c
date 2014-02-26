/*
 * Himax QT_CAP_TOUCH-A Touchscreen driver
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#define QT_CAP_TOUCH_I2C_GPIO
#define QT_CAP_TOUCH_MULTI_TOUCH

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#ifdef QT_CAP_TOUCH_I2C_GPIO
#include "s5pv210_ts_gpio_i2c.h"
#endif


//#define DEBUG_QT_CAP_TOUCH
#ifdef DEBUG_QT_CAP_TOUCH
#define QT_CAP_TOUCH_DBG(a, ...)        printk(KERN_INFO a, ##__VA_ARGS__)
#define QT_CAP_TOUCH_DBG2(a, ...)        printk(KERN_INFO a, ##__VA_ARGS__)
#define QT_CAP_TOUCH_DBG3(a, ...)        printk(KERN_INFO a, ##__VA_ARGS__)
#define QT_CAP_TOUCH_DBG4(a, ...)        printk(KERN_INFO a, ##__VA_ARGS__)
#define QT_CAP_TOUCH_ERR(a, ...)        printk(KERN_INFO a, ##__VA_ARGS__)
#else   
#define QT_CAP_TOUCH_DBG(a, ...)        
#define QT_CAP_TOUCH_DBG2(a, ...)      
#define QT_CAP_TOUCH_DBG3(a, ...)         
#define QT_CAP_TOUCH_DBG4(a, ...)
#define QT_CAP_TOUCH_ERR(a, ...)        
#endif


/*
 * Standard Command
 */
#define QT_CAP_TOUCH_NOP		0x0	/* no param */
#define QT_CAP_TOUCH_SLEEP_IN		0x80	/* no param */
#define QT_CAP_TOUCH_SLEEP_OUT	0x81	/* no param */
#define QT_CAP_TOUCH_SENSE_OFF	0x82	/* no param */
#define QT_CAP_TOUCH_SENSE_ON		0x83	/* no param */

#define QT_CAP_TOUCH_READ_STATUS	0x84	/* 4 bytes param */
#define QT_CAP_TOUCH_READ_EVENT	0x85	/* 4 bytes param */
#define QT_CAP_TOUCH_READ_ALL_EVENT	0x86	/* (4n + 4) bytes param */
#define QT_CAP_TOUCH_READ_LAST_EVENT	0x87	/* 4 bytes param */

#define QT_CAP_TOUCH_CLEAR_STACK	0x88	/* no param */
#define QT_CAP_TOUCH_LOAD_TOUCH	0x89	/* no param */
#define QT_CAP_TOUCH_TS_ORIENT	0x8F	/* 4 bytes param */
#define QT_CAP_TOUCH_SOFT_RESET	0x9E	/* no param */

/*
 * User defined command
 */
#define QT_CAP_TOUCH_DEV_ID	0x31	/* 3 bytes param */
#define QT_CAP_TOUCH_VER_ID	0x32	/* 1 byte param  */


#define qt_cap_touch_send_nop(c)	qt_cap_touch_write_reg(c, QT_CAP_TOUCH_NOP, 0, NULL);
#define qt_cap_touch_sleep_in(c)	qt_cap_touch_write_reg(c, QT_CAP_TOUCH_SLEEP_IN, 0, NULL);
#define qt_cap_touch_sleep_out(c)	qt_cap_touch_write_reg(c, QT_CAP_TOUCH_SLEEP_OUT, 0, NULL);
#define qt_cap_touch_sense_on(c)	qt_cap_touch_write_reg(c, QT_CAP_TOUCH_SENSE_ON, 0, NULL);
#define qt_cap_touch_sense_off(c)	qt_cap_touch_write_reg(c, QT_CAP_TOUCH_SENSE_OFF, 0, NULL);
#define qt_cap_touch_clear_stack(c)	qt_cap_touch_write_reg(c, QT_CAP_TOUCH_CLEAR_STACK, 0, NULL);
#define qt_cap_touch_soft_reset(c)	qt_cap_touch_write_reg(c, QT_CAP_TOUCH_SOFT_RESET, 0, NULL);


#define QT_CAP_TOUCH_RESET_TIME             65      /* msec */

/* Touchscreen absolute values */
#if 0   /* for 800 x 480 */
#define QT_CAP_TOUCH_MAX_XC		799
#define QT_CAP_TOUCH_MAX_YC		479
#else
#define QT_CAP_TOUCH_MAX_XC		(1024 - 1)
#define QT_CAP_TOUCH_MAX_YC		(600 - 1)
//CRZ test 
//#define QT_CAP_TOUCH_MAX_XC		(600 - 1)
//#define QT_CAP_TOUCH_MAX_YC		(1024 - 1)
#endif
#define QT_CAP_TOUCH_MAX_AREA		0xff

#define QT_CAP_TOUCH_MAX_POINT	4
#define QT_CAP_TOUCH_MAX_BUFLEN	20

#define QT_CAP_TOUCH_PRESS	0x1
#define QT_CAP_TOUCH_RELEASE	0x2

struct qt_cap_touch_point {
	unsigned char data[QT_CAP_TOUCH_MAX_POINT * 4];	// 16 bytes
	unsigned char area[4];		// 4 bytes
	unsigned char cnt;
	unsigned char id;
	unsigned char hotkey;
	unsigned char reserved;
};

struct qt_cap_touch_finger {
	int x;
	int y;
	int status;
};

/* Each client has this additional data */
struct qt_cap_touch_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct qt_cap_touch_point point;
	struct qt_cap_touch_finger pt[QT_CAP_TOUCH_MAX_POINT];
	unsigned int irq;
    unsigned int input_start;
    char phys[32];
    struct delayed_work work;
};

static void qt_cap_touch_dump_all(struct qt_cap_touch_point *pt)
{
	int i;
    unsigned char *cp = (unsigned char *)pt;
    unsigned char sbuf[256];
    unsigned char *sp = sbuf;

    memset(sbuf, 0x0, 256);
	for (i = 0; i < sizeof(struct qt_cap_touch_point); i++) {
        sprintf(sp, "0x%02X, ", *cp++);
        sp += 6;
    }
	QT_CAP_TOUCH_ERR("%s\n", sbuf);
}

static void qt_cap_touch_dump_xy_data(struct qt_cap_touch_point *pt)
{
#ifdef DEBUG_QT_CAP_TOUCH
	int i;

	QT_CAP_TOUCH_DBG3("cnt = 0x%x, id = 0x%x\n", pt->cnt, pt->id);
	//for (i = 0; i < QT_CAP_TOUCH_MAX_POINT; i++) {
	for (i = 0; i < 1; i++) {
		QT_CAP_TOUCH_DBG3("[%d]: 0x%x, 0x%x, 0x%x, 0x%x\n",
			i, pt->data[i*4], pt->data[i*4+1],
			   pt->data[i*4+2], pt->data[i*4+3]);
	}
#endif
}

static int qt_cap_touch_read_reg(struct i2c_client *client,
			       u8 reg, u16 len, void *val)
{
#ifdef QT_CAP_TOUCH_I2C_GPIO
    int ret;

    ret = s5pv210_ts_write(reg, NULL, 0);
    if (ret) {
		QT_CAP_TOUCH_DBG4("%s: failed to read 0x%x\n", __func__, reg);
        return ret;
    }
    return s5pv210_ts_read(val, len);
#else
	struct i2c_msg xfer[2];
	u8 buf[2];

	//QT_CAP_TOUCH_DBG("%s: addr = 0x%x, reg = 0x%x, len = %d\n", __func__, client->addr, reg, len);
	buf[0] = reg & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	if (i2c_transfer(client->adapter, xfer, 2) != 2) {
		QT_CAP_TOUCH_ERR("%s: failed to read 0x%x\n", __func__, reg);
		return -EIO;
	}

	return 0;
#endif
}

static int qt_cap_touch_write_reg(struct i2c_client *client, u8 reg, int len, u8 *param)
{
#ifdef QT_CAP_TOUCH_I2C_GPIO
    return s5pv210_ts_write(reg, param, (u8)len);
#else
	u8 buf[QT_CAP_TOUCH_MAX_BUFLEN];
	int i;

	//QT_CAP_TOUCH_DBG("%s: addr = 0x%x, reg = 0x%x\n", __func__, client->addr, reg);
	buf[0] = reg & 0xff;
	if (len != 0) {
		for (i = 0; i < len; i++) {
			buf[i+1] = param[i];
		}
	}

	if (i2c_master_send(client, buf, len+1) != (len+1)) {
		QT_CAP_TOUCH_ERR("%s: failed to write 0x%x\n", __func__, reg);
		return -EIO;
	}

	return 0;
#endif
}

static int
qt_cap_touch_set_idle(struct i2c_client *client, int mode, u8 interval)
{
	int ret = 0;
	return ret;
}

static int
qt_cap_touch_read_event(struct i2c_client *client, u8 *buf)
{
	return qt_cap_touch_read_reg(client, QT_CAP_TOUCH_READ_EVENT, 4, buf);
}

static int
qt_cap_touch_read_last_event(struct i2c_client *client, u8 *buf)
{
	return qt_cap_touch_read_reg(client, QT_CAP_TOUCH_READ_LAST_EVENT, 4, buf);
}

static int
qt_cap_touch_read_all_event(struct i2c_client *client, u8 *buf)
{
	return qt_cap_touch_read_reg(client, QT_CAP_TOUCH_READ_ALL_EVENT, sizeof(struct qt_cap_touch_point), buf);
}

static int
qt_cap_touch_read_message(struct qt_cap_touch_data *data)
{
	int ret;

	ret = qt_cap_touch_read_all_event(data->client, (u8 *)&data->point);
	//qt_cap_touch_dump_xy_data(&data->point);
	//qt_cap_touch_dump_all(&data->point);

    return ret;
}

static int qt_cap_check_release(struct qt_cap_touch_point *point)
{
    int i;
	unsigned char *dp = point;

    for (i = 0; i < sizeof(struct qt_cap_touch_point); i++)  {
        if (*dp++ != 0xff)
            return 0;
    }
    return 1;
}

static int pendown = 0;
static void qt_cap_touch_input_report(struct qt_cap_touch_data *data)
{
	struct qt_cap_touch_point *point = &data->point;
	struct input_dev *input_dev = data->input_dev;
	int temp_x, temp_y;
	int i;
	int finger_num = 0;
	int single_id = 0xff;
	int sync = 0;

#ifdef QT_CAP_TOUCH_MULTI_TOUCH
	for(i = 0; i < QT_CAP_TOUCH_MAX_POINT; i++) {
#else
	for(i = 0; i < 1; i++) {
#endif
		temp_x = (point->data[i*4 + 0] << 8) | (point->data[i*4 + 1]);
		temp_y = (point->data[i*4 + 2] << 8) | (point->data[i*4 + 3]);
		if((point->id != 0x0) && (point->id != 0xff) && (point->id & (1<<i))) {

			if((temp_y != 0xffff) && (temp_x != 0xffff)) {
				if((temp_x < QT_CAP_TOUCH_MAX_XC) && (temp_y < QT_CAP_TOUCH_MAX_YC)) {
					data->pt[i].x = temp_x;
					data->pt[i].y = temp_y;
					QT_CAP_TOUCH_DBG2("Touch[%d:%d]:  x:%d, y:%d area:%d\n", point->id, i, data->pt[i].x, data->pt[i].y, point->area[i]);
					data->pt[i].status = QT_CAP_TOUCH_PRESS;
					finger_num++;
                    if (single_id == 0xff)
                        single_id = i;
					sync = 1;
#ifdef QT_CAP_TOUCH_MULTI_TOUCH
                    input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, point->area[i]*2);
                    input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, point->area[i]/2);
                    input_report_abs(input_dev, ABS_MT_POSITION_X, data->pt[i].x);
                    input_report_abs(input_dev, ABS_MT_POSITION_Y, data->pt[i].y);
	      input_report_abs(data->input_dev, ABS_PRESSURE, 255);//CRZ		    
                    input_mt_sync(input_dev);
#endif
					//qt_cap_touch_clear_stack(data->client);
				} else {
				    QT_CAP_TOUCH_DBG2("Out of range: x = 0x%x\n", temp_x);
					qt_cap_touch_clear_stack(data->client);
				}
			} else {
				QT_CAP_TOUCH_DBG2("Invalid Pos: x = 0x%x\n", temp_x);
				qt_cap_touch_clear_stack(data->client);
			}
		} else {
            if((data->pt[i].status == QT_CAP_TOUCH_PRESS) && qt_cap_check_release(point)) {
#ifdef QT_CAP_TOUCH_MULTI_TOUCH
                input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0x0);
                input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 0x0);
                input_report_abs(input_dev, ABS_MT_POSITION_X, data->pt[i].x);
                input_report_abs(input_dev, ABS_MT_POSITION_Y, data->pt[i].y);
                input_report_abs(data->input_dev, ABS_PRESSURE, 0);		 
                input_mt_sync(input_dev);
#endif
				data->pt[i].status = QT_CAP_TOUCH_RELEASE;
				sync = 1;
                pendown = 0;
	            //qt_cap_touch_dump_all(point);
				QT_CAP_TOUCH_DBG2("pt: num=%d release\n", point->id);
			}
		}
	}

	input_report_key(input_dev, BTN_TOUCH, finger_num > 0);
	if (finger_num) {
		input_report_abs(input_dev, ABS_X, data->pt[single_id].x);
		input_report_abs(input_dev, ABS_Y, data->pt[single_id].y);
    }

	if (sync)
		input_sync(input_dev);

}

static irqreturn_t qt_cap_touch_interrupt(int irq, void *dev_id)
{
	struct qt_cap_touch_data *data = dev_id;
    unsigned long flags;

	//QT_CAP_TOUCH_DBG("%s: %d\n", __func__, irq);
    local_irq_save(flags);
    local_irq_disable();
	do {
		if (qt_cap_touch_read_message(data)) {
			QT_CAP_TOUCH_DBG4("Failed to read message\n");
#if 1   /* ignore error */
			continue;
#else
            schedule_delayed_work(&data->work, msecs_to_jiffies(1));
			break;
#endif
		}
        if (data->input_start)
		    qt_cap_touch_input_report(data);
    //} while (0);
    } while (gpio_get_value(S5PV210_GPH0(6)) == 0);

    local_irq_restore(flags);
	return IRQ_HANDLED;
}

#define GPIO_QT_CAP_TOUCH_RESET   S5PV210_GPH0(5)
static void qt_cap_touch_reset()
{
	QT_CAP_TOUCH_DBG4("%s: Entered\n", __func__);
    if (gpio_request(GPIO_QT_CAP_TOUCH_RESET, "qt_cap_touch-reset") < 0) {
        QT_CAP_TOUCH_ERR("%s: failed to request GPIO\n", __func__);
    } else {
        if (gpio_direction_output(GPIO_QT_CAP_TOUCH_RESET, 0))
            QT_CAP_TOUCH_ERR(KERN_WARNING "%s: GPIO cannot be configured as input\n",  __func__);
        else
            QT_CAP_TOUCH_DBG3("%s: GPIO140 = %d\n", __func__, gpio_get_value(GPIO_QT_CAP_TOUCH_RESET));
    }

    gpio_direction_output(GPIO_QT_CAP_TOUCH_RESET, 1);
    mdelay(6);
    gpio_direction_output(GPIO_QT_CAP_TOUCH_RESET, 0);
    mdelay(1);
    gpio_direction_output(GPIO_QT_CAP_TOUCH_RESET, 1);
    mdelay(10); /* FIXIT */
    gpio_free(GPIO_QT_CAP_TOUCH_RESET);
}

static void qt_cap_touch_error_handle(struct work_struct *work)
{
	int error;

    //struct qt_cap_touch_data *data = dev_id;
    struct qt_cap_touch_data *data = container_of(to_delayed_work(work), struct qt_cap_touch_data, work);
	QT_CAP_TOUCH_ERR("%s: Entered\n", __func__);
#if 1
	if (qt_cap_touch_read_message(data)) {
		QT_CAP_TOUCH_DBG4("Failed to read message\n");
        schedule_delayed_work(&data->work, msecs_to_jiffies(1));
    }
#else
    qt_cap_touch_reset();
	error = qt_cap_touch_initialize(data);
	if (error) {
		QT_CAP_TOUCH_ERR("Ooops Can't init qt_cap_touch\n");
        schedule_delayed_work(&data->work, msecs_to_jiffies(1));
    }
#endif
}

static void qt_cap_touch_start(struct qt_cap_touch_data *data)
{
               QT_CAP_TOUCH_DBG4("%s: \n", __func__);
	data->input_start = 1;
	/* Touch sense enable */
	qt_cap_touch_sense_on(data->client);
	qt_cap_touch_clear_stack(data->client);
}

static void qt_cap_touch_stop(struct qt_cap_touch_data *data)
{
               QT_CAP_TOUCH_DBG4("%s: \n", __func__);
	data->input_start = 0;
	/* Touch sense disable */
	qt_cap_touch_sense_off(data->client);
}

static int qt_cap_touch_input_open(struct input_dev *dev)
{
	struct qt_cap_touch_data *data = input_get_drvdata(dev);

	QT_CAP_TOUCH_DBG4("%s: \n", __func__);
	qt_cap_touch_start(data);

	return 0;
}

static void qt_cap_touch_input_close(struct input_dev *dev)
{
	struct qt_cap_touch_data *data = input_get_drvdata(dev);

	QT_CAP_TOUCH_DBG4("%s: \n", __func__);
	qt_cap_touch_stop(data);
}

static void qt_cap_touch_read_fw(struct qt_cap_touch_data *data)
{
    struct i2c_client *client = data->client;
    u8 param[4];
    u8 buf[4];
    u8 ascii[20];
    u8 iword, ipage, isector;
    int i;

#define WADDR   0x1000
    iword = WADDR & 0x1F;
    ipage = (WADDR & 0x03E0) >> 5;
    isector = (WADDR & 0x1C00) >> 10;

    qt_cap_touch_sense_off(client);
    param[0] = 0x01;
    param[1] = 0x00;
    param[2] = 0x02;
    qt_cap_touch_write_reg(client, 0x43, 3, param);
    mdelay(1);

    QT_CAP_TOUCH_DBG4("%s: Reading FW Ver...\n", __func__);
    memset(ascii, 0, 20);
    for (i = 0; i < 20; i++) {
        param[0] = iword + i;
        param[1] = ipage;
        param[2] = isector;
        qt_cap_touch_write_reg(client, 0x44, 3, param);
        mdelay(1);

        qt_cap_touch_write_reg(client, 0x46, 0, NULL);
        mdelay(1);
        qt_cap_touch_read_reg(client, 0x59, 4, buf);
        QT_CAP_TOUCH_DBG4("0x%02X, 0x%02X, 0x%02X, 0x%02X, ",buf[0], buf[1], buf[2], buf[3]);
        //QT_CAP_TOUCH_DBG4("%c, %c, %c, %c",buf[0], buf[1], buf[2], buf[3]);
        if (((i+1) % 4) == 0) {
            QT_CAP_TOUCH_DBG4("\n");
            //QT_CAP_TOUCH_DBG4("%s\n", ascii);
        } else {
            ascii[(i%4)*4 + 0] = buf[0];
            ascii[(i%4)*4 + 1] = buf[1];
            ascii[(i%4)*4 + 2] = buf[2];
            ascii[(i%4)*4 + 3] = buf[3];
        }
    }

    param[0] = 0x00;
    param[1] = 0x00;
    param[2] = 0x02;
    qt_cap_touch_write_reg(client, 0x43, 3, param);
}

static int qt_cap_touch_initialize(struct qt_cap_touch_data *data)
{
    struct i2c_client *client = data->client;
    int error;
    u8 param[2];

    qt_cap_touch_sleep_out(client);  /* wakeup IC */
    mdelay(120);    /* delay 120ms */

    /* Reload Disable */
    param[0] = 0x02;
    error = qt_cap_touch_write_reg(client, 0x42, 1, param);

    /* enable MCU */
    param[0] = 0x02;
    error = qt_cap_touch_write_reg(client, 0x35, 1, param);
    mdelay(1);  /* delay 100 us */

    /* enable flash */
    param[0] = 0x0F;
    param[1] = 0x53;
    error = qt_cap_touch_write_reg(client, 0x36, 2, param);
    mdelay(1);  /* delay 100 us */

    /* prefetch */
    param[0] = 0x04;
    param[1] = 0x02;
    error = qt_cap_touch_write_reg(client, 0xDD, 2, param);

    /* set output */
    param[0] = 0x00;
    error = qt_cap_touch_write_reg(client, 0xE9, 1, param);

    qt_cap_touch_start(data);
    mdelay(1);  /* delay 100 us */

#if 1
{
    u8 buf[4];
    qt_cap_touch_read_reg(client, QT_CAP_TOUCH_DEV_ID, 3, buf);
    QT_CAP_TOUCH_ERR("%s: DEV id = 0x%x, 0x%x, 0x%x\n", __func__, buf[0], buf[1], buf[2]);
    qt_cap_touch_read_reg(client, QT_CAP_TOUCH_VER_ID, 1, buf);
    QT_CAP_TOUCH_ERR("%s: FW id = 0x%x\n", __func__, buf[0]);
    qt_cap_touch_read_fw(data);
}
#endif

    dev_info(&client->dev, "QT_CAP_TOUCH found and ready\n");

    return 0;
}

static int __devinit qt_cap_touch_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct qt_cap_touch_data *data;
	struct input_dev *input_dev;
	int error;

	QT_CAP_TOUCH_DBG("%s: \n", __func__);

	data = kzalloc(sizeof(struct qt_cap_touch_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		QT_CAP_TOUCH_ERR("Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	input_dev->name = "s3c_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = qt_cap_touch_input_open;
	input_dev->close = qt_cap_touch_input_close;
    INIT_DELAYED_WORK(&data->work, qt_cap_touch_error_handle);

    snprintf(data->phys, sizeof(data->phys), "%s/input0", dev_name(&client->dev));
    //input_dev->phys = data->phys;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X, 0, QT_CAP_TOUCH_MAX_XC, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, QT_CAP_TOUCH_MAX_YC, 0, 0);

#ifdef QT_CAP_TOUCH_MULTI_TOUCH
	/* For multi touch */
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, QT_CAP_TOUCH_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, QT_CAP_TOUCH_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, QT_CAP_TOUCH_MAX_XC, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, QT_CAP_TOUCH_MAX_YC, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);//CRZ 
#endif

	input_set_drvdata(input_dev, data);

	data->client = client;
	data->input_dev = input_dev;
	data->irq = client->irq;
	data->input_start = 1;

#ifdef QT_CAP_TOUCH_I2C_GPIO
    s5pv210_ts_port_init();
#else
	i2c_set_clientdata(client, data);
#endif

	QT_CAP_TOUCH_DBG3("%s: GPIO140-0 = %d\n", __func__, gpio_get_value(140));
	error = qt_cap_touch_initialize(data);
	QT_CAP_TOUCH_DBG3("%s: GPIO140-1 = %d\n", __func__, gpio_get_value(140));
	if (error)
		goto err_free_mem;

#if 1
	error = request_irq(client->irq, qt_cap_touch_interrupt,
			IRQF_TRIGGER_FALLING | IRQF_DISABLED, client->dev.driver->name, data);
#else
	error = request_threaded_irq(client->irq, NULL, qt_cap_touch_interrupt,
			IRQF_TRIGGER_FALLING, client->dev.driver->name, data);
#endif

	if (error) {
		QT_CAP_TOUCH_ERR("Failed to register interrupt\n");
		goto err_free_irq;
	}

	error = input_register_device(input_dev);
	if (error)
		goto err_free_irq;

	return 0;

err_free_irq:
	free_irq(client->irq, data);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
	return error;
}

static int __devexit qt_cap_touch_remove(struct i2c_client *client)
{
	struct qt_cap_touch_data *data = i2c_get_clientdata(client);

	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int qt_cap_touch_suspend(struct device *dev)
{
	QT_CAP_TOUCH_DBG4("%s: \n", __func__);
#if 0//CRZ
	struct i2c_client *client = to_i2c_client(dev);
	struct qt_cap_touch_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
              
	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		qt_cap_touch_stop(data);

	mutex_unlock(&input_dev->mutex);
#endif
	return 0;
}

static int qt_cap_touch_resume(struct device *dev)
{
	QT_CAP_TOUCH_DBG4("%s: \n", __func__);
#if 0//CRZ	
	struct i2c_client *client = to_i2c_client(dev);
	struct qt_cap_touch_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
              QT_CAP_TOUCH_DBG4("%s: \n", __func__);
	msleep(QT_CAP_TOUCH_RESET_TIME);

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		qt_cap_touch_start(data);

	mutex_unlock(&input_dev->mutex);
#endif
	return 0;
}

static const struct dev_pm_ops qt_cap_touch_pm_ops = {
	.suspend	= qt_cap_touch_suspend,
	.resume		= qt_cap_touch_resume,
};
#endif

static const struct i2c_device_id qt_cap_touch_id[] = {
	{ "qt_cap_touch_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, qt_cap_touch_id);

static struct i2c_driver qt_cap_touch_driver = {
	.driver = {
		.name	= "s3c_ts",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &qt_cap_touch_pm_ops,
#endif
	},
	.probe		= qt_cap_touch_probe,
	.remove		= __devexit_p(qt_cap_touch_remove),
	.id_table	= qt_cap_touch_id,
};

static int __init qt_cap_touch_init(void)
{
	return i2c_add_driver(&qt_cap_touch_driver);
}

static void __exit qt_cap_touch_exit(void)
{
	i2c_del_driver(&qt_cap_touch_driver);
}

module_init(qt_cap_touch_init);
module_exit(qt_cap_touch_exit);

/* Module information */
MODULE_AUTHOR("Edward Choi <euddeum.choi@crz-tech.com>");
MODULE_DESCRIPTION("QT_CAP_TOUCH-A Touchscreen driver");
MODULE_LICENSE("GPL");
