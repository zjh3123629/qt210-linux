/*
 * i2c driver
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/fs.h>

#include <mach/irqs.h>
#include <asm/system.h>

#include <linux/delay.h>
#include <mach/regs-gpio.h>

#include "s5pv210_ts_gpio_i2c.h"


/* Touch I2C Address Define */
#if defined(CONFIG_QT_TOUCH_HX8526_7INCH)
#define	TOUCH_WR_ADDR 0x94
#define	TOUCH_RD_ADDR 0x95
#define	DELAY_TIME 4
#define	PORT_CHANGE_DELAY_TIME 4
#elif defined(CONFIG_QT_TOUCH_FT5x06_7INCH) || defined(CONFIG_QT_TOUCH_FT5x06_101INCH)
#define	TOUCH_WR_ADDR 0x70
#define	TOUCH_RD_ADDR 0x71
#define	DELAY_TIME 2
#define	PORT_CHANGE_DELAY_TIME 2 
#endif

/* Touch I2C Port define */
#define S5PV210_GPBCON (S5PV210_GPB_BASE + 0x00)
#define S5PV210_GPBDAT (S5PV210_GPB_BASE + 0x04)
#define S5PV210_GPBPUD (S5PV210_GPB_BASE + 0x08)
#define S5PV210_GPBDRV (S5PV210_GPB_BASE + 0x0c)

#define	GPIO_I2C_SDA_CON_PORT (*(unsigned long *)S5PV210_GPBCON)
#define	GPIO_I2C_SDA_DAT_PORT (*(unsigned long *)S5PV210_GPBDAT)
#define	GPIO_SDA_PIN 7  /* GPB7*/

#define	GPIO_I2C_CLK_CON_PORT (*(unsigned long *)S5PV210_GPBCON)
#define	GPIO_I2C_CLK_DAT_PORT (*(unsigned long *)S5PV210_GPBDAT)
#define	GPIO_CLK_PIN 4  /* GPB4*/

#define	GPIO_CON_PORT_MASK 0xF
#define	GPIO_CON_PORT_OFFSET 0x4

#define	GPIO_CON_INPUT 0x0
#define	GPIO_CON_OUTPUT 0x1

#define	HIGH 1
#define	LOW 0

#define IGNORE_ACK
#define I2C_DELAY   mydelay

/*  debug enable flag */
//#define	DEBUG_GPIO_I2C
/*
 * static function prototype
 */
static	void			gpio_i2c_sda_port_control	(unsigned char inout);
static	void			gpio_i2c_clk_port_control	(unsigned char inout);

static	unsigned char	gpio_i2c_get_sda			(void);
static	void			gpio_i2c_set_sda			(unsigned char hi_lo);
static	void			gpio_i2c_set_clk			(unsigned char hi_lo);

static 	void			gpio_i2c_start				(void);
static 	void			gpio_i2c_stop				(void);

static 	void			gpio_i2c_send_ack			(void);
static 	void			gpio_i2c_send_noack			(void);
static 	unsigned char	gpio_i2c_chk_ack			(void);

static 	void 			gpio_i2c_byte_write			(unsigned char wdata);
static 	void			gpio_i2c_byte_read			(unsigned char *rdata);

/* extern function prototype */
int s5pv210_ts_write(unsigned char addr, unsigned char *wdata, unsigned char wsize);
int s5pv210_ts_read(unsigned char *rdata, unsigned char rsize);
void s5pv210_ts_port_init(void);

static void mydelay(int cnt)
{
    int i;
    for (i = 0; i < cnt * 25*3; i++)
        __asm__ __volatile__ ( " nop; nop");
}

static void gpio_i2c_sda_port_control(unsigned char inout)
{
	GPIO_I2C_SDA_CON_PORT &=  (unsigned long)(~(GPIO_CON_PORT_MASK << (GPIO_SDA_PIN * GPIO_CON_PORT_OFFSET)));
	GPIO_I2C_SDA_CON_PORT |=  (unsigned long)( (inout << (GPIO_SDA_PIN * GPIO_CON_PORT_OFFSET)));
}

static void gpio_i2c_clk_port_control(unsigned char inout)
{
	GPIO_I2C_CLK_CON_PORT &=  (unsigned long)(~(GPIO_CON_PORT_MASK << (GPIO_CLK_PIN * GPIO_CON_PORT_OFFSET)));
	GPIO_I2C_CLK_CON_PORT |=  (unsigned long)( (inout << (GPIO_CLK_PIN * GPIO_CON_PORT_OFFSET)));
}

static unsigned char gpio_i2c_get_sda(void)
{
	return	GPIO_I2C_SDA_DAT_PORT & (HIGH << GPIO_SDA_PIN) ? 1 : 0;
}

static void gpio_i2c_set_sda(unsigned char hi_lo)
{
	if(hi_lo)	{
		gpio_i2c_sda_port_control(GPIO_CON_INPUT);
		I2C_DELAY(PORT_CHANGE_DELAY_TIME);
	}
	else		{
		GPIO_I2C_SDA_DAT_PORT &= ~(HIGH << GPIO_SDA_PIN);
		gpio_i2c_sda_port_control(GPIO_CON_OUTPUT);
		I2C_DELAY(PORT_CHANGE_DELAY_TIME);
	}
}

static void gpio_i2c_set_clk (unsigned char hi_lo)
{
	if(hi_lo)
	{
		gpio_i2c_clk_port_control(GPIO_CON_INPUT);
		I2C_DELAY(PORT_CHANGE_DELAY_TIME);
	}
	else
	{
		GPIO_I2C_CLK_DAT_PORT &= ~(HIGH << GPIO_CLK_PIN);
		gpio_i2c_clk_port_control(GPIO_CON_OUTPUT);
		I2C_DELAY(PORT_CHANGE_DELAY_TIME);
	}
}

static void gpio_i2c_start(void)
{
	// Setup SDA, CLK output High
	gpio_i2c_set_sda(HIGH);
	gpio_i2c_set_clk(HIGH);

	I2C_DELAY(DELAY_TIME);

	// SDA low before CLK low
	gpio_i2c_set_sda(LOW);	I2C_DELAY(DELAY_TIME);
	gpio_i2c_set_clk(LOW);	I2C_DELAY(DELAY_TIME);
}

static void gpio_i2c_stop(void)
{
	// Setup SDA, CLK output low
	gpio_i2c_set_sda(LOW);
	gpio_i2c_set_clk(LOW);

	I2C_DELAY(DELAY_TIME);

	// SDA high after CLK high
	gpio_i2c_set_clk(HIGH);	I2C_DELAY(DELAY_TIME);
	gpio_i2c_set_sda(HIGH);	I2C_DELAY(DELAY_TIME);
}

static void gpio_i2c_send_ack(void)
{
	// SDA Low
	gpio_i2c_set_sda(LOW);
	I2C_DELAY(DELAY_TIME);
	gpio_i2c_set_clk(HIGH);
	I2C_DELAY(DELAY_TIME);
	gpio_i2c_set_clk(LOW);
	I2C_DELAY(DELAY_TIME);
}

static void gpio_i2c_send_noack(void)
{
	// SDA High
	gpio_i2c_set_sda(HIGH);
	I2C_DELAY(DELAY_TIME);
	gpio_i2c_set_clk(HIGH);
	I2C_DELAY(DELAY_TIME);
	gpio_i2c_set_clk(LOW);
	I2C_DELAY(DELAY_TIME);
}

static unsigned char gpio_i2c_chk_ack (void)
{
	unsigned char	count = 0, ret = 0;

	gpio_i2c_set_sda(LOW);
	I2C_DELAY(DELAY_TIME);
	gpio_i2c_set_clk(HIGH);
	I2C_DELAY(DELAY_TIME);

	gpio_i2c_sda_port_control(GPIO_CON_INPUT);
	I2C_DELAY(PORT_CHANGE_DELAY_TIME);

	while(gpio_i2c_get_sda())
	{
		if(count++ > 100)  // by crazyboy 100 -> 1000
		{
			ret = 1;
			break;
		}
		else
			I2C_DELAY(DELAY_TIME);
	}

	gpio_i2c_set_clk(LOW);		I2C_DELAY(DELAY_TIME);

#if defined(DEBUG_GPIO_I2C)
	if(ret)
		printk("%s %d: no ack!!\n",__func__, ret);
#endif

	return	ret;
}

static	void 		gpio_i2c_byte_write		(unsigned char wdata)
{
	unsigned char	cnt, mask;

	for(cnt = 0, mask = 0x80; cnt < 8; cnt++, mask >>= 1)	{
		if(wdata & mask)		gpio_i2c_set_sda(HIGH);
		else					gpio_i2c_set_sda(LOW);

		gpio_i2c_set_clk(HIGH);		I2C_DELAY(DELAY_TIME);
		gpio_i2c_set_clk(LOW);		I2C_DELAY(DELAY_TIME);
	}
}

static	void		gpio_i2c_byte_read		(unsigned char *rdata)
{
	unsigned char	cnt, mask;

	gpio_i2c_sda_port_control(GPIO_CON_INPUT);
	I2C_DELAY(PORT_CHANGE_DELAY_TIME);

	for(cnt = 0, mask = 0x80, *rdata = 0; cnt < 8; cnt++, mask >>= 1)	{
		gpio_i2c_set_clk(HIGH);		I2C_DELAY(DELAY_TIME);

		if(gpio_i2c_get_sda())		*rdata |= mask;

		gpio_i2c_set_clk(LOW);		I2C_DELAY(DELAY_TIME);
	}
}

int s5pv210_ts_write (unsigned char addr, unsigned char *wdata, unsigned char wsize)
{
	unsigned char cnt, ack;
    int ret = 0;

	/* start */
	gpio_i2c_start();

	gpio_i2c_byte_write(TOUCH_WR_ADDR);	// i2c address

	if((ack = gpio_i2c_chk_ack()))	{
#if defined(IGNORE_ACK)
		ret = 1;
#else
		goto	write_stop;
#endif
	}

	gpio_i2c_byte_write(addr);	// register

	if((ack = gpio_i2c_chk_ack()))	{
	}

	if(wsize)	{

		for(cnt = 0; cnt < wsize; cnt++) {

			gpio_i2c_byte_write(wdata[cnt]);

			if((ack = gpio_i2c_chk_ack()))	{
#if defined(IGNORE_ACK)
	        	ret = 1;
#else
				goto	write_stop;
#endif
			}
		}
	}

write_stop:
    if (wsize)
	    gpio_i2c_stop();

#if defined(DEBUG_GPIO_I2C)
	printk("%s : %d\n", __func__, ack);
#endif
	return	ret;
}

int s5pv210_ts_read(unsigned char *rdata, unsigned char rsize)
{
	unsigned char ack, cnt;
    int ret = 0;

	// start
	gpio_i2c_start();

	gpio_i2c_byte_write(TOUCH_RD_ADDR);	// i2c address

	if((ack = gpio_i2c_chk_ack()))	{
#if defined(IGNORE_ACK)
		ret = 1;
#else
		goto	read_stop;
#endif
	}

	for(cnt=0; cnt < rsize; cnt++)	{

		gpio_i2c_byte_read(&rdata[cnt]);

		if(cnt == rsize -1)
			gpio_i2c_send_noack();
		else
			gpio_i2c_send_ack();
	}

read_stop:
	gpio_i2c_stop();


#if defined(DEBUG_GPIO_I2C)
	printk("%s : %d\n", __func__, ack);
#endif
	return	ret;
}

void s5pv210_ts_port_init (void)
{
#if 1
    *(unsigned long *)S5PV210_GPBPUD = 0xaaaa;
    *(unsigned long *)S5PV210_GPBDRV = 0xffff;
#endif
	gpio_i2c_set_sda(HIGH);
	gpio_i2c_set_clk(HIGH);
}

