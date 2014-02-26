/* linux/drivers/video/samsung/s3cfb_lb070wv6.c
 *
 * Samsung LTE480 4.8" WVGA Display Panel Support
 *
 * Jinsung Yang, Copyright (c) 2009 Samsung Electronics
 * 	http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include "s3cfb.h"

static struct s3cfb_lcd qt_7inch_at070tna2 = {
	.width = 1024,
	.height = 600,
	.bpp = 32,
	.freq = 65,

	.timing = {
                .h_fp   = 60,
                .h_bp   = 60,
                .h_sw   = 200,
                .v_fp   = 5,
                .v_fpe  = 1,
                .v_bp   = 5,
                .v_bpe  = 1,
                .v_sw   = 25,
	},

	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};

/* name should be fixed as 's3cfb_set_lcd_info' */
void s3cfb_set_lcd_info(struct s3cfb_global *ctrl)
{
	qt_7inch_at070tna2.init_ldi = NULL;
	ctrl->lcd = &qt_7inch_at070tna2;
}


