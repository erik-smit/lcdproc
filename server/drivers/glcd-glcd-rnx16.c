/** \file server/drivers/glcd-glcd-rnx16.c
 * Driver for the oled panel in the  from mini-box.com.  Based on
 * kernel/patches/321-drivers-add-new-readynas-io-and-readynas-led-drivers.patch
 */

/*-
 * Copyright (c) 2023 Erik Smit <erik.lucas.smit@gmail.com>
 *
 * This file is released under the GNU General Public License. Refer to the
 * COPYING file distributed with this package.
 */

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <gpiod.h>

#include "lcd.h"
#include "shared/report.h"
#include "glcd-low.h"

#ifndef	GPIO_CONSUMER
#define	GPIO_CONSUMER	"lcdproc_glcd_rnx16"
#endif

/* Display properties */
#define RNX16_WIDTH		128
#define RNX16_HEIGHT	64

/** Private data for the rnx16 connection type */
typedef struct glcd_rnx16_data {
	struct gpiod_chip *chip;
	struct gpiod_line *sdin;
	struct gpiod_line *sclk;
	struct gpiod_line *dc;
	struct gpiod_line *cs;
	struct gpiod_line *ctrl;
	struct gpiod_line *reset;
	unsigned char inverted;
	int keytimeout;

	unsigned char *backingstore;
} CT_picolcdgfx_data;

/* Prototypes */
void glcd_rnx16_blit(PrivateData *p);
void glcd_rnx16_close(PrivateData *p);
void glcd_picolcdgfx_set_backlight(PrivateData *p, int state);
void glcd_picolcdgfx_set_contrast(PrivateData *p, int value);

/* Local functions */
void spi_send(PrivateData *p, unsigned char c, bool cmd) {
	CT_picolcdgfx_data *ct_data = (CT_picolcdgfx_data *) p->ct_data;
	unsigned char mask = 0x80;

	gpiod_line_set_value(ct_data->cs, 0);
	gpiod_line_set_value(ct_data->dc, !cmd);

	while (mask) {
		gpiod_line_set_value(ct_data->sclk, 0);
		gpiod_line_set_value(ct_data->sdin, !!(mask & c));
		gpiod_line_set_value(ct_data->sclk, 1);
		mask >>= 1;
	}
	gpiod_line_set_value(ct_data->cs, 1);
	gpiod_line_set_value(ct_data->dc, 1);
}

void spi_send_data(PrivateData *p, unsigned char d) {
	spi_send(p, d, false);
}

void spi_send_cmd(PrivateData *p, unsigned char c) {
	spi_send(p, c, true);
}


/**
 * API: Initialize the connection type driver.
 * \param drvthis  Pointer to driver structure.
 * \retval 0       Success.
 * \retval <0      Error.
 */
int
glcd_rnx16_init(Driver *drvthis)
{
	PrivateData *p = (PrivateData *) drvthis->private_data;
	CT_picolcdgfx_data *ct_data;

	char driver[1024];
	char product[1024];
	char manufacturer[1024];
	char serialnumber[1024];
	int ret;

	report(RPT_INFO, "GLCD/rnx16: intializing");

	/* Set up connection type low-level functions */
	p->glcd_functions->blit = glcd_rnx16_blit;
	p->glcd_functions->close = glcd_rnx16_close;
	// p->glcd_functions->set_backlight = glcd_picolcdgfx_set_backlight;
	// p->glcd_functions->set_contrast = glcd_picolcdgfx_set_contrast;

	/* Allocate memory structures */
	ct_data = (CT_picolcdgfx_data *) calloc(1, sizeof(CT_picolcdgfx_data));
	if (ct_data == NULL) {
		report(RPT_ERR, "GLCD/rnx16: error allocating connection data");
		return -1;
	}
	p->ct_data = ct_data;

	/* Fix display size to 128x64 */
	p->framebuf.layout = FB_TYPE_VPAGED;
	p->framebuf.px_width = RNX16_WIDTH;
	p->framebuf.px_height = RNX16_HEIGHT;

	/* Since the display is fixed to 256x64 we have to recalculate. */
	p->framebuf.size = (RNX16_HEIGHT / 8) * RNX16_WIDTH;

	ct_data->backingstore = malloc(p->framebuf.size);
	if (ct_data->backingstore == NULL) {
		report(RPT_ERR, "GLCD/rnx16: unable to allocate backing store");
		return -1;
	}

	/* framebuf is initialized with 0x00 so initialize the backingstore with
	 * 0xFF so the first call to _blit will draw the entire screen.
	 * */
	memset(ct_data->backingstore, 0xFF, p->framebuf.size);

	// /* Get inverted option */
	// if (drvthis->config_get_bool(drvthis->name, "picolcdgfx_Inverted", 0, PICOLCDGFX_DEF_INVERTED))
	// 	ct_data->inverted = 0xFF;
	// else
	ct_data->inverted = 0;

	report(RPT_DEBUG, "GLCD/rnx16: scanning for rnx16 128x64...");

	const char *chipname = "gpiochip0";
	ct_data->chip = gpiod_chip_open_by_name(chipname);
	if (!ct_data->chip) {
		report(RPT_ERR, "GLCD/rnx16: could not open libgpiod");
		return -1;
	}
	ct_data->sdin = gpiod_chip_get_line(ct_data->chip, 54);
	ct_data->sclk = gpiod_chip_get_line(ct_data->chip, 52);
	ct_data->dc = gpiod_chip_get_line(ct_data->chip, 32);
	ct_data->cs = gpiod_chip_get_line(ct_data->chip, 50);
	ct_data->ctrl = gpiod_chip_get_line(ct_data->chip, 6);
	ct_data->reset = gpiod_chip_get_line(ct_data->chip, 7);

	gpiod_line_request_output(ct_data->sdin, GPIO_CONSUMER, 1);
	gpiod_line_request_output(ct_data->sclk, GPIO_CONSUMER, 1);
	gpiod_line_request_output(ct_data->dc, GPIO_CONSUMER, 1);
	gpiod_line_request_output(ct_data->cs, GPIO_CONSUMER, 1);
	gpiod_line_request_output(ct_data->ctrl, GPIO_CONSUMER, 1);
	gpiod_line_request_output(ct_data->reset, GPIO_CONSUMER, 1);

	report(RPT_DEBUG, "%s: init() done", drvthis->name);

	return 0;

	report(RPT_ERR, "GLCD/rnx16: could not find a rnx16");
	return -1;
}

/**
 * API: Write the framebuffer to the display
 * \param p  Pointer to glcd driver's private date structure.
 */
void
glcd_rnx16_blit(PrivateData *p)
{
	report(RPT_DEBUG, "%s()", __FUNCTION__);

	CT_picolcdgfx_data *ct_data = (CT_picolcdgfx_data *) p->ct_data;

	int offset;
	int index;
	unsigned char page, line;

	unsigned int xpix = 4;

	// 128 * 64 = 32 * 8?
	
	spi_send_cmd(p, 0xA6);
	spi_send_cmd(p, 0x40);

	for (page = 0; page < 2; page ++) {
		offset = page;
		int i;
		gpiod_line_set_value(ct_data->cs, 0);
		gpiod_line_set_value(ct_data->dc, 0);

		spi_send_cmd(p, 0xb0 + page);
		spi_send_cmd(p, (xpix >> 4) | 0x10);
		spi_send_cmd(p, xpix & 0xf);

		gpiod_line_set_value(ct_data->dc, 1);

		for (i = 0; i < 128; i++) {
			spi_send_data(p, *((p->framebuf.data) + (128 * page) + i));
		}
	}

	// for (cs = 0; cs < 4; cs++) {
	// 	unsigned char chipsel = (cs << 2);
	// 	for (line = 0; line < 8; line++) {
	// 		offset = line * RNX16_WIDTH + cs * 64;
	// 		if (memcmp((p->framebuf.data) + offset, (ct_data->backingstore) + offset, 64) == 0)
	// 			continue;

	// 		cmd3[0] = PICOLCDGFX_OUT_CMD_DATA;
	// 		cmd3[1] = chipsel;
	// 		cmd3[2] = 0x02;
	// 		cmd3[3] = 0x00;
	// 		cmd3[4] = 0x00;
	// 		cmd3[5] = 0xb8 | line;
	// 		cmd3[6] = 0x00;
	// 		cmd3[7] = 0x00;
	// 		cmd3[8] = 0x40;
	// 		cmd3[9] = 0x00;
	// 		cmd3[10] = 0x00;
	// 		cmd3[11] = 32;

	// 		cmd4[0] = PICOLCDGFX_OUT_DATA;
	// 		cmd4[1] = chipsel | 0x01;
	// 		cmd4[2] = 0x00;
	// 		cmd4[3] = 0x00;
	// 		cmd4[4] = 32;

	// 		for (index = 0; index < 32; index++) {
	// 			cmd3[12 + index] = *((p->framebuf.data) + offset + index) ^ ct_data->inverted;
	// 		}

	// 		for (index = 32; index < 64; index++) {
	// 			cmd4[5 + (index - 32)] = *((p->framebuf.data) + offset + index) ^ ct_data->inverted;
	// 		}

	// 		// picolcdgfx_write(ct_data->lcd, cmd3, 44);
	// 		// picolcdgfx_write(ct_data->lcd, cmd4, 37);
	// 	}
	// }

	memcpy(ct_data->backingstore, p->framebuf.data, p->framebuf.size);
}

/**
 * API: Release low-level resources.
 */
void
glcd_rnx16_close(PrivateData *p)
{
	if (p->ct_data != NULL) {
		CT_picolcdgfx_data *ct_data = (CT_picolcdgfx_data *) p->ct_data;

		if (ct_data->chip != NULL) {
			gpiod_chip_close(ct_data->chip);
		}

		if (ct_data->backingstore != NULL)
			free(ct_data->backingstore);

		free(p->ct_data);
		p->ct_data = NULL;
	}
}



/**
 * API: Set the backlight brightness.
 */
void
glcd_picolcdgfx_set_backlight(PrivateData *p, int state)
{
	CT_picolcdgfx_data *ct_data = (CT_picolcdgfx_data *) p->ct_data;
	int promille = (state == BACKLIGHT_ON) ? p->brightness : p->offbrightness;
	unsigned char cmd[2];

	// cmd[0] = PICOLCDGFX_OUT_BACKLIGHT;
	// cmd[1] = (promille * 255 / 1000);

	// picolcdgfx_write(ct_data->lcd, cmd, 2);
}


/**
 * API: Change LCD contrast.
 */
void
glcd_picolcdgfx_set_contrast(PrivateData *p, int value)
{
	CT_picolcdgfx_data *ct_data = (CT_picolcdgfx_data *) p->ct_data;
	// unsigned char cmd[2] = {PICOLCDGFX_OUT_CONTRAST};

	/* I believe hardware contrast values are 200 to 255.  Where a higher
	 * contrast setting sends a lower value to the deivce. (S. Meharg) */
	unsigned char val = ((1000 - value) * (255 - 200)) / 1000 + 200;

	// cmd[1] = val;
	// picolcdgfx_write(ct_data->lcd, cmd, 2);
}
