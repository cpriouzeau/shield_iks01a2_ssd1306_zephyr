/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <string.h>
#include <sys/util.h>

#include <display/cfb.h>
#if defined(CONFIG_SSD1306)
#define DISPLAY_DRIVER		"SSD1306"
#endif


void main(void)
{
	struct sensor_value temp1, temp2, hum, press;
	struct sensor_value accel1[3], accel2[3];
	struct sensor_value gyro[3];
	struct sensor_value magn[3];
	struct device *hts221 = device_get_binding(DT_INST_0_ST_HTS221_LABEL);
	struct device *lps22hb = device_get_binding(DT_INST_0_ST_LPS22HB_PRESS_LABEL);
	struct device *lsm6dsl = device_get_binding(DT_INST_0_ST_LSM6DSL_LABEL);
	struct device *lsm303agr_a = device_get_binding(DT_INST_0_ST_LIS2DH_LABEL);
	struct device *lsm303agr_m = device_get_binding(DT_INST_0_ST_LIS2MDL_MAGN_LABEL);
	bool hts221_available = true;
	bool lps22hb_available = true;
	bool lsm6dsl_available = true;
	bool lsm303agr_a_available = true;
	bool lsm303agr_m_available = true;
	char display_buf[16];
	u8_t line = 0;

	struct device *dev_display;
	u16_t display_rows;
	u8_t display_ppt;
	u8_t display_font_width;
	u8_t display_font_height;

	bool display_available = false;

	if (hts221 == NULL) {
		printf("Could not get HTS221 device\n");
		hts221_available = false;
	}
	if (lps22hb == NULL) {
		printf("Could not get LPS22HB device\n");
		lps22hb_available = false;
	}
	if (lsm6dsl == NULL) {
		printf("Could not get LSM6DSL device\n");
		lsm6dsl_available = false;
	}
	if (lsm303agr_a == NULL) {
		printf("Could not get LSM303AGR Accel device\n");
		lsm303agr_a_available = false;
	}
	if (lsm303agr_m == NULL) {
		printf("Could not get LSM303AGR Magn device\n");
		lsm303agr_m_available = false;
	}

	/* set LSM6DSL accel/gyro sampling frequency to 104 Hz */
	struct sensor_value odr_attr;

	odr_attr.val1 = 104;
	odr_attr.val2 = 0;

	if (lsm6dsl_available && sensor_attr_set(lsm6dsl, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
		return;
	}

	if (lsm6dsl_available && sensor_attr_set(lsm6dsl, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for gyro.\n");
		return;
	}

	/* init display */
	dev_display = device_get_binding(DISPLAY_DRIVER);
	if (dev_display == NULL) {
		printf("Display Device not found\n");
	} else {
		display_available = true;

		if (display_set_pixel_format(dev_display, PIXEL_FORMAT_MONO10) != 0) {
			printf("Failed to set required pixel format PIXEL_FORMAT_MONO10\n");
			return;
		}
		if (cfb_framebuffer_init(dev_display)) {
			printf("Framebuffer initialization failed!\n");
			return;
		}
		cfb_framebuffer_clear(dev_display, true);

		//display_blanking_off(dev_display);

		display_rows = cfb_get_display_parameter(dev_display, CFB_DISPLAY_ROWS);
		display_ppt = cfb_get_display_parameter(dev_display, CFB_DISPLAY_PPT);
		for (int idx = 0; idx < 42; idx++) {
			if (cfb_get_font_size(dev_display, idx, &display_font_width, &display_font_height)) {
				break;
			}
			cfb_framebuffer_set_font(dev_display, idx);
			printf("font width %d, font height %d\n",
					display_font_width, display_font_height);
		}
		printf("x_res %d, y_res %d, ppt %d, rows %d, cols %d\n",
			cfb_get_display_parameter(dev_display, CFB_DISPLAY_WIDTH),
			cfb_get_display_parameter(dev_display, CFB_DISPLAY_HEIGH),
			display_ppt,
			display_rows,
			cfb_get_display_parameter(dev_display, CFB_DISPLAY_COLS));

		}

	while (1) {
		int ret;
		line = 0;

		if (display_available)
			cfb_framebuffer_clear(dev_display, false);

		/* Get sensor samples */

		if (hts221_available && sensor_sample_fetch(hts221) < 0) {
			printf("HTS221 Sensor sample update error\n");
		}
		if (lps22hb_available && sensor_sample_fetch(lps22hb) < 0) {
			printf("LPS22HB Sensor sample update error\n");
		}
		if (lsm303agr_a_available && lsm6dsl_available && sensor_sample_fetch(lsm6dsl) < 0) {
			printf("LSM6DSL Sensor sample update error\n");
		}
		if (lsm303agr_a_available) {
			ret = sensor_sample_fetch(lsm303agr_a);
			if (ret < 0 && ret != -EBADMSG) {
				printf("LSM303AGR Accel Sensor sample update error\n");
			}
		}
		if (lsm303agr_m_available && sensor_sample_fetch(lsm303agr_m) < 0) {
			printf("LSM303AGR Magn Sensor sample update error\n");
		}

		/* Get sensor data */

		if (hts221_available) {
			sensor_channel_get(hts221, SENSOR_CHAN_AMBIENT_TEMP, &temp1);
			sensor_channel_get(hts221, SENSOR_CHAN_HUMIDITY, &hum);
		}
		if (lps22hb_available) {
			sensor_channel_get(lps22hb, SENSOR_CHAN_PRESS, &press);
			sensor_channel_get(lps22hb, SENSOR_CHAN_AMBIENT_TEMP, &temp2);
		}
		if (lsm6dsl_available) {
			sensor_channel_get(lsm6dsl, SENSOR_CHAN_ACCEL_XYZ, accel1);
			sensor_channel_get(lsm6dsl, SENSOR_CHAN_GYRO_XYZ, gyro);
		}
		if (lsm303agr_a_available)
			sensor_channel_get(lsm303agr_a, SENSOR_CHAN_ACCEL_XYZ, accel2);
		if (lsm303agr_m_available)
			sensor_channel_get(lsm303agr_m, SENSOR_CHAN_MAGN_XYZ, magn);

		/* Display sensor data */

		/* Erase previous */
/*		printf("\0033\014");*/

		printf("X-NUCLEO-IKS01A2 sensor dashboard\n\n");

		if (display_available)
			cfb_print(dev_display, "Zephyr", 0, line++*display_font_height);


		if (hts221_available) {
			/* temperature */
			printf("HTS221: Temperature: %.1f C\n",
				sensor_value_to_double(&temp1));

			/* humidity */
			printf("HTS221: Relative Humidity: %.1f%%\n",
					sensor_value_to_double(&hum));
			snprintf(display_buf, sizeof(display_buf), "T: %1f C",
					sensor_value_to_double(&temp1));
			if (display_available)
				cfb_print(dev_display, display_buf, 0, line++*display_font_height);

			snprintf(display_buf, sizeof(display_buf), "H: %1f%%",
					sensor_value_to_double(&hum));
			if (display_available)
				cfb_print(dev_display, display_buf, 0, line++*display_font_height);

		}

		if (lps22hb_available) {
			/* pressure */
			printf("LPS22HB: Pressure:%.3f kpa\n",
					sensor_value_to_double(&press));

			/* lps22hb temperature */
			printf("LPS22HB: Temperature: %.1f C\n",
					sensor_value_to_double(&temp2));

			snprintf(display_buf, sizeof(display_buf), "P: %3f kpa",
					sensor_value_to_double(&press));
			if (display_available)
				cfb_print(dev_display, display_buf, 0, line++*display_font_height);
			snprintf(display_buf, sizeof(display_buf), "T: %1fC",
					sensor_value_to_double(&temp2));
/*            if (display_available)*/
/*                cfb_print(dev_display, display_buf, 0, line++*display_font_height);*/

		}

		if (lsm6dsl_available) {
			/* lsm6dsl accel */
			printf("LSM6DSL: Accel (m.s-2): x: %.1f, y: %.1f, z: %.1f\n",
				sensor_value_to_double(&accel1[0]),
				sensor_value_to_double(&accel1[1]),
				sensor_value_to_double(&accel1[2]));

			/* lsm6dsl gyro */
			printf("LSM6DSL: Gyro (dps): x: %.3f, y: %.3f, z: %.3f\n",
				sensor_value_to_double(&gyro[0]),
				sensor_value_to_double(&gyro[1]),
				sensor_value_to_double(&gyro[2]));
		}

		if (lsm303agr_a) {
			/* lsm303agr accel */
			printf("LSM303AGR: Accel (m.s-2): x: %.1f, y: %.1f, z: %.1f\n",
				sensor_value_to_double(&accel2[0]),
				sensor_value_to_double(&accel2[1]),
				sensor_value_to_double(&accel2[2]));
		}

		if (lsm303agr_m) {
			/* lsm303agr magn */
			printf("LSM303AGR: Magn (gauss): x: %.3f, y: %.3f, z: %.3f\n",
				sensor_value_to_double(&magn[0]),
				sensor_value_to_double(&magn[1]),
				sensor_value_to_double(&magn[2]));
		}

		if (display_available)
			cfb_framebuffer_finalize(dev_display);

		k_sleep(2000);
	}
}
