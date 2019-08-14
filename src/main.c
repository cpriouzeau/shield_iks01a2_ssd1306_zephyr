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

/* Display / Framebuffer */
#include <display/cfb.h>
#if defined(CONFIG_SSD1306)
#define DISPLAY_DRIVER		"SSD1306"
#endif

static struct sensor_value temp1, temp2, hum, press;
static struct sensor_value accel1[3], accel2[3];
static struct sensor_value gyro[3];
static struct sensor_value magn[3];
static struct device *hts221;
static struct device *lps22hb;
static struct device *lsm6dsl;
static struct device *lsm303agr_a;
static struct device *lsm303agr_m;
static bool hts221_available = true;
static bool lps22hb_available = true;
static bool lsm6dsl_available = true;
static bool lsm303agr_a_available = true;
static bool lsm303agr_m_available = true;
static struct device *dev_display;
static u16_t display_rows;
static u8_t display_ppt;
static u8_t display_font_width;
static u8_t display_font_height;
static bool display_available = false;

void init_sensors();
void init_display();
void stop_display();

#define _USE_OPENAMP 1

/* openAMP */
#ifdef _USE_OPENAMP
#include <ipm.h>

#include <openamp/open_amp.h>
#include <metal/device.h>
#include <resource_table.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(zephyr_shield_openamp_tty, LOG_LEVEL_DBG);

#define RPMSG_CHAN_NAME	"rpmsg-tty-channel"
#define SHM_DEVICE_NAME	"shm"

/* constant derivated from linker symbols */
#define SHM_START_ADDR	DT_IPC_SHM_BASE_ADDRESS
#define SHM_SIZE	(DT_IPC_SHM_SIZE * 1024)

#define APP_TASK_STACK_SIZE (1024)
K_THREAD_STACK_DEFINE(thread_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_data;

static struct device *ipm_handle;

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;

struct metal_device shm_device = {
	.name = SHM_DEVICE_NAME,
	.num_regions = 2,
	.regions = {
		{.virt = NULL}, /* shared memory */
		{.virt = NULL}, /* rsc_table memory */
	},
	.node = { NULL },
	.irq_num = 0,
	.irq_info = NULL
};

static struct metal_io_region *shm_io;
static struct rpmsg_virtio_shm_pool shpool;

static struct metal_io_region *rsc_io;
static struct rpmsg_virtio_device rvdev;

static void *rsc_table;

static char rcv_msg[20];  /* should receive "Hello world!" */
static unsigned int rcv_len;
static struct rpmsg_endpoint rcv_ept;

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_rx_sem, 0, 1);

static bool first_data = true;

static void platform_ipm_callback(void *context, u32_t id, volatile void *data)
{
	printf("%s: msg received from mb %d\n", __func__, id);

	if (2 == id) {
		/* receive shutdown */
		stop_display();
	}

	if (first_data)
		k_sem_give(&data_sem);
}

static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data,
			       size_t len, uint32_t src, void *priv)
{
	memcpy(rcv_msg, data, len);
	rcv_len = len;
	printf("--rpmsg_recv--\n");
	k_sem_give(&data_rx_sem);

	return RPMSG_SUCCESS;
}

static void receive_message(unsigned char **msg, unsigned int *len)
{
	while (k_sem_take(&data_rx_sem, K_NO_WAIT) != 0) {
		int status;
		if (first_data) {
			status = k_sem_take(&data_sem, K_FOREVER);
			first_data = false;
		} else {
			status = 0;
		}

		if (status == 0) {
			rproc_virtio_notified(rvdev.vdev, VRING1_ID);
		}
	}
	*len = rcv_len;
	*msg = rcv_msg;
}
static void new_service_cb(struct rpmsg_device *rdev, const char *name,
			   uint32_t src)
{
	printf("%s: unexpected ns service receive for name %s\n",
		__func__, name);
}

int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);

	printf("%s: msg sent\n", __func__);
	ipm_send(ipm_handle, 0, id, NULL, 0);

	return 0;
}
int platform_init(void)
{
	void *rsc_tab_addr;
	int rsc_size;
	struct metal_device *device;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	int status;

	status = metal_init(&metal_params);
	if (status) {
		printf("metal_init: failed: %d\n", status);
		return -1;
	}

	status = metal_register_generic_device(&shm_device);
	if (status) {
		printf("Couldn't register shared memory: %d\n", status);
		return -1;
	}

	status = metal_device_open("generic", SHM_DEVICE_NAME, &device);
	if (status) {
		printf("metal_device_open failed: %d\n", status);
		return -1;
	}

	/* declare shared memory region */
	metal_io_init(&device->regions[0], (void *)SHM_START_ADDR, &shm_physmap,
		      SHM_SIZE, -1, 0, NULL);

	shm_io = metal_device_io_region(device, 0);
	if (!shm_io) {
		printf("Failed to get shm_io region\n");
		return -1;
	}

	/* declare resource table region */
	rsc_table_get(&rsc_tab_addr, &rsc_size);
	rsc_table = (struct st_resource_table *)rsc_tab_addr;

	metal_io_init(&device->regions[1], rsc_table,
		      (metal_phys_addr_t *)rsc_table, rsc_size, -1, 0, NULL);

	rsc_io = metal_device_io_region(device, 1);
	if (!rsc_io) {
		printf("Failed to get rsc_io region\n");
		return -1;
	}

	/* setup IPM */
	ipm_handle = device_get_binding(DT_IPM_DEV/*"MAILBOX_0"*/);
	if (!ipm_handle) {
		printf("Failed to find ipm device\n");
		return -1;
	}

	ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

	status = ipm_set_enabled(ipm_handle, 1);
	if (status) {
		printf("ipm_set_enabled failed\n");
		return -1;
	}

	return 0;
}

static void cleanup_system(void)
{
	ipm_set_enabled(ipm_handle, 0);
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
}

struct  rpmsg_device *
platform_create_rpmsg_vdev(unsigned int vdev_index,
			   unsigned int role,
			   void (*rst_cb)(struct virtio_device *vdev),
			   rpmsg_ns_bind_cb ns_cb)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_SLAVE, VDEV_ID,
					rsc_table_to_vdev(rsc_table),
					rsc_io, NULL, mailbox_notify, NULL);

	if (!vdev) {
		printf("failed to create vdev\r\n");
		return NULL;
	}

	/* wait master rpmsg init completion */
	rproc_virtio_wait_remote_ready(vdev);

	vring_rsc = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		printf("failed to init vring 0\r\n");
		goto failed;
	}

	vring_rsc = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		printf("failed to init vring 1\r\n");
		goto failed;
	}

	rpmsg_virtio_init_shm_pool(&shpool, NULL, SHM_SIZE);
	ret =  rpmsg_init_vdev(&rvdev, vdev, ns_cb, shm_io, &shpool);

	if (ret) {
		printf("failed rpmsg_init_vdev\r\n");
		goto failed;
	}

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	rproc_virtio_remove_vdev(vdev);

	return NULL;
}
#endif /* _USE_OPENAMP */

#ifdef _USE_OPENAMP

void tty_display_send(char *msg, int len) {
		rpmsg_send(&rcv_ept, msg, len);
}
void trace_send(char *msg) {
	//printf("%s", msg);
}
void app_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	struct rpmsg_device *rpdev;
	unsigned char *msg;
	int len;
	int ret = 0;
	char message[128];
	char display_buf[16];
	u8_t line = 0;

	printk("\r\nOpenAMP[remote]  linux responder demo started\r\n");

	/* Initialize platform */
	ret = platform_init();
	if (ret) {
		printf("Failed to initialize platform\n");
		ret = -1;
		goto task_end;
	}

	rpdev = platform_create_rpmsg_vdev(0, VIRTIO_DEV_SLAVE, NULL,
					   new_service_cb);
	if (!rpdev) {
		printf("Failed to create rpmsg virtio device\n");
		ret = -1;
		goto task_end;
	}

	ret = rpmsg_create_ept(&rcv_ept, rpdev, RPMSG_CHAN_NAME,
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_callback, NULL);
	if (ret != 0)
		printf("error while creating endpoint(%d)\n", ret);

	ipm_send(ipm_handle, 0, 0, NULL, 0);

	init_sensors();
	init_display();

	if (display_available) {
		line = 0;
		printk("--init display screen --\n");

		cfb_framebuffer_clear(dev_display, false);
		cfb_print(dev_display, "Zephyr", 0, line++*display_font_height);
		cfb_print(dev_display, "   Waiting ", 0, line++*display_font_height);
		cfb_print(dev_display, "     TTY   ", 0, line++*display_font_height);
		cfb_print(dev_display, "    data   ", 0, line++*display_font_height);
		cfb_framebuffer_finalize(dev_display);
	}

	receive_message(&msg, &len);

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

		trace_send("X-NUCLEO-IKS01A2 sensor dashboard\n\n");
		len = snprintf(message, sizeof(message), "X-NUCLEO-IKS01A2 sensor dashboard\n");
		tty_display_send(message, len);

		if (display_available)
			cfb_print(dev_display, "Zephyr", 0, line++*display_font_height);


		if (hts221_available) {
			/* temperature */
			len = snprintf(message, sizeof(message), "HTS221: Temperature: %.1f C\n",
				sensor_value_to_double(&temp1));

			trace_send(message);
			tty_display_send(message, len);

			/* humidity */
			len = snprintf(message, sizeof(message), "HTS221: Relative Humidity: %.1f%%\n",
					sensor_value_to_double(&hum));
			trace_send(message);
			tty_display_send(message, len);
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
			len = snprintf(message, sizeof(message), "LPS22HB: Pressure: %.3f hpa\n",
					sensor_value_to_double(&press)*10.0);
			trace_send(message);
			tty_display_send(message, len);

			/* lps22hb temperature */
			len = snprintf(message, sizeof(message), "LPS22HB: Temperature: %.1f C\n",
					sensor_value_to_double(&temp2));
			trace_send(message);
			tty_display_send(message, len);

			snprintf(display_buf, sizeof(display_buf), "P: %3f kpa",
					sensor_value_to_double(&press)*10.0);
			if (display_available)
				cfb_print(dev_display, display_buf, 0, line++*display_font_height);
			snprintf(display_buf, sizeof(display_buf), "T: %1fC",
					sensor_value_to_double(&temp2));
/*            if (display_available)*/
/*                cfb_print(dev_display, display_buf, 0, line++*display_font_height);*/
		}

		if (lsm6dsl_available) {
			/* lsm6dsl accel */
			len = snprintf(message, sizeof(message), "LSM6DSL: Accel (m.s-2): x: %.1f, y: %.1f, z: %.1f\n",
				sensor_value_to_double(&accel1[0]),
				sensor_value_to_double(&accel1[1]),
				sensor_value_to_double(&accel1[2]));
			trace_send(message);
			tty_display_send(message, len);

			/* lsm6dsl gyro */
			len = snprintf(message, sizeof(message), "LSM6DSL: Gyro (dps): x: %.3f, y: %.3f, z: %.3f\n",
				sensor_value_to_double(&gyro[0]),
				sensor_value_to_double(&gyro[1]),
				sensor_value_to_double(&gyro[2]));
			printf("%s", message);
			tty_display_send(message, len);
		}

		if (lsm303agr_a) {
			/* lsm303agr accel */
			len = snprintf(message, sizeof(message), "LSM303AGR: Accel (m.s-2): x: %.1f, y: %.1f, z: %.1f\n",
				sensor_value_to_double(&accel2[0]),
				sensor_value_to_double(&accel2[1]),
				sensor_value_to_double(&accel2[2]));
			trace_send(message);
			tty_display_send(message, len);
		}

		if (lsm303agr_m) {
			/* lsm303agr magn */
			len = snprintf(message, sizeof(message), "LSM303AGR: Magn (gauss): x: %.3f, y: %.3f, z: %.3f\n",
				sensor_value_to_double(&magn[0]),
				sensor_value_to_double(&magn[1]),
				sensor_value_to_double(&magn[2]));
			trace_send(message);
			tty_display_send(message, len);
		}

		if (display_available)
			cfb_framebuffer_finalize(dev_display);

		k_sleep(2000);
	}

	rpmsg_destroy_ept(&rcv_ept);

task_end:
	cleanup_system();

	printk("OpenAMP demo ended\n");
}

#endif /* DEMO_OPENAMP */

void init_sensors() {
	/* search in devicetree if sensors are referenced */
	hts221 = device_get_binding(DT_INST_0_ST_HTS221_LABEL);
	lps22hb = device_get_binding(DT_INST_0_ST_LPS22HB_PRESS_LABEL);
	lsm6dsl = device_get_binding(DT_INST_0_ST_LSM6DSL_LABEL);
	lsm303agr_a = device_get_binding(DT_INST_0_ST_LIS2DH_LABEL);
	lsm303agr_m = device_get_binding(DT_INST_0_ST_LIS2MDL_MAGN_LABEL);

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
}
void init_display() {
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

		display_blanking_off(dev_display);

		display_rows = cfb_get_display_parameter(dev_display, CFB_DISPLAY_ROWS);
		display_ppt = cfb_get_display_parameter(dev_display, CFB_DISPLAY_PPT);
		for (int idx = 0; idx < 42; idx++) {
			if (cfb_get_font_size(dev_display, idx, &display_font_width, &display_font_height)) {
				break;
			}
			cfb_framebuffer_set_font(dev_display, idx);
			printf("idx: %d font width %d, font height %d\n",
					idx, display_font_width, display_font_height);
		}
		// for idx: 0 font width 10, font height 16
		cfb_framebuffer_set_font(dev_display, 0);
		cfb_get_font_size(dev_display, 0, &display_font_width, &display_font_height);

		printf("x_res %d, y_res %d, ppt %d, rows %d, cols %d\n",
			cfb_get_display_parameter(dev_display, CFB_DISPLAY_WIDTH),
			cfb_get_display_parameter(dev_display, CFB_DISPLAY_HEIGH),
			display_ppt,
			display_rows,
			cfb_get_display_parameter(dev_display, CFB_DISPLAY_COLS));
	}
}
void stop_display() {
	display_blanking_on(dev_display);
}

void main(void)
{
#ifdef _USE_OPENAMP
	/* openAMP */
	printk("Starting application thread!\n");
	k_thread_create(&thread_data, thread_stack, APP_TASK_STACK_SIZE,
			(k_thread_entry_t)app_task,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, 0);
#else

	char display_buf[16];
	u8_t line = 0;

	init_sensors();
	init_display();

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
#endif /* DEMO_OPENAMP */
}
