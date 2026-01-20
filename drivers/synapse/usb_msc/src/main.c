/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB Mass Storage driver for SD card log file retrieval.
 * Auto-enables when USB is plugged in (while disarmed).
 * Blocks arming while USB is connected.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>
#include <synapse_topic_list.h>

#if defined(CONFIG_CEREBRI_SYNAPSE_LOG_SDCARD)
#include <log_sdcard.h>
#endif

#include <usb_msc.h>

LOG_MODULE_REGISTER(usb_msc, CONFIG_CEREBRI_SYNAPSE_USB_MSC_LOG_LEVEL);

/* Define the SD card as a USB MSC LUN */
USBD_DEFINE_MSC_LUN(SD, "SD", "CogniPilot", "VMU SD Card", "1.00");

/* USB device vendor/product IDs */
#define COGNIPILOT_USB_VID 0x2fe3 /* Zephyr project VID for samples */
#define COGNIPILOT_USB_PID 0x000A /* Mass storage PID */

/* Timeout for detecting USB disconnect after suspend (ms) */
#define USB_DISCONNECT_TIMEOUT_MS 2000

/* USB device context */
USBD_DEVICE_DEFINE(usb_msc_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), COGNIPILOT_USB_VID,
		   COGNIPILOT_USB_PID);

/* USB string descriptors */
USBD_DESC_LANG_DEFINE(usb_msc_lang);
USBD_DESC_MANUFACTURER_DEFINE(usb_msc_mfr, "CogniPilot");
USBD_DESC_PRODUCT_DEFINE(usb_msc_product, "VMU SD Card");
USBD_DESC_CONFIG_DEFINE(usb_msc_fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(usb_msc_hs_cfg_desc, "HS Configuration");

/* USB configurations - 250 = 500mA max power */
USBD_CONFIGURATION_DEFINE(usb_msc_fs_config, 0, 250, &usb_msc_fs_cfg_desc);
USBD_CONFIGURATION_DEFINE(usb_msc_hs_config, 0, 250, &usb_msc_hs_cfg_desc);

/* State tracking */
static bool usb_msc_initialized;
static bool usb_msc_enabled;
static bool usb_vbus_detected;  /* True when USB cable is plugged in (VBUS present) */
static bool usb_host_connected; /* True when USB host has configured us */
static bool usb_suspended;      /* True when in USB suspend state */

/* Arming state from status topic */
static bool system_armed;

/* Timer for detecting disconnect after suspend */
static struct k_work_delayable usb_disconnect_work;

/* ZROS for arming state */
static struct zros_node g_node;
static struct zros_sub g_sub_status;
static synapse_pb_Status g_status;

/* Forward declarations */
static int usb_msc_do_enable(void);
static int usb_msc_do_disable(void);

static int usb_msc_setup(void)
{
	int err;

	if (usb_msc_initialized) {
		return 0;
	}

	err = usbd_add_descriptor(&usb_msc_usbd, &usb_msc_lang);
	if (err) {
		LOG_ERR("Failed to add language descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&usb_msc_usbd, &usb_msc_mfr);
	if (err) {
		LOG_ERR("Failed to add manufacturer descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&usb_msc_usbd, &usb_msc_product);
	if (err) {
		LOG_ERR("Failed to add product descriptor (%d)", err);
		return err;
	}

	/* Check if device supports high speed */
	if (usbd_caps_speed(&usb_msc_usbd) == USBD_SPEED_HS) {
		err = usbd_add_configuration(&usb_msc_usbd, USBD_SPEED_HS, &usb_msc_hs_config);
		if (err) {
			LOG_ERR("Failed to add HS configuration (%d)", err);
			return err;
		}

		err = usbd_register_all_classes(&usb_msc_usbd, USBD_SPEED_HS, 1, NULL);
		if (err) {
			LOG_ERR("Failed to register HS classes (%d)", err);
			return err;
		}

		usbd_device_set_code_triple(&usb_msc_usbd, USBD_SPEED_HS, 0, 0, 0);
	}

	err = usbd_add_configuration(&usb_msc_usbd, USBD_SPEED_FS, &usb_msc_fs_config);
	if (err) {
		LOG_ERR("Failed to add FS configuration (%d)", err);
		return err;
	}

	err = usbd_register_all_classes(&usb_msc_usbd, USBD_SPEED_FS, 1, NULL);
	if (err) {
		LOG_ERR("Failed to register FS classes (%d)", err);
		return err;
	}

	usbd_device_set_code_triple(&usb_msc_usbd, USBD_SPEED_FS, 0, 0, 0);

	err = usbd_init(&usb_msc_usbd);
	if (err) {
		LOG_ERR("Failed to initialize USB device (%d)", err);
		return err;
	}

	usb_msc_initialized = true;
	LOG_INF("USB MSC initialized");

	return 0;
}

/* Work handler for detecting disconnect after suspend timeout */
static void usb_disconnect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (usb_suspended && usb_host_connected) {
		LOG_INF("USB host disconnected (suspend timeout)");
		usb_host_connected = false;
		/* Disable USB MSC when host disconnects */
		if (usb_msc_enabled) {
			usb_msc_do_disable();
		}
	}
}

/* Internal function to enable USB MSC */
static int usb_msc_do_enable(void)
{
	int ret;

	if (usb_msc_enabled) {
		return -EALREADY;
	}

	/* Don't allow enabling while armed */
	if (system_armed) {
		LOG_WRN("Cannot enable USB MSC while armed");
		return -EBUSY;
	}

	/* Setup USB device if not already done */
	ret = usb_msc_setup();
	if (ret) {
		LOG_ERR("Failed to setup USB MSC: %d", ret);
		return ret;
	}

#if defined(CONFIG_CEREBRI_SYNAPSE_LOG_SDCARD)
	/* Stop SD card logging before enabling USB MSC */
	if (log_sdcard_is_running()) {
		LOG_INF("Stopping SD card logging for USB MSC...");
		log_sdcard_stop();
		k_msleep(100);
	}
	if (log_sdcard_writer_is_running()) {
		log_sdcard_writer_stop();
		k_msleep(500);
	}
#endif

	/* Reinitialize disk to ensure it's accessible */
	ret = disk_access_init("SD");
	if (ret) {
		LOG_ERR("Failed to init disk: %d", ret);
		return ret;
	}

	/* Give disk time to initialize */
	k_msleep(100);

	/* Verify disk is accessible */
	ret = disk_access_status("SD");
	if (ret != DISK_STATUS_OK) {
		LOG_ERR("Disk not ready, status: %d", ret);
		return -EIO;
	}

	/* Log disk info for debugging */
	uint32_t sector_count = 0;
	uint32_t sector_size = 0;
	disk_access_ioctl("SD", DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
	disk_access_ioctl("SD", DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
	LOG_INF("SD card ready: %u sectors, %u bytes/sector", sector_count, sector_size);

	/* Enable USB */
	ret = usbd_enable(&usb_msc_usbd);
	if (ret) {
		LOG_ERR("Failed to enable USB: %d", ret);
		return ret;
	}

	usb_msc_enabled = true;
	return 0;
}

/* Internal function to disable USB MSC */
static int usb_msc_do_disable(void)
{
	int ret;

	if (!usb_msc_enabled) {
		return -EALREADY;
	}

	ret = usbd_disable(&usb_msc_usbd);
	if (ret) {
		LOG_ERR("Failed to disable USB: %d", ret);
		return ret;
	}

	usb_msc_enabled = false;
	usb_host_connected = false;
	usb_suspended = false;
	LOG_INF("USB MSC disabled");

	return 0;
}

/* USB device message callback */
static void usb_msc_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *const msg)
{
	ARG_UNUSED(ctx);

	switch (msg->type) {
	case USBD_MSG_VBUS_READY:
		LOG_INF("USB VBUS detected (cable connected)");
		usb_vbus_detected = true;
		/* Auto-enable USB MSC when cable plugged in (if disarmed) */
		if (!system_armed && !usb_msc_enabled) {
			usb_msc_do_enable();
		}
		break;

	case USBD_MSG_VBUS_REMOVED:
		LOG_INF("USB VBUS removed (cable disconnected)");
		usb_vbus_detected = false;
		usb_host_connected = false;
		usb_suspended = false;
		k_work_cancel_delayable(&usb_disconnect_work);
		/* Disable USB MSC when cable unplugged */
		if (usb_msc_enabled) {
			usb_msc_do_disable();
		}
		break;

	case USBD_MSG_RESET:
		LOG_DBG("USB bus reset");
		usb_suspended = false;
		k_work_cancel_delayable(&usb_disconnect_work);
		break;

	case USBD_MSG_CONFIGURATION:
		LOG_INF("USB host connected and configured");
		usb_host_connected = true;
		usb_suspended = false;
		k_work_cancel_delayable(&usb_disconnect_work);
		/* Auto-enable USB MSC when host connects (if disarmed) */
		if (!system_armed && !usb_msc_enabled) {
			usb_msc_do_enable();
		}
		break;

	case USBD_MSG_SUSPEND:
		LOG_DBG("USB suspended");
		usb_suspended = true;
		/* Start timer to detect actual disconnect */
		k_work_schedule(&usb_disconnect_work, K_MSEC(USB_DISCONNECT_TIMEOUT_MS));
		break;

	case USBD_MSG_RESUME:
		LOG_DBG("USB resumed");
		usb_suspended = false;
		k_work_cancel_delayable(&usb_disconnect_work);
		break;

	default:
		break;
	}
}

/* Shell command handlers */
static int cmd_usb_msc_enable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (system_armed) {
		shell_error(sh, "Cannot enable USB MSC while armed");
		return -EBUSY;
	}

	int ret = usb_msc_do_enable();
	if (ret == -EALREADY) {
		shell_print(sh, "USB MSC already enabled");
		return 0;
	} else if (ret) {
		shell_error(sh, "Failed to enable USB MSC: %d", ret);
		return ret;
	}

	shell_print(sh, "USB MSC enabled - connect USB cable to access SD card");
	return 0;
}

static int cmd_usb_msc_disable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int ret = usb_msc_do_disable();
	if (ret == -EALREADY) {
		shell_print(sh, "USB MSC not enabled");
		return 0;
	} else if (ret) {
		shell_error(sh, "Failed to disable USB MSC: %d", ret);
		return ret;
	}

	shell_print(sh, "USB MSC disabled");
	return 0;
}

static int cmd_usb_msc_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "USB MSC Status:");
	shell_print(sh, "  Initialized: %s", usb_msc_initialized ? "yes" : "no");
	shell_print(sh, "  Enabled: %s", usb_msc_enabled ? "yes" : "no");
	shell_print(sh, "  VBUS detected: %s", usb_vbus_detected ? "yes" : "no");
	shell_print(sh, "  Host connected: %s", usb_host_connected ? "yes" : "no");
	shell_print(sh, "  System armed: %s", system_armed ? "yes" : "no");
	shell_print(sh, "  Arming blocked: %s", usb_msc_is_connected() ? "YES" : "no");

	/* Check SD card at block device level */
	int ret = disk_access_init("SD");
	if (ret == 0) {
		uint32_t sector_count = 0;
		uint32_t sector_size = 0;
		disk_access_ioctl("SD", DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
		disk_access_ioctl("SD", DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
		uint32_t size_mb = (uint32_t)(((uint64_t)sector_count * sector_size) >> 20);
		shell_print(sh, "  SD card: %u MB (%u sectors)", size_mb, sector_count);
	} else {
		shell_print(sh, "  SD card: not detected");
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	usb_msc_subcmds,
	SHELL_CMD(enable, NULL, "Enable USB Mass Storage mode", cmd_usb_msc_enable),
	SHELL_CMD(disable, NULL, "Disable USB Mass Storage mode", cmd_usb_msc_disable),
	SHELL_CMD(status, NULL, "Show USB MSC status", cmd_usb_msc_status), SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(usb_msc, &usb_msc_subcmds, "USB Mass Storage commands", NULL);

/* Public API functions */
bool usb_msc_is_connected(void)
{
	/* Return true if USB cable is plugged in (VBUS detected) or host connected */
	return usb_vbus_detected || usb_host_connected;
}

bool usb_msc_is_enabled(void)
{
	return usb_msc_enabled;
}

int usb_msc_enable(void)
{
	return usb_msc_do_enable();
}

int usb_msc_disable(void)
{
	return usb_msc_do_disable();
}

/* Thread to monitor arming state */
#define USB_MSC_MONITOR_STACK_SIZE 2048
#define USB_MSC_MONITOR_PRIORITY   6

static K_THREAD_STACK_DEFINE(g_monitor_stack, USB_MSC_MONITOR_STACK_SIZE);
static struct k_thread g_monitor_thread;

static void usb_msc_monitor_run(void *p0, void *p1, void *p2)
{
	ARG_UNUSED(p0);
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	/* Initialize ZROS subscription */
	zros_node_init(&g_node, "usb_msc_monitor");
	zros_sub_init(&g_sub_status, &g_node, &topic_status, &g_status, 10);

	LOG_INF("USB MSC monitor started");

	while (true) {
		/* Poll for status updates */
		struct k_poll_event events[] = {
			*zros_sub_get_event(&g_sub_status),
		};

		int rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
		if (rc == 0 && zros_sub_update_available(&g_sub_status)) {
			zros_sub_update(&g_sub_status);

			/* Update arming state */
			bool was_armed = system_armed;
			system_armed = (g_status.arming == synapse_pb_Status_Arming_ARMING_ARMED);

			/* If just disarmed and USB host is connected, enable USB MSC */
			if (was_armed && !system_armed && usb_host_connected && !usb_msc_enabled) {
				LOG_INF("Disarmed with USB connected - enabling USB MSC");
				usb_msc_do_enable();
			}
		}
	}
}

/* Initialize USB MSC */
static int usb_msc_boot_init(void)
{
	int ret;

	/* Initialize disconnect detection work */
	k_work_init_delayable(&usb_disconnect_work, usb_disconnect_work_handler);

	/* Setup USB device (but don't enable yet) */
	ret = usb_msc_setup();
	if (ret) {
		LOG_ERR("Failed to setup USB MSC during init: %d", ret);
		return ret;
	}

	/* Register message callback for connection detection */
	ret = usbd_msg_register_cb(&usb_msc_usbd, usb_msc_msg_cb);
	if (ret) {
		LOG_ERR("Failed to register USB message callback: %d", ret);
		return ret;
	}

	/* Enable USB MSC at boot (includes stopping SD logging and preparing disk) */
	ret = usb_msc_do_enable();
	if (ret && ret != -EALREADY) {
		LOG_ERR("Failed to enable USB MSC at boot: %d", ret);
		return ret;
	}
	LOG_INF("USB MSC enabled at boot");

	/* Start monitor thread for arming state */
	k_tid_t tid = k_thread_create(&g_monitor_thread, g_monitor_stack,
				      USB_MSC_MONITOR_STACK_SIZE, usb_msc_monitor_run, NULL, NULL,
				      NULL, USB_MSC_MONITOR_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(tid, "usb_msc_monitor");

	LOG_INF("USB MSC ready - will auto-enable when USB connected and disarmed");
	return 0;
}

SYS_INIT(usb_msc_boot_init, APPLICATION, 50);
