/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/reboot.h>

#include <stdio.h>
#include <stdlib.h>

#include <cerebri/core/perf_duration.h>

LOG_MODULE_REGISTER(core_common, CONFIG_CEREBRI_CORE_COMMON_LOG_LEVEL);

#if defined(CONFIG_REBOOT)
void do_reboot()
{
	sys_reboot(SYS_REBOOT_WARM);
};

SHELL_CMD_REGISTER(reboot, &do_reboot, "reboot autopilot", NULL);
#endif

const struct device *get_device(const struct device *const dev)
{
	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		LOG_ERR("no device found");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		LOG_ERR("device %s is not ready, check the driver initialization logs for errors",
			dev->name);
		return NULL;
	}
	return dev;
}

#if defined(CONFIG_CEREBRI_CORE_COMMON_BOOT_BANNER)
const char *banner_brain =
	"\n"
	"                            \033[0m\033[38;5;252m              ▄▄▄▄▄▄▄▄\n"
	"\033[2;34m         ▄▄▄▄▄ \033[2;33m▄▄▄▄▄\033[0m\033[38;5;252m                    "
	"▀▀▀▀▀▀▀▀▀\n"
	"\033[2;34m     ▄███████▀\033[2;33m▄██████▄\033[0m\033[38;5;252m   "
	"▀█████████████████████▀\n"
	"\033[2;34m  ▄██████████ \033[2;33m████████\033[31m ▄\033[0m\033[38;5;249m   "
	"▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄\n"
	"\033[2;34m ███████████▀ \033[2;33m███████▀\033[31m ██\033[0m\033[38;5;249m   "
	"▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀\n"
	"\033[2;34m█████████▀   \033[2;33m▀▀▀▀▀▀▀▀\033[31m ████\033[0m\033[38;5;246m   "
	"▀███████████▀\n"
	"\033[2;34m▀█████▀ \033[2;32m▄▄███████████▄\033[31m ████\033[0m\033[38;5;243m   ▄▄▄▄▄▄▄▄▄\n"
	"\033[2;34m  ▀▀▀ \033[2;32m███████████████▀\033[31m ████\033[0m\033[38;5;243m   ▀▀▀▀▀▀▀▀\n"
	"       \033[2;32m▀▀█████▀▀▀▀▀▀\033[31m  ▀▀▀▀\033[0m\033[38;5;240m   ▄█████▀\n"
	"              \033[2;90m ████████▀    ▄▄▄\n"
	"              \033[2;90m ▀███▀       ▀▀▀\n"
	"              \033[2;90m  ▀▀      \033[0m\n";
const char *banner_name = "╔═══╗╔═══╗╔═══╗╔═╗ ╔╗╔══╗╔═══╗╔══╗╔╗   ╔═══╗╔════╗\n"
			  "║╔═╗║║╔═╗║║╔═╗║║║║ ║║╚╣╠╝║╔═╗║╚╣╠╝║║   ║╔═╗║║╔╗╔╗║\n"
			  "║║ ╚╝║║ ║║║║ ╚╝║║╚╗║║ ║║ ║║ ║║ ║║ ║║   ║║ ║║╚╝║║╚╝\n"
			  "║║   ║║ ║║║║╔═╗║╔╗╚╝║ ║║ ║╚═╝║ ║║ ║║   ║║ ║║  ║║  \n"
			  "║║ ╔╗║║ ║║║║╚╗║║║╚╗║║ ║║ ║╔══╝ ║║ ║║ ╔╗║║ ║║  ║║  \n"
			  "║╚═╝║║╚═╝║║╚═╝║║║ ║║║╔╣╠╗║║   ╔╣╠╗║╚═╝║║╚═╝║ ╔╝╚╗ \n"
			  "╚═══╝╚═══╝╚═══╝╚╝ ╚═╝╚══╝╚╝   ╚══╝╚═══╝╚═══╝ ╚══╝ \n\033[31m"
			  "       ┏━━━┓┏━━━┓┏━━━┓┏━━━┓┏━━┓ ┏━━━┓┏━━┓\n"
			  "       ┃┏━┓┃┃┏━━┛┃┏━┓┃┃┏━━┛┃┏┓┃ ┃┏━┓┃┗┫┣┛\n"
			  "       ┃┃ ┗┛┃┗━┓ ┃┗━┛┃┃┗━┓ ┃┗┛┗┓┃┗━┛┃ ┃┃ \n"
			  "       ┃┃ ┏┓┃┏━┛ ┃┏┓┏┛┃┏━┛ ┃┏━┓┃┃┏┓┏┛ ┃┃ \n"
			  "       ┃┗━┛┃┃┗━━┓┃┃┃┗┓┃┗━━┓┃┗━┛┃┃┃┃┗┓┏┫┣┓\n"
			  "       ┗━━━┛┗━━━┛┗┛┗━┛┗━━━┛┗━━━┛┗┛┗━┛┗━━┛\n\033[0m";
#endif

// vi: ts=4 sw=4 et
