/*
 * Copyright (c) 2025 CogniPilot Foundation
 * Copyright 2025 NXP
 * SPDX-License-Identifier: Apache-2.0
 */

#include "synapse_rpmsg.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/ipm.h>

#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/io.h>
#include <resource_table.h>

#ifdef CONFIG_SHELL_BACKEND_RPMSG
#include <zephyr/shell/shell_rpmsg.h>
#endif

#ifdef CONFIG_FILE_SYSTEM_RPMSGFS
#include <zephyr/fs/rpmsgfs_fs.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(synapse_rpmsg, LOG_LEVEL_INF);

#define SHM_DEVICE_NAME "shm"

#if !DT_HAS_CHOSEN(zephyr_ipc_shm)
#error "Synapse rpmsg requires definition of shared memory for rpmsg"
#endif

/* Constants derived from device tree */
#define SHM_NODE       DT_CHOSEN(zephyr_ipc_shm)
#define SHM_START_ADDR DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE       DT_REG_SIZE(SHM_NODE)

#define APP_TASK_STACK_SIZE (1024)
#define MY_PRIORITY         1

K_THREAD_STACK_DEFINE(thread_mng_stack, APP_TASK_STACK_SIZE);

static struct k_thread thread_mng_data;

static const struct device *const ipm_handle = DEVICE_DT_GET(DT_CHOSEN(zephyr_ipc));

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;
static metal_phys_addr_t rsc_tab_physmap;

static struct metal_io_region shm_io_data; /* shared memory */
static struct metal_io_region rsc_io_data; /* rsc_table memory */

struct rpmsg_rcv_msg {
	void *data;
	size_t len;
};

static struct metal_io_region *shm_io = &shm_io_data;

static struct metal_io_region *rsc_io = &rsc_io_data;
static struct rpmsg_virtio_device rvdev;

static void *rsc_table;
static struct rpmsg_device *rpdev = 0;

static char rx_synapse_msg[RPMSG_BUFFER_SIZE];
static struct rpmsg_endpoint synapse_ept;
static struct rpmsg_rcv_msg synapse_msg = {.data = rx_synapse_msg};

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_synapse_sem, 0, 1);
static K_SEM_DEFINE(rpdev_sem, 0, 1);
static K_MUTEX_DEFINE(data_synapse_mutex);

static void platform_ipm_callback(const struct device *dev, void *context, uint32_t id,
				  volatile void *data)
{
	LOG_DBG("%s: msg received from mb %d", __func__, id);
	k_sem_give(&data_sem);
}

static int rpmsg_recv_synapse_callback(struct rpmsg_endpoint *ept, void *data, size_t len,
				       uint32_t src, void *priv)
{
	if (len > sizeof(rx_synapse_msg)) {
		LOG_ERR("synapse msg too big %i", len);
		return RPMSG_SUCCESS;
	}

	k_mutex_lock(&data_synapse_mutex, K_FOREVER);

	memcpy(synapse_msg.data, data, len);
	synapse_msg.len = len;
	k_sem_give(&data_synapse_sem);

	k_mutex_unlock(&data_synapse_mutex);

	return RPMSG_SUCCESS;
}

static void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t src)
{
	LOG_ERR("%s: unexpected ns service receive for name %s", __func__, name);
}

static int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);

	LOG_DBG("%s: msg received", __func__);
	ipm_send(ipm_handle, 0, id, NULL, 0);

	return 0;
}

static int platform_init(void)
{
	int status;

	/* setup IPM */
	if (!device_is_ready(ipm_handle)) {
		LOG_ERR("IPM device is not ready");
		return -1;
	}

	ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

	status = ipm_set_enabled(ipm_handle, 1);
	if (status) {
		LOG_ERR("ipm_set_enabled failed");
		return -1;
	}

	return 0;
}

struct rpmsg_device *platform_create_rpmsg_vdev(unsigned int vdev_index, unsigned int role,
						void (*rst_cb)(struct virtio_device *vdev),
						rpmsg_ns_bind_cb ns_cb)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, VDEV_ID, rsc_table_to_vdev(rsc_table),
					rsc_io, NULL, mailbox_notify, NULL);

	if (!vdev) {
		LOG_ERR("failed to create vdev");
		return NULL;
	}

	/* wait master rpmsg init completion */
	rproc_virtio_wait_remote_ready(vdev);

	vring_rsc = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid, (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 0");
		goto failed;
	}

	vring_rsc = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid, (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 1");
		goto failed;
	}

	ret = rpmsg_init_vdev(&rvdev, vdev, ns_cb, shm_io, NULL);
	if (ret) {
		LOG_ERR("failed rpmsg_init_vdev");
		goto failed;
	}

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	rproc_virtio_remove_vdev(vdev);

	return NULL;
}

void rpmsg_synapse_send(uint8_t *buf, size_t size)
{
	rpmsg_send(&synapse_ept, buf, size);
}

size_t rpmsg_synapse_receive(uint8_t *buf, size_t size)
{
	int ret = 0;

	k_sem_take(&data_synapse_sem, K_FOREVER);

	k_mutex_lock(&data_synapse_mutex, K_FOREVER);

	if (synapse_msg.len <= size) {
		memcpy(buf, synapse_msg.data, synapse_msg.len);
		ret = synapse_msg.len;
	}

	k_mutex_unlock(&data_synapse_mutex);

	return ret;
}

void rpmsg_rpdev_wait()
{
	if (!rpdev) {
		k_sem_take(&rpdev_sem, K_FOREVER);
	}
}

void rpmsg_mng_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	int ret;

	rpdev = platform_create_rpmsg_vdev(0, VIRTIO_DEV_DEVICE, NULL, new_service_cb);
	if (!rpdev) {
		LOG_ERR("Failed to create rpmsg virtio device");
		return;
	}

#ifdef CONFIG_FILE_SYSTEM_RPMSGFS
	rpmsgfs_init_rpmsg(rpdev);
#endif

	k_sem_give(&rpdev_sem);

#ifdef CONFIG_SHELL_BACKEND_RPMSG
	(void)shell_backend_rpmsg_init_transport(rpdev);
#endif

	ret = rpmsg_create_ept(&synapse_ept, rpdev, "rpmsg-synapse", RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_synapse_callback, NULL);

	if (ret) {
		LOG_ERR("[Synapse] Could not create endpoint: %d", ret);
	} else {
		LOG_INF("OpenAMP[remote] Synapse responder started");
	}

	while (1) {
		int status = k_sem_take(&data_sem, K_FOREVER);

		if (status == 0) {
			rproc_virtio_notified(rvdev.vdev, VRING1_ID);
		}
	}
}

static int init_metal_and_rsc(void)
{
	int rsc_size;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	int status;

	status = metal_init(&metal_params);
	if (status) {
		LOG_ERR("metal_init: failed: %d", status);
		return -1;
	}

	/* declare shared memory region */
	metal_io_init(shm_io, (void *)SHM_START_ADDR, &shm_physmap, SHM_SIZE, -1, 0, NULL);

	/* declare resource table region */
	rsc_table_get(&rsc_table, &rsc_size);
	rsc_tab_physmap = (uintptr_t)rsc_table;

	metal_io_init(rsc_io, rsc_table, &rsc_tab_physmap, rsc_size, -1, 0, NULL);

	return 0;
}

static int init_synapse_rpmsg(void)
{
	/* Initialize platform */
	int ret = platform_init();
	if (ret) {
		LOG_ERR("Failed to initialize platform");
		return -1;
	}

	LOG_INF("Starting rpmsg mng thread");
	k_tid_t tid = k_thread_create(&thread_mng_data, thread_mng_stack, APP_TASK_STACK_SIZE,
				      rpmsg_mng_task, NULL, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "synapse_rpmsg");
	k_thread_start(tid);

	return 0;
};

SYS_INIT(init_metal_and_rsc, PRE_KERNEL_1, 0);
SYS_INIT(init_synapse_rpmsg, APPLICATION, 0);

/* vi: ts=4 sw=4 et: */
