/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "imu_stream.h"

#include "rate_control.h"

#include <errno.h>
#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>

LOG_MODULE_DECLARE(rdd2, LOG_LEVEL_INF);

#define IMU_NODE                DT_ALIAS(imu0)
#define RDD2_IMU_TIMEOUT_NS     (4ULL * RDD2_CONTROL_PERIOD_NS)
#define RDD2_IMU_RTIO_SQ_COUNT  16U
#define RDD2_IMU_RTIO_CQ_COUNT  16U
#define RDD2_IMU_RTIO_BUF_COUNT 32U
#define RDD2_IMU_RTIO_BUF_SIZE  128U

static void imu_outputs_zero(synapse_topic_Vec3f_t *gyro, synapse_topic_Vec3f_t *accel)
{
	*gyro = (synapse_topic_Vec3f_t){0};
	*accel = (synapse_topic_Vec3f_t){0};
}

#if DT_NODE_HAS_COMPAT(IMU_NODE, invensense_icm45686) && defined(CONFIG_SENSOR_ASYNC_API)

SENSOR_DT_STREAM_IODEV(rdd2_imu_stream_iodev, IMU_NODE,
		       {SENSOR_TRIG_DATA_READY, SENSOR_STREAM_DATA_INCLUDE});

/*
 * The ICM45686 stream path is self-sustaining. Give it enough queue and buffer
 * headroom to absorb brief startup stalls without starving the ISR-side RTIO
 * buffer acquisition path.
 */
RTIO_DEFINE_WITH_MEMPOOL(rdd2_imu_rtio, RDD2_IMU_RTIO_SQ_COUNT, RDD2_IMU_RTIO_CQ_COUNT,
			 RDD2_IMU_RTIO_BUF_COUNT, RDD2_IMU_RTIO_BUF_SIZE, sizeof(void *));

static const struct device *const g_imu_dev = DEVICE_DT_GET(IMU_NODE);
static struct rtio_sqe *g_imu_stream_handle;
static const struct sensor_decoder_api *g_imu_decoder;
static uint64_t g_last_sample_ns;
static bool g_have_last_sample;
static const struct sensor_chan_spec g_imu_accel_chan = {
	.chan_type = SENSOR_CHAN_ACCEL_XYZ,
	.chan_idx = 0,
};
static const struct sensor_chan_spec g_imu_gyro_chan = {
	.chan_type = SENSOR_CHAN_GYRO_XYZ,
	.chan_idx = 0,
};

static void imu_sensor_axes_to_body(float gyro_x, float gyro_y, float gyro_z, float accel_x,
				    float accel_y, float accel_z, synapse_topic_Vec3f_t *gyro_out,
				    synapse_topic_Vec3f_t *accel_out)
{

	/*
	 * Control code consumes IMU data in FLU body axes:
	 * body x (forward) <- sensor y
	 * body y (left)    <- -sensor x
	 * body z (up)      <- sensor z
	 */
	gyro_out->x = gyro_y;
	gyro_out->y = -gyro_x;
	gyro_out->z = gyro_z;

	accel_out->x = accel_y;
	accel_out->y = -accel_x;
	accel_out->z = accel_z;
}

static float imu_q31_to_float(q31_t value, int8_t shift)
{
	return ldexpf((float)value, (int)shift - 31);
}

static int imu_stream_start(void)
{
	return sensor_stream(&rdd2_imu_stream_iodev, &rdd2_imu_rtio, NULL, &g_imu_stream_handle);
}

static void imu_stream_timing_reset(void)
{
	g_last_sample_ns = 0U;
	g_have_last_sample = false;
}

static void imu_stream_drain_cq(void)
{
	struct rtio_cqe *cqe;

	while ((cqe = rtio_cqe_consume(&rdd2_imu_rtio)) != NULL) {
		uint8_t *buf = NULL;
		uint32_t buf_len = 0U;

		if (cqe->result == 0 &&
		    rtio_cqe_get_mempool_buffer(&rdd2_imu_rtio, cqe, &buf, &buf_len) == 0 &&
		    buf != NULL) {
			rtio_release_buffer(&rdd2_imu_rtio, buf, buf_len);
		}

		rtio_cqe_release(&rdd2_imu_rtio, cqe);
	}
}

static void imu_stream_restart(const char *reason, int rc)
{
	int restart_rc;

	LOG_WRN("imu stream restart after %s: %d", reason, rc);

	if (g_imu_stream_handle != NULL) {
		(void)rtio_sqe_cancel(g_imu_stream_handle);
	}

	imu_stream_drain_cq();
	imu_stream_timing_reset();

	restart_rc = imu_stream_start();
	if (restart_rc != 0) {
		LOG_ERR("imu stream restart failed: %d", restart_rc);
	}
}

static bool imu_stream_decode_latest(const uint8_t *buf, synapse_topic_Vec3f_t *gyro,
				     synapse_topic_Vec3f_t *accel, uint64_t *sample_ns)
{
	struct sensor_three_axis_data accel_data = {0};
	struct sensor_three_axis_data gyro_data = {0};
	uint16_t accel_frame_count = 0U;
	uint16_t gyro_frame_count = 0U;
	uint32_t accel_fit;
	uint32_t gyro_fit;

	if (g_imu_decoder == NULL) {
		return false;
	}

	if (g_imu_decoder->get_frame_count(buf, g_imu_accel_chan, &accel_frame_count) != 0 ||
	    g_imu_decoder->get_frame_count(buf, g_imu_gyro_chan, &gyro_frame_count) != 0 ||
	    accel_frame_count == 0U || gyro_frame_count == 0U) {
		return false;
	}

	accel_fit = accel_frame_count - 1U;
	gyro_fit = gyro_frame_count - 1U;

	if (g_imu_decoder->decode(buf, g_imu_accel_chan, &accel_fit, 1U, &accel_data) <= 0 ||
	    g_imu_decoder->decode(buf, g_imu_gyro_chan, &gyro_fit, 1U, &gyro_data) <= 0) {
		return false;
	}

	imu_sensor_axes_to_body(imu_q31_to_float(gyro_data.readings[0].x, gyro_data.shift),
				imu_q31_to_float(gyro_data.readings[0].y, gyro_data.shift),
				imu_q31_to_float(gyro_data.readings[0].z, gyro_data.shift),
				imu_q31_to_float(accel_data.readings[0].x, accel_data.shift),
				imu_q31_to_float(accel_data.readings[0].y, accel_data.shift),
				imu_q31_to_float(accel_data.readings[0].z, accel_data.shift), gyro,
				accel);
	*sample_ns = gyro_data.header.base_timestamp_ns + gyro_data.readings[0].timestamp_delta;
	return true;
}

int rdd2_imu_stream_init(void)
{
	int rc;

	if (!device_is_ready(g_imu_dev)) {
		return -ENODEV;
	}

	rc = sensor_get_decoder(g_imu_dev, &g_imu_decoder);
	if (rc != 0) {
		LOG_ERR("imu decoder init failed: %d", rc);
		return rc;
	}

	rc = imu_stream_start();
	if (rc != 0) {
		LOG_ERR("imu stream start failed: %d", rc);
		return rc;
	}

	imu_stream_timing_reset();
	return 0;
}

bool rdd2_imu_stream_wait_next(synapse_topic_Vec3f_t *gyro, synapse_topic_Vec3f_t *accel, float *dt,
			       uint64_t *interrupt_timestamp_ns)
{
	struct rtio_cqe *cqe;
	uint8_t *buf = NULL;
	uint32_t buf_len = 0;
	uint64_t sample_ns = 0;
	int rc;

	*dt = RDD2_CONTROL_DT_S;
	if (interrupt_timestamp_ns != NULL) {
		*interrupt_timestamp_ns = 0U;
	}

	cqe = rtio_cqe_consume_block_timeout(&rdd2_imu_rtio, K_NSEC(RDD2_IMU_TIMEOUT_NS));
	if (cqe == NULL) {
		imu_outputs_zero(gyro, accel);
		imu_stream_restart("timeout", -ETIMEDOUT);
		return false;
	}

	rc = cqe->result;
	if (rc == 0) {
		rc = rtio_cqe_get_mempool_buffer(&rdd2_imu_rtio, cqe, &buf, &buf_len);
	}
	rtio_cqe_release(&rdd2_imu_rtio, cqe);

	if (rc != 0) {
		imu_outputs_zero(gyro, accel);
		imu_stream_restart("cqe", rc);
		return false;
	}

	if (!imu_stream_decode_latest(buf, gyro, accel, &sample_ns)) {
		rtio_release_buffer(&rdd2_imu_rtio, buf, buf_len);
		imu_outputs_zero(gyro, accel);
		imu_stream_restart("decode", -EBADMSG);
		return false;
	}

	rtio_release_buffer(&rdd2_imu_rtio, buf, buf_len);

	if (g_have_last_sample && sample_ns > g_last_sample_ns) {
		*dt = (float)(sample_ns - g_last_sample_ns) * 1.0e-9f;
	}
	if (interrupt_timestamp_ns != NULL) {
		*interrupt_timestamp_ns = sample_ns;
	}

	g_last_sample_ns = sample_ns;
	g_have_last_sample = true;
	return true;
}

#else

static uint64_t imu_timestamp_now_ns(void)
{
	return k_cyc_to_ns_floor64(k_cycle_get_64());
}

static bool imu_fetch_sync(synapse_topic_Vec3f_t *gyro_out, synapse_topic_Vec3f_t *accel_out)
{
	const struct device *const imu_dev = DEVICE_DT_GET(IMU_NODE);
	struct sensor_value gyro[3];
	struct sensor_value accel_values[3];
	float gyro_sensor[3];
	float accel_sensor[3];
	int rc;

	rc = sensor_sample_fetch(imu_dev);
	if (rc != 0) {
		return false;
	}

	rc = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (rc != 0) {
		return false;
	}

	rc = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel_values);
	if (rc != 0) {
		return false;
	}

	for (size_t i = 0; i < 3; i++) {
		gyro_sensor[i] = sensor_value_to_float(&gyro[i]);
		accel_sensor[i] = sensor_value_to_float(&accel_values[i]);
	}

	gyro_out->x = gyro_sensor[1];
	gyro_out->y = -gyro_sensor[0];
	gyro_out->z = gyro_sensor[2];

	accel_out->x = accel_sensor[1];
	accel_out->y = -accel_sensor[0];
	accel_out->z = accel_sensor[2];

	return true;
}

int rdd2_imu_stream_init(void)
{
	return 0;
}

bool rdd2_imu_stream_wait_next(synapse_topic_Vec3f_t *gyro, synapse_topic_Vec3f_t *accel, float *dt,
			       uint64_t *interrupt_timestamp_ns)
{
	k_sleep(K_NSEC(RDD2_CONTROL_PERIOD_NS));
	*dt = RDD2_CONTROL_DT_S;
	if (interrupt_timestamp_ns != NULL) {
		*interrupt_timestamp_ns = 0U;
	}

	if (!imu_fetch_sync(gyro, accel)) {
		imu_outputs_zero(gyro, accel);
		return false;
	}

	if (interrupt_timestamp_ns != NULL) {
		*interrupt_timestamp_ns = imu_timestamp_now_ns();
	}

	return true;
}

#endif
