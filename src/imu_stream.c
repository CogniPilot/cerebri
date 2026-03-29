/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "imu_stream.h"

#include "rate_control.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>

LOG_MODULE_DECLARE(cerebri, LOG_LEVEL_INF);

#define IMU_NODE DT_ALIAS(imu0)
#define CEREBRI_IMU_TIMEOUT_US (4U * CEREBRI_CONTROL_PERIOD_US)
#define CEREBRI_IMU_RTIO_SQ_COUNT 16U
#define CEREBRI_IMU_RTIO_CQ_COUNT 16U
#define CEREBRI_IMU_RTIO_BUF_COUNT 32U
#define CEREBRI_IMU_RTIO_BUF_SIZE 128U

#if DT_NODE_HAS_COMPAT(IMU_NODE, invensense_icm45686) && defined(CONFIG_SENSOR_ASYNC_API) && \
	defined(CONFIG_DSP)
#include <zephyr/dsp/utils.h>
#endif

static void imu_outputs_zero(cerebri_topic_Vec3f_t *gyro, cerebri_topic_Vec3f_t *accel)
{
	*gyro = (cerebri_topic_Vec3f_t){0};
	*accel = (cerebri_topic_Vec3f_t){0};
}

#if DT_NODE_HAS_COMPAT(IMU_NODE, invensense_icm45686) && defined(CONFIG_SENSOR_ASYNC_API) && \
	defined(CONFIG_DSP)

SENSOR_DT_STREAM_IODEV(cerebri_imu_stream_iodev, IMU_NODE,
		       { SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE });

/*
 * The ICM45686 stream path is self-sustaining. Give it enough queue and buffer
 * headroom to absorb brief startup stalls without starving the ISR-side RTIO
 * buffer acquisition path.
 */
RTIO_DEFINE_WITH_MEMPOOL(cerebri_imu_rtio,
			 CEREBRI_IMU_RTIO_SQ_COUNT,
			 CEREBRI_IMU_RTIO_CQ_COUNT,
			 CEREBRI_IMU_RTIO_BUF_COUNT,
			 CEREBRI_IMU_RTIO_BUF_SIZE,
			 sizeof(void *));

static const struct device *const g_imu_dev = DEVICE_DT_GET(IMU_NODE);
static const struct sensor_decoder_api *g_imu_decoder;
static struct rtio_sqe *g_imu_stream_handle;
static uint64_t g_last_sample_ns;
static bool g_have_last_sample;

static uint64_t imu_sample_timestamp_ns(const struct sensor_three_axis_data *sample)
{
	return sample->header.base_timestamp_ns + sample->readings[0].timestamp_delta;
}

static float imu_q31_to_float(q31_t value, int8_t shift)
{
	/*
	 * ICM45686 only emits non-negative q31 shifts in Zephyr's decoder.
	 * Use the shared DSP utility rather than maintaining a local conversion.
	 */
	return (float)Z_SHIFT_Q31_TO_F32(value, shift);
}

static void imu_sensor_axes_to_body(const struct sensor_three_axis_data *gyro_sensor,
				    const struct sensor_three_axis_data *accel_sensor,
				    cerebri_topic_Vec3f_t *gyro_out,
				    cerebri_topic_Vec3f_t *accel_out)
{
	const float gyro_x = imu_q31_to_float(gyro_sensor->readings[0].x, gyro_sensor->shift);
	const float gyro_y = imu_q31_to_float(gyro_sensor->readings[0].y, gyro_sensor->shift);
	const float gyro_z = imu_q31_to_float(gyro_sensor->readings[0].z, gyro_sensor->shift);
	const float accel_x = imu_q31_to_float(accel_sensor->readings[0].x, accel_sensor->shift);
	const float accel_y = imu_q31_to_float(accel_sensor->readings[0].y, accel_sensor->shift);
	const float accel_z = imu_q31_to_float(accel_sensor->readings[0].z, accel_sensor->shift);

	/*
	 * Control code consumes IMU data in FRD body axes:
	 * body roll  (F / x) <- sensor gyro y
	 * body pitch (R / y) <- sensor gyro x
	 * body yaw   (D / z) <- -sensor gyro z
	 *
	 * Apply the same axis remap to accel so future attitude-estimator code
	 * sees a consistent body frame.
	 */
	gyro_out->x = gyro_y;
	gyro_out->y = gyro_x;
	gyro_out->z = -gyro_z;

	accel_out->x = accel_y;
	accel_out->y = accel_x;
	accel_out->z = -accel_z;
}

static int imu_decode_latest_three_axis(const uint8_t *buf, struct sensor_chan_spec chan_spec,
					struct sensor_three_axis_data *sample,
					uint64_t *timestamp_ns)
{
	uint16_t frame_count = 0;
	uint32_t fit = 0;
	int rc;

	rc = g_imu_decoder->get_frame_count(buf, chan_spec, &frame_count);
	if (rc != 0 || frame_count == 0U) {
		return rc == 0 ? -ENODATA : rc;
	}

	while (fit < frame_count) {
		rc = g_imu_decoder->decode(buf, chan_spec, &fit, 1, sample);
		if (rc <= 0) {
			return rc == 0 ? -ENODATA : rc;
		}
	}

	*timestamp_ns = imu_sample_timestamp_ns(sample);
	return 0;
}

static bool imu_stream_decode_latest(const uint8_t *buf, cerebri_topic_Vec3f_t *gyro,
				     cerebri_topic_Vec3f_t *accel, uint64_t *sample_ns)
{
	const struct sensor_chan_spec gyro_chan = { SENSOR_CHAN_GYRO_XYZ, 0 };
	const struct sensor_chan_spec accel_chan = { SENSOR_CHAN_ACCEL_XYZ, 0 };
	struct sensor_three_axis_data gyro_sample = {0};
	struct sensor_three_axis_data accel_sample = {0};
	uint64_t gyro_ns = 0;
	uint64_t accel_ns = 0;
	int rc;

	rc = imu_decode_latest_three_axis(buf, gyro_chan, &gyro_sample, &gyro_ns);
	if (rc != 0) {
		return false;
	}

	rc = imu_decode_latest_three_axis(buf, accel_chan, &accel_sample, &accel_ns);
	if (rc != 0) {
		return false;
	}

	imu_sensor_axes_to_body(&gyro_sample, &accel_sample, gyro, accel);
	*sample_ns = (gyro_ns >= accel_ns) ? gyro_ns : accel_ns;
	return true;
}

int cerebri_imu_stream_init(void)
{
	int rc;

	if (!device_is_ready(g_imu_dev)) {
		return -ENODEV;
	}

	rc = sensor_get_decoder(g_imu_dev, &g_imu_decoder);
	if (rc != 0) {
		LOG_ERR("imu decoder unavailable: %d", rc);
		return rc;
	}

	rc = sensor_stream(&cerebri_imu_stream_iodev, &cerebri_imu_rtio, NULL,
			   &g_imu_stream_handle);
	if (rc != 0) {
		LOG_ERR("imu stream start failed: %d", rc);
		return rc;
	}

	g_have_last_sample = false;
	return 0;
}

bool cerebri_imu_stream_wait_next(cerebri_topic_Vec3f_t *gyro,
				  cerebri_topic_Vec3f_t *accel,
				  float *dt)
{
	struct rtio_cqe *cqe;
	uint8_t *buf = NULL;
	uint32_t buf_len = 0;
	uint64_t sample_ns = 0;
	int rc;

	*dt = (float)CEREBRI_CONTROL_PERIOD_US * 1.0e-6f;

	cqe = rtio_cqe_consume_block_timeout(&cerebri_imu_rtio, K_USEC(CEREBRI_IMU_TIMEOUT_US));
	if (cqe == NULL) {
		imu_outputs_zero(gyro, accel);
		return false;
	}

	rc = cqe->result;
	if (rc == 0) {
		rc = rtio_cqe_get_mempool_buffer(&cerebri_imu_rtio, cqe, &buf, &buf_len);
	}
	rtio_cqe_release(&cerebri_imu_rtio, cqe);

	if (rc != 0) {
		imu_outputs_zero(gyro, accel);
		return false;
	}

	if (!imu_stream_decode_latest(buf, gyro, accel, &sample_ns)) {
		rtio_release_buffer(&cerebri_imu_rtio, buf, buf_len);
		imu_outputs_zero(gyro, accel);
		return false;
	}

	rtio_release_buffer(&cerebri_imu_rtio, buf, buf_len);

	if (g_have_last_sample && sample_ns > g_last_sample_ns) {
		*dt = (float)(sample_ns - g_last_sample_ns) * 1.0e-9f;
	}

	g_last_sample_ns = sample_ns;
	g_have_last_sample = true;
	return true;
}

#else

static bool imu_fetch_sync(cerebri_topic_Vec3f_t *gyro_out, cerebri_topic_Vec3f_t *accel_out)
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
	gyro_out->y = gyro_sensor[0];
	gyro_out->z = -gyro_sensor[2];

	accel_out->x = accel_sensor[1];
	accel_out->y = accel_sensor[0];
	accel_out->z = -accel_sensor[2];

	return true;
}

int cerebri_imu_stream_init(void)
{
	return 0;
}

bool cerebri_imu_stream_wait_next(cerebri_topic_Vec3f_t *gyro,
				  cerebri_topic_Vec3f_t *accel,
				  float *dt)
{
	k_sleep(K_USEC(CEREBRI_CONTROL_PERIOD_US));
	*dt = (float)CEREBRI_CONTROL_PERIOD_US * 1.0e-6f;

	if (!imu_fetch_sync(gyro, accel)) {
		imu_outputs_zero(gyro, accel);
		return false;
	}

	return true;
}

#endif
