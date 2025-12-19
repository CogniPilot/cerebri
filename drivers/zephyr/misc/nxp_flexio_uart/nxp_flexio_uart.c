/*
 * Copyright 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NXP FlexIO UART Driver - Single-wire half-duplex UART using FlexIO
 *
 * This driver implements a software UART using the NXP FlexIO peripheral.
 * It is designed for single-wire half-duplex communication, such as
 * BLHeli/AM32 ESC bootloader protocols.
 *
 * Key features:
 *   - Multiple independent UART channels on FlexIO pins
 *   - Half-duplex operation with automatic TX/RX switching
 *   - Hardware-generated baud rate timing (no CPU bit-banging)
 *   - Support for AM32 ESC bootloader entry (line HIGH at power-up)
 *
 * Design notes:
 *   - Each channel uses one FlexIO shifter and one timer
 *   - TX mode: push-pull output, idle HIGH
 *   - RX mode: high-Z input with external pull-up
 *   - Timer compare format: ((data_bits*2-1) << 8) | (divider-1)
 *   - Uses SHIFTBUFBYS for RX (byte-swapped, per NXP SDK)
 *
 * Hardware requirements:
 *   - External pull-up resistor on each signal line (4.7k typical)
 *   - FlexIO clock configured for target baud rate
 *   - For 19200 baud with 8-bit divider: ~5 MHz FlexIO clock
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#define LOG_MODULE_NAME nxp_flexio_uart
#include <fsl_clock.h>
#include <fsl_flexio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_NXP_FLEXIO_UART_LOG_LEVEL);

#include <zephyr/drivers/misc/nxp_flexio/nxp_flexio.h>
#include "nxp_flexio_uart.h"

#define DT_DRV_COMPAT nxp_flexio_uart

/*
 * UART timing constants
 * Frame: 1 start + 8 data + 1 stop = 10 bits total
 * Timer counts only data bits; start/stop handled by shifter
 */
#define UART_BITS_PER_FRAME 10
#define UART_DATA_BITS      8

/* Channel operating modes */
typedef enum {
	FLEXIO_UART_MODE_IDLE = 0,
	FLEXIO_UART_MODE_TX,
	FLEXIO_UART_MODE_RX,
	FLEXIO_UART_MODE_PIN,
} flexio_uart_mode_t;

/* Per-channel runtime state */
struct nxp_flexio_uart_channel_config {
	uint8_t pin_id;
	bool init;
	flexio_uart_mode_t mode;
	volatile bool tx_complete;
	volatile bool rx_ready;
	volatile uint8_t rx_data;
};

/* Channel collection */
struct nxp_flexio_uart_channel {
	uint8_t channel_count;
	struct nxp_flexio_uart_channel_config *channel_info;
};

/* Device configuration (from devicetree) */
struct nxp_flexio_uart_config {
	const struct device *flexio_dev;
	FLEXIO_Type *flexio_base;
	const struct pinctrl_dev_config *pincfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct nxp_flexio_uart_channel *channel;
	const struct nxp_flexio_child *child;
	uint32_t baud_rate;
};

/* Device runtime data */
struct nxp_flexio_uart_data {
	uint32_t flexio_clk;
	uint32_t timer_cmp;
	uint32_t shifter_mask;
	uint32_t timer_mask;
};

/*
 * Configure FlexIO shifter and timer for UART transmit mode.
 *
 * TX configuration:
 *   - Shifter in transmit mode with push-pull output
 *   - Timer triggered by shifter status (data written)
 *   - Automatic start bit (LOW) and stop bit (HIGH) generation
 *   - Idle state is HIGH (required for UART)
 *
 * Glitch prevention:
 *   SHIFTBUF is pre-loaded with 0xFF before reconfiguration. In disabled
 *   mode with output enabled, SHIFTBUF[0] drives the pin. This ensures
 *   the line stays HIGH during the brief window when the shifter is
 *   disabled but the pin is configured as output, preventing spurious
 *   LOW glitches that could be misinterpreted as start bits.
 */
static void flexio_uart_config_tx(const struct device *dev, uint32_t channel)
{
	const struct nxp_flexio_uart_config *config = dev->config;
	struct nxp_flexio_uart_data *data = dev->data;
	FLEXIO_Type *flexio_base = config->flexio_base;
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)config->child;
	struct nxp_flexio_uart_channel_config *ch_info = &config->channel->channel_info[channel];
	uint8_t shifter_idx = child->res.shifter_index[channel];
	uint8_t timer_idx = child->res.timer_index[channel];

	flexio_shifter_config_t shifterConfig = {0};
	flexio_timer_config_t timerConfig = {0};

	/*
	 * Pre-load shifter buffer with 0xFF to ensure line stays HIGH.
	 * When shifter is in disabled mode with output enabled, SHIFTBUF[0]
	 * drives the pin. This prevents spurious LOW glitch during reconfig.
	 */
	flexio_base->SHIFTBUF[shifter_idx] = 0xFF;

	/* Disable shifter before reconfiguring */
	FLEXIO_SetShifterConfig(flexio_base, shifter_idx, &shifterConfig);

	/* Shifter: transmit mode, push-pull output, start/stop bit generation */
	shifterConfig.inputSource = kFLEXIO_ShifterInputFromPin;
	shifterConfig.shifterStop = kFLEXIO_ShifterStopBitHigh;
	shifterConfig.shifterStart = kFLEXIO_ShifterStartBitLow;
	shifterConfig.timerSelect = timer_idx;
	shifterConfig.timerPolarity = kFLEXIO_ShifterTimerPolarityOnPositive;
	shifterConfig.pinConfig = kFLEXIO_PinConfigOutput;
	shifterConfig.pinSelect = ch_info->pin_id;
	shifterConfig.pinPolarity = kFLEXIO_PinActiveHigh;
	shifterConfig.shifterMode = kFLEXIO_ShifterModeTransmit;
	FLEXIO_SetShifterConfig(flexio_base, shifter_idx, &shifterConfig);

	/* Timer: dual 8-bit baud/bit mode, triggered by shifter write */
	timerConfig.timerOutput = kFLEXIO_TimerOutputOneNotAffectedByReset;
	timerConfig.timerDecrement = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
	timerConfig.timerReset = kFLEXIO_TimerResetNever;
	timerConfig.timerDisable = kFLEXIO_TimerDisableOnTimerCompare;
	timerConfig.timerEnable = kFLEXIO_TimerEnableOnTriggerHigh;
	timerConfig.timerStop = kFLEXIO_TimerStopBitEnableOnTimerDisable;
	timerConfig.timerStart = kFLEXIO_TimerStartBitEnabled;
	timerConfig.timerCompare = data->timer_cmp;
	timerConfig.triggerSelect = FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(shifter_idx);
	timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;
	timerConfig.triggerSource = kFLEXIO_TimerTriggerSourceInternal;
	timerConfig.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	timerConfig.pinSelect = 0;
	timerConfig.pinPolarity = kFLEXIO_PinActiveHigh;
	timerConfig.timerMode = kFLEXIO_TimerModeDual8BitBaudBit;
	FLEXIO_SetTimerConfig(flexio_base, timer_idx, &timerConfig);

	ch_info->mode = FLEXIO_UART_MODE_TX;
	ch_info->tx_complete = true;
}

/*
 * Configure FlexIO shifter and timer for UART receive mode.
 *
 * RX configuration:
 *   - Optionally reset timer and shifter (only needed when coming from RX)
 *   - Shifter in receive mode with pin as input (high-Z)
 *   - Timer triggered by start bit falling edge
 *   - Timer resets on each start bit for re-synchronization
 *
 * Note: The timer trigger select calculation uses ((pin_id << 1) | 1)
 * because some NXP SDK versions have a bug in the FLEXIO_TIMER_TRIGGER_SEL_PININPUT
 * macro that omits the | 1 required for pin input triggers.
 *
 * The full_reset parameter controls whether to clear stale RX state:
 *   - true: Clear flags and drain buffer (use when re-entering RX from RX)
 *   - false: Skip reset to minimize latency (use when transitioning from TX)
 */
static void flexio_uart_config_rx_internal(const struct device *dev, uint32_t channel,
					   bool full_reset)
{
	const struct nxp_flexio_uart_config *config = dev->config;
	struct nxp_flexio_uart_data *data = dev->data;
	FLEXIO_Type *flexio_base = config->flexio_base;
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)config->child;
	struct nxp_flexio_uart_channel_config *ch_info = &config->channel->channel_info[channel];
	uint8_t shifter_idx = child->res.shifter_index[channel];
	uint8_t timer_idx = child->res.timer_index[channel];

	flexio_shifter_config_t shifterConfig = {0};
	flexio_timer_config_t timerConfig = {0};

	/*
	 * Full reset sequence: disable timer/shifter, clear flags, drain buffer.
	 * This ensures clean state before configuring RX mode.
	 */
	FLEXIO_SetTimerConfig(flexio_base, timer_idx, &timerConfig);
	FLEXIO_SetShifterConfig(flexio_base, shifter_idx, &shifterConfig);
	FLEXIO_ClearShifterStatusFlags(flexio_base, 1 << shifter_idx);
	FLEXIO_ClearTimerStatusFlags(flexio_base, 1 << timer_idx);
	(void)flexio_base->SHIFTBUFBYS[shifter_idx];

	(void)full_reset; /* Currently always do full reset */

	/* Shifter: receive mode, input from pin, start/stop bit detection */
	shifterConfig.inputSource = kFLEXIO_ShifterInputFromPin;
	shifterConfig.shifterStop = kFLEXIO_ShifterStopBitHigh;
	shifterConfig.shifterStart = kFLEXIO_ShifterStartBitLow;
	shifterConfig.timerSelect = timer_idx;
	shifterConfig.timerPolarity = kFLEXIO_ShifterTimerPolarityOnNegitive; /* NXP SDK typo */
	shifterConfig.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	shifterConfig.pinSelect = ch_info->pin_id;
	shifterConfig.pinPolarity = kFLEXIO_PinActiveHigh;
	shifterConfig.shifterMode = kFLEXIO_ShifterModeReceive;
	FLEXIO_SetShifterConfig(flexio_base, shifter_idx, &shifterConfig);

	/* Timer: enable on start bit (falling edge), resync on each byte */
	uint32_t trig_sel = (ch_info->pin_id << 1) | 1; /* Pin input trigger */
	timerConfig.triggerSelect = trig_sel;
	timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveHigh;
	timerConfig.triggerSource = kFLEXIO_TimerTriggerSourceExternal;
	timerConfig.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	timerConfig.pinSelect = ch_info->pin_id;
	timerConfig.pinPolarity = kFLEXIO_PinActiveLow;
	timerConfig.timerMode = kFLEXIO_TimerModeDual8BitBaudBit;
	timerConfig.timerOutput = kFLEXIO_TimerOutputOneAffectedByReset;
	timerConfig.timerDecrement = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
	timerConfig.timerReset = kFLEXIO_TimerResetOnTimerPinRisingEdge;
	timerConfig.timerDisable = kFLEXIO_TimerDisableOnTimerCompare;
	timerConfig.timerEnable = kFLEXIO_TimerEnableOnPinRisingEdge;
	timerConfig.timerStop = kFLEXIO_TimerStopBitEnableOnTimerDisable;
	timerConfig.timerStart = kFLEXIO_TimerStartBitEnabled;
	timerConfig.timerCompare = data->timer_cmp;
	FLEXIO_SetTimerConfig(flexio_base, timer_idx, &timerConfig);

	ch_info->mode = FLEXIO_UART_MODE_RX;
	ch_info->rx_ready = false;
}

/*
 * Configure FlexIO clock for the target baud rate.
 *
 * The FlexIO timer divider is only 8 bits (max 256), so the FlexIO clock
 * must be low enough that divider = clock / (baud * 2) fits in 8 bits.
 * For 19200 baud: clock ~= 19200 * 2 * 130 = 5 MHz
 */
static uint32_t nxp_flexio_uart_set_clock(const struct nxp_flexio_uart_config *cfg)
{
	uint32_t flexio_clk = 0;

#ifdef CONFIG_SOC_MIMXRT1064
	/*
	 * i.MX RT1064 FlexIO clock configuration:
	 * PLL3 PFD2 @ frac=27: 480 * 18 / 27 = 320 MHz
	 * PreDiv=7 (div 8), Div=7 (div 8): 320 / 64 = 5 MHz
	 */
	const uint32_t pfd_frac = 27U;
	const uint32_t prediv = 7U;
	const uint32_t div = 7U;

	CLOCK_SetDiv(kCLOCK_Flexio1PreDiv, prediv);
	CLOCK_SetDiv(kCLOCK_Flexio1Div, div);
	CLOCK_InitUsb1Pfd(kCLOCK_Pfd2, pfd_frac);
	CLOCK_SetMux(kCLOCK_Flexio1Mux, 1);

	flexio_clk = (480000000U / pfd_frac) * 18U / (prediv + 1) / (div + 1);
	LOG_INF("FlexIO1 clock: %u Hz (frac=%u prediv=%u div=%u)", flexio_clk, pfd_frac, prediv,
		div);
#endif

#ifdef CONFIG_SOC_MIMXRT1176
	/*
	 * i.MX RT1176 FlexIO clock configuration:
	 * SysPll2 PFD3 @ frac=22, div=64: ~6 MHz
	 */
	const uint32_t pfd_frac = 22U;
	const uint32_t div = 64U;

	CLOCK_InitPfd(kCLOCK_PllSys2, kCLOCK_Pfd3, pfd_frac);
	clock_root_config_t rootCfg = {0};
	rootCfg.mux = kCLOCK_FLEXIO1_ClockRoot_MuxSysPll2Pfd3;
	rootCfg.div = div;
	CLOCK_SetRootClock(kCLOCK_Root_Flexio1, &rootCfg);

	/* Calculate clock: (528MHz * 18 / frac) / div - reorder to avoid overflow */
	flexio_clk = ((528000000U / pfd_frac) * 18U) / div;
	LOG_INF("FlexIO1 clock: %u Hz", flexio_clk);
#endif

	return flexio_clk;
}

/*
 * Driver initialization.
 *
 * Critical sequence for AM32 ESC bootloader support:
 *   1. Configure FlexIO clock
 *   2. Attach to FlexIO parent device
 *   3. Configure all channels for TX mode (outputs HIGH)
 *   4. Apply pinctrl (pins now driven HIGH by FlexIO)
 *
 * This ensures signal lines are HIGH when ESCs power up, which is
 * required for AM32 bootloader entry.
 */
static int nxp_flexio_uart_init(const struct device *dev)
{
	const struct nxp_flexio_uart_config *config = dev->config;
	struct nxp_flexio_uart_data *data = dev->data;
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)config->child;
	int err;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("Clock device not ready");
		return -ENODEV;
	}

	/* Configure FlexIO clock */
	data->flexio_clk = nxp_flexio_uart_set_clock(config);
	if (data->flexio_clk == 0) {
		LOG_ERR("Failed to configure FlexIO clock");
		return -EINVAL;
	}

	/*
	 * Calculate timer compare value:
	 *   Upper byte: (data_bits * 2 - 1) = bit count for timer
	 *   Lower byte: (divider - 1) = baud rate divider
	 */
	uint32_t divider = (data->flexio_clk / (config->baud_rate * 2)) - 1;
	if (divider > 255) {
		LOG_ERR("Baud rate %u too low for clock %u Hz (divider %u > 255)",
			config->baud_rate, data->flexio_clk, divider);
		return -EINVAL;
	}
	data->timer_cmp = ((UART_DATA_BITS * 2 - 1) << 8) | (divider & 0xFF);

	uint32_t actual_baud = data->flexio_clk / (2 * (divider + 1));
	LOG_INF("Baud: target=%u actual=%u (divider=%u, timer_cmp=0x%04X)", config->baud_rate,
		actual_baud, divider, data->timer_cmp);

	/* Attach to FlexIO parent */
	err = nxp_flexio_child_attach(config->flexio_dev, child);
	if (err < 0) {
		LOG_ERR("Failed to attach FlexIO child: %d", err);
		return err;
	}

	/* Configure all channels for TX mode (idle HIGH) BEFORE applying pinctrl */
	data->shifter_mask = 0;
	data->timer_mask = 0;
	for (uint8_t ch = 0; ch < config->channel->channel_count; ch++) {
		config->channel->channel_info[ch].init = true;
		config->channel->channel_info[ch].mode = FLEXIO_UART_MODE_IDLE;
		data->shifter_mask |= (1 << child->res.shifter_index[ch]);
		data->timer_mask |= (1 << child->res.timer_index[ch]);
		flexio_uart_config_tx(dev, ch);
	}

	/* Apply pinctrl - pins now driven HIGH by FlexIO */
	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		LOG_ERR("Failed to configure pins: %d", err);
		return err;
	}

	LOG_INF("FlexIO UART initialized with %d channels", config->channel->channel_count);
	return 0;
}

/* Public API: Enable TX mode on channel */
int nxp_flexio_uart_tx_enable(const struct device *dev, uint8_t channel)
{
	const struct nxp_flexio_uart_config *config = dev->config;

	if (channel >= config->channel->channel_count) {
		return -EINVAL;
	}

	if (config->channel->channel_info[channel].mode != FLEXIO_UART_MODE_TX) {
		flexio_uart_config_tx(dev, channel);
	}

	return 0;
}

/*
 * Public API: Enable RX mode on channel.
 *
 * Automatically determines whether to do a full reset based on previous mode:
 *   - From TX/IDLE: No reset (minimize latency for half-duplex turnaround)
 *   - From RX: Full reset (clear stale data before new receive session)
 */
int nxp_flexio_uart_rx_enable(const struct device *dev, uint8_t channel)
{
	const struct nxp_flexio_uart_config *config = dev->config;

	if (channel >= config->channel->channel_count) {
		return -EINVAL;
	}

	flexio_uart_mode_t prev_mode = config->channel->channel_info[channel].mode;
	if (prev_mode != FLEXIO_UART_MODE_RX) {
		/*
		 * Only do full reset if re-entering RX from RX (not currently the case).
		 * When transitioning from TX/IDLE/PIN, skip reset to minimize latency.
		 */
		bool full_reset = false; /* Coming from TX/IDLE/PIN, no stale RX data */
		flexio_uart_config_rx_internal(dev, channel, full_reset);
	}

	return 0;
}

/*
 * Public API: Write data to UART channel.
 *
 * For half-duplex operation, this function:
 *   1. Ensures TX mode is configured
 *   2. Writes all bytes to the shifter
 *   3. Waits for transmission to complete
 *   4. Releases the line to high-Z for RX turnaround
 */
int nxp_flexio_uart_write(const struct device *dev, uint8_t channel, const uint8_t *data,
			  size_t len)
{
	const struct nxp_flexio_uart_config *config = dev->config;
	FLEXIO_Type *flexio_base = config->flexio_base;
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)config->child;
	struct nxp_flexio_uart_channel_config *ch_info = &config->channel->channel_info[channel];

	if (channel >= config->channel->channel_count) {
		return -EINVAL;
	}

	nxp_flexio_uart_tx_enable(dev, channel);

	uint8_t shifter_idx = child->res.shifter_index[channel];

	/* Transmit all bytes */
	for (size_t i = 0; i < len; i++) {
		while (!(FLEXIO_GetShifterStatusFlags(flexio_base) & (1 << shifter_idx))) {
			/* Wait for shifter buffer empty */
		}
		flexio_base->SHIFTBUF[shifter_idx] = data[i];
	}

	/* Wait for last byte to leave the buffer */
	while (!(FLEXIO_GetShifterStatusFlags(flexio_base) & (1 << shifter_idx))) {
	}

	/* Wait for byte to fully shift out (10 bits at baud rate) */
	k_busy_wait(520);

	/*
	 * Release line to high-Z for half-duplex turnaround.
	 * External pull-up brings line HIGH. ESC can now respond.
	 */
	flexio_base->SHIFTCTL[shifter_idx] =
		FLEXIO_SHIFTCTL_PINCFG(0) | /* Output disabled (high-Z) */
		FLEXIO_SHIFTCTL_PINSEL(ch_info->pin_id) | FLEXIO_SHIFTCTL_PINPOL(0) |
		FLEXIO_SHIFTCTL_SMOD(0); /* Shifter disabled */

	k_busy_wait(10); /* Brief delay for line to settle */

	ch_info->mode = FLEXIO_UART_MODE_IDLE;
	return len;
}

/*
 * Public API: Read data from UART channel.
 *
 * Reads up to 'len' bytes with per-byte timeout. Returns number of bytes read.
 * Handles spurious 0xFF bytes that may occur during TX-to-RX transition.
 */
int nxp_flexio_uart_read(const struct device *dev, uint8_t channel, uint8_t *data, size_t len,
			 uint32_t timeout_us)
{
	const struct nxp_flexio_uart_config *config = dev->config;
	FLEXIO_Type *flexio_base = config->flexio_base;
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)config->child;

	if (channel >= config->channel->channel_count) {
		return -EINVAL;
	}

	nxp_flexio_uart_rx_enable(dev, channel);

	uint8_t shifter_idx = child->res.shifter_index[channel];
	size_t received = 0;

	/*
	 * Handle potential spurious byte from TX-to-RX transition.
	 * A 0xFF typically indicates line was HIGH (idle) during transition.
	 * Any other value is likely real data that arrived immediately.
	 */
	if (FLEXIO_GetShifterStatusFlags(flexio_base) & (1 << shifter_idx)) {
		uint8_t byte = (uint8_t)flexio_base->SHIFTBUFBYS[shifter_idx];
		if (byte == 0xFF) {
			LOG_DBG("RX ch%d: discarded spurious 0xFF", channel);
		} else {
			data[0] = byte;
			LOG_DBG("RX ch%d byte1: 0x%02X (immediate)", channel, byte);
			received = 1;
		}
	}

	FLEXIO_ClearShifterStatusFlags(flexio_base, 1 << shifter_idx);
	uint32_t start_time = k_uptime_get_32();

	while (received < len) {
		uint32_t elapsed_ms = k_uptime_get_32() - start_time;
		if ((uint64_t)elapsed_ms * 1000 > timeout_us) {
			break;
		}

		if (FLEXIO_GetShifterStatusFlags(flexio_base) & (1 << shifter_idx)) {
			uint8_t byte = (uint8_t)flexio_base->SHIFTBUFBYS[shifter_idx];
			data[received++] = byte;
			LOG_DBG("RX ch%d byte%d: 0x%02X", channel, (int)received, byte);
			start_time = k_uptime_get_32();
		}
	}

	return received;
}

/* Public API: Get channel count */
uint8_t nxp_flexio_uart_channel_count(const struct device *dev)
{
	const struct nxp_flexio_uart_config *config = dev->config;
	return config->channel->channel_count;
}

/* Public API: Check if TX is complete */
bool nxp_flexio_uart_tx_complete(const struct device *dev, uint8_t channel)
{
	const struct nxp_flexio_uart_config *config = dev->config;
	FLEXIO_Type *flexio_base = config->flexio_base;
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)config->child;

	if (channel >= config->channel->channel_count) {
		return true;
	}

	uint8_t shifter_idx = child->res.shifter_index[channel];
	return (FLEXIO_GetShifterStatusFlags(flexio_base) & (1 << shifter_idx)) != 0;
}

/*
 * Public API: Direct pin control.
 *
 * Sets the pin to a specific level by using the FlexIO shifter in
 * disabled mode with output enabled. Bit 0 of SHIFTBUF drives the pin.
 */
int nxp_flexio_uart_set_pin(const struct device *dev, uint8_t channel, bool level)
{
	const struct nxp_flexio_uart_config *config = dev->config;
	FLEXIO_Type *flexio_base = config->flexio_base;
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)config->child;
	struct nxp_flexio_uart_channel_config *ch_info = &config->channel->channel_info[channel];

	if (channel >= config->channel->channel_count) {
		return -EINVAL;
	}

	uint8_t shifter_idx = child->res.shifter_index[channel];

	flexio_base->SHIFTBUF[shifter_idx] = level ? 1 : 0;
	flexio_base->SHIFTCTL[shifter_idx] = FLEXIO_SHIFTCTL_PINCFG(3) | /* Output enabled */
					     FLEXIO_SHIFTCTL_PINSEL(ch_info->pin_id) |
					     FLEXIO_SHIFTCTL_PINPOL(0) |
					     FLEXIO_SHIFTCTL_SMOD(0); /* Disabled mode */

	ch_info->mode = FLEXIO_UART_MODE_PIN;
	return 0;
}

/* Devicetree instantiation macros */
#define _FLEXIO_UART_GEN_CONFIG(n)                                                                 \
	{                                                                                          \
		.pin_id = DT_PROP(n, pin_id),                                                      \
	},

#define FLEXIO_UART_GEN_CONFIG(n)                                                                  \
	static struct nxp_flexio_uart_channel_config flexio_uart_##n##_init[] = {                  \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(n, _FLEXIO_UART_GEN_CONFIG)};                    \
	static const struct nxp_flexio_uart_channel flexio_uart_##n##_info = {                     \
		.channel_count = ARRAY_SIZE(flexio_uart_##n##_init),                               \
		.channel_info = flexio_uart_##n##_init,                                            \
	};

#define FLEXIO_UART_FLEXIO_INDEX_INIT(n)                                                           \
	static uint8_t flexio_uart_##n##_timer_index[ARRAY_SIZE(flexio_uart_##n##_init)];          \
	static uint8_t flexio_uart_##n##_shifter_index[ARRAY_SIZE(flexio_uart_##n##_init)];

#define FLEXIO_UART_CHILD_CONFIG(n)                                                                \
	static const struct nxp_flexio_child flexio_uart_child_##n = {                             \
		.isr = NULL,                                                                       \
		.user_data = (void *)DEVICE_DT_INST_GET(n),                                        \
		.res = {.shifter_index = (uint8_t *)flexio_uart_##n##_shifter_index,               \
			.shifter_count = ARRAY_SIZE(flexio_uart_##n##_init),                       \
			.timer_index = (uint8_t *)flexio_uart_##n##_timer_index,                   \
			.timer_count = ARRAY_SIZE(flexio_uart_##n##_init)}};

#define FLEXIO_UART_GEN_GET_CONFIG(n) .channel = &flexio_uart_##n##_info,

#define NXP_FLEXIO_UART_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	FLEXIO_UART_GEN_CONFIG(n)                                                                  \
	FLEXIO_UART_FLEXIO_INDEX_INIT(n)                                                           \
	FLEXIO_UART_CHILD_CONFIG(n)                                                                \
	static const struct nxp_flexio_uart_config nxp_flexio_uart_config_##n = {                  \
		.flexio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                    \
		.flexio_base = (FLEXIO_Type *)DT_REG_ADDR(DT_INST_PARENT(n)),                      \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(n))),                     \
		.clock_subsys = (clock_control_subsys_t)DT_CLOCKS_CELL(DT_INST_PARENT(n), name),   \
		.child = &flexio_uart_child_##n,                                                   \
		.baud_rate = DT_INST_PROP(n, current_speed),                                       \
		FLEXIO_UART_GEN_GET_CONFIG(n)};                                                    \
                                                                                                   \
	static struct nxp_flexio_uart_data nxp_flexio_uart_data_##n;                               \
	DEVICE_DT_INST_DEFINE(n, &nxp_flexio_uart_init, NULL, &nxp_flexio_uart_data_##n,           \
			      &nxp_flexio_uart_config_##n, POST_KERNEL,                            \
			      CONFIG_NXP_FLEXIO_UART_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(NXP_FLEXIO_UART_INIT)
