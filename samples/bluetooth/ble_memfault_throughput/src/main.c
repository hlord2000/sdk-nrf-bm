/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <ble_gap.h>
#include <ble_gatts.h>
#include <nrf_error.h>

#include <bm/bluetooth/ble_adv.h>
#include <bm/bluetooth/ble_conn_params.h>
#include <bm/bluetooth/ble_qwr.h>
#include <bm/bluetooth/services/ble_dis.h>
#include <bm/bluetooth/services/ble_mcumgr.h>
#include <bm/bluetooth/services/ble_mds.h>
#include <bm/bluetooth/services/ble_nus.h>
#include <bm/bm_scheduler.h>
#include <bm/shell/backend_bm_uarte.h>
#include <bm/softdevice_handler/nrf_sdh.h>
#include <bm/softdevice_handler/nrf_sdh_ble.h>

#include <memfault/core/log.h>
#include <memfault/core/trace_event.h>
#include <memfault/metrics/metrics.h>
#include <memfault/metrics/platform/timer.h>

#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/retention/bootmode.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(sample, CONFIG_SAMPLE_BLE_MEMFAULT_THROUGHPUT_LOG_LEVEL);

#define PAYLOAD_MAX_LEN BLE_NUS_MAX_DATA_LEN
#define DEFAULT_SUP_TIMEOUT 400
#define MIN_CONN_INTERVAL_UNITS 6
#define MAX_CONN_INTERVAL_UNITS 3200
#define MIN_SUP_TIMEOUT_UNITS 10
#define MAX_SUP_TIMEOUT_UNITS 3200
#define MIN_DATA_LENGTH 27
#define MAX_DATA_LENGTH 251
#define TX_WINDOW_MIN 1
#define TX_WINDOW_MAX 32

BLE_ADV_DEF(ble_adv);
BLE_MDS_DEF(ble_mds);
BLE_NUS_DEF(ble_nus);
BLE_QWR_DEF(ble_qwr);

static atomic_t running = ATOMIC_INIT(1);
static volatile bool should_reboot_to_firmware_loader;
static volatile bool reset_response_sent;
static volatile bool reset_peer_disconnected;

static enum mgmt_cb_return os_mgmt_reboot_hook(uint32_t event,
					       enum mgmt_cb_return prev_status,
					       int32_t *rc, uint16_t *group,
					       bool *abort_more, void *data,
					       size_t data_size)
{
	ARG_UNUSED(prev_status);
	ARG_UNUSED(group);
	ARG_UNUSED(abort_more);
	ARG_UNUSED(data_size);

	if (event == MGMT_EVT_OP_OS_MGMT_RESET) {
		struct os_mgmt_reset_data *cb_data = data;

		if (cb_data != NULL) {
			(void)bootmode_set(cb_data->boot_mode);
		}

		should_reboot_to_firmware_loader = true;
		reset_response_sent = false;
		reset_peer_disconnected = false;
		*rc = MGMT_ERR_EOK;

		return MGMT_CB_ERROR_RC;
	}

	return MGMT_CB_OK;
}

static struct mgmt_callback os_mgmt_reboot_callback = {
	.callback = os_mgmt_reboot_hook,
	.event_id = MGMT_EVT_OP_OS_MGMT_RESET,
};

#if IS_ENABLED(CONFIG_MEMFAULT) && !IS_ENABLED(CONFIG_MULTITHREADING)
/* Memfault's Zephyr port uses k_mutex for SDK locking. BM samples are
 * single-context, so the no-thread syscall hooks collapse to no-ops here.
 */
int z_impl_k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout)
{
	ARG_UNUSED(mutex);
	ARG_UNUSED(timeout);

	return 0;
}

int z_impl_k_mutex_unlock(struct k_mutex *mutex)
{
	ARG_UNUSED(mutex);

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_MEMFAULT_METRICS_TIMER_CUSTOM)
static MemfaultPlatformTimerCallback *memfault_metrics_timer_cb;
static int64_t memfault_metrics_period_ms;
static int64_t memfault_metrics_next_ms;

bool memfault_platform_metrics_timer_boot(uint32_t period_sec,
					  MemfaultPlatformTimerCallback callback)
{
	if ((period_sec == 0) || (callback == NULL)) {
		return false;
	}

	memfault_metrics_timer_cb = callback;
	memfault_metrics_period_ms = (int64_t)period_sec * 1000;
	memfault_metrics_next_ms = k_uptime_get() + memfault_metrics_period_ms;

	return true;
}

static void memfault_metrics_timer_process(void)
{
	int64_t now;

	if (memfault_metrics_timer_cb == NULL) {
		return;
	}

	now = k_uptime_get();
	if (now < memfault_metrics_next_ms) {
		return;
	}

	memfault_metrics_next_ms += memfault_metrics_period_ms;
	if (now >= memfault_metrics_next_ms) {
		memfault_metrics_next_ms = now + memfault_metrics_period_ms;
	}

	memfault_metrics_timer_cb();
}
#else
static void memfault_metrics_timer_process(void)
{
}
#endif

struct throughput_state {
	uint16_t conn_handle;
	bool notifications_enabled;
	bool test_running;
	bool tx_blocked;
	bool payload_tracks_mtu;
	uint16_t tx_in_flight;
	uint16_t tx_window;

	uint16_t att_mtu;
	uint16_t payload_len;
	uint16_t data_len_tx;
	uint16_t data_len_rx;
	ble_gap_phys_t phy;
	ble_gap_conn_params_t conn_params;

	uint32_t duration_ms;
	int64_t start_ms;
	int64_t last_report_ms;

	uint64_t tx_bytes;
	uint64_t rx_bytes;
	uint32_t tx_notifications;
	uint32_t rx_writes;
	uint32_t tx_complete_events;
	uint32_t resource_errors;
	uint32_t hvx_errors;
	uint32_t partial_notifications;
	uint32_t start_requests;
	uint32_t pump_calls;
	uint32_t hvx_attempts;
	uint32_t last_hvx_nrf_error;
	uint8_t last_disconnect_reason;
	uint32_t seq;
};

static struct throughput_state tput = {
	.conn_handle = BLE_CONN_HANDLE_INVALID,
	.payload_tracks_mtu = true,
	.tx_window = CONFIG_SAMPLE_BLE_MEMFAULT_THROUGHPUT_TX_WINDOW,
	.att_mtu = BLE_GATT_ATT_MTU_DEFAULT,
	.payload_len = BLE_NUS_MAX_DATA_LEN_CALC(BLE_GATT_ATT_MTU_DEFAULT),
	.data_len_tx = MIN_DATA_LENGTH,
	.data_len_rx = MIN_DATA_LENGTH,
	.phy = {
		.tx_phys = BLE_GAP_PHY_AUTO,
		.rx_phys = BLE_GAP_PHY_AUTO,
	},
	.conn_params = {
		.min_conn_interval = CONFIG_BLE_CONN_PARAMS_MIN_CONN_INTERVAL,
		.max_conn_interval = CONFIG_BLE_CONN_PARAMS_MAX_CONN_INTERVAL,
		.slave_latency = CONFIG_BLE_CONN_PARAMS_PERIPHERAL_LATENCY,
		.conn_sup_timeout = CONFIG_BLE_CONN_PARAMS_SUP_TIMEOUT,
	},
};

static uint8_t payload[PAYLOAD_MAX_LEN];

static uint16_t current_max_payload_len(void)
{
	if (tput.att_mtu <= BLE_GATT_ATT_MTU_DEFAULT) {
		return BLE_NUS_MAX_DATA_LEN_CALC(BLE_GATT_ATT_MTU_DEFAULT);
	}

	return MIN(PAYLOAD_MAX_LEN, BLE_NUS_MAX_DATA_LEN_CALC(tput.att_mtu));
}

static uint16_t effective_payload_len(void)
{
	return MIN(tput.payload_len, current_max_payload_len());
}

static uint32_t conn_interval_us(uint16_t interval_units)
{
	return (uint32_t)interval_units * 1250U;
}

static uint16_t conn_sup_timeout_min_units(uint16_t max_conn_interval, uint16_t latency)
{
	uint32_t min_timeout;

	/* BLE supervision timeout, in 10 ms units, must be strictly greater
	 * than 2 * (1 + latency) * max_conn_interval.
	 */
	min_timeout = (((uint32_t)latency + 1U) * max_conn_interval) / 4U + 1U;

	return (uint16_t)MAX(min_timeout, MIN_SUP_TIMEOUT_UNITS);
}

static uint16_t conn_sup_timeout_default_units(uint16_t max_conn_interval, uint16_t latency)
{
	uint32_t preferred;

	/* Use roughly 3x the effective interval, with a 4 s floor for short
	 * intervals. This keeps 4 s interval characterization legal without
	 * requiring the user to calculate supervision timeout units.
	 */
	preferred = DIV_ROUND_UP(((uint32_t)latency + 1U) * max_conn_interval * 3U, 8U);
	preferred = MAX(preferred, DEFAULT_SUP_TIMEOUT);
	preferred = MAX(preferred, conn_sup_timeout_min_units(max_conn_interval, latency));

	return (uint16_t)MIN(preferred, MAX_SUP_TIMEOUT_UNITS);
}

static bool conn_sup_timeout_is_valid(const ble_gap_conn_params_t *params)
{
	return ((uint32_t)params->conn_sup_timeout * 4U) >
	       (((uint32_t)params->slave_latency + 1U) * params->max_conn_interval);
}

static void conn_params_defaults_set(void)
{
	tput.conn_params.min_conn_interval = CONFIG_BLE_CONN_PARAMS_MIN_CONN_INTERVAL;
	tput.conn_params.max_conn_interval = CONFIG_BLE_CONN_PARAMS_MAX_CONN_INTERVAL;
	tput.conn_params.slave_latency = CONFIG_BLE_CONN_PARAMS_PERIPHERAL_LATENCY;
	tput.conn_params.conn_sup_timeout = CONFIG_BLE_CONN_PARAMS_SUP_TIMEOUT;
}

static uint32_t elapsed_ms_get(void)
{
	if (tput.start_ms <= 0) {
		return 0;
	}

	return (uint32_t)MAX((int64_t)0, k_uptime_get() - tput.start_ms);
}

static uint32_t throughput_bps_get(void)
{
	const uint32_t elapsed_ms = elapsed_ms_get();

	if (elapsed_ms == 0) {
		return 0;
	}

	return (uint32_t)MIN(UINT32_MAX, (tput.tx_bytes * 8000ULL) / elapsed_ms);
}

static bool throughput_can_pump(void)
{
	return tput.test_running && !tput.tx_blocked &&
	       (tput.conn_handle != BLE_CONN_HANDLE_INVALID) &&
	       tput.notifications_enabled &&
	       (tput.tx_in_flight < tput.tx_window);
}

static void metrics_update(void)
{
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_tx_bytes, MIN(tput.tx_bytes, UINT32_MAX));
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_tx_notifications, tput.tx_notifications);
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_rx_bytes, MIN(tput.rx_bytes, UINT32_MAX));
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_rx_writes, tput.rx_writes);
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_bps, throughput_bps_get());
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_duration_ms, elapsed_ms_get());
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_att_mtu, tput.att_mtu);
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_payload_len, effective_payload_len());
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_tx_window, tput.tx_window);
	(void)MEMFAULT_METRIC_SET_UNSIGNED(
		tput_conn_interval_us, conn_interval_us(tput.conn_params.max_conn_interval));
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_data_length_tx, tput.data_len_tx);
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_data_length_rx, tput.data_len_rx);
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_phy_tx, tput.phy.tx_phys);
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_phy_rx, tput.phy.rx_phys);
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_resource_errors, tput.resource_errors);
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_hvx_errors, tput.hvx_errors);
	(void)MEMFAULT_METRIC_SET_UNSIGNED(tput_partial_notifications,
					    tput.partial_notifications);
}

static void stats_reset(void)
{
	tput.tx_bytes = 0;
	tput.rx_bytes = 0;
	tput.tx_notifications = 0;
	tput.rx_writes = 0;
	tput.tx_complete_events = 0;
	tput.resource_errors = 0;
	tput.hvx_errors = 0;
	tput.partial_notifications = 0;
	tput.pump_calls = 0;
	tput.hvx_attempts = 0;
	tput.last_hvx_nrf_error = NRF_SUCCESS;
	tput.seq = 0;
	tput.tx_blocked = false;
}

static void link_runtime_reset(void)
{
	tput.notifications_enabled = false;
	tput.tx_blocked = false;
	tput.tx_in_flight = 0;
	tput.duration_ms = 0;
	tput.start_ms = 0;
	tput.last_report_ms = 0;
	tput.att_mtu = BLE_GATT_ATT_MTU_DEFAULT;
	tput.data_len_tx = MIN_DATA_LENGTH;
	tput.data_len_rx = MIN_DATA_LENGTH;
	tput.phy.tx_phys = BLE_GAP_PHY_AUTO;
	tput.phy.rx_phys = BLE_GAP_PHY_AUTO;

	if (tput.payload_tracks_mtu) {
		tput.payload_len = current_max_payload_len();
	}
}

static void throughput_summary_log(const char *reason)
{
	LOG_INF("Throughput %s: %u ms, %llu bytes, %u notifications, %u bit/s, "
		"resources %u, hvx_errors %u, partial %u",
		reason, elapsed_ms_get(), tput.tx_bytes, tput.tx_notifications,
		throughput_bps_get(), tput.resource_errors, tput.hvx_errors,
		tput.partial_notifications);
}

static void throughput_memfault_publish(void)
{
	/* Ensure the deferred Zephyr log backend has captured the summary before
	 * the Memfault log data source snapshots the RAM log buffer.
	 */
	log_flush();
	memfault_metrics_heartbeat_debug_trigger();
	memfault_log_trigger_collection();
}

static int throughput_stop_internal(const char *reason)
{
	if (!tput.test_running) {
		return -EALREADY;
	}

	tput.test_running = false;
	tput.tx_blocked = false;
	metrics_update();
	throughput_summary_log(reason);
	MEMFAULT_TRACE_EVENT(tput_stopped);
	throughput_memfault_publish();

	return 0;
}

static int throughput_start_internal(uint32_t duration_ms)
{
	if (tput.conn_handle == BLE_CONN_HANDLE_INVALID) {
		return -ENOTCONN;
	}

	if (!tput.notifications_enabled) {
		return -EACCES;
	}

	if (tput.test_running) {
		return -EALREADY;
	}

	if (tput.tx_in_flight != 0) {
		return -EBUSY;
	}

	stats_reset();
	tput.duration_ms = duration_ms;
	tput.start_ms = k_uptime_get();
	tput.last_report_ms = tput.start_ms;
	tput.test_running = true;
	tput.start_requests++;

	LOG_INF("Throughput started: duration %u ms, payload %u/%u bytes, MTU %u, window %u",
		duration_ms, effective_payload_len(), current_max_payload_len(), tput.att_mtu,
		tput.tx_window);
	MEMFAULT_TRACE_EVENT(tput_started);

	return 0;
}

static void payload_prepare(uint16_t len)
{
	const uint32_t seq = tput.seq++;

	if (len >= sizeof(uint32_t)) {
		sys_put_le32(seq, &payload[0]);
	}

	if (len >= (2 * sizeof(uint32_t))) {
		sys_put_le32((uint32_t)tput.tx_bytes, &payload[4]);
	}

	for (uint16_t i = 8; i < len; i++) {
		payload[i] = (uint8_t)(seq + i);
	}
}

static void throughput_pump(void)
{
	if (!throughput_can_pump()) {
		return;
	}

	tput.pump_calls++;

	if ((tput.duration_ms != 0) && (elapsed_ms_get() >= tput.duration_ms)) {
		(void)throughput_stop_internal("complete");
		return;
	}

	while (throughput_can_pump()) {
		uint16_t requested_len = effective_payload_len();
		uint16_t sent_len = requested_len;
		uint32_t nrf_err;

		if (requested_len == 0) {
			(void)throughput_stop_internal("zero-payload");
			return;
		}

		payload_prepare(requested_len);

		tput.hvx_attempts++;
		nrf_err = ble_nus_data_send(&ble_nus, payload, &sent_len, tput.conn_handle);
		tput.last_hvx_nrf_error = nrf_err;
		if (nrf_err == NRF_SUCCESS) {
			tput.tx_bytes += sent_len;
			tput.tx_notifications++;
			tput.tx_in_flight++;

			if (sent_len != requested_len) {
				tput.partial_notifications++;
			}
			continue;
		}

		if (nrf_err == NRF_ERROR_RESOURCES) {
			tput.resource_errors++;
			tput.tx_blocked = true;
			return;
		}

		tput.hvx_errors++;

		if (nrf_err == NRF_ERROR_INVALID_STATE) {
			LOG_DBG("Notification temporarily unavailable, nrf_error %#x", nrf_err);
			tput.tx_blocked = true;
			return;
		}

		if (nrf_err == BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
			LOG_WRN("Stopping throughput, notifications unavailable, nrf_error %#x",
				nrf_err);
			(void)throughput_stop_internal("notify-unavailable");
			return;
		}

		LOG_ERR("sd_ble_gatts_hvx failed, nrf_error %#x", nrf_err);
		tput.tx_blocked = true;
		return;
	}
}

static void throughput_report_if_due(void)
{
	const int64_t now = k_uptime_get();

	if (!tput.test_running) {
		return;
	}

	if ((now - tput.last_report_ms) < CONFIG_SAMPLE_BLE_MEMFAULT_THROUGHPUT_REPORT_INTERVAL_MS) {
		return;
	}

	tput.last_report_ms = now;
	metrics_update();

	LOG_INF("Throughput: %u ms, %llu bytes, %u notifications, %u bit/s",
		elapsed_ms_get(), tput.tx_bytes, tput.tx_notifications, throughput_bps_get());
}

static void mcumgr_reboot_process(void)
{
	uint32_t nrf_err;

	if (!should_reboot_to_firmware_loader) {
		return;
	}

	if (tput.test_running) {
		(void)throughput_stop_internal("mcumgr-reset");
	}

	if ((tput.conn_handle != BLE_CONN_HANDLE_INVALID) && !reset_peer_disconnected) {
		if (!reset_response_sent) {
			return;
		}

		nrf_err = sd_ble_gap_disconnect(tput.conn_handle,
						BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		if ((nrf_err == NRF_SUCCESS) || (nrf_err == NRF_ERROR_INVALID_STATE) ||
		    (nrf_err == BLE_ERROR_INVALID_CONN_HANDLE)) {
			if (nrf_err != NRF_SUCCESS) {
				reset_peer_disconnected = true;
			}
			return;
		}

		LOG_WRN("Failed to disconnect before reset, nrf_error %#x", nrf_err);
		reset_peer_disconnected = true;
		return;
	}

	LOG_INF("Rebooting to firmware loader");
	log_flush();
	sys_reboot(SYS_REBOOT_WARM);
}

static void data_length_max_request(const char *reason)
{
	const struct ble_conn_params_data_length dl = {
		.tx = MAX_DATA_LENGTH,
		.rx = MAX_DATA_LENGTH,
	};
	uint32_t nrf_err;

	if (tput.conn_handle == BLE_CONN_HANDLE_INVALID) {
		return;
	}

	if ((tput.data_len_tx >= MAX_DATA_LENGTH) && (tput.data_len_rx >= MAX_DATA_LENGTH)) {
		return;
	}

	nrf_err = ble_conn_params_data_length_set(tput.conn_handle, dl);
	if (nrf_err) {
		LOG_WRN("Data length max request after %s failed, nrf_error %#x",
			reason, nrf_err);
	} else {
		LOG_INF("Data length max request sent after %s", reason);
	}
}

static void state_print(const struct shell *sh)
{
	shell_print(sh, "connected: %s handle: 0x%04x notify: %s running: %s",
		    (tput.conn_handle == BLE_CONN_HANDLE_INVALID) ? "no" : "yes",
		    tput.conn_handle,
		    tput.notifications_enabled ? "yes" : "no",
		    tput.test_running ? "yes" : "no");
	shell_print(sh, "att_mtu: %u payload: %u max_payload: %u payload_mode: %s",
		    tput.att_mtu, effective_payload_len(), current_max_payload_len(),
		    tput.payload_tracks_mtu ? "max" : "fixed");
	shell_print(sh, "data_length: tx %u rx %u phy: tx %u rx %u",
		    tput.data_len_tx, tput.data_len_rx, tput.phy.tx_phys, tput.phy.rx_phys);
	shell_print(sh, "conn: min %u (%u us) max %u (%u us) latency %u timeout %u (%u ms)",
		    tput.conn_params.min_conn_interval,
		    conn_interval_us(tput.conn_params.min_conn_interval),
		    tput.conn_params.max_conn_interval,
		    conn_interval_us(tput.conn_params.max_conn_interval),
		    tput.conn_params.slave_latency,
		    tput.conn_params.conn_sup_timeout,
		    tput.conn_params.conn_sup_timeout * 10U);
	shell_print(sh, "tx: %llu bytes %u notifications %u bit/s elapsed %u ms",
		    tput.tx_bytes, tput.tx_notifications, throughput_bps_get(), elapsed_ms_get());
	shell_print(sh, "tx_flow: in_flight %u window %u complete %u resources %u hvx_errors %u",
		    tput.tx_in_flight, tput.tx_window, tput.tx_complete_events,
		    tput.resource_errors, tput.hvx_errors);
	shell_print(sh, "diag: starts %u pumps %u hvx_attempts %u last_hvx %#x last_disc %#x",
		    tput.start_requests, tput.pump_calls, tput.hvx_attempts,
		    tput.last_hvx_nrf_error, tput.last_disconnect_reason);
	shell_print(sh, "rx: %llu bytes %u writes", tput.rx_bytes, tput.rx_writes);
}

static int parse_u32_value(const char *arg, uint32_t *out)
{
	uint32_t value = 0;
	uint8_t base = 10;
	const char *p = arg;

	if ((arg == NULL) || (arg[0] == '\0') || (arg[0] == '-')) {
		return -EINVAL;
	}

	if (arg[0] == '0') {
		if ((arg[1] == 'x') || (arg[1] == 'X')) {
			base = 16;
			p = &arg[2];
			if (*p == '\0') {
				return -EINVAL;
			}
		} else {
			base = 8;
		}
	}

	for (; *p != '\0'; p++) {
		uint8_t digit;

		if ((*p >= '0') && (*p <= '9')) {
			digit = (uint8_t)(*p - '0');
		} else if (base == 16 && (*p >= 'a') && (*p <= 'f')) {
			digit = (uint8_t)(*p - 'a' + 10);
		} else if (base == 16 && (*p >= 'A') && (*p <= 'F')) {
			digit = (uint8_t)(*p - 'A' + 10);
		} else {
			return -EINVAL;
		}

		if (digit >= base) {
			return -EINVAL;
		}

		if (value > ((UINT32_MAX - digit) / base)) {
			return -ERANGE;
		}

		value = (value * base) + digit;
	}

	*out = value;
	return 0;
}

static int parse_u16(const struct shell *sh, const char *arg, uint16_t min,
		     uint16_t max, uint16_t *out)
{
	uint32_t value;
	int err = parse_u32_value(arg, &value);

	if (err || value < min || value > max) {
		shell_error(sh, "Expected %u..%u, got %s", min, max, arg);
		return -EINVAL;
	}

	*out = (uint16_t)value;
	return 0;
}

static int parse_u32(const struct shell *sh, const char *arg, uint32_t *out)
{
	int err = parse_u32_value(arg, out);

	if (err) {
		shell_error(sh, "Invalid value: %s", arg);
		return -EINVAL;
	}

	return 0;
}

static int parse_conn_interval_ms(const struct shell *sh, const char *arg, uint16_t min_units,
				  uint16_t *out)
{
	uint32_t ms;
	uint32_t units_x5;
	uint32_t units;
	int err;

	err = parse_u32(sh, arg, &ms);
	if (err) {
		return err;
	}

	if (ms > (UINT32_MAX / 4U)) {
		shell_error(sh, "Connection interval too large: %s ms", arg);
		return -EINVAL;
	}

	units_x5 = ms * 4U;
	if ((units_x5 % 5U) != 0U) {
		shell_error(sh,
			    "Integer-ms intervals must map exactly to 1.25 ms units; got %s ms",
			    arg);
		return -EINVAL;
	}

	units = units_x5 / 5U;
	if ((units < min_units) || (units > MAX_CONN_INTERVAL_UNITS)) {
		shell_error(sh, "Expected %u..%u units, got %u units from %s ms",
			    min_units, MAX_CONN_INTERVAL_UNITS, units, arg);
		return -EINVAL;
	}

	*out = (uint16_t)units;
	return 0;
}

static int parse_sup_timeout_ms(const struct shell *sh, const char *arg, uint16_t *out)
{
	uint32_t ms;
	uint32_t units;
	int err;

	err = parse_u32(sh, arg, &ms);
	if (err) {
		return err;
	}

	if ((ms % 10U) != 0U) {
		shell_error(sh, "Supervision timeout must be a 10 ms multiple; got %s ms", arg);
		return -EINVAL;
	}

	units = ms / 10U;
	if ((units < MIN_SUP_TIMEOUT_UNITS) || (units > MAX_SUP_TIMEOUT_UNITS)) {
		shell_error(sh, "Expected %u..%u timeout units, got %u units from %s ms",
			    MIN_SUP_TIMEOUT_UNITS, MAX_SUP_TIMEOUT_UNITS, units, arg);
		return -EINVAL;
	}

	*out = (uint16_t)units;
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	metrics_update();
	state_print(sh);

	return 0;
}

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t seconds = 0;
	uint32_t duration_ms = 0;
	int err;

	if (argc > 1) {
		err = parse_u32(sh, argv[1], &seconds);
		if (err) {
			return err;
		}

		if (seconds > (UINT32_MAX / 1000U)) {
			shell_error(sh, "Duration too large");
			return -EINVAL;
		}
		duration_ms = seconds * 1000U;
	}

	if (argc > 2) {
		if (!strcmp(argv[2], "max")) {
			tput.payload_tracks_mtu = true;
			tput.payload_len = current_max_payload_len();
		} else {
			uint16_t payload_len;

			err = parse_u16(sh, argv[2], 1, PAYLOAD_MAX_LEN, &payload_len);
			if (err) {
				return err;
			}

			tput.payload_tracks_mtu = false;
			tput.payload_len = payload_len;
		}
	}

	err = throughput_start_internal(duration_ms);
	if (err == -ENOTCONN) {
		shell_error(sh, "No active connection");
	} else if (err == -EACCES) {
		shell_error(sh, "Enable notifications on NUS TX first");
	} else if (err == -EALREADY) {
		shell_error(sh, "Throughput already running");
	} else if (err == -EBUSY) {
		shell_error(sh, "Waiting for outstanding notifications to complete");
	} else if (err) {
		shell_error(sh, "Start failed: %d", err);
	}

	return err;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	err = throughput_stop_internal("stopped");
	if (err == -EALREADY) {
		shell_error(sh, "Throughput is not running");
		return err;
	}

	state_print(sh);

	return err;
}

static int cmd_payload(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t payload_len;
	int err;

	if (argc != 2) {
		shell_error(sh, "Usage: tput payload <len|max>");
		return -EINVAL;
	}

	if (!strcmp(argv[1], "max")) {
		tput.payload_tracks_mtu = true;
		tput.payload_len = current_max_payload_len();
	} else {
		err = parse_u16(sh, argv[1], 1, PAYLOAD_MAX_LEN, &payload_len);
		if (err) {
			return err;
		}

		tput.payload_tracks_mtu = false;
		tput.payload_len = payload_len;
	}

	shell_print(sh, "payload: %u bytes (%s)", effective_payload_len(),
		    tput.payload_tracks_mtu ? "max" : "fixed");

	return 0;
}

static int cmd_window(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t tx_window;
	int err;

	if (argc != 2) {
		shell_error(sh, "Usage: tput window <count>");
		return -EINVAL;
	}

	err = parse_u16(sh, argv[1], TX_WINDOW_MIN, TX_WINDOW_MAX, &tx_window);
	if (err) {
		return err;
	}

	tput.tx_window = tx_window;
	shell_print(sh, "tx window: %u outstanding notification%s",
		    tput.tx_window, tput.tx_window == 1 ? "" : "s");

	return 0;
}

static int cmd_mtu(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t mtu;
	uint32_t nrf_err;
	int err;

	if (argc != 2) {
		shell_error(sh, "Usage: tput mtu <bytes|max>");
		return -EINVAL;
	}

	if (tput.conn_handle == BLE_CONN_HANDLE_INVALID) {
		shell_error(sh, "No active connection");
		return -ENOTCONN;
	}

	if (!strcmp(argv[1], "max")) {
		mtu = CONFIG_NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
	} else {
		err = parse_u16(sh, argv[1], BLE_GATT_ATT_MTU_DEFAULT,
				CONFIG_NRF_SDH_BLE_GATT_MAX_MTU_SIZE, &mtu);
		if (err) {
			return err;
		}
	}

	nrf_err = ble_conn_params_att_mtu_set(tput.conn_handle, mtu);
	if (nrf_err) {
		shell_error(sh, "ATT MTU request failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	shell_print(sh, "ATT MTU request sent: %u", mtu);
	return 0;
}

static int cmd_dlen(const struct shell *sh, size_t argc, char **argv)
{
	struct ble_conn_params_data_length dl;
	uint32_t nrf_err;
	int err;

	if (argc != 2 && argc != 3) {
		shell_error(sh, "Usage: tput dlen <tx> <rx> | tput dlen max");
		return -EINVAL;
	}

	if (tput.conn_handle == BLE_CONN_HANDLE_INVALID) {
		shell_error(sh, "No active connection");
		return -ENOTCONN;
	}

	if (!strcmp(argv[1], "max")) {
		dl.tx = MAX_DATA_LENGTH;
		dl.rx = MAX_DATA_LENGTH;
	} else {
		uint16_t tx;
		uint16_t rx;

		if (argc != 3) {
			shell_error(sh, "Usage: tput dlen <tx> <rx>");
			return -EINVAL;
		}

		err = parse_u16(sh, argv[1], MIN_DATA_LENGTH, MAX_DATA_LENGTH, &tx);
		if (err) {
			return err;
		}
		err = parse_u16(sh, argv[2], MIN_DATA_LENGTH, MAX_DATA_LENGTH, &rx);
		if (err) {
			return err;
		}

		dl.tx = (uint8_t)tx;
		dl.rx = (uint8_t)rx;
	}

	nrf_err = ble_conn_params_data_length_set(tput.conn_handle, dl);
	if (nrf_err) {
		shell_error(sh, "Data length request failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	shell_print(sh, "Data length request sent: tx %u rx %u", dl.tx, dl.rx);
	return 0;
}

static int cmd_phy(const struct shell *sh, size_t argc, char **argv)
{
	ble_gap_phys_t phy;
	uint32_t nrf_err;

	if (argc != 2) {
		shell_error(sh, "Usage: tput phy <auto|1m|2m|coded>");
		return -EINVAL;
	}

	if (tput.conn_handle == BLE_CONN_HANDLE_INVALID) {
		shell_error(sh, "No active connection");
		return -ENOTCONN;
	}

	if (!strcmp(argv[1], "auto")) {
		phy.tx_phys = BLE_GAP_PHY_AUTO;
		phy.rx_phys = BLE_GAP_PHY_AUTO;
	} else if (!strcmp(argv[1], "1m")) {
		phy.tx_phys = BLE_GAP_PHY_1MBPS;
		phy.rx_phys = BLE_GAP_PHY_1MBPS;
	} else if (!strcmp(argv[1], "2m")) {
		phy.tx_phys = BLE_GAP_PHY_2MBPS;
		phy.rx_phys = BLE_GAP_PHY_2MBPS;
	} else if (!strcmp(argv[1], "coded")) {
		phy.tx_phys = BLE_GAP_PHY_CODED;
		phy.rx_phys = BLE_GAP_PHY_CODED;
	} else {
		shell_error(sh, "Unknown PHY: %s", argv[1]);
		return -EINVAL;
	}

	nrf_err = ble_conn_params_phy_radio_mode_set(tput.conn_handle, phy);
	if (nrf_err) {
		shell_error(sh, "PHY request failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	shell_print(sh, "PHY request sent: tx %u rx %u", phy.tx_phys, phy.rx_phys);
	return 0;
}

static int conn_param_request(const struct shell *sh, const ble_gap_conn_params_t *params)
{
	uint32_t nrf_err;
	uint16_t min_timeout;

	if (tput.conn_handle == BLE_CONN_HANDLE_INVALID) {
		shell_error(sh, "No active connection");
		return -ENOTCONN;
	}

	if (!conn_sup_timeout_is_valid(params)) {
		min_timeout =
			conn_sup_timeout_min_units(params->max_conn_interval, params->slave_latency);
		shell_error(sh,
			    "Supervision timeout too short; need at least %u units (%u ms)",
			    min_timeout, min_timeout * 10U);
		return -EINVAL;
	}

	nrf_err = ble_conn_params_override(tput.conn_handle, params);
	if (nrf_err) {
		shell_error(sh, "Connection parameter request failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	shell_print(sh, "Connection parameter request sent: min %u max %u latency %u timeout %u",
		    params->min_conn_interval, params->max_conn_interval,
		    params->slave_latency, params->conn_sup_timeout);

	return 0;
}

static int cmd_conn(const struct shell *sh, size_t argc, char **argv)
{
	ble_gap_conn_params_t params = {
		.slave_latency = 0,
		.conn_sup_timeout = DEFAULT_SUP_TIMEOUT,
	};
	int err;

	if (argc == 2 && !strcmp(argv[1], "min")) {
		params.min_conn_interval = MIN_CONN_INTERVAL_UNITS;
		params.max_conn_interval = MIN_CONN_INTERVAL_UNITS;
		return conn_param_request(sh, &params);
	}

	if (argc == 2 && !strcmp(argv[1], "max")) {
		params.min_conn_interval = MAX_CONN_INTERVAL_UNITS;
		params.max_conn_interval = MAX_CONN_INTERVAL_UNITS;
		params.conn_sup_timeout = MAX_SUP_TIMEOUT_UNITS;
		return conn_param_request(sh, &params);
	}

	if (argc != 3 && argc != 5) {
		shell_error(sh, "Usage: tput conn <min_units> <max_units> [latency timeout_units]");
		shell_error(sh, "       tput conn <min|max>");
		return -EINVAL;
	}

	err = parse_u16(sh, argv[1], MIN_CONN_INTERVAL_UNITS, MAX_CONN_INTERVAL_UNITS,
			&params.min_conn_interval);
	if (err) {
		return err;
	}

	err = parse_u16(sh, argv[2], params.min_conn_interval, MAX_CONN_INTERVAL_UNITS,
			&params.max_conn_interval);
	if (err) {
		return err;
	}

	if (argc == 5) {
		err = parse_u16(sh, argv[3], 0, 499, &params.slave_latency);
		if (err) {
			return err;
		}
		err = parse_u16(sh, argv[4], MIN_SUP_TIMEOUT_UNITS, MAX_SUP_TIMEOUT_UNITS,
				&params.conn_sup_timeout);
		if (err) {
			return err;
		}
	} else {
		params.conn_sup_timeout =
			conn_sup_timeout_default_units(params.max_conn_interval, params.slave_latency);
	}

	return conn_param_request(sh, &params);
}

static int cmd_conn_ms(const struct shell *sh, size_t argc, char **argv)
{
	ble_gap_conn_params_t params = {
		.slave_latency = 0,
	};
	int err;

	if (argc != 3 && argc != 5) {
		shell_error(sh, "Usage: tput conn_ms <min_ms> <max_ms> [latency timeout_ms]");
		shell_error(sh, "       use `tput conn min` for 7.5 ms");
		return -EINVAL;
	}

	err = parse_conn_interval_ms(sh, argv[1], MIN_CONN_INTERVAL_UNITS,
				     &params.min_conn_interval);
	if (err) {
		return err;
	}

	err = parse_conn_interval_ms(sh, argv[2], params.min_conn_interval,
				     &params.max_conn_interval);
	if (err) {
		return err;
	}

	if (argc == 5) {
		err = parse_u16(sh, argv[3], 0, 499, &params.slave_latency);
		if (err) {
			return err;
		}
		err = parse_sup_timeout_ms(sh, argv[4], &params.conn_sup_timeout);
		if (err) {
			return err;
		}
	} else {
		params.conn_sup_timeout =
			conn_sup_timeout_default_units(params.max_conn_interval, params.slave_latency);
	}

	return conn_param_request(sh, &params);
}

static int cmd_adv(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t nrf_err;

	if (argc != 2) {
		shell_error(sh, "Usage: tput adv <start|stop>");
		return -EINVAL;
	}

	if (!strcmp(argv[1], "start")) {
		nrf_err = ble_adv_start(&ble_adv, BLE_ADV_MODE_FAST);
	} else if (!strcmp(argv[1], "stop")) {
		nrf_err = ble_adv_stop(&ble_adv);
	} else {
		shell_error(sh, "Usage: tput adv <start|stop>");
		return -EINVAL;
	}

	if (nrf_err) {
		shell_error(sh, "Advertising command failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	shell_print(sh, "Advertising %s", argv[1]);
	return 0;
}

static int cmd_disconnect(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t nrf_err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (tput.conn_handle == BLE_CONN_HANDLE_INVALID) {
		shell_error(sh, "No active connection");
		return -ENOTCONN;
	}

	nrf_err = sd_ble_gap_disconnect(tput.conn_handle,
					BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	if (nrf_err) {
		shell_error(sh, "Disconnect failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	shell_print(sh, "Disconnect requested");
	return 0;
}

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (tput.test_running) {
		shell_error(sh, "Stop throughput before resetting stats");
		return -EBUSY;
	}

	if (tput.tx_in_flight != 0) {
		shell_error(sh, "Wait for outstanding notifications before resetting stats");
		return -EBUSY;
	}

	stats_reset();
	metrics_update();
	shell_print(sh, "stats reset");

	return 0;
}

static int cmd_quit(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	atomic_set(&running, 0);
	shell_print(sh, "terminating sample shell loop");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_tput,
	SHELL_CMD_ARG(status, NULL, "show throughput state", cmd_status, 1, 0),
	SHELL_CMD_ARG(start, NULL, "start notifications: start [seconds] [payload|max]",
		      cmd_start, 1, 2),
	SHELL_CMD_ARG(stop, NULL, "stop notifications", cmd_stop, 1, 0),
	SHELL_CMD_ARG(payload, NULL, "set payload: payload <len|max>", cmd_payload, 2, 0),
	SHELL_CMD_ARG(window, NULL, "set outstanding notification window: window <count>",
		      cmd_window, 2, 0),
	SHELL_CMD_ARG(mtu, NULL, "request ATT MTU: mtu <bytes|max>", cmd_mtu, 2, 0),
	SHELL_CMD_ARG(dlen, NULL, "request data length: dlen <tx> <rx> | dlen max",
		      cmd_dlen, 2, 1),
	SHELL_CMD_ARG(phy, NULL, "request PHY: phy <auto|1m|2m|coded>", cmd_phy, 2, 0),
	SHELL_CMD_ARG(conn, NULL, "request conn params: conn <min|max|units>",
		      cmd_conn, 2, 3),
	SHELL_CMD_ARG(conn_ms, NULL, "request conn params in ms: conn_ms <min_ms> <max_ms>",
		      cmd_conn_ms, 3, 2),
	SHELL_CMD_ARG(adv, NULL, "advertising control: adv <start|stop>", cmd_adv, 2, 0),
	SHELL_CMD_ARG(disconnect, NULL, "disconnect the active peer", cmd_disconnect, 1, 0),
	SHELL_CMD_ARG(reset, NULL, "reset counters", cmd_reset, 1, 0),
	SHELL_CMD_ARG(quit, NULL, "terminate shell loop", cmd_quit, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(tput, &sub_tput, "BLE throughput control", NULL);

static void on_ble_evt(const ble_evt_t *evt, void *ctx)
{
	uint32_t nrf_err;

	ARG_UNUSED(ctx);

	switch (evt->header.evt_id) {
	case BLE_GAP_EVT_CONNECTED:
		tput.conn_handle = evt->evt.gap_evt.conn_handle;
		tput.conn_params = evt->evt.gap_evt.params.connected.conn_params;
		tput.last_disconnect_reason = 0;
		link_runtime_reset();

		LOG_INF("Peer connected, handle %#x, interval %u units",
			tput.conn_handle, tput.conn_params.max_conn_interval);

		nrf_err = sd_ble_gatts_sys_attr_set(tput.conn_handle, NULL, 0, 0);
		if (nrf_err) {
			LOG_ERR("Failed to set system attributes, nrf_error %#x", nrf_err);
		}

		nrf_err = ble_qwr_conn_handle_assign(&ble_qwr, tput.conn_handle);
		if (nrf_err) {
			LOG_ERR("Failed to assign QWR handle, nrf_error %#x", nrf_err);
		}
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		tput.last_disconnect_reason = evt->evt.gap_evt.params.disconnected.reason;
		if (tput.conn_handle == evt->evt.gap_evt.conn_handle) {
			if (tput.test_running) {
				(void)throughput_stop_internal("disconnected");
			}
			tput.conn_handle = BLE_CONN_HANDLE_INVALID;
			conn_params_defaults_set();
			link_runtime_reset();
		}
		if (should_reboot_to_firmware_loader) {
			reset_peer_disconnected = true;
		}
		LOG_INF("Peer disconnected, reason %#x",
			evt->evt.gap_evt.params.disconnected.reason);
		break;

	case BLE_GAP_EVT_AUTH_STATUS:
		LOG_INF("Authentication status: %#x",
			evt->evt.gap_evt.params.auth_status.auth_status);
		break;

	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		nrf_err = sd_ble_gap_sec_params_reply(evt->evt.gap_evt.conn_handle,
						      BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
						      NULL, NULL);
		if (nrf_err) {
			LOG_ERR("Failed to reply with security params, nrf_error %#x", nrf_err);
		}
		break;

	case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		nrf_err = sd_ble_gatts_sys_attr_set(evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
		if (nrf_err) {
			LOG_ERR("Failed to set system attributes, nrf_error %#x", nrf_err);
		}
		break;

	case BLE_GATTS_EVT_HVN_TX_COMPLETE:
	{
		const uint16_t count = evt->evt.gatts_evt.params.hvn_tx_complete.count;

		tput.tx_complete_events += count;
		if (count >= tput.tx_in_flight) {
			tput.tx_in_flight = 0;
		} else {
			tput.tx_in_flight -= count;
		}
		tput.tx_blocked = false;
		if (should_reboot_to_firmware_loader) {
			reset_response_sent = true;
		}
		break;
	}

	default:
		break;
	}
}
NRF_SDH_BLE_OBSERVER(sdh_ble, on_ble_evt, NULL, USER_LOW);

static void on_conn_params_evt(const struct ble_conn_params_evt *evt)
{
	if (evt->conn_handle != tput.conn_handle) {
		return;
	}

	switch (evt->evt_type) {
	case BLE_CONN_PARAMS_EVT_UPDATED:
		tput.conn_params = evt->conn_params;
		LOG_INF("Connection params updated: min %u max %u latency %u timeout %u",
			evt->conn_params.min_conn_interval,
			evt->conn_params.max_conn_interval,
			evt->conn_params.slave_latency,
			evt->conn_params.conn_sup_timeout);
		data_length_max_request("conn-param update");
		break;

	case BLE_CONN_PARAMS_EVT_REJECTED:
		LOG_WRN("Connection parameter request rejected");
		MEMFAULT_TRACE_EVENT(tput_conn_param_rejected);
		break;

	case BLE_CONN_PARAMS_EVT_ATT_MTU_UPDATED:
		tput.att_mtu = evt->att_mtu;
		tput.tx_blocked = false;
		if (tput.payload_tracks_mtu) {
			tput.payload_len = current_max_payload_len();
		}
		LOG_INF("ATT MTU updated: %u, payload max %u", tput.att_mtu,
			current_max_payload_len());
		break;

	case BLE_CONN_PARAMS_EVT_DATA_LENGTH_UPDATED:
		tput.data_len_tx = evt->data_length.tx;
		tput.data_len_rx = evt->data_length.rx;
		LOG_INF("Data length updated: tx %u rx %u", tput.data_len_tx, tput.data_len_rx);
		break;

	case BLE_CONN_PARAMS_EVT_RADIO_PHY_MODE_UPDATED:
		if (evt->phy_update_evt.status == BLE_HCI_STATUS_CODE_SUCCESS) {
			tput.phy.tx_phys = evt->phy_update_evt.tx_phy;
			tput.phy.rx_phys = evt->phy_update_evt.rx_phy;
			LOG_INF("PHY updated: tx %u rx %u",
				tput.phy.tx_phys, tput.phy.rx_phys);
			data_length_max_request("PHY update");
		} else {
			LOG_WRN("PHY update completed with status %#x",
				evt->phy_update_evt.status);
		}
		break;

	case BLE_CONN_PARAMS_EVT_ERROR:
		LOG_ERR("Connection parameter procedure error, reason %#x", evt->error.reason);
		break;

	default:
		break;
	}

	metrics_update();
}

static void ble_adv_evt_handler(struct ble_adv *adv, const struct ble_adv_evt *adv_evt)
{
	ARG_UNUSED(adv);

	if (adv_evt->evt_type == BLE_ADV_EVT_ERROR) {
		LOG_ERR("Advertising error %#x", adv_evt->error.reason);
	}
}

static uint16_t ble_qwr_evt_handler(struct ble_qwr *qwr, const struct ble_qwr_evt *qwr_evt)
{
	ARG_UNUSED(qwr);

	switch (qwr_evt->evt_type) {
	case BLE_QWR_EVT_ERROR:
		LOG_ERR("QWR error event, nrf_error %#x", qwr_evt->error.reason);
		break;
	case BLE_QWR_EVT_EXECUTE_WRITE:
		LOG_INF("QWR execute write event");
		break;
	case BLE_QWR_EVT_AUTH_REQUEST:
		LOG_INF("QWR auth request event");
		break;
	}

	return BLE_GATT_STATUS_SUCCESS;
}

static void ble_nus_evt_handler(struct ble_nus *nus, const struct ble_nus_evt *evt)
{
	ARG_UNUSED(nus);

	switch (evt->evt_type) {
	case BLE_NUS_EVT_COMM_STARTED:
		tput.notifications_enabled = true;
		tput.tx_blocked = false;
		LOG_INF("NUS notifications enabled");
		if (IS_ENABLED(CONFIG_SAMPLE_BLE_MEMFAULT_THROUGHPUT_AUTOSTART)) {
			(void)throughput_start_internal(0);
		}
		break;

	case BLE_NUS_EVT_COMM_STOPPED:
		tput.notifications_enabled = false;
		if (tput.test_running) {
			(void)throughput_stop_internal("notifications-disabled");
		}
		LOG_INF("NUS notifications disabled");
		break;

	case BLE_NUS_EVT_RX_DATA:
		tput.rx_writes++;
		tput.rx_bytes += evt->rx_data.length;
		metrics_update();
		break;

	case BLE_NUS_EVT_TX_RDY:
		tput.tx_blocked = false;
		break;

	case BLE_NUS_EVT_ERROR:
		LOG_ERR("NUS error event, reason %#x", evt->error.reason);
		break;
	}
}

static int bluetooth_init(void)
{
	int err;
	uint32_t nrf_err;
	ble_gap_conn_sec_mode_t device_name_write_sec;
	struct ble_adv_config ble_adv_cfg = {
		.conn_cfg_tag = CONFIG_NRF_SDH_BLE_CONN_TAG,
		.evt_handler = ble_adv_evt_handler,
		.adv_data = {
			.name_type = BLE_ADV_DATA_FULL_NAME,
			.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,
		},
	};
	struct ble_nus_config nus_cfg = {
		.evt_handler = ble_nus_evt_handler,
		.sec_mode = BLE_NUS_CONFIG_SEC_MODE_DEFAULT,
	};
	struct ble_dis_config dis_cfg = {
		.sec_mode = BLE_DIS_CONFIG_SEC_MODE_DEFAULT,
	};
	struct ble_mcumgr_config mcumgr_cfg = {
		.sec_mode = BLE_MCUMGR_CONFIG_SEC_MODE_DEFAULT,
	};
	struct ble_mds_config mds_cfg = {
		.sec_mode = BLE_MDS_CONFIG_SEC_MODE_DEFAULT,
	};
	struct ble_qwr_config qwr_config = {
		.evt_handler = ble_qwr_evt_handler,
	};

	err = nrf_sdh_enable_request();
	if (err) {
		LOG_ERR("Failed to enable SoftDevice, err %d", err);
		return err;
	}

	LOG_INF("SoftDevice enabled");

	err = nrf_sdh_ble_enable(CONFIG_NRF_SDH_BLE_CONN_TAG);
	if (err) {
		LOG_ERR("Failed to enable BLE, err %d", err);
		return err;
	}

	LOG_INF("Bluetooth enabled");

	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&device_name_write_sec);
	nrf_err = sd_ble_gap_device_name_set(&device_name_write_sec,
					     CONFIG_SAMPLE_BLE_DEVICE_NAME,
					     strlen(CONFIG_SAMPLE_BLE_DEVICE_NAME));
	if (nrf_err) {
		LOG_ERR("Failed to set device name, nrf_error %#x", nrf_err);
		return -EIO;
	}

	nrf_err = ble_qwr_init(&ble_qwr, &qwr_config);
	if (nrf_err) {
		LOG_ERR("ble_qwr_init failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	nrf_err = ble_nus_init(&ble_nus, &nus_cfg);
	if (nrf_err) {
		LOG_ERR("ble_nus_init failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	nrf_err = ble_dis_init(&dis_cfg);
	if (nrf_err) {
		LOG_ERR("ble_dis_init failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	nrf_err = ble_mcumgr_init(&mcumgr_cfg);
	if (nrf_err) {
		LOG_ERR("ble_mcumgr_init failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	nrf_err = ble_mds_init(&ble_mds, &mds_cfg);
	if (nrf_err) {
		LOG_ERR("ble_mds_init failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	ble_uuid_t adv_uuid_list[] = {
		{ .uuid = BLE_MCUMGR_SERVICE_UUID_SUB,
		  .type = ble_mcumgr_service_uuid_type() },
	};

	ble_adv_cfg.sr_data.uuid_lists.complete.uuid = &adv_uuid_list[0];
	ble_adv_cfg.sr_data.uuid_lists.complete.len = ARRAY_SIZE(adv_uuid_list);
	LOG_INF("Services initialized: NUS, DIS, MCUmgr, MDS");

	nrf_err = ble_conn_params_evt_handler_set(on_conn_params_evt);
	if (nrf_err) {
		LOG_ERR("ble_conn_params_evt_handler_set failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	nrf_err = ble_adv_init(&ble_adv, &ble_adv_cfg);
	if (nrf_err) {
		LOG_ERR("ble_adv_init failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	nrf_err = ble_adv_start(&ble_adv, BLE_ADV_MODE_FAST);
	if (nrf_err) {
		LOG_ERR("ble_adv_start failed, nrf_error %#x", nrf_err);
		return -EIO;
	}

	LOG_INF("Advertising as %s", CONFIG_SAMPLE_BLE_DEVICE_NAME);

	return 0;
}

int main(void)
{
	const struct shell *sh = shell_backend_bm_uarte_get_ptr();
	const struct shell_backend_config_flags cfg_flags = SHELL_DEFAULT_BACKEND_CONFIG_FLAGS;
	unsigned int key;
	int err;

	LOG_INF("BLE Memfault throughput sample started");

	for (uint16_t i = 0; i < sizeof(payload); i++) {
		payload[i] = (uint8_t)i;
	}

	shell_init(sh, NULL, cfg_flags, false, 0);
	shell_start(sh);

	LOG_INF("Shell started. Use `tput status` and `mflt export`.");
	mgmt_callback_register(&os_mgmt_reboot_callback);

	err = bluetooth_init();
	if (err) {
		LOG_ERR("Bluetooth initialization failed, err %d", err);
	}

	while (atomic_get(&running)) {
		(void)bm_scheduler_process();
		memfault_metrics_timer_process();
		ble_mds_process(&ble_mds);
		throughput_pump();
		throughput_report_if_due();
		mcumgr_reboot_process();
		shell_process(sh);
		log_flush();

		if (throughput_can_pump()) {
			continue;
		}

		key = irq_lock();
		if (shell_backend_bm_uarte_rx_ready()) {
			irq_unlock(key);
			continue;
		}

		if (throughput_can_pump()) {
			irq_unlock(key);
			continue;
		}

		k_cpu_atomic_idle(key);
	}

	if (tput.test_running) {
		(void)throughput_stop_internal("quit");
	}

	shell_uninit(sh, NULL);
	LOG_INF("Shell terminated");

	while (true) {
		log_flush();
		k_cpu_idle();
	}

	return 0;
}
