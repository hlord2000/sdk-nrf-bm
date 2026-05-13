/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *
 * @defgroup ble_mds Bluetooth LE Memfault Diagnostic Service library
 * @{
 * @brief Library for exposing Memfault Diagnostic Service over Bluetooth LE.
 */

#ifndef BLE_MDS_H__
#define BLE_MDS_H__

#include <stdbool.h>
#include <stdint.h>

#include <ble.h>
#include <bm/bluetooth/ble_common.h>
#include <bm/softdevice_handler/nrf_sdh_ble.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Vendor specific UUID base for Memfault Diagnostic Service. */
#define BLE_MDS_UUID_BASE                                                                         \
	{                                                                                         \
		0x36, 0x84, 0xBD, 0x4E, 0x2F, 0x72, 0x71, 0xA3, 0x07, 0x40, 0xA5, 0xF6,           \
			0x00, 0x00, 0x22, 0x54                                                   \
	}

#define BLE_UUID_MDS_SERVICE 0x0000
#define BLE_UUID_MDS_SUPPORTED_FEATURES_CHAR 0x0001
#define BLE_UUID_MDS_DEVICE_IDENTIFIER_CHAR 0x0002
#define BLE_UUID_MDS_DATA_URI_CHAR 0x0003
#define BLE_UUID_MDS_AUTHORIZATION_CHAR 0x0004
#define BLE_UUID_MDS_DATA_EXPORT_CHAR 0x0005

void ble_mds_on_ble_evt(const ble_evt_t *ble_evt, void *context);

/**
 * @brief Macro for defining a ble_mds instance.
 *
 * @param _name Name of the instance.
 * @hideinitializer
 */
#define BLE_MDS_DEF(_name)                                                                        \
	static struct ble_mds _name;                                                              \
	NRF_SDH_BLE_OBSERVER(_name##_obs, ble_mds_on_ble_evt, &_name, HIGH)

/** @brief Default security configuration. */
#define BLE_MDS_CONFIG_SEC_MODE_DEFAULT                                                            \
	{                                                                                          \
		.feature_char = {                                                                  \
			.read = BLE_GAP_CONN_SEC_MODE_OPEN,                                        \
		},                                                                                 \
		.device_id_char = {                                                                \
			.read = BLE_GAP_CONN_SEC_MODE_OPEN,                                        \
		},                                                                                 \
		.data_uri_char = {                                                                 \
			.read = BLE_GAP_CONN_SEC_MODE_OPEN,                                        \
		},                                                                                 \
		.auth_char = {                                                                     \
			.read = BLE_GAP_CONN_SEC_MODE_OPEN,                                        \
		},                                                                                 \
		.data_export_char = {                                                              \
			.write = BLE_GAP_CONN_SEC_MODE_OPEN,                                      \
			.cccd_write = BLE_GAP_CONN_SEC_MODE_OPEN,                                  \
		},                                                                                 \
	}

struct ble_mds_config {
	struct {
		struct {
			ble_gap_conn_sec_mode_t read;
		} feature_char;
		struct {
			ble_gap_conn_sec_mode_t read;
		} device_id_char;
		struct {
			ble_gap_conn_sec_mode_t read;
		} data_uri_char;
		struct {
			ble_gap_conn_sec_mode_t read;
		} auth_char;
		struct {
			ble_gap_conn_sec_mode_t write;
			ble_gap_conn_sec_mode_t cccd_write;
		} data_export_char;
	} sec_mode;
};

struct ble_mds {
	uint8_t uuid_type;
	uint16_t service_handle;
	ble_gatts_char_handles_t supported_features_handles;
	ble_gatts_char_handles_t device_id_handles;
	ble_gatts_char_handles_t data_uri_handles;
	ble_gatts_char_handles_t auth_handles;
	ble_gatts_char_handles_t data_export_handles;
	uint16_t conn_handle;
	bool initialized;
	bool subscriber_active;
	bool streaming_enabled;
	bool tx_blocked;
	bool hvx_pending;
	uint8_t seq_num;
	int64_t next_empty_poll_ms;
	int64_t next_log_collection_ms;
	uint8_t pending_payload[CONFIG_NRF_SDH_BLE_GATT_MAX_MTU_SIZE - ATT_OPCODE_LEN -
				ATT_HANDLE_LEN];
	uint16_t pending_len;
};

/**
 * @brief Initialize the Memfault Diagnostic Service.
 *
 * @retval NRF_SUCCESS On success.
 * @retval NRF_ERROR_NULL If @p mds or @p cfg is NULL.
 */
uint32_t ble_mds_init(struct ble_mds *mds, const struct ble_mds_config *cfg);

/**
 * @brief Pump pending Memfault chunks to an active MDS subscriber.
 *
 * Call from the application main loop.
 */
void ble_mds_process(struct ble_mds *mds);

/**
 * @brief Get the SoftDevice UUID type assigned to the MDS UUID base.
 */
uint8_t ble_mds_service_uuid_type(const struct ble_mds *mds);

#ifdef __cplusplus
}
#endif

#endif /* BLE_MDS_H__ */

/** @} */
