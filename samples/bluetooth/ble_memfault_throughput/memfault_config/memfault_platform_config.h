/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

/* Keep the sample focused on logging, metrics, trace events, and RAM-backed
 * coredumps. The Zephyr port provides the rest of the platform defaults.
 */
#define MEMFAULT_LOG_MAX_LINE_SAVE_LEN 160

/* This sample follows the nRF Cloud powered-by-Memfault quickstart path, so
 * MDS gateways should post chunks to the Nordic-hosted Memfault endpoint.
 */
#define MEMFAULT_HTTP_CHUNKS_API_HOST "chunks-nrf.memfault.com"

/* sdk-nrf-bm's SoftDevice IRQ forwarder owns HardFault_Handler and dispatches
 * application hard faults through C_HardFault_Handler when the SoftDevice is
 * not consuming the exception.
 */
#define MEMFAULT_EXC_HANDLER_HARD_FAULT C_HardFault_Handler
