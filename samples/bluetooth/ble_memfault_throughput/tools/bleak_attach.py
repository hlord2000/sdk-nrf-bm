#!/usr/bin/env python3
#
# Copyright (c) 2026 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

"""Attach to the BLE Memfault throughput sample with normal Bleak discovery."""

import argparse
import asyncio
import contextlib
import time

from bleak import BleakClient, BleakScanner


NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Scan for nRF_BM_TPUT, connect with Bleak, subscribe to NUS TX.",
    )
    parser.add_argument("--name", default="nRF_BM_TPUT", help="advertised device name")
    parser.add_argument("--address", help="skip name scan and connect to this BLE address")
    parser.add_argument("--scan-timeout", type=float, default=3.0, help="seconds per scan pass")
    parser.add_argument("--connect-timeout", type=float, default=25.0, help="Bleak connect timeout")
    parser.add_argument("--seconds", type=float, default=0.0, help="0 means run until Ctrl-C")
    parser.add_argument("--write", default="hello from bleak", help="initial NUS RX text")
    parser.add_argument("--no-write", action="store_true", help="do not write to NUS RX")
    parser.add_argument(
        "--write-response",
        action="store_true",
        help="use write-with-response for the initial NUS RX write",
    )
    parser.add_argument("--preview-bytes", type=int, default=16, help="bytes to print per preview")
    parser.add_argument("--show-first", type=int, default=5, help="preview this many notifications")
    parser.add_argument("--stats-every", type=float, default=1.0, help="seconds between stats lines")
    return parser.parse_args()


async def scan_for_device(name: str, scan_timeout: float):
    while True:
        devices = await BleakScanner.discover(timeout=scan_timeout, return_adv=True)
        for device, adv in devices.values():
            service_uuids = [uuid.lower() for uuid in (adv.service_uuids or [])]
            if device.name == name or NUS_SERVICE_UUID in service_uuids:
                print(f"found {device.address} name={device.name} rssi={adv.rssi}")
                return device

        print(f"still scanning for {name} / {NUS_SERVICE_UUID} ...")


async def wait_forever_or_seconds(seconds: float):
    if seconds > 0:
        await asyncio.sleep(seconds)
        return

    await asyncio.Event().wait()


async def main() -> None:
    args = parse_args()
    target = args.address if args.address else await scan_for_device(args.name, args.scan_timeout)

    notify_count = 0
    notify_bytes = 0
    first_notify_time = None
    last_notify_time = None

    def on_notify(_characteristic, data: bytearray) -> None:
        nonlocal notify_count, notify_bytes, first_notify_time, last_notify_time

        now = time.monotonic()
        notify_count += 1
        notify_bytes += len(data)
        first_notify_time = first_notify_time or now
        last_notify_time = now

        if notify_count <= args.show_first:
            preview = bytes(data[: args.preview_bytes]).hex()
            print(f"notify #{notify_count}: len={len(data)} preview={preview}")

    async def print_stats() -> None:
        last_count = 0
        last_bytes = 0

        while True:
            await asyncio.sleep(args.stats_every)
            delta_count = notify_count - last_count
            delta_bytes = notify_bytes - last_bytes
            last_count = notify_count
            last_bytes = notify_bytes

            if first_notify_time is not None and last_notify_time is not None:
                elapsed = max(last_notify_time - first_notify_time, 1e-9)
                avg_bps = int((notify_bytes * 8) / elapsed)
            else:
                avg_bps = 0

            print(
                "stats: "
                f"notifications={notify_count} bytes={notify_bytes} "
                f"delta_notifications={delta_count} delta_bytes={delta_bytes} "
                f"avg_bps={avg_bps}"
            )

    stats_task = None
    client = BleakClient(target, timeout=args.connect_timeout)

    try:
        await client.connect()
        print(f"connected={client.is_connected}")
        print("services:", ", ".join(str(service.uuid) for service in client.services))

        await client.start_notify(NUS_TX_UUID, on_notify)
        print(f"subscribed: {NUS_TX_UUID}")

        if not args.no_write:
            payload = args.write.encode("utf-8")
            await client.write_gatt_char(NUS_RX_UUID, payload, response=args.write_response)
            print(f"wrote {len(payload)} byte(s) to {NUS_RX_UUID}")

        print("ready; use the board shell on if00, for example: tput status")
        stats_task = asyncio.create_task(print_stats())
        await wait_forever_or_seconds(args.seconds)

    finally:
        if stats_task is not None:
            stats_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await stats_task

        with contextlib.suppress(Exception):
            if client.is_connected:
                await client.stop_notify(NUS_TX_UUID)

        with contextlib.suppress(Exception):
            if client.is_connected:
                await client.disconnect()

        print(
            "done: "
            f"notifications={notify_count} bytes={notify_bytes} "
            f"connected={client.is_connected}"
        )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("interrupted")
