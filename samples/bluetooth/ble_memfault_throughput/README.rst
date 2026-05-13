.. _ble_memfault_throughput_sample:

Bluetooth LE Memfault throughput
################################

Overview
********

This sample runs on the nRF54L15 DK as a Bluetooth LE peripheral. It exposes the
Nordic UART Service and uses the TX notification characteristic as a
peripheral-to-central throughput stream. A central can subscribe to notifications
and the board shell can then start a run that sends ATT MTU-sized payloads.

On connection, the sample requests the shortest 7.5 ms connection interval, 2M
PHY, 251-octet GAP data length, and ATT MTU 247. The negotiated values are
logged and can be inspected or changed with the ``tput`` shell. Notification
transmission uses explicit SoftDevice HVN completion credits so connection
recycling tests do not depend on host-side GATT cache behavior.

Memfault is enabled with Zephyr log capture, metrics, trace events, RAM-backed
coredumps, and the built-in ``mflt`` shell commands. This sample does not enable
an IP transport; use ``mflt export`` on the UART shell to retrieve Memfault
chunks for upload or inspection.

Building and running
********************

Build for the nRF54L15 DK with S115:

.. code-block:: console

   west build -b bm_nrf54l15dk/nrf54l15/cpuapp/s115_softdevice samples/bluetooth/ble_memfault_throughput
   west flash

Connect a Bluetooth LE central, enable notifications on the NUS TX
characteristic, and use the UART shell:

.. code-block:: console

   tput status
   tput mtu max
   tput dlen max
   tput phy 2m
   tput conn min
   tput window 32
   tput payload max
   tput start 30
   tput stop
   mflt export

Bleak host helper
*****************

Create and activate a local Python environment from the repository root:

.. code-block:: console

   uv venv .venv-bleak
   source .venv-bleak/bin/activate
   uv pip install -r samples/bluetooth/ble_memfault_throughput/tools/requirements-bleak.txt

Attach with normal Bleak scanning and service discovery:

.. code-block:: console

   python samples/bluetooth/ble_memfault_throughput/tools/bleak_attach.py

The helper subscribes to the NUS TX notification characteristic, writes one
small NUS RX message by default, prints notification statistics, and keeps the
connection open until Ctrl-C. Use the board shell on ``if00`` while the helper
is running. Reset and connection logs are printed on ``if02``.

Shell commands
**************

``tput status``
   Show connection state, negotiated ATT MTU, payload length, data length, PHY,
   connection interval, and current counters.

``tput start [seconds] [payload|max]``
   Start sending notifications. ``seconds`` set to ``0`` runs until
   ``tput stop``. Payload defaults to the current configured value.

``tput stop``
   Stop the active run and print the summary.

``tput payload <len|max>``
   Set the notification payload length. ``max`` tracks the negotiated
   ``ATT_MTU - 3`` value.

``tput window <count>``
   Set the maximum outstanding notification count before the sample waits for
   ``BLE_GATTS_EVT_HVN_TX_COMPLETE``. The default is ``32`` so the sample
   aggressively fills the SoftDevice HVN queue and relies on
   ``NRF_ERROR_RESOURCES`` for stack-side backpressure.

``tput mtu <bytes|max>``
   Request an ATT MTU exchange. The sample is configured for the iOS-oriented
   maximum MTU 247, so the maximum notification payload is 244 bytes when the
   peer agrees.

``tput dlen <tx> <rx>`` or ``tput dlen max``
   Request GAP data length update values. ``max`` requests 251 octets both ways.

``tput phy <auto|1m|2m|coded>``
   Request the link PHY.

``tput conn <min_units> <max_units> [latency] [timeout_units]``
   Request connection parameters. Intervals use 1.25 ms units and supervision
   timeout uses 10 ms units. If the timeout is omitted, the sample picks a
   legal default for the requested interval.

``tput conn_ms <min_ms> <max_ms> [latency] [timeout_ms]``
   Request connection parameters in integer milliseconds. This command is useful
   for sweeps such as ``tput conn_ms 400 400`` through
   ``tput conn_ms 4000 4000``. Integer-ms values must map exactly to BLE's 1.25
   ms interval units; use ``tput conn min`` for the 7.5 ms preset.

``tput conn min`` and ``tput conn max``
   Request 7.5 ms and 4 s interval presets respectively.

eData-style timed checks
************************

Assuming each eData block is 512 bytes, 120 blocks per day is 61440 bytes and
eight days is 491520 bytes. A 1753-block maximum transfer is 897536 bytes.

For an eight-day-at-400-ms interval check:

.. code-block:: console

   tput mtu max
   tput dlen max
   tput phy 2m
   tput conn_ms 400 400
   tput window 32
   tput payload max
   tput reset
   tput start 80 max

The final ``tx`` byte count should be at least ``491520`` bytes. For the
1753-block maximum case, run ``tput start 150 max`` and check for at least
``897536`` bytes.

``tput adv <start|stop>``
   Start or stop connectable advertising.

``tput disconnect``
   Disconnect the active peer. This is useful for connection recycling tests;
   advertising restarts automatically after the SoftDevice reports the
   disconnection.

The sample logs one throughput line per second by default and updates custom
Memfault heartbeat metrics with the latest run counters.
