#!/usr/bin/env python3
"""
kb_bms — reads the pack's JBD/Xiaoxiang smart BMS over Bluetooth LE and
publishes sensor_msgs/BatteryState on /battery/state.

The Orin's built-in Bluetooth connects directly to the BMS (no ESP32/CAN in
the loop), so this works even when the ESP32 serial link is down — which is
exactly when the dashboard's battery gauge would otherwise read "--".

Design (matches the dashboard node's threading pattern):
  * A background thread runs bleak's asyncio loop: connect → subscribe to the
    notify characteristic → poll the pack, parse, and stash the latest reading
    under a lock. It reconnects forever on any failure (pack powered off, BLE
    drop, out of range) without ever killing the node.
  * A ROS timer on the spin thread publishes the latest reading. Publishing
    happens ONLY on the spin thread — rclpy cross-thread publishing from the
    asyncio thread silently no-ops (see .agents notes / MEMORY).

JBD protocol: write a command to char 0xFF02, read the reply as notifications
on 0xFF01. Frames are `DD <reg> <status> <len> <payload…> <chk> <chk> 77`,
big-endian.
"""

import asyncio
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

from kb_bms.soc_model import SocFilter

# JBD GATT (16-bit UUIDs expanded to the Bluetooth base UUID)
NOTIFY_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"
WRITE_UUID = "0000ff02-0000-1000-8000-00805f9b34fb"
CMD_BASIC = bytes.fromhex("dda50300fffd77")   # register 0x03 — pack summary
CMD_CELLS = bytes.fromhex("dda50400fffc77")   # register 0x04 — per-cell mV


def _u16(b, i):
    return (b[i] << 8) | b[i + 1]


def _s16(b, i):
    v = _u16(b, i)
    return v - 65536 if v > 32767 else v


def _frames(buf: bytes):
    """Yield complete JBD frames (DD … 77) from a raw notification buffer."""
    out = []
    i = 0
    while i < len(buf):
        if buf[i] == 0xDD:
            j = buf.find(b"\x77", i)
            if j == -1:
                break
            out.append(buf[i:j + 1])
            i = j + 1
        else:
            i += 1
    return out


def _parse_basic(f: bytes):
    if len(f) < 6 or f[1] != 0x03 or f[2] != 0x00:
        return None
    ln = f[3]
    d = f[4:4 + ln]
    if len(d) < 23:
        return None
    ntc = d[22]
    temps = [round((_u16(d, 23 + 2 * k) - 2731) / 10.0, 1) for k in range(ntc)
             if 23 + 2 * k + 1 < len(d)]
    return {
        "voltage": _u16(d, 0) / 100.0,
        "current": _s16(d, 2) / 100.0,
        "remain_ah": _u16(d, 4) / 100.0,
        "nominal_ah": _u16(d, 6) / 100.0,
        "cycles": _u16(d, 8),
        "protection": _u16(d, 16),
        "soc": d[19],
        "n_cells": d[21],
        "temps": temps,
    }


def _parse_cells(f: bytes):
    if len(f) < 6 or f[1] != 0x04 or f[2] != 0x00:
        return None
    ln = f[3]
    d = f[4:4 + ln]
    return [_u16(d, 2 * k) for k in range(ln // 2)]


class BmsNode(Node):
    """@brief Publishes the smart BMS pack state on /battery/state."""

    def __init__(self):
        super().__init__("kb_bms")
        self.declare_parameter("mac", "A5:C2:37:39:58:5D")
        self.declare_parameter("name", "SP22S003BP21S100A")
        self.declare_parameter("publish_period", 1.0)
        # Off until soc_model.OCV_TABLE and CELL_RESISTANCE_OHM are measured on this
        # pack rather than copied from generic NMC figures. While off, the fused
        # estimate is still computed and published, so its behaviour can be compared
        # against the raw BMS figure on a real run before anything depends on it.
        self.declare_parameter("soc_fusion", False)
        self.mac = self.get_parameter("mac").value
        self.name = self.get_parameter("name").value
        period = float(self.get_parameter("publish_period").value)
        self.soc_fusion = bool(self.get_parameter("soc_fusion").value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(BatteryState, "/battery/state", qos)
        # Fused charge estimate, 0..1, on its own topic. /battery/state.percentage
        # deliberately keeps carrying the BMS's raw figure whatever soc_fusion says,
        # so the two can always be compared — their disagreement IS the drift, and
        # it is the thing worth being able to see.
        self.soc_pub = self.create_publisher(Float32, "/battery/soc_fused", qos)
        self._soc_filter: SocFilter | None = None
        self._last_fuse_t: float | None = None

        self._lock = threading.Lock()
        self._latest = None       # dict from _parse_basic + optional "cells"
        self._latest_t = 0.0      # monotonic time of last successful read
        self._connected = False
        self._fail_count = 0      # consecutive BLE failures (drives self-heal)

        # Publish from the spin thread (cross-thread publish from asyncio no-ops).
        self.create_timer(period, self._publish)

        self._ble_thread = threading.Thread(target=self._ble_loop, daemon=True)
        self._ble_thread.start()
        self.get_logger().info(
            f"kb_bms started — BLE target {self.name} ({self.mac}), "
            f"publishing /battery/state every {period:.1f}s"
        )

    # ── BLE side (background thread) ────────────────────────────────────
    def _ble_loop(self):
        asyncio.run(self._ble_main())

    async def _ble_main(self):
        from bleak import BleakClient, BleakScanner
        while rclpy.ok():
            try:
                address = self.mac
                # Connect straight by MAC. The pack advertises a *public* BLE
                # address, so it does not rotate and this is the reliable path
                # — provided BlueZ still holds the device in its object cache.
                try:
                    client = BleakClient(address, timeout=20)
                    await client.connect()
                except Exception:
                    # Cache miss. Retry with an unfiltered discovery held open
                    # across the connect — see _connect_with_scan_active.
                    client, address = await self._connect_with_scan_active(
                        BleakClient
                    )

                try:
                    self._connected = True
                    self._fail_count = 0
                    self.get_logger().info(f"BMS connected ({address})")
                    buf = bytearray()

                    def cb(_, data):
                        buf.extend(data)

                    await client.start_notify(NOTIFY_UUID, cb)
                    while rclpy.ok() and client.is_connected:
                        # basic summary
                        buf.clear()
                        await client.write_gatt_char(WRITE_UUID, CMD_BASIC, response=False)
                        await asyncio.sleep(0.6)
                        basic = None
                        for fr in _frames(bytes(buf)):
                            basic = _parse_basic(fr) or basic
                        # per-cell voltages
                        buf.clear()
                        await client.write_gatt_char(WRITE_UUID, CMD_CELLS, response=False)
                        await asyncio.sleep(0.6)
                        cells = None
                        for fr in _frames(bytes(buf)):
                            cells = _parse_cells(fr) or cells
                        if basic:
                            if cells:
                                basic["cells"] = cells
                            with self._lock:
                                self._latest = basic
                                self._latest_t = time.monotonic()
                        await asyncio.sleep(1.0)
                finally:
                    try:
                        await client.disconnect()
                    except Exception:
                        pass
            except Exception as e:  # noqa: BLE stack raises many types
                self._connected = False
                self._fail_count += 1
                self.get_logger().warn(f"BMS BLE error ({e}); retrying in 5s")
                # Self-heal: a crashed/leaked client can leave BlueZ holding a
                # stale "Connected: yes" for the BMS. While connected it stops
                # advertising, so both connect-by-MAC and scan-by-name fail
                # forever and the node loops here until a human clears it by
                # hand. Every few consecutive failures, force BlueZ to drop and
                # forget the device so the next attempt re-discovers it fresh.
                if self._fail_count % 3 == 0:
                    await self._bluez_recover()
                await asyncio.sleep(5.0)

    async def _bluez_recover(self):
        """@brief Clear a stale BlueZ connection to the BMS (see _ble_main).

        Runs `bluetoothctl disconnect`/`remove` for the target MAC. Best-effort:
        any failure is logged and ignored — the retry loop tries again anyway.
        """
        self.get_logger().warn(
            f"BMS unreachable for {self._fail_count} tries — "
            f"clearing stale BlueZ state for {self.mac}"
        )
        # NOTE: "remove" deletes the BlueZ cache entry for the pack, and the
        # connect-by-MAC path in _ble_main depends on that entry existing. So
        # every remove MUST be followed by a repopulating unfiltered scan —
        # see the _bluez_scan_unfiltered call at the end of this method.
        for verb in ("disconnect", "remove"):
            try:
                proc = await asyncio.create_subprocess_exec(
                    "bluetoothctl", verb, self.mac,
                    stdout=asyncio.subprocess.DEVNULL,
                    stderr=asyncio.subprocess.DEVNULL,
                )
                await asyncio.wait_for(proc.wait(), timeout=10)
            except Exception as e:  # noqa: bluetoothctl absent / timeout
                self.get_logger().warn(f"bluetoothctl {verb} failed: {e}")
        # Undo the cache deletion the remove above just caused.
        await self._bluez_scan_unfiltered()

    async def _connect_with_scan_active(self, BleakClient):
        """@brief Connect to the pack with an unfiltered discovery kept running.

        Two separate BlueZ behaviours have to be worked around at once, and
        both were confirmed on this Orin on 2026-07-18:

        1. The bleak scanner cannot see the pack at all. Its BlueZ backend sets
           a discovery filter, so bluetoothd issues
           MGMT_OP_START_SERVICE_DISCOVERY rather than a plain discovery, and
           on BlueZ 5.64 that path reports nothing when it has no UUIDs to
           match. This pack advertises only flags and its name, no service
           UUIDs, so BleakScanner returns an empty list while btmgmt find and
           bluetoothctl scan on both see it at RSSI -67 in the same minute.
           Hence bluetoothctl, not BleakScanner, drives discovery here.

        2. BlueZ treats a discovered-but-unpaired device as *temporary* and
           destroys the D-Bus object once discovery stops. A scan that exits
           before the connect therefore leaves nothing to connect to, and
           bleak reports "Device with address ... was not found" against a
           cache that was populated moments earlier. Trusting the device is
           not sufficient on its own to keep the object alive.

        So the scan is started as a background process and left running for the
        whole connect attempt, then terminated in the finally block. Returns
        (client, address).
        """
        scan = None
        try:
            scan = await asyncio.create_subprocess_exec(
                "bluetoothctl", "scan", "on",
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL,
            )
            # Give discovery time to pick the pack up before connecting.
            await asyncio.sleep(8.0)

            # Trust it while the object exists. This persists across reboots
            # and lets later cold starts connect without reaching this path.
            await self._bluez_cmd("trust", self.mac)

            address = self.mac
            found = await self._lookup_by_name()
            if found:
                address = found
                if found != self.mac:
                    await self._bluez_cmd("trust", found)

            client = BleakClient(address, timeout=20)
            await client.connect()
            return client, address
        finally:
            if scan is not None:
                try:
                    scan.terminate()
                    await asyncio.wait_for(scan.wait(), timeout=5)
                except Exception:  # noqa: already dead / will not die
                    pass

    async def _lookup_by_name(self):
        """@brief Return the address bluetoothctl lists for the target name.

        Lets a replaced pack with a different address still be found without
        anyone editing the mac parameter. Returns None if it is not listed.
        """
        try:
            proc = await asyncio.create_subprocess_exec(
                "bluetoothctl", "devices",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.DEVNULL,
            )
            out, _ = await asyncio.wait_for(proc.communicate(), timeout=10)
            for line in out.decode("utf-8", "replace").splitlines():
                parts = line.split(None, 2)
                if len(parts) == 3 and parts[0] == "Device":
                    if parts[2].strip() == self.name:
                        return parts[1]
        except Exception as e:  # noqa: bluetoothctl absent / timeout
            self.get_logger().warn(f"device lookup failed: {e}")
        return None

    async def _bluez_cmd(self, *args):
        """@brief Run a bluetoothctl subcommand, best-effort.

        Failures are logged and swallowed: every caller sits inside the retry
        loop, so a missing bluetoothctl or a timeout must not take the node
        down — the next pass tries again.
        """
        try:
            proc = await asyncio.create_subprocess_exec(
                "bluetoothctl", *args,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL,
            )
            await asyncio.wait_for(proc.wait(), timeout=10)
        except Exception as e:  # noqa: bluetoothctl absent / timeout
            self.get_logger().warn(f"bluetoothctl {' '.join(args)} failed: {e}")

    async def _bluez_scan_unfiltered(self):
        """@brief Repopulate the BlueZ device cache; return the pack address.

        Why this exists instead of BleakScanner: the bleak BlueZ backend sets a
        discovery filter, which makes bluetoothd issue
        MGMT_OP_START_SERVICE_DISCOVERY rather than a plain discovery. On BlueZ
        5.64 that filtered path reports *no* devices when it has no UUIDs to
        match against, and this pack advertises only flags plus its name — no
        service UUIDs. So BleakScanner returns an empty list while the pack is
        advertising perfectly well. Confirmed on 2026-07-18: btmgmt find and
        bluetoothctl scan on both saw it at RSSI -67 during the same minute
        that BleakScanner reported zero devices, with and without filters.

        bluetoothctl scan on sets no filter and does populate the cache, after
        which a plain connect-by-MAC succeeds. Returns the address matching the
        target name if one turns up, else None.
        """
        try:
            proc = await asyncio.create_subprocess_exec(
                "bluetoothctl", "--timeout", "12", "scan", "on",
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL,
            )
            await asyncio.wait_for(proc.wait(), timeout=25)
        except Exception as e:  # noqa: bluetoothctl absent / timeout
            self.get_logger().warn(f"unfiltered scan failed: {e}")
            return None

        # Mark the pack trusted. BlueZ garbage-collects devices that are
        # neither paired nor trusted once discovery stops, which used to make
        # the repopulation above a race: the scan added the device, then BlueZ
        # dropped it again before the connect landed, and the node reported
        # "Device with address ... was not found" on a cache it had just
        # filled. Trusting makes the entry persist, including across reboots,
        # so after the first success the scan is never needed again and a plain
        # connect-by-MAC works from cold. Idempotent, so it is safe every time.
        await self._bluez_cmd("trust", self.mac)

        # Resolve by name as well, so a replaced pack with a different address
        # is still found without anyone editing the mac parameter.
        try:
            proc = await asyncio.create_subprocess_exec(
                "bluetoothctl", "devices",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.DEVNULL,
            )
            out, _ = await asyncio.wait_for(proc.communicate(), timeout=10)
            for line in out.decode("utf-8", "replace").splitlines():
                parts = line.split(None, 2)
                if len(parts) == 3 and parts[0] == "Device":
                    if parts[2].strip() == self.name:
                        if parts[1] != self.mac:
                            await self._bluez_cmd("trust", parts[1])
                        return parts[1]
        except Exception as e:  # noqa: bluetoothctl absent / timeout
            self.get_logger().warn(f"device lookup failed: {e}")
        return None

    # ── ROS side (spin thread) ──────────────────────────────────────────
    def _fuse_soc(self, r, bms_soc):
        """@brief Run one step of the voltage-corrected charge estimate.

        Returns the fused fractional charge, or None if the pack has not reported a
        usable capacity yet (without one there is nothing to convert the BMS's
        remaining-Ah readings into a fraction with). Always publishes whatever it
        computes on /battery/soc_fused, whether or not soc_fusion is on, so the
        estimate can be watched against the raw figure before being relied on.

        The rationale for fusing at all, and for the specific split of duties
        between the coulomb count and the voltage, is in kb_bms/soc_model.py.
        """
        capacity_ah = float(r.get("nominal_ah") or 0.0)
        if capacity_ah <= 0.0:
            return None
        if self._soc_filter is None:
            self._soc_filter = SocFilter(capacity_ah)

        now = time.monotonic()
        dt = (now - self._last_fuse_t) if self._last_fuse_t is not None else 0.0
        self._last_fuse_t = now

        was_cold = self._soc_filter.soc is None
        fused = self._soc_filter.update(
            pack_voltage=float(r["voltage"]),
            current=float(r["current"]),
            remain_ah=float(r["remain_ah"]),
            bms_soc=bms_soc,
            dt=dt,
        )
        if was_cold:
            source = (
                "resting pack voltage" if self._soc_filter.seeded_from_voltage
                else "the BMS figure (pack was under load, so its voltage was unusable)"
            )
            self.get_logger().info(
                f"SOC estimate seeded from {source}: "
                f"{fused * 100:.0f}% (BMS says {bms_soc * 100:.0f}%)"
            )
        self.soc_pub.publish(Float32(data=float(fused)))
        return fused

    def _publish(self):
        with self._lock:
            r = self._latest
            age = time.monotonic() - self._latest_t if r else None
        if r is None:
            return
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(r["voltage"])
        msg.current = float(r["current"])
        msg.charge = float(r["remain_ah"])
        msg.capacity = float(r["remain_ah"])
        msg.design_capacity = float(r["nominal_ah"])
        bms_soc = float(r["soc"]) / 100.0
        fused_soc = self._fuse_soc(r, bms_soc)
        msg.percentage = (
            fused_soc if (self.soc_fusion and fused_soc is not None) else bms_soc
        )
        msg.temperature = float(r["temps"][0]) if r.get("temps") else float("nan")
        msg.present = age is not None and age < 10.0
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING if r["current"] > 0.2
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if r["current"] < -0.2
            else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        )
        msg.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_GOOD if r["protection"] == 0
            else BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        )
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        if r.get("cells"):
            msg.cell_voltage = [c / 1000.0 for c in r["cells"]]  # mV → V
        if r.get("temps"):
            msg.cell_temperature = [float(t) for t in r["temps"]]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BmsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
