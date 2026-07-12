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
        self.mac = self.get_parameter("mac").value
        self.name = self.get_parameter("name").value
        period = float(self.get_parameter("publish_period").value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(BatteryState, "/battery/state", qos)

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
                # If a plain MAC connect fails, fall back to scanning by name —
                # BLE addresses can rotate, but the JBD model-name is stable.
                try:
                    client = BleakClient(address, timeout=20)
                    await client.connect()
                except Exception:
                    dev = await BleakScanner.find_device_by_name(self.name, timeout=15)
                    if dev is None:
                        raise RuntimeError("BMS not found by MAC or name")
                    address = dev.address
                    client = BleakClient(dev, timeout=20)
                    await client.connect()

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

    # ── ROS side (spin thread) ──────────────────────────────────────────
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
        msg.percentage = float(r["soc"]) / 100.0
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
