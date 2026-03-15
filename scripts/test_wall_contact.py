#!/usr/bin/env python3
"""
Wall contact test: fly hexrotor forward until rod touches wall,
hold contact, and display real-time force/torque sensor data.

Usage:
    1. Start simulation:  make px4_sitl gz_hexa_scorpion
    2. In another terminal: python3 scripts/test_wall_contact.py
"""

import asyncio
import subprocess
import threading
import math
import sys
import shutil

# ── Configuration ──────────────────────────────────────────────
TAKEOFF_ALT = 1.5            # meters
APPROACH_SPEED = 0.5         # m/s forward during approach
CONTACT_PUSH_SPEED = 0.05   # m/s gentle push after contact
CONTACT_HOLD_TIME = 15.0    # seconds to hold contact
FORCE_THRESHOLD = 1.0       # N, minimum force to count as contact
SAFETY_TIMEOUT = 60.0       # seconds before aborting
PX4_CONNECTION = "udp://:14540"
# ───────────────────────────────────────────────────────────────


def discover_ft_topic():
    """Auto-discover the force/torque sensor gz transport topic."""
    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True, text=True, timeout=5
        )
        for line in result.stdout.splitlines():
            if "forcetorque" in line.lower():
                return line.strip()
    except Exception:
        pass
    return None


class ForceSensorMonitor:
    """Stream force/torque data from a gz transport topic."""

    def __init__(self, topic):
        self.topic = topic
        self.force = [0.0, 0.0, 0.0]
        self.torque = [0.0, 0.0, 0.0]
        self._proc = None
        self._running = False

    def start(self):
        self._running = True
        self._proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", self.topic],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
        )
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        section = None
        idx_map = {"x": 0, "y": 1, "z": 2}
        for line in iter(self._proc.stdout.readline, ""):
            if not self._running:
                break
            s = line.strip()
            if s.startswith("force"):
                section = "f"
            elif s.startswith("torque"):
                section = "t"
            elif s == "}":
                section = None
            elif section and ":" in s:
                key, _, val = s.partition(":")
                key = key.strip()
                if key in idx_map:
                    try:
                        v = float(val.strip())
                        if section == "f":
                            self.force[idx_map[key]] = v
                        else:
                            self.torque[idx_map[key]] = v
                    except ValueError:
                        pass

    @property
    def force_magnitude(self):
        return math.sqrt(sum(c * c for c in self.force))

    @property
    def contact(self):
        return self.force_magnitude > FORCE_THRESHOLD

    def stop(self):
        self._running = False
        if self._proc:
            self._proc.terminate()

    def fmt(self):
        f, t = self.force, self.torque
        mag = self.force_magnitude
        return (
            f"F=({f[0]:+8.3f}, {f[1]:+8.3f}, {f[2]:+8.3f}) N  "
            f"|F|={mag:6.3f} N  "
            f"T=({t[0]:+8.3f}, {t[1]:+8.3f}, {t[2]:+8.3f}) Nm"
        )


def print_bar(force_mag, max_force=30.0):
    """Print a simple bar visualization of force magnitude."""
    cols = shutil.get_terminal_size().columns - 2
    bar_width = min(cols - 50, 40)
    filled = int(min(force_mag / max_force, 1.0) * bar_width)
    bar = "█" * filled + "░" * (bar_width - filled)
    return f"[{bar}]"


async def run():
    from mavsdk import System
    from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

    # ── Discover FT topic ──────────────────────────────────────
    print("🔍 Search FT Sensor Topic...")
    ft_topic = discover_ft_topic()
    if ft_topic:
        print(f"   Found: {ft_topic}")
    else:
        print("   ⚠ No topic found (Simulation may not be started)")
        print("   Wait for starting simulation...\n")

    # ── Connect to PX4 ────────────────────────────────────────
    drone = System()
    print(f"📡 Connect PX4 SITL ({PX4_CONNECTION})...")
    await drone.connect(system_address=PX4_CONNECTION)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("   Connected!\n")
            break

    # Retry FT topic discovery after connection
    if not ft_topic:
        await asyncio.sleep(2)
        ft_topic = discover_ft_topic()
        if ft_topic:
            print(f"🔍 Find FT sensor topic: {ft_topic}\n")
        else:
            print("⚠ Can not find FT topic\n")

    # ── Wait for GPS lock ──────────────────────────────────────
    print("🛰  Wait for GPS ...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("   Localization done!\n")
            break

    # ── Start FT monitor ───────────────────────────────────────
    ft = None
    if ft_topic:
        ft = ForceSensorMonitor(ft_topic)
        ft.start()
        await asyncio.sleep(0.5)

    # ── Arm & Takeoff ──────────────────────────────────────────
    print(f"🚁 Arm and takeoff to {TAKEOFF_ALT} m ...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT)
    await drone.action.arm()
    await drone.action.takeoff()

    print("   Wait for takeoff done...")
    await asyncio.sleep(8)
    print("   Flight stablizing...")
    await asyncio.sleep(3)

    # ── Switch to Offboard ─────────────────────────────────────
    print("\n⚡ Switch to Offboard mode...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"   Offboard launch failed: {e}")
        await drone.action.land()
        if ft:
            ft.stop()
        return

    print(f"   Under {APPROACH_SPEED} m/s approach the wall...\n")
    print("=" * 80)

    # ── Approach & Contact Loop ────────────────────────────────
    loop = asyncio.get_event_loop()
    t0 = loop.time()
    t_contact = None

    try:
        while True:
            now = loop.time()
            elapsed = now - t0

            force_mag = ft.force_magnitude if ft else 0.0
            in_contact = ft.contact if ft else False

            if not in_contact:
                # Approaching
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(APPROACH_SPEED, 0.0, 0.0, 0.0)
                )
                t_contact = None
                phase = f"➡ Approaching  ({elapsed:.1f}s)"
            else:
                if t_contact is None:
                    t_contact = now
                    print("\n\n🟢 Contact detected!\n")

                hold_elapsed = now - t_contact

                # Gentle forward push to maintain contact
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(CONTACT_PUSH_SPEED, 0.0, 0.0, 0.0)
                )
                phase = f"🔴 Keep contact {hold_elapsed:.1f}/{CONTACT_HOLD_TIME}s"

                if hold_elapsed >= CONTACT_HOLD_TIME:
                    print("\n\n✅ Contact hold done!")
                    break

            # Safety timeout
            if elapsed > SAFETY_TIMEOUT:
                print(f"\n\n⏰ Safety timeout ({SAFETY_TIMEOUT}s)")
                break

            # Display
            if ft:
                bar = print_bar(force_mag)
                print(f"\r  {phase}  {bar}  {ft.fmt()}", end="", flush=True)
            else:
                print(f"\r  {phase}  (no FT data)", end="", flush=True)

            await asyncio.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n⛔ user interrupt")

    print("\n" + "=" * 80)

    # ── Return & Land ──────────────────────────────────────────
    print("\n🔙 Go back and land...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(-0.5, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(4)

    print("🛬 Landing...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(2)

    try:
        await drone.offboard.stop()
    except OffboardError:
        pass

    await drone.action.land()
    await asyncio.sleep(8)

    try:
        await drone.action.disarm()
    except Exception:
        pass

    if ft:
        ft.stop()

    print("\n✅ Test done!\n")


if __name__ == "__main__":
    asyncio.run(run())
