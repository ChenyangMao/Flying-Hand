"""
Read ATI Digital F/T streaming strain-gage samples from a serial port (default COM4).

Based on ATI document 9620-05-Digital FT, section 8 (Programming Information):
- Modbus RTU, slave address 10, even parity, 8 data bits
- Default baud 1,250,000 (also 115200 / 19200 possible)
- Custom FC 70 (0x46) + data 0x55 starts streaming; sensor sends raw 13-byte samples
- Stop streaming: send >= 14 arbitrary bytes (jamming sequence)
- Optional: write GaugeGains / GaugeOffsets (manual 8.6) via FC 106 unlock/lock + FC 16
  to holding registers 0x0000–0x000B before streaming.

Requires: pip install pyserial
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("Install pyserial: pip install pyserial", file=sys.stderr)
    raise SystemExit(1)

SLAVE_ADDR = 0x0A
FC_START_STREAMING = 70  # 0x46
FC_UNLOCK_STORAGE = 106  # 0x6A — manual table 8.3.1
FC_WRITE_MULTIPLE = 0x10
REG_ACTIVE_GAIN_BASE = 0x0000  # 6 gains then 6 offsets (0x0006–0x000B) — manual table 8.4
SAMPLE_SIZE = 13
JAM_LEN = 14
UNLOCK_STORAGE_BYTE = 0xAA
LOCK_STORAGE_BYTE = 0x18


def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_start_streaming_frame() -> bytes:
    pdu = bytes([SLAVE_ADDR, FC_START_STREAMING, 0x55])
    c = modbus_crc16(pdu)
    return pdu + bytes([c & 0xFF, (c >> 8) & 0xFF])


def _frame_crc_ok(frame5: bytes) -> bool:
    if len(frame5) != 5:
        return False
    c = modbus_crc16(frame5[:3])
    return frame5[3] == (c & 0xFF) and frame5[4] == ((c >> 8) & 0xFF)


def _append_crc(pdu: bytes) -> bytes:
    c = modbus_crc16(pdu)
    return pdu + bytes([c & 0xFF, (c >> 8) & 0xFF])


def wait_custom_fc_ack(ser: serial.Serial, func_code: int) -> bool:
    """Wait for [slave][func][0x01][crc][crc] (manual custom Modbus responses)."""
    deadline = time.monotonic() + 0.4
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf.extend(chunk)
        for i in range(0, len(buf) - 4):
            cand = bytes(buf[i : i + 5])
            if (
                _frame_crc_ok(cand)
                and cand[0] == SLAVE_ADDR
                and cand[1] == func_code
                and cand[2] == 0x01
            ):
                return True
        if len(buf) > 128:
            del buf[:64]
        if not chunk:
            time.sleep(0.002)
    return False


def build_unlock_storage_frame() -> bytes:
    return _append_crc(bytes([SLAVE_ADDR, FC_UNLOCK_STORAGE, UNLOCK_STORAGE_BYTE]))


def build_lock_storage_frame() -> bytes:
    return _append_crc(bytes([SLAVE_ADDR, FC_UNLOCK_STORAGE, LOCK_STORAGE_BYTE]))


def build_write_multiple_registers(start_addr: int, values: list[int]) -> bytes:
    n = len(values)
    if not (1 <= n <= 123):
        raise ValueError("invalid register count")
    body = bytearray(
        [
            SLAVE_ADDR,
            FC_WRITE_MULTIPLE,
            (start_addr >> 8) & 0xFF,
            start_addr & 0xFF,
            (n >> 8) & 0xFF,
            n & 0xFF,
            2 * n,
        ]
    )
    for v in values:
        if v < 0 or v > 0xFFFF:
            raise ValueError(f"register value out of uint16 range: {v}")
        body.append((v >> 8) & 0xFF)
        body.append(v & 0xFF)
    return _append_crc(bytes(body))


def wait_write_multiple_ack(ser: serial.Serial) -> bool:
    """FC 16 normal response: 8 bytes, same start_addr and qty echo."""
    deadline = time.monotonic() + 0.4
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf.extend(chunk)
        for i in range(0, len(buf) - 7):
            cand = bytes(buf[i : i + 8])
            if cand[0] != SLAVE_ADDR:
                continue
            if cand[1] == (FC_WRITE_MULTIPLE | 0x80):
                print(f"Modbus exception on write multiple: code=0x{cand[2]:02x}", file=sys.stderr)
                return False
            if cand[1] != FC_WRITE_MULTIPLE:
                continue
            c = modbus_crc16(cand[:6])
            if cand[6] == (c & 0xFF) and cand[7] == ((c >> 8) & 0xFF):
                return True
        if len(buf) > 160:
            del buf[:80]
        if not chunk:
            time.sleep(0.002)
    return False


def apply_gauge_gains_offsets(ser: serial.Serial, gains: list[int], offsets: list[int]) -> bool:
    """
    Manual 8.6 steps 4–6: unlock, write 6 gains @ 0x0000 and 6 offsets @ 0x0006, lock.
    Sensor must not be streaming Modbus must be idle.
    """
    if len(gains) != 6 or len(offsets) != 6:
        print("gains and offsets must each have 6 entries", file=sys.stderr)
        return False

    ser.reset_input_buffer()
    ser.write(build_unlock_storage_frame())
    ser.flush()
    if not wait_custom_fc_ack(ser, FC_UNLOCK_STORAGE):
        print("Unlock storage (FC 106 / 0xaa) failed or timed out.", file=sys.stderr)
        return False

    time.sleep(0.02)
    ser.reset_input_buffer()
    regs = list(gains) + list(offsets)
    tx_wm = build_write_multiple_registers(REG_ACTIVE_GAIN_BASE, regs)
    ser.write(tx_wm)
    ser.flush()
    if not wait_write_multiple_ack(ser):
        print("Write gains/offsets (FC 16, 12 registers @ 0x0000) failed or timed out.", file=sys.stderr)
        return False

    time.sleep(0.02)
    ser.reset_input_buffer()
    ser.write(build_lock_storage_frame())
    ser.flush()
    if not wait_custom_fc_ack(ser, FC_UNLOCK_STORAGE):
        print("Lock storage (FC 106 / 0x18) failed or timed out.", file=sys.stderr)
        return False

    time.sleep(0.02)
    return True


def read_start_streaming_ack(ser: serial.Serial) -> tuple[bool, bytes]:
    """
    Read Modbus RTU response to FC 70: [10][70][1][crc_lo][crc_hi].
    If the interface echoes the TX frame, additional bytes follow (manual 3.3.1).
    Returns (ok, tail_bytes): tail_bytes are any serial payload already read that
    follow the ACK (often the first streaming samples) — must be prepended to the
    streaming buffer or framing breaks and checksums fail.
    """
    deadline = time.monotonic() + 0.4
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf.extend(chunk)
        for i in range(0, len(buf) - 4):
            cand = bytes(buf[i : i + 5])
            if (
                _frame_crc_ok(cand)
                and cand[0] == SLAVE_ADDR
                and cand[1] == FC_START_STREAMING
                and cand[2] == 0x01
            ):
                return True, bytes(buf[i + 5 :])
        if len(buf) > 128:
            del buf[:64]
        if not chunk:
            time.sleep(0.002)
    return False, b""


def verify_sample_checksum(sample: bytes) -> bool:
    if len(sample) != SAMPLE_SIZE:
        return False
    s = sum(sample[i] for i in range(12)) & 0x7F
    return s == (sample[12] & 0x7F)


def resync_drop_until_valid(buf: bytearray) -> None:
    """Drop leading bytes until buf starts with a 13-byte sample that passes checksum (max 12 drops)."""
    for _ in range(SAMPLE_SIZE):
        if len(buf) < SAMPLE_SIZE:
            return
        if verify_sample_checksum(bytes(buf[:SAMPLE_SIZE])):
            return
        del buf[0]


def gages_to_ft(
    gages_g0_g5: list[int],
    basic_matrix: list[list[float]],
    counts_per_force: int,
    counts_per_torque: int,
    bias: list[float] | None,
) -> tuple[list[float], list[float]]:
    """
    Manual 8.6 step 10: result = BasicMatrix @ (G - bias); then scale forces/torques
    by CountsPerForce / CountsPerTorque (matrix values are in 'counts').
    """
    if bias is None:
        bias = [0.0] * 6
    v = [float(gages_g0_g5[i]) - bias[i] for i in range(6)]
    out = [0.0] * 6
    for i in range(6):
        s = 0.0
        row = basic_matrix[i]
        for j in range(6):
            s += row[j] * v[j]
        out[i] = s
    f = [out[0] / counts_per_force, out[1] / counts_per_force, out[2] / counts_per_force]
    t = [out[3] / counts_per_torque, out[4] / counts_per_torque, out[5] / counts_per_torque]
    return f, t


def load_calibration_json(path: Path) -> tuple[list[list[float]], int, int]:
    data = json.loads(path.read_text(encoding="utf-8"))
    m = data["basic_matrix"]
    if len(m) != 6 or any(len(row) != 6 for row in m):
        raise ValueError("basic_matrix must be 6x6")
    cf = int(data["counts_per_force"])
    ct = int(data["counts_per_torque"])
    if cf == 0 or ct == 0:
        raise ValueError("counts_per_force and counts_per_torque must be non-zero")
    return m, cf, ct


def load_gauge_hw_from_json(path: Path) -> tuple[list[int], list[int]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    gains = [int(x) for x in data["gauge_gains"]]
    offsets = [int(x) for x in data["gauge_offsets"]]
    if len(gains) != 6 or len(offsets) != 6:
        raise ValueError("gauge_gains and gauge_offsets must each have 6 entries")
    return gains, offsets


def parse_sample(sample: bytes) -> tuple[list[int], int, bool]:
    """
    Returns (gage_ordered G0..G5), check_byte, checksum_ok.
    Wire order is G0,G2,G4,G1,G3,G5 per manual 8.4.2.
    """
    g0, g2, g4, g1, g3, g5 = struct.unpack(">hhhhhh", sample[:12])
    check = sample[12]
    ok = verify_sample_checksum(sample)
    ordered = [g0, g1, g2, g3, g4, g5]
    return ordered, check, ok


def logical_g_to_demo_column_order(logical: list[int]) -> list[int]:
    """
    ATI Demo log prints streaming order but labels columns G0..G5:
    col0=G0, col1=G2, col2=G4, col3=G1, col4=G3, col5=G5 (logical indices).
    """
    return [logical[0], logical[2], logical[4], logical[1], logical[3], logical[5]]


def demo_column_order_to_logical(demo_vals: list[float]) -> list[float]:
    """Inverse: six numbers as printed left-to-right in Demo → logical G0..G5."""
    if len(demo_vals) != 6:
        raise ValueError("need 6 values")
    d = demo_vals
    return [d[0], d[3], d[1], d[4], d[2], d[5]]


def open_port(
    port: str,
    baud: int,
    *,
    timeout: float,
) -> serial.Serial:
    return serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
    )


def _parse_host_port(s: str) -> tuple[str, int]:
    if ":" not in s:
        raise ValueError("expected HOST:PORT")
    host, _, port_s = s.rpartition(":")
    host = host.strip()
    if not host:
        raise ValueError("expected HOST:PORT")
    port = int(port_s.strip())
    if not (1 <= port <= 65535):
        raise ValueError("port must be 1..65535")
    return host, port


def read_streaming_samples(
    ser: serial.Serial,
    max_samples: int | None,
    print_every: int,
    *,
    cal_matrix: list[list[float]] | None,
    counts_force: int | None,
    counts_torque: int | None,
    tare_samples: int,
    demo_format: bool,
    fixed_bias_logical: list[float] | None,
    udp_publish: tuple[str, int] | None = None,
) -> None:
    ser.reset_input_buffer()
    tx = build_start_streaming_frame()
    ser.write(tx)
    ser.flush()
    ack_ok, pre_stream = read_start_streaming_ack(ser)
    if not ack_ok:
        print(
            "No valid Modbus ACK for start streaming. "
            "Check COM port, baud, parity (even), wiring, and power.",
            file=sys.stderr,
        )
        return
    # Manual: after ACK, sensor waits 20 ms then sends raw streaming (not Modbus).
    time.sleep(0.025)

    bias: list[float] | None = None
    tare_acc = [0.0] * 6
    tare_n = 0
    want_tare = tare_samples > 0 and fixed_bias_logical is None
    if fixed_bias_logical is not None:
        bias = list(fixed_bias_logical)
        print(
            f"# fixed bias (logical G0..G5)={['%.2f' % x for x in bias]}",
            file=sys.stderr,
        )

    udp_sock: socket.socket | None = None
    if udp_publish is not None:
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(
            f"# UDP publish Fz → {udp_publish[0]}:{udp_publish[1]} (ASCII per datagram)",
            file=sys.stderr,
        )

    n = 0
    buf = bytearray(pre_stream)
    try:
        while max_samples is None or n < max_samples:
            need = SAMPLE_SIZE - len(buf)
            chunk = ser.read(need if need > 0 else SAMPLE_SIZE)
            if not chunk:
                print("Timeout waiting for sample bytes.", file=sys.stderr)
                break
            buf.extend(chunk)
            while len(buf) >= SAMPLE_SIZE:
                resync_drop_until_valid(buf)
                if len(buf) < SAMPLE_SIZE:
                    break
                sample = bytes(buf[:SAMPLE_SIZE])
                del buf[:SAMPLE_SIZE]
                gages, check, ok = parse_sample(sample)
                status_err = (check & 0x80) != 0
                if want_tare and ok and tare_n < tare_samples:
                    for i in range(6):
                        tare_acc[i] += float(gages[i])
                    tare_n += 1
                    if tare_n >= tare_samples:
                        bias = [tare_acc[i] / tare_samples for i in range(6)]
                        want_tare = False
                        print(
                            f"# tare: bias G[0..5]={['%.2f' % x for x in bias]}",
                            file=sys.stderr,
                        )
                n += 1
                ft_ready = (
                    cal_matrix is not None
                    and counts_force is not None
                    and counts_torque is not None
                    and ok
                    and bias is not None
                )
                fxyz: list[float] | None = None
                txyz: list[float] | None = None
                if ft_ready:
                    assert cal_matrix is not None
                    assert counts_force is not None
                    assert counts_torque is not None
                    fxyz, txyz = gages_to_ft(
                        gages, cal_matrix, counts_force, counts_torque, bias
                    )
                    if udp_sock is not None and udp_publish is not None:
                        try:
                            udp_sock.sendto(
                                f"{fxyz[2]:.8f}\n".encode("ascii"),
                                udp_publish,
                            )
                        except OSError as e:
                            extra = ""
                            if getattr(e, "winerror", None) == 11001 or e.errno == 11001:
                                extra = (
                                    " — hostname not resolved; use Jetson IP "
                                    "(e.g. 192.168.1.10) not a placeholder."
                                )
                            print(f"# UDP send failed: {e}{extra}", file=sys.stderr)
                            udp_sock.close()
                            udp_sock = None

                if print_every <= 1 or (n % print_every == 0):
                    if demo_format:
                        lt = time.localtime()
                        ts = f"{lt.tm_mon}/{lt.tm_mday}/{lt.tm_year} {lt.tm_hour:02d}:{lt.tm_min:02d}:{lt.tm_sec:02d}"
                        g_demo = logical_g_to_demo_column_order(gages)
                        parts = [f"G{i}: {g_demo[i]}" for i in range(6)]
                        line = f"{ts}  " + "  ".join(parts)
                        if fxyz is not None and txyz is not None:
                            line += (
                                f"  Fx: {fxyz[0]:.2f}N Fy: {fxyz[1]:.2f}N Fz: {fxyz[2]:.2f}N"
                                f" Tx: {txyz[0]:.2f}N-m Ty: {txyz[1]:.2f}N-m Tz: {txyz[2]:.2f}N-m"
                            )
                        print(line)
                    else:
                        chk = "OK" if ok else "BAD_CHK"
                        st = "ERR" if status_err else "OK"
                        line = (
                            f"#{n:6d}  G[0..5]={gages}  "
                            f"check=0x{check:02x}  checksum={chk}  status={st}"
                        )
                        if fxyz is not None and txyz is not None:
                            line += (
                                f"  F=[{fxyz[0]:.4f},{fxyz[1]:.4f},{fxyz[2]:.4f}]"
                                f"  T=[{txyz[0]:.4f},{txyz[1]:.4f},{txyz[2]:.4f}]"
                            )
                        print(line)
    finally:
        if udp_sock is not None:
            try:
                udp_sock.close()
            except OSError:
                pass
        ser.write(bytes([0xFF]) * JAM_LEN)
        ser.flush()
        time.sleep(0.05)


def main() -> None:
    p = argparse.ArgumentParser(
        description="Read ATI Digital F/T from serial (e.g. COM4).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Shell tips:\n"
            "  If the first gage is negative, use equals so the value is not parsed as a flag:\n"
            "    --bias-demo-order=-2185,413,1026,-434,1826,1286\n"
            "    --bias-logical=-2185,490,1153,-130,1927,1595\n"
            "  Or quote the list: --bias-demo-order \"-2185,413,...\"\n"
            "  PowerShell line continuation is backtick ` at end of line (not cmd's ^)."
        ),
    )
    p.add_argument("--port", default="COM4", help="Serial port (default COM4)")
    p.add_argument(
        "--baud",
        type=int,
        default=1_250_000,
        choices=(1_250_000, 115_200, 19_200),
        help="Must match sensor (default 1250000)",
    )
    p.add_argument(
        "--samples",
        type=int,
        default=100,
        help="Number of samples to print (0 = until Ctrl+C)",
    )
    p.add_argument(
        "--every",
        type=int,
        default=1,
        help="Print every Nth sample (reduce console flood)",
    )
    p.add_argument(
        "--cal-json",
        type=Path,
        default=None,
        help="Calibration JSON with basic_matrix (6x6), counts_per_force, counts_per_torque",
    )
    p.add_argument(
        "--tare-samples",
        type=int,
        default=0,
        help="Average this many valid samples at start as bias (recommended with --cal-json)",
    )
    p.add_argument(
        "--apply-gains-from-json",
        type=Path,
        default=None,
        help="Before streaming: Modbus unlock + write gauge_gains/offsets (manual 8.6) from JSON",
    )
    p.add_argument(
        "--no-stream",
        action="store_true",
        help="Only apply gains/offsets (--apply-gains-from-json required) and exit",
    )
    p.add_argument(
        "--demo-format",
        action="store_true",
        help="Print like ATI Demo log: G in Demo column order, Fx/Fy/… with 2 decimals",
    )
    p.add_argument(
        "--bias-logical",
        type=str,
        default=None,
        metavar="LIST",
        help="Fixed tare: logical G0..G5 comma-separated. Use --bias-logical=-1,2,... if first value is negative.",
    )
    p.add_argument(
        "--bias-demo-order",
        type=str,
        default=None,
        metavar="LIST",
        help="Fixed tare: six values as Demo log order. Use --bias-demo-order=-1,2,... if first value is negative.",
    )
    p.add_argument(
        "--publish-udp",
        type=str,
        default=None,
        metavar="HOST:PORT",
        help="Send each Fz (N) as one UDP datagram (UTF-8 text, newline-terminated) to Jetson or other host",
    )
    args = p.parse_args()

    if args.no_stream and args.apply_gains_from_json is None:
        p.error("--no-stream requires --apply-gains-from-json")
    if args.bias_logical is not None and args.bias_demo_order is not None:
        p.error("use only one of --bias-logical and --bias-demo-order")
    if args.bias_logical is not None and args.tare_samples > 0:
        print("Note: --tare-samples ignored when --bias-logical is set.", file=sys.stderr)
    if args.bias_demo_order is not None and args.tare_samples > 0:
        print("Note: --tare-samples ignored when --bias-demo-order is set.", file=sys.stderr)

    def _parse_six_csv(s: str) -> list[float]:
        parts = [p.strip() for p in s.split(",")]
        if len(parts) != 6:
            raise ValueError("need exactly six comma-separated numbers")
        return [float(x) for x in parts]

    fixed_bias: list[float] | None = None
    try:
        if args.bias_logical is not None:
            fixed_bias = _parse_six_csv(args.bias_logical)
        elif args.bias_demo_order is not None:
            fixed_bias = demo_column_order_to_logical(_parse_six_csv(args.bias_demo_order))
    except ValueError as e:
        p.error(str(e))

    udp_target: tuple[str, int] | None = None
    if args.publish_udp is not None:
        try:
            udp_target = _parse_host_port(args.publish_udp)
        except ValueError as e:
            p.error(f"--publish-udp: {e}")
        assert udp_target is not None
        try:
            socket.getaddrinfo(
                udp_target[0], udp_target[1], socket.AF_INET, socket.SOCK_DGRAM
            )
        except OSError as e:
            print(
                f"Cannot resolve --publish-udp host {udp_target[0]!r}: {e}\n"
                "Use the Jetson's numeric IP on your LAN (e.g. 192.168.1.10).",
                file=sys.stderr,
            )
            raise SystemExit(2) from e

    cal_m = cal_cf = cal_ct = None
    if args.cal_json is not None:
        cal_m, cal_cf, cal_ct = load_calibration_json(args.cal_json)
        if args.tare_samples <= 0 and fixed_bias is None:
            print(
                "Tip: use --tare-samples 50, or --bias-demo-order / --bias-logical to match Demo tare.",
                file=sys.stderr,
            )

    max_samples = None if args.samples == 0 else args.samples
    ser = open_port(args.port, args.baud, timeout=0.5)
    try:
        if args.apply_gains_from_json is not None:
            gh, oh = load_gauge_hw_from_json(args.apply_gains_from_json)
            print(
                f"Applying hardware gains {gh} and offsets {oh} …",
                file=sys.stderr,
            )
            if not apply_gauge_gains_offsets(ser, gh, oh):
                raise SystemExit(1)
            print("Hardware gains/offsets applied (unlock → write → lock).", file=sys.stderr)
        if args.no_stream:
            return
        read_streaming_samples(
            ser,
            max_samples,
            max(1, args.every),
            cal_matrix=cal_m,
            counts_force=cal_cf,
            counts_torque=cal_ct,
            tare_samples=max(0, args.tare_samples),
            demo_format=args.demo_format,
            fixed_bias_logical=fixed_bias,
            udp_publish=udp_target,
        )
    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)
    finally:
        if ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
