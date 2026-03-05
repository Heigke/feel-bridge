#!/usr/bin/env python3
"""
fpga_host_eth.py — UDP Ethernet interface for NS-RAM FPGA bridge
================================================================
Replaces serial UART with UDP for ~50-100× faster communication.

FPGA IP: 192.168.0.50, UDP port: 7700
Protocol: same as UART — [0x55][CMD][payload]
Telemetry: 1029 bytes [0x55][0x02][LEN_HI][LEN_LO][128×8B][CRC8]

Usage:
    from fpga_host_eth import FPGAEthBridge
    fpga = FPGAEthBridge()
    fpga.connect()
    fpga.set_vg(0, 0.58)
    telem = fpga.read_telemetry()
    fpga.enable_auto_telemetry(1000)  # 1 kHz push
"""

import socket
import struct
import time
import numpy as np
from typing import Optional, List, Tuple

FPGA_IP   = "192.168.0.50"
FPGA_PORT = 7700
LOCAL_PORT = 7700  # bind to same port so FPGA replies here

CMD_SET_VG       = 0x01
CMD_READ_TELEM   = 0x02
CMD_SET_KILL     = 0x03
CMD_SET_SYNAPSE  = 0x04
CMD_SET_TEMP     = 0x05
CMD_SET_MAC      = 0x06
CMD_READ_DEBUG   = 0x07
CMD_SET_VG_BATCH = 0x08
CMD_SET_RATE     = 0x09
CMD_SET_LEAK     = 0x0A
CMD_SET_THRESH   = 0x0B
CMD_SET_BASE_EXC = 0x0C
CMD_SET_BIAS_GAIN= 0x0D
CMD_SET_DT_C     = 0x0E
CMD_SET_REFRACT  = 0x0F

NUM_NEURONS = 128
BYTES_PER_NEURON  = 8    # spike_cnt(2) + vmem_q16(4) + refract(2)
TELEM_PAYLOAD_LEN = 1024 # 128 * 8
TELEM_PACKET_LEN  = 1029 # 4 header + 1024 data + 1 CRC
# Legacy 6B format for backward compat detection
LEGACY_PAYLOAD_LEN = 768
LEGACY_PACKET_LEN  = 773


def crc8(data: bytes) -> int:
    """CRC-8 matching FPGA firmware (polynomial 0x07)."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


class FPGAEthBridge:
    """UDP-based FPGA bridge — drop-in replacement for serial FPGABridge."""

    def __init__(self, fpga_ip: str = FPGA_IP, fpga_port: int = FPGA_PORT,
                 local_port: int = LOCAL_PORT, timeout: float = 0.5):
        self.fpga_ip = fpga_ip
        self.fpga_port = fpga_port
        self.local_port = local_port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None
        self.num_neurons = NUM_NEURONS
        self._auto_telem = False

    def connect(self) -> bool:
        """Open UDP socket and send initial ping."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.local_port))
        self.sock.settimeout(self.timeout)

        # Send a kill-switch-off to register our IP with the FPGA
        self._send(bytes([0x55, CMD_SET_KILL, 0x00]))

        # Try to read telemetry to confirm connectivity
        try:
            telem = self.read_telemetry()
            if telem is not None:
                print(f"[ETH] Connected to FPGA at {self.fpga_ip}:{self.fpga_port}")
                print(f"[ETH] {self.num_neurons} neurons, round-trip < {self.timeout*1000:.0f}ms")
                return True
        except socket.timeout:
            pass

        print(f"[ETH] Warning: no telemetry response, FPGA may need programming")
        return False

    def close(self):
        """Close UDP socket."""
        if self.sock:
            self.sock.close()
            self.sock = None

    def _send(self, data: bytes):
        """Send raw bytes to FPGA."""
        if self.sock is None:
            raise RuntimeError("Not connected")
        self.sock.sendto(data, (self.fpga_ip, self.fpga_port))

    def _recv(self, bufsize: int = 2048, timeout: Optional[float] = None) -> bytes:
        """Receive raw bytes from FPGA."""
        if self.sock is None:
            raise RuntimeError("Not connected")
        old_timeout = self.sock.gettimeout()
        if timeout is not None:
            self.sock.settimeout(timeout)
        try:
            data, addr = self.sock.recvfrom(bufsize)
            return data
        finally:
            if timeout is not None:
                self.sock.settimeout(old_timeout)

    # ---- Commands ----

    def set_kill(self, kill: bool):
        """Set kill switch (True = all neurons off)."""
        self._send(bytes([0x55, CMD_SET_KILL, 0x01 if kill else 0x00]))

    def set_vg(self, neuron_id: int, vg: float):
        """Set gate voltage for one neuron. vg in volts, Q16.16 encoding."""
        vg_q16 = int(vg * 65536) & 0xFFFFFFFF
        pkt = struct.pack(">BBBI", 0x55, CMD_SET_VG, neuron_id & 0x7F, vg_q16)
        self._send(pkt)

    def set_vg_batch(self, start_id: int, vg_values: List[float]):
        """Set Vg for multiple neurons starting at start_id."""
        count = len(vg_values)
        hdr = struct.pack(">BBBB", 0x55, CMD_SET_VG_BATCH, start_id & 0x7F, count & 0xFF)
        payload = b""
        for v in vg_values:
            payload += struct.pack(">I", int(v * 65536) & 0xFFFFFFFF)
        self._send(hdr + payload)

    def set_temp(self, temp_k: float):
        """Set temperature in Kelvin, Q16.16."""
        tk_q16 = int(temp_k * 65536) & 0xFFFFFFFF
        self._send(struct.pack(">BBI", 0x55, CMD_SET_TEMP, tk_q16))

    def set_mac_signal(self, mac: float):
        """Set MAC feedback signal, Q16.16."""
        mac_q16 = int(mac * 65536) & 0xFFFFFFFF
        self._send(struct.pack(">BBI", 0x55, CMD_SET_MAC, mac_q16))

    def set_leak_cond(self, leak_q16: int):
        """Set leak conductance (raw Q16.16 value). E.g. 0x0004, 0x0008, 0x0011."""
        self._send(struct.pack(">BBI", 0x55, CMD_SET_LEAK, leak_q16 & 0xFFFFFFFF))

    def set_threshold(self, threshold: float):
        """Set spike threshold in volts, Q16.16. E.g. 0.50."""
        q16 = int(threshold * 65536) & 0xFFFFFFFF
        self._send(struct.pack(">BBI", 0x55, CMD_SET_THRESH, q16))

    def set_threshold_raw(self, threshold_q16: int):
        """Set spike threshold (raw Q16.16). E.g. 0x8000 = 0.50V."""
        self._send(struct.pack(">BBI", 0x55, CMD_SET_THRESH, threshold_q16 & 0xFFFFFFFF))

    def set_base_exc(self, base_exc: float):
        """Set avalanche excitation gain, Q16.16. E.g. 0.0125."""
        q16 = int(base_exc * 65536) & 0xFFFFFFFF
        self._send(struct.pack(">BBI", 0x55, CMD_SET_BASE_EXC, q16))

    def set_base_exc_raw(self, base_exc_q16: int):
        """Set avalanche excitation gain (raw Q16.16). E.g. 0x0333."""
        self._send(struct.pack(">BBI", 0x55, CMD_SET_BASE_EXC, base_exc_q16 & 0xFFFFFFFF))

    def set_bias_gain(self, bias_gain: float):
        """Set MAC current injection gain, Q16.16. E.g. 0.03125."""
        q16 = int(bias_gain * 65536) & 0xFFFFFFFF
        self._send(struct.pack(">BBI", 0x55, CMD_SET_BIAS_GAIN, q16))

    def set_bias_gain_raw(self, bias_gain_q16: int):
        """Set MAC current injection gain (raw Q16.16). E.g. 0x0800."""
        self._send(struct.pack(">BBI", 0x55, CMD_SET_BIAS_GAIN, bias_gain_q16 & 0xFFFFFFFF))

    def set_dt_over_c(self, dt_c: float):
        """Set integration step size, Q16.16. E.g. 0.0078."""
        q16 = int(dt_c * 65536) & 0xFFFFFFFF
        self._send(struct.pack(">BBI", 0x55, CMD_SET_DT_C, q16))

    def set_dt_over_c_raw(self, dt_c_q16: int):
        """Set integration step size (raw Q16.16). E.g. 0x0200."""
        self._send(struct.pack(">BBI", 0x55, CMD_SET_DT_C, dt_c_q16 & 0xFFFFFFFF))

    def set_refract_cycles(self, cycles: int):
        """Set refractory period in 10MHz clock cycles. E.g. 50 = 5us."""
        self._send(struct.pack(">BBH", 0x55, CMD_SET_REFRACT, cycles & 0xFFFF))

    def enable_auto_telemetry(self, rate_hz: int = 1000):
        """Enable FPGA auto-push telemetry at given rate.
        rate_hz: target rate in Hz. FPGA divider = 125_000_000 / rate_hz.
        Set to 0 to disable.
        """
        if rate_hz == 0:
            self._send(struct.pack(">BBH", 0x55, CMD_SET_RATE, 0))
            self._auto_telem = False
        else:
            # Divider in clock cycles at 125 MHz
            div = max(1, 125_000_000 // rate_hz)
            # Cap to 16-bit — host sends raw cycle count (limited to 65535)
            # For rates < ~1907 Hz, we need larger values. Use the 20-bit field:
            # But SET_RATE only takes 16 bits... so max div = 65535 → min rate = 1907 Hz
            # For lower rates, use polling instead
            if div > 65535:
                print(f"[ETH] Warning: rate {rate_hz} Hz too slow for auto-telem, using {125_000_000//65535} Hz")
                div = 65535
            self._send(struct.pack(">BBH", 0x55, CMD_SET_RATE, div & 0xFFFF))
            self._auto_telem = True

    def disable_auto_telemetry(self):
        """Disable auto-push telemetry."""
        self.enable_auto_telemetry(0)

    def read_telemetry(self, timeout: Optional[float] = None) -> Optional[dict]:
        """Request telemetry and parse response.
        Returns dict with 'spike_counts', 'vmem', 'bvpar' arrays (128 each).
        """
        if not self._auto_telem:
            # Send read request
            self._send(bytes([0x55, CMD_READ_TELEM]))

        try:
            data = self._recv(2048, timeout=timeout or self.timeout)
        except socket.timeout:
            return None

        return self._parse_telemetry(data)

    def recv_auto_telemetry(self, timeout: float = 0.1) -> Optional[dict]:
        """Receive next auto-pushed telemetry packet (no request sent)."""
        try:
            data = self._recv(2048, timeout=timeout)
        except socket.timeout:
            return None
        return self._parse_telemetry(data)

    def _parse_telemetry(self, data: bytes) -> Optional[dict]:
        """Parse telemetry packet. Supports both 8B (new) and 6B (legacy) formats."""
        if data[0] != 0x55 or data[1] != 0x02:
            return None

        payload_len = (data[2] << 8) | data[3]

        # Detect format by payload length
        if payload_len == TELEM_PAYLOAD_LEN:
            bpn = 8  # bytes per neuron
            pkt_len = TELEM_PACKET_LEN
        elif payload_len == LEGACY_PAYLOAD_LEN:
            bpn = 6
            pkt_len = LEGACY_PACKET_LEN
        else:
            return None

        if len(data) < pkt_len:
            return None

        # Validate CRC
        expected_crc = crc8(data[:pkt_len - 1])
        if data[pkt_len - 1] != expected_crc:
            return None

        spike_counts = np.zeros(self.num_neurons, dtype=np.uint16)
        vmem = np.zeros(self.num_neurons, dtype=np.float32)
        refract = np.zeros(self.num_neurons, dtype=np.uint16)

        for i in range(self.num_neurons):
            off = 4 + i * bpn
            sc = (data[off] << 8) | data[off + 1]
            spike_counts[i] = sc

            if bpn == 8:
                # New format: full 32-bit vmem (Q16.16) + 16-bit refractory
                vm_raw = (data[off+2] << 24) | (data[off+3] << 16) | (data[off+4] << 8) | data[off+5]
                vmem[i] = np.int32(vm_raw) / 65536.0
                refract[i] = (data[off+6] << 8) | data[off+7]
            else:
                # Legacy 6B format: 16-bit vmem + 16-bit bvpar
                vm_raw = (data[off+2] << 8) | data[off+3]
                vmem[i] = np.int16(vm_raw) / 256.0
                # No refract in legacy format

        return {
            "spike_counts": spike_counts,
            "vmem": vmem,
            "refract": refract,
            "timestamp": time.time(),
        }

    def read_telemetry_fast(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Minimal-overhead telemetry read for tight recurrence loops.
        Returns (spike_counts, vmem, refract) or raises TimeoutError.
        """
        if not self._auto_telem:
            self._send(bytes([0x55, CMD_READ_TELEM]))

        data = self._recv(2048, timeout=self.timeout)
        if len(data) < TELEM_PACKET_LEN or data[0] != 0x55 or data[1] != 0x02:
            raise TimeoutError("Bad telemetry")

        spike_counts = np.zeros(self.num_neurons, dtype=np.uint16)
        vmem = np.zeros(self.num_neurons, dtype=np.float32)
        refract = np.zeros(self.num_neurons, dtype=np.uint16)

        for i in range(self.num_neurons):
            off = 4 + i * 8
            spike_counts[i] = (data[off] << 8) | data[off + 1]
            vm_raw = (data[off+2] << 24) | (data[off+3] << 16) | (data[off+4] << 8) | data[off+5]
            vmem[i] = np.int32(vm_raw) / 65536.0
            refract[i] = (data[off+6] << 8) | data[off+7]

        return spike_counts, vmem, refract


# ---- Convenience: backward-compatible find_fpga + read_telem ----

def find_fpga_eth(fpga_ip: str = FPGA_IP, timeout: float = 0.5) -> Optional[FPGAEthBridge]:
    """Find and connect to FPGA over Ethernet. Returns FPGAEthBridge or None."""
    bridge = FPGAEthBridge(fpga_ip=fpga_ip, timeout=timeout)
    try:
        if bridge.connect():
            return bridge
    except Exception as e:
        print(f"[ETH] Connection failed: {e}")
        bridge.close()
    return None


def benchmark_roundtrip(fpga: FPGAEthBridge, n_trials: int = 100):
    """Measure UDP round-trip latency."""
    latencies = []
    for _ in range(n_trials):
        t0 = time.perf_counter()
        telem = fpga.read_telemetry(timeout=0.5)
        t1 = time.perf_counter()
        if telem is not None:
            latencies.append((t1 - t0) * 1000)

    if latencies:
        arr = np.array(latencies)
        print(f"[ETH] Round-trip latency: mean={arr.mean():.2f}ms, "
              f"std={arr.std():.2f}ms, min={arr.min():.2f}ms, max={arr.max():.2f}ms")
        print(f"[ETH] Max sustainable rate: {1000/arr.mean():.0f} Hz")
        return arr
    else:
        print("[ETH] No successful round-trips!")
        return None


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="FPGA Ethernet Bridge Test")
    parser.add_argument("--ip", default=FPGA_IP, help="FPGA IP address")
    parser.add_argument("--benchmark", action="store_true", help="Run latency benchmark")
    parser.add_argument("--auto", type=int, default=0, help="Enable auto-telemetry at Hz")
    parser.add_argument("--test", action="store_true", help="Basic connectivity test")
    args = parser.parse_args()

    fpga = FPGAEthBridge(fpga_ip=args.ip)
    if not fpga.connect():
        print("[ETH] Failed to connect. Is FPGA programmed with nsram_eth_top?")
        exit(1)

    if args.test:
        print("\n[TEST] Kill switch off...")
        fpga.set_kill(False)
        time.sleep(0.1)

        print("[TEST] Setting Vg for neurons 0-7...")
        vg_vals = [0.50, 0.55, 0.58, 0.60, 0.62, 0.65, 0.70, 0.75]
        fpga.set_vg_batch(0, vg_vals)
        time.sleep(0.5)

        print("[TEST] Reading telemetry...")
        telem = fpga.read_telemetry()
        if telem:
            sc = telem["spike_counts"]
            vm = telem["vmem"]
            rf = telem["refract"]
            print(f"  Spike counts (first 8): {sc[:8]}")
            print(f"  Vmem Q16.16 (first 8): {vm[:8]}")
            print(f"  Refract ctr (first 8): {rf[:8]}")
            print(f"  Non-zero neurons: {np.count_nonzero(sc)}/{len(sc)}")
            print(f"  Total spikes: {sc.sum()}")
            print(f"  Packet: {len(telem.get('_raw', b''))} bytes (expect 1029)")
        else:
            print("  No telemetry received!")

        print("\n[TEST] Kill switch on...")
        fpga.set_kill(True)
        time.sleep(0.5)
        telem = fpga.read_telemetry()
        if telem:
            print(f"  Spike counts after kill: {telem['spike_counts'].sum()}")

    if args.benchmark:
        print("\n[BENCHMARK] Round-trip latency measurement...")
        benchmark_roundtrip(fpga, n_trials=200)

    if args.auto > 0:
        print(f"\n[AUTO] Enabling auto-telemetry at {args.auto} Hz...")
        fpga.enable_auto_telemetry(args.auto)
        t0 = time.time()
        count = 0
        try:
            while time.time() - t0 < 5.0:
                telem = fpga.recv_auto_telemetry(timeout=0.1)
                if telem:
                    count += 1
                    if count % 100 == 0:
                        sc = telem["spike_counts"]
                        print(f"  [{count}] total_spikes={sc.sum()}, non-zero={np.count_nonzero(sc)}")
        except KeyboardInterrupt:
            pass
        elapsed = time.time() - t0
        print(f"  Received {count} packets in {elapsed:.1f}s = {count/elapsed:.0f} Hz")
        fpga.disable_auto_telemetry()

    fpga.close()
