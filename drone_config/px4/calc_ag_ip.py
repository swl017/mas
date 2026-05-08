#!/usr/bin/env python3
"""Convert between an IPv4 address and the int32 value PX4 stores in UXRCE_DDS_AG_IP.

PX4 packs the address MSB-first: A.B.C.D -> (A<<24)|(B<<16)|(C<<8)|D.
The param is INT32, so values with A >= 128 appear as negative in QGC.

Usage:
    calc_ag_ip.py 192.168.144.10     # IP -> int32 (both signed and unsigned)
    calc_ag_ip.py -1062694902        # int32 -> IP
"""
import sys


def ip_to_int(ip: str) -> int:
    octets = [int(x) for x in ip.split(".")]
    if len(octets) != 4 or any(o < 0 or o > 255 for o in octets):
        raise ValueError(f"not a valid IPv4 address: {ip}")
    return (octets[0] << 24) | (octets[1] << 16) | (octets[2] << 8) | octets[3]


def int_to_ip(value: int) -> str:
    u = value & 0xFFFFFFFF
    return f"{(u >> 24) & 0xFF}.{(u >> 16) & 0xFF}.{(u >> 8) & 0xFF}.{u & 0xFF}"


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 1
    arg = argv[1]
    if "." in arg:
        unsigned = ip_to_int(arg)
        signed = unsigned - (1 << 32) if unsigned >= (1 << 31) else unsigned
        print(f"IP            : {arg}")
        print(f"UXRCE_DDS_AG_IP (signed,   for QGC): {signed}")
        print(f"UXRCE_DDS_AG_IP (unsigned, for ref): {unsigned}")
        print(f"hex                                : 0x{unsigned:08X}")
    else:
        print(f"IP: {int_to_ip(int(arg))}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
