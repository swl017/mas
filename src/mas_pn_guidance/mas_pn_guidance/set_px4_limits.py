"""Push an interceptor performance band onto a PX4 SITL vehicle via MAVLink.

`mavros_replicator` is a uORB->mavros-shaped translator with no MAVLink param
path, so PX4 limits are set here over MAVLink directly with pymavlink (the same
transport PegasusSimulator's PX4MavlinkBackend uses). Each band's MPC params come
from ``config/px4_bands.yaml``; every param is set and then read back to confirm,
so the harness reproduces the point-mass physics instead of the airframe default
(`MPC_XY_VEL_MAX=12`, `MPC_ACC_HOR_MAX=5`).

Used standalone in Slice 1 and called by the engagement conductor (Slice 5)
before each trial.

Examples::

    # Dry run — print intended param sets, no connection (testable offline):
    ros2 run mas_pn_guidance set_px4_limits --band paper_nominal_low --dry-run

    # Live — resolve the interceptor's MAVLink sysid and set the band:
    ros2 run mas_pn_guidance set_px4_limits --band paper_nominal_low \
        --connection udpin:0.0.0.0:14550 --sysid 2

`--connection` and `--sysid` must be confirmed against the running sim at first
bring-up (PX4 SITL MAVLink endpoints depend on the instance/rcS); once known,
wire the defaults into the conductor. `--role`/`--vehicles` can auto-resolve the
sysid from mas_offboard's vehicles.yaml (target_system) instead of `--sysid`.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Dict, Optional

import yaml


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def _share_config(filename: str) -> str:
    """Resolve a config file from the installed share dir, then the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = os.path.join(
            get_package_share_directory('mas_pn_guidance'), 'config', filename)
        if os.path.isfile(cand):
            return cand
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, '..', 'config', filename))


def load_band_params(band: str, bands_file: Optional[str] = None) -> Dict[str, float]:
    path = bands_file or _share_config('px4_bands.yaml')
    with open(path, 'r') as fh:
        doc = yaml.safe_load(fh) or {}
    bands = doc.get('bands') or {}
    if band not in bands:
        raise KeyError(f"band '{band}' not in {path} (have: {sorted(bands)})")
    params = bands[band].get('px4_params') or {}
    if not params:
        raise ValueError(f"band '{band}' has no px4_params in {path}")
    return {str(k): float(v) for k, v in params.items()}


def resolve_sysid_from_vehicles(role: str,
                                vehicles_file: Optional[str] = None,
                                roles_file: Optional[str] = None) -> int:
    """Map a logical role -> PX4 namespace (roles.yaml) -> MAV sysid (vehicles.yaml).

    mas_offboard/config/vehicles.yaml lists `target_system` per namespace, which is
    the PX4 MAV_SYS_ID for that SITL instance.
    """
    from .roles import Roles
    ns = Roles.load(path=roles_file).namespace(role)
    if vehicles_file is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            vehicles_file = os.path.join(
                get_package_share_directory('mas_offboard'), 'config', 'vehicles.yaml')
        except Exception:
            raise FileNotFoundError(
                "vehicles.yaml not found; pass --vehicles explicitly")
    with open(vehicles_file, 'r') as fh:
        doc = yaml.safe_load(fh) or {}
    for v in doc.get('vehicles', []):
        if v.get('namespace') == ns:
            return int(v.get('target_system'))
    raise KeyError(f"namespace '{ns}' (role '{role}') not in {vehicles_file}")


# ---------------------------------------------------------------------------
# MAVLink set + verify
# ---------------------------------------------------------------------------

def set_and_verify(connection: str,
                   params: Dict[str, float],
                   sysid: Optional[int] = None,
                   rel_tol: float = 1e-3,
                   per_param_timeout: float = 3.0,
                   heartbeat_timeout: float = 15.0) -> bool:
    """Set each param over MAVLink and confirm the read-back. Returns all-ok."""
    from pymavlink import mavutil

    master = mavutil.mavlink_connection(connection, source_system=255)
    print(f"[set_px4_limits] waiting for heartbeat on {connection}"
          f"{f' (sysid {sysid})' if sysid else ''} ...")
    hb = master.wait_heartbeat(timeout=heartbeat_timeout)
    if hb is None:
        print("[set_px4_limits] ERROR: no heartbeat", file=sys.stderr)
        return False
    if sysid is not None:
        master.target_system = sysid
        # component is autopilot main (1)
        master.target_component = master.target_component or 1
    print(f"[set_px4_limits] target sys={master.target_system} "
          f"comp={master.target_component}")

    all_ok = True
    for name, value in params.items():
        ok = _set_one(master, name, value, rel_tol, per_param_timeout)
        flag = "ok" if ok else "FAIL"
        print(f"  {name:16s} -> {value:<8g} [{flag}]")
        all_ok = all_ok and ok
    print(f"[set_px4_limits] {'all params confirmed' if all_ok else 'SOME PARAMS FAILED'}")
    return all_ok


def _set_one(master, name: str, value: float,
             rel_tol: float, timeout: float) -> bool:
    from pymavlink import mavutil
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode('ascii'), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    # Read back: request the one param, wait for a matching PARAM_VALUE.
    deadline = time.time() + timeout
    master.mav.param_request_read_send(
        master.target_system, master.target_component, name.encode('ascii'), -1)
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg is None:
            master.mav.param_request_read_send(
                master.target_system, master.target_component,
                name.encode('ascii'), -1)
            continue
        pid = msg.param_id
        if isinstance(pid, bytes):
            pid = pid.decode('ascii', 'ignore')
        pid = pid.rstrip('\x00')
        if pid != name:
            continue
        got = float(msg.param_value)
        tol = max(abs(value) * rel_tol, 1e-4)
        return abs(got - value) <= tol
    return False


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Set PX4 MPC limits for a band.")
    p.add_argument('--band', required=True, help="band key in px4_bands.yaml")
    p.add_argument('--bands-file', default=None, help="px4_bands.yaml override")
    p.add_argument('--connection', default=None,
                   help="pymavlink connection string, e.g. udpin:0.0.0.0:14550")
    p.add_argument('--sysid', type=int, default=None,
                   help="target MAV_SYS_ID (disambiguates multi-vehicle)")
    p.add_argument('--role', default=None,
                   help="resolve --sysid from this role via roles+vehicles.yaml")
    p.add_argument('--vehicles', default=None, help="mas_offboard vehicles.yaml override")
    p.add_argument('--dry-run', action='store_true',
                   help="print intended sets without connecting")
    args = p.parse_args(argv)

    params = load_band_params(args.band, args.bands_file)
    sysid = args.sysid
    if sysid is None and args.role is not None:
        sysid = resolve_sysid_from_vehicles(args.role, args.vehicles)

    print(f"[set_px4_limits] band '{args.band}' "
          f"({len(params)} params){f', sysid {sysid}' if sysid else ''}")
    if args.dry_run:
        for name, value in params.items():
            print(f"  {name:16s} -> {value:g}")
        print("[set_px4_limits] dry-run: nothing sent")
        return 0

    if not args.connection:
        print("[set_px4_limits] ERROR: --connection required for a live set "
              "(or use --dry-run)", file=sys.stderr)
        return 2
    ok = set_and_verify(args.connection, params, sysid=sysid)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
