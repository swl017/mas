#!/usr/bin/env python3
"""
Bench test: measure 0x0D pitch drift under sustained yaw rotation.

Since 0x0D pitch is heading-frame (IMU-based), centrifugal force during
sustained turns could corrupt the accelerometer gravity reference,
causing pitch to drift.

Test procedure:
  1. Place gimbal on bench, let it settle (Lock mode)
  2. Record baseline pitch for 5s (stationary)
  3. Continuously rotate the base in yaw (simulate orbit) for 20s
  4. Stop rotation, record recovery for 5s
  5. Report: pitch deviation during rotation vs baseline

Usage:
  python3 test_pitch_drift.py [IP]

  Phase 1 (5s):  Keep gimbal STATIONARY — baseline
  Phase 2 (20s): ROTATE the base continuously in yaw (steady circular motion)
  Phase 3 (5s):  STOP rotating — observe recovery
"""

import sys
import time
sys.path.insert(0, '/home/usrg/mas/src/gimbal_controller')

from gimbal_controller.siyi_sdk.siyi_sdk import SIYISDK

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.144.26"


def record(cam, duration, rate_hz=20):
    """Record 0x0D samples."""
    samples = []
    t0 = time.time()
    while time.time() - t0 < duration:
        yaw, pitch, roll = cam.getAttitude()
        samples.append({
            't': time.time() - t0,
            'yaw': yaw,
            'pitch': pitch,
            'roll': roll,
        })
        time.sleep(1.0 / rate_hz)
    return samples


def stats(samples, axis='pitch'):
    vals = [s[axis] for s in samples]
    if not vals:
        return {}
    mean = sum(vals) / len(vals)
    return {
        'min': min(vals),
        'max': max(vals),
        'mean': mean,
        'range': max(vals) - min(vals),
        'abs_max': max(abs(v) for v in vals),
        'n': len(vals),
    }


def print_phase(name, samples):
    for axis in ['yaw', 'pitch', 'roll']:
        s = stats(samples, axis)
        if not s:
            continue
        print(f"  {axis:<6}: mean={s['mean']:>7.2f}  range={s['range']:>7.2f}  "
              f"min={s['min']:>7.2f}  max={s['max']:>7.2f}  ({s['n']} samples)")


def main():
    cam = SIYISDK(server_ip=IP, port=37260)
    if not cam.connect():
        print(f"FAILED to connect to {IP}")
        sys.exit(1)

    print(f"Connected to {IP}")
    time.sleep(0.5)

    # Lock mode, center gimbal
    cam.requestLockMode()
    time.sleep(0.5)
    cam.requestCenterGimbal()
    time.sleep(2)

    yaw, pitch, roll = cam.getAttitude()
    print(f"Initial: yaw={yaw:.1f} pitch={pitch:.1f} roll={roll:.1f}")

    # Phase 1: Baseline (stationary)
    print(f"\n{'='*60}")
    print(f"  Phase 1: BASELINE (keep gimbal STATIONARY) — 5 seconds")
    print(f"{'='*60}")
    baseline = record(cam, 5.0, rate_hz=20)
    print_phase("Baseline", baseline)
    baseline_pitch = stats(baseline, 'pitch')['mean']

    # Phase 2: Sustained rotation
    print(f"\n{'='*60}")
    print(f"  Phase 2: ROTATE the base continuously in yaw — 20 seconds")
    print(f"  >>> START ROTATING NOW <<<")
    print(f"  (steady back-and-forth or circular yaw motion)")
    print(f"{'='*60}")
    rotation = record(cam, 20.0, rate_hz=20)
    print_phase("Rotation", rotation)

    # Phase 3: Recovery (stop rotation)
    print(f"\n{'='*60}")
    print(f"  Phase 3: RECOVERY (STOP rotating) — 5 seconds")
    print(f"{'='*60}")
    recovery = record(cam, 5.0, rate_hz=20)
    print_phase("Recovery", recovery)

    # Summary
    rot_pitch = stats(rotation, 'pitch')
    rec_pitch = stats(recovery, 'pitch')

    print(f"\n{'='*60}")
    print(f"  PITCH DRIFT SUMMARY")
    print(f"{'='*60}")
    print(f"  Baseline pitch mean:   {baseline_pitch:>7.2f}°")
    print(f"  Rotation pitch range:  {rot_pitch['range']:>7.2f}°  (max deviation: {rot_pitch['abs_max']:.2f}°)")
    print(f"  Recovery pitch mean:   {rec_pitch['mean']:>7.2f}°  (offset from baseline: {rec_pitch['mean'] - baseline_pitch:.2f}°)")
    print(f"")
    if rot_pitch['range'] < 1.0:
        print(f"  VERDICT: Pitch drift is NEGLIGIBLE (<1°) — safe for triangulation")
    elif rot_pitch['range'] < 5.0:
        print(f"  VERDICT: Pitch drift is MODERATE ({rot_pitch['range']:.1f}°) — may need compensation")
    else:
        print(f"  VERDICT: Pitch drift is SIGNIFICANT ({rot_pitch['range']:.1f}°) — needs compensation or filter")

    cam.disconnect()
    print("\nDone.")


if __name__ == '__main__':
    main()
