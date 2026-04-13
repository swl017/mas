#!/usr/bin/env python3
"""
Bench test: verify 0x0D yaw/pitch/roll behavior in Lock, Follow, FPV modes.

Test procedure:
  1. Connect to gimbal, center it
  2. For each mode (Lock, Follow, FPV):
     a. Set the mode
     b. Stream 0x0D attitude at ~10 Hz for 10 seconds
     c. User rotates gimbal base (yaw) during recording
  3. Compare: does 0x0D yaw change when base rotates?
     - Lock: yaw should STAY CONSTANT (earth-frame, all axes)
     - Follow: yaw should CHANGE with body (body-frame yaw)
     - FPV: all axes should change with body

Usage:
  python3 test_modes.py [IP]

  Manually rotate the gimbal base in yaw during each 10s recording window.
"""

import sys
import time
sys.path.insert(0, '/home/usrg/mas/src/gimbal_controller')

from gimbal_controller.siyi_sdk.siyi_sdk import SIYISDK

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.144.26"


def set_mode(cam, mode_name):
    """Set gimbal mode by name."""
    if mode_name == 'Lock':
        return cam.requestLockMode()
    elif mode_name == 'Follow':
        return cam.requestFollowMode()
    elif mode_name == 'FPV':
        return cam.requestFPVMode()
    return False


def record_attitude(cam, duration=10.0, rate_hz=10):
    """Record 0x0D attitude samples for `duration` seconds."""
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


def print_summary(mode_name, samples):
    """Print min/max/range for each axis."""
    if not samples:
        print("  No samples recorded!")
        return
    yaws = [s['yaw'] for s in samples]
    pitches = [s['pitch'] for s in samples]
    rolls = [s['roll'] for s in samples]

    print(f"\n  === {mode_name} mode ({len(samples)} samples over {samples[-1]['t']:.1f}s) ===")
    print(f"  {'Axis':<8} {'Min':>8} {'Max':>8} {'Range':>8} {'Start':>8} {'End':>8}")
    print(f"  {'----':<8} {'---':>8} {'---':>8} {'-----':>8} {'-----':>8} {'---':>8}")
    for name, vals in [('Yaw', yaws), ('Pitch', pitches), ('Roll', rolls)]:
        print(f"  {name:<8} {min(vals):>8.1f} {max(vals):>8.1f} {max(vals)-min(vals):>8.1f} {vals[0]:>8.1f} {vals[-1]:>8.1f}")

    # Print first and last 3 samples for inspection
    fmt = lambda s: f"({s['t']:.1f}s: y={s['yaw']:.1f} p={s['pitch']:.1f} r={s['roll']:.1f})"
    print(f"\n  First 3: {', '.join(fmt(s) for s in samples[:3])}")
    print(f"  Last  3: {', '.join(fmt(s) for s in samples[-3:])}")


def main():
    cam = SIYISDK(server_ip=IP, port=37260)
    if not cam.connect():
        print(f"FAILED to connect to {IP}")
        sys.exit(1)

    print(f"Connected to {IP}")
    time.sleep(0.5)

    # Center gimbal first
    print("\nCentering gimbal...")
    cam.requestCenterGimbal()
    time.sleep(2)

    # Read initial attitude
    yaw, pitch, roll = cam.getAttitude()
    print(f"Initial attitude: yaw={yaw:.1f} pitch={pitch:.1f} roll={roll:.1f}")

    for mode_name in ['Lock', 'Follow', 'FPV']:
        print(f"\n{'='*60}")
        print(f"  Testing: {mode_name} mode")
        print(f"{'='*60}")

        ok = set_mode(cam, mode_name)
        print(f"  Mode set command sent: {ok}")
        time.sleep(1.5)

        # Read attitude right after mode switch
        yaw, pitch, roll = cam.getAttitude()
        print(f"  After mode switch: yaw={yaw:.1f} pitch={pitch:.1f} roll={roll:.1f}")

        print(f"\n  >>> ROTATE THE BASE IN YAW NOW (10 seconds) <<<")
        samples = record_attitude(cam, duration=10.0, rate_hz=10)
        print_summary(mode_name, samples)

        # Pause between modes
        print(f"\n  Returning to center...")
        cam.requestCenterGimbal()
        time.sleep(2)

    # Reset to Lock mode
    print(f"\nResetting to Lock mode...")
    cam.requestLockMode()
    time.sleep(0.5)

    cam.disconnect()
    print("\nDone. Compare yaw ranges across modes:")
    print("  Lock:   yaw range should be SMALL (earth-frame, ignores body rotation)")
    print("  Follow: yaw range should be LARGE (body-frame yaw, follows body)")
    print("  FPV:    yaw range should be LARGE (all axes body-frame)")


if __name__ == '__main__':
    main()
