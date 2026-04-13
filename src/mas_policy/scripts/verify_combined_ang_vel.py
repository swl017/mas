#!/usr/bin/env python3
"""Verify combined angular velocity Jacobian: training vs deploy.

Compares the ZXY Jacobian from the training env (derived_field_computers.py)
with the simplified computation in los_rate_controller.py.

Run:
    cd src/mas_policy
    python3 scripts/verify_combined_ang_vel.py
"""

import numpy as np


def training_jacobian(pitch_rate, yaw_rate, roll_rate, yaw_pos, roll_pos):
    """Training: full ZXY Jacobian (derived_field_computers.py:340-344)."""
    cy, sy = np.cos(yaw_pos), np.sin(yaw_pos)
    cr, sr = np.cos(roll_pos), np.sin(roll_pos)
    return np.array([
        -sy * cr * pitch_rate + cy * roll_rate,   # X
         cy * cr * pitch_rate + sy * roll_rate,    # Y
         sr * pitch_rate + yaw_rate,               # Z
    ])


def deploy_simplified(pitch_rate, yaw_rate, roll_rate, yaw_pos, roll_pos):
    """Deploy: simplified (los_rate_controller.py:612, BEFORE fix).

    Assumes gimbal axes aligned with body axes (yaw=0, roll=0).
    """
    return np.array([0.0, pitch_rate, yaw_rate])


def deploy_fixed(pitch_rate, yaw_rate, roll_rate, yaw_pos, roll_pos):
    """Deploy: full ZXY Jacobian (los_rate_controller.py, AFTER fix)."""
    cy, sy = np.cos(yaw_pos), np.sin(yaw_pos)
    cr, sr = np.cos(roll_pos), np.sin(roll_pos)
    return np.array([
        -sy * cr * pitch_rate + cy * roll_rate,
         cy * cr * pitch_rate + sy * roll_rate,
         sr * pitch_rate + yaw_rate,
    ])


def main():
    print("=" * 70)
    print("Combined Angular Velocity Jacobian Verification")
    print("Training (ZXY Jacobian) vs Deploy (simplified)")
    print("=" * 70)

    # Test at various gimbal positions
    test_cases = [
        # (yaw_pos, roll_pos, pitch_rate, yaw_rate, roll_rate, description)
        (0.0, 0.0, 1.0, 0.0, 0.0, "pitch_rate=1, gimbal at origin"),
        (0.0, 0.0, 0.0, 1.0, 0.0, "yaw_rate=1, gimbal at origin"),
        (0.0, 0.0, 0.0, 0.0, 1.0, "roll_rate=1, gimbal at origin"),
        (0.5, 0.0, 1.0, 0.0, 0.0, "pitch_rate=1, yaw=0.5 rad (29 deg)"),
        (1.0, 0.0, 1.0, 0.0, 0.0, "pitch_rate=1, yaw=1.0 rad (57 deg)"),
        (np.pi/2, 0.0, 1.0, 0.0, 0.0, "pitch_rate=1, yaw=90 deg"),
        (0.0, 0.3, 1.0, 0.0, 0.0, "pitch_rate=1, roll=0.3 rad (17 deg)"),
        (0.5, 0.3, 1.0, 1.0, 0.0, "mixed rates, yaw=0.5, roll=0.3"),
        (1.0, 0.0, 2.0, 1.5, 0.0, "realistic: large yaw, high rates"),
    ]

    print(f"\n{'Description':<50s} {'Max |diff|':>10s} {'Train':>20s} {'Deploy':>20s}")
    print("-" * 105)

    max_overall = 0.0
    for yaw_pos, roll_pos, p_rate, y_rate, r_rate, desc in test_cases:
        train = training_jacobian(p_rate, y_rate, r_rate, yaw_pos, roll_pos)
        deploy = deploy_simplified(p_rate, y_rate, r_rate, yaw_pos, roll_pos)
        diff = np.max(np.abs(train - deploy))
        max_overall = max(max_overall, diff)

        t_str = f"[{train[0]:+.3f}, {train[1]:+.3f}, {train[2]:+.3f}]"
        d_str = f"[{deploy[0]:+.3f}, {deploy[1]:+.3f}, {deploy[2]:+.3f}]"
        marker = " <<<" if diff > 0.01 else ""
        print(f"{desc:<50s} {diff:10.4f} {t_str:>20s} {d_str:>20s}{marker}")

    print(f"\n{'Max overall difference:':<50s} {max_overall:10.4f}")

    # Statistical test: random gimbal configurations
    np.random.seed(42)
    N = 10000
    yaw_pos = np.random.uniform(-np.pi, np.pi, N)
    roll_pos = np.random.uniform(-0.5, 0.5, N)  # roll is typically small
    pitch_rate = np.random.uniform(-3.0, 3.0, N)
    yaw_rate = np.random.uniform(-3.0, 3.0, N)
    roll_rate = np.zeros(N)  # roll rate is typically zero in deployment

    diffs = []
    for i in range(N):
        train = training_jacobian(pitch_rate[i], yaw_rate[i], roll_rate[i], yaw_pos[i], roll_pos[i])
        deploy = deploy_simplified(pitch_rate[i], yaw_rate[i], roll_rate[i], yaw_pos[i], roll_pos[i])
        diffs.append(np.max(np.abs(train - deploy)))
    diffs = np.array(diffs)

    print(f"\n--- Random test ({N} samples) ---")
    print(f"  yaw_pos ~ U(-pi, pi), roll_pos ~ U(-0.5, 0.5)")
    print(f"  pitch_rate, yaw_rate ~ U(-3, 3), roll_rate = 0")
    print(f"  Max diff:  {diffs.max():.4f}")
    print(f"  Mean diff: {diffs.mean():.4f}")
    print(f"  Median:    {np.median(diffs):.4f}")
    print(f"  95th pct:  {np.percentile(diffs, 95):.4f}")
    print(f"  99th pct:  {np.percentile(diffs, 99):.4f}")
    print(f"  Fraction > 0.1: {(diffs > 0.1).mean():.1%}")
    print(f"  Fraction > 0.5: {(diffs > 0.5).mean():.1%}")

    print(f"\n{'=' * 70}")
    if max_overall < 1e-6:
        print("RESULT (old): Jacobians are IDENTICAL (max diff < 1e-6)")
    elif diffs.mean() < 0.01:
        print("RESULT (old): Jacobians DIFFER but mean error is small (<0.01)")
    else:
        print("RESULT (old): Jacobians DIFFER SIGNIFICANTLY")

    # --- Verify the FIXED deploy implementation ---
    print(f"\n\n{'=' * 70}")
    print("Verification of FIXED deploy (full ZXY Jacobian)")
    print("=" * 70)

    np.random.seed(42)
    N = 10000
    yaw_pos = np.random.uniform(-np.pi, np.pi, N)
    roll_pos = np.random.uniform(-0.5, 0.5, N)
    pitch_rate = np.random.uniform(-3.0, 3.0, N)
    yaw_rate = np.random.uniform(-3.0, 3.0, N)
    roll_rate = np.random.uniform(-1.0, 1.0, N)

    diffs_fixed = []
    for i in range(N):
        train = training_jacobian(pitch_rate[i], yaw_rate[i], roll_rate[i], yaw_pos[i], roll_pos[i])
        fixed = deploy_fixed(pitch_rate[i], yaw_rate[i], roll_rate[i], yaw_pos[i], roll_pos[i])
        diffs_fixed.append(np.max(np.abs(train - fixed)))
    diffs_fixed = np.array(diffs_fixed)

    print(f"  Max diff:  {diffs_fixed.max():.2e}")
    print(f"  Mean diff: {diffs_fixed.mean():.2e}")
    if diffs_fixed.max() < 1e-10:
        print("RESULT (fixed): IDENTICAL — ZXY Jacobian matches training exactly")
    else:
        print(f"RESULT (fixed): MISMATCH — max diff {diffs_fixed.max():.2e}")


if __name__ == "__main__":
    main()
