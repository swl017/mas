"""Unit tests for the stamped LOS-rate differentiator (ticket 012, review #6).

Pins the anti-aliasing / dropout / source-switch discipline that both the
range-tolerant guidance modes (`bearing_pn`, `raw_ibvs`) rely on. Pure math —
no ROS spin required (the node feeds this class the LOS + a timestamp).
"""
import numpy as np

from mas_pn_guidance.los_rate import StampedLosRateDifferentiator, coast_decay

DT_NS = 40_000_000   # 25 Hz detection stamp spacing (ns)
OMEGA_TRUE = 0.1     # rad/s LOS rotation rate about +z


def _los(theta):
    return np.array([np.cos(theta), np.sin(theta), 0.0])


def test_recovers_constant_los_rate():
    """A LOS rotating at 0.1 rad/s about +z -> Ω = (0,0,0.1)."""
    d = StampedLosRateDifferentiator(ema_alpha=0.7)
    t = 0
    for k in range(60):
        d.update(_los(OMEGA_TRUE * (t * 1e-9)), t)
        t += DT_NS
    assert abs(d.omega[2] - OMEGA_TRUE) < 1e-3, d.omega
    assert abs(d.omega[0]) < 1e-6 and abs(d.omega[1]) < 1e-6, d.omega


def test_no_aliasing_when_consumed_faster_than_sampled():
    """The review's aliasing test: a 25 Hz LOS consumed by a 50 Hz loop must
    not be aliased. Feeding the same stamp twice (two control ticks per
    detection) holds the rate — it does not double-count or blow up."""
    d = StampedLosRateDifferentiator(ema_alpha=0.7)
    t = 0
    for k in range(60):
        n = _los(OMEGA_TRUE * (t * 1e-9))
        first = d.update(n, t).copy()          # detection tick
        held = d.update(n, t).copy()            # extra control tick, same stamp
        assert np.array_equal(first, held), (first, held)  # repeat -> exact hold
        t += DT_NS
    assert abs(d.omega[2] - OMEGA_TRUE) < 1e-3, d.omega


def test_zero_los_rate_gives_zero_omega():
    d = StampedLosRateDifferentiator(ema_alpha=0.5)
    n = _los(0.3)                                # fixed bearing, never rotates
    t = 0
    for _ in range(20):
        d.update(n, t)
        t += DT_NS
    assert np.linalg.norm(d.omega) < 1e-9, d.omega


def test_dropout_holds_last_rate():
    """After detections stop, repeated calls with the last stamp hold Ω (the
    node then applies coast_decay / declares loss separately)."""
    d = StampedLosRateDifferentiator(ema_alpha=0.7)
    t = 0
    for _ in range(40):
        d.update(_los(OMEGA_TRUE * (t * 1e-9)), t)
        t += DT_NS
    held = d.omega.copy()
    last_stamp = t - DT_NS
    for _ in range(25):                          # no new detections
        out = d.update(_los(OMEGA_TRUE * (last_stamp * 1e-9)), last_stamp)
        assert np.array_equal(out, held)
    assert np.array_equal(d.omega, held)


def test_reset_clears_state_and_no_cross_source_diff():
    """Source/mode switch: reset() zeroes the rate and drops the baseline, so
    the first sample of the new source seeds without a spurious cross-source
    finite difference."""
    d = StampedLosRateDifferentiator(ema_alpha=0.7)
    t = 0
    for _ in range(20):
        d.update(_los(OMEGA_TRUE * (t * 1e-9)), t)
        t += DT_NS
    assert np.linalg.norm(d.omega) > 0.05
    d.reset()
    assert np.linalg.norm(d.omega) < 1e-12
    # A single sample right after reset differentiates against nothing -> 0.
    out = d.update(_los(2.0), t + 5 * DT_NS)
    assert np.linalg.norm(out) < 1e-12


def test_out_of_order_stamp_is_ignored():
    d = StampedLosRateDifferentiator(ema_alpha=0.7)
    t = 0
    for _ in range(10):
        d.update(_los(OMEGA_TRUE * (t * 1e-9)), t)
        t += DT_NS
    good = d.omega.copy()
    # A stale (earlier) stamp must not differentiate backward in time.
    out = d.update(_los(5.0), t - 10 * DT_NS)
    assert np.array_equal(out, good)


def test_non_finite_input_is_safe():
    d = StampedLosRateDifferentiator(ema_alpha=0.7)
    t = 0
    for _ in range(10):
        d.update(_los(OMEGA_TRUE * (t * 1e-9)), t)
        t += DT_NS
    good = d.omega.copy()
    out = d.update(np.array([np.nan, 0.0, 0.0]), t)      # garbage detection
    assert np.array_equal(out, good)
    out = d.update(np.zeros(3), t)                        # degenerate zero LOS
    assert np.array_equal(out, good)


def test_equivalence_to_legacy_bearing_pn_inline_diff():
    """The refactor must not change the live-validated bearing_pn result. Run the
    OLD inline differentiation (ticket 011 pn_guidance_node) and the new shared
    class on the same 25 Hz stamped LOS sequence -> identical Ω at every step."""
    alpha = 0.7
    d = StampedLosRateDifferentiator(ema_alpha=alpha)
    # legacy inline state
    last_n_hat = None
    last_rx = None
    omega_ema = np.zeros(3)

    rng = np.random.default_rng(3)
    t = 0
    for k in range(80):
        # a rotating LOS with a little jitter, always > 1 ms apart (25 Hz)
        theta = 0.08 * (t * 1e-9) + 0.01 * np.sin(0.7 * k)
        n = _los(theta) + 0.002 * rng.normal(size=3)
        n = n / np.linalg.norm(n)
        rx = t

        # --- legacy inline (verbatim from the ticket-011 node) ---
        if rx != last_rx:
            if last_n_hat is not None and last_rx is not None:
                dts = (rx - last_rx) * 1e-9
                if dts > 1e-3:
                    omega_raw = np.cross(n, (n - last_n_hat) / dts)
                    omega_ema = alpha * omega_ema + (1.0 - alpha) * omega_raw
            last_n_hat = n
            last_rx = rx

        # --- new shared class ---
        omega_new = d.update(n, rx)

        assert np.allclose(omega_new, omega_ema, atol=1e-15), (k, omega_new, omega_ema)
        t += DT_NS


def test_coast_decay_profile():
    assert coast_decay(0.0, 0.3, 0.8) == 1.0
    assert coast_decay(0.3, 0.3, 0.8) == 1.0            # inclusive at timeout
    assert abs(coast_decay(0.55, 0.3, 0.8) - 0.5) < 1e-9  # linear midpoint
    assert coast_decay(0.8, 0.3, 0.8) == 0.0            # at loss
    assert coast_decay(2.0, 0.3, 0.8) == 0.0            # past loss
    # monot, bounded
    prev = 1.0
    for a in np.linspace(0.3, 0.8, 20):
        f = coast_decay(float(a), 0.3, 0.8)
        assert 0.0 <= f <= 1.0 and f <= prev + 1e-12
        prev = f
