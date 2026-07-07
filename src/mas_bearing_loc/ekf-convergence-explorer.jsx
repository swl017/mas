import { useState, useEffect, useRef, useCallback, useMemo } from "react";

const TAU = Math.PI * 2;
function lerp(a, b, t) { return a + (b - a) * t; }
function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
function vecLen(x, y) { return Math.sqrt(x * x + y * y); }
function normalizeAngle(a) { while (a > Math.PI) a -= TAU; while (a < -Math.PI) a += TAU; return a; }
function gaussNoise(sigma) { const u1 = Math.random(), u2 = Math.random(); return sigma * Math.sqrt(-2 * Math.log(u1)) * Math.cos(TAU * u2); }

// ---- Drawing Primitives ----
function drawGrid(ctx, w, h, spacing = 40) {
  ctx.strokeStyle = "rgba(100,140,180,0.06)";
  ctx.lineWidth = 1;
  for (let x = 0; x < w; x += spacing) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke(); }
  for (let y = 0; y < h; y += spacing) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
}
function drawArrow(ctx, x1, y1, x2, y2, color, w = 1.5) {
  const dx = x2 - x1, dy = y2 - y1, len = Math.sqrt(dx * dx + dy * dy);
  if (len < 1) return;
  const ux = dx / len, uy = dy / len;
  ctx.strokeStyle = color; ctx.lineWidth = w;
  ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  ctx.fillStyle = color; ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - ux * 7 + uy * 3.5, y2 - uy * 7 - ux * 3.5);
  ctx.lineTo(x2 - ux * 7 - uy * 3.5, y2 - uy * 7 + ux * 3.5);
  ctx.closePath(); ctx.fill();
}
function drawDrone(ctx, x, y, angle, color, label, scale = 1) {
  ctx.save(); ctx.translate(x, y); ctx.rotate(angle);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(10 * scale, 0); ctx.lineTo(-7 * scale, -6 * scale); ctx.lineTo(-3 * scale, 0); ctx.lineTo(-7 * scale, 6 * scale);
  ctx.closePath(); ctx.fill();
  ctx.restore();
  if (label) {
    ctx.fillStyle = color; ctx.font = `bold ${9 * scale}px 'JetBrains Mono', monospace`;
    ctx.textAlign = "center"; ctx.fillText(label, x, y - 13 * scale);
  }
}
function drawTarget(ctx, x, y, color, size = 6) {
  ctx.fillStyle = color; ctx.beginPath(); ctx.arc(x, y, size, 0, TAU); ctx.fill();
  ctx.strokeStyle = "rgba(0,0,0,0.5)"; ctx.lineWidth = 1; ctx.stroke();
  ctx.strokeStyle = color; ctx.lineWidth = 0.8;
  ctx.beginPath(); ctx.arc(x, y, size + 4, 0, TAU); ctx.stroke();
}
function drawLOS(ctx, x1, y1, x2, y2, color, alpha = 0.4) {
  ctx.save(); ctx.globalAlpha = alpha; ctx.strokeStyle = color;
  ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  ctx.restore();
}
function drawEllipse(ctx, cx, cy, rx, ry, angle, color, alpha = 0.12) {
  ctx.save(); ctx.translate(cx, cy); ctx.rotate(angle);
  ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.setLineDash([3, 3]); ctx.globalAlpha = 0.5;
  ctx.beginPath(); ctx.ellipse(0, 0, Math.max(rx, 2), Math.max(ry, 2), 0, 0, TAU); ctx.stroke();
  ctx.globalAlpha = alpha; ctx.fillStyle = color; ctx.fill();
  ctx.restore();
}

// ---- Simulation Engine ----
function runSim(config) {
  const { maxSteps, observers, targetPath, targetManeuver, noiseStd } = config;
  const data = { obs: [], int: [], est: [], covTrace: [], posErr: [], velErr: [], losAngles: [] };

  // True intruder state
  const getTarget = (k) => targetPath(k);

  // EKF state init: offset from true
  let ex, ey, evx, evy;
  const t0 = getTarget(0);
  ex = t0.x + 80 + gaussNoise(30);
  ey = t0.y - 50 + gaussNoise(30);
  evx = t0.vx + gaussNoise(1.5);
  evy = t0.vy + gaussNoise(1.5);

  // Covariance (simplified scalar traces)
  let Pxx = 8000, Pyy = 6000, Pvxvx = 20, Pvyvy = 20;
  const Q = { xx: 0.5, yy: 0.5, vv: 0.1 }; // process noise

  for (let k = 0; k <= maxSteps; k++) {
    const tgt = getTarget(k);
    const obsPositions = observers.map(o => o(k));

    // Predict
    ex += evx;
    ey += evy;
    Pxx += Pvxvx + Q.xx;
    Pyy += Pvyvy + Q.yy;
    Pvxvx += Q.vv;
    Pvyvy += Q.vv;

    // Update from each observer's bearing measurement
    let losAngs = [];
    for (const obs of obsPositions) {
      const trueAngle = Math.atan2(tgt.y - obs.y, tgt.x - obs.x) + gaussNoise(noiseStd);
      const estAngle = Math.atan2(ey - obs.y, ex - obs.x);
      const residual = normalizeAngle(trueAngle - estAngle);
      losAngs.push(trueAngle);

      const dist2 = (ex - obs.x) ** 2 + (ey - obs.y) ** 2;
      const dist = Math.sqrt(dist2);

      // Jacobian of atan2 measurement w.r.t. (x,y)
      const Hx = -(ey - obs.y) / dist2;
      const Hy = (ex - obs.x) / dist2;

      const R = (noiseStd * noiseStd) + 0.001; // measurement noise
      const S = Hx * Hx * Pxx + Hy * Hy * Pyy + R;

      const Kx = Pxx * Hx / S;
      const Ky = Pyy * Hy / S;
      const Kvx = Pvxvx * Hx * 0.3 / S; // weak coupling to velocity
      const Kvy = Pvyvy * Hy * 0.3 / S;

      ex += Kx * residual;
      ey += Ky * residual;
      evx += Kvx * residual;
      evy += Kvy * residual;

      Pxx *= (1 - Kx * Hx);
      Pyy *= (1 - Ky * Hy);
      Pvxvx *= 0.995;
      Pvyvy *= 0.995;
    }

    Pxx = Math.max(Pxx, 2);
    Pyy = Math.max(Pyy, 2);

    const posE = vecLen(ex - tgt.x, ey - tgt.y);
    const velE = vecLen(evx - tgt.vx, evy - tgt.vy);

    data.obs.push(obsPositions);
    data.int.push(tgt);
    data.est.push({ x: ex, y: ey, vx: evx, vy: evy });
    data.covTrace.push(Math.sqrt(Pxx + Pyy));
    data.posErr.push(posE);
    data.velErr.push(velE);
    data.losAngles.push(losAngs);
  }
  return data;
}

// ---- Scenario Definitions ----
const SCENARIOS = [
  {
    id: "weave_single",
    label: "Single Observer · Weave",
    desc: "One drone executes an S-weave maneuver. The lateral displacement creates bearing diversity, but convergence is slow — a single viewpoint limits information gain, especially at range.",
    colors: { obs: ["#4fc3f7"], cov: "#66bb6a" },
    config: {
      maxSteps: 120,
      noiseStd: 0.012,
      observers: [
        (k) => ({
          x: 60 + k * 3.5 + Math.sin(k * 0.1) * 50,
          y: 320 - k * 0.8 + Math.cos(k * 0.07) * 20
        })
      ],
      targetPath: (k) => ({ x: 500 - k * 2.5, y: 60 + k * 2.2, vx: -2.5, vy: 2.2 }),
    }
  },
  {
    id: "orbit_single",
    label: "Single Observer · Orbit",
    desc: "One drone orbits around the target area. This creates maximal bearing diversity from a single platform — the gold standard for single-observer bearing-only tracking. Compare the convergence speed to the simple weave.",
    colors: { obs: ["#4fc3f7"], cov: "#66bb6a" },
    config: {
      maxSteps: 120,
      noiseStd: 0.012,
      observers: [
        (k) => {
          const cx = 300 - k * 1.0, cy = 200 + k * 1.0;
          return {
            x: cx + Math.cos(k * 0.08) * 160,
            y: cy + Math.sin(k * 0.08) * 120
          };
        }
      ],
      targetPath: (k) => ({ x: 420 - k * 2.0, y: 80 + k * 2.0, vx: -2.0, vy: 2.0 }),
    }
  },
  {
    id: "multi_2",
    label: "2 Observers · Pincer",
    desc: "Two drones approach from different angles creating a pincer geometry. Each frame provides two bearing lines that intersect — effectively achieving instantaneous triangulation. Notice how dramatically faster the covariance collapses vs. single observer.",
    colors: { obs: ["#4fc3f7", "#ce93d8"], cov: "#66bb6a" },
    config: {
      maxSteps: 120,
      noiseStd: 0.012,
      observers: [
        (k) => ({
          x: 50 + k * 3.2 + Math.sin(k * 0.12) * 30,
          y: 340 - k * 1.2
        }),
        (k) => ({
          x: 580 - k * 2.0 + Math.sin(k * 0.1) * 25,
          y: 330 - k * 0.5 + Math.cos(k * 0.09) * 20
        })
      ],
      targetPath: (k) => ({ x: 380 - k * 1.8, y: 70 + k * 2.0, vx: -1.8, vy: 2.0 }),
    }
  },
  {
    id: "multi_3",
    label: "3 Observers · Surround",
    desc: "Three drones approach from spread-out directions — near-optimal geometry for bearing-only localization. This mimics your MAPPO triangulation setup: multiple viewpoints make the problem well-conditioned. The covariance shrinks almost immediately.",
    colors: { obs: ["#4fc3f7", "#ce93d8", "#ffb74d"], cov: "#66bb6a" },
    config: {
      maxSteps: 120,
      noiseStd: 0.012,
      observers: [
        (k) => ({
          x: 50 + k * 3.0 + Math.sin(k * 0.11) * 25,
          y: 300 - k * 1.0
        }),
        (k) => ({
          x: 580 - k * 1.8,
          y: 320 - k * 0.8 + Math.sin(k * 0.09) * 20
        }),
        (k) => ({
          x: 320 + Math.sin(k * 0.07) * 30,
          y: 370 - k * 2.0
        })
      ],
      targetPath: (k) => ({ x: 350 - k * 1.5, y: 60 + k * 2.0, vx: -1.5, vy: 2.0 }),
    }
  },
  {
    id: "maneuver_target",
    label: "Maneuvering Target",
    desc: "The target executes a sharp turn mid-flight. A constant-velocity EKF model struggles here — the estimate diverges temporarily before re-converging. This is the core challenge: your MAPPO-based approach can learn to anticipate and react to such maneuvers, while the EKF is purely reactive.",
    colors: { obs: ["#4fc3f7", "#ce93d8"], cov: "#66bb6a" },
    config: {
      maxSteps: 120,
      noiseStd: 0.012,
      observers: [
        (k) => ({
          x: 60 + k * 3.0 + Math.sin(k * 0.12) * 35,
          y: 320 - k * 1.0
        }),
        (k) => ({
          x: 560 - k * 1.5,
          y: 300 - k * 0.6 + Math.sin(k * 0.1) * 25
        })
      ],
      targetPath: (k) => {
        if (k < 50) {
          return { x: 420 - k * 2.5, y: 70 + k * 1.8, vx: -2.5, vy: 1.8 };
        } else {
          const dk = k - 50;
          const turnRate = 0.04;
          const baseAngle = Math.atan2(1.8, -2.5);
          const angle = baseAngle + dk * turnRate;
          const speed = 3.0;
          const px = 420 - 50 * 2.5 + dk * speed * Math.cos(angle);
          const py = 70 + 50 * 1.8 + dk * speed * Math.sin(angle);
          return { x: px, y: py, vx: speed * Math.cos(angle), vy: speed * Math.sin(angle) };
        }
      },
    }
  },
  {
    id: "noisy",
    label: "High Noise · Stress Test",
    desc: "Bearing measurements corrupted with 5× higher noise (simulating poor detection, vibration, or tiny bounding boxes). Two observers still converge but slowly and with residual error. This highlights why your covariance-based reward shaping matters — the filter's uncertainty directly reflects detection quality.",
    colors: { obs: ["#4fc3f7", "#ce93d8"], cov: "#66bb6a" },
    config: {
      maxSteps: 120,
      noiseStd: 0.06,
      observers: [
        (k) => ({
          x: 50 + k * 3.2 + Math.sin(k * 0.12) * 30,
          y: 340 - k * 1.2
        }),
        (k) => ({
          x: 580 - k * 2.0 + Math.sin(k * 0.1) * 25,
          y: 330 - k * 0.5 + Math.cos(k * 0.09) * 20
        })
      ],
      targetPath: (k) => ({ x: 380 - k * 1.8, y: 70 + k * 2.0, vx: -1.8, vy: 2.0 }),
    }
  },
];

// ---- Chart Drawing ----
function drawChart(ctx, W, H, datasets, step, maxSteps, yMax, yLabel, legendItems) {
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = "#080e16"; ctx.fillRect(0, 0, W, H);

  const pad = { l: 48, r: 12, t: 22, b: 24 };
  const gw = W - pad.l - pad.r, gh = H - pad.t - pad.b;

  // Axes
  ctx.strokeStyle = "rgba(100,140,180,0.15)"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(pad.l, pad.t); ctx.lineTo(pad.l, pad.t + gh); ctx.lineTo(pad.l + gw, pad.t + gh); ctx.stroke();

  // Y gridlines
  const yTicks = 4;
  for (let i = 1; i <= yTicks; i++) {
    const y = pad.t + gh - (i / yTicks) * gh;
    ctx.strokeStyle = "rgba(100,140,180,0.06)";
    ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(pad.l + gw, y); ctx.stroke();
    ctx.fillStyle = "#455a64"; ctx.font = "9px 'JetBrains Mono', monospace"; ctx.textAlign = "right";
    ctx.fillText((yMax * i / yTicks).toFixed(0), pad.l - 4, y + 3);
  }

  ctx.fillStyle = "#455a64"; ctx.font = "9px 'JetBrains Mono', monospace"; ctx.textAlign = "center";
  ctx.fillText("step →", pad.l + gw / 2, H - 4);

  // Datasets
  datasets.forEach((ds, di) => {
    ctx.strokeStyle = ds.color; ctx.lineWidth = 1.5; ctx.setLineDash(ds.dash || []);
    ctx.beginPath();
    for (let i = 0; i <= step; i++) {
      const x = pad.l + (i / maxSteps) * gw;
      const y = pad.t + gh - clamp(ds.data[i] / yMax, 0, 1) * gh;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }
    ctx.stroke(); ctx.setLineDash([]);
  });

  // Legend
  legendItems.forEach((item, i) => {
    const lx = pad.l + 6 + i * (gw / legendItems.length);
    ctx.fillStyle = item.color; ctx.font = "9px 'JetBrains Mono', monospace"; ctx.textAlign = "left";
    ctx.fillRect(lx, pad.t + 3, 12, 2);
    ctx.fillText(item.label, lx + 16, pad.t + 8);
  });
}

// ---- Main Visualization for a Scenario ----
function ScenarioViz({ scenario, simData }) {
  const canvasRef = useRef(null);
  const chartRef = useRef(null);
  const velChartRef = useRef(null);
  const [step, setStep] = useState(0);
  const [playing, setPlaying] = useState(false);
  const maxSteps = scenario.config.maxSteps;

  useEffect(() => {
    if (playing) {
      const interval = setInterval(() => {
        setStep(s => {
          if (s >= maxSteps) { setPlaying(false); return maxSteps; }
          return s + 1;
        });
      }, 50);
      return () => clearInterval(interval);
    }
  }, [playing, maxSteps]);

  // Reset step when scenario changes
  useEffect(() => { setStep(0); setPlaying(false); }, [scenario.id]);

  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    const chart = chartRef.current;
    const velChart = velChartRef.current;
    if (!canvas || !chart || !velChart) return;
    const ctx = canvas.getContext("2d");
    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0, 0, W, H);
    drawGrid(ctx, W, H);
    const d = simData;

    // Observer trails
    scenario.colors.obs.forEach((col, oi) => {
      ctx.strokeStyle = col.replace(")", ",0.15)").replace("rgb", "rgba");
      ctx.lineWidth = 1.2; ctx.setLineDash([]); ctx.beginPath();
      for (let i = 0; i <= step; i++) {
        const o = d.obs[i][oi];
        if (!o) continue;
        i === 0 ? ctx.moveTo(o.x, o.y) : ctx.lineTo(o.x, o.y);
      }
      ctx.stroke();
    });

    // Intruder trail
    ctx.strokeStyle = "rgba(255,112,67,0.18)"; ctx.lineWidth = 1.5; ctx.beginPath();
    for (let i = 0; i <= step; i++) {
      const t = d.int[i];
      i === 0 ? ctx.moveTo(t.x, t.y) : ctx.lineTo(t.x, t.y);
    }
    ctx.stroke();

    // EKF estimate trail
    ctx.strokeStyle = "rgba(102,187,106,0.3)"; ctx.lineWidth = 2; ctx.beginPath();
    for (let i = 0; i <= step; i++) {
      const e = d.est[i];
      i === 0 ? ctx.moveTo(e.x, e.y) : ctx.lineTo(e.x, e.y);
    }
    ctx.stroke();

    // Covariance ellipse
    const covR = clamp(d.covTrace[step] * 0.7, 4, 200);
    drawEllipse(ctx, d.est[step].x, d.est[step].y, covR, covR * 0.7, 0.4, scenario.colors.cov, 0.1);

    // LOS lines from each observer
    scenario.colors.obs.forEach((col, oi) => {
      const o = d.obs[step][oi];
      if (o) drawLOS(ctx, o.x, o.y, d.int[step].x, d.int[step].y, col, 0.3);
    });

    // Maneuver marker for maneuvering target
    if (scenario.id === "maneuver_target" && step >= 50) {
      const tp = d.int[50];
      ctx.save();
      ctx.strokeStyle = "#ff5252"; ctx.lineWidth = 1.5; ctx.setLineDash([3, 3]); ctx.globalAlpha = 0.6;
      ctx.beginPath(); ctx.arc(tp.x, tp.y, 14, 0, TAU); ctx.stroke();
      ctx.fillStyle = "#ff5252"; ctx.font = "bold 9px 'JetBrains Mono', monospace"; ctx.textAlign = "center";
      ctx.fillText("TURN", tp.x, tp.y - 18);
      ctx.restore();
    }

    // Draw observers
    scenario.colors.obs.forEach((col, oi) => {
      const o = d.obs[step][oi];
      if (!o) return;
      let angle = 0;
      if (step > 0 && d.obs[step - 1][oi]) {
        const prev = d.obs[step - 1][oi];
        angle = Math.atan2(o.y - prev.y, o.x - prev.x);
      }
      drawDrone(ctx, o.x, o.y, angle, col, oi === 0 ? "OBS" + (scenario.colors.obs.length > 1 ? "₁" : "") : "OBS₂");
    });

    // True target
    drawTarget(ctx, d.int[step].x, d.int[step].y, "#ff7043");

    // EKF estimate dot
    ctx.fillStyle = "#66bb6a"; ctx.beginPath(); ctx.arc(d.est[step].x, d.est[step].y, 4, 0, TAU); ctx.fill();
    ctx.strokeStyle = "#fff"; ctx.lineWidth = 0.8; ctx.stroke();

    // Legend bar
    ctx.font = "10px 'JetBrains Mono', monospace"; ctx.textAlign = "left";
    let lx = 10;
    scenario.colors.obs.forEach((col, i) => {
      ctx.fillStyle = col;
      ctx.fillText(`▸ Obs${i + 1}`, lx, H - 8);
      lx += 60;
    });
    ctx.fillStyle = "#ff7043"; ctx.fillText("● Target", lx, H - 8); lx += 70;
    ctx.fillStyle = "#66bb6a"; ctx.fillText("● EKF Est", lx, H - 8); lx += 75;

    // Metrics
    ctx.fillStyle = "#78909c"; ctx.textAlign = "right"; ctx.font = "10px 'JetBrains Mono', monospace";
    ctx.fillText(`step ${step}/${maxSteps}`, W - 8, 16);
    ctx.fillText(`pos err: ${d.posErr[step].toFixed(1)}`, W - 8, 30);
    ctx.fillText(`σ: ${d.covTrace[step].toFixed(1)}`, W - 8, 44);

    // Charts
    const maxPosErr = Math.max(80, ...d.posErr.slice(0, step + 1)) * 1.1;
    const maxVelErr = Math.max(5, ...d.velErr.slice(0, step + 1)) * 1.1;

    drawChart(chart.getContext("2d"), chart.width, chart.height,
      [
        { data: d.posErr, color: "#ef5350" },
        { data: d.covTrace, color: "#66bb6a", dash: [4, 3] },
      ],
      step, maxSteps, maxPosErr, "px",
      [
        { color: "#ef5350", label: "Position Error" },
        { color: "#66bb6a", label: "Covariance (σ)" },
      ]
    );

    drawChart(velChart.getContext("2d"), velChart.width, velChart.height,
      [
        { data: d.velErr, color: "#ffb74d" },
      ],
      step, maxSteps, maxVelErr, "px/s",
      [
        { color: "#ffb74d", label: "Velocity Error" },
      ]
    );
  }, [step, simData, scenario]);

  useEffect(() => { draw(); }, [draw]);

  return (
    <div>
      <canvas ref={canvasRef} width={640} height={380} style={{ width: "100%", height: "auto", borderRadius: 8, background: "#0d1520" }} />
      <div style={{ display: "flex", gap: 6, marginTop: 6 }}>
        <canvas ref={chartRef} width={320} height={120} style={{ flex: 1, borderRadius: 6, background: "#080e16" }} />
        <canvas ref={velChartRef} width={320} height={120} style={{ flex: 1, borderRadius: 6, background: "#080e16" }} />
      </div>
      <div style={{ marginTop: 10, display: "flex", alignItems: "center", gap: 8 }}>
        <button onClick={() => { if (step >= maxSteps) setStep(0); setPlaying(!playing); }}
          style={{ padding: "7px 16px", borderRadius: 6, cursor: "pointer", fontFamily: "'JetBrains Mono', monospace", fontSize: 11, border: "none", background: "#1a2a38", color: "#e0e0e0" }}>
          {playing ? "⏸" : "▶"}
        </button>
        <button onClick={() => { setPlaying(false); setStep(0); }}
          style={{ padding: "7px 12px", borderRadius: 6, cursor: "pointer", fontFamily: "'JetBrains Mono', monospace", fontSize: 11, border: "none", background: "#1a2332", color: "#78909c" }}>
          ↺
        </button>
        <input type="range" min={0} max={maxSteps} step={1} value={step}
          onChange={e => { setPlaying(false); setStep(parseInt(e.target.value)); }}
          style={{ flex: 1, accentColor: "#66bb6a" }} />
        <span style={{ color: "#546e7a", fontFamily: "'JetBrains Mono', monospace", fontSize: 10, minWidth: 42 }}>t={step}</span>
      </div>
    </div>
  );
}

// ---- Comparison Summary ----
function ComparisonPanel({ allData }) {
  // Compute convergence metrics
  const metrics = SCENARIOS.map((sc, i) => {
    const d = allData[i];
    if (!d) return null;
    const maxS = sc.config.maxSteps;
    const finalErr = d.posErr[maxS];
    const finalCov = d.covTrace[maxS];
    // Time to reach < 20px error
    let convStep = maxS;
    for (let k = 0; k <= maxS; k++) {
      if (d.posErr[k] < 20) { convStep = k; break; }
    }
    // Average error in last 20%
    const tail = d.posErr.slice(Math.floor(maxS * 0.8));
    const avgTail = tail.reduce((a, b) => a + b, 0) / tail.length;
    return { label: sc.label, finalErr, finalCov, convStep, avgTail, nObs: sc.config.observers.length };
  }).filter(Boolean);

  return (
    <div style={{
      background: "rgba(15,25,40,0.7)", border: "1px solid rgba(100,140,180,0.12)",
      borderRadius: 8, padding: "16px 18px", marginTop: 8
    }}>
      <div style={{ fontSize: 13, fontWeight: 600, color: "#b0bec5", marginBottom: 10, fontFamily: "'Space Grotesk', sans-serif" }}>
        Convergence Comparison
      </div>
      <div style={{ overflowX: "auto" }}>
        <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 11, fontFamily: "'JetBrains Mono', monospace" }}>
          <thead>
            <tr style={{ color: "#546e7a", borderBottom: "1px solid rgba(100,140,180,0.15)" }}>
              <th style={{ textAlign: "left", padding: "6px 8px", fontWeight: 600 }}>Scenario</th>
              <th style={{ textAlign: "center", padding: "6px 6px", fontWeight: 600 }}>#Obs</th>
              <th style={{ textAlign: "center", padding: "6px 6px", fontWeight: 600 }}>Conv. Step</th>
              <th style={{ textAlign: "center", padding: "6px 6px", fontWeight: 600 }}>Final Err</th>
              <th style={{ textAlign: "center", padding: "6px 6px", fontWeight: 600 }}>Avg Tail Err</th>
              <th style={{ textAlign: "center", padding: "6px 6px", fontWeight: 600 }}>Final σ</th>
            </tr>
          </thead>
          <tbody>
            {metrics.map((m, i) => {
              const isGood = m.convStep < 40;
              const isBad = m.convStep >= 100;
              const rowColor = isGood ? "rgba(102,187,106,0.06)" : isBad ? "rgba(239,83,80,0.06)" : "transparent";
              return (
                <tr key={i} style={{ background: rowColor, borderBottom: "1px solid rgba(100,140,180,0.06)" }}>
                  <td style={{ padding: "6px 8px", color: "#b0bec5" }}>{m.label}</td>
                  <td style={{ textAlign: "center", padding: "6px", color: "#78909c" }}>{m.nObs}</td>
                  <td style={{ textAlign: "center", padding: "6px", color: isGood ? "#66bb6a" : isBad ? "#ef5350" : "#ffb74d", fontWeight: 600 }}>
                    {m.convStep >= m.finalErr ? ">" + m.convStep : m.convStep}
                  </td>
                  <td style={{ textAlign: "center", padding: "6px", color: "#cfd8dc" }}>{m.finalErr.toFixed(1)}</td>
                  <td style={{ textAlign: "center", padding: "6px", color: "#cfd8dc" }}>{m.avgTail.toFixed(1)}</td>
                  <td style={{ textAlign: "center", padding: "6px", color: "#90a4ae" }}>{m.finalCov.toFixed(1)}</td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
      <div style={{ marginTop: 12, fontSize: 11, color: "#546e7a", lineHeight: 1.7 }}>
        <span style={{ color: "#66bb6a" }}>●</span> Conv. Step = first step where position error {"<"} 20px. Lower = faster convergence.<br />
        <span style={{ color: "#ffb74d" }}>Key takeaway:</span> Multi-observer bearing-only EKF approaches triangulation performance but remains reactive.
        Your MAPPO agents can proactively optimize geometry — the EKF only passively benefits from whatever geometry it gets.
      </div>
    </div>
  );
}

// =================== Main ===================
export default function EKFConvergenceExplorer() {
  const [activeIdx, setActiveIdx] = useState(0);
  const [showComparison, setShowComparison] = useState(false);
  const [seed, setSeed] = useState(42);

  // Run all simulations (memoized per seed)
  const allData = useMemo(() => {
    // Seed the random with a simple approach
    let _s = seed;
    const origRandom = Math.random;
    // Simple seeded random for reproducibility
    const seededRandom = () => { _s = (_s * 16807 + 0) % 2147483647; return (_s & 0x7fffffff) / 0x7fffffff; };
    Math.random = seededRandom;
    const results = SCENARIOS.map(sc => runSim(sc.config));
    Math.random = origRandom;
    return results;
  }, [seed]);

  const scenario = SCENARIOS[activeIdx];

  return (
    <div style={{
      fontFamily: "'JetBrains Mono', 'Fira Code', monospace",
      background: "linear-gradient(160deg, #060c16 0%, #0c1824 50%, #081018 100%)",
      minHeight: "100vh", color: "#cfd8dc", padding: "20px 16px"
    }}>
      <link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@300;400;600;700&family=Space+Grotesk:wght@400;600;700&display=swap" rel="stylesheet" />

      <div style={{ maxWidth: 680, margin: "0 auto" }}>
        {/* Header */}
        <div style={{ marginBottom: 16 }}>
          <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 4 }}>
            <div style={{ width: 7, height: 7, borderRadius: "50%", background: "#66bb6a", boxShadow: "0 0 6px #66bb6a" }} />
            <span style={{ fontSize: 10, color: "#455a64", letterSpacing: 2, textTransform: "uppercase" }}>EKF Bearing-Only Tracking</span>
          </div>
          <h1 style={{ fontFamily: "'Space Grotesk', sans-serif", fontSize: 22, fontWeight: 700, color: "#e0e0e0", margin: "4px 0 2px" }}>
            Convergence Explorer
          </h1>
          <p style={{ fontSize: 11, color: "#455a64", margin: 0 }}>
            Compare single vs. multi-observer EKF — and see where learned coordination (MAPPO) wins
          </p>
        </div>

        {/* Scenario selector */}
        <div style={{ display: "flex", flexWrap: "wrap", gap: 4, marginBottom: 12 }}>
          {SCENARIOS.map((sc, i) => (
            <button key={sc.id} onClick={() => setActiveIdx(i)}
              style={{
                padding: "7px 10px", borderRadius: 5, cursor: "pointer",
                fontFamily: "'JetBrains Mono', monospace", fontSize: 10,
                border: activeIdx === i ? "1px solid rgba(102,187,106,0.5)" : "1px solid rgba(100,140,180,0.1)",
                background: activeIdx === i ? "rgba(102,187,106,0.1)" : "rgba(13,21,32,0.6)",
                color: activeIdx === i ? "#a5d6a7" : "#607d8b",
                transition: "all 0.15s", whiteSpace: "nowrap"
              }}>
              {sc.label}
            </button>
          ))}
        </div>

        {/* Description */}
        <div style={{
          background: "rgba(18,28,42,0.6)", border: "1px solid rgba(100,140,180,0.08)",
          borderRadius: 8, padding: "12px 16px", marginBottom: 12
        }}>
          <div style={{ fontSize: 12, color: "#78909c", lineHeight: 1.65 }}>{scenario.desc}</div>
        </div>

        {/* Visualization */}
        <ScenarioViz scenario={scenario} simData={allData[activeIdx]} />

        {/* Re-roll and Compare buttons */}
        <div style={{ display: "flex", gap: 8, marginTop: 12 }}>
          <button onClick={() => setSeed(s => s + 1)}
            style={{
              padding: "8px 16px", borderRadius: 6, cursor: "pointer",
              fontFamily: "'JetBrains Mono', monospace", fontSize: 11, border: "none",
              background: "#1a2a38", color: "#90a4ae"
            }}>
            🎲 Re-roll noise
          </button>
          <button onClick={() => setShowComparison(!showComparison)}
            style={{
              padding: "8px 16px", borderRadius: 6, cursor: "pointer",
              fontFamily: "'JetBrains Mono', monospace", fontSize: 11, border: "none",
              background: showComparison ? "rgba(102,187,106,0.15)" : "#1a2a38",
              color: showComparison ? "#a5d6a7" : "#90a4ae",
              border: showComparison ? "1px solid rgba(102,187,106,0.3)" : "1px solid transparent"
            }}>
            {showComparison ? "▾ Hide" : "▸ Show"} Comparison Table
          </button>
        </div>

        {/* Comparison */}
        {showComparison && <ComparisonPanel allData={allData} />}

        {/* Insight panel */}
        <div style={{
          background: "rgba(15,25,40,0.5)", border: "1px solid rgba(255,183,77,0.12)",
          borderRadius: 8, padding: "14px 16px", marginTop: 14
        }}>
          <div style={{ fontSize: 12, fontWeight: 600, color: "#ffb74d", marginBottom: 8, fontFamily: "'Space Grotesk', sans-serif" }}>
            EKF vs. Active Triangulation (MAPPO)
          </div>
          <div style={{ fontSize: 11, color: "#78909c", lineHeight: 1.75 }}>
            <strong style={{ color: "#b0bec5" }}>What EKF gives you:</strong> Principled uncertainty quantification (covariance), works with any number of observers, well-understood convergence guarantees under linear-Gaussian assumptions.<br /><br />
            <strong style={{ color: "#b0bec5" }}>Where it falls short:</strong> Purely reactive — it cannot plan trajectories to maximize information gain. The constant-velocity assumption breaks on maneuvering targets. Convergence depends entirely on geometry it doesn't control.<br /><br />
            <strong style={{ color: "#b0bec5" }}>Your MAPPO advantage:</strong> Learned policies proactively optimize observer geometry for the covariance objective. The RNN hidden state can implicitly model target maneuvers. The reward-shaping from det(P) or tr(P) directly drives the behavior EKF passively hopes for.
            The EKF is the <em style={{ color: "#a5d6a7" }}>estimation backend</em>; MAPPO is the <em style={{ color: "#a5d6a7" }}>planning frontend</em>. They're complementary, not competing.
          </div>
        </div>
      </div>
    </div>
  );
}
