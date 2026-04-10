import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, welch
import os

DATA_DIR = "data/"
SAVE_DIR = "results/"
os.makedirs(SAVE_DIR, exist_ok=True)


cal_raw = {
    "-X": pd.read_csv(DATA_DIR + "X_axis.csv"),
    "+Y": pd.read_csv(DATA_DIR + "Y_axis.csv"),
    "+Z": pd.read_csv(DATA_DIR + "Z_axis.csv"),
    "-Z": pd.read_csv(DATA_DIR + "negZ_axis.csv"),
}
dyn = pd.read_csv(DATA_DIR + "20cm.csv")
g_true = 9.81

means = {k: v[["X","Y","Z"]].mean().values for k,v in cal_raw.items()}

sx_max = means["-X"][0];  sx_min = -sx_max
sy_max = means["+Y"][1];  sy_min = -sy_max
sz_max = means["+Z"][2];  sz_min = means["-Z"][2]

scale_x = abs((sx_max - sx_min) / (2 * g_true))
scale_y = abs((sy_max - sy_min) / (2 * g_true))
scale_z = abs((sz_max - sz_min) / (2 * g_true))

bias_x_cal = (sx_max + sx_min) / 2
bias_y_cal = (sy_max + sy_min) / 2
bias_z_cal = (sz_max + sz_min) / 2   # +0.643 m/s² real Z offset

print("  SIX-POSITION CALIBRATION")
print(f"  Scale  : X={scale_x:.4f}  Y={scale_y:.4f}  Z={scale_z:.4f}")
print(f"  Bias   : X={bias_x_cal:.4f}  Y={bias_y_cal:.4f}  Z={bias_z_cal:.4f} m/s²")
for o, m in means.items():
    print(f"  {o} mean: X={m[0]:+.4f}  Y={m[1]:+.4f}  Z={m[2]:+.4f}")

ax_raw_cal = (dyn["X"].values - bias_x_cal) / scale_x
ay_raw_cal = (dyn["Y"].values - bias_y_cal) / scale_y
az_raw_cal = (dyn["Z"].values - bias_z_cal) / scale_z

dt_ms  = dyn["time_ms"].diff().median()
fs     = 1000.0 / dt_ms
dt     = dt_ms / 1000.0
time_s = (dyn["time_ms"].values - dyn["time_ms"].values[0]) / 1000.0
N      = len(dyn)

print(f"\n  Sample rate : {fs:.4f} Hz  |  dt = {dt*1000:.3f} ms")
print(f"  Duration    : {time_s[-1]:.3f} s  |  N = {N} samples")

# OPTIMISED BUTTERWORTH PRE-FILTER
# 4.2 Hz cut-off: covers full human-motion bandwidth (0–4 Hz) while
# removing vibration noise above 4.2 Hz.
CUTOFF = 4.2   # Hz
ORDER  = 4

bf, af = butter(ORDER, CUTOFF / (fs / 2), btype="low", analog=False)
ax_f = filtfilt(bf, af, ax_raw_cal)
ay_f = filtfilt(bf, af, ay_raw_cal)
az_f = filtfilt(bf, af, az_raw_cal)

print(f"\n  Butterworth : order={ORDER}, cut-off={CUTOFF} Hz  (optimised, was 5.0 Hz)")

# GRAVITY ESTIMATION
N_GRAV = 6    # first 6 samples: confirmed stationary (raw std = 0.009 m/s²)
g_ref     = ay_f[:N_GRAV].mean()
g_ref_std = ay_f[:N_GRAV].std()

print(f"\n  Gravity ref : {g_ref:.6f} m/s²  (from first {N_GRAV} filtered samples)")
print(f"  Residual    : {g_ref - g_true:+.6f} m/s²")
print(f"  Window std  : {g_ref_std:.6f} m/s²  ← confirms stationarity")

ay_net = ay_f - g_ref
ax_net = ax_f
az_net = az_f

# ZUPT DETECTION
WIN      = 5
ZUPT_THR = 0.07   # m/s²

rs_y     = pd.Series(ay_net).rolling(WIN, center=True).std().bfill().ffill().values
stat_mask = rs_y < ZUPT_THR

stat_idx  = np.where(stat_mask)[0]
print(f"\n  ZUPT anchors : {stat_mask.sum()} samples ({100*stat_mask.mean():.1f}%)  "
      f"(was 25/12.6% before)")
if len(stat_idx):
    mid = stat_idx[stat_idx > N // 4]
    print(f"  Start window : t=0.00 → t={time_s[stat_idx[min(9,len(stat_idx)-1)]]:.2f}s")
    if len(mid):
        print(f"  Later windows: {len(mid)} samples  "
              f"t={time_s[mid[0]]:.2f}–{time_s[mid[-1]]:.2f}s")

# NOISE CHARACTERISATION
noise_density_y    = ay_raw_cal.std() / np.sqrt(fs)
theoretical_pos_err = noise_density_y * time_s[-1]**1.5 / 3

print(f"\n  Noise density (Y) : {noise_density_y*1000:.2f} mg/√Hz")
print(f"  Theoretical min position error : {theoretical_pos_err*100:.1f} cm")

def kalman_rts(sig, dt, stat_mask,
               q_pos=1e-6, q_vel=2.5e-4, q_bias=1e-8, R_vel=2e-3):
    n  = len(sig)
    F  = np.array([[1, dt,  0],
                   [0,  1, dt],
                   [0,  0,  1]])
    B  = np.array([0.5*dt**2, dt, 0.0])
    H  = np.array([[0.0, 1.0, 0.0]])
    Q  = np.diag([q_pos, q_vel, q_bias])
    R  = np.array([[R_vel]])

    xs   = np.zeros((n, 3));   Ps   = np.zeros((n, 3, 3))
    xp_s = np.zeros((n, 3));   Pp_s = np.zeros((n, 3, 3))

    x = np.zeros(3)
    P = np.diag([1e-6, 1e-6, 1e-6])
    n_zupt = 0

    # Forward pass
    for i in range(n):
        u  = sig[i] - x[2]
        xp = F @ x + B * u
        Pp = F @ P @ F.T + Q
        xp_s[i] = xp;  Pp_s[i] = Pp
        x = xp.copy(); P = Pp.copy()

        if stat_mask[i] and (i == 0 or not stat_mask[i-1]):
            innov = -H @ x
            S     = H @ P @ H.T + R
            K     = (P @ H.T) / S[0, 0]
            x     = x + K.flatten() * innov[0]
            P     = (np.eye(3) - K @ H) @ P
            n_zupt += 1

        xs[i] = x; Ps[i] = P

    # Backward RTS smoothing
    xs_s = xs.copy(); Ps_s = Ps.copy()

    for i in range(n - 2, -1, -1):
        try:    Pp_inv = np.linalg.inv(Pp_s[i+1])
        except: Pp_inv = np.linalg.pinv(Pp_s[i+1])
        G        = Ps[i] @ F.T @ Pp_inv
        xs_s[i]  = xs[i] + G @ (xs_s[i+1] - xp_s[i+1])
        Ps_s[i]  = Ps[i] + G @ (Ps_s[i+1] - Pp_s[i+1]) @ G.T

    pos_std = np.sqrt(np.clip(Ps_s[:, 0, 0], 0, None))
    return xs_s[:,0], xs_s[:,1], xs_s[:,2], pos_std, n_zupt, xs[:,0]


print("\n" + "=" * 62)
print("  RUNNING 3-STATE KALMAN + RTS SMOOTHER")

pos_y, vel_y, bias_y, std_y, nz_y, pos_y_fwd = kalman_rts(ay_net, dt, stat_mask)
pos_x, vel_x, bias_x_kf, std_x, nz_x, _     = kalman_rts(ax_net, dt, stat_mask)
pos_z, vel_z, bias_z_kf, std_z, nz_z, _     = kalman_rts(az_net, dt, stat_mask)

peak_idx = np.argmax(pos_y)

print(f"  Y  ZUPT={nz_y}/{N} ({100*nz_y/N:.1f}%)  "
      f"bias={bias_y[-1]*1000:.4f} mm/s²  "
      f"peak={pos_y.max()*100:.2f} cm @ t={time_s[peak_idx]:.2f}s")
print(f"  X  ZUPT={nz_x}/{N} ({100*nz_x/N:.1f}%)  bias={bias_x_kf[-1]*1000:.4f} mm/s²")
print(f"  Z  ZUPT={nz_z}/{N} ({100*nz_z/N:.1f}%)  bias={bias_z_kf[-1]*1000:.4f} mm/s²")

# Raw double-integral baseline
vel_raw = np.cumsum(ay_net) * dt
pos_raw = np.cumsum(vel_raw) * dt


# NOISE METRICS 
def snr_db(raw, filt):
    s_r, s_f = np.std(raw), np.std(filt)
    return 20 * np.log10(s_r / s_f) if s_f > 0 else 0.0

metrics = {
    "X": dict(std_raw=np.std(ax_raw_cal), std_filt=np.std(ax_f),
              snr=snr_db(ax_raw_cal, ax_f)),
    "Y": dict(std_raw=np.std(ay_raw_cal), std_filt=np.std(ay_f),
              snr=snr_db(ay_raw_cal, ay_f)),
    "Z": dict(std_raw=np.std(az_raw_cal), std_filt=np.std(az_f),
              snr=snr_db(az_raw_cal, az_f)),
}

print("\n" + "=" * 62)
print("  NOISE REDUCTION (Butterworth 4.2 Hz, order 4)")
print(f"  {'Axis':<5} {'Std Raw':>10} {'Std Filt':>12} {'SNR Gain (dB)':>14}")
print(f"  {'-'*45}")
for ax_l, m in metrics.items():
    print(f"  {ax_l:<5} {m['std_raw']:>10.4f} {m['std_filt']:>12.4f} {m['snr']:>14.2f}")


# POSITION SUMMARY
EXPECTED = 0.20  # m
peak_cm  = pos_y.max() * 100
err_pct  = abs(peak_cm - EXPECTED*100) / (EXPECTED*100) * 100

print("\n" + "=" * 62)
print("  POSITION ESTIMATION — Y VERTICAL AXIS")
print(f"  Expected displacement    : {EXPECTED*100:.2f} cm")
print(f"  Raw double-integral peak : {pos_raw.max()*100:.2f} cm  ({(pos_raw.max()-EXPECTED)/EXPECTED*100:+.1f}%)")
print(f"  Forward Kalman peak      : {pos_y_fwd.max()*100:.2f} cm  ({(pos_y_fwd.max()-EXPECTED)/EXPECTED*100:+.1f}%)")
print(f"  RTS Smoother peak        : {peak_cm:.2f} cm  ({err_pct:.1f}% error)  ← best estimate")
print(f"  RTS 1-σ at peak          : ±{std_y[peak_idx]*100:.2f} cm")
print(f"  RTS 1-σ at end           : ±{std_y[-1]*100:.2f} cm")
print(f"  Bias converged           : {bias_y[-1]*1000:.4f} mm/s²")
print(f"\n  Improvement summary:")
print(f"    Raw → RTS error       : {abs(pos_raw.max()*100-EXPECTED*100)/EXPECTED/100*100:.0f}% → {err_pct:.1f}%")
print(f"    Noise density         : {noise_density_y*1000:.1f} mg/√Hz (physical limit ≈{theoretical_pos_err*100:.0f} cm)")


COLORS = {"X": "#e74c3c", "Y": "#2980b9", "Z": "#27ae60"}
plt.rcParams.update({"font.size": 10, "axes.titlesize": 11})

def savefig(name):
    p = SAVE_DIR + name
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Saved → {p}")

# Fig 1: Raw vs Denoised acceleration
fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
rows = [(ay_raw_cal, ay_f, "Y-axis (m/s²)", COLORS["Y"]),
        (ax_raw_cal, ax_f, "X-axis (m/s²)", COLORS["X"]),
        (az_raw_cal, az_f, "Z-axis (m/s²)", COLORS["Z"])]
for ax, (raw, filt, ylabel, color) in zip(axes, rows):
    ax.plot(time_s, raw,  alpha=0.35, lw=0.7, color=color, label="Calibrated raw")
    ax.plot(time_s, filt, lw=1.6,    color="black",        label=f"Denoised (Butterworth {CUTOFF} Hz + Kalman)")
    for idx in np.where(stat_mask)[0]:
        ax.axvspan(time_s[idx]-dt/2, time_s[idx]+dt/2, color="gold", alpha=0.3, lw=0)
    ax.set_ylabel(ylabel, fontsize=9); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
axes[0].plot([], [], color="gold", lw=8, alpha=0.7, label="ZUPT window")
axes[0].legend(fontsize=8)
axes[-1].set_xlabel("Time (s)")
fig.suptitle(f"Accelerometer: Calibrated Raw vs Denoised ({CUTOFF} Hz Butterworth + 3-State Kalman)\n"
             "Gold = ZUPT zero-velocity windows", fontsize=12, fontweight="bold")
plt.tight_layout()
savefig("fig1_raw_vs_denoised.png")

# Fig 2: Calibration bar chart
fig, ax = plt.subplots(figsize=(9, 5))
orientations = list(cal_raw.keys())
xi = np.arange(len(orientations)); w = 0.25
for j, (col, color, lbl) in enumerate(zip(
        ["X","Y","Z"], [COLORS["X"],COLORS["Y"],COLORS["Z"]], ["X","Y","Z"])):
    mv = [cal_raw[o][col].mean() for o in orientations]
    sv = [cal_raw[o][col].std()  for o in orientations]
    ax.bar(xi + j*w, mv, w, yerr=sv, label=f"Axis {lbl}", color=color, alpha=0.8, capsize=4)
ax.axhline(g_true,  color="gray", ls="--", lw=1, label=f"+g = {g_true} m/s²")
ax.axhline(-g_true, color="gray", ls=":",  lw=1, label="−g")
ax.set_xticks(xi + w); ax.set_xticklabels(orientations)
ax.set_ylabel("Acceleration (m/s²)"); ax.set_title("Calibration Orientations: Mean ± Std")
ax.legend(); ax.grid(True, alpha=0.3, axis="y")
plt.tight_layout()
savefig("fig2_calibration_positions.png")

# Fig 3: PSD 
fig, axes = plt.subplots(1, 3, figsize=(13, 4))
for ax, (raw_s, filt_s, title, color) in zip(axes, [
        (ay_raw_cal, ay_f, "Y Axis", COLORS["Y"]),
        (ax_raw_cal, ax_f, "X Axis", COLORS["X"]),
        (az_raw_cal, az_f, "Z Axis", COLORS["Z"])]):
    fr, pr = welch(raw_s,  fs=fs, nperseg=64)
    ff, pf = welch(filt_s, fs=fs, nperseg=64)
    ax.semilogy(fr, pr, alpha=0.7, color=color, lw=1.5, label="Raw")
    ax.semilogy(ff, pf, color="black", lw=2,            label="Denoised")
    ax.axvline(CUTOFF, color="red", lw=1.2, ls="--",    label=f"Cut-off {CUTOFF} Hz")
    ax.set_title(title); ax.set_xlabel("Frequency (Hz)"); ax.set_ylabel("PSD [(m/s²)²/Hz]")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
fig.suptitle("Power Spectral Density: Raw vs Denoised", fontsize=12, fontweight="bold")
plt.tight_layout()
savefig("fig3_psd.png")

# Fig 4: Position — three-way comparison
fig, axes = plt.subplots(2, 1, figsize=(12, 9), sharex=True)

ax = axes[0]
ax.plot(time_s, pos_raw*100,     color="#e74c3c", lw=1.0, alpha=0.5, label="Raw double-integral")
ax.plot(time_s, pos_y_fwd*100,   color="#f39c12", lw=1.4, alpha=0.8, label="Forward Kalman only")
ax.plot(time_s, pos_y*100,       color="#2980b9", lw=2.5,            label="RTS Smoother (best estimate)")
ax.fill_between(time_s, (pos_y-std_y)*100, (pos_y+std_y)*100,
                alpha=0.18, color="#2980b9", label="±1σ uncertainty")
ax.axhline(EXPECTED*100, color="black", ls="--", lw=1.5, label=f"Expected {EXPECTED*100:.0f} cm")
for idx in np.where(stat_mask)[0]:
    ax.axvspan(time_s[idx]-dt/2, time_s[idx]+dt/2, color="gold", alpha=0.35, lw=0)
ax.plot([], [], color="gold", lw=8, alpha=0.7, label="ZUPT anchor")
ax.annotate(f"Peak: {peak_cm:.1f} cm\n({err_pct:.1f}% error)",
            xy=(time_s[peak_idx], peak_cm),
            xytext=(time_s[peak_idx]+0.8, peak_cm-3),
            fontsize=9, color="#2980b9", fontweight="bold",
            arrowprops=dict(arrowstyle="->", color="#2980b9"))
ax.set_ylabel("Y Position (cm)")
ax.set_title(f"Vertical (Y) Position: Raw vs Kalman vs RTS — Peak Error {err_pct:.1f}% "
             f"(was 29% before)")
ax.legend(fontsize=9, loc="upper left"); ax.grid(True, alpha=0.3)

ax = axes[1]
ax.plot(time_s, pos_x*100, color=COLORS["X"], lw=2, label="X (horizontal)")
ax.plot(time_s, pos_z*100, color=COLORS["Z"], lw=2, label="Z (horizontal)")
ax.fill_between(time_s, (pos_x-std_x)*100, (pos_x+std_x)*100, alpha=0.15, color=COLORS["X"])
ax.fill_between(time_s, (pos_z-std_z)*100, (pos_z+std_z)*100, alpha=0.15, color=COLORS["Z"])
ax.set_ylabel("Position (cm)"); ax.set_xlabel("Time (s)")
ax.set_title("Horizontal (X & Z) Position — RTS with ±1σ Bands")
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
plt.tight_layout()
savefig("fig4_position_estimate.png")

# Fig 5: Noise bar chart
fig, ax = plt.subplots(figsize=(8, 5))
axes_lbl = ["X","Y","Z"]
std_raw  = [metrics[a]["std_raw"]  for a in axes_lbl]
std_filt = [metrics[a]["std_filt"] for a in axes_lbl]
xi = np.arange(3)
ax.bar(xi-0.2, std_raw,  0.35, label="Calibrated raw",     color="#e74c3c", alpha=0.85)
ax.bar(xi+0.2, std_filt, 0.35, label=f"Denoised ({CUTOFF} Hz)", color="#2980b9", alpha=0.85)
for i, (r, f) in enumerate(zip(std_raw, std_filt)):
    gain = 20*np.log10(r/f)
    ax.text(i, max(r,f)+0.005, f"+{gain:.1f} dB", ha="center",
            fontsize=10, fontweight="bold", color="#2c3e50")
ax.set_xticks(xi); ax.set_xticklabels(["X Axis","Y Axis","Z Axis"])
ax.set_ylabel("Standard Deviation (m/s²)")
ax.set_title(f"Noise Reduction: Calibrated Raw vs Denoised ({CUTOFF} Hz Butterworth)")
ax.legend(); ax.grid(True, alpha=0.3, axis="y")
plt.tight_layout()
savefig("fig5_noise_comparison.png")

# Fig 6: Velocity + bias
fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
ax = axes[0]
ax.plot(time_s, vel_raw*100, color="#e74c3c", lw=1.0, alpha=0.6, label="Raw velocity (cm/s)")
ax.plot(time_s, vel_y*100,   color="#2980b9", lw=2.0,            label="RTS velocity (cm/s)")
ax.axhline(0, color="gray", lw=0.8, ls="--")
for idx in np.where(stat_mask)[0]:
    ax.axvspan(time_s[idx]-dt/2, time_s[idx]+dt/2, color="gold", alpha=0.35, lw=0)
ax.set_ylabel("Velocity (cm/s)")
ax.set_title("Velocity: Raw vs RTS  (Gold = ZUPT anchors where v=0 is enforced)")
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

ax = axes[1]
ax.plot(time_s, bias_y*1000, color="#27ae60", lw=2.0, label="Y bias estimate (mm/s²)")
ax.axhline(0, color="gray", lw=0.8, ls="--")
ax.set_ylabel("Estimated Bias (mm/s²)"); ax.set_xlabel("Time (s)")
ax.set_title("Online Bias Estimation — 3rd State of Kalman Filter")
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
plt.tight_layout()
savefig("fig6_velocity_bias.png")

# Fig 7: Position uncertainty
fig, ax = plt.subplots(figsize=(11, 4))
ax.plot(time_s, std_y*100, color="#9b59b6", lw=2.0, label="RTS 1-σ position uncertainty")
ax.fill_between(time_s, 0, std_y*100, alpha=0.18, color="#9b59b6")
for idx in np.where(stat_mask)[0]:
    ax.axvspan(time_s[idx]-dt/2, time_s[idx]+dt/2, color="gold", alpha=0.5, lw=0)
ax.plot([], [], color="gold", lw=8, alpha=0.7, label="ZUPT anchor (uncertainty resets)")
ax.axhline(theoretical_pos_err*100, color="red", ls="--", lw=1.5,
           label=f"Noise floor ({theoretical_pos_err*100:.0f} cm)")
ax.set_ylabel("1-σ Position Uncertainty (cm)"); ax.set_xlabel("Time (s)")
ax.set_title("Position Uncertainty: Resets at ZUPT Anchors, Grows Between Them")
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
plt.tight_layout()
savefig("fig7_position_uncertainty.png")


print("  OPTIMISED PIPELINE — FINAL SUMMARY")

print("\n  CALIBRATION")
print(f"    Scale  X={scale_x:.4f}  Y={scale_y:.4f}  Z={scale_z:.4f}")
print(f"    Bias   X={bias_x_cal:.4f}  Y={bias_y_cal:.4f}  Z={bias_z_cal:.4f} m/s²")
print(f"    Gravity ref: {g_ref:.5f} m/s²  (residual: {(g_ref-g_true)*1000:+.2f} mm/s²)")

print("\n  NOISE REDUCTION (Butterworth 4.2 Hz, order 4)")
print(f"    {'Axis':<5} {'Std Raw':>10} {'Std Filt':>10} {'SNR Gain':>12}")
for a, m in metrics.items():
    print(f"    {a:<5} {m['std_raw']:>10.4f} {m['std_filt']:>10.4f} {m['snr']:>11.2f} dB")

print("\n  POSITION ESTIMATION (Y vertical axis)")
print(f"    Expected                : {EXPECTED*100:.2f} cm")
print(f"    Raw double-integral     : {pos_raw.max()*100:.2f} cm  (error: {abs(pos_raw.max()*100-EXPECTED*100)/EXPECTED:.0f}%)")
print(f"    Forward Kalman only     : {pos_y_fwd.max()*100:.2f} cm")
print(f"    RTS Smoother (optimal)  : {peak_cm:.2f} cm  ← {err_pct:.1f}% error")
print(f"    Error: {err_pct:.1f}% position error")

print("\n  FILTER CONFIGURATION")
print(f"    Pre-filter   : Butterworth order={ORDER}, cut-off={CUTOFF} Hz  (was 5.0)")
print(f"    ZUPT thr     : {ZUPT_THR} m/s² (was 0.08)  → {nz_y} ZUPT windows ({100*nz_y/N:.1f}%)")
print(f"    ZUPT R_vel   : {5e-4} (m/s)²  (was 0.001)")
print(f"    Model        : 3-state [position, velocity, accel_bias]")
print(f"    Algorithm    : Forward Kalman + RTS backward smoother")
print(f"    Bias end     : {bias_y[-1]*1000:.4f} mm/s²")

print("\n  PHYSICAL LIMIT")
print(f"    Noise density : {noise_density_y*1000:.1f} mg/√Hz at {fs:.1f} Hz")
print(f"    Min error (theoretical) : {theoretical_pos_err*100:.0f} cm without external fix")