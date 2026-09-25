"""07 - Joint position control: DC motor model and Ziegler-Nichols tuning (thesis 4.3).

1) Motor model from the datasheet (values at the 50:1 gearbox output)
     no-load speed w0 = 100 rpm, stall torque Ts = 42 kg·cm, supply V = 12 V
     Ke = Kt = V / w0            (back-EMF = torque constant, SI units)
     R       = V Kt / Ts         (stall: all voltage across the winding)
   Joint J1 with the arm as load (inertia J, viscous friction b):
     J th'' = Kt (u - Ke th') / R - b th'
     G(s) = th(s)/u(s) = K / (s (tau s + 1)),
       K = (Kt/R) / (b + Kt Ke / R),   tau = J / (b + Kt Ke / R)
   A dead time Td (5 ms loop, ADC filtering, PWM, gear backlash) is added:
     G(s) e^(-s Td).

2) Ziegler-Nichols (closed-loop method, used in the thesis):
   ultimate frequency wu where the phase is -180 deg:
     -90 deg - atan(wu tau) - wu Td = -180 deg
   ultimate gain Ku = 1 / |G(j wu)|, period Pu = 2 pi / wu
     P  : Kp = 0.50 Ku
     PI : Kp = 0.45 Ku, Ti = Pu / 1.2
     PID: Kp = 0.60 Ku, Ti = Pu / 2, Td = Pu / 8
   ZN aims at a quarter-amplitude decay, i.e. ~30-50 % overshoot. For a robot
   joint that is too aggressive, so the Tyreus-Luyben PI rule is also shown:
     TL-PI: Kp = Ku / 3.2, Ti = 2.2 Pu
   On a motor (an integrating plant) every controller with integral action
   overshoots a large *step* once the 12 V limit saturates. The robot is never
   commanded with steps, though: ros2_control sends smooth trajectories. So the
   second test tracks a quintic 30 deg move and adds the industrial fix,
   feed-forward from the reference speed w_r and acceleration a_r:
     u_ff = w_r / K + (J R / Kt) a_r       (voltage the motor needs anyway)

3) Step responses are simulated with voltage saturation (+/-12 V) and with the
   3.52 deg potentiometer quantisation of the 2019 prototype (see 06).

Run:  python3 analysis/07_motor_control_tuning.py
"""
import math

import matplotlib.pyplot as plt
import numpy as np

from common import KGCM, header, save

header("07 Motor model and Ziegler-Nichols tuning")
V, w0, Ts = 12.0, 100 * 2 * math.pi / 60, 42 * KGCM
Kt = V / w0
R = V * Kt / Ts
Td = 0.020      # total dead time [s]
# load per joint: inertia seen by the motor (arm from script 03 + reflected rotor) and friction
JOINTS = {"J1": (0.13, 0.4), "J2": (0.061, 0.3)}
print(f"motor: Kt = {Kt:.3f} N·m/A, R = {R:.2f} ohm")
pwm_per_deg = (255 / V) * (math.pi / 180)


def ultimate(J, b):
    """Plant parameters and Ziegler-Nichols ultimate gain / period of one joint."""
    b_eff = b + Kt * Kt / R
    K, tau = (Kt / R) / b_eff, J / b_eff
    lo, hi = 1e-3, 1e4
    for _ in range(200):                              # bisection on the phase condition
        wm = 0.5 * (lo + hi)
        phase = -math.pi / 2 - math.atan(wm * tau) - wm * Td
        lo, hi = (wm, hi) if phase > -math.pi else (lo, wm)
    return K, tau, wm * math.sqrt(1 + (wm * tau) ** 2) / K, 2 * math.pi / wm


for name, (Jj, bj) in JOINTS.items():
    Kj, tj, Kuj, Puj = ultimate(Jj, bj)
    print(f"{name}: J = {Jj:.3f} kg·m^2, K = {Kj:.3f}, tau = {tj * 1000:.0f} ms -> Ku = {Kuj:.1f} V/rad "
          f"= {Kuj * pwm_per_deg:.1f} PWM/deg, Pu = {Puj:.2f} s   (firmware KU/PU)")

J, b = JOINTS["J1"]
K, tau, Ku, Pu = ultimate(J, b)

tunings = {
    "P  (0.5 Ku)": (0.5 * Ku, math.inf, 0.0),
    "PI (0.45 Ku, Pu/1.2)": (0.45 * Ku, Pu / 1.2, 0.0),
    "PID (0.6 Ku, Pu/2, Pu/8)": (0.6 * Ku, Pu / 2, Pu / 8),
    "TL-PI (Ku/3.2, 2.2 Pu)": (Ku / 3.2, 2.2 * Pu, 0.0),
    "P (0.5 Ku) + feed-forward": (0.5 * Ku, math.inf, 0.0, True),
}
print("\nJ1 gains in firmware units (PWM counts per degree of error):")
for name, (kp, ti, td, *_) in tunings.items():
    print(f"  {name:26s}: Kp = {kp:6.1f} V/rad = {kp * pwm_per_deg:5.2f} PWM/deg, "
          f"Ti = {ti if ti < 1e9 else float('inf'):.3f} s, Td = {td:.3f} s")


def simulate(kp, ti, td, ff=False, quant_deg=0.0, ref=None, T=3.0, dt=1e-3):
    """Discrete loop simulation at 1 kHz with dead time, +/-12 V saturation,
    anti-windup and optional sensor quantisation. ``ref(t)`` in degrees
    (default: 30 deg step)."""
    ref = ref or (lambda _t: 30.0)
    n_delay = int(round(Td / dt))
    th, w, integ, e_prev = 0.0, 0.0, 0.0, 0.0
    buf = [0.0] * (n_delay + 1)
    t_out, y_out, r_out = [], [], []
    for i in range(int(T / dt)):
        t_i = i * dt
        meas = th if quant_deg == 0 else math.radians(quant_deg) * round(th / math.radians(quant_deg))
        e = math.radians(ref(t_i)) - meas
        u = kp * (e + (integ / ti if ti < math.inf else 0.0) + td * (e - e_prev) / dt)
        if ff:  # reference speed / acceleration by finite differences
            w_r = math.radians(ref(t_i + dt) - ref(t_i - dt)) / (2 * dt)
            a_r = math.radians(ref(t_i + dt) - 2 * ref(t_i) + ref(t_i - dt)) / dt ** 2
            u += w_r / K + J * R / Kt * a_r
        u_sat = max(-V, min(V, u))
        if u == u_sat and ti < math.inf:
            integ += e * dt                       # anti-windup: integrate only when not saturated
        e_prev = e
        buf.append(u_sat)
        u_applied = buf.pop(0)
        w += ((Kt * (u_applied - Kt * w) / R - b * w) / J) * dt
        th += w * dt
        t_out.append(t_i)
        y_out.append(math.degrees(th))
        r_out.append(ref(t_i))
    return np.array(t_out), np.array(y_out), np.array(r_out)


def quintic_ref(t, amp=30.0, T=0.8):
    """Smooth reference as sent by the trajectory controller (rest to rest)."""
    u = min(max(t / T, 0.0), 1.0)
    return amp * (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5)


fig, axs = plt.subplots(1, 2, figsize=(12, 4.3))
print("\n30 deg STEP (classic ZN test):")
for name, gains in tunings.items():
    if len(gains) == 4:
        continue  # feed-forward needs a smooth reference, not a step
    t, y, _ = simulate(*gains)
    overshoot = max(0.0, y.max() - 30.0) / 30.0 * 100
    settled = np.abs(y - 30.0) > 0.5
    t_settle = t[np.nonzero(settled)[0][-1]] if settled.any() else 0.0
    print(f"  {name:26s}: overshoot {overshoot:4.1f} %, settling (+/-0.5 deg) {t_settle:.2f} s")
    axs[0].plot(t, y, label=name)
print("30 deg SMOOTH trajectory in 0.8 s (how the robot is really commanded):")
for name, gains in tunings.items():
    t, y, r = simulate(*gains, ref=quintic_ref, T=1.5)
    t_q, y_q, r_q = simulate(*gains, ref=quintic_ref, T=1.5, quant_deg=3600 / 1023)
    print(f"  {name:26s}: max tracking error {np.abs(y - r).max():4.2f} deg, "
          f"with 2019 pot resolution {np.abs(y_q - r_q).max():4.2f} deg")
    axs[1].plot(t, y - r, label=name)
axs[0].axhline(30, color="k", lw=0.8, ls=":")
axs[0].set_title("30 deg step on J1 (ZN tuning test)")
axs[0].set_ylabel("joint angle [deg]")
axs[1].set_title("Tracking error on a smooth 30 deg trajectory")
axs[1].set_ylabel("error [deg]")
for ax in axs:
    ax.set_xlabel("time [s]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
save(fig, "07_motor_control_tuning.png")
