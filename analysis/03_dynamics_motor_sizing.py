"""03 - Dynamics and motor sizing (thesis Sections 2.3 and 4.1).

Question: is the selected DC gear motor (12 V, 50:1, 100 rpm no-load,
42 kg·cm stall) strong enough, and how much margin is left?

Model: planar 2-link arm, Lagrange equations (thesis 2.3.1). Link 2 carries
the quill, wrist, gripper and the 0.2 kg payload required by the thesis.
    M11 = I1 + I2 + m1 c1^2 + m2 (a^2 + c2^2 + 2 a c2 cos q2)
    M12 = I2 + m2 (c2^2 + a c2 cos q2)
    M22 = I2 + m2 c2^2
    h   = m2 a c2 sin q2
    tau1 = M11 q1dd + M12 q2dd - h (2 q1d q2d + q2d^2) + d1 q1d
    tau2 = M12 q1dd + M22 q2dd + h q1d^2              + d2 q2d
(c = centre-of-mass distance, I = inertia about the CoM, d = viscous friction.)

SCARA property: J1 and J2 rotate about *vertical* axes, so gravity loads the
bearings, not the motors. Gravity only matters for the Z axis (J3) and for the
bending moment on the J1 bearings (checked at the end).

DC motor torque-speed line:  tau_max(w) = tau_stall (1 - w / w_noload)

Run:  python3 analysis/03_dynamics_motor_sizing.py
"""
import math

import matplotlib.pyplot as plt
import numpy as np

from common import G, KGCM, config, header, save
from robocraft_kinematics.trajectory import ptp_duration, quintic

header("03 Dynamics and motor sizing")
cfg = config()
A, L = cfg["arm"], cfg["limits"]
a, b = A["link1_length"], A["link2_length"]
payload = 0.200                                    # thesis requirement [kg]

# ---- mass properties -----------------------------------------------------------
m1, c1 = A["link1_mass"], 0.043                    # CoM pulled to J1 by the rear motor
I1 = m1 * ((a + A["link1_tail"]) ** 2 + A["link_width"] ** 2) / 12
m_bar, c_bar = A["link2_mass"], 0.15
m_tip = A["quill_mass"] + A["wrist_mass"] + 2 * A["finger_mass"] + payload
m2 = m_bar + m_tip
c2 = (m_bar * c_bar + m_tip * b) / m2
I2 = m_bar * (b + 0.04) ** 2 / 12 + m_bar * (c_bar - c2) ** 2 + m_tip * (b - c2) ** 2
d1, d2 = 0.4, 0.3                                  # joint damping [N·m·s/rad] (same as the URDF)
print(f"link 1: m = {m1:.2f} kg, CoM {c1 * 1000:.0f} mm, I = {I1 * 1e3:.2f} g·m^2")
print(f"link 2 + tool + payload: m = {m2:.2f} kg, CoM {c2 * 1000:.0f} mm, I = {I2 * 1e3:.2f} g·m^2")


def inverse_dynamics(q, qd, qdd):
    c, s = math.cos(q[1]), math.sin(q[1])
    M11 = I1 + I2 + m1 * c1 ** 2 + m2 * (a ** 2 + c2 ** 2 + 2 * a * c2 * c)
    M12 = I2 + m2 * (c2 ** 2 + a * c2 * c)
    M22 = I2 + m2 * c2 ** 2
    h = m2 * a * c2 * s
    t1 = M11 * qdd[0] + M12 * qdd[1] - h * (2 * qd[0] * qd[1] + qd[1] ** 2) + d1 * qd[0]
    t2 = M12 * qdd[0] + M22 * qdd[1] + h * qd[0] ** 2 + d2 * qd[1]
    return t1, t2


# ---- worst-case motion: both joints, fastest allowed PTP, arm nearly stretched --
q0, qf = np.array([-1.2, 0.3]), np.array([1.2, 2.2])
vmax = [L["j1"]["velocity"], L["j2"]["velocity"]]
T = ptp_duration(q0, qf, vmax, [6.0, 6.0])
t = np.linspace(0, T, 400)
tau, omega = [], []
for ti in t:
    q, qd, qdd = quintic(q0, qf, T, ti)
    tau.append(inverse_dynamics(q, qd, qdd))
    omega.append(qd)
tau, omega = np.abs(np.array(tau)), np.abs(np.array(omega))

tau_stall = 42 * KGCM                              # 4.12 N·m at the gearbox output
w_noload = 100 * 2 * math.pi / 60                  # 10.47 rad/s
need = 16 * KGCM                                   # thesis requirement 16 kg·cm
margin = np.min(tau_stall * (1 - omega / w_noload) / np.maximum(tau, 1e-9))
print(f"\nfastest PTP ({T:.2f} s, 30 rpm rated speed): peak torque J1 = {tau[:, 0].max():.2f} N·m, "
      f"J2 = {tau[:, 1].max():.2f} N·m")
print(f"thesis requirement: {need:.2f} N·m (16 kg·cm);  motor stall: {tau_stall:.2f} N·m (42 kg·cm)")
print(f"smallest margin to the motor torque-speed line: x{margin:.1f}")

# ---- Z axis (quill) and bearing load ---------------------------------------------
lead, eta = 0.005, 0.9                             # 5 mm ball screw, 90 % efficiency
F_z = (A["quill_mass"] + A["wrist_mass"] + 2 * A["finger_mass"] + payload) * (G + 2.0)
print(f"\nZ axis: {F_z:.1f} N (weight + 2 m/s^2) -> screw torque {F_z * lead / (2 * math.pi * eta) * 1000:.1f} mN·m")
M_bend = G * (m1 * c1 + m2 * (a + c2))             # arm stretched, about the J1 bearings
print(f"bending moment on the J1 bearings (arm stretched): {M_bend:.2f} N·m "
      f"-> the thesis uses two 17 mm deep-groove bearings to carry it")

w = np.linspace(0, w_noload, 50)
fig, axs = plt.subplots(1, 2, figsize=(12, 4.5))
axs[0].plot(t, tau[:, 0], label="J1")
axs[0].plot(t, tau[:, 1], label="J2")
axs[0].axhline(need, ls="--", color="tab:red", label="thesis requirement 16 kg·cm")
axs[0].set_xlabel("time [s]")
axs[0].set_ylabel("|torque| [N·m]")
axs[0].set_title("Fastest PTP move with 0.2 kg payload")
axs[0].legend()
axs[0].grid(alpha=0.3)
axs[1].plot(w * 60 / (2 * math.pi), tau_stall * (1 - w / w_noload), "k", label="motor limit (12 V)")
axs[1].scatter(omega[:, 0] * 60 / (2 * math.pi), tau[:, 0], s=4, label="J1 operating points")
axs[1].scatter(omega[:, 1] * 60 / (2 * math.pi), tau[:, 1], s=4, label="J2 operating points")
axs[1].set_xlabel("joint speed [rpm]")
axs[1].set_ylabel("torque [N·m]")
axs[1].set_title("All operating points stay under the motor line")
axs[1].legend()
axs[1].grid(alpha=0.3)
save(fig, "03_dynamics_motor_sizing.png")
