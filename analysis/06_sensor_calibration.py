"""06 - Position sensor: potentiometer calibration and resolution (thesis 4.2).

Sensor: Bourns 3590 10-turn 10 kOhm precision potentiometer read by the
10-bit ADC of the ATmega (0..1023 counts over 0..3600 deg).

1) Calibration (thesis Fig. 4.2.2): commanded angle A vs. measured angle B
   between 360 and 720 deg, least-squares line  B = k A + d.
   The thesis reports  y = 1.0074 x - 2.6413.
   (Values below were transcribed from the thesis table; a few OCR-damaged
   digits were reconstructed from their neighbours. Fitting only this
   360..720 deg range gives a slightly steeper line than the thesis value,
   whose plot axis suggests a fit over a wider range - both are shown.)

2) Resolution: one ADC count = 3600 / 1023 = 3.52 deg of pot rotation. With
   the pot coupled 1:1 to the joint, the tip position quantisation is
       dx = (a + b) * dq   (arm stretched)
   This single number explains most of the positioning error of the 2019
   prototype and is the main hardware upgrade recommended.

Run:  python3 analysis/06_sensor_calibration.py
"""
import math

import matplotlib.pyplot as plt
import numpy as np

from common import header, save

header("06 Potentiometer calibration and resolution")
commanded = np.arange(360, 730, 10, dtype=float)
measured = np.array([355.54, 370.00, 380.06, 390.62, 401.17, 409.97, 420.88, 431.79, 439.88, 450.44,
                     461.00, 472.20, 481.66, 491.10, 500.43, 510.26, 520.82, 531.38, 541.94, 552.90,
                     563.05, 573.63, 584.18, 594.37, 604.93, 615.86, 624.99, 633.43, 643.99, 654.55,
                     665.30, 675.66, 686.58, 695.72, 705.92, 717.18, 727.39])
k, d = np.polyfit(commanded, measured, 1)
resid = measured - (k * commanded + d)
print(f"least-squares fit: B = {k:.4f} A {d:+.4f}   (thesis: 1.0074 A - 2.6413)")
print(f"residual RMS = {np.sqrt(np.mean(resid ** 2)):.2f} deg, max = {np.abs(resid).max():.2f} deg")

reach = 0.44
options = {
    "10-bit ADC, 10-turn pot (2019)": 3600 / 1023,
    "16-bit ADC (ADS1115), same pot": 3600 / 65535,
    "12-bit magnetic encoder (AS5600)": 360 / 4096,
    "10-bit ADC, pot geared 1:10": 360 / 1023,
}
print("\nangular resolution and resulting TCP quantisation at 440 mm reach:")
for name, dq in options.items():
    print(f"  {name:34s}: {dq:6.3f} deg -> {reach * math.radians(dq) * 1000:6.2f} mm")

fig, axs = plt.subplots(1, 2, figsize=(12, 4.3))
axs[0].plot(commanded, measured, "o", ms=4, label="measured (thesis)")
axs[0].plot(commanded, k * commanded + d, "-", label=f"fit of this table: B = {k:.4f} A {d:+.2f}")
axs[0].plot(commanded, 1.0074 * commanded - 2.6413, "--", label="thesis: B = 1.0074 A - 2.64")
axs[0].set_xlabel("commanded angle A [deg]")
axs[0].set_ylabel("measured angle B [deg]")
axs[0].set_title("Potentiometer linearisation")
axs[0].legend()
axs[0].grid(alpha=0.3)
names = list(options)
axs[1].barh(names, [reach * math.radians(v) * 1000 for v in options.values()], color="tab:orange")
axs[1].set_xscale("log")
axs[1].set_xlabel("TCP position step for 1 sensor count [mm] (log)")
axs[1].set_title("Why the 2019 prototype was imprecise")
axs[1].grid(alpha=0.3, which="both")
save(fig, "06_sensor_calibration.png")
