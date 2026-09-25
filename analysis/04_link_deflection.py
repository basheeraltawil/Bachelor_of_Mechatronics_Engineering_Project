"""04 - Link stiffness: deflection and stress (thesis design constraint).

Constraint from the thesis: with at least 0.2 kg payload the arm tip may not
deflect more than 2.5 mm. Links: aluminium bars 25 mm wide.

Model: the stretched arm (worst case, both links in a line, L = a + b) as a
cantilever clamped at J1, with
  * a point load F at the tip (tool + payload weight, or the thesis 70 N test)
  * its own weight as a distributed load w = m g / L
    delta_tip = F L^3 / (3 E I)  +  w L^4 / (8 E I)
    I = b h^3 / 12               (rectangle, bending about its width)
    sigma_max = M c / I,  M = F L + w L^2 / 2,  c = h / 2
Safety factor n = yield strength / sigma_max.

The joints are treated as rigid, so this is the deflection of the links alone;
bearing play adds to it on the real robot.

Run:  python3 analysis/04_link_deflection.py
"""
import matplotlib.pyplot as plt
import numpy as np

from common import G, config, header, save

header("04 Link deflection and stress")
A = config()["arm"]
E, yield_strength = 69e9, 276e6            # aluminium 6061-T6 [Pa]
width, L = A["link_width"], A["link1_length"] + A["link2_length"]
m_links = A["link1_mass"] + A["link2_mass"]
payload = 0.200
F_tool = (A["quill_mass"] + A["wrist_mass"] + 2 * A["finger_mass"] + payload) * G


def tip(h, F):
    I = width * h ** 3 / 12
    w = m_links * G / L
    delta = F * L ** 3 / (3 * E * I) + w * L ** 4 / (8 * E * I)
    sigma = (F * L + w * L ** 2 / 2) * (h / 2) / I
    return delta, sigma


h = A["link_thickness"]
for label, F in (("normal load (tool + 0.2 kg)", F_tool), ("thesis overload test (70 N)", 70.0)):
    d, s = tip(h, F)
    print(f"{label:30s}: F = {F:5.1f} N  deflection = {d * 1000:5.2f} mm  "
          f"stress = {s / 1e6:5.1f} MPa  safety factor = {yield_strength / s:4.1f}")

hs = np.linspace(0.005, 0.030, 200)
d_normal = np.array([tip(x, F_tool)[0] for x in hs]) * 1000
h_min = hs[np.argmax(d_normal <= 2.5)]
print(f"\nthinnest link that keeps the 2.5 mm limit under normal load: {h_min * 1000:.1f} mm "
      f"(selected: {h * 1000:.0f} mm)")

fig, ax = plt.subplots(figsize=(7, 4.5))
ax.plot(hs * 1000, d_normal, label="tool + 0.2 kg payload")
ax.plot(hs * 1000, [tip(x, 70.0)[0] * 1000 for x in hs], label="70 N overload (thesis test)")
ax.axhline(2.5, color="tab:red", ls="--", label="limit 2.5 mm")
ax.axvline(h * 1000, color="k", ls=":", label=f"selected {h * 1000:.0f} mm")
ax.set_ylim(0, 10)
ax.set_xlabel("link thickness h [mm]  (width 25 mm)")
ax.set_ylabel("tip deflection, arm stretched [mm]")
ax.set_title("Link thickness vs deflection (cantilever, L = 440 mm)")
ax.legend()
ax.grid(alpha=0.3)
save(fig, "04_link_deflection.png")
