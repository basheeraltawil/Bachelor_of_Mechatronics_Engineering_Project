# Code origin

The files in this folder were reconstructed from the thesis code appendix
([`../code_appendix/code_appendix.pdf`](../code_appendix/code_appendix.pdf)). They keep the
original names, constants, comments and control approach as closely as the PDF extraction allowed.

| File | Appendix section |
|---|---|
| `trajectory/trajectory_generation.py` | 1 – trajectory generation (via point, IK, cubic segments) |
| `communication/master_raspberry_pi.py` | 2a – Raspberry Pi I²C master |
| `communication/slave_atmega.ino` | 2b – ATmega I²C slave, potentiometers, P control |
| `tasks/pure_rotation.py` | 3 – pure rotation / platform attach‑detach |
| `tasks/drawing_square.py` | 4 – drawing a square (pure translation) |

PDF extraction can change indentation or characters, so these files are **reference material**,
not runnable code. Known hardware problems are listed in
[`docs/04_hardware.md`](../../docs/04_hardware.md#43-firmware); every algorithm has a tested
replacement in [`robocraft_ws`](../../robocraft_ws).
