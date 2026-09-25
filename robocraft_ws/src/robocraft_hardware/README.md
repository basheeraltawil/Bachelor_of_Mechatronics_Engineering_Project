# robocraft_hardware

The real robot.

* `src/robocraft_i2c_system.cpp`: ros2_control **hardware interface**. It sends joint targets to one
  ATmega per arm over I²C and reads back the measured angles (CRC‑8 protected frames, error counter,
  simulated J3/J4 until that hardware exists).
* `firmware/robocraft_joint_firmware/`: **ATmega328 firmware**: PI + velocity feed-forward position
  loop, potentiometer linearisation, watchdog, soft limits, gripper and brake servos.
* `test/`: unit tests of the frame format.

Protocol, pin map and commissioning: [`docs/04_hardware.md`](../../../docs/04_hardware.md).
