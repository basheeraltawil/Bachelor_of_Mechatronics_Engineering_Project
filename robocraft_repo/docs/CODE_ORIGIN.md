# Code provenance

The files under `src/` were reconstructed from the supplied `codesappendix.pdf`.
They preserve the appendix's legacy naming, constants, comments, and control approach as
closely as practical during PDF-to-source extraction.

The appendix contains:
1. Trajectory generation
2. Raspberry Pi master ↔ ATmega slave communication
3. Pure rotation / platform attachment-detachment task
4. Drawing-square / pure-translation task

Because PDF extraction can alter indentation or special characters, the extracted files should
be reviewed before being used on hardware. No claim is made here that the reconstructed source
is directly runnable without cleanup and hardware-specific configuration.
