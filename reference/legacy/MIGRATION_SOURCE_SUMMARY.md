# VOX_EFX Legacy Migration Source Summary

## Legacy Project

The pre-MPDF VOX_EFX project was preserved locally as:

`~/Projects/VOX_EFX_legacy`

The active MPDF-managed VOX_EFX project continues the same Git history on the existing repository and `develop` branch.

## Migration Purpose

The legacy working copy is retained as historical evidence while the current VOX_EFX engineering state is reconstructed under MPDF V2.

## Migration Basis

Baseline reconstruction uses:

- the existing Git history;
- current ESP32 firmware and generated SquareLine/LVGL UI source;
- current Teensy firmware and test source;
- SquareLine project files and assets;
- mechanical CAD, STEP, and STL artifacts;
- relevant prior project discussions where they clarify intent or historical decisions.

## Migration Rule

Existing implementation is evidence of current engineering state, but implementation presence alone does not prove verification.

Historical intent must not automatically be treated as a current requirement unless confirmed by the current implementation or explicitly re-approved during MPDF baseline reconstruction.

Unfinished or placeholder functionality must remain identified as incomplete rather than being promoted to verified capability merely because UI controls or protocol commands already exist.
