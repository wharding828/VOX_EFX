---
id: PRJ-001
type: project
status: active
mpdf_version: "2.0"
---

# VOX_EFX

## Purpose

VOX_EFX is an embedded vocal-effects and preamp-control system built around a dedicated audio processor and a separate graphical user-interface controller.

The product provides a physical touchscreen/encoder user interface for controlling vocal audio processing, global input/output settings, system configuration, connectivity, and operating status.

## Engineering State

VOX_EFX is an existing multidisciplinary project being migrated into MPDF V2.

The project predates MPDF and contains substantial firmware, graphical UI, CAD, and enclosure work. MPDF records therefore establish a current-state engineering baseline rather than attempting to recreate every historical development event.

The current architecture uses:

- an ESP32 as the graphical UI, connectivity, and supervisory controller;
- LVGL with a SquareLine-generated user interface;
- a 480 x 320 TFT with FT6336U touch support and rotary encoder navigation;
- Wi-Fi configuration and stored credentials on the ESP32;
- MQTT for diagnostic/status messaging and future network integration;
- a Teensy 4.0 with Audio Shield Rev D as the real-time audio-processing controller;
- UART communication between ESP32 and Teensy;
- a mono audio path using the SGTL5000 line input and output;
- an implemented multi-band EQ stage with true bypass;
- compressor, reverb, and delay processing slots whose control interfaces exist but whose DSP remains incomplete;
- FreeCAD/STEP/STL mechanical and enclosure artifacts;
- SquareLine UI source and exported LVGL artifacts.

The existing Git history remains authoritative implementation history. Historical intent recovered from older development discussions must be reconciled against the current source before being treated as a current requirement.

## Current Focus

Establish the authoritative MPDF baseline for the existing VOX_EFX project.

Immediate engineering work is to:

1. preserve and classify the existing firmware, UI, and mechanical artifacts;
2. reconstruct approved current requirements and architectural decisions;
3. distinguish implemented DSP functions from placeholders and unfinished functions;
4. establish current verification status;
5. identify unresolved integration and hardware issues;
6. resume VOX_EFX development using MPDF as the engineering source of truth.
