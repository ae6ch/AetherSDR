# HPSDR / Anan RX Support — Design Spec

**Date:** 2026-04-10  
**Scope:** RX-only support for Apache Labs Anan 10E via OpenHPSDR Protocol 2 (P2)  
**Approach:** Self-contained `HpsdrRadio` module; minimal changes to SmartSDR code paths

---

## Background

The Apache Labs Anan 10E is an HF SDR transceiver that uses the **OpenHPSDR Protocol 2 (P2)**,
the same protocol family used by ThetisSDR (itself descended from PowerSDR, the original
FlexRadio client). Unlike FlexRadio SmartSDR — where the radio does all DSP and sends
processed audio and spectrum — HPSDR radios send **raw IQ samples** and the host PC performs
all demodulation, filtering, and FFT.

This spec covers RX only (spectrum display + audio demodulation). TX and full feature parity
are deferred to a future phase.

---

## Goals

- Display spectrum and waterfall for the Anan 10E in AetherSDR
- Demodulate LSB, USB, AM, and CW for audio output
- Allow frequency and mode control via the existing slice control UI
- Discover Anan radios alongside FlexRadio radios in the connection panel
- No changes to the SmartSDR/FlexRadio code path

## Non-Goals (deferred)

- TX / transmit path
- Multiple simultaneous receivers
- Proper `IRadioBackend` abstraction (prerequisite for clean TX + multi-radio)
- Band stacking, spots, SmartLink, serial PTT
- Simultaneous Flex + Anan connection
- Auto-reconnect on P2 packet loss (flag as not implemented; add in follow-up)

---

## Architecture

All new code lives in `src/hpsdr/`. Existing files are changed at five injection points
(see table below).

```
MainWindow
  ├── RadioModel          (existing — FlexRadio path, unchanged)
  │     ├── PanadapterStream
  │     ├── CommandParser
  │     └── AudioEngine ◄─────────────────────────────────────┐
  │                                                            │ feedHpsdrAudio()
  └── HpsdrRadio          (new — Anan path)                   │
        ├── HpsdrDiscovery                                     │
        ├── HpsdrP2Connection                                  │
        ├── HpsdrDsp ─────────────────────────────────────────┘
        │     └── (FFT) ──────────────────────► SpectrumWidget
        └── HpsdrSliceModel                      feedFftBins()
```

Only one radio type (FlexRadio or Anan) is active at a time.

---

## New Classes (`src/hpsdr/`)

### HpsdrDiscovery
Owns a UDP socket bound to local port **1024** for receiving HPSDR P2 discovery broadcasts.
Note: port 1024 is a privileged port on Linux/macOS; the process needs `CAP_NET_BIND_SERVICE`
(Linux) or root (macOS) to bind it. If this proves unworkable, an alternative is to use
`SO_REUSEPORT` or to send the Start packet from an ephemeral port and receive discovery
replies there — this should be validated against actual Anan 10E behaviour on hardware before
implementation.
Parses each packet to extract IP address, MAC address (used as the unique identity key for
stale detection), firmware version, and board ID. Emits `radioFound(HpsdrRadioInfo)` signals.
Run in parallel with the existing `RadioDiscovery` from `MainWindow` so both radio types
appear in the same connection list.

`HpsdrRadioInfo` carries: IP, MAC (unique key), board ID, firmware version.

### HpsdrP2Connection
Uses a **separate UDP socket** from `HpsdrDiscovery` — bound to an OS-assigned ephemeral
local port — targeting the radio at its IP, port 1024. (This avoids a port conflict with
`HpsdrDiscovery`'s socket.)

- **Start packet:** sent to radio port 1024 from our ephemeral port; initiates IQ streaming
  and advertises our local port; sets sample rate and receiver count (1)
- **Control packets:** sent periodically (~1ms interval) carrying RX frequency, sample rate,
  preamp/attenuator flags, dither/random settings
- **IQ data packets:** received from the radio on our ephemeral port; each contains a sequence
  number and blocks of 24-bit signed big-endian IQ samples
- **Stop packet:** sent on disconnect to halt streaming

Runs IQ reception on a dedicated thread. Emits `iqReady(QByteArray)` to `HpsdrDsp`.

### HpsdrDsp
Processes raw IQ on a dedicated DSP thread. Two parallel paths:

**FFT path (spectrum):**
1. Accumulate 4096 samples
2. Apply Hann window
3. FFT via FFTW3 (see Build Dependencies below)
4. Convert to dBFS
5. Emit `fftReady(quint64 centerHz, float bandwidthHz, QVector<float> bins)` at ~30 fps

**Audio path (demodulation):**
1. Frequency translate: mix IQ down to baseband centered on slice frequency using an NCO
2. FIR low-pass filter + decimate 384 kHz → 48 kHz (÷8)
3. Demodulate by mode using the analytic signal approach:
   - **USB:** emit Re(I + jQ) — real part of the complex analytic signal
   - **LSB:** emit Re(I − jQ) — negate Q before taking real part
   - **AM:** emit magnitude √(I² + Q²) — envelope detection
   - **CW:** narrow bandpass filter (pre-decimation) + BFO frequency offset, then USB path
4. Convert float32 mono 48 kHz → int16 stereo 24 kHz (duplicate mono to both channels,
   resample 48k→24k via a short FIR, convert to int16)
5. Emit `pcmReady(QByteArray int16Stereo24k)`

Cross-thread parameter updates (frequency, mode) use `std::atomic` per the project's existing
convention for audio-thread-safe parameters.

### HpsdrSliceModel
Thin QObject model holding: frequency (Hz), mode (LSB/USB/AM/CW), filter low/high (Hz).
Frequency, mode, and filter are **session-only** — they are not persisted in AppSettings,
because for HPSDR the host is authoritative (unlike FlexRadio where the radio echoes state
back). Changes update `HpsdrDsp` atomically and schedule values into the next outgoing
control packet.

**UI wiring:** `HpsdrSliceModel` inherits from `SliceModel` and overrides `setFrequency()`,
`setMode()`, and `setFilterWidth(int low, int high)` to update the HPSDR DSP/control path
instead of emitting SmartSDR `commandReady` strings. This allows `RxApplet::setSlice(SliceModel*)`
and `VfoWidget::setSlice(SliceModel*)` to accept it without modification. SmartSDR-specific
signals (`rxAntennaChanged`, `modeListChanged`, etc.) are simply never emitted.

---

## Sample Rate

Default: **384 kHz** (provides ~350 kHz visible spectrum, manageable CPU load).

Stored in `AppSettings` under key `"HpsdrSampleRate"` (flat PascalCase, no path separator,
matching project convention). Changing requires reconnect — sample rate is set in the P2
Start packet, and the FIR decimation filter coefficients are computed at connection time
from this value. Supported values: 48000, 96000, 192000, 384000, 768000, 1536000.

---

## Build Dependencies

`HpsdrDsp` requires **FFTW3** for the FFT path. FFTW3 is currently an optional dependency in
the build system. When `src/hpsdr/` is compiled, FFTW3 must be present; the CMakeLists change
will add `find_package(FFTW3 REQUIRED)` scoped to the HPSDR target (or gate the entire
`src/hpsdr/` subdirectory on `FFTW3_FOUND` and disable it with a status message if absent).
No other new dependencies are introduced.

---

## Injection Points (Existing Files Modified)

| File | Change |
|---|---|
| `src/core/RadioDiscovery.h/.cpp` | Start `HpsdrDiscovery` in parallel; its `radioFound` signals are forwarded to `ConnectionPanel` |
| `src/gui/ConnectionPanel.h/.cpp` | Accept `HpsdrRadioInfo` results; add `RadioType` discriminant to the displayed list; emit `hpsdrConnectRequested(HpsdrRadioInfo)` when user selects an Anan radio |
| `src/gui/MainWindow.cpp` | Connect `hpsdrConnectRequested` from `ConnectionPanel`; instantiate `HpsdrRadio` instead of calling `RadioModel::connectToRadio()` for Anan radios |
| `src/core/AudioEngine.h/.cpp` | Add `feedHpsdrAudio(QByteArray int16Stereo24k)` public slot; writes PCM directly to the output device, bypassing the NR/DSP pipeline (which is FlexRadio-specific), but still applying the existing gain (`m_rxVolume`) and mute (`m_muted`) stage so volume and mute controls work correctly |
| `src/gui/SpectrumWidget.h/.cpp` | Add `feedFftBins(quint64 centerHz, float bandwidthHz, QVector<float> binsDbfs)` public slot; converts Hz arguments to MHz, calls `setFrequencyRange(centerMhz, bandwidthMhz)`, then calls `updateSpectrum(bins)`. The bins are dBFS (not dBm); Y-axis calibration is left uncalibrated for the RX MVP and tracked as a follow-up item |

---

## UI Behaviour

- Anan radios appear in `ConnectionPanel` alongside FlexRadio radios with a distinct
  label/icon indicating HPSDR type
- Connecting to an Anan radio activates the existing `RxApplet` and `VfoWidget` wired to
  `HpsdrSliceModel` (which inherits `SliceModel` — no widget changes needed)
- Controls not applicable to HPSDR RX (TX power, AGC level, panadapter count, band
  stacking, spots) are not wired to `HpsdrSliceModel` and remain greyed out
- Volume control works through `AudioEngine`'s existing gain stage
- Waterfall works automatically — it consumes the same FFT data path as the spectrum

---

## Threading Model

| Thread | Responsibility |
|---|---|
| Main (Qt event loop) | UI, `HpsdrSliceModel`, lifecycle signals |
| P2 receive thread | UDP recv loop in `HpsdrP2Connection`, emits `iqReady` |
| DSP thread | `HpsdrDsp` runs FFT + demodulation, emits `fftReady` + `pcmReady` |
| Audio output thread | Existing `AudioEngine` output thread, unchanged |

These two new threads are active only while an Anan radio is connected. They do not run
when a FlexRadio is connected. `docs/architecture-pipelines.md` should be updated in the
same PR to reflect the conditional thread count (11 for FlexRadio, 13 for Anan).

Cross-thread communication uses Qt auto-connected signals (queued across thread boundaries)
and `std::atomic` for DSP parameter updates from the main thread. Specifically,
`HpsdrDsp::pcmReady` must connect to `AudioEngine::feedHpsdrAudio` via `Qt::AutoConnection`
(never a direct call) so that audio data is correctly marshalled onto the `AudioEngine`
worker thread.

---

## File Layout

```
src/hpsdr/
  HpsdrDiscovery.h/.cpp
  HpsdrRadio.h/.cpp
  HpsdrP2Connection.h/.cpp
  HpsdrDsp.h/.cpp
  HpsdrSliceModel.h/.cpp
  HpsdrRadioInfo.h          (plain struct, no .cpp needed)
```

Add `src/hpsdr/` as a subdirectory in `CMakeLists.txt`, gated on FFTW3 availability.

---

## Out of Scope / Future Work

- TX path (control packets, IQ uplink, PTT)
- Multiple simultaneous receivers (P2 supports up to 12)
- `IRadioBackend` abstraction (prerequisite for clean TX + multi-radio)
- Auto-reconnect on P2 packet loss / sequence number gap detection
- Serial PTT / CW keying
- Band stacking, spots, DX cluster integration
- SmartLink / WAN connection for Anan
