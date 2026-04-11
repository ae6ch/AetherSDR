# HPSDR / Anan RX Support — Design Spec

**Date:** 2026-04-10  
**Scope:** RX-only support for Apache Labs Anan 10E via OpenHPSDR Protocol 2 (P2)  
**Approach:** Self-contained `HpsdrRadio` module; zero changes to SmartSDR code paths

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
- Zero regression risk to the existing SmartSDR/FlexRadio path

## Non-Goals (deferred)

- TX / transmit path
- Multiple simultaneous receivers
- Band stacking, spots, SmartLink, serial PTT
- Simultaneous Flex + Anan connection

---

## Architecture

All new code lives in `src/hpsdr/`. Existing SmartSDR files are changed only at four
well-defined injection points.

```
MainWindow
  ├── RadioModel          (existing — FlexRadio path, unchanged)
  │     ├── PanadapterStream
  │     ├── CommandParser
  │     └── AudioEngine ◄─────────────────────────────────────┐
  │                                                            │ feedPcmAudio()
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
Listens on UDP port 1024 for HPSDR P2 discovery broadcasts. Parses each packet to
extract IP address, MAC, firmware version, and board ID (used to identify Anan 10E).
Emits `radioFound(HpsdrRadioInfo)` signals. Run in parallel with the existing
`RadioDiscovery` so both radio types appear in the same connection list.

### HpsdrRadio
Top-level coordinator. Owns `HpsdrP2Connection`, `HpsdrDsp`, and `HpsdrSliceModel`.
Manages connection lifecycle (start, stop, error recovery). Wires DSP output to
`AudioEngine::feedPcmAudio()` and `SpectrumWidget::feedFftBins()`. Instantiated by
`MainWindow` when the user connects to an Anan radio.

### HpsdrP2Connection
Single UDP socket on port 1024 for both control and IQ data.

- **Start packet:** initiates IQ streaming; sets sample rate and receiver count (1)
- **Control packets:** sent periodically (~1ms interval) carrying RX frequency,
  sample rate, preamp/attenuator flags, dither/random settings
- **IQ data packets:** received from radio; each contains a sequence number and
  blocks of 24-bit signed big-endian IQ samples
- **Stop packet:** sent on disconnect to halt streaming

Runs IQ reception on a dedicated thread. Emits `iqReady(QByteArray)` to `HpsdrDsp`.

### HpsdrDsp
Processes raw IQ on a dedicated DSP thread. Two parallel paths:

**FFT path (spectrum):**
1. Accumulate 4096 samples
2. Apply Hann window
3. FFT via FFTW (already a project dependency)
4. Convert to dBFS
5. Emit `fftReady(quint64 centerHz, QVector<float> bins)` at ~30 fps

**Audio path (demodulation):**
1. Frequency translate: mix IQ down to baseband centered on slice frequency
2. FIR low-pass filter + decimate 384 kHz → 48 kHz (÷8)
3. Demodulate by mode:
   - **LSB/USB:** take real part of complex signal after frequency shift
   - **AM:** envelope detection (magnitude of complex sample)
   - **CW:** narrow bandpass filter + BFO offset
4. Emit `pcmReady(QByteArray float32Mono48k)`

DSP is implemented in C++ with no new library dependencies beyond FFTW. FIR
coefficients are pre-computed at startup based on the configured sample rate.
Cross-thread parameter updates (frequency, mode) use `std::atomic` per the
project's existing pattern for audio-thread-safe parameters.

### HpsdrSliceModel
Thin QObject model holding: frequency (Hz), mode (LSB/USB/AM/CW), filter low/high
(Hz). Exposes the same property names as `SliceModel` for the overlapping subset so
the existing `SliceControlWidget` can be wired to it without modification. Changes
to frequency or mode update `HpsdrDsp` atomically and schedule the updated values
into the next outgoing control packet.

---

## Sample Rate

Default: **384 kHz** (provides ~350 kHz visible spectrum, manageable CPU load).

Stored in `AppSettings` under key `"Hpsdr/SampleRate"`. Changing requires reconnect
(sample rate is set in the P2 Start packet). The DSP decimation ratio and FIR filter
coefficients are computed at connection time from this value, making future rate
changes (e.g. 192 kHz, 768 kHz) a matter of changing the setting and reconnecting.

---

## Injection Points (Existing Files Modified)

| File | Change |
|---|---|
| `src/core/RadioDiscovery.h/.cpp` | Start `HpsdrDiscovery` in parallel; merge `HpsdrRadioInfo` results into the existing discovery list with a `RadioType::Hpsdr` tag |
| `src/gui/MainWindow.cpp` | On connect: check `RadioType`; instantiate `HpsdrRadio` instead of calling `RadioModel::connectToRadio()` for Anan radios |
| `src/core/AudioEngine.h/.cpp` | Add `feedPcmAudio(QByteArray pcm)` public slot; pushes float32 mono 48 kHz PCM into the existing output device via the existing gain stage |
| `src/gui/SpectrumWidget.h/.cpp` | Add `feedFftBins(quint64 centerHz, QVector<float> bins)` public slot; renders FFT data the same way as VITA-49 sourced data |

No other existing files are modified.

---

## UI Behaviour

- Anan radios appear in `ConnectionPanel` alongside FlexRadio radios with a distinct
  label/icon indicating HPSDR type
- Connecting to an Anan radio activates the existing `SliceControlWidget` wired to
  `HpsdrSliceModel` (frequency display, mode buttons, filter edges)
- Controls not applicable to HPSDR RX (TX power, AGC level, panadapter count, band
  stacking, spots) remain greyed out — no special casing needed since they are wired
  only to `RadioModel` / `SliceModel`
- Volume control works through `AudioEngine`'s existing gain stage
- Waterfall works automatically — it consumes the same FFT data fed to `SpectrumWidget`

---

## Threading Model

| Thread | Responsibility |
|---|---|
| Main (Qt event loop) | UI, `HpsdrSliceModel`, lifecycle signals |
| P2 receive thread | UDP recv loop in `HpsdrP2Connection`, emits `iqReady` |
| DSP thread | `HpsdrDsp` runs FFT + demodulation, emits `fftReady` + `pcmReady` |
| Audio output thread | Existing `AudioEngine` output thread, unchanged |

Cross-thread communication uses Qt auto-connected signals (queued across thread
boundaries) and `std::atomic` for DSP parameter updates from the main thread, per
the project's existing convention.

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

Add `src/hpsdr/` as a subdirectory in `CMakeLists.txt`.

---

## Out of Scope / Future Work

- TX path (control packets, IQ uplink, PTT)
- Multiple simultaneous receivers (P2 supports up to 12)
- Proper `IRadioBackend` abstraction (prerequisite for clean TX + multi-radio)
- Serial PTT / CW keying
- Band stacking, spots, DX cluster integration
- SmartLink / WAN connection for Anan
