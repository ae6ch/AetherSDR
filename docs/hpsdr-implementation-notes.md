# HPSDR Anan RX — Implementation Notes

Branch: `feature/hpsdr-anan-rx`  
Last updated: 2026-04-11  
Status: **RX functional** — audio, spectrum, waterfall, S-meter, CW decoder implemented

---

## What is working (as of 2026-04-11)

| Feature | Status | Notes |
|---|---|---|
| P1 (Metis) discovery | ✅ | UDP broadcast on port 1024, 60-byte reply, MAC at bytes 3–8 |
| P2 discovery | ✅ | OpenHPSDR P2 handshake |
| IQ stream RX | ✅ | 384 kHz, 24-bit, 126 samples/packet |
| USB/LSB/AM/CW demod | ✅ | NCO + FIR + 8:1 decimate + biquad voice passband |
| Spectrum / FFT | ✅ | 4096-point Hann FFT @ 30 fps, +25 dB calibration offset |
| Waterfall | ✅ | Uses FFT bins via pushWaterfallRow (no native tiles for HPSDR) |
| Audio output | ✅ | 48 kHz stereo int16, direct or 2:1 decimated for 24 kHz sinks |
| S-meter | ✅ | 100 ms IQ RMS → levelReady → SMeterWidget |
| CW decoder | ✅ | ggmorse via feedAudio48k (2:1 decimate + stereo→mono) |
| Click-to-tune | ✅ | Via VFO widget / HpsdrSliceModel |
| Mode/filter controls | ✅ | Routed to DSP and/or hardware |
| RF gain / preamp / ATT | ✅ | Anan 10E discrete steps mapped from dB slider |
| Antenna selection | ✅ | ANT1/2/3/EXT → TX antenna + RX input routing |

---

## Architecture

```
HpsdrRadio (main thread)
  ├── HpsdrP1Connection / HpsdrP2Connection  ← UDP socket, IQ RX, hardware cmds
  ├── HpsdrSliceModel                        ← frequency, mode, filter, gain
  └── HpsdrDsp (m_dspThread)                 ← NCO, FIR, decimate, demod, FFT
        ├── fftReady  → HpsdrRadio → SpectrumWidget::feedFftBins
        ├── pcmReady  → HpsdrRadio → AudioEngine::feedHpsdrAudio
        │                         → CwDecoder::feedAudio48k
        └── levelReady → HpsdrRadio → SMeterWidget::setLevel
```

---

## DSP signal chain (HpsdrDsp::processIq)

1. **s24beToFloat** — 24-bit big-endian I and Q → float [-1, 1]
2. **FFT path** — accumulate Hann-windowed IQ into 4096-point buffer → runFft() every m_fftSkip frames
3. **NCO** — mix to baseband at slice offset: `tI = I*cos + Q*sin`, `tQ = -I*sin + Q*cos`
4. **FIR LPF** — 128-tap Hamming-windowed sinc, fc = 0.9/decimRatio, separate I/Q delay lines
5. **8:1 decimate** — keep every 8th sample (384k → 48k)
6. **S-meter accumulate** — `m_levelAccum += fI² + fQ²`; emit levelReady every 4800 samples (100 ms)
7. **Demodulate** — USB: `I+Q`, LSB: `I-Q`, AM: `√(I²+Q²)`, CW: USB path
8. **Voice bandpass** — HP biquad 300 Hz, LP biquad 3 kHz (both 2nd-order Butterworth)
9. **AF gain** — `audio × m_afGain` (UI=50 → ×10000)
10. **Squelch** — EMA RMS gate
11. **Pan + float→int16** — constant-power sin/cos law, clamp to int16 range
12. **Emit** — 480 stereo frames (10 ms) per chunk via pcmReady

---

## Calibration constants (HpsdrDsp.cpp)

These are estimates based on Anan 10E ADC characteristics. Tune against a calibrated signal source.

```cpp
static constexpr float kCalibDbm    = 10.0f;  // S-meter: dBFS → dBm (ADC FS ≈ +10 dBm)
static constexpr float kFftCalibDbm = 25.0f;  // Waterfall: dBFS → display dBm
// AF gain: m_afGain = gain * 200.0f  (UI=50 → ×10000)
// Default: float m_afGain{10000.0f}
```

**Tuning guide:**
- `kFftCalibDbm`: increase if waterfall looks too dark; decrease if everything is bright/clipped
- `kCalibDbm`: increase if S-meter reads too low vs. a known signal; decrease if too high
- `m_afGain` formula: reduce multiplier if audio clips on normal signals; increase if still too quiet

---

## Biquad filter implementation

2nd-order Butterworth via bilinear transform, Direct Form II Transposed (numerically stable).

```
Low-pass:  wc = tan(π·fc/fs),  k = 1 + √2·wc + wc²
           b0 = b2 = wc²/k,  b1 = 2wc²/k
           a1 = 2(wc²-1)/k,  a2 = (1 - √2·wc + wc²)/k

High-pass: same wc, k
           b0 = b2 = 1/k,  b1 = -2/k
           a1 and a2 same as LP

Apply:  y = b0·x + z1;  z1 = b1·x - a1·y + z2;  z2 = b2·x - a2·y
```

`setFilterBandwidth(low, high)` updates the LP cutoff to `max(3000, max(|low|, |high|))` Hz.
HP stays fixed at 300 Hz.

---

## Known limitations / next steps

### AGC (high priority)
Without AGC, the AF gain slider is the only level control. Strong signals (S9+) clip at default
settings; very weak signals may still be inaudible at UI=100. A simple fast-attack / slow-decay
AGC before the AF gain stage would fix this.

### TX path (deferred)
Not implemented. The `HpsdrConnection` interface has stub methods for TX but no wiring.

### Auto-reconnect (deferred)
Watchdog timer detects connection loss but no reconnect logic. User must manually reconnect.

### Multiple receivers (deferred)
HpsdrP1Connection supports only receiver 0. The P2 protocol supports multiple receivers but
is not exposed in the current UI.

### Calibration validation
All three calibration constants (kCalibDbm, kFftCalibDbm, AF gain) need validation against
a calibrated signal source (e.g., a signal generator at a known dBm level).

---

## Git history summary

Key commits on `feature/hpsdr-anan-rx`:
- `feat(hpsdr): DSP audio quality, S-meter, CW decoder, waterfall` — current session
- `fix(hpsdr): hide SmartSDR AppletPanel when HPSDR radio is connected`
- `fix(hpsdr): fix click-to-tune and add VFO widget for HPSDR`
- `fix(hpsdr): route activeSlice() to HpsdrSliceModel when HPSDR connected`
- `feat(hpsdr): add Protocol 1 (Metis) support alongside existing P2`
- `hpsdr: MainWindow — full HPSDR connect/disconnect flow (Task 11)`

Branch pushed to: `origin` (ae6ch/AetherSDR)  
PR to upstream (ten9876/AetherSDR): **not yet created**
