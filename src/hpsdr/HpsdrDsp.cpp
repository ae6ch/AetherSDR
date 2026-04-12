// src/hpsdr/HpsdrDsp.cpp
#include "HpsdrDsp.h"
#include "core/LogManager.h"
#include <cmath>
#include <algorithm>

namespace AetherSDR {

static constexpr float kPi       = static_cast<float>(M_PI);
static constexpr float kTwoPi   = 2.0f * kPi;
static constexpr float kSqrt2   = 1.41421356f;
// Converts raw dBFS (0 = ADC full-scale) to approximate dBm for Anan 10E.
// S9 on HF = -73 dBm. ADC full-scale ≈ +10 dBm at the antenna connector
// (includes LNA + ADC headroom). Tune kCalibDbm if the displayed S-unit is
// systematically high or low; a calibrated signal source is the best reference.
static constexpr float kCalibDbm = 10.0f;

// FFT bin offset: shifts dBFS values into the dBm range the SpectrumWidget
// expects (display defaults: top = -50 dBm, floor = -150 dBm, effective color
// range ≈ -86 to -41 dBm with default black-level/gain settings).
//
// The Hann window coherent gain of 0.5 costs 6 dB on top of the dBFS level,
// so a -90 dBFS IQ signal produces a peak FFT bin at ≈ -96 dBFS.  Adding
// kFftCalibDbm ≈ 25 shifts that to ≈ -71 dBm, landing in the visible range
// and matching the on-air feel of a SmartSDR display.
//
// Tune upward if the display looks too dark; downward if it clips (all bright).
static constexpr float kFftCalibDbm = 25.0f;

HpsdrDsp::HpsdrDsp(QObject* parent) : QObject(parent)
{
    initFft();
    computeFirCoeffs();
    // Default voice bandpass: 300 Hz HP + 3000 Hz LP at 48 kHz.
    // setFilterBandwidth() updates these when the slice filter width changes.
    m_hpBq = makeHpButtw2(300.f,  48000.f);
    m_lpBq = makeLpButtw2(3000.f, 48000.f);
    qCInfo(lcHpsdr) << "HpsdrDsp: initialised, FFT_SIZE=" << FFT_SIZE
                    << "sample rate=" << m_sampleRate;
}

HpsdrDsp::~HpsdrDsp()
{
    destroyFft();
}

void HpsdrDsp::setSampleRate(int rate)
{
    m_sampleRate = rate;
    m_decimRatio = rate / 48000;
    m_fftSkip    = std::max(1, static_cast<int>(std::round(
                       static_cast<double>(rate) / FFT_SIZE / FFT_FPS_TARGET)));
    computeFirCoeffs();
    updateNco();
    qCInfo(lcHpsdr) << "HpsdrDsp: sample rate set to" << rate
                    << "decimRatio=" << m_decimRatio << "fftSkip=" << m_fftSkip;
}

void HpsdrDsp::setCenterFrequency(double hz)
{
    m_centerHz.store(hz);
    updateNco();
}

void HpsdrDsp::setRxFrequency(double hz)
{
    m_rxHz.store(hz);
    updateNco();
}

void HpsdrDsp::setAfGain(float gain)
{
    // gain: 0–100 from UI (SliceModel::audioGain range).
    // Scale so that 50 (default) → ×10000, calibrated for typical HF signal levels.
    // HPSDR IQ is raw 24-bit ADC data; HF signals typically land at -100 to -80 dBFS,
    // so ×10000 (at UI=50) puts an S5 signal (~-85 dBFS) at roughly -6 dBFS out.
    // Strong signals (S9+) will clip without AGC — use the slider to reduce gain.
    m_afGain = gain * 200.0f;
}
void HpsdrDsp::setMute(bool mute)             { m_mute = mute; }
void HpsdrDsp::setAudioPan(float pan)         { m_audioPan = std::clamp(pan, -1.0f, 1.0f); }

void HpsdrDsp::setSquelch(bool enabled, float threshold)
{
    m_squelchEnabled   = enabled;
    m_squelchThreshold = threshold;
}

void HpsdrDsp::setFilterBandwidth(int lowHz, int highHz)
{
    m_filterLowHz  = lowHz;
    m_filterHighHz = highHz;

    // Derive post-demodulation voice bandpass from filter edges (Hz offsets from carrier;
    // negative for LSB). LP cutoff = wider edge; HP always 300 Hz to pass voice.
    // Minimum LP = 3000 Hz so a default/symmetric init value (e.g. ±1500) doesn't over-narrow.
    const float lpHz = std::max(3000.f,
                                static_cast<float>(std::max(std::abs(lowHz),
                                                            std::abs(highHz))));
    m_lpBq = makeLpButtw2(lpHz, 48000.f);
    // HP stays at 300 Hz; only recompute if caller explicitly narrows it
    // (the 300 Hz default set in the constructor is already appropriate for USB/LSB/AM/CW)
}

void HpsdrDsp::setMode(const QString& mode)
{
    if      (mode == "USB") { m_mode.store(0); }
    else if (mode == "LSB") { m_mode.store(1); }
    else if (mode == "AM")  { m_mode.store(2); }
    else if (mode == "CW")  { m_mode.store(3); }
    else { qCWarning(lcHpsdr) << "HpsdrDsp: unknown mode" << mode; }
}

// ── Biquad helpers ────────────────────────────────────────────────────────────

float HpsdrDsp::applyBiquad(Biquad& bq, float x) noexcept
{
    // Direct Form II Transposed: numerically stable, minimal state.
    const float y = bq.b0 * x + bq.z1;
    bq.z1 = bq.b1 * x - bq.a1 * y + bq.z2;
    bq.z2 = bq.b2 * x - bq.a2 * y;
    return y;
}

HpsdrDsp::Biquad HpsdrDsp::makeLpButtw2(float fcHz, float fsHz) noexcept
{
    // 2nd-order Butterworth low-pass via bilinear transform.
    const float wc  = std::tan(kPi * fcHz / fsHz);
    const float wc2 = wc * wc;
    const float k   = 1.f + kSqrt2 * wc + wc2;
    Biquad bq;
    bq.b0 = wc2 / k;
    bq.b1 = 2.f * wc2 / k;
    bq.b2 = bq.b0;
    bq.a1 = 2.f * (wc2 - 1.f) / k;
    bq.a2 = (1.f - kSqrt2 * wc + wc2) / k;
    return bq;  // z1, z2 zero-initialised (state cleared on coefficient change)
}

HpsdrDsp::Biquad HpsdrDsp::makeHpButtw2(float fcHz, float fsHz) noexcept
{
    // 2nd-order Butterworth high-pass via bilinear transform.
    const float wc  = std::tan(kPi * fcHz / fsHz);
    const float wc2 = wc * wc;
    const float k   = 1.f + kSqrt2 * wc + wc2;
    Biquad bq;
    bq.b0 =  1.f / k;
    bq.b1 = -2.f / k;
    bq.b2 =  bq.b0;
    bq.a1 = 2.f * (wc2 - 1.f) / k;
    bq.a2 = (1.f - kSqrt2 * wc + wc2) / k;
    return bq;
}

void HpsdrDsp::updateNco()
{
    double offset = m_rxHz.load() - m_centerHz.load();
    m_ncoPhaseInc = static_cast<float>(kTwoPi * offset / m_sampleRate);
}

void HpsdrDsp::initFft()
{
    m_fftIn  = fftwf_alloc_complex(FFT_SIZE);
    m_fftOut = fftwf_alloc_complex(FFT_SIZE);
    // FFTW_FORWARD = negative exponent = correct for baseband IQ (e^{-j2πkn/N})
    m_fftPlan = fftwf_plan_dft_1d(FFT_SIZE, m_fftIn, m_fftOut, FFTW_FORWARD, FFTW_ESTIMATE);

    // Hann window: w[n] = 0.5 * (1 - cos(2π·n / (N-1)))
    m_window.resize(FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; ++i) {
        m_window[i] = 0.5f * (1.0f - std::cos(kTwoPi * i / (FFT_SIZE - 1)));
    }
}

void HpsdrDsp::destroyFft()
{
    if (m_fftPlan) { fftwf_destroy_plan(m_fftPlan); m_fftPlan = nullptr; }
    if (m_fftIn)   { fftwf_free(m_fftIn);  m_fftIn  = nullptr; }
    if (m_fftOut)  { fftwf_free(m_fftOut); m_fftOut = nullptr; }
}

void HpsdrDsp::computeFirCoeffs()
{
    constexpr int kTaps = 128;
    m_firCoeffs.resize(kTaps);
    m_firStateI.fill(0.0f, kTaps);
    m_firStateQ.fill(0.0f, kTaps);

    // Hamming-windowed sinc low-pass filter, normalised cutoff = 0.9 / decimRatio
    // Cutoff at 90% of the decimated Nyquist to avoid aliasing with a small guard band
    const float fc  = 0.9f / static_cast<float>(m_decimRatio);
    float sum = 0.0f;
    for (int i = 0; i < kTaps; ++i) {
        const float n    = static_cast<float>(i) - (kTaps - 1) / 2.0f;
        const float sinc = (n == 0.0f) ? 1.0f
                         : std::sin(kPi * fc * n) / (kPi * fc * n);
        const float win  = 0.54f - 0.46f * std::cos(kTwoPi * i / (kTaps - 1));
        m_firCoeffs[i]   = sinc * win;
        sum += m_firCoeffs[i];
    }
    // Normalise so DC gain = 1
    for (float& c : m_firCoeffs) { c /= sum; }
}

// Convert 24-bit signed big-endian triplet to float in [-1, 1]
static float s24beToFloat(const char* p)
{
    qint32 v = (static_cast<quint8>(p[0]) << 16)
             | (static_cast<quint8>(p[1]) <<  8)
             |  static_cast<quint8>(p[2]);
    if (v & 0x800000) { v |= static_cast<qint32>(0xFF000000); }  // sign-extend
    return static_cast<float>(v) / 8388608.0f;  // 2^23
}

void HpsdrDsp::processIq(const QByteArray& raw)
{
    // Each sample = 6 bytes: 3 bytes I (24-bit signed big-endian) then 3 bytes Q
    // Verify I-before-Q byte order in Thetis protocol2.cs ProcessPacket()
    const int numSamples = raw.size() / 6;
    const char* p = raw.constData();

    for (int i = 0; i < numSamples; ++i, p += 6) {
        const float iSam = s24beToFloat(p);
        const float qSam = s24beToFloat(p + 3);

        // ── FFT path: accumulate windowed IQ into the complex FFT input buffer ──
        if (m_fftAccumPos < FFT_SIZE) {
            m_fftIn[m_fftAccumPos][0] = iSam * m_window[m_fftAccumPos];
            m_fftIn[m_fftAccumPos][1] = qSam * m_window[m_fftAccumPos];
            ++m_fftAccumPos;
        }
        // Separate if (not else-if): the last sample of a frame both fills the
        // buffer AND triggers runFft() in the same iteration. This is intentional.
        if (m_fftAccumPos >= FFT_SIZE) {
            m_fftAccumPos = 0;
            ++m_fftSkipCount;
            if (m_fftSkipCount >= m_fftSkip) {
                m_fftSkipCount = 0;
                runFft();
            }
        }

        // ── Audio demod path ──────────────────────────────────────────────
        // 1. Frequency translate: mix IQ down to baseband at slice frequency
        float cosP =  std::cos(m_ncoPhase);
        float sinP =  std::sin(m_ncoPhase);
        float tI   =  iSam * cosP + qSam * sinP;
        float tQ   = -iSam * sinP + qSam * cosP;
        m_ncoPhase += m_ncoPhaseInc;
        // Wrap phase to [-π, π] to avoid float precision drift
        if (m_ncoPhase >  kPi) { m_ncoPhase -= kTwoPi; }
        if (m_ncoPhase < -kPi) { m_ncoPhase += kTwoPi; }

        // 2. FIR low-pass filter (separate delay lines for I and Q)
        auto applyFir = [&](QVector<float>& state, float input) -> float {
            std::copy_backward(state.begin(), state.end() - 1, state.end());
            state[0] = input;
            float acc = 0.0f;
            const int taps = static_cast<int>(m_firCoeffs.size());
            for (int k = 0; k < taps; ++k) { acc += m_firCoeffs[k] * state[k]; }
            return acc;
        };
        float fI = applyFir(m_firStateI, tI);
        float fQ = applyFir(m_firStateQ, tQ);

        // 3. Decimate: keep every m_decimRatio-th sample (e.g. 384k → 48k at ratio 8)
        if (++m_audioAccumPos < m_decimRatio) { continue; }
        m_audioAccumPos = 0;

        // 3b. S-meter: accumulate IQ power at 48 kHz (post-decimate, pre-demod, pre-AF gain).
        //     Uses filtered fI/fQ so the level estimate is bandwidth-limited to the
        //     decimated passband (~24 kHz each side), not the full ADC range.
        m_levelAccum += fI * fI + fQ * fQ;
        ++m_levelCount;
        if (m_levelCount >= kLevelBlock) {
            const float meanPow = m_levelAccum / static_cast<float>(m_levelCount);
            const float dbfs    = (meanPow > 0.0f) ? 10.0f * std::log10(meanPow) : -140.0f;
            emit levelReady(dbfs + kCalibDbm);
            m_levelAccum = 0.0f;
            m_levelCount = 0;
        }

        // 4. Demodulate (analytic signal approach):
        float audio = 0.0f;
        switch (m_mode.load()) {
            case 0: audio =  fI + fQ; break;                               // USB: Re(I + jQ)
            case 1: audio =  fI - fQ; break;                               // LSB: Re(I - jQ)
            case 2: audio =  std::sqrt(fI * fI + fQ * fQ); break;         // AM: envelope
            case 3: audio =  fI + fQ; break;                               // CW: USB path (BFO offset applied via NCO)
            default: break;
        }

        // 4b. Voice bandpass (48 kHz post-demodulation):
        //     HP at 300 Hz removes DC offset and sub-bass rumble.
        //     LP tracks the slice filter-width (setFilterBandwidth); min 3 kHz.
        //     Removing the ~18 kHz of broadband noise outside the voice band
        //     improves SNR by ~9 dB and makes the audio sound clearer.
        audio = applyBiquad(m_hpBq, audio);
        audio = applyBiquad(m_lpBq, audio);

        // 5a. AF gain: Anan 10E 24-bit IQ at noise floor ≈ 1e-5; needs ×1000 to clear int16 quantisation
        audio *= m_afGain;

        // 5b. Squelch: exponential-moving-average RMS; gate output when below threshold
        m_squelchRms = 0.999f * m_squelchRms + 0.001f * (audio * audio);
        if (m_squelchEnabled && m_squelchRms < m_squelchThreshold * m_squelchThreshold) {
            audio = 0.0f;
        }

        // 5c. Mute
        if (m_mute) { audio = 0.0f; }

        // 5. float → int16 stereo with audio pan (output at 48kHz — FIR+decimate already
        //    band-limits to < 24 kHz so the sink's own resampler can downconvert cleanly
        //    if needed; avoids the quality loss of an in-DSP 2-tap average).
        //    pan: -1.0=full-L, 0=centre, +1.0=full-R (constant-power sin/cos law)
        const float angle  = (m_audioPan + 1.0f) * (kPi / 4.0f);  // [0, π/2]
        const float gainL  = std::cos(angle);
        const float gainR  = std::sin(angle);
        qint16 sL = static_cast<qint16>(std::clamp(audio * gainL * 32767.0f, -32768.0f, 32767.0f));
        qint16 sR = static_cast<qint16>(std::clamp(audio * gainR * 32767.0f, -32768.0f, 32767.0f));
        m_audioOutBuf.append(reinterpret_cast<const char*>(&sL), 2);  // L channel
        m_audioOutBuf.append(reinterpret_cast<const char*>(&sR), 2);  // R channel

        // 6. Emit in 10 ms chunks at 48kHz stereo
        //    48000 Hz × 10 ms × 2 ch × 2 bytes = 1920 bytes = 480 stereo frames × 4 bytes/frame
        constexpr int kEmitBytes = 480 * 4;
        if (m_audioOutBuf.size() >= kEmitBytes) {
            emit pcmReady(m_audioOutBuf.left(kEmitBytes));
            m_audioOutBuf.remove(0, kEmitBytes);
        }
    }
}

void HpsdrDsp::runFft()
{
    fftwf_execute(m_fftPlan);

    // Complex FFT output layout (FFTW_FORWARD):
    //   bin 0        → DC
    //   bins 1..N/2-1  → positive frequencies (above centre)
    //   bin N/2        → Nyquist
    //   bins N/2+1..N-1 → negative frequencies (below centre, aliased)
    //
    // FFT-shift: rotate by N/2 so that DC is in the centre of the output array.
    // After shift: index 0 = most negative freq, index N/2 = DC, index N-1 = most positive freq.
    // This matches the display convention where low frequencies are on the left.
    const float normSq = 1.0f / (static_cast<float>(FFT_SIZE) * static_cast<float>(FFT_SIZE));
    QVector<float> dbfs(FFT_SIZE);
    for (int k = 0; k < FFT_SIZE; ++k) {
        const float re   = m_fftOut[k][0];
        const float im   = m_fftOut[k][1];
        const float mag2 = (re * re + im * im) * normSq;
        dbfs[k] = (mag2 > 0.0f) ? 10.0f * std::log10(mag2) + kFftCalibDbm : -140.0f + kFftCalibDbm;
    }

    // FFT-shift: swap first half and second half in-place
    std::rotate(dbfs.begin(), dbfs.begin() + FFT_SIZE / 2, dbfs.end());

    emit fftReady(static_cast<quint64>(m_centerHz.load()),
                  static_cast<float>(m_sampleRate),
                  dbfs);
}

} // namespace AetherSDR
