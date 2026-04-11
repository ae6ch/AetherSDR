// src/hpsdr/HpsdrDsp.cpp
#include "HpsdrDsp.h"
#include "core/LogManager.h"
#include <cmath>
#include <algorithm>

namespace AetherSDR {

static constexpr float kPi    = static_cast<float>(M_PI);
static constexpr float kTwoPi = 2.0f * kPi;

HpsdrDsp::HpsdrDsp(QObject* parent) : QObject(parent)
{
    initFft();
    computeFirCoeffs();
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

void HpsdrDsp::setMode(const QString& mode)
{
    if      (mode == "USB") { m_mode.store(0); }
    else if (mode == "LSB") { m_mode.store(1); }
    else if (mode == "AM")  { m_mode.store(2); }
    else if (mode == "CW")  { m_mode.store(3); }
    else { qCWarning(lcHpsdr) << "HpsdrDsp: unknown mode" << mode; }
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
        if (m_ncoPhase > kPi) { m_ncoPhase -= kTwoPi; }

        // 2. FIR low-pass filter (separate delay lines for I and Q)
        auto applyFir = [&](QVector<float>& state, float input) -> float {
            std::copy_backward(state.begin(), state.end() - 1, state.end());
            state[0] = input;
            float acc = 0.0f;
            for (int k = 0; k < m_firCoeffs.size(); ++k) { acc += m_firCoeffs[k] * state[k]; }
            return acc;
        };
        float fI = applyFir(m_firStateI, tI);
        float fQ = applyFir(m_firStateQ, tQ);

        // 3. Decimate: keep every m_decimRatio-th sample (e.g. 384k → 48k at ratio 8)
        if (++m_audioAccumPos < m_decimRatio) { continue; }
        m_audioAccumPos = 0;

        // 4. Demodulate (analytic signal approach):
        float audio = 0.0f;
        switch (m_mode.load()) {
            case 0: audio =  fI + fQ; break;                               // USB: Re(I + jQ)
            case 1: audio =  fI - fQ; break;                               // LSB: Re(I - jQ)
            case 2: audio =  std::sqrt(fI * fI + fQ * fQ); break;         // AM: envelope
            case 3: audio =  fI + fQ; break;                               // CW: USB path (BFO offset applied via NCO)
            default: break;
        }

        // 5. Resample 48k → 24k: simple 2-sample average (upgrade to half-band FIR later)
        m_audio48kBuf.append(audio);
        if (m_audio48kBuf.size() < 2) { continue; }
        float s24k = (m_audio48kBuf[0] + m_audio48kBuf[1]) * 0.5f;
        m_audio48kBuf.clear();

        // 6. float → int16 stereo (mono duplicated to L and R channels)
        qint16 s16 = static_cast<qint16>(
            std::clamp(s24k * 32767.0f, -32768.0f, 32767.0f));
        m_audioOutBuf.append(reinterpret_cast<const char*>(&s16), 2);  // L channel
        m_audioOutBuf.append(reinterpret_cast<const char*>(&s16), 2);  // R channel

        // 7. Emit in 10 ms chunks: 10ms at 24kHz stereo
        //    24000 Hz × 10 ms × 2 ch × 2 bytes = 960 bytes = 240 stereo frames × 4 bytes/frame
        constexpr int kEmitBytes = 240 * 4;
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
        dbfs[k] = (mag2 > 0.0f) ? 10.0f * std::log10(mag2) : -140.0f;
    }

    // FFT-shift: swap first half and second half in-place
    std::rotate(dbfs.begin(), dbfs.begin() + FFT_SIZE / 2, dbfs.end());

    emit fftReady(static_cast<quint64>(m_centerHz.load()),
                  static_cast<float>(m_sampleRate),
                  dbfs);
}

} // namespace AetherSDR
