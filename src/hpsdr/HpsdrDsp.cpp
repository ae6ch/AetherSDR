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
    m_fftSkip    = std::max(1, (rate / FFT_SIZE) / FFT_FPS_TARGET);
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
        if (m_fftAccumPos >= FFT_SIZE) {
            m_fftAccumPos = 0;
            ++m_fftSkipCount;
            if (m_fftSkipCount >= m_fftSkip) {
                m_fftSkipCount = 0;
                runFft();
            }
        }

        // ── Audio demod path: added in Task 7 ──
        Q_UNUSED(iSam)  // suppress unused-variable warnings until Task 7
        Q_UNUSED(qSam)
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
