#pragma once
// src/hpsdr/HpsdrDsp.h
// On-host DSP for HPSDR: complex FFT for spectrum + NCO/FIR/demod for audio.
#include <QObject>
#include <QVector>
#include <QString>
#include <atomic>
#include <fftw3.h>

namespace AetherSDR {

class HpsdrDsp : public QObject {
    Q_OBJECT
public:
    static constexpr int FFT_SIZE       = 4096;
    static constexpr int FFT_FPS_TARGET = 30;

    explicit HpsdrDsp(QObject* parent = nullptr);
    ~HpsdrDsp() override;

    void setSampleRate(int rate);
    void setCenterFrequency(double hz);     // atomic — main thread safe
    void setRxFrequency(double hz);         // atomic — main thread safe
    void setMode(const QString& mode);      // main thread only
    void setAfGain(float gain);             // UI range 0–100; 50 → ×10000 internal scalar
    void setMute(bool mute);               // silence audio output when true
    void setFilterBandwidth(int lowHz, int highHz);  // voice passband edges relative to carrier
    void setSquelch(bool enabled, float threshold);  // threshold: 0.0–1.0 normalised RMS
    void setAudioPan(float pan);            // -1.0=full left, 0.0=centre, +1.0=full right

public slots:
    // Connected via Qt::QueuedConnection from HpsdrP2Connection (different thread)
    void processIq(const QByteArray& raw24bitIq);

signals:
    void fftReady(quint64 centerHz, float bandwidthHz, QVector<float> binsDbfs);
    void pcmReady(const QByteArray& int16Stereo48k);
    // Signal-level estimate (dBm, ~100 ms update rate) — drives SMeterWidget.
    // Calibration offset kCalibDbm converts raw dBFS to approximate dBm for Anan 10E.
    void levelReady(float dbm);

private:
    // ── Voice bandpass biquad (post-demodulation at 48 kHz) ──────────────────
    struct Biquad {
        float b0{1.f}, b1{0.f}, b2{0.f};  // numerator coefficients
        float a1{0.f}, a2{0.f};           // denominator (a0 = 1 normalised)
        float z1{0.f}, z2{0.f};           // Direct Form II transposed state
    };
    void initFft();
    void destroyFft();
    void computeFirCoeffs();
    void updateNco();
    void runFft();

    static float  applyBiquad(Biquad& bq, float x) noexcept;
    static Biquad makeLpButtw2(float fcHz, float fsHz) noexcept;
    static Biquad makeHpButtw2(float fcHz, float fsHz) noexcept;

    // FFT (complex input)
    fftwf_complex* m_fftIn{nullptr};
    fftwf_complex* m_fftOut{nullptr};
    fftwf_plan     m_fftPlan{nullptr};
    QVector<float> m_window;          // Hann window (FFT_SIZE coefficients)
    int            m_fftAccumPos{0};  // number of IQ pairs accumulated so far
    int            m_fftSkip{1};      // emit every m_fftSkip-th completed frame
    int            m_fftSkipCount{0};

    // Audio demod (values used by Task 7; computed here so the header is complete)
    QVector<float> m_firCoeffs;   // low-pass FIR, length 128
    QVector<float> m_firStateI;   // FIR delay line for I
    QVector<float> m_firStateQ;   // FIR delay line for Q
    int            m_decimRatio{8};   // 384k → 48k

    float          m_ncoPhase{0.0f};
    float          m_ncoPhaseInc{0.0f};
    int            m_audioAccumPos{0};
    QByteArray     m_audioOutBuf;  // int16 stereo 48k output

    Biquad         m_hpBq;   // voice high-pass — 300 Hz default, updated by setFilterBandwidth
    Biquad         m_lpBq;   // voice low-pass  — 3000 Hz default, tracks filter-width setting

    float          m_afGain{10000.0f};       // internal scalar; setAfGain(50) → 10000
    bool           m_mute{false};            // silence output when true
    float          m_audioPan{0.0f};         // -1.0=L, 0=centre, +1.0=R
    bool           m_squelchEnabled{false};
    float          m_squelchThreshold{0.01f};// normalised RMS threshold
    float          m_squelchRms{0.0f};       // running RMS estimate

    // S-meter: accumulate post-decimate IQ power; emit levelReady every kLevelBlock samples
    static constexpr int kLevelBlock = 4800;  // 100 ms at 48 kHz
    float          m_levelAccum{0.0f};
    int            m_levelCount{0};
    // Filter bandwidth edges (Hz relative to carrier, stored for future post-decimation filter)
    int            m_filterLowHz{-3000};
    int            m_filterHighHz{3000};

    std::atomic<double> m_centerHz{14225000.0};
    std::atomic<double> m_rxHz{14225000.0};
    std::atomic<int>    m_mode{0};   // 0=USB 1=LSB 2=AM 3=CW
    int                 m_sampleRate{384000};
};

} // namespace AetherSDR
