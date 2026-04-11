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
    void setCenterFrequency(double hz);  // atomic — main thread safe
    void setRxFrequency(double hz);      // atomic — main thread safe
    void setMode(const QString& mode);   // main thread only

public slots:
    // Connected via Qt::QueuedConnection from HpsdrP2Connection (different thread)
    void processIq(const QByteArray& raw24bitIq);

signals:
    void fftReady(quint64 centerHz, float bandwidthHz, QVector<float> binsDbfs);
    void pcmReady(QByteArray int16Stereo24k);

private:
    void initFft();
    void destroyFft();
    void computeFirCoeffs();
    void updateNco();
    void runFft();

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
    QVector<float> m_audio48kBuf;  // intermediate 48k float samples
    QByteArray     m_audioOutBuf;  // int16 stereo 24k output

    std::atomic<double> m_centerHz{14225000.0};
    std::atomic<double> m_rxHz{14225000.0};
    std::atomic<int>    m_mode{0};   // 0=USB 1=LSB 2=AM 3=CW
    int                 m_sampleRate{384000};
};

} // namespace AetherSDR
