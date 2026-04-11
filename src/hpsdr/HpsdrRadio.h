#pragma once
// src/hpsdr/HpsdrRadio.h
// Coordinator that owns HpsdrP2Connection, HpsdrDsp, and HpsdrSliceModel.
// Manages connection lifecycle and DSP thread.
//
// Threading model:
//   m_dsp lives on m_dspThread (QThread).
//   m_conn and m_slice live on the caller (main) thread.
//   Signals between m_conn and m_dsp cross the thread boundary via
//   Qt::QueuedConnection (auto-queued because they live on different threads).

#include "HpsdrRadioInfo.h"
#include <QObject>
#include <QThread>
#include <QVector>
#include <memory>

namespace AetherSDR {

class HpsdrP2Connection;
class HpsdrDsp;
class HpsdrSliceModel;

class HpsdrRadio : public QObject {
    Q_OBJECT
public:
    explicit HpsdrRadio(QObject* parent = nullptr);
    ~HpsdrRadio() override;

    // Attempt to connect and start RX streaming.
    // Returns false if the connection could not be established.
    // Emits connected() on success, connectionError() on failure.
    bool connectToRadio(const HpsdrRadioInfo& info);

    // Stop streaming and release the UDP socket.
    void disconnectFromRadio();

    // The slice model for this radio (valid after connectToRadio() succeeds).
    HpsdrSliceModel* sliceModel() const { return m_slice.get(); }

signals:
    void connected();
    void disconnected();
    void connectionError(const QString& msg);

    // Forwarded from HpsdrDsp — consumed by SpectrumWidget::feedFftBins
    void fftReady(quint64 centerHz, float bandwidthHz, QVector<float> binsDbfs);

    // Forwarded from HpsdrDsp — consumed by AudioEngine::feedHpsdrAudio
    void pcmReady(QByteArray int16Stereo24k);

private slots:
    void onConnectionLost();

private:
    std::unique_ptr<HpsdrP2Connection> m_conn;
    std::unique_ptr<HpsdrDsp>          m_dsp;
    std::unique_ptr<HpsdrSliceModel>   m_slice;
    QThread                            m_dspThread;
    bool                               m_connected{false};
};

} // namespace AetherSDR
