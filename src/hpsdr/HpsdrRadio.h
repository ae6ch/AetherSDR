#pragma once
// src/hpsdr/HpsdrRadio.h
// Coordinator that owns a HpsdrConnection (P1 or P2), HpsdrDsp, and HpsdrSliceModel.
// Manages connection lifecycle and DSP thread.
//
// Threading model:
//   m_dsp lives on m_dspThread (QThread).
//   m_conn and m_slice live on the caller (main) thread.
//   Signals between m_conn and m_dsp cross the thread boundary via
//   Qt::QueuedConnection (auto-queued because they live on different threads).
//
// m_conn is created lazily in connectToRadio() as either HpsdrP1Connection or
// HpsdrP2Connection based on HpsdrRadioInfo::protocolVersion.

#include "HpsdrRadioInfo.h"
#include <QObject>
#include <QThread>
#include <QVector>
#include <memory>

namespace AetherSDR {

class HpsdrConnection;  // abstract base; P1 or P2 concrete type selected at connect time
class HpsdrDsp;
class HpsdrSliceModel;

class HpsdrRadio : public QObject {
    Q_OBJECT
public:
    explicit HpsdrRadio(QObject* parent = nullptr);
    ~HpsdrRadio() override;

    // Attempt to connect and start RX streaming.
    // Creates the protocol-appropriate connection (P1 or P2) based on info.protocolVersion.
    // Returns false if the connection could not be established.
    // Emits connected() on success, connectionError() on failure.
    bool connectToRadio(const HpsdrRadioInfo& info);

    // Stop streaming and release the UDP socket.
    void disconnectFromRadio();

    // The slice model for this radio (always valid; wired to the active connection).
    HpsdrSliceModel* sliceModel() const { return m_slice.get(); }

signals:
    void connected();
    void disconnected();
    void connectionError(const QString& msg);

    // Forwarded from HpsdrDsp — consumed by SpectrumWidget::feedFftBins
    void fftReady(quint64 centerHz, float bandwidthHz, QVector<float> binsDbfs);

    // Forwarded from HpsdrDsp — consumed by AudioEngine::feedHpsdrAudio
    void pcmReady(const QByteArray& int16Stereo24k);

    // Forwarded from HpsdrDsp — drives SMeterWidget (~100 ms update rate)
    void levelReady(float dbm);

private slots:
    void onConnectionLost();

private:
    std::unique_ptr<HpsdrConnection>  m_conn;   // null until connectToRadio(); P1 or P2
    std::unique_ptr<HpsdrDsp>         m_dsp;
    std::unique_ptr<HpsdrSliceModel>  m_slice;
    QThread                           m_dspThread;
    bool                              m_connected{false};
};

} // namespace AetherSDR
