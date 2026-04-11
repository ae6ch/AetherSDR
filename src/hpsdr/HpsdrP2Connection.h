#pragma once
// src/hpsdr/HpsdrP2Connection.h
// Manages the OpenHPSDR P2 UDP connection: start/stop, control packets, IQ reception.
#include "HpsdrRadioInfo.h"
#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QHostAddress>
#include <atomic>

namespace AetherSDR {

class HpsdrP2Connection : public QObject {
    Q_OBJECT
public:
    explicit HpsdrP2Connection(QObject* parent = nullptr);
    ~HpsdrP2Connection() override;

    bool connectToRadio(const HpsdrRadioInfo& info);
    void disconnectFromRadio();  // Named to avoid shadowing QObject::disconnect()

    void setRxFrequency(double hz);   // atomic — safe from main thread
    void setSampleRate(quint32 rate); // atomic — safe from main thread

signals:
    void iqReady(QByteArray samples);  // raw 24-bit IQ, big-endian, I then Q per sample
    void connectionLost();

private slots:
    void onReadyRead();
    void onControlTimer();

private:
    void sendStartPacket();
    void sendStopPacket();
    void sendControlPacket();
    QByteArray buildControlPacket() const;

    QUdpSocket            m_socket;
    QTimer                m_controlTimer;
    QHostAddress          m_radioAddress;
    quint32               m_seqNum{0};
    std::atomic<double>   m_rxFreqHz{14225000.0};
    std::atomic<quint32>  m_sampleRate{384000};
    bool                  m_running{false};
};

} // namespace AetherSDR
