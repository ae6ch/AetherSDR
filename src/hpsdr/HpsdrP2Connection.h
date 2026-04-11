#pragma once
// src/hpsdr/HpsdrP2Connection.h
// OpenHPSDR Protocol 2 UDP connection: start/stop, control packets, IQ reception.
// Implements HpsdrConnection; HpsdrRadio selects this when protocolVersion == 2.
#include "HpsdrConnection.h"
#include <QUdpSocket>
#include <QTimer>
#include <QHostAddress>
#include <atomic>

namespace AetherSDR {

class HpsdrP2Connection : public HpsdrConnection {
    Q_OBJECT
public:
    explicit HpsdrP2Connection(QObject* parent = nullptr);
    ~HpsdrP2Connection() override;

    bool connectToRadio(const HpsdrRadioInfo& info) override;
    void disconnectFromRadio() override;  // Named to avoid shadowing QObject::disconnect()

    void setRxFrequency(double hz) override;   // atomic — safe from main thread
    void setSampleRate(quint32 rate) override; // atomic — safe from main thread
    // signals iqReady and connectionLost inherited from HpsdrConnection

private slots:
    void onReadyRead();
    void onControlTimer();
    void onWatchdogTimeout();   // emits connectionLost() when radio stops sending IQ

private:
    void sendStartPacket();
    void sendStopPacket();
    void sendControlPacket();
    QByteArray buildControlPacket() const;

    // If no IQ packet arrives within kWatchdogMs, the radio has gone away.
    // Set conservatively: 384kHz ÷ ~504 bytes/pkt ≈ 762 pkts/sec; 3s = ~2286 missed.
    static constexpr int kWatchdogMs = 3000;

    QUdpSocket            m_socket;
    QTimer                m_controlTimer;
    QTimer                m_watchdogTimer;
    QHostAddress          m_radioAddress;
    quint32               m_seqNum{0};  // event-loop only; not shared across threads (unlike m_rxFreqHz/m_sampleRate)
    std::atomic<double>   m_rxFreqHz{14225000.0};
    std::atomic<quint32>  m_sampleRate{384000};
    bool                  m_running{false};
};

} // namespace AetherSDR
