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

    void setRxFrequency(double hz) override;
    void setSampleRate(quint32 rate) override;

    // Antenna / RF chain — stored; P2 packet encoding TBD from Thetis protocol2.cs
    void setTxAntenna(int ant) override;
    void setRxInput(int input) override;
    void setPreamp(bool on) override;
    void setAttenuation(int db) override;
    void setAdcDither(bool on) override;
    void setAdcRandom(bool on) override;

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
    quint32               m_seqNum{0};
    std::atomic<double>   m_rxFreqHz{14225000.0};
    std::atomic<quint32>  m_sampleRate{384000};
    std::atomic<int>      m_txAntenna{1};
    std::atomic<int>      m_rxInput{0};
    std::atomic<bool>     m_preamp{false};
    std::atomic<int>      m_attenDb{0};
    std::atomic<bool>     m_adcDither{false};
    std::atomic<bool>     m_adcRandom{false};
    bool                  m_running{false};
};

} // namespace AetherSDR
