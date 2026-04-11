#pragma once
// src/hpsdr/HpsdrP1Connection.h
// OpenHPSDR Protocol 1 / Metis UDP connection: start/stop, EP2 control, IQ reception.
// Implements HpsdrConnection; HpsdrRadio selects this when protocolVersion == 1.
//
// P1 IQ packets (radio → host, EP6): 1032 bytes
//   Header: EF FE 01 06 <seq32>
//   2 × 512-byte USB frames, each: [7F 7F 7F C0 C1 C2 C3 C4] + 63 × [I2 I1 I0 Q2 Q1 Q0 Mic_H Mic_L]
//
// P1 control packets (host → radio, EP2): 1032 bytes
//   Header: EF FE 01 02 <seq32>
//   Frame 1: C0=0x00 (sample rate + num receivers + duplex)
//   Frame 2: C0=0x04 (RX1 frequency, big-endian uint32 Hz)
//
// Reference: OpenHPSDR Metis bootloader specification (openhpsdr.org wiki)
#include "HpsdrConnection.h"
#include <QUdpSocket>
#include <QTimer>
#include <QHostAddress>
#include <atomic>

namespace AetherSDR {

class HpsdrP1Connection : public HpsdrConnection {
    Q_OBJECT
public:
    explicit HpsdrP1Connection(QObject* parent = nullptr);
    ~HpsdrP1Connection() override;

    bool connectToRadio(const HpsdrRadioInfo& info) override;
    void disconnectFromRadio() override;

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

    static constexpr quint16 kHpsdrPort   = 1024;  // Metis listens on UDP 1024
    static constexpr int     kControlMs   = 5;     // 200 Hz control rate
    // Same watchdog threshold as P2: 384kHz gives ~762 pkts/sec; 3s = ~2286 missed.
    static constexpr int     kWatchdogMs  = 3000;

    QUdpSocket            m_socket;
    QTimer                m_controlTimer;
    QTimer                m_watchdogTimer;
    QHostAddress          m_radioAddress;
    quint32               m_seqNum{0};
    std::atomic<double>   m_rxFreqHz{14225000.0};
    std::atomic<quint32>  m_sampleRate{384000};
    bool                  m_running{false};
};

} // namespace AetherSDR
