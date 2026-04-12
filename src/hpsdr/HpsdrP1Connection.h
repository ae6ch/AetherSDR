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
//   Frame 1 (C0=0x00): configuration — sample rate, preamp, attenuation, dither,
//                       random, TX antenna (C4[1:0]), RX input (C3[6:5])
//   Frame 2 (C0=0x04): RX1 frequency (big-endian uint32 Hz)
//
// P1 C3 bit layout (address 0x00):
//   bit 0: 10 dB attenuation
//   bit 1: 20 dB attenuation
//   bit 2: RX preamp (~+20 dB LNA)
//   bit 3: ADC dither
//   bit 4: ADC random
//   bits 5-6: RX input (00=follow-TX, 01=Rx1In, 10=Rx2In, 11=XVTR/EXT)
//   bit 7: Rx_1_Out (reserved; always 0 here)
//
// P1 C4 bit layout (address 0x00):
//   bits 0-1: TX antenna (00=ANT1, 01=ANT2, 10=ANT3)
//   bit 2:    duplex enable (always 1)
//   bits 3-5: numRx - 1 (000 = 1 receiver)
//
// Reference: OpenHPSDR Metis bootloader specification; Thetis networkproto1.c
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

    // ── Frequency / rate (atomic — safe from main thread) ──────────────────
    void setRxFrequency(double hz) override;
    void setSampleRate(quint32 rate) override;

    // ── Antenna / RF chain (atomic — safe from main thread) ────────────────
    void setTxAntenna(int ant) override;   // 1=ANT1, 2=ANT2, 3=ANT3
    void setRxInput(int input) override;   // 0=follow-TX, 1=Rx1In, 2=Rx2In, 3=XVTR
    void setPreamp(bool on) override;
    void setAttenuation(int db) override;  // 0, 10, 20, or 30 dB
    void setAdcDither(bool on) override;
    void setAdcRandom(bool on) override;

private slots:
    void onReadyRead();
    void onControlTimer();
    void onWatchdogTimeout();

private:
    void sendStartPacket();
    void sendStopPacket();
    void sendControlPacket();

    static constexpr quint16 kHpsdrPort  = 1024;
    static constexpr int     kControlMs  = 5;     // 200 Hz control rate
    static constexpr int     kWatchdogMs = 3000;

    QUdpSocket   m_socket;
    QTimer       m_controlTimer;
    QTimer       m_watchdogTimer;
    QHostAddress m_radioAddress;
    quint32      m_seqNum{0};

    std::atomic<double>   m_rxFreqHz{14225000.0};
    std::atomic<quint32>  m_sampleRate{384000};
    std::atomic<int>      m_txAntenna{1};    // 1=ANT1, 2=ANT2, 3=ANT3
    std::atomic<int>      m_rxInput{0};      // 0=follow-TX, 1=Rx1In, 2=Rx2In, 3=XVTR
    std::atomic<bool>     m_preamp{false};
    std::atomic<int>      m_attenDb{0};      // 0, 10, 20, or 30
    std::atomic<bool>     m_adcDither{false};
    std::atomic<bool>     m_adcRandom{false};
    bool                  m_running{false};
};

} // namespace AetherSDR
