#pragma once
// src/hpsdr/HpsdrConnection.h
// Abstract interface for OpenHPSDR protocol connections.
// HpsdrP1Connection (Metis/Hermes) and HpsdrP2Connection both implement this.
// HpsdrRadio selects the concrete class based on HpsdrRadioInfo::protocolVersion.
#include "HpsdrRadioInfo.h"
#include <QObject>

namespace AetherSDR {

class HpsdrConnection : public QObject {
    Q_OBJECT
public:
    explicit HpsdrConnection(QObject* parent = nullptr) : QObject(parent) {}
    ~HpsdrConnection() override = default;

    virtual bool connectToRadio(const HpsdrRadioInfo& info) = 0;
    virtual void disconnectFromRadio() = 0;

    // ── Frequency / sample rate ─────────────────────────────────────────────
    // Thread-safe: written from the main thread, read by the connection thread.
    virtual void setRxFrequency(double hz) = 0;
    virtual void setSampleRate(quint32 rate) = 0;

    // ── Antenna selection ───────────────────────────────────────────────────
    // txAntenna: 1=ANT1, 2=ANT2, 3=ANT3 (also selects RX antenna when rxInput==0)
    // rxInput:   0=follow TX antenna, 1=Rx1In, 2=Rx2In, 3=XVTR/EXT RX port
    // P1 ref: networkproto1.c WriteMainLoop — C4[1:0] and C3[6:5] at address 0x00.
    virtual void setTxAntenna(int ant) = 0;
    virtual void setRxInput(int input) = 0;

    // ── RF chain ────────────────────────────────────────────────────────────
    // preamp:      true = ~+20 dB LNA (C3[2] @ C0=0x00)
    // attenuation: 0 / 10 / 20 / 30 dB — maps to C3[0] (10 dB) and C3[1] (20 dB)
    // adcDither:   enables ADC dither for improved noise floor (C3[3] @ C0=0x00)
    // adcRandom:   enables ADC random bit for noise shaping (C3[4] @ C0=0x00)
    // P1 ref: networkproto1.c WriteMainLoop — C3 byte at address 0x00.
    virtual void setPreamp(bool on) = 0;
    virtual void setAttenuation(int db) = 0;   // accepted values: 0, 10, 20, 30
    virtual void setAdcDither(bool on) = 0;
    virtual void setAdcRandom(bool on) = 0;

signals:
    // Raw 24-bit IQ samples, big-endian: 3 bytes I then 3 bytes Q per sample.
    // Mic/auxiliary bytes are already stripped; format is identical for P1 and P2.
    void iqReady(QByteArray samples);

    // Emitted when the radio stops responding (watchdog timeout).
    void connectionLost();
};

} // namespace AetherSDR
