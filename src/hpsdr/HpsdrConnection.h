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

    // Thread-safe: written from the main thread, read by the connection thread.
    virtual void setRxFrequency(double hz) = 0;
    virtual void setSampleRate(quint32 rate) = 0;

signals:
    // Raw 24-bit IQ samples, big-endian: 3 bytes I then 3 bytes Q per sample.
    // Mic/auxiliary bytes are already stripped; format is identical for P1 and P2.
    void iqReady(QByteArray samples);

    // Emitted when the radio stops responding (watchdog timeout).
    void connectionLost();
};

} // namespace AetherSDR
