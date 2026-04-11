#pragma once
#include <QString>
#include <QHostAddress>

namespace AetherSDR {

struct HpsdrRadioInfo {
    QHostAddress address;
    QString      mac;           // unique key for stale detection
    quint8       boardId{0};    // P2 board ID — Anan 10E value from Thetis protocol2.cs
    quint8       fwMajor{0};
    quint8       fwMinor{0};
    quint8       numReceivers{1};

    // TODO: switch on boardId for multi-board support (Hermes, Red Pitaya, etc.)
    QString displayName() const {
        return QString("Anan (HPSDR) — %1").arg(address.toString());
    }
};

} // namespace AetherSDR
