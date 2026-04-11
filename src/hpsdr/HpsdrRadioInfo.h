#pragma once
#include <QString>
#include <QHostAddress>

namespace AetherSDR {

struct HpsdrRadioInfo {
    QHostAddress address;
    QString      mac;              // unique key for stale detection
    quint8       boardId{0};       // hardware board ID (see discovery byte offsets below)
    quint8       fwMajor{0};
    quint8       fwMinor{0};
    quint8       numReceivers{1};
    // Protocol version detected from the discovery reply:
    //   1 = OpenHPSDR Protocol 1 / Metis (older Hermes, Anan 10E P1 firmware)
    //   2 = OpenHPSDR Protocol 2 / Thetis (Anan 10E P2 firmware, board ID >= 2)
    quint8       protocolVersion{1};

    QString displayName() const {
        return QString("Anan (HPSDR P%1) — %2").arg(protocolVersion).arg(address.toString());
    }
};

} // namespace AetherSDR
