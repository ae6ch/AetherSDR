#pragma once
// Discovers HPSDR radios on the LAN via OpenHPSDR P2 UDP broadcast.
#include <QObject>

namespace AetherSDR {

class HpsdrDiscovery : public QObject {
    Q_OBJECT
public:
    explicit HpsdrDiscovery(QObject* parent = nullptr);
};

} // namespace AetherSDR
