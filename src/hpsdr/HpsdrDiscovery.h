#pragma once
#include <QObject>

namespace AetherSDR {

class HpsdrDiscovery : public QObject {
    Q_OBJECT
public:
    explicit HpsdrDiscovery(QObject* parent = nullptr);
};

} // namespace AetherSDR
