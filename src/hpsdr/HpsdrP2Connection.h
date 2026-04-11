#pragma once
#include <QObject>

namespace AetherSDR {

class HpsdrP2Connection : public QObject {
    Q_OBJECT
public:
    explicit HpsdrP2Connection(QObject* parent = nullptr);
};

} // namespace AetherSDR
