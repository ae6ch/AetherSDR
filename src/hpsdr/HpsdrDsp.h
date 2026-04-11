#pragma once
#include <QObject>

namespace AetherSDR {

class HpsdrDsp : public QObject {
    Q_OBJECT
public:
    explicit HpsdrDsp(QObject* parent = nullptr);
};

} // namespace AetherSDR
