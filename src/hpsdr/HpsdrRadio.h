#pragma once
#include <QObject>

namespace AetherSDR {

class HpsdrRadio : public QObject {
    Q_OBJECT
public:
    explicit HpsdrRadio(QObject* parent = nullptr);
};

} // namespace AetherSDR
