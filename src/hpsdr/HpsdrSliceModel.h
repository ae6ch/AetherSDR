#pragma once
#include <QObject>

namespace AetherSDR {

// Placeholder — will be refactored to integrate with SliceModel in Task 9
class HpsdrSliceModel : public QObject {
    Q_OBJECT
public:
    explicit HpsdrSliceModel(QObject* parent = nullptr);
};

} // namespace AetherSDR
