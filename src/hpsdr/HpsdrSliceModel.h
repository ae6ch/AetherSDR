#pragma once
#include <QObject>

namespace AetherSDR {

// SliceModel subclass for HPSDR: routes setFrequency/setMode/setFilterWidth to DSP instead of SmartSDR.
// Task 9 will change the base class from QObject to SliceModel (one-line swap).
class HpsdrSliceModel : public QObject {
    Q_OBJECT
public:
    explicit HpsdrSliceModel(QObject* parent = nullptr);
};

} // namespace AetherSDR
