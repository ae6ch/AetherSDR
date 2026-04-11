#pragma once
// On-host DSP for HPSDR: complex FFT for spectrum + NCO/FIR/demod for audio.
#include <QObject>

namespace AetherSDR {

class HpsdrDsp : public QObject {
    Q_OBJECT
public:
    explicit HpsdrDsp(QObject* parent = nullptr);
};

} // namespace AetherSDR
