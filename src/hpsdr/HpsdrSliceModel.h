#pragma once
// src/hpsdr/HpsdrSliceModel.h
// SliceModel subclass for HPSDR: routes setFrequency/setMode/setFilterWidth to DSP
// instead of emitting SmartSDR commandReady strings.
#include "models/SliceModel.h"

namespace AetherSDR {

class HpsdrP2Connection;
class HpsdrDsp;

class HpsdrSliceModel : public SliceModel {
    Q_OBJECT
public:
    explicit HpsdrSliceModel(HpsdrP2Connection* conn, HpsdrDsp* dsp,
                             QObject* parent = nullptr);

    void setFrequency(double mhz) override;
    void setMode(const QString& mode) override;
    void setFilterWidth(int low, int high) override;

private:
    HpsdrP2Connection* m_conn;  // not owned — owned by HpsdrRadio
    HpsdrDsp*          m_dsp;   // not owned — owned by HpsdrRadio
};

} // namespace AetherSDR
