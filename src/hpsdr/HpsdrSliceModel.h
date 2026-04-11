#pragma once
// src/hpsdr/HpsdrSliceModel.h
// SliceModel subclass for HPSDR: routes setFrequency/setMode/setFilterWidth to DSP
// instead of emitting SmartSDR commandReady strings.
// Works with both P1 and P2 connections via the HpsdrConnection interface.
#include "models/SliceModel.h"

namespace AetherSDR {

class HpsdrConnection;  // abstract; concrete type selected by HpsdrRadio
class HpsdrDsp;

class HpsdrSliceModel : public SliceModel {
    Q_OBJECT
public:
    // conn may be nullptr initially; call setConnection() once the connection is established.
    explicit HpsdrSliceModel(HpsdrConnection* conn, HpsdrDsp* dsp,
                             QObject* parent = nullptr);
    ~HpsdrSliceModel() override = default;

    // Update the connection pointer when HpsdrRadio creates or destroys it.
    // Must be called on the main thread only (same thread as this object).
    void setConnection(HpsdrConnection* conn) { m_conn = conn; }

    void setFrequency(double mhz) override;
    void tuneAndRecenter(double mhz) override;  // same hardware routing as setFrequency
    void setMode(const QString& mode) override;
    void setFilterWidth(int low, int high) override;

private:
    void routeFrequencyToHardware(double mhz);

    HpsdrConnection* m_conn{nullptr};  // not owned — owned by HpsdrRadio
    HpsdrDsp*        m_dsp{nullptr};   // not owned — owned by HpsdrRadio
};

} // namespace AetherSDR
