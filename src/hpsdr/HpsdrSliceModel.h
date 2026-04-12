#pragma once
// src/hpsdr/HpsdrSliceModel.h
// SliceModel subclass for HPSDR: overrides virtual setters to route commands to
// HpsdrConnection (protocol hardware) and HpsdrDsp (host-side DSP) instead of
// emitting SmartSDR commandReady strings.
// Works with both P1 and P2 connections via the HpsdrConnection interface.
#include "models/SliceModel.h"

namespace AetherSDR {

class HpsdrConnection;
class HpsdrDsp;

class HpsdrSliceModel : public SliceModel {
    Q_OBJECT
public:
    explicit HpsdrSliceModel(HpsdrConnection* conn, HpsdrDsp* dsp,
                             QObject* parent = nullptr);
    ~HpsdrSliceModel() override = default;

    void setConnection(HpsdrConnection* conn) { m_conn = conn; }

    // ── Capability ──────────────────────────────────────────────────────────
    bool hasExtendedApplet() const override { return false; }

    // ── Frequency / mode / filter ───────────────────────────────────────────
    void setFrequency(double mhz) override;
    void tuneAndRecenter(double mhz) override;
    void setMode(const QString& mode) override;
    void setFilterWidth(int low, int high) override;

    // ── Audio / DSP (host-side, via HpsdrDsp) ──────────────────────────────
    void setAudioGain(float gain) override;
    void setAudioMute(bool mute) override;
    void setAudioPan(int pan) override;       // -100..+100 → HpsdrDsp -1..+1
    void setSquelch(bool on, int level) override; // level 0–100 → HpsdrDsp threshold

    // ── RF chain (hardware, via HpsdrConnection) ────────────────────────────
    void setRfGain(float gain) override;      // dB float → preamp/attenuation steps
    void setRxAntenna(const QString& ant) override; // "ANT1"/"ANT2"/"ANT3"/"EXT"

private:
    void routeFrequencyToHardware(double mhz);

    HpsdrConnection* m_conn{nullptr};  // not owned — owned by HpsdrRadio
    HpsdrDsp*        m_dsp{nullptr};   // not owned — owned by HpsdrRadio
};

} // namespace AetherSDR
