// src/hpsdr/HpsdrSliceModel.cpp
#include "HpsdrSliceModel.h"
#include "HpsdrConnection.h"
#include "HpsdrDsp.h"
#include "core/LogManager.h"

namespace AetherSDR {

static constexpr double kHzPerMhz = 1.0e6;

HpsdrSliceModel::HpsdrSliceModel(HpsdrConnection* conn, HpsdrDsp* dsp,
                                  QObject* parent)
    : SliceModel(0, parent), m_conn(conn), m_dsp(dsp)
{
    // Disconnect commandReady so SmartSDR commands are never emitted.
    QObject::disconnect(this, &SliceModel::commandReady, nullptr, nullptr);
}

// ── Frequency helpers ──────────────────────────────────────────────────────

void HpsdrSliceModel::routeFrequencyToHardware(double mhz)
{
    const double hz = mhz * kHzPerMhz;
    if (m_conn) { m_conn->setRxFrequency(hz); }
    if (m_dsp)  {
        m_dsp->setCenterFrequency(hz);
        m_dsp->setRxFrequency(hz);
    }
}

void HpsdrSliceModel::setFrequency(double mhz)
{
    SliceModel::setFrequency(mhz);
    routeFrequencyToHardware(mhz);
}

void HpsdrSliceModel::tuneAndRecenter(double mhz)
{
    SliceModel::tuneAndRecenter(mhz);
    routeFrequencyToHardware(mhz);
}

// ── Mode / filter ──────────────────────────────────────────────────────────

void HpsdrSliceModel::setMode(const QString& mode)
{
    SliceModel::setMode(mode);
    if (m_dsp) { m_dsp->setMode(mode); }
}

void HpsdrSliceModel::setFilterWidth(int low, int high)
{
    SliceModel::setFilterWidth(low, high);
    if (m_dsp) { m_dsp->setFilterBandwidth(low, high); }
}

// ── Audio / DSP controls (host-side) ──────────────────────────────────────

void HpsdrSliceModel::setAudioGain(float gain)
{
    SliceModel::setAudioGain(gain);
    if (m_dsp) { m_dsp->setAfGain(gain); }
}

void HpsdrSliceModel::setAudioMute(bool mute)
{
    SliceModel::setAudioMute(mute);
    if (m_dsp) { m_dsp->setMute(mute); }
}

void HpsdrSliceModel::setAudioPan(int pan)
{
    SliceModel::setAudioPan(pan);
    // SliceModel pan is -100..+100; HpsdrDsp wants -1.0..+1.0
    if (m_dsp) { m_dsp->setAudioPan(static_cast<float>(pan) / 100.0f); }
}

void HpsdrSliceModel::setSquelch(bool on, int level)
{
    SliceModel::setSquelch(on, level);
    // SliceModel level is 0–100; map to normalised RMS threshold 0.0–1.0.
    // The gain stage multiplies by 1000, so threshold is scaled accordingly.
    const float threshold = static_cast<float>(level) / 100.0f;
    if (m_dsp) { m_dsp->setSquelch(on, threshold); }
}

// ── RF chain controls (hardware) ──────────────────────────────────────────

void HpsdrSliceModel::setRfGain(float gain)
{
    SliceModel::setRfGain(gain);
    if (!m_conn) { return; }
    // Map gain (dB) to discrete HPSDR preamp / attenuation steps.
    // Anan 10E RF chain: preamp ≈ +20 dB, attenuators: 10 dB and 20 dB (combinable).
    //   gain > 10   → preamp on, no attenuation
    //   0..10       → no preamp, no attenuation
    //  -10..-1      → 10 dB attenuation
    //  -20..-11     → 20 dB attenuation
    //  < -20        → 30 dB attenuation (10 + 20)
    if (gain > 10.0f) {
        m_conn->setPreamp(true);
        m_conn->setAttenuation(0);
    } else if (gain >= 0.0f) {
        m_conn->setPreamp(false);
        m_conn->setAttenuation(0);
    } else if (gain >= -10.0f) {
        m_conn->setPreamp(false);
        m_conn->setAttenuation(10);
    } else if (gain >= -20.0f) {
        m_conn->setPreamp(false);
        m_conn->setAttenuation(20);
    } else {
        m_conn->setPreamp(false);
        m_conn->setAttenuation(30);
    }
}

void HpsdrSliceModel::setRxAntenna(const QString& ant)
{
    SliceModel::setRxAntenna(ant);
    if (!m_conn) { return; }
    // "ANT1" / "ANT2" / "ANT3" → select TX antenna port (RX follows TX, rxInput=0)
    // "EXT"                    → dedicated external RX port (rxInput=3, XVTR/EXT)
    if (ant == "ANT1") {
        m_conn->setTxAntenna(1);
        m_conn->setRxInput(0);
    } else if (ant == "ANT2") {
        m_conn->setTxAntenna(2);
        m_conn->setRxInput(0);
    } else if (ant == "ANT3") {
        m_conn->setTxAntenna(3);
        m_conn->setRxInput(0);
    } else if (ant == "EXT") {
        m_conn->setTxAntenna(1);   // TX stays on ANT1 (RX-only operation)
        m_conn->setRxInput(3);     // EXT / XVTR dedicated RX port
    } else {
        qCWarning(lcHpsdr) << "HpsdrSliceModel: unknown antenna" << ant;
    }
}

} // namespace AetherSDR
