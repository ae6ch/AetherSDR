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
    // This is safer than using QSignalBlocker in every setter override because
    // QSignalBlocker only suppresses signals for the current scope and could
    // miss async paths.
    QObject::disconnect(this, &SliceModel::commandReady, nullptr, nullptr);
}

// Helper: route a new frequency to both the connection and the DSP engine.
// The base class already guards m_locked and qFuzzyCompare before calling the
// virtual override, so we do not repeat those checks here.
//
// For a single-VFO P1/P2 HPSDR radio the IQ stream is always centred at the
// tuned frequency, so m_centerHz must track m_rxHz (NCO offset stays zero).
// Setting both atomics lets the FFT emit the correct center for the waterfall.
void HpsdrSliceModel::routeFrequencyToHardware(double mhz)
{
    const double hz = mhz * kHzPerMhz;
    if (m_conn) {
        m_conn->setRxFrequency(hz);
    }
    if (m_dsp) {
        m_dsp->setCenterFrequency(hz);  // waterfall center + NCO baseline
        m_dsp->setRxFrequency(hz);      // NCO offset = rxHz - centerHz = 0
    }
}

void HpsdrSliceModel::setFrequency(double mhz)
{
    // Call base: checks m_locked and qFuzzyCompare, updates m_frequency,
    // emits frequencyChanged. commandReady is disconnected so no SmartSDR
    // command fires.
    SliceModel::setFrequency(mhz);
    routeFrequencyToHardware(mhz);
}

void HpsdrSliceModel::tuneAndRecenter(double mhz)
{
    // Same hardware routing as setFrequency — HPSDR has no panadapter pan
    // concept so recenter vs. no-recenter makes no difference at the hardware
    // level. The base class still emits frequencyChanged for the GUI.
    SliceModel::tuneAndRecenter(mhz);
    routeFrequencyToHardware(mhz);
}

void HpsdrSliceModel::setMode(const QString& mode)
{
    SliceModel::setMode(mode);
    if (m_dsp) {
        m_dsp->setMode(mode);
    }
}

void HpsdrSliceModel::setFilterWidth(int low, int high)
{
    SliceModel::setFilterWidth(low, high);
    // DSP bandwidth adjustment for filter width is a follow-up enhancement.
}

} // namespace AetherSDR
