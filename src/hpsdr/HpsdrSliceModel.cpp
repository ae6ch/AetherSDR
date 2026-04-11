// src/hpsdr/HpsdrSliceModel.cpp
#include "HpsdrSliceModel.h"
#include "HpsdrP2Connection.h"
#include "HpsdrDsp.h"
#include "core/LogManager.h"

namespace AetherSDR {

HpsdrSliceModel::HpsdrSliceModel(HpsdrP2Connection* conn, HpsdrDsp* dsp,
                                  QObject* parent)
    : SliceModel(0, parent), m_conn(conn), m_dsp(dsp)
{
    // Disconnect commandReady so SmartSDR commands are never emitted.
    // This is safer than using QSignalBlocker in every setter override because
    // QSignalBlocker only suppresses signals for the current scope and could
    // miss async paths.
    QObject::disconnect(this, &SliceModel::commandReady, nullptr, nullptr);
}

void HpsdrSliceModel::setFrequency(double mhz)
{
    if (isLocked()) {
        qCDebug(lcHpsdr) << "HpsdrSliceModel: setFrequency ignored — slice is locked";
        return;
    }
    // Call base: updates m_frequency, emits frequencyChanged.
    // commandReady is disconnected in constructor so no SmartSDR command fires.
    SliceModel::setFrequency(mhz);
    // Route to HPSDR hardware (Hz)
    if (m_conn) { m_conn->setRxFrequency(mhz * 1.0e6); }
    if (m_dsp)  { m_dsp->setRxFrequency(mhz * 1.0e6); }
}

void HpsdrSliceModel::setMode(const QString& mode)
{
    SliceModel::setMode(mode);
    if (m_dsp) { m_dsp->setMode(mode); }
}

void HpsdrSliceModel::setFilterWidth(int low, int high)
{
    SliceModel::setFilterWidth(low, high);
    // DSP bandwidth adjustment for filter width is a follow-up enhancement.
}

} // namespace AetherSDR
