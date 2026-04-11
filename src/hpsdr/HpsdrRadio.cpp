// src/hpsdr/HpsdrRadio.cpp
#include "HpsdrRadio.h"
#include "HpsdrP2Connection.h"
#include "HpsdrDsp.h"
#include "HpsdrSliceModel.h"
#include "core/AppSettings.h"
#include "core/LogManager.h"

namespace AetherSDR {

HpsdrRadio::HpsdrRadio(QObject* parent)
    : QObject(parent)
    , m_conn(std::make_unique<HpsdrP2Connection>())
    , m_dsp(std::make_unique<HpsdrDsp>())
    , m_slice(std::make_unique<HpsdrSliceModel>(m_conn.get(), m_dsp.get()))
{
    // Move DSP to its own thread. m_conn and m_slice stay on the caller's
    // (main) thread. The iqReady → processIq connection below crosses the
    // thread boundary and will be auto-queued.
    m_dsp->moveToThread(&m_dspThread);

    // IQ from connection (main thread) → DSP processing (DSP thread).
    // Qt auto-detects the thread boundary and uses QueuedConnection.
    connect(m_conn.get(), &HpsdrP2Connection::iqReady,
            m_dsp.get(),  &HpsdrDsp::processIq);

    // DSP output (DSP thread) → forwarded signals on HpsdrRadio (main thread).
    connect(m_dsp.get(), &HpsdrDsp::fftReady,
            this,         &HpsdrRadio::fftReady);

    connect(m_dsp.get(), &HpsdrDsp::pcmReady,
            this,         &HpsdrRadio::pcmReady);

    // Connection-lost notification (main thread → main thread, direct).
    connect(m_conn.get(), &HpsdrP2Connection::connectionLost,
            this,          &HpsdrRadio::onConnectionLost);
}

HpsdrRadio::~HpsdrRadio()
{
    disconnectFromRadio();
}

bool HpsdrRadio::connectToRadio(const HpsdrRadioInfo& info)
{
    if (m_connected) {
        qCWarning(lcHpsdr) << "HpsdrRadio: connectToRadio() called while already connected";
        return false;
    }

    // Read sample rate from persistent settings.
    // Default 384000 matches the Anan 10E maximum and P2Connection default.
    // (v1.4.0.0 firmware note does not apply here — Anan uses its own protocol)
    auto& settings = AppSettings::instance();
    bool convOk = false;
    int sampleRate = settings.value("HpsdrSampleRate", "384000").toInt(&convOk);
    if (!convOk || sampleRate <= 0) {
        qCWarning(lcHpsdr) << "HpsdrRadio: invalid HpsdrSampleRate setting, using 384000";
        sampleRate = 384000;
    }

    m_conn->setSampleRate(static_cast<quint32>(sampleRate));
    m_dsp->setSampleRate(sampleRate);

    // Start the DSP thread before opening the connection so processIq() slots
    // are ready to receive queued signals the moment the first IQ packet arrives.
    m_dspThread.start();

    if (!m_conn->connectToRadio(info)) {
        qCWarning(lcHpsdr) << "HpsdrRadio: failed to connect to" << info.address.toString();
        m_dspThread.quit();
        m_dspThread.wait();
        return false;
    }

    m_connected = true;
    emit connected();
    qCInfo(lcHpsdr) << "HpsdrRadio: connected to" << info.displayName()
                    << "sample rate" << sampleRate;
    return true;
}

void HpsdrRadio::disconnectFromRadio()
{
    if (!m_connected) {
        return;
    }
    m_connected = false;

    m_conn->disconnectFromRadio();

    // Stop the DSP thread gracefully.
    m_dspThread.quit();
    m_dspThread.wait();

    emit disconnected();
    qCInfo(lcHpsdr) << "HpsdrRadio: disconnected";
}

void HpsdrRadio::onConnectionLost()
{
    if (!m_connected) {
        return;
    }
    qCWarning(lcHpsdr) << "HpsdrRadio: connection lost (radio stopped responding)";
    m_connected = false;

    m_dspThread.quit();
    m_dspThread.wait();

    emit disconnected();
}

} // namespace AetherSDR
