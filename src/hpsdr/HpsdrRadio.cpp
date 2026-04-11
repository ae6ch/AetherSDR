// src/hpsdr/HpsdrRadio.cpp
#include "HpsdrRadio.h"
#include "HpsdrConnection.h"
#include "HpsdrP1Connection.h"
#include "HpsdrP2Connection.h"
#include "HpsdrDsp.h"
#include "HpsdrSliceModel.h"
#include "core/AppSettings.h"
#include "core/LogManager.h"

namespace AetherSDR {

HpsdrRadio::HpsdrRadio(QObject* parent)
    : QObject(parent)
    , m_dsp(std::make_unique<HpsdrDsp>())
    , m_slice(std::make_unique<HpsdrSliceModel>(nullptr, m_dsp.get()))
{
    // Move DSP to its own thread; m_slice stays on the caller (main) thread.
    // The concrete m_conn is created in connectToRadio() once the protocol version
    // is known.  m_conn signals are wired there too.
    m_dspThread.setObjectName("HpsdrDspThread");
    m_dsp->moveToThread(&m_dspThread);

    // DSP output (DSP thread) → forwarded signals on HpsdrRadio (main thread).
    connect(m_dsp.get(), &HpsdrDsp::fftReady, this, &HpsdrRadio::fftReady);
    connect(m_dsp.get(), &HpsdrDsp::pcmReady, this, &HpsdrRadio::pcmReady);
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

    // Create the protocol-appropriate connection object.
    // P1 = Metis/Hermes firmware; P2 = OpenHPSDR Protocol 2 / Thetis firmware.
    if (info.protocolVersion == 1) {
        m_conn = std::make_unique<HpsdrP1Connection>();
        qCInfo(lcHpsdr) << "HpsdrRadio: using Protocol 1 (Metis) connection";
    } else {
        m_conn = std::make_unique<HpsdrP2Connection>();
        qCInfo(lcHpsdr) << "HpsdrRadio: using Protocol 2 connection";
    }

    // Wire IQ and watchdog signals from the new connection.
    // iqReady crosses the thread boundary (conn on main, DSP on m_dspThread) —
    // Qt auto-queues it.
    connect(m_conn.get(), &HpsdrConnection::iqReady,
            m_dsp.get(),  &HpsdrDsp::processIq);
    connect(m_conn.get(), &HpsdrConnection::connectionLost,
            this,          &HpsdrRadio::onConnectionLost);

    // Give the slice model access to the new connection for frequency routing.
    m_slice->setConnection(m_conn.get());

    // Read sample rate from persistent settings.
    // Default 384000 matches the Anan 10E maximum and both P1/P2 driver defaults.
    AppSettings& settings = AppSettings::instance();
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
        QString errMsg = QString("Failed to bind UDP socket for %1").arg(info.address.toString());
        qCWarning(lcHpsdr) << "HpsdrRadio:" << errMsg;
        m_dspThread.quit();
        m_dspThread.wait();
        m_slice->setConnection(nullptr);
        m_conn.reset();
        emit connectionError(errMsg);
        return false;
    }

    m_connected = true;
    emit connected();
    qCInfo(lcHpsdr) << "HpsdrRadio: connected to" << info.displayName()
                    << "(P" << info.protocolVersion << ") sample rate" << sampleRate;
    return true;
}

void HpsdrRadio::disconnectFromRadio()
{
    if (!m_connected) {
        return;
    }
    m_connected = false;

    m_conn->disconnectFromRadio();

    // Stop the DSP thread gracefully before destroying the connection,
    // so no in-flight iqReady signals can reach the stopped DSP.
    m_dspThread.quit();
    m_dspThread.wait();

    m_slice->setConnection(nullptr);
    m_conn.reset();

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

    // DSP thread must stop before m_conn is destroyed.
    m_dspThread.quit();
    m_dspThread.wait();

    m_slice->setConnection(nullptr);
    m_conn.reset();

    emit disconnected();
}

} // namespace AetherSDR
