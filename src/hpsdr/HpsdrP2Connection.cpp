// src/hpsdr/HpsdrP2Connection.cpp
#include "HpsdrP2Connection.h"
#include "core/LogManager.h"
#include <QNetworkDatagram>

namespace AetherSDR {

static constexpr quint16 kHpsdrRadioPort   = 1024;
static constexpr int     kControlIntervalMs = 1;

HpsdrP2Connection::HpsdrP2Connection(QObject* parent) : QObject(parent)
{
    connect(&m_socket,       &QUdpSocket::readyRead, this, &HpsdrP2Connection::onReadyRead);
    connect(&m_controlTimer, &QTimer::timeout,       this, &HpsdrP2Connection::onControlTimer);
    m_controlTimer.setInterval(kControlIntervalMs);
}

HpsdrP2Connection::~HpsdrP2Connection()
{
    disconnectFromRadio();
}

bool HpsdrP2Connection::connectToRadio(const HpsdrRadioInfo& info)
{
    if (m_running) {
        qCWarning(lcHpsdr) << "HpsdrP2Connection: connectToRadio() called while already running";
        return false;
    }
    m_radioAddress = info.address;
    if (!m_socket.bind(QHostAddress::AnyIPv4, 0)) {
        qCWarning(lcHpsdr) << "P2Connection: bind failed:" << m_socket.errorString();
        return false;
    }
    m_running = true;
    m_seqNum  = 0;
    sendStartPacket();
    m_controlTimer.start();
    qCInfo(lcHpsdr) << "P2Connection: started, local port" << m_socket.localPort()
                    << "-> radio" << m_radioAddress.toString();
    return true;
}

void HpsdrP2Connection::disconnectFromRadio()
{
    if (!m_running) {
        return;
    }
    m_controlTimer.stop();
    sendStopPacket();
    m_socket.close();
    m_running = false;
    qCInfo(lcHpsdr) << "P2Connection: stopped";
}

void HpsdrP2Connection::setRxFrequency(double hz)  { m_rxFreqHz.store(hz); }
void HpsdrP2Connection::setSampleRate(quint32 rate) { m_sampleRate.store(rate); }

void HpsdrP2Connection::sendStartPacket()
{
    // P2 start: the first control packet with sequence 0 acts as the stream start.
    // Verify against Thetis protocol2.cs StartStream() — some implementations
    // use a dedicated start packet type at data[2].
    sendControlPacket();
}

void HpsdrP2Connection::sendStopPacket()
{
    // P2 stop packet — verify byte layout in Thetis protocol2.cs StopStream()
    QByteArray pkt(63, 0x00);
    pkt[0] = static_cast<char>(0xEF);
    pkt[1] = static_cast<char>(0xFE);
    pkt[2] = 0x08;  // stop type — verify in Thetis protocol2.cs
    m_socket.writeDatagram(pkt, m_radioAddress, kHpsdrRadioPort);
}

QByteArray HpsdrP2Connection::buildControlPacket() const
{
    // Cross-reference ALL offsets with Thetis protocol2.cs SendControlPacket()
    // before trusting the byte positions below. These are placeholder offsets.
    QByteArray pkt(64, 0x00);
    pkt[0] = static_cast<char>(0xEF);
    pkt[1] = static_cast<char>(0xFE);
    pkt[2] = 0x04;  // PC->Radio data packet type — verify in Thetis

    // Sequence number (big-endian uint32) — verify offset in Thetis
    quint32 seq = m_seqNum;
    pkt[4] = static_cast<char>((seq >> 24) & 0xFF);
    pkt[5] = static_cast<char>((seq >> 16) & 0xFF);
    pkt[6] = static_cast<char>((seq >>  8) & 0xFF);
    pkt[7] = static_cast<char>( seq        & 0xFF);

    // RX frequency (big-endian uint32 Hz) — verify offset in Thetis
    quint32 freqHz = static_cast<quint32>(m_rxFreqHz.load());
    pkt[8]  = static_cast<char>((freqHz >> 24) & 0xFF);
    pkt[9]  = static_cast<char>((freqHz >> 16) & 0xFF);
    pkt[10] = static_cast<char>((freqHz >>  8) & 0xFF);
    pkt[11] = static_cast<char>( freqHz        & 0xFF);

    // Sample rate code — encoding verify in Thetis protocol2.cs
    quint32 sr = m_sampleRate.load();
    quint8 rateCode = (sr <= 48000)  ? 0
                    : (sr <= 96000)  ? 1
                    : (sr <= 192000) ? 2
                    : (sr <= 384000) ? 3
                    : (sr <= 768000) ? 4
                    :                  5;
    pkt[12] = static_cast<char>(rateCode);  // verify offset in Thetis

    return pkt;
}

void HpsdrP2Connection::sendControlPacket()
{
    m_socket.writeDatagram(buildControlPacket(), m_radioAddress, kHpsdrRadioPort);
}

void HpsdrP2Connection::onControlTimer()
{
    if (!m_running) {
        return;
    }
    ++m_seqNum;
    sendControlPacket();
}

void HpsdrP2Connection::onReadyRead()
{
    while (m_socket.hasPendingDatagrams()) {
        QNetworkDatagram dg = m_socket.receiveDatagram();
        const QByteArray data = dg.data();

        // Verify magic bytes and IQ packet type in Thetis protocol2.cs ProcessPacket()
        if (data.size() < 8) {
            continue;
        }
        if (static_cast<quint8>(data[0]) != 0xEF) {
            continue;
        }
        if (static_cast<quint8>(data[1]) != 0xFE) {
            continue;
        }
        if (static_cast<quint8>(data[2]) != 0x06) {
            continue;  // 0x06 = Radio->PC IQ data — verify in Thetis
        }

        // Strip header and emit raw IQ payload.
        // Each sample = 6 bytes: 3 bytes I (24-bit signed big-endian) + 3 bytes Q.
        // Verify header size in Thetis protocol2.cs ProcessPacket()
        constexpr int kHeaderSize = 8;
        emit iqReady(data.mid(kHeaderSize));
    }
}

} // namespace AetherSDR
