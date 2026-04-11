// src/hpsdr/HpsdrP1Connection.cpp
#include "HpsdrP1Connection.h"
#include "core/LogManager.h"
#include <QNetworkDatagram>

namespace AetherSDR {

HpsdrP1Connection::HpsdrP1Connection(QObject* parent) : HpsdrConnection(parent)
{
    connect(&m_socket,        &QUdpSocket::readyRead, this, &HpsdrP1Connection::onReadyRead);
    connect(&m_controlTimer,  &QTimer::timeout,       this, &HpsdrP1Connection::onControlTimer);
    connect(&m_watchdogTimer, &QTimer::timeout,       this, &HpsdrP1Connection::onWatchdogTimeout);
    m_controlTimer.setInterval(kControlMs);
    m_watchdogTimer.setInterval(kWatchdogMs);
    m_watchdogTimer.setSingleShot(true);
}

HpsdrP1Connection::~HpsdrP1Connection()
{
    disconnectFromRadio();
}

bool HpsdrP1Connection::connectToRadio(const HpsdrRadioInfo& info)
{
    if (m_running) {
        qCWarning(lcHpsdr) << "HpsdrP1Connection: connectToRadio() called while already running";
        return false;
    }
    m_radioAddress = info.address;
    if (!m_socket.bind(QHostAddress::AnyIPv4, 0)) {
        qCWarning(lcHpsdr) << "HpsdrP1Connection: bind failed:" << m_socket.errorString();
        return false;
    }
    m_running = true;
    m_seqNum  = 0;
    sendStartPacket();
    m_controlTimer.start();
    m_watchdogTimer.start();
    qCInfo(lcHpsdr) << "HpsdrP1Connection: started, local port" << m_socket.localPort()
                    << "-> radio" << m_radioAddress.toString();
    return true;
}

void HpsdrP1Connection::disconnectFromRadio()
{
    if (!m_running) {
        return;
    }
    m_watchdogTimer.stop();
    m_controlTimer.stop();
    sendStopPacket();
    m_socket.close();
    m_running = false;
    qCInfo(lcHpsdr) << "HpsdrP1Connection: stopped";
}

void HpsdrP1Connection::setRxFrequency(double hz)  { m_rxFreqHz.store(hz); }
void HpsdrP1Connection::setSampleRate(quint32 rate) { m_sampleRate.store(rate); }

void HpsdrP1Connection::sendStartPacket()
{
    // P1 start: 64-byte command packet, data[3]=0x01.
    // Reference: OpenHPSDR Metis bootloader spec, "Start" command.
    QByteArray pkt(64, 0x00);
    pkt[0] = static_cast<char>(0xEF);
    pkt[1] = static_cast<char>(0xFE);
    pkt[2] = 0x04;  // command type
    pkt[3] = 0x01;  // start
    m_socket.writeDatagram(pkt, m_radioAddress, kHpsdrPort);
}

void HpsdrP1Connection::sendStopPacket()
{
    // P1 stop: 64-byte command packet, data[3]=0x00.
    // Reference: OpenHPSDR Metis bootloader spec, "Stop" command.
    QByteArray pkt(64, 0x00);
    pkt[0] = static_cast<char>(0xEF);
    pkt[1] = static_cast<char>(0xFE);
    pkt[2] = 0x04;  // command type
    pkt[3] = 0x00;  // stop
    m_socket.writeDatagram(pkt, m_radioAddress, kHpsdrPort);
}

void HpsdrP1Connection::sendControlPacket()
{
    // P1 EP2 control packet: 1032 bytes.
    // Header (8 bytes): EF FE 01 02 <seq32>
    // Two 512-byte USB frames, each containing: [7F 7F 7F C0 C1 C2 C3 C4] + 504 zeros.
    // Frame 1 carries C0=0x00 (configuration: sample rate + num receivers + duplex).
    // Frame 2 carries C0=0x04 (RX1 frequency, big-endian uint32 Hz).
    // Reference: OpenHPSDR Metis spec, EP2 C&C word encoding.
    QByteArray pkt(1032, 0x00);
    pkt[0] = static_cast<char>(0xEF);
    pkt[1] = static_cast<char>(0xFE);
    pkt[2] = 0x01;  // host → radio data stream
    pkt[3] = 0x02;  // EP2

    // Sequence number (big-endian uint32)
    pkt[4] = static_cast<char>((m_seqNum >> 24) & 0xFF);
    pkt[5] = static_cast<char>((m_seqNum >> 16) & 0xFF);
    pkt[6] = static_cast<char>((m_seqNum >>  8) & 0xFF);
    pkt[7] = static_cast<char>( m_seqNum        & 0xFF);

    // ── Frame 1: C0=0x00 — configuration ────────────────────────────────
    // Byte layout (offsets relative to packet start):
    //   [8..10]  = sync 0x7F 0x7F 0x7F
    //   [11]     = C0: bits[7:1]=address=0, bit[0]=MOX=0
    //   [12]     = C1: bits[1:0] = sample rate code
    //              (00=48kHz, 01=96kHz, 10=192kHz, 11=384kHz)
    //   [13]     = C2: 0x00
    //   [14]     = C3: 0x00
    //   [15]     = C4: bits[5:3]=numRx-1 (000=1rx), bit[2]=duplex=1
    pkt[8]  = static_cast<char>(0x7F);
    pkt[9]  = static_cast<char>(0x7F);
    pkt[10] = static_cast<char>(0x7F);
    pkt[11] = 0x00;  // C0: address 0, no MOX

    quint32 sr = m_sampleRate.load();
    quint8 srCode = (sr <= 48000)  ? 0
                  : (sr <= 96000)  ? 1
                  : (sr <= 192000) ? 2
                  :                  3;  // 384000 and above
    pkt[12] = static_cast<char>(srCode);  // C1
    pkt[13] = 0x00;  // C2
    pkt[14] = 0x00;  // C3
    pkt[15] = 0x04;  // C4: duplex bit[2]=1, numRx-1=0 (1 receiver)

    // ── Frame 2: C0=0x04 — RX1 frequency ────────────────────────────────
    // Byte layout (offsets relative to packet start):
    //   [520..522] = sync 0x7F 0x7F 0x7F
    //   [523]      = C0: bits[7:1]=address=2 → byte value=(2<<1)=0x04, bit[0]=MOX=0
    //   [524..527] = C1..C4: RX1 frequency in Hz (big-endian uint32)
    pkt[520] = static_cast<char>(0x7F);
    pkt[521] = static_cast<char>(0x7F);
    pkt[522] = static_cast<char>(0x7F);
    pkt[523] = 0x04;  // C0: address 2 (RX1 freq), no MOX

    quint32 freqHz = static_cast<quint32>(m_rxFreqHz.load());
    pkt[524] = static_cast<char>((freqHz >> 24) & 0xFF);  // C1
    pkt[525] = static_cast<char>((freqHz >> 16) & 0xFF);  // C2
    pkt[526] = static_cast<char>((freqHz >>  8) & 0xFF);  // C3
    pkt[527] = static_cast<char>( freqHz        & 0xFF);  // C4

    m_socket.writeDatagram(pkt, m_radioAddress, kHpsdrPort);
}

void HpsdrP1Connection::onControlTimer()
{
    if (!m_running) {
        return;
    }
    ++m_seqNum;
    sendControlPacket();
}

void HpsdrP1Connection::onReadyRead()
{
    while (m_socket.hasPendingDatagrams()) {
        QNetworkDatagram dg = m_socket.receiveDatagram();
        const QByteArray data = dg.data();

        // P1 IQ packet: exactly 1032 bytes, header EF FE 01 06.
        // Reference: OpenHPSDR Metis spec, EP6 (IQ data, radio → host).
        if (data.size() < 1032) { continue; }
        if (static_cast<quint8>(data[0]) != 0xEF) { continue; }
        if (static_cast<quint8>(data[1]) != 0xFE) { continue; }
        if (static_cast<quint8>(data[2]) != 0x01) { continue; }
        if (static_cast<quint8>(data[3]) != 0x06) { continue; }

        // Extract IQ from two 512-byte USB frames.
        // Each frame layout (relative to frame start):
        //   [0..2]   = sync 0x7F 0x7F 0x7F
        //   [3..7]   = C&C word (C0..C4), ignored on RX path
        //   [8..511] = 63 × 8-byte sample rows: [I2 I1 I0 Q2 Q1 Q0 Mic_H Mic_L]
        // We keep the first 6 bytes of each row (I+Q) and discard the 2 mic bytes.
        // Output: 126 samples × 6 bytes = 756 bytes of 24-bit big-endian IQ.
        constexpr int kFrame1Start     = 8;    // byte offset of frame 1 in packet
        constexpr int kFrame2Start     = 520;  // byte offset of frame 2 in packet
        constexpr int kCCHeaderBytes   = 8;    // 3 sync + 1 C0 + 4 C1-C4
        constexpr int kSamplesPerFrame = 63;
        constexpr int kRowBytes        = 8;    // I2 I1 I0 Q2 Q1 Q0 Mic_H Mic_L
        constexpr int kIqBytesPerRow   = 6;    // bytes to keep (drop last 2)

        QByteArray iq;
        iq.reserve(kSamplesPerFrame * 2 * kIqBytesPerRow);  // 756 bytes

        for (int frame = 0; frame < 2; ++frame) {
            int iqBase = ((frame == 0) ? kFrame1Start : kFrame2Start) + kCCHeaderBytes;
            for (int s = 0; s < kSamplesPerFrame; ++s) {
                iq.append(data.mid(iqBase + s * kRowBytes, kIqBytesPerRow));
            }
        }

        m_watchdogTimer.start();  // reset watchdog — radio is alive
        emit iqReady(iq);
    }
}

void HpsdrP1Connection::onWatchdogTimeout()
{
    if (!m_running) {
        return;
    }
    qCWarning(lcHpsdr) << "HpsdrP1Connection: watchdog expired — no IQ received in"
                       << kWatchdogMs << "ms; radio assumed lost";
    // Stop timers and socket before emitting so the caller can safely
    // call disconnectFromRadio() from the connected slot.
    m_controlTimer.stop();
    m_socket.close();
    m_running = false;
    emit connectionLost();
}

} // namespace AetherSDR
