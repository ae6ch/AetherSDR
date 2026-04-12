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
    if (!m_running) { return; }
    m_watchdogTimer.stop();
    m_controlTimer.stop();
    sendStopPacket();
    m_socket.close();
    m_running = false;
    qCInfo(lcHpsdr) << "HpsdrP1Connection: stopped";
}

// ── Atomic setters (main-thread safe) ─────────────────────────────────────

void HpsdrP1Connection::setRxFrequency(double hz)  { m_rxFreqHz.store(hz); }
void HpsdrP1Connection::setSampleRate(quint32 rate) { m_sampleRate.store(rate); }
void HpsdrP1Connection::setTxAntenna(int ant)       { m_txAntenna.store(ant); }
void HpsdrP1Connection::setRxInput(int input)       { m_rxInput.store(input); }
void HpsdrP1Connection::setPreamp(bool on)          { m_preamp.store(on); }
void HpsdrP1Connection::setAttenuation(int db)      { m_attenDb.store(db); }
void HpsdrP1Connection::setAdcDither(bool on)       { m_adcDither.store(on); }
void HpsdrP1Connection::setAdcRandom(bool on)       { m_adcRandom.store(on); }

// ── Packet construction ────────────────────────────────────────────────────

void HpsdrP1Connection::sendStartPacket()
{
    // P1 start: 64-byte command, data[3]=0x01.
    // Reference: OpenHPSDR Metis bootloader spec, "Start" command.
    QByteArray pkt(64, 0x00);
    pkt[0] = static_cast<char>(0xEF);
    pkt[1] = static_cast<char>(0xFE);
    pkt[2] = 0x04;
    pkt[3] = 0x01;
    m_socket.writeDatagram(pkt, m_radioAddress, kHpsdrPort);
}

void HpsdrP1Connection::sendStopPacket()
{
    // P1 stop: 64-byte command, data[3]=0x00.
    QByteArray pkt(64, 0x00);
    pkt[0] = static_cast<char>(0xEF);
    pkt[1] = static_cast<char>(0xFE);
    pkt[2] = 0x04;
    pkt[3] = 0x00;
    m_socket.writeDatagram(pkt, m_radioAddress, kHpsdrPort);
}

void HpsdrP1Connection::sendControlPacket()
{
    // P1 EP2 control packet: 1032 bytes.
    // Header (8 bytes):  EF FE 01 02 <seq32>
    // Two 512-byte USB frames, each: [7F 7F 7F C0 C1 C2 C3 C4] + 504 zeros.
    //
    // Frame 1 (C0=0x00) — configuration:
    //   C1 bits[1:0] = sample rate code (00=48k 01=96k 10=192k 11=384k)
    //   C2           = 0x00 (reserved)
    //   C3 bit 0     = 10 dB attenuation
    //   C3 bit 1     = 20 dB attenuation
    //   C3 bit 2     = preamp (~+20 dB LNA)
    //   C3 bit 3     = ADC dither
    //   C3 bit 4     = ADC random
    //   C3 bits 5-6  = RX input select (00=follow-TX 01=Rx1In 10=Rx2In 11=XVTR)
    //   C4 bits 0-1  = TX antenna (00=ANT1 01=ANT2 10=ANT3)
    //   C4 bit 2     = duplex enable (always 1)
    //   C4 bits 3-5  = numRx-1 (000 = 1 receiver)
    //
    // Frame 2 (C0=0x04) — RX1 frequency: C1..C4 = big-endian uint32 Hz.
    //
    // Reference: OpenHPSDR Metis bootloader spec; Thetis networkproto1.c WriteMainLoop.

    QByteArray pkt(1032, 0x00);
    pkt[0] = static_cast<char>(0xEF);
    pkt[1] = static_cast<char>(0xFE);
    pkt[2] = 0x01;  // host → radio
    pkt[3] = 0x02;  // EP2

    // Sequence number (big-endian uint32)
    pkt[4] = static_cast<char>((m_seqNum >> 24) & 0xFF);
    pkt[5] = static_cast<char>((m_seqNum >> 16) & 0xFF);
    pkt[6] = static_cast<char>((m_seqNum >>  8) & 0xFF);
    pkt[7] = static_cast<char>( m_seqNum        & 0xFF);

    // ── Frame 1: C0=0x00 ────────────────────────────────────────────────────
    pkt[8]  = static_cast<char>(0x7F);
    pkt[9]  = static_cast<char>(0x7F);
    pkt[10] = static_cast<char>(0x7F);
    pkt[11] = 0x00;  // C0: address 0, MOX=0

    // C1: sample rate code
    const quint32 sr = m_sampleRate.load();
    const quint8 srCode = (sr <= 48000)  ? 0
                        : (sr <= 96000)  ? 1
                        : (sr <= 192000) ? 2
                        :                  3;
    pkt[12] = static_cast<char>(srCode);  // C1

    pkt[13] = 0x00;  // C2: reserved

    // C3: attenuation, preamp, ADC controls, RX input select
    const int  attenDb    = m_attenDb.load();
    const bool preamp     = m_preamp.load();
    const bool dither     = m_adcDither.load();
    const bool random     = m_adcRandom.load();
    const int  rxInput    = m_rxInput.load();

    quint8 c3 = 0;
    if (attenDb >= 10) { c3 |= 0x01; }   // bit 0: 10 dB atten
    if (attenDb >= 20) { c3 |= 0x02; }   // bit 1: 20 dB atten
    if (preamp)        { c3 |= 0x04; }   // bit 2: preamp
    if (dither)        { c3 |= 0x08; }   // bit 3: ADC dither
    if (random)        { c3 |= 0x10; }   // bit 4: ADC random
    // bits 6:5 = RX input select
    switch (rxInput) {
        case 1: c3 |= 0x20; break;  // Rx1In:   01
        case 2: c3 |= 0x40; break;  // Rx2In:   10
        case 3: c3 |= 0x60; break;  // XVTR:    11
        default: break;              // 0: follow TX (00)
    }
    pkt[14] = static_cast<char>(c3);  // C3

    // C4: TX antenna select, duplex, numRx
    const int txAnt = m_txAntenna.load();
    quint8 c4 = 0x04;  // bit 2 = duplex enable (always 1)
    if      (txAnt == 2) { c4 |= 0x01; }  // ANT2: bits[1:0]=01
    else if (txAnt == 3) { c4 |= 0x02; }  // ANT3: bits[1:0]=10
    // ANT1 (default): bits[1:0]=00
    // numRx-1 = 0 (single receiver), bits[5:3]=000 — already zero
    pkt[15] = static_cast<char>(c4);  // C4

    // ── Frame 2: C0=0x04 — RX1 frequency ────────────────────────────────────
    pkt[520] = static_cast<char>(0x7F);
    pkt[521] = static_cast<char>(0x7F);
    pkt[522] = static_cast<char>(0x7F);
    pkt[523] = 0x04;  // C0: address 2 = (2 << 1) = 0x04, MOX=0

    const quint32 freqHz = static_cast<quint32>(m_rxFreqHz.load());
    pkt[524] = static_cast<char>((freqHz >> 24) & 0xFF);
    pkt[525] = static_cast<char>((freqHz >> 16) & 0xFF);
    pkt[526] = static_cast<char>((freqHz >>  8) & 0xFF);
    pkt[527] = static_cast<char>( freqHz        & 0xFF);

    m_socket.writeDatagram(pkt, m_radioAddress, kHpsdrPort);
}

void HpsdrP1Connection::onControlTimer()
{
    if (!m_running) { return; }
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
        // Each frame: [0..2]=sync [3..7]=C&C [8..511]=63×8-byte rows [I2 I1 I0 Q2 Q1 Q0 Mic_H Mic_L]
        // Keep first 6 bytes of each row (I+Q), discard 2 mic bytes.
        // Output: 126 samples × 6 bytes = 756 bytes of 24-bit big-endian IQ.
        constexpr int kFrame1Start     = 8;
        constexpr int kFrame2Start     = 520;
        constexpr int kCCHeaderBytes   = 8;
        constexpr int kSamplesPerFrame = 63;
        constexpr int kRowBytes        = 8;
        constexpr int kIqBytesPerRow   = 6;

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
    if (!m_running) { return; }
    qCWarning(lcHpsdr) << "HpsdrP1Connection: watchdog expired — no IQ in"
                       << kWatchdogMs << "ms; radio assumed lost";
    m_controlTimer.stop();
    m_socket.close();
    m_running = false;
    emit connectionLost();
}

} // namespace AetherSDR
