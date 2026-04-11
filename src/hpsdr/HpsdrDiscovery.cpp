// src/hpsdr/HpsdrDiscovery.cpp
#include "HpsdrDiscovery.h"
#include "core/LogManager.h"
#include <QDateTime>
#include <QNetworkDatagram>

namespace AetherSDR {

HpsdrDiscovery::HpsdrDiscovery(QObject* parent) : QObject(parent) {
    connect(&m_socket,     &QUdpSocket::readyRead, this, &HpsdrDiscovery::onReadyRead);
    connect(&m_pollTimer,  &QTimer::timeout,       this, &HpsdrDiscovery::onPollTimer);
    connect(&m_staleTimer, &QTimer::timeout,       this, &HpsdrDiscovery::onStaleTimer);
    m_staleTimer.setInterval(POLL_INTERVAL_MS);
}

void HpsdrDiscovery::startListening() {
    if (m_socket.state() != QAbstractSocket::UnconnectedState) {
        qCWarning(lcHpsdr) << "HpsdrDiscovery: startListening() called while already active";
        return;
    }
    // Bind to ephemeral port — avoids needing root/CAP_NET_BIND_SERVICE for port <1024
    if (!m_socket.bind(QHostAddress::AnyIPv4, 0)) {
        qCWarning(lcHpsdr) << "HpsdrDiscovery: bind failed:" << m_socket.errorString();
        return;
    }
    m_pollTimer.start(POLL_INTERVAL_MS);
    m_staleTimer.start();
    sendDiscoveryRequest();
}

void HpsdrDiscovery::stopListening() {
    m_pollTimer.stop();
    m_staleTimer.stop();
    m_socket.close();
}

void HpsdrDiscovery::sendDiscoveryRequest() {
    // P2 discovery request: 63 bytes.
    // Byte layout verified against Thetis protocol2.cs → SendDiscoveryPacket():
    //   [0] = 0xEF, [1] = 0xFE, [2] = 0x02 (discovery request type)
    //   [3..62] = 0x00
    QByteArray pkt(63, 0x00);
    pkt[0] = static_cast<char>(0xEF);
    pkt[1] = static_cast<char>(0xFE);
    pkt[2] = 0x02;  // discovery request type
    m_socket.writeDatagram(pkt, QHostAddress::Broadcast, HPSDR_PORT);
}

void HpsdrDiscovery::onReadyRead() {
    while (m_socket.hasPendingDatagrams()) {
        QNetworkDatagram dg = m_socket.receiveDatagram();
        const QByteArray data = dg.data();

        // P1 (Metis) replies are 60 bytes; P2 replies are 63 bytes.
        // Our own discovery broadcast is 63 bytes, so 60-byte packets can
        // only originate from a radio (no ownBroadcast guard needed for P1).
        if (data.size() != 60 && data.size() != 63) { continue; }
        if (static_cast<quint8>(data[0]) != 0xEF) { continue; }
        if (static_cast<quint8>(data[1]) != 0xFE) { continue; }

        // For 63-byte packets (P2 or our own echo): the MAC is at [4..9].
        // Our sent broadcast has all zeros there; real P2 replies do not.
        if (data.size() == 63) {
            bool ownBroadcast = true;
            for (int i = 4; i <= 9; ++i) {
                if (static_cast<quint8>(data[i]) != 0x00) { ownBroadcast = false; break; }
            }
            if (ownBroadcast) { continue; }
        }

        HpsdrRadioInfo info = parseDiscoveryReply(data);
        info.address = dg.senderAddress();
        if (info.mac.isEmpty()) { continue; }

        bool isNew = !m_lastSeen.contains(info.mac);
        m_lastSeen[info.mac] = QDateTime::currentMSecsSinceEpoch();
        if (isNew) {
            qCInfo(lcHpsdr) << "HpsdrDiscovery: radio found:" << info.displayName()
                            << "MAC:" << info.mac << "P" << info.protocolVersion;
            emit radioFound(info);
        }
    }
}

HpsdrRadioInfo HpsdrDiscovery::parseDiscoveryReply(const QByteArray& data) const {
    // Packet size is the reliable P1/P2 discriminator (confirmed by Wireshark):
    //   60 bytes → OpenHPSDR Protocol 1 / Metis
    //   63 bytes → OpenHPSDR Protocol 2 (Thetis)
    //
    // P1 layout (60 bytes): [0]=EF [1]=FE [2]=02/03 [3..8]=MAC [9]=fw_version [10]=board_id
    // P2 layout (63 bytes): [0]=EF [1]=FE [2]=02    [3]=board_id [4..9]=MAC [10]=fwMajor [11]=numRx [12]=fwMinor
    //
    // References:
    //   P1: OpenHPSDR Metis bootloader spec (openhpsdr.org) — MAC at [3..8] confirmed by capture
    //   P2: Thetis protocol2.cs → ProcessDiscoveryData()
    HpsdrRadioInfo info;

    if (data.size() == 60) {
        // Protocol 1 / Metis
        info.protocolVersion = 1;
        info.mac = QString("%1:%2:%3:%4:%5:%6")
            .arg(static_cast<quint8>(data[3]), 2, 16, QChar('0'))
            .arg(static_cast<quint8>(data[4]), 2, 16, QChar('0'))
            .arg(static_cast<quint8>(data[5]), 2, 16, QChar('0'))
            .arg(static_cast<quint8>(data[6]), 2, 16, QChar('0'))
            .arg(static_cast<quint8>(data[7]), 2, 16, QChar('0'))
            .arg(static_cast<quint8>(data[8]), 2, 16, QChar('0'));
        info.fwMajor      = static_cast<quint8>(data[9]);
        info.boardId      = static_cast<quint8>(data[10]);
        info.numReceivers = 1;  // P1 presents a single RX stream
    } else {
        // Protocol 2 — Thetis protocol2.cs ProcessDiscoveryData()
        info.protocolVersion = 2;
        info.boardId         = static_cast<quint8>(data[3]);
        info.mac = QString("%1:%2:%3:%4:%5:%6")
            .arg(static_cast<quint8>(data[4]), 2, 16, QChar('0'))
            .arg(static_cast<quint8>(data[5]), 2, 16, QChar('0'))
            .arg(static_cast<quint8>(data[6]), 2, 16, QChar('0'))
            .arg(static_cast<quint8>(data[7]), 2, 16, QChar('0'))
            .arg(static_cast<quint8>(data[8]), 2, 16, QChar('0'))
            .arg(static_cast<quint8>(data[9]), 2, 16, QChar('0'));
        info.fwMajor      = static_cast<quint8>(data[10]);
        info.numReceivers = static_cast<quint8>(data[11]);
        info.fwMinor      = static_cast<quint8>(data[12]);
    }
    return info;
}

void HpsdrDiscovery::onPollTimer()  { sendDiscoveryRequest(); }

void HpsdrDiscovery::onStaleTimer() {
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    QStringList toRemove;
    for (auto it = m_lastSeen.begin(); it != m_lastSeen.end(); ++it) {
        if (now - it.value() > STALE_MS) toRemove.append(it.key());
    }
    for (const QString& mac : toRemove) {
        m_lastSeen.remove(mac);
        qCInfo(lcHpsdr) << "HpsdrDiscovery: radio lost (stale):" << mac;
        emit radioLost(mac);
    }
}

} // namespace AetherSDR
