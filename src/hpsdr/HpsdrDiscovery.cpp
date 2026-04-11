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
        // Discovery replies are 63 bytes for both P1 and P2.
        if (data.size() < 63) { continue; }
        if (static_cast<quint8>(data[0]) != 0xEF) { continue; }
        if (static_cast<quint8>(data[1]) != 0xFE) { continue; }
        // MAC address sits at bytes [4..9] in both P1 and P2 layouts.
        // Our own broadcast has all zeros there — skip it.
        bool ownBroadcast = true;
        for (int i = 4; i <= 9; ++i) {
            if (static_cast<quint8>(data[i]) != 0x00) { ownBroadcast = false; break; }
        }
        if (ownBroadcast) { continue; }

        HpsdrRadioInfo info = parseDiscoveryReply(data);
        info.address = dg.senderAddress();
        if (info.mac.isEmpty()) { continue; }

        bool isNew = !m_lastSeen.contains(info.mac);
        m_lastSeen[info.mac] = QDateTime::currentMSecsSinceEpoch();
        if (isNew) {
            qCInfo(lcHpsdr) << "HpsdrDiscovery: radio found:" << info.displayName()
                            << "MAC:" << info.mac;
            emit radioFound(info);
        }
    }
}

HpsdrRadioInfo HpsdrDiscovery::parseDiscoveryReply(const QByteArray& data) const {
    // MAC is at bytes [4..9] in both P1 (Metis) and P2 (OpenHPSDR Protocol 2).
    // Verified against:
    //   P1: OpenHPSDR Metis bootloader spec (openhpsdr.org wiki)
    //   P2: Thetis protocol2.cs → ProcessDiscoveryData()
    HpsdrRadioInfo info;
    info.mac = QString("%1:%2:%3:%4:%5:%6")
        .arg(static_cast<quint8>(data[4]), 2, 16, QChar('0'))
        .arg(static_cast<quint8>(data[5]), 2, 16, QChar('0'))
        .arg(static_cast<quint8>(data[6]), 2, 16, QChar('0'))
        .arg(static_cast<quint8>(data[7]), 2, 16, QChar('0'))
        .arg(static_cast<quint8>(data[8]), 2, 16, QChar('0'))
        .arg(static_cast<quint8>(data[9]), 2, 16, QChar('0'));

    // Distinguish P1 from P2 by data[3]:
    //   P2: data[3] = board ID — always >= 2 for known P2 hardware
    //       (1=Hermes P2, 6=Anan10E P2, 7=Angelia, 10=Orion, 11=Orion2)
    //   P1: data[3] = status byte (0=idle, 1=in use by another host)
    // Board ID 1 (Hermes in P2) is ambiguous with P1 in-use status=1;
    // we treat >= 2 as unambiguously P2.  Hermes P2 users running data[3]==1
    // will fall through to P1 handling which still discovers the radio.
    if (static_cast<quint8>(data[3]) >= 2) {
        // OpenHPSDR Protocol 2 — Thetis protocol2.cs ProcessDiscoveryData()
        info.protocolVersion = 2;
        info.boardId         = static_cast<quint8>(data[3]);
        info.fwMajor         = static_cast<quint8>(data[10]);
        info.numReceivers    = static_cast<quint8>(data[11]);
        info.fwMinor         = static_cast<quint8>(data[12]);
    } else {
        // OpenHPSDR Protocol 1 / Metis — openhpsdr.org Metis bootloader spec
        //   [3]  = status (ignored here; used to flag in-use by another host)
        //   [10] = firmware code version
        //   [20] = board ID (0=Metis, 1=Hermes, 2=Griffin, 4=Angelia,
        //                    5=Orion, 6=HermesLite)
        info.protocolVersion = 1;
        info.fwMajor         = static_cast<quint8>(data[10]);
        info.boardId         = (data.size() > 20) ? static_cast<quint8>(data[20]) : 0;
        info.numReceivers    = 1;  // P1 radios present a single RX stream
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
