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
    m_staleTimer.setInterval(2000);
}

void HpsdrDiscovery::startListening() {
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
        if (data.size() < 60) continue;
        if (static_cast<quint8>(data[0]) != 0xEF) continue;
        if (static_cast<quint8>(data[1]) != 0xFE) continue;
        if (static_cast<quint8>(data[3]) == 0x00) continue;  // skip our own requests

        HpsdrRadioInfo info = parseDiscoveryReply(data);
        info.address = dg.senderAddress();
        if (info.mac.isEmpty()) continue;

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
    // Byte layout verified against Thetis protocol2.cs → ProcessDiscoveryData():
    //   [3]     = board ID
    //   [4..9]  = MAC address (6 bytes)
    //   [10]    = firmware major version
    //   [11]    = number of receivers
    //   [12]    = firmware minor version
    HpsdrRadioInfo info;
    info.boardId = static_cast<quint8>(data[3]);
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
