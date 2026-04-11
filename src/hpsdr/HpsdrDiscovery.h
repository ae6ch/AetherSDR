// src/hpsdr/HpsdrDiscovery.h
// Discovers HPSDR radios on the LAN via OpenHPSDR P2 UDP broadcast.
#pragma once
#include "HpsdrRadioInfo.h"
#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QMap>

namespace AetherSDR {

class HpsdrDiscovery : public QObject {
    Q_OBJECT
public:
    static constexpr quint16 HPSDR_PORT       = 1024;
    static constexpr int     POLL_INTERVAL_MS = 2000;
    static constexpr int     STALE_MS         = 6000;

    explicit HpsdrDiscovery(QObject* parent = nullptr);

    void startListening();
    void stopListening();

signals:
    void radioFound(const AetherSDR::HpsdrRadioInfo& info);
    void radioLost(const QString& mac);

private slots:
    void onReadyRead();
    void onPollTimer();
    void onStaleTimer();

private:
    void sendDiscoveryRequest();
    HpsdrRadioInfo parseDiscoveryReply(const QByteArray& data) const;

    QUdpSocket            m_socket;
    QTimer                m_pollTimer;
    QTimer                m_staleTimer;
    QMap<QString, qint64> m_lastSeen;  // mac → timestamp ms
};

} // namespace AetherSDR
