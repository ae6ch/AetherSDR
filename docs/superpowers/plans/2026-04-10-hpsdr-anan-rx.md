# HPSDR Anan RX Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add RX-only support for Apache Labs Anan 10E (OpenHPSDR Protocol 2) to AetherSDR — spectrum display, waterfall, and SSB/AM/CW demodulation.

**Architecture:** Self-contained `src/hpsdr/` module. `HpsdrRadio` coordinates discovery, P2 UDP control/IQ, on-host DSP (FFT + demodulation), and a `HpsdrSliceModel` that inherits `SliceModel`. Five injection points in existing files; no changes to SmartSDR code paths.

**Tech Stack:** Qt6, C++20, FFTW3 (complex FFT), `QUdpSocket` (P2 protocol), `std::atomic` (cross-thread DSP params), `QAudioSink` (audio out via existing `AudioEngine`)

**Protocol Reference:** Thetis source at https://github.com/ramdor/Thetis — primary reference for all P2 packet byte layouts. Check `Thetis/Source/Console/radio.cs` and `Thetis/Source/Console/protocol2.cs` for exact offsets before implementing each packet type.

---

## File Map

### New Files

| File | Responsibility |
|---|---|
| `src/hpsdr/HpsdrRadioInfo.h` | Plain struct: IP, MAC, board ID, firmware version |
| `src/hpsdr/HpsdrDiscovery.h/.cpp` | UDP discovery; emits `radioFound`/`radioLost` |
| `src/hpsdr/HpsdrP2Connection.h/.cpp` | P2 UDP socket; start/stop/control packets; IQ reception (async via `readyRead`) |
| `src/hpsdr/HpsdrDsp.h/.cpp` | FFTW3 complex FFT path + NCO/FIR/demod audio path; dedicated DSP thread |
| `src/hpsdr/HpsdrSliceModel.h/.cpp` | Inherits `SliceModel`; overrides setters to route to DSP, not SmartSDR |
| `src/hpsdr/HpsdrRadio.h/.cpp` | Coordinator; owns all above; lifecycle management |

### Modified Files

| File | What Changes |
|---|---|
| `CMakeLists.txt` | Add HPSDR sources (gated on `FFTW3_FOUND`), `HAVE_HPSDR` define |
| `src/core/LogManager.h/.cpp` | Register `lcHpsdr` logging category |
| `src/core/AudioEngine.h/.cpp` | Add `feedHpsdrAudio(QByteArray)` slot |
| `src/gui/SpectrumWidget.h/.cpp` | Add `feedFftBins(quint64, float, QVector<float>)` slot |
| `src/core/RadioDiscovery.h/.cpp` | Forward `HpsdrDiscovery` results |
| `src/gui/ConnectionPanel.h/.cpp` | Accept HPSDR radios; emit `hpsdrConnectRequested(HpsdrRadioInfo)` |
| `src/gui/MainWindow.h/.cpp` | Handle `hpsdrConnectRequested`; instantiate/destroy `HpsdrRadio` |
| `docs/architecture-pipelines.md` | Update thread count (11 Flex / 13 Anan) |

---

## Task 1: Scaffold — HpsdrRadioInfo + CMake + Logging Category

**Files:**
- Create: `src/hpsdr/HpsdrRadioInfo.h`
- Modify: `CMakeLists.txt`
- Modify: `src/core/LogManager.h`, `src/core/LogManager.cpp`

- [ ] **Step 1: Create `src/hpsdr/` and `HpsdrRadioInfo.h`**

```cpp
// src/hpsdr/HpsdrRadioInfo.h
#pragma once
#include <QString>
#include <QHostAddress>

namespace AetherSDR {

struct HpsdrRadioInfo {
    QHostAddress address;
    QString      mac;           // unique key for stale detection
    quint8       boardId{0};    // P2 board ID — Anan 10E value from Thetis protocol2.cs
    quint8       fwMajor{0};
    quint8       fwMinor{0};
    quint8       numReceivers{1};

    QString displayName() const {
        return QString("Anan (HPSDR) — %1").arg(address.toString());
    }
};

} // namespace AetherSDR
```

- [ ] **Step 2: Register `lcHpsdr` in `LogManager`**

Read `src/core/LogManager.h` and `src/core/LogManager.cpp` to see how existing categories are declared (e.g. `lcRadio`, `lcAudio`). Mirror that exact pattern:

In `LogManager.h`, add:
```cpp
Q_DECLARE_LOGGING_CATEGORY(lcHpsdr)
```

In `LogManager.cpp`, add:
```cpp
Q_LOGGING_CATEGORY(lcHpsdr, "aether.hpsdr", QtWarningMsg)
```

This makes `lcHpsdr` available to all HPSDR `.cpp` files via `#include "core/LogManager.h"`.

- [ ] **Step 3: Add HPSDR sources to `CMakeLists.txt`**

Locate the `if(FFTW3_FOUND)` block (around line 693). After the existing FFTW3 linkage lines, add:

```cmake
    # HPSDR / Anan support (requires FFTW3 for on-host DSP)
    target_compile_definitions(AetherSDR PRIVATE HAVE_HPSDR)
    target_sources(AetherSDR PRIVATE
        src/hpsdr/HpsdrDiscovery.cpp
        src/hpsdr/HpsdrP2Connection.cpp
        src/hpsdr/HpsdrDsp.cpp
        src/hpsdr/HpsdrSliceModel.cpp
        src/hpsdr/HpsdrRadio.cpp
    )
    message(STATUS "HPSDR support enabled (Anan 10E)")
```

After the `endif()`, add:
```cmake
if(NOT FFTW3_FOUND)
    message(STATUS "FFTW3 not found — HPSDR support disabled")
endif()
```

- [ ] **Step 4: Create stub `.h/.cpp` files**

Create each with a minimal class stub inside `namespace AetherSDR { ... }`:

```
src/hpsdr/HpsdrDiscovery.h / .cpp
src/hpsdr/HpsdrP2Connection.h / .cpp
src/hpsdr/HpsdrDsp.h / .cpp
src/hpsdr/HpsdrSliceModel.h / .cpp
src/hpsdr/HpsdrRadio.h / .cpp
```

Each stub `.h`: `#pragma once` + `#include <QObject>` + class declaration inheriting `QObject`.
Each stub `.cpp`: just `#include "Hpsdr<Name>.h"`.

- [ ] **Step 5: Verify build compiles**

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_PREFIX_PATH="$(brew --prefix);$(brew --prefix qt@6)"
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) 2>&1 | tail -10
```

Expected: build succeeds, `HPSDR support enabled (Anan 10E)` in CMake output.

- [ ] **Step 6: Commit**

```bash
git add src/hpsdr/ CMakeLists.txt src/core/LogManager.h src/core/LogManager.cpp
git commit -m "hpsdr: scaffold — HpsdrRadioInfo, CMake integration, logging category"
```

---

## Task 2: HpsdrDiscovery

**Files:**
- Implement: `src/hpsdr/HpsdrDiscovery.h/.cpp`

P2 discovery: send a 63-byte request from an ephemeral port to UDP broadcast `255.255.255.255:1024`. The radio replies unicast to our ephemeral port. Verify exact byte layout against Thetis `protocol2.cs` → `SendDiscoveryPacket()` and `ProcessDiscoveryData()`.

- [ ] **Step 1: Write `HpsdrDiscovery.h`**

```cpp
// src/hpsdr/HpsdrDiscovery.h
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
```

- [ ] **Step 2: Write `HpsdrDiscovery.cpp`**

```cpp
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
    // Verify exact format against Thetis protocol2.cs → SendDiscoveryPacket()
    QByteArray pkt(63, 0x00);
    pkt[0] = static_cast<char>(0xEF);
    pkt[1] = static_cast<char>(0xFE);
    pkt[2] = 0x02;  // discovery request type — confirm byte value in Thetis
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
        if (isNew) emit radioFound(info);
    }
}

HpsdrRadioInfo HpsdrDiscovery::parseDiscoveryReply(const QByteArray& data) const {
    // Byte layout — verify ALL offsets against Thetis protocol2.cs ProcessDiscoveryData()
    // before trusting these values:
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
        emit radioLost(mac);
    }
}

} // namespace AetherSDR
```

- [ ] **Step 3: Build and check**

```bash
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) 2>&1 | grep "error:" | head -20
```

- [ ] **Step 4: Commit**

```bash
git add src/hpsdr/HpsdrDiscovery.h src/hpsdr/HpsdrDiscovery.cpp
git commit -m "hpsdr: HpsdrDiscovery — P2 UDP radio discovery"
```

---

## Task 3: Injection Point — ConnectionPanel + RadioDiscovery

**Files:**
- Modify: `src/gui/ConnectionPanel.h/.cpp`
- Modify: `src/core/RadioDiscovery.h/.cpp`

- [ ] **Step 1: Add HPSDR support to `ConnectionPanel.h`**

Add inside `#ifdef HAVE_HPSDR` guards. The include must be inside the guard:

```cpp
// In ConnectionPanel.h, inside class declaration:
#ifdef HAVE_HPSDR
#include "hpsdr/HpsdrRadioInfo.h"
signals:
    void hpsdrConnectRequested(const AetherSDR::HpsdrRadioInfo& radio);
public slots:
    void onHpsdrRadioFound(const AetherSDR::HpsdrRadioInfo& info);
    void onHpsdrRadioLost(const QString& mac);
private:
    QList<AetherSDR::HpsdrRadioInfo> m_hpsdrRadios;
#endif
```

Note: the `#include` must be physically inside the `#ifdef HAVE_HPSDR` block so it is not compiled when HPSDR is disabled.

- [ ] **Step 2: Implement new slots in `ConnectionPanel.cpp`**

```cpp
#ifdef HAVE_HPSDR
void ConnectionPanel::onHpsdrRadioFound(const HpsdrRadioInfo& info) {
    m_hpsdrRadios.removeIf([&](const HpsdrRadioInfo& r){ return r.mac == info.mac; });
    m_hpsdrRadios.append(info);

    // Remove stale list entry if present
    for (int i = 0; i < m_radioList->count(); ++i) {
        if (m_radioList->item(i)->data(Qt::UserRole + 2).toString() == info.mac) {
            delete m_radioList->takeItem(i);
            break;
        }
    }
    QListWidgetItem* item = new QListWidgetItem(info.displayName(), m_radioList);
    item->setData(Qt::UserRole + 1, QStringLiteral("hpsdr"));
    item->setData(Qt::UserRole + 2, info.mac);
}

void ConnectionPanel::onHpsdrRadioLost(const QString& mac) {
    m_hpsdrRadios.removeIf([&](const HpsdrRadioInfo& r){ return r.mac == mac; });
    for (int i = 0; i < m_radioList->count(); ++i) {
        if (m_radioList->item(i)->data(Qt::UserRole + 2).toString() == mac) {
            delete m_radioList->takeItem(i);
            break;
        }
    }
}
#endif
```

- [ ] **Step 3: Update `onConnectClicked()` in `ConnectionPanel.cpp`**

Add HPSDR branch before the existing FlexRadio path:

```cpp
void ConnectionPanel::onConnectClicked() {
    QListWidgetItem* item = m_radioList->currentItem();
    if (!item) return;

#ifdef HAVE_HPSDR
    if (item->data(Qt::UserRole + 1).toString() == QStringLiteral("hpsdr")) {
        QString mac = item->data(Qt::UserRole + 2).toString();
        for (const HpsdrRadioInfo& r : m_hpsdrRadios) {
            if (r.mac == mac) { emit hpsdrConnectRequested(r); return; }
        }
        return;
    }
#endif
    // ... existing FlexRadio connect logic unchanged ...
}
```

- [ ] **Step 4: Add `HpsdrDiscovery` to `RadioDiscovery`**

In `RadioDiscovery.h`, inside `#ifdef HAVE_HPSDR`:
```cpp
#ifdef HAVE_HPSDR
#include "hpsdr/HpsdrDiscovery.h"
signals:
    void hpsdrRadioFound(const AetherSDR::HpsdrRadioInfo& info);
    void hpsdrRadioLost(const QString& mac);
private:
    AetherSDR::HpsdrDiscovery m_hpsdrDiscovery;
#endif
```

In `RadioDiscovery::startListening()`:
```cpp
#ifdef HAVE_HPSDR
    connect(&m_hpsdrDiscovery, &HpsdrDiscovery::radioFound,
            this, &RadioDiscovery::hpsdrRadioFound);
    connect(&m_hpsdrDiscovery, &HpsdrDiscovery::radioLost,
            this, &RadioDiscovery::hpsdrRadioLost);
    m_hpsdrDiscovery.startListening();
#endif
```

In `RadioDiscovery::stopListening()`:
```cpp
#ifdef HAVE_HPSDR
    m_hpsdrDiscovery.stopListening();
#endif
```

- [ ] **Step 5: Wire in `MainWindow`**

Find where `m_discovery` signals are connected in `MainWindow` and add:
```cpp
#ifdef HAVE_HPSDR
    connect(&m_discovery, &RadioDiscovery::hpsdrRadioFound,
            m_connPanel, &ConnectionPanel::onHpsdrRadioFound);
    connect(&m_discovery, &RadioDiscovery::hpsdrRadioLost,
            m_connPanel, &ConnectionPanel::onHpsdrRadioLost);
    // hpsdrConnectRequested handled in Task 11
#endif
```

- [ ] **Step 6: Build and test discovery**

```bash
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) 2>&1 | grep "error:" | head -10
QT_LOGGING_RULES="aether.hpsdr=true" ./build/AetherSDR.app/Contents/MacOS/AetherSDR
```

Expected: Anan appears in connection list if powered on and on the same network.

- [ ] **Step 7: Commit**

```bash
git add src/gui/ConnectionPanel.h src/gui/ConnectionPanel.cpp \
        src/core/RadioDiscovery.h src/core/RadioDiscovery.cpp \
        src/gui/MainWindow.cpp
git commit -m "hpsdr: connection panel — Anan radios appear in discovery list"
```

---

## Task 4: HpsdrP2Connection — UDP Control & IQ Reception

**Files:**
- Implement: `src/hpsdr/HpsdrP2Connection.h/.cpp`

Note on threading: `QUdpSocket::readyRead` fires asynchronously on whichever thread the socket's QObject lives on. `HpsdrP2Connection` lives on the main thread; the Qt event loop delivers `readyRead` without blocking it because UDP parsing is fast (just header check + emit). `HpsdrDsp` runs on its own thread and receives IQ via `Qt::QueuedConnection`. This is sufficient — no dedicated receive thread is needed.

Rename the disconnect method to `disconnectFromRadio()` to avoid shadowing `QObject::disconnect()`.

- [ ] **Step 1: Write `HpsdrP2Connection.h`**

```cpp
// src/hpsdr/HpsdrP2Connection.h
#pragma once
#include "HpsdrRadioInfo.h"
#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QHostAddress>
#include <atomic>

namespace AetherSDR {

class HpsdrP2Connection : public QObject {
    Q_OBJECT
public:
    explicit HpsdrP2Connection(QObject* parent = nullptr);
    ~HpsdrP2Connection() override;

    bool connectToRadio(const HpsdrRadioInfo& info);
    void disconnectFromRadio();  // Named to avoid shadowing QObject::disconnect()

    void setRxFrequency(double hz);   // atomic — safe from main thread
    void setSampleRate(quint32 rate); // atomic — safe from main thread

signals:
    void iqReady(QByteArray samples);  // raw 24-bit IQ, big-endian, I then Q per sample
    void connectionLost();

private slots:
    void onReadyRead();
    void onControlTimer();

private:
    void sendStartPacket();
    void sendStopPacket();
    void sendControlPacket();
    QByteArray buildControlPacket() const;

    QUdpSocket            m_socket;
    QTimer                m_controlTimer;
    QHostAddress          m_radioAddress;
    quint32               m_seqNum{0};
    std::atomic<double>   m_rxFreqHz{14225000.0};
    std::atomic<quint32>  m_sampleRate{384000};
    bool                  m_running{false};
};

} // namespace AetherSDR
```

- [ ] **Step 2: Write `HpsdrP2Connection.cpp`**

```cpp
// src/hpsdr/HpsdrP2Connection.cpp
#include "HpsdrP2Connection.h"
#include "core/LogManager.h"
#include <QNetworkDatagram>

namespace AetherSDR {

static constexpr quint16 HPSDR_RADIO_PORT    = 1024;
static constexpr int     CONTROL_INTERVAL_MS = 1;

HpsdrP2Connection::HpsdrP2Connection(QObject* parent) : QObject(parent) {
    connect(&m_socket,       &QUdpSocket::readyRead, this, &HpsdrP2Connection::onReadyRead);
    connect(&m_controlTimer, &QTimer::timeout,       this, &HpsdrP2Connection::onControlTimer);
    m_controlTimer.setInterval(CONTROL_INTERVAL_MS);
}

HpsdrP2Connection::~HpsdrP2Connection() { disconnectFromRadio(); }

bool HpsdrP2Connection::connectToRadio(const HpsdrRadioInfo& info) {
    m_radioAddress = info.address;
    if (!m_socket.bind(QHostAddress::AnyIPv4, 0)) {
        qCWarning(lcHpsdr) << "P2Connection: bind failed:" << m_socket.errorString();
        return false;
    }
    m_running = true;
    sendStartPacket();
    m_controlTimer.start();
    qCInfo(lcHpsdr) << "P2Connection: started, local port" << m_socket.localPort();
    return true;
}

void HpsdrP2Connection::disconnectFromRadio() {
    if (!m_running) return;
    m_controlTimer.stop();
    sendStopPacket();
    m_socket.close();
    m_running = false;
}

void HpsdrP2Connection::setRxFrequency(double hz) { m_rxFreqHz.store(hz); }
void HpsdrP2Connection::setSampleRate(quint32 rate) { m_sampleRate.store(rate); }

void HpsdrP2Connection::sendStartPacket() {
    // Verify start vs control packet difference in Thetis protocol2.cs StartStream()
    // Some P2 implementations use the first control packet as the start signal.
    sendControlPacket();
}

void HpsdrP2Connection::sendStopPacket() {
    // Verify byte layout in Thetis protocol2.cs StopStream()
    QByteArray pkt(63, 0x00);
    pkt[0] = static_cast<char>(0xEF);
    pkt[1] = static_cast<char>(0xFE);
    pkt[2] = 0x08;  // stop type — verify in Thetis
    m_socket.writeDatagram(pkt, m_radioAddress, HPSDR_RADIO_PORT);
}

QByteArray HpsdrP2Connection::buildControlPacket() const {
    // Cross-reference ALL offsets with Thetis protocol2.cs SendControlPacket()
    // before trusting the byte positions below.
    QByteArray pkt(64, 0x00);
    pkt[0] = static_cast<char>(0xEF);
    pkt[1] = static_cast<char>(0xFE);
    pkt[2] = 0x04;  // PC→Radio data — verify in Thetis

    // Sequence number (big-endian uint32) — offset verify in Thetis:
    quint32 seq = m_seqNum;
    pkt[4] = (seq >> 24) & 0xFF;
    pkt[5] = (seq >> 16) & 0xFF;
    pkt[6] = (seq >>  8) & 0xFF;
    pkt[7] =  seq        & 0xFF;

    // RX frequency (big-endian uint32 Hz) — offset verify in Thetis:
    quint32 freqHz = static_cast<quint32>(m_rxFreqHz.load());
    pkt[8]  = (freqHz >> 24) & 0xFF;
    pkt[9]  = (freqHz >> 16) & 0xFF;
    pkt[10] = (freqHz >>  8) & 0xFF;
    pkt[11] =  freqHz        & 0xFF;

    // Sample rate code — encoding verify in Thetis:
    quint32 sr = m_sampleRate.load();
    quint8 rateCode = (sr <= 48000) ? 0 : (sr <= 96000) ? 1 : (sr <= 192000) ? 2
                    : (sr <= 384000) ? 3 : (sr <= 768000) ? 4 : 5;
    pkt[12] = rateCode;  // offset verify in Thetis

    return pkt;
}

void HpsdrP2Connection::sendControlPacket() {
    m_socket.writeDatagram(buildControlPacket(), m_radioAddress, HPSDR_RADIO_PORT);
}

void HpsdrP2Connection::onControlTimer() {
    if (m_running) { ++m_seqNum; sendControlPacket(); }
}

void HpsdrP2Connection::onReadyRead() {
    while (m_socket.hasPendingDatagrams()) {
        QNetworkDatagram dg = m_socket.receiveDatagram();
        const QByteArray data = dg.data();
        // Verify magic and IQ type byte in Thetis protocol2.cs ProcessPacket():
        if (data.size() < 8) continue;
        if (static_cast<quint8>(data[0]) != 0xEF) continue;
        if (static_cast<quint8>(data[1]) != 0xFE) continue;
        if (static_cast<quint8>(data[2]) != 0x06) continue;  // IQ data — verify in Thetis

        // Strip header; emit raw IQ payload (24-bit signed big-endian, I then Q per sample)
        // Verify header size in Thetis:
        constexpr int HEADER_SIZE = 8;
        emit iqReady(data.mid(HEADER_SIZE));
    }
}

} // namespace AetherSDR
```

- [ ] **Step 3: Build**

```bash
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) 2>&1 | grep "error:" | head -20
```

- [ ] **Step 4: Commit**

```bash
git add src/hpsdr/HpsdrP2Connection.h src/hpsdr/HpsdrP2Connection.cpp
git commit -m "hpsdr: HpsdrP2Connection — P2 UDP control and IQ reception"
```

---

## Task 5: HpsdrDsp — FFT Path (Spectrum)

**Files:**
- Implement (partial): `src/hpsdr/HpsdrDsp.h/.cpp` — FFT path only

Use a **complex-to-complex FFT** (`fftwf_plan_dft_1d`) on the raw IQ samples. This gives the correct double-sided spectrum; bins 0..N/2 map to positive frequencies (above center) and N/2+1..N-1 map to negative frequencies (below center). Avoid the real FFT on magnitude — it produces a one-sided, distorted spectrum.

- [ ] **Step 1: Write `HpsdrDsp.h`**

```cpp
// src/hpsdr/HpsdrDsp.h
#pragma once
#include <QObject>
#include <QVector>
#include <QString>
#include <atomic>
#include <fftw3.h>

namespace AetherSDR {

class HpsdrDsp : public QObject {
    Q_OBJECT
public:
    static constexpr int FFT_SIZE       = 4096;
    static constexpr int FFT_FPS_TARGET = 30;

    explicit HpsdrDsp(QObject* parent = nullptr);
    ~HpsdrDsp() override;

    void setSampleRate(int rate);
    void setCenterFrequency(double hz);  // atomic — main thread safe
    void setRxFrequency(double hz);      // atomic — main thread safe
    void setMode(const QString& mode);   // main thread only

public slots:
    // Connected via Qt::QueuedConnection from P2Connection (different thread)
    void processIq(const QByteArray& raw24bitIq);

signals:
    void fftReady(quint64 centerHz, float bandwidthHz, QVector<float> binsDbfs);
    void pcmReady(QByteArray int16Stereo24k);

private:
    void initFft();
    void destroyFft();
    void computeFirCoeffs();
    void updateNco();
    void runFft();

    // FFT (complex input)
    fftwf_complex* m_fftIn{nullptr};
    fftwf_complex* m_fftOut{nullptr};
    fftwf_plan     m_fftPlan{nullptr};
    QVector<float> m_window;         // Hann window (size FFT_SIZE)
    int            m_fftAccumPos{0}; // counts IQ pairs accumulated
    int            m_fftSkip{1};
    int            m_fftSkipCount{0};

    // Audio demod
    QVector<float> m_firCoeffs;      // low-pass FIR coefficients
    QVector<float> m_firStateI;      // FIR delay line for I channel
    QVector<float> m_firStateQ;      // FIR delay line for Q channel
    int            m_decimRatio{8};  // 384k → 48k

    float          m_ncoPhase{0.0f};
    float          m_ncoPhaseInc{0.0f};
    int            m_audioAccumPos{0};
    QVector<float> m_audio48kBuf;    // intermediate 48k samples
    QByteArray     m_audioOutBuf;    // output int16 stereo 24k

    std::atomic<double>  m_centerHz{14225000.0};
    std::atomic<double>  m_rxHz{14225000.0};
    std::atomic<int>     m_mode{0};  // 0=USB 1=LSB 2=AM 3=CW
    int                  m_sampleRate{384000};
};

} // namespace AetherSDR
```

- [ ] **Step 2: Implement FFT init and `runFft()` in `HpsdrDsp.cpp`**

```cpp
// src/hpsdr/HpsdrDsp.cpp
#include "HpsdrDsp.h"
#include "core/LogManager.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace AetherSDR {

static constexpr float kPi = static_cast<float>(M_PI);
static constexpr float kTwoPi = 2.0f * kPi;

HpsdrDsp::HpsdrDsp(QObject* parent) : QObject(parent) {
    initFft();
    computeFirCoeffs();
}

HpsdrDsp::~HpsdrDsp() { destroyFft(); }

void HpsdrDsp::setSampleRate(int rate) {
    m_sampleRate = rate;
    m_decimRatio = rate / 48000;
    m_fftSkip    = std::max(1, (rate / FFT_SIZE) / FFT_FPS_TARGET);
    computeFirCoeffs();
}

void HpsdrDsp::setCenterFrequency(double hz) { m_centerHz.store(hz); updateNco(); }
void HpsdrDsp::setRxFrequency(double hz)     { m_rxHz.store(hz);     updateNco(); }

void HpsdrDsp::setMode(const QString& mode) {
    if      (mode == "USB") m_mode.store(0);
    else if (mode == "LSB") m_mode.store(1);
    else if (mode == "AM")  m_mode.store(2);
    else if (mode == "CW")  m_mode.store(3);
}

void HpsdrDsp::updateNco() {
    double offset = m_rxHz.load() - m_centerHz.load();
    m_ncoPhaseInc = static_cast<float>(kTwoPi * offset / m_sampleRate);
}

void HpsdrDsp::initFft() {
    m_fftIn  = fftwf_alloc_complex(FFT_SIZE);
    m_fftOut = fftwf_alloc_complex(FFT_SIZE);
    // Forward complex FFT (negative exponent) — correct for baseband IQ
    m_fftPlan = fftwf_plan_dft_1d(FFT_SIZE, m_fftIn, m_fftOut, FFTW_FORWARD, FFTW_ESTIMATE);

    // Hann window
    m_window.resize(FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; ++i)
        m_window[i] = 0.5f * (1.0f - std::cos(kTwoPi * i / (FFT_SIZE - 1)));
}

void HpsdrDsp::destroyFft() {
    if (m_fftPlan) { fftwf_destroy_plan(m_fftPlan); m_fftPlan = nullptr; }
    if (m_fftIn)   { fftwf_free(m_fftIn);  m_fftIn  = nullptr; }
    if (m_fftOut)  { fftwf_free(m_fftOut); m_fftOut = nullptr; }
}

void HpsdrDsp::computeFirCoeffs() {
    constexpr int TAPS = 128;
    m_firCoeffs.resize(TAPS);
    m_firStateI.fill(0.0f, TAPS);
    m_firStateQ.fill(0.0f, TAPS);

    float fc = 0.9f / static_cast<float>(m_decimRatio);  // normalised cutoff
    float sum = 0.0f;
    for (int i = 0; i < TAPS; ++i) {
        float n = i - (TAPS - 1) / 2.0f;
        float sinc = (n == 0.0f) ? 1.0f : std::sin(kPi * fc * n) / (kPi * fc * n);
        float win  = 0.54f - 0.46f * std::cos(kTwoPi * i / (TAPS - 1));
        m_firCoeffs[i] = sinc * win;
        sum += m_firCoeffs[i];
    }
    for (float& c : m_firCoeffs) c /= sum;
}

// Convert 24-bit signed big-endian to float in [-1, 1]
static float s24beToFloat(const char* p) {
    qint32 v = (static_cast<quint8>(p[0]) << 16)
             | (static_cast<quint8>(p[1]) <<  8)
             |  static_cast<quint8>(p[2]);
    if (v & 0x800000) v |= 0xFF000000;
    return v / 8388608.0f;
}

void HpsdrDsp::processIq(const QByteArray& raw) {
    // Each sample = 6 bytes: 3 bytes I (24-bit signed BE) + 3 bytes Q
    // Verify I-before-Q byte order in Thetis protocol2.cs ProcessPacket()
    const int numSamples = raw.size() / 6;
    const char* p = raw.constData();

    for (int i = 0; i < numSamples; ++i, p += 6) {
        float iSam = s24beToFloat(p);
        float qSam = s24beToFloat(p + 3);

        // --- FFT path: accumulate complex IQ pairs ---
        if (m_fftAccumPos < FFT_SIZE) {
            m_fftIn[m_fftAccumPos][0] = iSam * m_window[m_fftAccumPos];
            m_fftIn[m_fftAccumPos][1] = qSam * m_window[m_fftAccumPos];
            ++m_fftAccumPos;
        }
        if (m_fftAccumPos >= FFT_SIZE) {
            m_fftAccumPos = 0;
            if (++m_fftSkipCount >= m_fftSkip) {
                m_fftSkipCount = 0;
                runFft();
            }
        }

        // --- Audio demod path (Task 7) ---
        // TODO: added in Task 7
    }
}

void HpsdrDsp::runFft() {
    fftwf_execute(m_fftPlan);

    // Complex FFT output: bin k covers frequency center + k * (sampleRate / FFT_SIZE).
    // Bins 0..N/2   → positive frequencies (above center)
    // Bins N/2+1..N-1 → negative frequencies (below center) — fftshift to display correctly
    QVector<float> dbfs(FFT_SIZE);
    const float norm = 1.0f / (FFT_SIZE * FFT_SIZE);
    for (int k = 0; k < FFT_SIZE; ++k) {
        float re = m_fftOut[k][0];
        float im = m_fftOut[k][1];
        float mag2 = (re * re + im * im) * norm;
        dbfs[k] = (mag2 > 0.0f) ? 10.0f * std::log10(mag2) : -140.0f;
    }

    // FFT-shift: swap first and second halves so DC is in the centre
    // (negative frequencies on the left, positive on the right)
    std::rotate(dbfs.begin(), dbfs.begin() + FFT_SIZE / 2, dbfs.end());

    emit fftReady(static_cast<quint64>(m_centerHz.load()),
                  static_cast<float>(m_sampleRate),
                  dbfs);
}

} // namespace AetherSDR
```

- [ ] **Step 3: Build**

```bash
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) 2>&1 | grep "error:" | head -20
```

- [ ] **Step 4: Commit**

```bash
git add src/hpsdr/HpsdrDsp.h src/hpsdr/HpsdrDsp.cpp
git commit -m "hpsdr: HpsdrDsp — complex FFT path for spectrum display"
```

---

## Task 6: Injection Point — SpectrumWidget::feedFftBins

**Files:**
- Modify: `src/gui/SpectrumWidget.h/.cpp`

- [ ] **Step 1: Add slot to `SpectrumWidget.h`**

In `public slots:`:
```cpp
#ifdef HAVE_HPSDR
    void feedFftBins(quint64 centerHz, float bandwidthHz, const QVector<float>& binsDbfs);
#endif
```

- [ ] **Step 2: Implement in `SpectrumWidget.cpp`**

```cpp
#ifdef HAVE_HPSDR
void SpectrumWidget::feedFftBins(quint64 centerHz, float bandwidthHz,
                                  const QVector<float>& binsDbfs) {
    double centerMhz    = static_cast<double>(centerHz)    / 1e6;
    double bandwidthMhz = static_cast<double>(bandwidthHz) / 1e6;
    setFrequencyRange(centerMhz, bandwidthMhz);
    updateSpectrum(binsDbfs);  // bins are dBFS (not dBm); Y-axis uncalibrated for MVP
}
#endif
```

- [ ] **Step 3: Build**

```bash
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) 2>&1 | grep "error:" | head -10
```

- [ ] **Step 4: Commit**

```bash
git add src/gui/SpectrumWidget.h src/gui/SpectrumWidget.cpp
git commit -m "hpsdr: SpectrumWidget::feedFftBins injection point"
```

---

## Task 7: HpsdrDsp — Audio Demodulation Path

**Files:**
- Extend: `src/hpsdr/HpsdrDsp.cpp`

- [ ] **Step 1: Add audio demod to the `processIq` loop**

In `HpsdrDsp::processIq`, after the FFT accumulation block, add:

```cpp
        // --- Audio demod path ---
        // Frequency translate: mix down to baseband centered on slice frequency
        float cosP =  std::cos(m_ncoPhase);
        float sinP =  std::sin(m_ncoPhase);
        float tI   =  iSam * cosP + qSam * sinP;
        float tQ   = -iSam * sinP + qSam * cosP;
        m_ncoPhase += m_ncoPhaseInc;
        if (m_ncoPhase >  kPi) m_ncoPhase -= kTwoPi;

        // FIR low-pass filter (separate state for I and Q)
        auto applyFir = [&](QVector<float>& state, float input) -> float {
            std::copy_backward(state.begin(), state.end() - 1, state.end());
            state[0] = input;
            float acc = 0.0f;
            for (int k = 0; k < m_firCoeffs.size(); ++k) acc += m_firCoeffs[k] * state[k];
            return acc;
        };
        float fI = applyFir(m_firStateI, tI);
        float fQ = applyFir(m_firStateQ, tQ);

        // Decimate: keep every m_decimRatio-th sample
        if (++m_audioAccumPos < m_decimRatio) continue;
        m_audioAccumPos = 0;

        // Demodulate (analytic signal — I and Q are in quadrature after FIR):
        float audio = 0.0f;
        switch (m_mode.load()) {
            case 0: audio =  fI + fQ; break;   // USB: Re(I + jQ)
            case 1: audio =  fI - fQ; break;   // LSB: Re(I - jQ)
            case 2: audio =  std::sqrt(fI * fI + fQ * fQ); break;  // AM: envelope
            case 3: audio =  fI + fQ; break;   // CW: USB path (BFO offset via NCO)
        }

        // Resample 48k → 24k (simple 2-sample average; upgrade to half-band FIR later)
        m_audio48kBuf.append(audio);
        if (m_audio48kBuf.size() < 2) continue;
        float s24k = (m_audio48kBuf[0] + m_audio48kBuf[1]) * 0.5f;
        m_audio48kBuf.clear();

        // float → int16 stereo (duplicate mono to L and R)
        qint16 s16 = static_cast<qint16>(
            std::clamp(s24k * 32767.0f, -32768.0f, 32767.0f));
        m_audioOutBuf.append(reinterpret_cast<const char*>(&s16), 2);  // L
        m_audioOutBuf.append(reinterpret_cast<const char*>(&s16), 2);  // R

        // Emit in 10ms chunks (480 stereo frames × 4 bytes = 1920 bytes)
        constexpr int EMIT_BYTES = 480 * 4;
        if (m_audioOutBuf.size() >= EMIT_BYTES) {
            emit pcmReady(m_audioOutBuf.left(EMIT_BYTES));
            m_audioOutBuf.remove(0, EMIT_BYTES);
        }
```

- [ ] **Step 2: Build**

```bash
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) 2>&1 | grep "error:" | head -20
```

- [ ] **Step 3: Commit**

```bash
git add src/hpsdr/HpsdrDsp.h src/hpsdr/HpsdrDsp.cpp
git commit -m "hpsdr: HpsdrDsp — NCO/FIR/demod audio path (USB/LSB/AM/CW)"
```

---

## Task 8: Injection Point — AudioEngine::feedHpsdrAudio

**Files:**
- Modify: `src/core/AudioEngine.h/.cpp`

- [ ] **Step 1: Add slot to `AudioEngine.h`**

In `public slots:`:
```cpp
#ifdef HAVE_HPSDR
    void feedHpsdrAudio(const QByteArray& int16Stereo24k);
#endif
```

- [ ] **Step 2: Implement in `AudioEngine.cpp`**

First, read `AudioEngine.cpp` to find the exact member variable names for the `QAudioSink` and its write device (search for `QAudioSink`, `m_audioSink`, `m_audioDevice`, or similar). Then:

```cpp
#ifdef HAVE_HPSDR
void AudioEngine::feedHpsdrAudio(const QByteArray& pcm) {
    // Write directly to the output sink, bypassing NR/DSP pipeline.
    // Volume and mute are applied at the QAudioSink level automatically
    // (set via setVolume() in startRxStream() — so startRxStream() must be
    // called before feeding HPSDR audio; see Task 11).
    // Use the correct state constant: check for NOT stopped, not for Active.
    if (m_audioSink && m_audioSink->state() != QAudio::StoppedState
            && m_audioDevice) {
        m_audioDevice->write(pcm);
    }
}
#endif
```

Replace `m_audioSink` and `m_audioDevice` with the actual member names found in `AudioEngine.cpp`.

- [ ] **Step 3: Build**

```bash
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) 2>&1 | grep "error:" | head -10
```

- [ ] **Step 4: Commit**

```bash
git add src/core/AudioEngine.h src/core/AudioEngine.cpp
git commit -m "hpsdr: AudioEngine::feedHpsdrAudio injection point"
```

---

## Task 9: HpsdrSliceModel

**Files:**
- Modify: `src/models/SliceModel.h` — make three setters virtual
- Implement: `src/hpsdr/HpsdrSliceModel.h/.cpp`

- [ ] **Step 1: Make the three setters virtual in `SliceModel.h`**

`HpsdrSliceModel` inherits `SliceModel` and is passed to widgets as `SliceModel*`. Without `virtual`, the widget calls `SliceModel::setFrequency()` — the override never fires and the Anan never gets frequency updates. Add `virtual` to these three declarations:

```cpp
// src/models/SliceModel.h — change these three lines:
virtual void setFrequency(double mhz);
virtual void setMode(const QString& mode);
virtual void setFilterWidth(int low, int high);
```

Also add a `virtual` destructor to `SliceModel` if one is not already present (required when a class has virtual methods and is used polymorphically). The existing FlexRadio code path is unaffected — `SliceModel` instances are still called the same way.

- [ ] **Step 2: Build after making setters virtual**

```bash
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) 2>&1 | grep "error:" | head -10
```

Expected: clean build — no changes to existing behaviour.

- [ ] **Step 3: Write `HpsdrSliceModel.h`**

```cpp
// src/hpsdr/HpsdrSliceModel.h
#pragma once
#include "models/SliceModel.h"

namespace AetherSDR {

class HpsdrP2Connection;
class HpsdrDsp;

class HpsdrSliceModel : public SliceModel {
    Q_OBJECT
public:
    explicit HpsdrSliceModel(HpsdrP2Connection* conn, HpsdrDsp* dsp,
                             QObject* parent = nullptr);

    void setFrequency(double mhz) override;
    void setMode(const QString& mode) override;
    void setFilterWidth(int low, int high) override;

private:
    HpsdrP2Connection* m_conn;
    HpsdrDsp*          m_dsp;
};

} // namespace AetherSDR
```

- [ ] **Step 4: Write `HpsdrSliceModel.cpp`**

```cpp
// src/hpsdr/HpsdrSliceModel.cpp
#include "HpsdrSliceModel.h"
#include "HpsdrP2Connection.h"
#include "HpsdrDsp.h"

namespace AetherSDR {

HpsdrSliceModel::HpsdrSliceModel(HpsdrP2Connection* conn, HpsdrDsp* dsp,
                                  QObject* parent)
    : SliceModel(0, parent), m_conn(conn), m_dsp(dsp)
{
    // Disconnect commandReady so SmartSDR commands are never emitted from this model.
    // This is cleaner than using QSignalBlocker in every setter override.
    QObject::disconnect(this, &SliceModel::commandReady, nullptr, nullptr);
}

void HpsdrSliceModel::setFrequency(double mhz) {
    // Guard: check locked state via base class getter (m_locked is private)
    if (isLocked()) return;
    // Call base to update m_frequency and emit frequencyChanged
    // (commandReady is disconnected in constructor so no SmartSDR command fires)
    SliceModel::setFrequency(mhz);
    // Route to HPSDR hardware
    m_conn->setRxFrequency(mhz * 1e6);
    m_dsp->setRxFrequency(mhz * 1e6);
}

void HpsdrSliceModel::setMode(const QString& mode) {
    SliceModel::setMode(mode);
    m_dsp->setMode(mode);
}

void HpsdrSliceModel::setFilterWidth(int low, int high) {
    SliceModel::setFilterWidth(low, high);
    // Filter width → DSP bandwidth adjustment is a follow-up enhancement
}

} // namespace AetherSDR
```

**Note:** `SliceModel::setFrequency` calls `sendCommand()` which calls `emit commandReady(cmd)`. Because we disconnected `commandReady` in the constructor, the signal fires but has no connected slots — effectively a no-op. The `frequencyChanged` signal still fires normally and updates the UI.

- [ ] **Step 5: Build**

```bash
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) 2>&1 | grep "error:" | head -20
```

- [ ] **Step 6: Commit**

```bash
git add src/models/SliceModel.h \
        src/hpsdr/HpsdrSliceModel.h src/hpsdr/HpsdrSliceModel.cpp
git commit -m "hpsdr: HpsdrSliceModel — make SliceModel setters virtual, add HPSDR subclass"
```

---

## Task 10: HpsdrRadio Coordinator

**Files:**
- Implement: `src/hpsdr/HpsdrRadio.h/.cpp`

- [ ] **Step 1: Write `HpsdrRadio.h`**

```cpp
// src/hpsdr/HpsdrRadio.h
#pragma once
#include "HpsdrRadioInfo.h"
#include <QObject>
#include <QThread>
#include <memory>

namespace AetherSDR {

class HpsdrP2Connection;
class HpsdrDsp;
class HpsdrSliceModel;

class HpsdrRadio : public QObject {
    Q_OBJECT
public:
    explicit HpsdrRadio(QObject* parent = nullptr);
    ~HpsdrRadio() override;

    bool connectToRadio(const HpsdrRadioInfo& info);
    void disconnectFromRadio();

    HpsdrSliceModel* sliceModel() const { return m_slice.get(); }

signals:
    void connected();
    void disconnected();
    void connectionError(const QString& msg);
    void fftReady(quint64 centerHz, float bandwidthHz, QVector<float> binsDbfs);
    void pcmReady(QByteArray int16Stereo24k);

private:
    std::unique_ptr<HpsdrP2Connection> m_conn;
    std::unique_ptr<HpsdrDsp>          m_dsp;
    std::unique_ptr<HpsdrSliceModel>   m_slice;
    QThread                            m_dspThread;
    bool                               m_connected{false};
};

} // namespace AetherSDR
```

- [ ] **Step 2: Write `HpsdrRadio.cpp`**

```cpp
// src/hpsdr/HpsdrRadio.cpp
#include "HpsdrRadio.h"
#include "HpsdrP2Connection.h"
#include "HpsdrDsp.h"
#include "HpsdrSliceModel.h"
#include "core/AppSettings.h"
#include "core/LogManager.h"

namespace AetherSDR {

HpsdrRadio::HpsdrRadio(QObject* parent) : QObject(parent) {}

HpsdrRadio::~HpsdrRadio() { disconnectFromRadio(); }

bool HpsdrRadio::connectToRadio(const HpsdrRadioInfo& info) {
    int sampleRate = AppSettings::instance()
        .value("HpsdrSampleRate", "384000").toInt();

    m_conn  = std::make_unique<HpsdrP2Connection>();
    m_dsp   = std::make_unique<HpsdrDsp>();
    m_slice = std::make_unique<HpsdrSliceModel>(m_conn.get(), m_dsp.get());

    m_dsp->setSampleRate(sampleRate);
    m_dsp->setCenterFrequency(14225000.0);
    m_dsp->setRxFrequency(14225000.0);

    // DSP runs on its own thread
    m_dsp->moveToThread(&m_dspThread);
    m_dspThread.start();

    // IQ: P2Connection (main thread) → HpsdrDsp (DSP thread) — queued
    connect(m_conn.get(), &HpsdrP2Connection::iqReady,
            m_dsp.get(),  &HpsdrDsp::processIq,
            Qt::QueuedConnection);

    // Forward DSP output to MainWindow for wiring to widgets
    connect(m_dsp.get(), &HpsdrDsp::fftReady, this, &HpsdrRadio::fftReady);
    connect(m_dsp.get(), &HpsdrDsp::pcmReady, this, &HpsdrRadio::pcmReady);

    if (!m_conn->connectToRadio(info)) {
        qCWarning(lcHpsdr) << "HpsdrRadio: connection failed to" << info.address;
        disconnectFromRadio();
        emit connectionError(QString("Failed to connect to %1").arg(info.address.toString()));
        return false;
    }

    m_connected = true;
    emit connected();
    return true;
}

void HpsdrRadio::disconnectFromRadio() {
    if (m_conn) m_conn->disconnectFromRadio();
    m_dspThread.quit();
    m_dspThread.wait();
    m_conn.reset();
    m_dsp.reset();
    m_slice.reset();
    if (m_connected) {
        m_connected = false;
        emit disconnected();
    }
}

} // namespace AetherSDR
```

- [ ] **Step 3: Build**

```bash
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) 2>&1 | grep "error:" | head -20
```

- [ ] **Step 4: Commit**

```bash
git add src/hpsdr/HpsdrRadio.h src/hpsdr/HpsdrRadio.cpp
git commit -m "hpsdr: HpsdrRadio coordinator — wires P2, DSP, slice model"
```

---

## Task 11: MainWindow — Full Connect Flow

**Files:**
- Modify: `src/gui/MainWindow.h/.cpp`

- [ ] **Step 1: Add `HpsdrRadio` member to `MainWindow.h`**

```cpp
#ifdef HAVE_HPSDR
#include "hpsdr/HpsdrRadio.h"
#include "hpsdr/HpsdrRadioInfo.h"
private:
    std::unique_ptr<AetherSDR::HpsdrRadio> m_hpsdrRadio;
private slots:
    void onHpsdrConnectRequested(const AetherSDR::HpsdrRadioInfo& info);
#endif
```

- [ ] **Step 2: Wire `hpsdrConnectRequested` in MainWindow constructor/setup**

```cpp
#ifdef HAVE_HPSDR
    connect(m_connPanel, &ConnectionPanel::hpsdrConnectRequested,
            this, &MainWindow::onHpsdrConnectRequested);
#endif
```

- [ ] **Step 3: Read `MainWindow::onSliceAdded()` before writing the wiring code**

Read the full implementation of `onSliceAdded()` in `MainWindow.cpp` to understand exactly how `RxApplet::setSlice()`, `VfoWidget::setSlice()`, and `wireVfoWidget()` are called. Mirror that pattern for `HpsdrSliceModel*` in the next step.

- [ ] **Step 4: Implement `onHpsdrConnectRequested()`**

```cpp
#ifdef HAVE_HPSDR
void MainWindow::onHpsdrConnectRequested(const HpsdrRadioInfo& info) {
    // Disconnect any existing radio
    if (m_radioModel.isConnected()) m_radioModel.disconnectFromRadio();
    if (m_hpsdrRadio) m_hpsdrRadio.reset();

    m_hpsdrRadio = std::make_unique<HpsdrRadio>(this);

    // Wire spectrum display
    if (SpectrumWidget* sw = spectrum()) {
        connect(m_hpsdrRadio.get(), &HpsdrRadio::fftReady,
                sw, &SpectrumWidget::feedFftBins,
                Qt::QueuedConnection);
    }

    // Wire audio — AutoConnection marshals to AudioEngine's thread.
    // startRxStream() MUST be called to open the QAudioSink before feeding audio.
    audioStartRx();
    connect(m_hpsdrRadio.get(), &HpsdrRadio::pcmReady,
            m_audio, &AudioEngine::feedHpsdrAudio);

    // Wire slice controls — HpsdrSliceModel* is-a SliceModel*, accepted by widgets unchanged.
    // Use the same wiring calls as onSliceAdded() — read that method first (Step 3).
    // Example (adapt to actual onSliceAdded pattern):
    //   if (m_panApplet) m_panApplet->rxApplet()->setSlice(m_hpsdrRadio->sliceModel());
    //   wireVfoWidget(spectrum()->vfoWidget(), m_hpsdrRadio->sliceModel());

    connect(m_hpsdrRadio.get(), &HpsdrRadio::connected, this, [this]{
        m_connStatusLabel->setText("Connected (Anan HPSDR)");
        m_connPanel->setConnected(true);
    });
    connect(m_hpsdrRadio.get(), &HpsdrRadio::disconnected, this, [this]{
        audioStopRx();
        m_connStatusLabel->setText("Disconnected");
        m_connPanel->setConnected(false);
    });
    connect(m_hpsdrRadio.get(), &HpsdrRadio::connectionError, this,
            [this](const QString& msg){ m_connStatusLabel->setText("Error: " + msg); });

    m_hpsdrRadio->connectToRadio(info);
}
#endif
```

- [ ] **Step 5: Handle disconnect in existing disconnect path**

Find where disconnect is triggered (e.g. `onConnectClicked()` when connected, or a dedicated disconnect handler) and add:

```cpp
#ifdef HAVE_HPSDR
    if (m_hpsdrRadio) {
        m_hpsdrRadio->disconnectFromRadio();
        m_hpsdrRadio.reset();
    }
#endif
```

- [ ] **Step 6: End-to-end hardware test**

```bash
QT_LOGGING_RULES="aether.hpsdr=true" ./build/AetherSDR.app/Contents/MacOS/AetherSDR
```

Test sequence:
1. Anan 10E visible in connection list
2. Connect → status "Connected (Anan HPSDR)"
3. Spectrum updates with visible noise floor
4. Tune VFO to 14.225 MHz USB → hear audio on an active frequency
5. Switch mode buttons (LSB/AM) → demodulation changes
6. Disconnect → status "Disconnected"

- [ ] **Step 7: Commit**

```bash
git add src/gui/MainWindow.h src/gui/MainWindow.cpp
git commit -m "hpsdr: MainWindow — full Anan connect/disconnect flow"
```

---

## Task 12: Update Architecture Documentation

**Files:**
- Modify: `docs/architecture-pipelines.md`

- [ ] **Step 1: Update thread count table**

Find the thread count documentation in `docs/architecture-pipelines.md` (currently states 11 threads) and update to reflect that HPSDR adds 2 conditional threads:

```
FlexRadio (SmartSDR): 11 threads
Anan (HPSDR):         12 threads (adds 1 dedicated DSP thread)

Note: The P2 receive loop is NOT a new thread — QUdpSocket::readyRead fires asynchronously
on the main thread's event loop. The only new thread is the HpsdrDsp worker thread.
Both are active only when an Anan radio is connected; FlexRadio mode is unchanged at 11.
```

- [ ] **Step 2: Commit**

```bash
git add docs/architecture-pipelines.md
git commit -m "docs: update thread count for HPSDR Anan support (11→12)"
```

---

## Protocol Verification Checklist

Before shipping, cross-check each item against Thetis source (`protocol2.cs`):

- [ ] Discovery request byte layout → `SendDiscoveryPacket()`
- [ ] Discovery reply parsing — board ID byte, MAC offset, fw version offset → `ProcessDiscoveryData()`
- [ ] Anan 10E board ID constant value in Thetis
- [ ] P2 Start packet vs first Control packet — identical or different? → `StartStream()`
- [ ] Control packet frequency field offset and byte order
- [ ] Control packet sample-rate encoding (bits and values)
- [ ] IQ data packet header size
- [ ] IQ byte order: I before Q, or Q before I? → `ProcessPacket()`
- [ ] Stop packet byte layout → `StopStream()`
