# Pico Testing & Mocking Strategie

**Datum:** 7. Oktober 2025  
**Basierend auf:** Raspberry Pi Pico SDK, pico-examples, Forum-Diskussionen

## Übersicht

Es gibt **mehrere Strategien** zum Testen von Pico-Firmware ohne Hardware:

## 1. 🎯 Host-Build (EMPFOHLEN für unseren Use-Case)

### Was ist das?

Der Pico SDK unterstützt **`PICO_PLATFORM=host`** - dies kompiliert Code für Ihr **Host-System** (x86_64 Linux) statt für ARM Cortex-M0+.

### Wie funktioniert es?

```bash
cd firmware
mkdir -p build_host
cd build_host
cmake .. -DPICO_PLATFORM=host
make -j4
```

### ✅ Vorteile

- **Offiziell unterstützt** vom Pico SDK Team
- **Schnelle Iteration** - kein Flash-Prozess nötig
- **Debugger-Support** - GDB, Valgrind, etc.
- **Viele SDK-Funktionen** funktionieren bereits
- **Perfekt für Logik-Tests** (PID, Odometrie, State-Machines)

### ⚠️ Einschränkungen

- **Keine echte Hardware-Simulation** (GPIO, SPI, I2C sind Stubs)
- **Timing unterschiedlich** - keine Echtzeit-Garantien
- **DMA/PIO nicht verfügbar** - hardwarespezifische Features fehlen

### Was ist bereits gemockt im SDK?

Laut Forum-Diskussion und Pico SDK:

```c
// Diese Funktionen haben Host-Implementierungen:
get_absolute_time()        // ✅ Funktioniert
sleep_ms() / sleep_us()    // ✅ Funktioniert
printf() / stdio           // ✅ Funktioniert
malloc() / free()          // ✅ Standard-libc
mutex / semaphores         // ✅ pthread-basiert

// Diese sind nur Stubs (tun nichts):
gpio_init()                // ❌ Stub
gpio_set_dir()             // ❌ Stub
spi_init()                 // ❌ Stub
i2c_init()                 // ❌ Stub
pwm_*()                    // ❌ Stub
```

### Perfekt für

✅ **Algorithmus-Tests** (PID, Odometrie, Kinematik)  
✅ **Logik-Tests** (State-Machines, Decision-Making)  
✅ **Math-Libraries** (Eigen, Geometry)  
✅ **Protokoll-Handler** (Parser, Serialization)  
✅ **Timing-unabhängige** Tests

### Nicht geeignet für

❌ Hardware-Interaktion (GPIO-Timing, SPI/I2C Communication)  
❌ PIO State-Machines  
❌ DMA-Transfers  
❌ Interrupt-Timing  
❌ Real-Time Performance

---

## 2. 🔧 Eigene Mocks erstellen

### Strategie

Erstellen Sie **Hardware Abstraction Layer (HAL)** Interfaces und mocken Sie diese.

### Beispiel-Struktur

```cpp
// hal/gpio_interface.h
class IGPIOInterface {
public:
    virtual void init(uint pin, uint dir) = 0;
    virtual bool get(uint pin) = 0;
    virtual void put(uint pin, bool value) = 0;
};

// hal/gpio_pico.cpp (Echte Hardware)
class GPIOPico : public IGPIOInterface {
    void init(uint pin, uint dir) override {
        gpio_init(pin);
        gpio_set_dir(pin, dir);
    }
    // ...
};

// hal/gpio_mock.cpp (Tests)
class GPIOMock : public IGPIOInterface {
    std::map<uint, bool> pinStates;
    void init(uint pin, uint dir) override { /* Track */ }
    bool get(uint pin) override { return pinStates[pin]; }
    void put(uint pin, bool value) override { pinStates[pin] = value; }
};
```

### Vorteile

✅ **Volle Kontrolle** über Mock-Verhalten  
✅ **Testbare Szenarien** (Fehler injizieren, Timing steuern)  
✅ **Gut dokumentiert** - Ihr eigener Code

### Nachteile

❌ **Viel Arbeit** - Interfaces für alles erstellen  
❌ **Wartung** - Bei SDK-Updates anpassen  
❌ **Performance-Overhead** - Virtuelle Funktionen

---

## 3. 🧪 GoogleTest + Mocks (UNSERE AKTUELLE LÖSUNG)

### Was wir bereits haben

```cpp
// tests/test_motor_pid.cpp
class MockMotorPID {  // Vereinfachte Version ohne Hardware
    float pid(float sp, float pv) { /* Pure math */ }
};
```

### Strategie

- **Reine Mathematik** extrahieren und testen
- **Business-Logik** von Hardware trennen
- **Mock-Klassen** für Tests erstellen (keine Pico SDK Calls)

### Aktuelle Coverage

- ✅ PID-Berechnungen (8 Tests)
- ✅ Odometrie-Mathematik (8 Tests)
- ✅ Math-Utilities (10 Tests)

### Nächste Schritte

1. **Mehr Logik** extrahieren (z.B. MotorsAgent Zustandsmaschine)
2. **Interfaces** für Hardware einführen
3. **Dependency Injection** verwenden

---

## 4. 🎮 Renode / QEMU (Hardware-Emulation)

### Was ist das?

**Vollständige ARM Cortex-M0+ Emulation** mit virtueller Hardware.

### Tools

- **Renode** - <https://renode.io/>
- **QEMU** - <https://www.qemu.org/>

### Beispiel

```bash
renode -e "mach create; machine LoadPlatformDescription @platforms/cpus/rp2040.repl"
```

### Vorteile

✅ **Fast echte Hardware** - GPIO, Timers, etc. emuliert  
✅ **Deterministisch** - Reproduzierbare Tests  
✅ **GDB-Support** - Debugging wie auf echter Hardware

### Nachteile

❌ **Komplex** - Setup und Konfiguration aufwendig  
❌ **Langsam** - Emulation ist deutlich langsamer  
❌ **Nicht alles unterstützt** - PIO oft nicht vollständig

---

## 📋 Empfohlene Strategie für my_firmware

### 3-Schichten-Ansatz

```txt
┌─────────────────────────────────────────┐
│  Application Logic (Testbar)           │  ← GoogleTest
│  - PID Controllers                      │
│  - Odometry Calculations                │
│  - State Machines                       │
└─────────────────────────────────────────┘
           ↓ Interfaces ↓
┌─────────────────────────────────────────┐
│  Hardware Abstraction Layer (HAL)       │  ← Host Mocks
│  - IGPIOInterface                       │
│  - IMotorInterface                      │
│  - ISensorInterface                     │
└─────────────────────────────────────────┘
           ↓ Real Hardware ↓
┌─────────────────────────────────────────┐
│  Pico SDK Hardware                      │  ← Nur auf Pico
│  - gpio_* calls                         │
│  - pwm_* calls                          │
│  - DMA, PIO, etc.                       │
└─────────────────────────────────────────┘
```

### Test-Matrix

| Test-Typ              | Tool               | Wo            | Geschwindigkeit |
| --------------------- | ------------------ | ------------- | --------------- |
| **Unit Tests**        | GoogleTest         | Host (x86_64) | ⚡ Sehr schnell  |
| **Integration Tests** | Host-Build         | Host (x86_64) | ⚡ Schnell       |
| **Hardware Tests**    | Pico + Serial      | Echter Pico   | 🐌 Langsam       |
| **End-to-End**        | Robot + Simulation | Gazebo        | 🐌 Langsam       |

---

## 🚀 Quick Reference

### Tests ausführen

```bash
# Unit Tests (GoogleTest, ohne Hardware)
make test

# Host-Build (mit Pico SDK stubs)
make build-host

# Spezifische Test-Suite
make test-pid
make test-odometry
make test-math
```

### Neue Tests hinzufügen

```bash
# 1. Erstelle Test-Datei
cd firmware/tests
touch test_my_feature.cpp

# 2. Füge zu CMakeLists.txt hinzu
# add_executable(firmware_tests
#     ...
#     test_my_feature.cpp
# )

# 3. Build und teste
cd build && make -j4 && ./firmware_tests
```

---

## 📚 Weitere Ressourcen

- [Pico SDK Examples](https://github.com/raspberrypi/pico-examples)
- [Pico Forum - Mocking Discussion](https://forums.raspberrypi.com/viewtopic.php?t=364096)
- [GoogleTest Documentation](https://google.github.io/googletest/)
- [Renode Pico Support](https://renode.readthedocs.io/en/latest/)

---

## ✅ Was wir NICHT brauchen

Für unsere aktuelle Firmware brauchen wir **KEIN**:

- ❌ Vollständige Hardware-Emulation (Renode/QEMU)
- ❌ Komplexe Mock-Frameworks (FakeIt, GMock)
- ❌ Pico-in-the-Loop Testing
- ❌ JTAG/SWD Debugging-Setup für Tests

**Warum?** Unsere Firmware hat **klare Trennung**:

- Mathematik/Logik → **GoogleTest** (bereits vorhanden)
- Hardware-Interaktion → **Integration Tests** auf echtem Robot
- System-Tests → **Gazebo Simulation**

---

## 🎯 Fazit

**Für my_firmware empfehle ich:**

1. **GoogleTest** für reine Logik (✅ bereits implementiert)
2. **HAL-Interfaces** einführen für bessere Testbarkeit
3. **Host-Build** (`PICO_PLATFORM=host`) für schnelle Integration-Tests
4. **Hardware-Tests** auf echtem Pico nur für kritische Features

**Nächster Schritt:** HAL-Layer einführen für bessere Dependency Injection
