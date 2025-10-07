# Firmware Unit Tests

## Übersicht

Dieses Verzeichnis enthält Unit-Tests für die Firmware-Komponenten. Die Tests laufen auf dem **Host-System** (x86_64 Linux), nicht auf dem Pico.

## Test-Suites

### ✅ test_motor_pid.cpp (8 Tests)
Tests für PID-Regler-Logik:
- PID-Konfiguration
- Proportional-, Integral-, Differential-Anteile
- Kombinierte PID-Berechnung
- Fehlerbehandlung (positiv/negativ)
- Reset-Funktionalität

### ✅ test_odometry.cpp (8 Tests)
Tests für Odometrie-Berechnungen:
- Vorwärts-/Rückwärtsbewegung
- Reine Rotation
- Differentialantrieb (gerade Linie, Kurven)
- Bewegung nach Rotation
- Reset-Funktionalität

### ✅ test_basic_math.cpp (10 Tests)
Tests für mathematische Hilfsfunktionen:
- RPM ↔ Rad/s Konvertierung
- Radumfang-Berechnung
- Grad ↔ Radiant Konvertierung
- Winkel-Normalisierung
- Distanz-Berechnung
- Geschwindigkeit → Motorspeed
- Sign, Clamp, Interpolation

## Bauen und Ausführen

### Schnellstart

```bash
cd firmware/tests
mkdir -p build && cd build
cmake ..
make -j4
./firmware_tests
```

### Mit farbiger Ausgabe

```bash
./firmware_tests --gtest_color=yes
```

### Einzelne Test-Suite ausführen

```bash
./firmware_tests --gtest_filter=MotorPIDTest.*
./firmware_tests --gtest_filter=OdometryTest.*
./firmware_tests --gtest_filter=MathUtilsTest.*
```

### Mit CTest

```bash
ctest --output-on-failure
```

## Testergebnisse

```
[==========] Running 26 tests from 3 test suites.
[  PASSED  ] 26 tests.
```

**Status: ✅ Alle Tests bestehen**

## Test-Struktur

```
firmware/tests/
├── CMakeLists.txt           # Build-Konfiguration
├── README.md                # Diese Datei
├── TEST_SUMMARY.md          # Validierungsbericht
├── test_motor_pid.cpp       # PID-Regler Tests
├── test_odometry.cpp        # Odometrie Tests
└── test_basic_math.cpp      # Math-Utilities Tests
```

## Hinweise

- **Keine Hardware erforderlich**: Tests laufen rein mathematisch
- **GoogleTest Framework**: Lokal in `../googletest/`
- **C++17 Standard**: Wie die Firmware selbst
- **Mock-Klassen**: Vereinfachte Versionen der echten Firmware-Klassen
- **Schnell**: Alle 26 Tests in < 1ms

## Neue Tests hinzufügen

1. Erstelle neue `.cpp` Datei mit GoogleTest-Tests
2. Füge sie in `CMakeLists.txt` zu `add_executable()` hinzu
3. Rebuild und teste:
   ```bash
   cd build && make -j4 && ./firmware_tests
   ```

## CI/CD Integration

Diese Tests können in CI/CD-Pipelines verwendet werden:

```bash
cd firmware/tests
mkdir build && cd build
cmake ..
make -j4
ctest --output-on-failure
```

Exit-Code 0 = alle Tests bestanden
