# Anleitung zur Anpassung von Geometrie und Aussehen des Roboters

Dieses Dokument bietet eine detaillierte Anleitung, wie Sie die Geometrie und das visuelle Erscheinungsbild der Roboter (robot und robot XL) in diesem ROS2-Workspace anpassen können.

## Dateistruktur

Die folgende Tabelle gibt einen Überblick über die wichtigsten Dateien und Verzeichnisse, die für die Anpassung des Roboters relevant sind.

| Pfad                                                                                       | Typ         | Beschreibung                                                                                                                  |
| ------------------------------------------------------------------------------------------ | ----------- | ----------------------------------------------------------------------------------------------------------------------------- |
| [`src/robot_ros/robot_description/`](src/robot_ros/robot_description/)                     | Verzeichnis | Das Hauptpaket, das alle Beschreibungsdateien für die Roboter enthält.                                                        |
| [`.../urdf/`](src/robot_ros/robot_description/urdf/)                                       | Verzeichnis | Enthält die URDF-Dateien (`.xacro`), die die Struktur, Gelenke und die allgemeine Geometrie der Roboter definieren.           |
| [`.../meshes/`](src/robot_ros/robot_description/meshes/)                                   | Verzeichnis | Enthält die 3D-Modelle (Mesh-Dateien) für die einzelnen Teile der Roboter, die für die visuelle Darstellung verwendet werden. |
| [`.../urdf/robot.urdf.xacro`](src/robot_ros/robot_description/urdf/robot.urdf.xacro)       | Datei       | Die Haupt-Beschreibungsdatei für den **robot**. Hier werden die einzelnen Komponenten zusammengefügt.                         |
| [`.../urdf/robot_xl.urdf.xacro`](src/robot_ros/robot_description/urdf/robot_xl.urdf.xacro) | Datei       | Die Haupt-Beschreibungsdatei für den **robot XL**.                                                                            |
| [`.../meshes/robot/`](src/robot_ros/robot_description/meshes/robot/)                       | Verzeichnis | 3D-Modelle für den **robot** (z.B. `body.stl`, `cover.stl`).                                                                  |
| [`.../meshes/robot_xl/`](src/robot_ros/robot_description/meshes/robot_xl/)                 | Verzeichnis | 3D-Modelle für den **robot XL**.                                                                                              |

## Schritte zur Änderung

### 1. Geometrie anpassen (Struktur des Roboters)

Wenn Sie die physische Struktur des Roboters ändern möchten (z. B. die Position eines Sensors, die Größe eines Rads), müssen Sie die `.urdf.xacro`-Dateien bearbeiten.

1.  **Öffnen Sie die relevante URDF-Datei:**
    *   Für den robot: [`robot.urdf.xacro`](src/robot_ros/robot_description/urdf/robot.urdf.xacro) (oder in XACRO/URDF: use `package://robot_description/urdf/robot.urdf.xacro`)
    *   Für den robot XL: [`robot_xl.urdf.xacro`](src/robot_ros/robot_description/urdf/robot_xl.urdf.xacro) (oder `package://robot_description/urdf/robot_xl.urdf.xacro`)
2.  **Änderungen vornehmen:** Suchen Sie die `<link>`- und `<joint>`-Tags, die Sie anpassen möchten. Sie können hier Werte für `origin` (Position und Orientierung) oder `geometry` (Form und Größe) ändern.

### 2. Aussehen anpassen (3D-Modelle und Farben)

Wenn Sie das visuelle Erscheinungsbild ändern möchten, ohne die Struktur zu beeinflussen, können Sie die Mesh-Dateien oder die Materialfarben anpassen.

#### 2.1. 3D-Modelle (Meshes) austauschen

1.  **Navigieren Sie zum Mesh-Verzeichnis:**
    *   Für den robot: [`meshes/robot/`](src/robot_ros/robot_description/meshes/robot/) (URDF/mesh-URI verwenden in XACRO: `package://robot_description/meshes/robot/base.stl`)
    *   Für den robot XL: [`meshes/robot_xl/`](src/robot_ros/robot_description/meshes/robot_xl/) (oder `package://robot_description/meshes/robot_xl/...`)
2.  **Modelle ersetzen:** Ersetzen Sie eine vorhandene `.stl`- oder `.dae`-Datei durch Ihr eigenes 3D-Modell.
    *   **Wichtig:** Behalten Sie den ursprünglichen Dateinamen bei, um die Referenzen in den URDF-Dateien nicht brechen. Wenn Sie einen neuen Dateinamen verwenden, müssen Sie den Pfad in der entsprechenden `.urdf.xacro`-Datei im `<mesh>`-Tag anpassen.

#### 2.2. Farben ändern

Die Farben der Roboterteile, die keine Mesh-Datei verwenden oder deren Material im URDF definiert ist, können direkt geändert werden.

1.  **Öffnen Sie die relevante URDF-Datei.**
2.  **Suchen Sie nach `<material>`-Tags:** Ein Material wird z.B. so definiert:
    ```xml
    <material name="black">
      <color rgba="0.1 0.1 0.1 1.0"/>
    </material>
    ```
3.  **Farbwerte anpassen:** Ändern Sie die `rgba`-Werte (Rot, Grün, Blau, Alpha), um die gewünschte Farbe zu erhalten.

## 3. Änderungen anwenden

Nachdem Sie Änderungen an den Beschreibungsdateien vorgenommen haben, müssen Sie den ROS2-Workspace neu kompilieren, damit die Änderungen übernommen werden.

Führen Sie den folgenden Befehl im Root des Repositories (Arbeitsverzeichnis) aus:

```bash
colcon build --packages-select robot_description
```

Dieser Befehl kompiliert nur das `robot_description`-Paket, was schneller ist als ein vollständiger Build. Starten Sie danach Ihre Simulation oder Visualisierung neu, um den geänderten Roboter zu sehen.
