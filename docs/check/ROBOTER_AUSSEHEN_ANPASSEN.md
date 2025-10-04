# Anleitung: Anpassen des Roboter-Aussehens

Dieses Dokument beschreibt, wie Sie das 3D-Modell des robot mit Ihren eigenen Dateien (z.B. aus STEP-Dateien konvertiert) anpassen können.

## 1. Übersicht der relevanten Dateien

Die visuellen Aspekte des Roboters werden durch URDF (Unified Robot Description Format) und XACRO-Dateien definiert. Die wichtigsten Dateien befinden sich im Verzeichnis `/home/marco/ros2_steel_ws/my_steel-robot_ws/src/robot_ros/robot_description/urdf/`.

- **`robot.urdf.xacro`**: Die Haupt-XACRO-Datei, die die verschiedenen Teile des Roboters zusammenfügt.
- **`robot_base.urdf.xacro`**: Definiert die Basis des Roboters und lädt die 3D-Modelle.
- **`robot_properties.urdf.xacro`**: **Die wichtigste Datei für Anpassungen.** Sie enthält die Variablen (Properties) für die Pfade zu den 3D-Modellen (Meshes) und die physikalischen Abmessungen.
- **`src/robot_ros/robot_description/meshes/`**: Das Verzeichnis, in dem die 3D-Modelle im STL-Format gespeichert sind. (In URDF/XACRO: `package://robot_description/meshes/...`)

## 2. Variablen für das Aussehen

In der Datei `robot_properties.urdf.xacro` finden Sie die folgenden Schlüsselvariablen, die das Aussehen bestimmen:

### Pfade zu den 3D-Modellen (Meshes)
Diese Variablen zeigen auf die zu ladenden STL-Dateien.

```xml
<xacro:property name="robot_base_mesh_file" value="package://robot_description/meshes/robot_base.stl" />
<xacro:property name="robot_wheel_mesh_file" value="package://robot_description/meshes/wheel.stl" />
<xacro:property name="robot_front_caster_mesh_file" value="package://robot_description/meshes/caster_front.stl" />
<xacro:property name="robot_rear_caster_mesh_file" value="package://robot_description/meshes/caster_back.stl" />
```

### Abmessungen und physikalische Eigenschaften
Diese Variablen definieren die Größe und Masse der Komponenten, was für die Simulation wichtig ist.

```xml
<!-- Base properties -->
<xacro:property name="base_x_size" value="0.230" />
<xacro:property name="base_y_size" value="0.236" />
<xacro:property name="base_z_size" value="0.080" />
<xacro:property name="base_mass" value="1.5" />

<!-- Wheel properties -->
<xacro:property name="wheel_radius" value="0.0425" />
<xacro:property name="wheel_width" value="0.02" />
<xacro:property name="wheel_mass" value="0.2" />
```

## 3. Eigene STEP-Dateien verwenden: Schritt-für-Schritt

Um Ihre eigenen 3D-Modelle zu verwenden, müssen Sie diese zuerst in das STL-Format konvertieren.

**Schritt 1: STEP-Datei nach STL konvertieren**
Verwenden Sie eine beliebige CAD-Software (z.B. FreeCAD), um Ihre `.step` oder `.stp` Datei in eine `.stl` Datei zu exportieren.

**Schritt 2: STL-Datei in das Projekt kopieren**
Platzieren Sie Ihre neue STL-Datei in das Verzeichnis `/home/marco/ros2_steel_ws/my_steel-robot_ws/src/robot_ros/robot_description/meshes/`.
Platzieren Sie Ihre neue STL-Datei in das Verzeichnis `src/robot_ros/robot_description/meshes/` oder referenzieren Sie sie über `package://robot_description/meshes/your_file.stl`.

**Schritt 3: Pfad in der XACRO-Datei anpassen**
Öffnen Sie die Datei `src/robot_ros/robot_description/urdf/robot_properties.urdf.xacro`.

Ändern Sie den Wert der entsprechenden `xacro:property`, um auf Ihre neue Datei zu verweisen. Wenn Sie zum Beispiel das Gehäuse (`robot_base`) ersetzen möchten, ändern Sie die Zeile:

**Vorher:**
```xml
<xacro:property name="robot_base_mesh_file" value="package://robot_description/meshes/robot_base.stl" />
```

**Nachher (Beispiel):**
```xml
<xacro:property name="robot_base_mesh_file" value="package://robot_description/meshes/mein_neues_gehaeuse.stl" />
```

**Schritt 4: Workspace neu bauen**
Nachdem Sie die Änderungen gespeichert haben, müssen Sie Ihren colcon-Workspace neu bauen, damit die Änderungen wirksam werden:

```bash
colcon build --packages-select robot_description
```

Danach sollte das neue Modell in Visualisierungen (wie RViz) und Simulationen (wie Gazebo) angezeigt werden.
