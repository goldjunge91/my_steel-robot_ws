# Anleitung zur Anpassung von Geometrie und Aussehen des Roboters

Diese Anleitung kombiniert die bisherigen Hinweise aus `ROBOTER_ANPASSEN.md` und `ROBOTER_AUSSEHEN_ANPASSEN.md`. Sie beschreibt, welche Dateien im Workspace für Anpassungen relevant sind und wie Sie bei Änderungen der Robotermodelle (robot und robot XL) vorgehen.
Diese Anleitung kombiniert die bisherigen Hinweise aus `ROBOTER_ANPASSEN.md` und `ROBOTER_AUSSEHEN_ANPASSEN.md`. Sie beschreibt, welche Dateien im Workspace für Anpassungen relevant sind und wie Sie bei Änderungen der Robotermodelle (robot und robot XL) vorgehen.

## Relevante Ressourcen

| Pfad                                                               | Typ         | Zweck                                                                                                |
| ------------------------------------------------------------------ | ----------- | ---------------------------------------------------------------------------------------------------- |
| `src/robot_ros/robot_description/`                                 | Verzeichnis | ROS-Paket mit allen Beschreibungen beider Robotervarianten.                                          |
| `src/robot_ros/robot_description/urdf/`                            | Verzeichnis | Enthält die XACRO-Dateien, die Struktur, Gelenke und Materialien definieren.                         |
| `src/robot_ros/robot_description/urdf/robot.urdf.xacro`            | Datei       | Haupteinstiegspunkt für den robot. Bindet Basis, Sensoren und Zubehör ein.                           |
| `src/robot_ros/robot_description/urdf/robot_xl.urdf.xacro`         | Datei       | Pendant für den robot XL.                                                                            |
| `src/robot_ros/robot_description/urdf/robot_base.urdf.xacro`       | Datei       | Definiert die Basisgeometrie und lädt die Meshes des robot.                                          |
| `src/robot_ros/robot_description/urdf/robot_properties.urdf.xacro` | Datei       | Bündelt Parameter wie Mesh-Pfade, Abmessungen und Massen – zentrale Stelle für optische Anpassungen. |
| `src/robot_ros/robot_description/meshes/robot/`                    | Verzeichnis | Meshes (z.B. `body.stl`, `cover.stl`) für den robot.                                                 |
| `src/robot_ros/robot_description/meshes/robot_xl/`                 | Verzeichnis | Meshes für den robot XL.                                                                             |
| Pfad                                                               | Typ         | Zweck                                                                                                |
| ---                                                                | ---         | ---                                                                                                  |
| `src/robot_ros/robot_description/`                                 | Verzeichnis | ROS-Paket mit allen Beschreibungen beider Robotervarianten.                                          |
| `src/robot_ros/robot_description/urdf/`                            | Verzeichnis | Enthält die XACRO-Dateien, die Struktur, Gelenke und Materialien definieren.                         |
| `src/robot_ros/robot_description/urdf/robot.urdf.xacro`            | Datei       | Haupteinstiegspunkt für den robot. Bindet Basis, Sensoren und Zubehör ein.                           |
| `src/robot_ros/robot_description/urdf/robot_xl.urdf.xacro`         | Datei       | Pendant für den robot XL.                                                                            |
| `src/robot_ros/robot_description/urdf/robot_base.urdf.xacro`       | Datei       | Definiert die Basisgeometrie und lädt die Meshes des robot.                                          |
| `src/robot_ros/robot_description/urdf/robot_properties.urdf.xacro` | Datei       | Bündelt Parameter wie Mesh-Pfade, Abmessungen und Massen – zentrale Stelle für optische Anpassungen. |
| `src/robot_ros/robot_description/meshes/robot/`                    | Verzeichnis | Meshes (z.B. `body.stl`, `cover.stl`) für den robot.                                                 |
| `src/robot_ros/robot_description/meshes/robot_xl/`                 | Verzeichnis | Meshes für den robot XL.                                                                             |

## Geometrie anpassen

1. **Zielmodell auswählen**: Öffnen Sie `robot.urdf.xacro` oder `robot_xl.urdf.xacro` – beide binden modulare XACRO-Dateien ein.
1. **Zielmodell auswählen**: Öffnen Sie `robot.urdf.xacro` oder `robot_xl.urdf.xacro` – beide binden modulare XACRO-Dateien ein.
2. **Komponente lokalisieren**: Suchen Sie in der jeweiligen Datei (oder den inkludierten Modulen) nach dem `<link>` oder `<joint>`, den Sie verändern möchten. Nutzen Sie die `xacro:include`-Zeilen als Wegweiser.
3. **Position und Dimensionen ändern**:
   - Passen Sie in `<origin xyz="..." rpy="..."/>` die Lage an.
   - Ändern Sie in `<geometry>`-Blöcken die Größe von primitiven Körpern.
   - Aktualisieren Sie zugehörige Masseneigenschaften (`<inertial>`) in `robot_properties.urdf.xacro`, damit Simulationen konsistent bleiben.
   - Aktualisieren Sie zugehörige Masseneigenschaften (`<inertial>`) in `robot_properties.urdf.xacro`, damit Simulationen konsistent bleiben.
4. **Validieren**: Prüfen Sie Änderungen mit Tools wie `check_urdf` oder durch Laden in RViz/Gazebo, um falsche Rotationen oder Abstände früh zu erkennen.

## Erscheinungsbild anpassen

### Schlüssel-Properties in `robot_properties.urdf.xacro`
### Schlüssel-Properties in `robot_properties.urdf.xacro`

Die wichtigsten Pfad- und Größenvariablen sind hier hinterlegt. Nutzen Sie diese Properties statt hart codierter Werte in den URDFs.

```xml
<!-- Mesh-Pfade -->
<xacro:property name="robot_base_mesh_file" value="package://robot_description/meshes/robot_base.stl" />
<xacro:property name="robot_wheel_mesh_file" value="package://robot_description/meshes/wheel.stl" />
<xacro:property name="robot_front_caster_mesh_file" value="package://robot_description/meshes/caster_front.stl" />
<xacro:property name="robot_rear_caster_mesh_file" value="package://robot_description/meshes/caster_back.stl" />
<xacro:property name="robot_base_mesh_file" value="package://robot_description/meshes/robot_base.stl" />
<xacro:property name="robot_wheel_mesh_file" value="package://robot_description/meshes/wheel.stl" />
<xacro:property name="robot_front_caster_mesh_file" value="package://robot_description/meshes/caster_front.stl" />
<xacro:property name="robot_rear_caster_mesh_file" value="package://robot_description/meshes/caster_back.stl" />

<!-- Abmessungen & Massen -->
<xacro:property name="base_x_size" value="0.230" />
<xacro:property name="base_y_size" value="0.236" />
<xacro:property name="base_z_size" value="0.080" />
<xacro:property name="base_mass" value="1.5" />
<xacro:property name="wheel_radius" value="0.0425" />
<xacro:property name="wheel_width" value="0.02" />
<xacro:property name="wheel_mass" value="0.2" />
```

### Mesh-Dateien austauschen

1. **Modelle vorbereiten**: Konvertieren Sie eigene CAD-Dateien (z.B. STEP) nach STL/DAE.
2. **Dateien ablegen**: Kopieren Sie die neuen Meshes in das passende Verzeichnis unter `meshes/robot/` oder `meshes/robot_xl/`.
3. **Pfad aktualisieren**: Ändern Sie in `robot_properties.urdf.xacro` die passende Property, z.B.:
2. **Dateien ablegen**: Kopieren Sie die neuen Meshes in das passende Verzeichnis unter `meshes/robot/` oder `meshes/robot_xl/`.
3. **Pfad aktualisieren**: Ändern Sie in `robot_properties.urdf.xacro` die passende Property, z.B.:
   ```xml
   <xacro:property name="robot_base_mesh_file" value="package://robot_description/meshes/mein_gehaeuse.stl" />
   <xacro:property name="robot_base_mesh_file" value="package://robot_description/meshes/mein_gehaeuse.stl" />
   ```
4. **Orientierung prüfen**: Stellen Sie sicher, dass das Koordinatensystem Ihrer Mesh-Datei zu den in den XACROs erwarteten Achsen passt. Bei Bedarf können Sie im `<mesh>`-Tag eine `scale` setzen.

### Farben und Materialien

1. Öffnen Sie die zugehörige URDF-Datei (häufig `robot_base.urdf.xacro`).
1. Öffnen Sie die zugehörige URDF-Datei (häufig `robot_base.urdf.xacro`).
2. Suchen Sie nach `<material>`-Definitionen und passen Sie die RGBA-Werte an:
   ```xml
   <material name="custom_blue">
     <color rgba="0.1 0.35 0.8 1.0" />
   </material>
   ```
3. Weisen Sie das Material dem gewünschten `<visual>`-Block zu.

### Physikalische Eigenschaften

Wenn Sie Geometrie oder Masse signifikant ändern, passen Sie die zugehörigen Properties in `robot_properties.urdf.xacro` an (`base_mass`, `wheel_mass`, Radien usw.), damit Simulation und Navigation realistisch bleiben.
Wenn Sie Geometrie oder Masse signifikant ändern, passen Sie die zugehörigen Properties in `robot_properties.urdf.xacro` an (`base_mass`, `wheel_mass`, Radien usw.), damit Simulation und Navigation realistisch bleiben.

## Änderungen anwenden und testen

1. Bauen Sie das Paket neu:
   ```bash
   colcon build --packages-select robot_description
   colcon build --packages-select robot_description
   ```
2. Laden Sie das Modell erneut (z.B. RViz, Gazebo oder den entsprechenden Launch-File), um die Anpassungen zu überprüfen.
3. Prüfen Sie insbesondere Kollisionsobjekte, Sensorpositionen und resultierende TF-Frames auf Plausibilität.

Mit dieser konsolidierten Anleitung stehen alle relevanten Schritte an einem Ort. Für Detailfragen zu speziellen Sensoren oder Manipulator-Anbauten können zusätzliche Paketspezifika erforderlich sein.
