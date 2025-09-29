# Anleitung zur Anpassung von Geometrie und Aussehen des Roboters

Diese Anleitung kombiniert die bisherigen Hinweise aus `ROBOTER_ANPASSEN.md` und `ROBOTER_AUSSEHEN_ANPASSEN.md`. Sie beschreibt, welche Dateien im Workspace für Anpassungen relevant sind und wie Sie bei Änderungen der Robotermodelle (ROSbot und ROSbot XL) vorgehen.

## Relevante Ressourcen

| Pfad | Typ | Zweck |
| --- | --- | --- |
| `src/rosbot_ros/rosbot_description/` | Verzeichnis | ROS-Paket mit allen Beschreibungen beider Robotervarianten. |
| `src/rosbot_ros/rosbot_description/urdf/` | Verzeichnis | Enthält die XACRO-Dateien, die Struktur, Gelenke und Materialien definieren. |
| `src/rosbot_ros/rosbot_description/urdf/rosbot.urdf.xacro` | Datei | Haupteinstiegspunkt für den ROSbot. Bindet Basis, Sensoren und Zubehör ein. |
| `src/rosbot_ros/rosbot_description/urdf/rosbot_xl.urdf.xacro` | Datei | Pendant für den ROSbot XL. |
| `src/rosbot_ros/rosbot_description/urdf/rosbot_base.urdf.xacro` | Datei | Definiert die Basisgeometrie und lädt die Meshes des ROSbot. |
| `src/rosbot_ros/rosbot_description/urdf/rosbot_properties.urdf.xacro` | Datei | Bündelt Parameter wie Mesh-Pfade, Abmessungen und Massen – zentrale Stelle für optische Anpassungen. |
| `src/rosbot_ros/rosbot_description/meshes/rosbot/` | Verzeichnis | Meshes (z.B. `body.stl`, `cover.stl`) für den ROSbot. |
| `src/rosbot_ros/rosbot_description/meshes/rosbot_xl/` | Verzeichnis | Meshes für den ROSbot XL. |

## Geometrie anpassen

1. **Zielmodell auswählen**: Öffnen Sie `rosbot.urdf.xacro` oder `rosbot_xl.urdf.xacro` – beide binden modulare XACRO-Dateien ein.
2. **Komponente lokalisieren**: Suchen Sie in der jeweiligen Datei (oder den inkludierten Modulen) nach dem `<link>` oder `<joint>`, den Sie verändern möchten. Nutzen Sie die `xacro:include`-Zeilen als Wegweiser.
3. **Position und Dimensionen ändern**:
   - Passen Sie in `<origin xyz="..." rpy="..."/>` die Lage an.
   - Ändern Sie in `<geometry>`-Blöcken die Größe von primitiven Körpern.
   - Aktualisieren Sie zugehörige Masseneigenschaften (`<inertial>`) in `rosbot_properties.urdf.xacro`, damit Simulationen konsistent bleiben.
4. **Validieren**: Prüfen Sie Änderungen mit Tools wie `check_urdf` oder durch Laden in RViz/Gazebo, um falsche Rotationen oder Abstände früh zu erkennen.

## Erscheinungsbild anpassen

### Schlüssel-Properties in `rosbot_properties.urdf.xacro`

Die wichtigsten Pfad- und Größenvariablen sind hier hinterlegt. Nutzen Sie diese Properties statt hart codierter Werte in den URDFs.

```xml
<!-- Mesh-Pfade -->
<xacro:property name="rosbot_base_mesh_file" value="package://rosbot_description/meshes/rosbot_base.stl" />
<xacro:property name="rosbot_wheel_mesh_file" value="package://rosbot_description/meshes/wheel.stl" />
<xacro:property name="rosbot_front_caster_mesh_file" value="package://rosbot_description/meshes/caster_front.stl" />
<xacro:property name="rosbot_rear_caster_mesh_file" value="package://rosbot_description/meshes/caster_back.stl" />

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
2. **Dateien ablegen**: Kopieren Sie die neuen Meshes in das passende Verzeichnis unter `meshes/rosbot/` oder `meshes/rosbot_xl/`.
3. **Pfad aktualisieren**: Ändern Sie in `rosbot_properties.urdf.xacro` die passende Property, z.B.:
   ```xml
   <xacro:property name="rosbot_base_mesh_file" value="package://rosbot_description/meshes/mein_gehaeuse.stl" />
   ```
4. **Orientierung prüfen**: Stellen Sie sicher, dass das Koordinatensystem Ihrer Mesh-Datei zu den in den XACROs erwarteten Achsen passt. Bei Bedarf können Sie im `<mesh>`-Tag eine `scale` setzen.

### Farben und Materialien

1. Öffnen Sie die zugehörige URDF-Datei (häufig `rosbot_base.urdf.xacro`).
2. Suchen Sie nach `<material>`-Definitionen und passen Sie die RGBA-Werte an:
   ```xml
   <material name="custom_blue">
     <color rgba="0.1 0.35 0.8 1.0" />
   </material>
   ```
3. Weisen Sie das Material dem gewünschten `<visual>`-Block zu.

### Physikalische Eigenschaften

Wenn Sie Geometrie oder Masse signifikant ändern, passen Sie die zugehörigen Properties in `rosbot_properties.urdf.xacro` an (`base_mass`, `wheel_mass`, Radien usw.), damit Simulation und Navigation realistisch bleiben.

## Änderungen anwenden und testen

1. Bauen Sie das Paket neu:
   ```bash
   colcon build --packages-select rosbot_description
   ```
2. Laden Sie das Modell erneut (z.B. RViz, Gazebo oder den entsprechenden Launch-File), um die Anpassungen zu überprüfen.
3. Prüfen Sie insbesondere Kollisionsobjekte, Sensorpositionen und resultierende TF-Frames auf Plausibilität.

Mit dieser konsolidierten Anleitung stehen alle relevanten Schritte an einem Ort. Für Detailfragen zu speziellen Sensoren oder Manipulator-Anbauten können zusätzliche Paketspezifika erforderlich sein.
