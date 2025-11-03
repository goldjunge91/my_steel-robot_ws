# Shadcn Theme Extensions Guide

Dieses Dokument beschreibt alle verfügbaren Extensions des Shadcn Themes und ihre Verwendung in unserer MkDocs-Dokumentation.

---

## 1. Code-Highlighting mit Zeilennummern (Fenced Code)

### Konfiguration

```yaml
# mkdocs.yml
markdown_extensions:
  - fenced_code          # Shadcn-kompatible Fenced Code Blocks
  - codehilite           # Syntax-Highlighting via Pygments
  - attr_list            # Notwendig für Attribute { .python ... }
```

### Syntax

Die Shadcn-Variante nutzt **Attribute nach der Sprache** in geschweiften Klammern:

```
``` { .python linenos="table" hl_lines="4 5" }
def fibonacci(n):
    # ...
```

```

#### Einfacher Code-Block

```python
def fibonacci(n):
    a, b = 0, 1
    for _ in range(n):
        yield a
        a, b = b, a + b

for num in fibonacci(10):
    print(num)
```

#### Mit Zeilennummern (table-Style)

``` { .python linenos="table" }
def fibonacci(n):
    a, b = 0, 1
    for _ in range(n):
        yield a
        a, b = b, a + b

for num in fibonacci(10):
    print(num)
```

#### Mit Zeilennummern + Highlighting

``` { .python linenos="table" hl_lines="4 5" }
def fibonacci(n):
    a, b = 0, 1
    for _ in range(n):
        yield a
        a, b = b, a + b

for num in fibonacci(10):
    print(num)
```

#### Mit Zeilennummern (inline) + Line-Anchors

``` { .python linenos="inline" hl_lines="4 5" anchorlinenos="true" lineanchors="fibo" }
def fibonacci(n):
    a, b = 0, 1
    for _ in range(n):
        yield a
        a, b = b, a + b

for num in fibonacci(10):
    print(num)
```

**Verfügbare Optionen** (Pygments HTML Formatter):
- `linenos="table"`: Zeilennummern als separate Tabelle (copy-friendly)
- `linenos="inline"`: Zeilennummern inline im Code
- `hl_lines="4 5"`: Highlightet Zeilen 4 und 5
- `anchorlinenos="true"`: Macht Zeilennummern klickbar
- `lineanchors="name"`: Prefix für Line-Anchors (z.B. `#fibo-4`)

---

## 2. Codexec - Code ausführen

**Unterstützte Sprachen**: `r`, `c`, `cpp`, `csharp`, `java`, `python`, `javascript`, `typescript`, `scala`, `dart`, `ruby`, `golang`, `php`, `swift`, `rust`

!!! warning "Wichtig"
    Bash/Shell wird **nicht** unterstützt!

### Konfiguration

```yaml
# mkdocs.yml
markdown_extensions:
  - codehilite                # Notwendig für Syntax-Highlighting
  - shadcn.extensions.codexec
```

### Syntax

Die Codexec-Extension nutzt spezielle Delimiter:

```text
/// codexec
    :::python
    print("Hello World!")
///
```

### Beispiel Python

/// codexec
    :::python
    print("Hello from Codexec!")
    for i in range(5):
        print(f"Count: {i}")
///

### Beispiel JavaScript

/// codexec
    :::javascript
    console.log("Hello from JavaScript!");
    const numbers = [1, 2, 3, 4, 5];
    numbers.forEach(n => console.log(n * 2));
///

### Fehlerbehandlung

Syntax-Fehler werden hervorgehoben. Exceptions werden normal ausgegeben.

---

## 3. ECharts - Interaktive Diagramme

### Konfiguration

```yaml
# mkdocs.yml
markdown_extensions:
  - shadcn.extensions.echarts.alpha
```

### Syntax

```markdown
/// echarts
{
  xAxis: {
    type: 'category',
    data: ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun']
  },
  yAxis: {
    type: 'value'
  },
  series: [{
    data: [150, 230, 224, 218, 135, 147, 260],
    type: 'line'
  }]
}
///
```

### Beispiele

#### Liniendiagramm

/// echarts
{
  xAxis: {
    type: 'category',
    data: ['Jan', 'Feb', 'Mär', 'Apr', 'Mai', 'Jun']
  },
  yAxis: {
    type: 'value'
  },
  series: [{
    name: 'Umsatz',
    data: [820, 932, 901, 934, 1290, 1330],
    type: 'line',
    smooth: true
  }]
}
///

#### Balkendiagramm

/// echarts
{
  xAxis: {
    type: 'category',
    data: ['Klasse 0', 'Klasse 1', 'Klasse 2', 'Klasse 3']
  },
  yAxis: {
    type: 'value'
  },
  series: [{
    data: [5, 20, 36, 10],
    type: 'bar'
  }]
}
///

#### Kreisdiagramm

/// echarts
{
  series: [{
    type: 'pie',
    data: [
      { value: 335, name: 'Search Engine' },
      { value: 310, name: 'Direct' },
      { value: 234, name: 'Email' },
      { value: 135, name: 'Ads' }
    ]
  }]
}
///

### Optionen

```markdown
/// echarts
    renderer: "canvas"
    attrs:
        style: "width:100%;height:60vh;"

{
  /* ECharts config */
}
///
```

**Verfügbare Optionen**:
- `renderer`: `"svg"` (default) oder `"canvas"`
- `attrs.class`: CSS-Klasse (default: `"echarts"`)
- `attrs.style`: CSS-Stil (default: `"width:100%;height:500px;"`)

---

## 4. Excalidraw - Hand-gezeichnete Diagramme

### Konfiguration

```yaml
# mkdocs.yml
plugins:
  - excalidraw:
      directory: docs/doc/excalidraw
```

!!! note "Automatisch"
    Excalidraw injiziert automatisch `shadcn.extensions.excalidraw` zur Laufzeit.

### Projektstruktur

```
my_steel-robot_ws/
├── mkdocs.yml
├── docs/
│   └── doc/
│       ├── index.md
│       └── excalidraw/
│           ├── architecture.json
│           └── architecture.svg
```

### Syntax

```text
~{Titel des Diagramms}(pfad/zur/datei.json)
```

#### Beispiel

```text
~{ROS2 Architektur}(excalidraw/architecture.json)
```

**Features**:
- Integrierter Editor im `mkdocs serve` Entwicklungsserver
- Automatische SVG-Generierung beim Build
- Titel wird als `<title>` Tag in SVG eingefügt

---

## 5. Admonitions - Hinweisboxen

### Konfiguration

```yaml
# mkdocs.yml
markdown_extensions:
  - admonition
```

### Syntax

```text
!!! info "Information:"
    Etwas **Neues** kommt zu `mkdocs-shadcn`

!!! note "Hinweis:"
    Wir bemerken, dass `x=2`

!!! warning "Warnung:"
    Es besteht ein *Risiko* bei `x/0`

!!! danger "Gefahr:"
    Schau nicht in `node_modules` **bitte**!
```

### Beispiele

!!! info "Information"
    Dieses Projekt nutzt ROS2 Humble mit C++17 und Python 3.8+

!!! note "Hinweis"
    Die Firmware läuft auf Raspberry Pi Pico (RP2040) mit 300ms Watchdog

!!! warning "Warnung"
    Vor dem Flashen der Firmware sicherstellen, dass der Pico im BOOTSEL-Modus ist

!!! danger "Gefahr"
    Nie die Motor-Versorgungsspannung mit der Pico-Versorgung kurzschließen!

### Code in Admonitions

!!! note "Python Beispiel"
    Mit `codehilite` kann Code in Admonitions verwendet werden:

    ``` { .python linenos="table" }
    def calculate_motor_speed(rpm):
        # Konvertierung RPM zu m/s
        return rpm * 0.1
    
    speed = calculate_motor_speed(100)
    print(f"Speed: {speed} m/s")
    ```

---

## Zusammenfassung der Konfiguration

Hier ist die **vollständige mkdocs.yml** Konfiguration für alle Shadcn Extensions:

```yaml
plugins:
  - search
  - excalidraw:
      directory: docs/doc/excalidraw
  - mermaid2:
      version: 11.5.0

markdown_extensions:
  # Basis-Extensions
  - admonition
  - footnotes
  - extra
  - attr_list            # WICHTIG für { .python ... } Syntax
  - md_in_html
  - toc:
      permalink: true
  
  # Code-Highlighting (Shadcn-kompatibel)
  - fenced_code          # Shadcn Fenced Code Blocks
  - codehilite           # Pygments Syntax-Highlighting
  
  # Shadcn Extensions
  - shadcn.extensions.codexec
  - shadcn.extensions.echarts.alpha
  - shadcn.extensions.iconify
  
  # PyMdown Extensions
  - pymdownx.blocks.details
  - pymdownx.blocks.tab
  - pymdownx.tabbed:
      alternate_style: true
  - pymdownx.superfences:
      custom_fences:
        - name: mermaid
          class: mermaid
          format: !!python/name:mermaid2.fence_mermaid
  - pymdownx.arithmatex:
      generic: true
```

---

## Best Practices

### 1. Code-Blöcke

- **Für Dokumentation**: `fenced_code` mit `{ .language linenos="table" hl_lines="..." }`
- **Für interaktive Demos**: `codexec` für Python/JavaScript (nicht für Bash!)
- **Für System-Commands**: Normale Code-Blöcke ohne Ausführung

### 2. Diagramme

- **Für Datenvisualisierung**: ECharts (interaktive Charts)
- **Für Architektur-Diagramme**: Mermaid oder Excalidraw
- **Für Hand-gezeichnete Skizzen**: Excalidraw mit Live-Editor

### 3. Hinweise

- **`info`**: Allgemeine Informationen
- **`note`**: Wichtige Hinweise
- **`warning`**: Warnungen vor Problemen
- **`danger`**: Kritische Sicherheitshinweise

### 4. Wichtige Unterschiede zu PyMdown

⚠️ **Shadcn nutzt NICHT `pymdownx.highlight`!**

| Feature | Shadcn (korrekt) | PyMdown (falsch) |
|---------|------------------|------------------|
| Extensions | `fenced_code` + `codehilite` | `pymdownx.highlight` |
| Syntax | `` ``` { .python linenos="table" } `` | `` ```python linenums="1" `` |
| Attribute | Nach Sprache in `{ }` | Direkt nach Sprache |
| Notwendig | `attr_list` Extension | - |

---

## Weitere Ressourcen

- [Shadcn Theme Dokumentation](https://asiffer.github.io/mkdocs-shadcn/)
- [PyMdown Extensions](https://facelessuser.github.io/pymdown-extensions/)
- [ECharts API](https://echarts.apache.org/en/option.html)
- [Excalidraw](https://excalidraw.com/)
