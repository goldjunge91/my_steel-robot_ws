# mkdocs-shadcn Extensions Documentation

This directory contains comprehensive documentation for all mkdocs-shadcn extensions and plugins.

## Available Extensions

### Built-in Extensions
- [Admonitions](admonitions.md) - Call-out boxes for notes, warnings, tips, etc.
- [Fenced Code](fenced_code.md) - Enhanced code blocks with syntax highlighting
- [Attribute Lists](attribute_lists.md) - Add HTML attributes to markdown elements

### shadcn Custom Extensions
- [Codexec](codexec.md) - Execute and display code output inline
- [Apache ECharts](echarts.md) - Interactive charts and data visualizations
- [Iconify](iconify.md) - Icon integration system

### PyMdown Extensions
- [Arithmatex](arithmatex.md) - Mathematical expressions with MathJax/KaTeX
- [Progressbar](progressbar.md) - Visual progress indicators
- [Tabbed](pymdownx_tabbed.md) - Tabbed content sections

### Plugins
- [Excalidraw](excalidraw.md) - Hand-drawn style diagrams and whiteboards

## Quick Setup

Add extensions to your `mkdocs.yml`:

```yaml
markdown_extensions:
  - admonition
  - fenced_code
  - attr_list
  - shadcn.extensions.codexec
  - shadcn.extensions.echarts.alpha
  - shadcn.extensions.iconify
  - pymdownx.arithmatex:
      generic: true
  - pymdownx.blocks.details
  - pymdownx.blocks.tab
  - pymdownx.blocks.caption
  - pymdownx.progressbar
  - pymdownx.tabbed:
      alternate_style: true

plugins:
  - excalidraw
```

## Theme Configuration

```yaml
theme:
  name: shadcn
  icon: heroicons:rocket-launch
  show_title: true
  pygments_style:
    light: shadcn-light
    dark: github-dark
```