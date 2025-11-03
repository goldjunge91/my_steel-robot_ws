---
title: Excalidraw
summary: The famous hand-drawn style whiteboard
alpha: true
---

!!! note
    The `excalidraw` injects a markdown extension  (`shadcn.extensions.excalidraw`) at runtime. You don't have to worry about it. 

The plugin need to store you excalidraw drawings (`json` and `svg` files). By default it stores these files under `excalidraw/`. So you are likely to have this layout:

```plaintext
my-project/
├── ...
├── mkdocs.yml
├── docs/
│   ├── index.md
│   └── utils.md
├── excalidraw/
│   ├── drawing0.json
│   └── drawing0.svg
```

You can change this folder with the `directory` option (the path is relative to the root directory, i.e. the directory where `mkdocs.yml` lives).

```yaml
# mkdocs.yml

plugins:
  search:
  excalidraw:
    directory: assets/excalidraw
```

## Syntax

The path to the json file is relative to the directory provided to the plugin. The title is injected as build time through a `<title></title>` tag inside the output svg file.

```markdown
~{title}(filename.json)
```

Example with existing diagram:

~{test}(drawing0.json)
