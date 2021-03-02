# Move Base Flex Docs

These docs are built with [MkDocs](https://mkdocs.org)

## Build locally

```
sudo apt install python3-pip
pip3 install mkdocs mkdocs-material
```

*Note: you may need to adapt your path `export PATH=$PATH:/home/<USER>/.local/bin` to access mkdocs binary*

### Build Static Files

```
mkdocs build --strict
```

### Live reloading server on :8000

```
mkdocs serve --strict
```