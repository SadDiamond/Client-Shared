#!/bin/bash
# Regenerates ui_*.py from a .ui file edited in Qt Creator.
# Qt Creator's Designer writes fully-qualified enums (Qt::Orientation::Horizontal)
# that pyuic5 doesn't translate correctly, producing a Python syntax error.
# ponytail: sed normalization, swap for `pyuic6`/newer pyuic if this repo ever moves off PyQt5.
set -e
for ui_file in "$@"; do
    py_file="ui_$(basename "$ui_file" .ui | tr '[:upper:]' '[:lower:]').py"
    sed -E 's/Qt::([A-Za-z]+)::([A-Za-z]+)/Qt::\2/g' "$ui_file" > "$ui_file.tmp"
    mv "$ui_file.tmp" "$ui_file"
    pyuic5 "$ui_file" -o "$py_file"
    echo "built $py_file"
done
