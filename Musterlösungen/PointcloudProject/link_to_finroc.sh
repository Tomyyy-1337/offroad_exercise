#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${FINROC_HOME:-}" ]]; then
    echo "FINROC_HOME is not set, try source scripts/setenv in the finroc directory"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="$(basename "$SCRIPT_DIR")"
TARGET_DIR="$FINROC_HOME/sources/cpp/projects"

mkdir -p "$TARGET_DIR"

LINK_PATH="$TARGET_DIR/$PROJECT_NAME"

if [[ -e "$LINK_PATH" || -L "$LINK_PATH" ]]; then
    echo "Path already exists: $LINK_PATH"
    exit 1
fi

ln -s "$SCRIPT_DIR" "$LINK_PATH"

echo "Created:"
echo "  $LINK_PATH -> $SCRIPT_DIR"
