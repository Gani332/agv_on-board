#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL_DIR="${MODEL_DIR:-$ROOT_DIR/models}"
MODEL_NAME="${MODEL_NAME:-ResNet18_64_cosplace.pth}"
MODEL_URL="${MODEL_URL:-https://github.com/gmberton/CosPlace/releases/download/v1.0/ResNet18_64_cosplace.pth}"
MODEL_PATH="$MODEL_DIR/$MODEL_NAME"

mkdir -p "$MODEL_DIR"

if [[ -f "$MODEL_PATH" ]]; then
  echo "Model already exists: $MODEL_PATH"
  exit 0
fi

tmp_path="$MODEL_PATH.tmp"
rm -f "$tmp_path"

echo "Downloading CosPlace ResNet18/64 model..."
curl -L --fail --show-error --output "$tmp_path" "$MODEL_URL"
mv "$tmp_path" "$MODEL_PATH"

echo "Model written to: $MODEL_PATH"
