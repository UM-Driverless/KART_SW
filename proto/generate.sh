#!/usr/bin/env bash
# Generate protobuf bindings from kart_msgs.proto
# Run from repo root: bash proto/generate.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
PROTO_DIR="$SCRIPT_DIR"
PROTO_FILE="$PROTO_DIR/kart_msgs.proto"

# ── Python bindings (standard protobuf) ──────────────────────────────
PY_OUT="$REPO_ROOT/src/kb_dashboard/kb_dashboard/generated"
mkdir -p "$PY_OUT"
protoc --proto_path="$PROTO_DIR" --python_out="$PY_OUT" "$PROTO_FILE"
touch "$PY_OUT/__init__.py"
echo "✓ Python bindings → $PY_OUT/kart_msgs_pb2.py"

# ── nanopb C bindings (for ESP32 / kart_medulla) ────────────────────
C_OUT="$PROTO_DIR/generated_c"
mkdir -p "$C_OUT"

NANOPB_GEN="$(which nanopb_generator 2>/dev/null || echo "")"
if [ -z "$NANOPB_GEN" ]; then
    # Try platformio's Python env
    NANOPB_GEN="$HOME/.platformio/penv/bin/nanopb_generator"
fi

if [ -x "$NANOPB_GEN" ]; then
    "$NANOPB_GEN" -I "$PROTO_DIR" -D "$C_OUT" -f "$PROTO_DIR/nanopb/kart_msgs.options" "$PROTO_FILE"
    echo "✓ nanopb C bindings → $C_OUT/"
else
    echo "⚠ nanopb_generator not found — skipping C generation"
    echo "  Install: pip install nanopb"
fi
