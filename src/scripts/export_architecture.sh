#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INPUT_FILE="$ROOT_DIR/ARCHITECTURE.md"
OUTPUT_DIR="$ROOT_DIR/doc/diagrams"
FORMAT="svg"
SCALE="2"
BASENAME="mas_architecture"

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Export the first Mermaid code block from ARCHITECTURE.md to SVG or PNG.

Options:
  -i, --input <file>       Input markdown file (default: ARCHITECTURE.md)
  -o, --output-dir <dir>   Output directory (default: doc/diagrams)
  -f, --format <svg|png>   Output format (default: svg)
  -s, --scale <number>     Render scale (default: 2)
  -n, --name <basename>    Output file basename (default: mas_architecture)
  -h, --help               Show this help

Examples:
  $(basename "$0")
  $(basename "$0") -f png -s 3
  $(basename "$0") -i ./ARCHITECTURE.md -o ./doc/diagrams -n architecture_detailed
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -i|--input)
            INPUT_FILE="$2"
            shift 2
            ;;
        -o|--output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -f|--format)
            FORMAT="$2"
            shift 2
            ;;
        -s|--scale)
            SCALE="$2"
            shift 2
            ;;
        -n|--name)
            BASENAME="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage
            exit 1
            ;;
    esac
done

if [[ ! -f "$INPUT_FILE" ]]; then
    echo "Input file not found: $INPUT_FILE" >&2
    exit 1
fi

if [[ "$FORMAT" != "svg" && "$FORMAT" != "png" ]]; then
    echo "Invalid format: $FORMAT (expected 'svg' or 'png')" >&2
    exit 1
fi

mkdir -p "$OUTPUT_DIR"

TMP_MMD="$(mktemp)"
cleanup() {
    rm -f "$TMP_MMD"
}
trap cleanup EXIT

# Extract first mermaid fenced block from markdown.
awk '
BEGIN { in_block = 0; found = 0 }
/^```mermaid[[:space:]]*$/ {
    if (found == 0) {
        in_block = 1
        found = 1
        next
    }
}
in_block && /^```[[:space:]]*$/ {
    in_block = 0
    exit
}
in_block { print }
' "$INPUT_FILE" > "$TMP_MMD"

if [[ ! -s "$TMP_MMD" ]]; then
    echo "No Mermaid block found in: $INPUT_FILE" >&2
    exit 1
fi

OUTPUT_FILE="$OUTPUT_DIR/${BASENAME}.${FORMAT}"

if command -v mmdc >/dev/null 2>&1; then
    mmdc -i "$TMP_MMD" -o "$OUTPUT_FILE" -s "$SCALE"
elif command -v npx >/dev/null 2>&1; then
    npx -y @mermaid-js/mermaid-cli@latest -i "$TMP_MMD" -o "$OUTPUT_FILE" -s "$SCALE"
else
    cat >&2 <<EOF
Neither 'mmdc' nor 'npx' is available.
Install one of the following:
  npm i -g @mermaid-js/mermaid-cli
EOF
    exit 1
fi

echo "Exported: $OUTPUT_FILE"
