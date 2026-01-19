#!/bin/bash
# Build wheel for vyra_base package
# This script builds a Python wheel using Poetry

set -e  # Exit on error

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$PROJECT_ROOT"

echo "========================================"
echo "Building vyra_base wheel"
echo "========================================"

# Build wheel with Poetry
poetry build

# Find the latest wheel
WHEEL_FILE=$(ls dist/vyra_base-*-py3-none-any.whl | sort -V | tail -n 1)

if [ -z "$WHEEL_FILE" ]; then
    echo "❌ Error: No wheel file found in dist/"
    exit 1
fi

echo "✅ Wheel built successfully: $WHEEL_FILE"
echo ""
echo "To install the wheel, run:"
echo "  python3 -m pip install $WHEEL_FILE"
