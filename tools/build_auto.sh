#!/bin/bash
# Automated build script for vyra_base
# - Increments build number in pyproject.toml
# - Builds wheel using build_wheel.sh
# - Installs wheel to system Python
# - Copies wheel to module directories

set -e  # Exit on error

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$PROJECT_ROOT"

echo "========================================"
echo "Vyra Base - Automated Build"
echo "========================================"
echo ""

# ========================================
# 1. Increment Build Number
# ========================================
echo "Step 1: Incrementing build number..."

PYPROJECT_FILE="$PROJECT_ROOT/pyproject.toml"

if [ ! -f "$PYPROJECT_FILE" ]; then
    echo "❌ Error: pyproject.toml not found"
    exit 1
fi

# Extract current version (e.g., "0.1.8+build.4")
CURRENT_VERSION=$(grep '^version = ' "$PYPROJECT_FILE" | sed 's/version = "\(.*\)"/\1/')

if [[ $CURRENT_VERSION =~ ^([0-9]+\.[0-9]+\.[0-9]+)\+build\.([0-9]+)$ ]]; then
    BASE_VERSION="${BASH_REMATCH[1]}"
    BUILD_NUMBER="${BASH_REMATCH[2]}"
    NEW_BUILD_NUMBER=$((BUILD_NUMBER + 1))
    NEW_VERSION="${BASE_VERSION}+build.${NEW_BUILD_NUMBER}"
else
    echo "❌ Error: Version format not recognized: $CURRENT_VERSION"
    echo "Expected format: X.Y.Z+build.N"
    exit 1
fi

# Update version in pyproject.toml
sed -i "s/^version = \".*\"/version = \"$NEW_VERSION\"/" "$PYPROJECT_FILE"

echo "  Version: $CURRENT_VERSION → $NEW_VERSION"
echo "  ✅ Build number incremented"
echo ""

# ========================================
# 2. Build Wheel
# ========================================
echo "Step 2: Building wheel..."

bash "$SCRIPT_DIR/build_wheel.sh"

# Find the latest wheel
WHEEL_FILE=$(ls dist/vyra_base-*-py3-none-any.whl | sort -V | tail -n 1)

if [ -z "$WHEEL_FILE" ]; then
    echo "❌ Error: No wheel file found"
    exit 1
fi

echo ""

# ========================================
# 3. Find Python Installation
# ========================================
echo "Step 3: Finding Python installation..."

# Try to find Python 3 automatically
PYTHON_PATH=""

# Check for common Python 3 installations
for py_cmd in python3.12 python3.11 python3.10 python3; do
    if command -v "$py_cmd" &> /dev/null; then
        PYTHON_PATH="$py_cmd"
        PYTHON_VERSION=$($py_cmd --version 2>&1 | cut -d' ' -f2)
        echo "  Found: $py_cmd (Python $PYTHON_VERSION)"
        break
    fi
done

if [ -z "$PYTHON_PATH" ]; then
    echo "❌ Error: No Python 3 installation found"
    exit 1
fi

echo "  Using: $PYTHON_PATH"
echo ""

# ========================================
# 4. Uninstall Old Version
# ========================================
echo "Step 4: Uninstalling old version..."

$PYTHON_PATH -m pip uninstall vyra_base -y --break-system-packages 2>/dev/null || true
echo "  ✅ Old version uninstalled"
echo ""

# ========================================
# 5. Install New Wheel
# ========================================
echo "Step 5: Installing new wheel..."

$PYTHON_PATH -m pip install "$WHEEL_FILE" --break-system-packages || {
    echo "❌ Installation failed"
    exit 1
}

echo "  ✅ Wheel installed: $(basename $WHEEL_FILE)"
echo ""

# ========================================
# 6. Copy to Module Directories
# ========================================
echo "Step 6: Copying wheel to module directories..."

# Copy to vyra_module_template
if [ -d "../vyra_module_template/wheels" ]; then
    cp "$WHEEL_FILE" ../vyra_module_template/wheels/
    echo "  ✅ Copied to vyra_module_template/wheels"
else
    echo "  ⚠️  Warning: ../vyra_module_template/wheels not found"
fi

# Copy to v2_modulemanager
MODULEMANAGER_DIR="../../VOS2_WORKSPACE/modules/v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/wheels"
if [ -d "$(dirname "$MODULEMANAGER_DIR")" ]; then
    # Clean old wheels
    rm -rf "$MODULEMANAGER_DIR"/vyra_base-*.whl 2>/dev/null || true
    
    # Copy new wheel
    mkdir -p "$MODULEMANAGER_DIR"
    cp "$WHEEL_FILE" "$MODULEMANAGER_DIR/"
    echo "  ✅ Copied to v2_modulemanager/wheels"
else
    echo "  ⚠️  Warning: v2_modulemanager directory not found"
fi

# Copy to vyra_base_image
VYRA_BASE_IMAGE_DIR="../../VOS2_WORKSPACE/vyra_base_image"
if [ -d "$VYRA_BASE_IMAGE_DIR" ]; then
    cp "$WHEEL_FILE" "$VYRA_BASE_IMAGE_DIR/"
    echo "  ✅ Copied to vyra_base_image"
else
    echo "  ⚠️  Warning: vyra_base_image directory not found"
fi

echo ""

# ========================================
# Summary
# ========================================
echo "========================================"
echo "✅ Build Complete!"
echo "========================================"
echo "Version:    $NEW_VERSION"
echo "Wheel:      $(basename $WHEEL_FILE)"
echo "Python:     $PYTHON_PATH ($PYTHON_VERSION)"
echo "========================================"