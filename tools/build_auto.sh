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

echo "  ✅ Wheel installed: $(basename $WHEEL_FILE)"
echo ""

$PYTHON_PATH -m pip install "$WHEEL_FILE" --break-system-packages || {
    echo "❌ Installation failed"
    exit 1
}
echo "  ✅ Wheel installed: $(basename $WHEEL_FILE)"
echo ""

# ========================================
# 5b. Install Wheel in .venv (falls vorhanden)
# ========================================
VENV_DIR="$PROJECT_ROOT/.venv"
VENV_PYTHON="$VENV_DIR/bin/python"
if [ -d "$VENV_DIR" ] && [ -x "$VENV_PYTHON" ]; then
    echo "Step 5b: Installing wheel in .venv..."
    "$VENV_PYTHON" -m pip install "$WHEEL_FILE" || {
        echo "❌ Installation in .venv failed"
        exit 1
    }
    echo "  ✅ Wheel installed in .venv"
else
    echo "  ℹ️  No .venv found in vyra_base_python"
fi
echo ""

# ========================================
# 6. Copy to Module Directories
# ========================================
echo "Step 6: Copying wheel to module directories..."

# Copy to vyra_module_template
if [ -d "../vyra_module_template/wheels" ]; then
    # Clean old wheels
    rm -rf ../vyra_module_template/wheels/vyra_base-*.whl 2>/dev/null || true
    
    # Copy new wheel
    cp "$WHEEL_FILE" ../vyra_module_template/wheels/
    echo "  ✅ Copied to vyra_module_template/wheels"
else
    echo "  ⚠️  Warning: ../vyra_module_template/wheels not found"
fi

# Copy to all modules in VOS2_WORKSPACE/modules/
VOS2_MODULES_DIR="../../VOS2_WORKSPACE/modules"
if [ -d "$VOS2_MODULES_DIR" ]; then
    echo "  Searching for modules in $VOS2_MODULES_DIR..."
    MODULE_COUNT=0
    
    # Iterate through all directories in modules/
    for MODULE_DIR in "$VOS2_MODULES_DIR"/*/; do
        if [ -d "$MODULE_DIR" ]; then
            MODULE_NAME=$(basename "$MODULE_DIR")
            WHEELS_DIR="${MODULE_DIR}wheels"
            
            # Check if wheels directory exists
            if [ -d "$WHEELS_DIR" ]; then
                # Clean old wheels
                rm -rf "$WHEELS_DIR"/vyra_base-*.whl 2>/dev/null || true
                
                # Copy new wheel
                cp "$WHEEL_FILE" "$WHEELS_DIR/"
                echo "  ✅ Copied to $MODULE_NAME/wheels"
                MODULE_COUNT=$((MODULE_COUNT + 1))
            fi
        fi
    done
    
    if [ $MODULE_COUNT -eq 0 ]; then
        echo "  ⚠️  Warning: No modules with wheels directory found"
    else
        echo "  ✅ Copied to $MODULE_COUNT module(s)"
    fi
else
    echo "  ⚠️  Warning: VOS2_WORKSPACE/modules directory not found"
fi

# Copy to vyra_base_image
VYRA_BASE_IMAGE_DIR="../../VOS2_WORKSPACE/vyra_base_image"
if [ -d "$VYRA_BASE_IMAGE_DIR" ]; then
    # Clean old wheels
    rm -rf "$VYRA_BASE_IMAGE_DIR"/vyra_base-*.whl 2>/dev/null || true

    # Copy new wheel
    cp "$WHEEL_FILE" "$VYRA_BASE_IMAGE_DIR/"
    echo "  ✅ Copied to vyra_base_image"
else
    echo "  ⚠️  Warning: vyra_base_image directory not found"
fi

# Copy to test_daemon
TEST_DAEMON_DIR="../../VOS2_WORKSPACE/development/test_daemon"
if [ -d "$TEST_DAEMON_DIR" ]; then
    # Create wheels directory if not exists
    mkdir -p "$TEST_DAEMON_DIR/wheels"
    
    # Clean old wheels
    rm -rf "$TEST_DAEMON_DIR"/wheels/vyra_base-*.whl 2>/dev/null || true
    
    # Copy new wheel
    cp "$WHEEL_FILE" "$TEST_DAEMON_DIR/wheels/"
    echo "  ✅ Copied to test_daemon/wheels"
else
    echo "  ⚠️  Warning: test_daemon directory not found"
fi

echo ""

# ========================================
# 7. Update CHANGELOG.md
# ========================================
echo "Step 7: Updating CHANGELOG.md..."

CHANGELOG_FILE="$PROJECT_ROOT/CHANGELOG.md"

# Prompt for changelog entry
echo ""
echo "======================================="
echo "📝 Changelog Entry for v$NEW_VERSION"
echo "======================================="
echo "Enter changelog description (press Ctrl+D when done):"
echo ""

# Read multiline input
CHANGELOG_ENTRY=$(cat)

if [ -z "$CHANGELOG_ENTRY" ]; then
    echo "  ⚠️  Warning: No changelog entry provided"
else
    # Get current date
    CURRENT_DATE=$(date +%Y-%m-%d)
    
    # Create changelog entry with proper formatting
    NEW_ENTRY="\n\n## [$NEW_VERSION] - $CURRENT_DATE\n\n### Build\n\n$CHANGELOG_ENTRY"
    
    # Insert after [Unreleased] section using awk for robust multiline handling
    TMP_CHANGELOG=$(mktemp)
    awk -v entry="$NEW_ENTRY" 'BEGIN{inserted=0} {print $0} $0 ~ /^## \[Unreleased\]/ && !inserted {print entry; inserted=1}' "$CHANGELOG_FILE" > "$TMP_CHANGELOG"
    mv "$TMP_CHANGELOG" "$CHANGELOG_FILE"
    echo "  ✅ Changelog updated"
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