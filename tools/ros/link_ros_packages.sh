#!/bin/bash

# Fehler sofort anzeigen
set -e

# Optional: ROS-Version setzen, falls nicht gesourced
# source /opt/ros/humble/setup.bash

echo "🔎 Suche Poetry-Venv..."
VENV_PATH=$(poetry env info --path)
PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
SITE_PACKAGES="$VENV_PATH/lib/python$PYTHON_VERSION/site-packages"

echo "📍 Virtuelle Umgebung: $VENV_PATH"
echo "📂 site-packages-Verzeichnis der Venv: $SITE_PACKAGES"

# ROS_SITE_PACKAGES dynamisch ermitteln
ROS_RCLPY_FILE=$(python3 -c "import rclpy; print(rclpy.__file__)")
ROS_SITE_PACKAGES=$(dirname "$(dirname "$ROS_RCLPY_FILE")")

echo "📂 ROS site-packages-Verzeichnis: $ROS_SITE_PACKAGES"

if [ ! -d "$ROS_SITE_PACKAGES" ]; then
    echo "❌ ROS site-packages nicht gefunden unter $ROS_SITE_PACKAGES"
    exit 1
fi

# Liste der ROS-Python-Module, die du brauchst
ROS_MODULES=("rclpy")

echo "🔗 Erstelle Symlinks..."

for module in "${ROS_MODULES[@]}"; do
    SRC="$ROS_SITE_PACKAGES/$module"
    DEST="$SITE_PACKAGES/$module"

    if [ -e "$DEST" ]; then
        echo "✅ $module ist bereits verlinkt oder vorhanden – überspringe"
    else
        ln -s "$SRC" "$DEST"
        echo "➕ Verlinkt: $module"
    fi
done

echo "✅ Alle Symlinks gesetzt."
