#!/bin/bash

poetry build

# Find wheel in the folder with the highest version number
WHEEL_FILE=$(ls dist/vyra_base-0.1.*-py3-none-any.whl | sort -V | tail -n 1)

if [ $1 == "1" ]; then
    PYTHON_PATH="/usr/local/bin/python3.12"
elif [ $1 == "2" ]; then
    PYTHON_PATH="python3"
else
    echo "Usage: $0 [1|2]"
    echo "  1: Use Python 3.12 at /usr/local/bin/python3.12"
    echo "  2: Use system Python 3"
    echo " Not selected: Use system Python 3"
    PYTHON_PATH="python3"
fi

# Uninstall old version
$PYTHON_PATH -m pip uninstall vyra_base -y --break-system-packages

echo "Installing wheel: $WHEEL_FILE using $PYTHON_PATH"
$PYTHON_PATH -m pip install "$WHEEL_FILE" --break-system-packages || true

cp "$WHEEL_FILE" ../vyra_module_template/wheels

# Cleaning
rm -rf ../../VOS2_WORKSPACE/modules/v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/wheels/vyra_base-*.whl

# Adding new wheel to modulemanager
cp "$WHEEL_FILE" ../../VOS2_WORKSPACE/modules/v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/wheels

# Adding new wheel to vyra_base_image
cp "$WHEEL_FILE" ../../VOS2_WORKSPACE/vyra_base_image/