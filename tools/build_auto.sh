#!/bin/bash

poetry build

if [ $1 == "1" ]; then
    /usr/local/bin/python3.12 -m pip uninstall vyra_base --break-system-packages -y
    /usr/local/bin/python3.12 -m pip install dist/vyra_base-0.1.5-py3-none-any.whl --break-system-packages
    cp dist/vyra_base-0.1.5-py3-none-any.whl ../vyra_module_template/wheels
    cp dist/vyra_base-0.1.5-py3-none-any.whl ../../VOS2_WORKSPACE/modules/v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/wheels

elif [ $1 == "2" ]; then
    python3 -m pip uninstall vyra_base --break-system-packages
    python3 -m pip install dist/vyra_base-0.1.5-py3-none-any.whl --break-system-packages
    cp dist/vyra_base-0.1.5-py3-none-any.whl ../../VOS2_WORKSPACE/modules/v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/wheels
fi

