#!/bin/bash

poetry build
pip uninstall vos-base --break-system-packages -y
pip install dist/vos_base-0.1.5-py3-none-any.whl --break-system-packages
cp dist/vos_base-0.1.5-py3-none-any.whl ../vos_module_template/wheels
