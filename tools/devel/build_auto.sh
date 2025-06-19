#!/bin/bash

cd ~/VOS2_WORKSPACE/varioboticos-base-python 
poetry build

python3 -m pip uninstall vos-base -y
python3 -m pip install dist/vos_base-0.1.5.tar.gz

cp dist/vos_base-0.1.5-py3-none-any.whl ../vos_module_template/wheels/
