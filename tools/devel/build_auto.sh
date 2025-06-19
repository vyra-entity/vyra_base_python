#!/bin/bash

cd ~/VYRA/vyra_base_python 
poetry build

python3 -m pip uninstall vyra_base -y
python3 -m pip install dist/vyra_base-0.1.5.tar.gz

cp dist/vyra_base-0.1.5-py3-none-any.whl ../vyra_module_template/wheels/
