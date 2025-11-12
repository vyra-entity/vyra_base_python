#!/bin/bash

poetry build
/usr/local/bin/python3.12 -m pip uninstall vyra_base --break-system-packages -y
/usr/local/bin/python3.12 -m pip install dist/vyra_base-0.1.5-py3-none-any.whl --break-system-packages
cp dist/vyra_base-0.1.5-py3-none-any.whl ../vyra_module_template/wheels
