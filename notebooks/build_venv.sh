#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

python3 -m venv --prompt cerebri $SCRIPT_DIR/../.venv-cerebri
source $SCRIPT_DIR/../.venv-cerebri/bin/activate
python3 -m pip install -r requirements.txt
