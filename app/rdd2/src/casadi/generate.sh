#!/bin/bash
poetry run -C ~/cognipilot/tools/src/cyecca python3 rdd2.py
poetry run -C ~/cognipilot/tools/src/cyecca python3 rdd2_loglinear.py
poetry run -C ~/cognipilot/tools/src/cyecca python3 bezier.py
