#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$repo_dir/asr_summer_school${PYTHONPATH:+:$PYTHONPATH}"
"${PYTHON:-python3}" -m unittest discover -s "$repo_dir/asr_summer_school/test" -p test_mission_core.py -v
"${PYTHON:-python3}" -m unittest discover -s "$repo_dir/asr_summer_school/test" -p test_offline_regressions.py -v
"${PYTHON:-python3}" "$repo_dir/tools/scenario_return.py"
"${PYTHON:-python3}" -m unittest discover -s "$repo_dir/tools" -p test_diagnostic_metrics.py -v
