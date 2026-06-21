#!/usr/bin/env bash
set -euo pipefail

if [[ ! -d .venv ]]; then
  echo "Missing virtual environment. Run scripts/setup.sh first."
  exit 1
fi

source .venv/bin/activate
python src/main.py --config config/settings.example.json
