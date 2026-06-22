#!/usr/bin/env bash
set -euo pipefail

# Prepare local Python virtual environment and dependencies.
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

# Install Tailscale if available on PATH; otherwise install it first.
if ! command -v tailscale >/dev/null 2>&1; then
	echo "Tailscale not found. Installing..."
	curl -fsSL https://tailscale.com/install.sh | sh
fi

echo "Setup complete. Activate environment with: source .venv/bin/activate"
echo "If this is your first time with Tailscale, run: sudo tailscale up"
