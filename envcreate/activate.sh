#!/bin/bash

# ============================================================
# Activate Python virtual environment
#
# Usage:
#   source ./envcrate/activate.sh
# ============================================================


# ------------------------------------------------------------
# Get directory paths
# ------------------------------------------------------------

# Directory containing this script: project/envcrate/
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Project directory: project/
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Virtual environment: project/venv/
VENV_DIR="$PROJECT_DIR/venv"


# ------------------------------------------------------------
# Check virtual environment
# ------------------------------------------------------------

if [ -d "$VENV_DIR" ]; then

    # Activate virtual environment
    source "$VENV_DIR/bin/activate"

    echo
    echo "========================================"
    echo " Installed Python packages"
    echo "========================================"
    echo

    python -m pip list

    echo
    echo "========================================"
    echo " Virtual environment activated"
    echo "========================================"
    echo

    echo "Python:"
    python --version

    echo
    echo "Python executable:"
    echo "    $(which python)"

    echo
    echo "Virtual environment:"
    echo "    $VENV_DIR"
    echo

else

    echo
    echo "Virtual environment 'venv' was not found."
    echo
    echo "Please create it first:"
    echo
    echo "    bash ./envcrate/mkvenv.sh"
    echo

    return 1 2>/dev/null || exit 1

fi