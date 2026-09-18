#!/bin/bash

# ============================================================
# Python virtual environment setup script
#
# Directory structure:
#
#   project/
#   ├── venv/
#   └── envcrate/
#       └── mkvenv.sh
#
# Usage:
#   bash ./envcrate/mkvenv.sh
# ============================================================


# ------------------------------------------------------------
# Packages to install
# ------------------------------------------------------------

PACKAGES=(
    numpy
    scipy
    matplotlib
    pandas
    pymoo
)


# ------------------------------------------------------------
# Get directory paths
# ------------------------------------------------------------

# Directory containing this script: project/envcrate/
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Working/project directory: project/
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Virtual environment: project/venv/
VENV_DIR="$PROJECT_DIR/venv"


echo "========================================"
echo " Python virtual environment setup"
echo "========================================"
echo
echo "Project directory : $PROJECT_DIR"
echo "Virtual environment: $VENV_DIR"
echo


# ------------------------------------------------------------
# Check Python
# ------------------------------------------------------------

if ! command -v python3 >/dev/null 2>&1; then
    echo "ERROR: python3 was not found."
    echo "Please install Python first."
    exit 1
fi

echo "Python:"
python3 --version
echo


# ------------------------------------------------------------
# Create virtual environment if it does not exist
# ------------------------------------------------------------

if [ ! -d "$VENV_DIR" ]; then

    echo "Virtual environment 'venv' was not found."
    echo "Creating virtual environment..."
    echo

    python3 -m venv "$VENV_DIR"

    if [ $? -ne 0 ]; then
        echo
        echo "ERROR: Failed to create virtual environment."
        echo
        echo "On Ubuntu/WSL, python3-venv may be required:"
        echo "    sudo apt install python3-venv"
        exit 1
    fi

    echo
    echo "Virtual environment created:"
    echo "    $VENV_DIR"
    echo

else

    echo "Virtual environment already exists:"
    echo "    $VENV_DIR"
    echo

fi


# ------------------------------------------------------------
# Activate virtual environment
# ------------------------------------------------------------

echo "Activating virtual environment..."

source "$VENV_DIR/bin/activate"

echo
echo "Python in virtual environment:"
python --version
echo "    $(which python)"
echo


# ------------------------------------------------------------
# Upgrade pip
# ------------------------------------------------------------

echo "Checking pip..."
python -m pip install --upgrade pip
echo


# ------------------------------------------------------------
# Check/install packages
# ------------------------------------------------------------

echo "========================================"
echo " Package check"
echo "========================================"
echo

for package in "${PACKAGES[@]}"; do

    if python -m pip show "$package" >/dev/null 2>&1; then

        VERSION=$(python -m pip show "$package" \
            | awk -F': ' '/^Version:/ {print $2}')

        printf "%-20s installed  (version %s)\n" \
            "$package" "$VERSION"

    else

        printf "%-20s not found -> installing...\n" "$package"

        python -m pip install "$package"

        if [ $? -eq 0 ]; then

            VERSION=$(python -m pip show "$package" \
                | awk -F': ' '/^Version:/ {print $2}')

            printf "%-20s installed successfully (version %s)\n" \
                "$package" "$VERSION"

        else

            printf "%-20s ERROR: installation failed\n" "$package"

        fi

    fi

done


# ------------------------------------------------------------
# Finished
# ------------------------------------------------------------

echo
echo "========================================"
echo " Setup completed"
echo "========================================"
echo
echo "To activate this environment later, run:"
echo
echo "    source ./envcrate/activate.sh"
echo