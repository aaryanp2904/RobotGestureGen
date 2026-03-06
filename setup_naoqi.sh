#!/bin/bash

# 1. Deactivate virtual environment if one is active
if [[ -n "$VIRTUAL_ENV" ]]; then
    echo "Deactivating current virtual environment: $VIRTUAL_ENV"
    # In some shells, 'deactivate' is a function; in scripts, we sometimes need to unset
    deactivate 2>/dev/null || unset VIRTUAL_ENV
else
    echo "No virtual environment active."
fi

# 2. Check if Python version is 2.7
PYTHON_VERSION=$(python -c 'import sys; print(".".join(map(str, sys.version_info[:2])))' 2>/dev/null)

if [ "$PYTHON_VERSION" != "2.7" ]; then
    echo "Error: Current Python is $PYTHON_VERSION. Please use Git Bash with Python 2.7 installed."
    # Exit or return depending on if sourced
    return 1 2>/dev/null || exit 1
fi

echo "Python 2.7 detected. Proceeding..."

# 3. Add Naoqi to PYTHONPATH
# We use the Windows-style path with forward slashes for Git Bash compatibility
NAOQI_PATH="C:/Python27/Lib/site-packages/pynaoqi/lib"

# Check if the directory actually exists before adding it
if [ -d "$NAOQI_PATH" ]; then
    export PYTHONPATH="$PYTHONPATH;$NAOQI_PATH"
    echo "PYTHONPATH updated. Current path includes: $NAOQI_PATH"
else
    echo "Warning: Naoqi directory not found at $NAOQI_PATH"
fi

echo "Setup complete. You can now run: python test.py"