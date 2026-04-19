#!/bin/bash

if [ $# -eq 0 ]; then
    echo "No arguments provided. Please provide a command to run."
    exit 1
fi

mkdir -p "$USER_WS/persistence/$(id -u)/ccache"
mkdir -p "$USER_WS/persistence/$(id -u)/install"
mkdir -p "$USER_WS/persistence/$(id -u)/build"

if [ ! -d "$HOME/.ccache" ]; then
    ln -s "$USER_WS/persistence/$(id -u)/ccache"  "$HOME/.ccache"
    ln -s "$USER_WS/persistence/$(id -u)/install" "$USER_WS/install"
    ln -s "$USER_WS/persistence/$(id -u)/build"   "$USER_WS/build"
fi

# if bash_history is not a symlink or if it does not exist, remove it and create a new symlink
if [ ! -h "$HOME/.bash_history" ]; then
    if [ -f "$HOME/.bash_history" ]; then
        rm "$HOME/.bash_history"
    fi
    touch "$USER_WS/persistence/$(id -u)/.bash_history"
    ln -s "$USER_WS/persistence/$(id -u)/.bash_history" "$HOME/.bash_history"
fi

. /opt/ros/humble/setup.bash

exec "$@"
