#!/usr/bin/env bash
GIT_BRANCH="$(git rev-parse --abbrev-ref HEAD)"
MOVE_TO="$1/$GIT_BRANCH/$2/${PWD##*/}/"

# Clean destination
rm -rf $MOVE_TO/*

# Run scan-build
make clean && scan-build-3.5 -o ./static_analysis make all

# Move over the results if any
[ "$(ls -A static_analysis)" ] && mv static_analysis/*/* $MOVE_TO || echo "No bugs"
rm -rf static_analysis/
