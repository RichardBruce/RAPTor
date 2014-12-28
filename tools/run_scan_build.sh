#!/usr/bin/env bash
MOVE_TO="$1/${PWD##*/}/"

# Clean destination
rm -rf $MOVE_TO/*

# Run scan-build
make clean && scan-build -o ./static_analysis make all

# Move over the results if any
[ "$(ls -A static_analysis)" ] && mv static_analysis/*/* $MOVE_TO || echo "No bugs"
rm -rf static_analysis/
