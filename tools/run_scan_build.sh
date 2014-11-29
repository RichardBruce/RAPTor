#!/usr/bin/env bash
MOVE_TO="$1/${PWD##*/}/"
make clean && scan-build -o ./static_analysis make all && mv static_analysis/*/* $MOVE_TO && rm -rf static_analysis/
