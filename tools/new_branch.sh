#!/usr/bin/env bash

GIT_BRANCH="$(git rev-parse --abbrev-ref HEAD)"

MASTER_DIR="$1/master/"
BRANCH_DIR="$1/$GIT_BRANCH/"

if [ ! -d "$BRANCH_DIR" ]; then
    cp -rf $MASTER_DIR $BRANCH_DIR
fi