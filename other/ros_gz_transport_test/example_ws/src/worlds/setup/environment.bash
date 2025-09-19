#!/bin/bash

PACKAGE_SHARE="$(ros2 pkg prefix worlds)/share/worlds"
ASSETS_PATH="$PACKAGE_SHARE/assets"

export GAZEBO_MODEL_PATH="$ASSETS_PATH:$GAZEBO_MODEL_PATH"
