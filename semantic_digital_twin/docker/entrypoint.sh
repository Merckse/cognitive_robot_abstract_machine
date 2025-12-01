#!/usr/bin/env bash
set -e

source /opt/ros/overlay_ws/install/setup.bash

source /opt/ros/semantic_digital_twin-venv/bin/activate

exec "$@"