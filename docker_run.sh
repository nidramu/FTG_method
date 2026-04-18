#!/bin/bash
set -euo pipefail

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <image_name> <command...>"
    echo "Example: $0 barn-ftg:latest python3 run.py --world_idx 0"
    exit 1
fi

IMAGE_NAME="$1"
shift
NETWORK_MODE="${DOCKER_NETWORK_MODE:-host}"

docker run --rm -it \
    --network "${NETWORK_MODE}" \
    -e ROS_MASTER_URI="http://127.0.0.1:11311" \
    -e ROS_IP="127.0.0.1" \
    -e ROS_HOSTNAME="127.0.0.1" \
    -e GAZEBO_RANDOM_SEED=42 \
    -v "$(pwd):/jackal_ws/src/the-barn-challenge" \
    "${IMAGE_NAME}" \
    "$@"