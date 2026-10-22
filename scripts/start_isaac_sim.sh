#!/usr/bin/env bash
set -e

if [[ $# -ne 2 || "$1" != "--scene" ]]; then
    echo "Usage: bash scripts/start_isaac_sim.sh --scene <filename.usd>" >&2
    exit 2
fi

scene_name="$2"

if [[ "$scene_name" == */* ]]; then
    echo "Pass only the scene filename, without a directory." >&2
    exit 2
fi

scene_path="/home/noel_614420090/reconview/scene/$scene_name"

if [[ ! -f "$scene_path" ]]; then
    echo "Scene file not found: $scene_path" >&2
    exit 2
fi

echo "Scene selected: $scene_path"

cd /home/noel_614420090/reconview

# 這些是 Fast DDS 的傳輸緩衝設定；32 MiB = 32 × 1024 × 1024 bytes = 33,554,432 bytes
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-DEFAULT?sockets_size=33554432}"
echo "Fast DDS transports: $FASTDDS_BUILTIN_TRANSPORTS"

# Load Isaac Sim's bundled ROS libraries instead of the caller's system ROS.
# Keep ROS_DOMAIN_ID and Fast DDS settings for communication and transport tests.
exec env \
    -u PYTHONPATH \
    -u LD_LIBRARY_PATH \
    -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    -u ROS_DISTRO \
    -u ROS_VERSION \
    -u ROS_PYTHON_VERSION \
    "$HOME/isaacsim/_build/linux-x86_64/release/isaac-sim.streaming.sh" \
    --livestream 2 \
    --/app/exts/devPaths/0="$HOME/PegasusSimulator/extensions/pegasus.simulator" \
    --enable pegasus.simulator \
    --enable isaacsim.ros2.bridge \
    --exec "/home/noel_614420090/reconview/scripts/initialize_pegasus_world.py --scene \"$scene_path\""
