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

exec "$HOME/isaacsim/_build/linux-x86_64/release/isaac-sim.streaming.sh" \
    --livestream 2 \
    --/app/exts/devPaths/0="$HOME/PegasusSimulator/extensions/pegasus.simulator" \
    --enable pegasus.simulator \
    --enable isaacsim.ros2.bridge \
    --exec "/home/noel_614420090/reconview/scripts/initialize_pegasus_world.py --scene \"$scene_path\""
