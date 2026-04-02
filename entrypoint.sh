#!/bin/bash
# entrypoint.sh — The bootstrap script that runs BEFORE our Python code.
#
# WHY THIS EXISTS:
# ROS 2's executables and Python packages are NOT on the system PATH by default.
# Running `source /opt/ros/humble/setup.bash` adds them.
# The problem: Docker's ENV and RUN layers can't `source` into the final runtime
# environment reliably. A dedicated entrypoint script solves this cleanly.
#
# HOW IT WORKS:
# The last line `exec "$@"` is the magic.
# "$@" means "all the arguments passed to this script."
# When Docker runs: ENTRYPOINT ["./entrypoint.sh"] CMD ["python3", "sensor.py"]
#   1. entrypoint.sh runs, sources ROS 2
#   2. exec replaces this shell process with: python3 sensor.py
#   3. Our Python script now runs in an environment that KNOWS about ROS 2

set -e  # Exit immediately if any command fails (good practice for scripts)

# Source the ROS 2 environment — this is the entire reason this file exists
source /opt/ros/humble/setup.bash

echo "✅ ROS 2 Humble environment sourced successfully."
echo "▶️  Executing: $@"

# Replace this shell with the command passed to the container
# e.g., "python3 sensor.py" or "python3 controller.py"
exec "$@"
