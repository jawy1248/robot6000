#!/usr/bin/env bash
set -euo pipefail

BUILD=true
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../franka_ros2" && pwd)"

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-build|-n)
            BUILD=false
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--no-build|-n]"
            exit 1
            ;;
    esac
done

# Ensure repo exists and has docker-compose.yml
if [[ ! -f "$REPO_DIR/docker-compose.yml" ]]; then
  echo "Error: docker-compose.yml not found in $REPO_DIR."
  echo "Please ensure this script is in the parent directory of the franka_ros2 repository."
  exit 1
fi

# Step 1: Save user UID and GID into .env for correct permissions
echo "USER_UID=$(id -u)" > "$REPO_DIR/.env"
echo "USER_GID=$(id -g)" >> "$REPO_DIR/.env"
echo "Created .env in franka_ros2 with UID=$(id -u), GID=$(id -g)"

# Step 2: Build the Docker container (optional)
if $BUILD; then
    echo "Building Docker container..."
    (cd "$REPO_DIR" && docker compose build)
else
    echo "Skipping Docker build (--no-build option used)."
fi

# Step 3: Start the container (detached)
echo "Starting Docker container..."
(cd "$REPO_DIR" && docker compose up -d)

# Step 4: Automatically open a shell in the container
echo "Opening shell inside container..."
docker exec -it franka_ros2 bash -c '
echo "
============================================================
 Welcome to the franka_ros2 Docker container (ROS 2 Humble)
============================================================

To set up your workspace inside the container:

  1. Import dependencies:
       vcs import src < src/franka.repos --recursive --skip-existing

  2. Build the ROS 2 workspace:
       colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

  3. Source the workspace:
       source install/setup.bash

When done, you can exit with:
       exit

On the host, stop the container with:
       cd franka_ros2 && docker compose down -t 0 && cd ../
============================================================
"

# Start interactive shell
exec bash
'
