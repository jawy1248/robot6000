#!/usr/bin/env bash
set -euo pipefail

BUILD=true
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../../franka_ros2" && pwd)"
CUSTOM_DIR="$SCRIPT_DIR"

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
  exit 1
fi

# Step 0: Replace the FRANKA Dockerfile with the personalized Dockerfile (and entrypoints)
# Remove existing files in franka_ros2
echo "Removing old Dockerfile from franka_ros2..."
rm -f "$REPO_DIR/Dockerfile"

# Copy custom versions into franka_ros2
echo "Copying custom Dockerfile..."
cp "$CUSTOM_DIR/Dockerfile" "$REPO_DIR/Dockerfile"

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

  2. Source install
        source ./install/set.bash

When done, you can exit with:
       exit

On the host, stop the container with:
       cd ../franka_ros2 && docker compose down -t 0 && cd ../robot6000
============================================================
"

# Start interactive shell
exec bash
'
