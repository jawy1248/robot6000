# Robot6000 – Franka ROS 2 Docker Launcher

This folder contains a helper script to start and access the `franka_ros2` ROS 2 Humble Docker container.

## 📂 Folder Layout

```
parent/
├── robot6000/
│   └── run_franka.sh     # This script
└── franka_ros2/          # Cloned Franka ROS 2 Humble repository
    ├── docker-compose.yml
    └── ...
```

> **Important:** The `franka_ros2` folder must be a sibling of the `robot6000` folder for the script to work without changes.

---

## 🛠 Prerequisites

- **Docker** and **Docker Compose** installed on your system.
- The `franka_ros2` repository cloned to the correct location:
  ```bash
  git clone -b humble https://github.com/frankarobotics/franka_ros2.git ../franka_ros2
  ```

---

## ▶ Running the Script

From inside the `robot6000` folder:

```bash
chmod +x run_franka.sh    # First time only, makes the script executable
./run_franka.sh           # Runs with container build
```

### Optional: Skip the Docker build
If you know the container image hasn’t changed:
```bash
./run_franka.sh --no-build
# or
./run_franka.sh -n
```

---

## 🐳 What the Script Does

1. Sets up `.env` in `franka_ros2` with your current UID/GID (avoids permission issues).
2. **Optionally builds** the Docker image (`docker compose build`).
3. Starts the `franka_ros2` container in the background.
4. Opens an **interactive bash shell inside the container**.
5. Displays instructions for importing dependencies, building the ROS 2 workspace, and sourcing the environment.

---

## 💻 Inside the Container

Once inside, follow these steps:

```bash
# 1. Import dependencies
vcs import src < src/franka.repos --recursive --skip-existing

# 2. Build the workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. Source the workspace
source install/setup.bash
```

---

## ⏹ Stopping the Container

From the **host machine** (outside the container):

```bash
cd ../franka_ros2 && docker compose down -t 0 && cd ../
```

---

## Notes

- If you move the `robot6000` or `franka_ros2` folders, update the path in `run_franka.sh`:
  ```bash
  REPO_DIR="$(cd "$SCRIPT_DIR/../franka_ros2" && pwd)"
  ```
- The script assumes `docker compose` is available. If you only have `docker-compose` (hyphen version), update the script accordingly.
