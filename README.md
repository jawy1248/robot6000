# FRANKA Project — Dependencies & Packages (README)

This doc summarizes **everything** to bring up and run the Franka Research 3 (FR3) in this project, plus what each piece is for. It’s written to match the current setup: **ROS 2 Humble in Docker**, using the upstream **`franka_ros2` (humble branch)**.

---

## Host machine prerequisites (outside Docker)

> You only need these on the host; all ROS2 packages are inside the container.

- **Docker Engine** + **Docker Compose plugin**  
  Used to build and run the ROS 2 workspace container.

- **Git**  
  To clone `franka_ros2` and this project’s helper scripts.

- _(Recommended)_ Add your user to the `docker` group so you can run Docker without `sudo`.

> For more details, please read the document found [here](readDocker.md)

---

## Networking & robot UI bits (outside ROS)

- **FR3 Web Interface** (robot panel at `robot.franka.de` / robot IP)  
  Used to complete first‑start, login, and safety checks (we hit the browser certificate prompt and used Chrome successfully).

- **Watchman** (on the robot)  
  Robot-side safety application. We encountered “Safety settings invalid… re‑validate in Watchman,” so be prepared to review/confirm safety scenarios there.

> Tip: Keep the robot and control PC on the same isolated network and confirm link by pinging the robot’s configured IP (as set on the robot panel).

---

## What’s inside the container (ROS 2 Humble)

All of the following are provided/installed **inside** the Docker image we build and run.

### Core ROS 2

- **ROS 2 Humble (desktop)**  
  Messaging, TF, RViz, tools—our base middleware.

### Franka stack

- **`franka_ros2` (humble branch)**

  - `franka_hardware` – ROS 2 hardware interface for the FR3 via libfranka
  - `franka_description` – URDF, meshes, and robot model
  - `franka_msgs` – message/service definitions
  - `franka_example_controllers` – simple example controllers (useful sanity checks)

- **libfranka** (brought in by the Docker image used by `franka_ros2`)  
  Low‑level C++ library that talks to the arm.

### ros2_control ecosystem

- **`ros2_control`** – framework for hardware/actuator control
- **`controller_manager`** – loads/starts/stops controllers at runtime (we used its services/CLI)
- **`ros2_controllers`** – common controllers:
  - `joint_state_broadcaster` – publishes joint states (must be running)
  - `joint_trajectory_controller` – follows joint trajectories
  - _(Optional later)_ Cartesian/impedance controllers if we add them

### Build & dev tools (in-container)

- **colcon/ament** – to build the workspace
- **rviz2** – visualization (optional but handy)
- **rqt** tools – debugging/inspection (optional)
- **Python 3 tooling** – for small examples/utilities (we used a `py_pubsub` style test while debugging)

---

## Why each package matters

| Package / Tool                       | Role in our setup                        | Notes                                                                   |
| ------------------------------------ | ---------------------------------------- | ----------------------------------------------------------------------- |
| Docker + Compose                     | Runs a reproducible ROS 2 environment    | Matches the _Docker Container Installation – Option A_ flow we scripted |
| `franka_ros2`                        | The FR3 ROS 2 integration                | We’re on the **humble** branch                                          |
| libfranka                            | Low-level FR3 comms                      | Pulled in by the image used with `franka_ros2`                          |
| `ros2_control`, `controller_manager` | Control framework + controller lifecycle | We used the `list_controllers`/`load_configure_start` flow              |
| `joint_state_broadcaster`            | Publishes `/joint_states`                | Start this first                                                        |
| `joint_trajectory_controller`        | Basic motion control via trajectories    | Our default controller for bring-up                                     |
| RViz / rqt (optional)                | Visualization & debugging                | Helpful during bring-up                                                 |
| colcon/ament                         | Builds the workspace                     | Used in Docker image                                                    |

---

## Minimal bring-up sequence (what we’ve been doing)

> This is here just so a reader sees _how_ the pieces above are used. Adjust names/paths to your repo layout.

1. **Start the container** (from the parent folder with our `robot6000/run_franka.sh`):

   ```bash
   ./robot6000/run_franka.sh        # builds (optionally) and runs the Humble container
   ```

2. **Inside the container**: source the workspace and launch the hardware & controllers (examples):

   ```bash
   # 1) Bring up the hardware (example launch; your file may differ)
   ros2 launch franka_bringup franka_hardware.launch.py robot_ip:=<ROBOT_IP>      robot:=fr3

   # 2) In a new terminal: start the joint state broadcaster
   ros2 control load_controller --set-state active joint_state_broadcaster

   # 3) Start a basic trajectory controller
   ros2 control load_controller --set-state active joint_trajectory_controller
   ```

3. **Verify**:
   ```bash
   ros2 control list_controllers
   ros2 topic echo /joint_states
   ```

---

## Optional add‑ons (not required yet, but commonly paired)

- **MoveIt 2** – for planning (we haven’t used it yet in this project)
- **RViz Motion Planning plugin** – if/when MoveIt is added
- **Keyboard/joystick teleop packages** – for manual jogging/testing

---

## Quick checklist

- [x] Host has Docker, Compose, Git
- [x] Robot first‑start done in the FR3 web UI; Watchman safety validated
- [x] Network link verified (PC ↔︎ robot)
- [x] `franka_ros2` (humble) present in the workspace
- [x] Inside container: `ros2_control` + `controller_manager` available
- [x] `joint_state_broadcaster` and `joint_trajectory_controller` load and go **active**

---

## FAQ we hit along the way

- **`ros2 control list_controllers` just waits**  
  Usually means the `controller_manager` node isn’t running—ensure the hardware/bringup launch is up first.

- **Browser warned about certificate on the robot UI**  
  In our case, Chrome allowed proceeding; Firefox blocked it without an “Accept Risk” path.

- **“Safety settings invalid… re-validate in Watchman”**  
  Open the robot UI, go to Watchman, and re‑validate/ack the highlighted safety scenarios.

---

### That’s it

If you’re cloning this repo fresh, install the **Host prerequisites**, run the **bring‑up sequence**, and you’ll be using the same package stack we’ve used to date.
