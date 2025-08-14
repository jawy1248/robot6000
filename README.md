# ROBOT 6000 Lab Setup Guide

In this repo and document, it is explained how to set up the FRANKA Erica Research 3 robots for the labs. There are also provided shell scripts to ease the process.

The only thing that needs to happen before starting this document is ensuring the machine you are using is running Ubuntu Pro. We _must_ have Ubunut Pro as it includes a realtime kernel that will be needed later on

> Ubuntu 24.04.1 LTS was used for this tutorial, with Pro installed

---

# 1. Install Required Software

First things that needs to happen is to download all the software that will be needed throughout this tutorial (and labs later on)

## Pre-git

1. Ubuntu Pro
2. VSCode (from the SNAP store) (just "code" in the store)
3. Chrome (from browser)
   > This is needed for FRANKA Desk later
4. Git
   ```bash
   sudo apt update
   sudo apt install git
   git config --global user.name "mek1156"
   git config --global user.email "labStation@robot6000.uofu"
   ```

> Now, it is suggested to download this repository and use the previously made shell scripts for ease of use

Setup the SSH key so you can access git/github

```bash
cd ./.ssh
ssh-keygen -t ed25519 -C "labStation@robot6000.uofu"
cat ~/.ssh/id_ed25519.pub
```

Add the SSH key to you github account, the move to desired folder and create the repo locally

```bash
mkdir -p ~/Documents/robot6000
cd ~/Documents/robot6000
git clone git@github.com:jawy1248/robot6000.git
cd robot6000
```

Now you should have this repository downloaded and continue with the next steps

## Post-git

5. Docker
   ```bash
   chmod +x scripts/installDocker.sh
   ./scripts/installDocker.sh
   ```
   > You should see a "Hello from Docker" message at the end, as the docker-hello-world script is ran as a check
6. Docker Desktop
   ```bash
   chmod +x scripts/installDockerDesk.sh
   ./scripts/installDockerDesk.sh
   ```

---

# 2. FRANKA Setup
Now that all the required softwares are installed, we will move on to setting up the FRANKA Erica Research 3 Robot

1. Download the FRANKA ROS2 packages
   ```bash
   cd ~/Documents/robot6000
   git clone git@github.com:frankarobotics/franka_ros2.git
   ```
2. Setup the FRANKA on FRANKA Desk
   1. Turn on the FRANKA (via the control box, on the back)
   2. Ensure the X5 port is connected to the computer via ethernet. This port is found **on the robot**, not on the control box
   3. Open **Chrome** and type in the url `robot.franka.de`
   4. Do the "First Start" reqirements
   5. Setup the admin, safety, and student profiles
3. Setup static IP addresses for control via FCI
   1. Linux machine (*Workstation PC*)
   2. FRANKA Desk (*Control*)
   ![staticIP](figs/staticIPs.png)