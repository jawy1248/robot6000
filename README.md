# ROBOT 6000 Lab Setup Guide

In this repo and document, it is explained how to set up the FRANKA Erica Research 3 robots for the labs. There are also provided shell scripts to ease the process.

The only thing that needs to happen before starting this document is ensuring the machine you are using is running Ubuntu Pro. We _must_ have Ubunut Pro as it includes a realtime kernel that will be needed later on

> Ubuntu 24.04.1 LTS was used for this tutorial, with Pro installed

---

---

# 1. Install Required Software

## Pre-git

1. Ubuntu Pro
2. VSCode (from the SNAP store) (just "code" in the store)
3. Git
   ```bash
   sudo apt update
   sudo apt install git
   git config --global user.name "mek1156"
   git config --global user.email "labStation@robot6000.uofu"
   ```

> Now, it is suggested to download this repository and use the previously made shell scripts for ease of use

```bash
cd ./.ssh
ssh-keygen -t ed25519 -C "labStation@robot6000.uofu"
```

## Post-git

4. Docker
   ```bash
   chmod +x scripts/installDocker.sh
   ./scripts/installDocker.sh
   ```
5. Docker Desktop
   ```bash
   chmod +x scripts/installDockerDesk.sh
   ./scripts/installDockerDesk.sh
   ```
