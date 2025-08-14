# ROBOT 6000 Lab Setup Guide

In this repo and document, it is explained how to set up the FRANKA Erica Research 3 robots for the labs. There are also provided shell scripts to ease the process.

The only thing that needs to happen before starting this document is ensuring the machine you are using is running Ubuntu Pro. We _must_ have Ubunut Pro as it includes a realtime kernel that will be needed later on

> Ubuntu 24.04.1 LTS was used for this tutorial, with Pro installed

---

---

# 1. Install Required Software

1. Ubuntu Pro
2. VSCode (from the SNAP store) (just "code" in the store)
3. Docker
   ```bash
   chmod +x installDocker.sh
   ./installDocker.sh
   ```
4. Docker Desktop
   ```bash
   chmod +x installDockerDesk.sh
   ./installDockerDesk.sh
   ```
