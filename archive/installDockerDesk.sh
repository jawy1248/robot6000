#!/bin/bash

set -euo pipefail

sudo apt install gnome-terminal
wget --show-progress -P ~/Downloads https://desktop.docker.com/linux/main/amd64/docker-desktop-amd64.deb?utm_source=docker&utm_medium=webreferral&utm_campaign=docs-driven-download-linux-amd64&_gl=1*1j7ak6n*_gcl_au*MTk3NDA2NzExNC4xNzU0NDE3Nzgx*_ga*MTA3MDI4MzI1Ny4xNzU0NDE3NzIx*_ga_XJWPQMJYHQ*czE3NTUxOTM0MjckbzMkZzEkdDE3NTUxOTM2NDgkajEyJGwwJGgw
sudo apt-get update
sudo apt-get install ~/Downloads/docker-desktop-amd64.deb?utm_source=docker