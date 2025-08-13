# Building and Installing a PREEMPT_RT Kernel on Ubuntu (Without Ubuntu Pro)

This guide describes how to build and install a Linux kernel with the **PREEMPT_RT** patch on Ubuntu 22.04 or 24.04 without using Ubuntu Pro. This will enable low-latency, deterministic scheduling required for real-time control of the Franka Research 3.

---

## Install Required Build Tools
```bash
sudo apt update
sudo apt install -y build-essential git bc bison flex libelf-dev   libssl-dev dwarves libncurses-dev xz-utils fakeroot
```

---

## Download Kernel Source & PREEMPT_RT Patch

1. Visit the official [PREEMPT_RT patch repository](https://mirrors.edge.kernel.org/pub/linux/kernel/).
2. Choose a **stable LTS kernel version** that has a matching PREEMPT_RT patch.  
   - Common choices: `6.1.x` or `6.6.x` (LTS).
3. Download:
   - `linux-<version>.tar.xz`
   - `patch-<version>-rtXX.patch.xz`

Example:
```bash
wget https://cdn.kernel.org/pub/linux/kernel/v6.x/linux-6.9.tar.xz
wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/6.9/patch-6.9-rt5.patch.xz
```

---

## Extract and Apply the RT Patch
```bash
tar -xf linux-6.1.92.tar.xz
cd linux-6.1.92
xzcat ../patch-6.1.92-rt39.patch.xz | patch -p1
```

---

## Configure the Kernel

Start from your current system configuration if possible:
```bash
zcat /proc/config.gz > .config 2>/dev/null || true
yes "" | make olddefconfig
make menuconfig
```

In the configuration menu:
- Navigate: **General Setup → Preemption Model → Fully Preemptible Kernel (Real-Time)**  
  (This sets `CONFIG_PREEMPT_RT=y`.)
- Save and exit.

---

## Build the Kernel as `.deb` Packages

```bash
make -j"$(nproc)" bindeb-pkg LOCALVERSION=-rt
```

When finished, `.deb` files will be located **one directory up**:
- `../linux-image-<version>-rt_*.deb`
- `../linux-headers-<version>-rt_*.deb`

---

## Install and Reboot
```bash
cd ..
sudo dpkg -i linux-image-*-rt_*.deb linux-headers-*-rt_*.deb
sudo update-grub
sudo reboot
```

---

## Verify PREEMPT_RT
```bash
uname -r                                   # should include "-rt"
grep PREEMPT_RT /boot/config-$(uname -r)   # should output CONFIG_PREEMPT_RT=y
```

---

## Post-Install Real-Time User Setup

### Add a `realtime` group and set limits:
```bash
sudo addgroup realtime
sudo usermod -a -G realtime $USER
sudo tee -a /etc/security/limits.conf >/dev/null <<'EOF'
@realtime   soft   rtprio   99
@realtime   hard   rtprio   99
@realtime   soft   memlock  unlimited
@realtime   hard   memlock  unlimited
EOF
```
Log out/in or reboot to apply group changes.

---

## Optimize CPU Performance

### Set CPU governor to `performance`:
```bash
sudo apt install -y linux-tools-common linux-tools-$(uname -r)
sudo cpupower frequency-set -g performance
```

---

## (Optional) Isolate a CPU Core for Control Threads
Edit `/etc/default/grub` and modify:
```
GRUB_CMDLINE_LINUX_DEFAULT="... nohz_full=3 isolcpus=3 rcu_nocbs=3 idle=poll"
```
Then:
```bash
sudo update-grub
sudo reboot
```

---

## Checklist Before Running the Franka
- `CONFIG_PREEMPT_RT=y` verified.
- User in `realtime` group.
- CPU governor set to `performance`.
- Controller launched with fixed RT priority (e.g., `chrt -f 80`).
- Stable ping/jitter to robot confirmed.

---
