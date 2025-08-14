#!/usr/bin/env bash
#
# build_preempt_rt.sh
#
# Automate building and installing a PREEMPT_RT Linux kernel on Ubuntu
# following the steps in kernel.md.
#
# Assumptions:
#   - This script lives in the "robot6000" directory.
#   - Sources and build artifacts will be placed in the parent directory.
#
# Usage:
#   ./build_preempt_rt.sh [--kernel 6.9] [--rt 5] [--menuconfig] [--no-install] [--jobs N]
#
# Defaults (can be overridden by flags or env vars):
#   KERNEL_VERSION=6.9        (example from kernel.md)
#   RT_PATCH_VERSION=5        (patch-6.9-rt5)
#   LOCALVERSION=-rt
#
# Notes:
#   - Requires sudo for package install and kernel install steps.
#   - If --no-install is used, the .deb packages are built but not installed.
#   - If --menuconfig is used, you'll be dropped into menuconfig before build.
#   - Post-install steps: realtime group/limits and cpu governor setup.
#

set -euo pipefail

# -------- Defaults --------
KERNEL_VERSION="${KERNEL_VERSION:-6.9}"
RT_PATCH_VERSION="${RT_PATCH_VERSION:-5}"
LOCALVERSION="${LOCALVERSION:--rt}"
JOBS="${JOBS:-$(nproc || echo 4)}"
DO_INSTALL="1"
DO_MENUCONFIG="0"

# -------- Parse Flags --------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --kernel)       KERNEL_VERSION="${2:-$KERNEL_VERSION}"; shift 2 ;;
    --rt)           RT_PATCH_VERSION="${2:-$RT_PATCH_VERSION}"; shift 2 ;;
    --jobs)         JOBS="${2:-$JOBS}"; shift 2 ;;
    --no-install)   DO_INSTALL="0"; shift ;;
    --menuconfig)   DO_MENUCONFIG="1"; shift ;;
    -h|--help)
      sed -n '1,80p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
  esac
done

# -------- Directories --------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORK_DIR="${PARENT_DIR}/kernel-${KERNEL_VERSION}-rt${RT_PATCH_VERSION}"
mkdir -p "${WORK_DIR}"
cd "${WORK_DIR}"

# -------- URLs and Filenames (from kernel.md patterns) --------
KERNEL_TARBALL="linux-${KERNEL_VERSION}.tar.xz"
KERNEL_MAJOR_MINOR="$(echo "${KERNEL_VERSION}" | awk -F. '{print $1"."$2}')"
RT_PATCH="patch-${KERNEL_VERSION}-rt${RT_PATCH_VERSION}.patch.xz"

# kernel v6.x url layout
KERNEL_URL="https://cdn.kernel.org/pub/linux/kernel/v6.x/${KERNEL_TARBALL}"
RT_URL="https://cdn.kernel.org/pub/linux/kernel/projects/rt/${KERNEL_MAJOR_MINOR}/${RT_PATCH}"

# -------- Helpers --------
require_cmd() {
  command -v "$1" >/dev/null 2>&1 || { echo "Missing required command: $1" >&2; exit 1; }
}

# -------- Checks --------
require_cmd wget
require_cmd xz
require_cmd tar
require_cmd patch
require_cmd grep

echo "==> Installing build dependencies (sudo required)..."
sudo apt update
sudo apt install -y build-essential git bc bison flex libelf-dev \
  libssl-dev dwarves libncurses-dev xz-utils fakeroot

# -------- Download --------
if [[ ! -f "${KERNEL_TARBALL}" ]]; then
  echo "==> Downloading kernel: ${KERNEL_URL}"
  wget -q --show-progress "${KERNEL_URL}"
else
  echo "==> Kernel tarball already present: ${KERNEL_TARBALL}"
fi

if [[ ! -f "${RT_PATCH}" ]]; then
  echo "==> Downloading PREEMPT_RT patch: ${RT_URL}"
  wget -q --show-progress "${RT_URL}"
else
  echo "==> RT patch already present: ${RT_PATCH}"
fi

# -------- Extract & Patch --------
SRC_DIR="linux-${KERNEL_VERSION}"
if [[ ! -d "${SRC_DIR}" ]]; then
  echo "==> Extracting kernel sources..."
  tar -xf "${KERNEL_TARBALL}"
fi

cd "${SRC_DIR}"

if [[ ! -f .rt_patched ]]; then
  echo "==> Applying PREEMPT_RT patch..."
  xzcat "../${RT_PATCH}" | patch -p1
  touch .rt_patched
else
  echo "==> PREEMPT_RT patch already applied."
fi

# -------- Configure --------
echo "==> Preparing kernel configuration..."
# Start from current config if available
if zcat /proc/config.gz >/dev/null 2>&1; then
  zcat /proc/config.gz > .config || true
fi

yes "" | make olddefconfig

if [[ "${DO_MENUCONFIG}" == "1" ]]; then
  echo "==> Launching menuconfig..."
  make menuconfig
else
  echo "==> Skipping menuconfig (use --menuconfig to enable)."
fi

# Try to ensure CONFIG_PREEMPT_RT=y (best-effort guard)
# Use 'scripts/config' if available; otherwise append and olddefconfig.
if [[ -x scripts/config ]]; then
  scripts/config --enable CONFIG_PREEMPT_RT || true
  yes "" | make olddefconfig
else
  if ! grep -q '^CONFIG_PREEMPT_RT=y' .config; then
    echo "CONFIG_PREEMPT_RT=y" >> .config
    yes "" | make olddefconfig
  fi
fi

# -------- Build --------
echo "==> Building Debian packages (this can take a while)..."
make -j"${JOBS}" bindeb-pkg LOCALVERSION="${LOCALVERSION}"

cd ..
IMAGE_DEB="$(ls -t linux-image-*${LOCALVERSION}_*.deb | head -n1 || true)"
HEADERS_DEB="$(ls -t linux-headers-*${LOCALVERSION}_*.deb | head -n1 || true)"

if [[ -z "${IMAGE_DEB}" || -z "${HEADERS_DEB}" ]]; then
  echo "ERROR: Could not find built .deb packages. Check the build output." >&2
  exit 1
fi
echo "==> Built packages:"
echo "    ${IMAGE_DEB}"
echo "    ${HEADERS_DEB}"

# -------- Install (optional) --------
if [[ "${DO_INSTALL}" == "1" ]]; then
  echo "==> Installing kernel packages (sudo required)..."
  sudo dpkg -i "${IMAGE_DEB}" "${HEADERS_DEB}"
  echo "==> Updating GRUB..."
  sudo update-grub
else
  echo "==> Skipping install (--no-install)."
fi

# -------- Post-install setup (best-effort) --------
if [[ "${DO_INSTALL}" == "1" ]]; then
  echo "==> Configuring realtime group and limits..."
  if ! getent group realtime >/dev/null; then
    sudo addgroup realtime
  fi
  sudo usermod -a -G realtime "$USER"

  LIMITS_FILE="/etc/security/limits.d/realtime.conf"
  if [[ ! -f "${LIMITS_FILE}" ]]; then
    sudo tee "${LIMITS_FILE}" >/dev/null <<'EOF'
@realtime   soft   rtprio   99
@realtime   hard   rtprio   99
@realtime   soft   memlock  unlimited
@realtime   hard   memlock  unlimited
EOF
  fi

  echo "==> Installing cpupower and setting CPU governor to performance (best-effort)..."
  sudo apt install -y linux-tools-common "linux-tools-$(uname -r)" || true
  if command -v cpupower >/dev/null 2>&1; then
    sudo cpupower frequency-set -g performance || true
  else
    echo "NOTE: cpupower not available; skipping governor change."
  fi
fi

echo
echo "==================== Summary ===================="
echo "Kernel version:        ${KERNEL_VERSION}${LOCALVERSION}"
echo "Work directory:        ${WORK_DIR}"
echo "Built image package:   ${IMAGE_DEB}"
echo "Built headers package: ${HEADERS_DEB}"
if [[ "${DO_INSTALL}" == "1" ]]; then
  echo "Install status:        Installed"
  echo "Next steps:"
  echo "  1) Reboot to the new kernel."
  echo "  2) Verify: grep PREEMPT_RT /boot/config-\\$(uname -r)"
  echo "  3) Ensure your user is in 'realtime' group (log out/in if needed)."
else
  echo "Install status:        Not installed (--no-install)"
  echo "You can install later with:"
  echo "  sudo dpkg -i \"${WORK_DIR}/${IMAGE_DEB}\" \"${WORK_DIR}/${HEADERS_DEB}\" && sudo update-grub && sudo reboot"
fi
echo "================================================="
