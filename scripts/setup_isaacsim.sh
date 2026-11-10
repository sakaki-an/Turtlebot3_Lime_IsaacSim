#!/usr/bin/env bash
# Install Isaac Sim 6.2 (pip) into a repo-local virtual environment: <repo>/.venv
#
# Usage:  ./scripts/setup_isaacsim.sh
# Then:   source scripts/isaac_env.sh   # activates .venv and clears any sourced ROS env
#         isaacsim                      # launches the Isaac Sim GUI
#
# Requirements: Ubuntu 22.04/24.04, NVIDIA driver (see Isaac Sim 6.2 requirements), ~30 GB free disk.
set -euo pipefail

ISAACSIM_VERSION="${ISAACSIM_VERSION:-6.2.0.0}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${REPO_ROOT}/.venv"

if ! command -v uv >/dev/null 2>&1; then
  echo "[setup] 'uv' not found -> installing it to ~/.local/bin (https://docs.astral.sh/uv/)"
  curl -LsSf https://astral.sh/uv/install.sh | sh
  export PATH="$HOME/.local/bin:$PATH"
fi

# Isaac Sim 6.x requires Python 3.12 exactly.
if [ ! -d "$VENV" ]; then
  uv venv --seed --python 3.12 "$VENV"   # --seed adds pip to the venv
fi

# shellcheck disable=SC1091
source "$VENV/bin/activate"

echo "[setup] Installing isaacsim==${ISAACSIM_VERSION} (this downloads several GB)..."
# Plain pip (as in NVIDIA's install guide): it resolves Isaac Sim's mix of pypi.org and
# pypi.nvidia.com packages, including a few pinned pre-releases, without extra flags.
python -m pip install --upgrade pip
python -m pip install "isaacsim[all,extscache]==${ISAACSIM_VERSION}" --extra-index-url https://pypi.nvidia.com

# Small extras used by the tutorial scripts (video export for result clips).
python -m pip install imageio imageio-ffmpeg

# Workaround for the full GUI app of a pip install (seen with 6.2.0): several bundled Kit extensions
# ship their own *differently compiled* copies of `charset_normalizer`. When the GUI starts, numpy's
# f2py ends up mixing modules from two copies and core extensions (isaacsim.core.experimental.*)
# fail with "charset_normalizer.md.MessDetectorPlugin size changed". Importing the package once at
# interpreter start (a .pth file runs at startup) makes every later import reuse one consistent copy.
SITE_PACKAGES="$(python -c 'import site; print(site.getsitepackages()[0])')"
echo "import charset_normalizer, charset_normalizer.md, charset_normalizer.cd" \
  > "${SITE_PACKAGES}/zz_preload_charset_normalizer.pth"

echo
echo "[setup] Done. Next:"
echo "  source scripts/isaac_env.sh"
echo "  isaacsim isaacsim.exp.compatibility_check   # optional: check GPU/driver"
echo "  isaacsim                                     # launch the GUI"
