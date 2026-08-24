#!/usr/bin/env bash
# One-time setup for the ipopt-rs build. Run this after `brew install ipopt`.
#
# Why this exists: ipopt-sys's vendored CMake build looks for headers at
# <include-dir>/coin/IpIpoptApplication.hpp, a layout COIN-OR abandoned when it
# renamed the headers directory from coin/ to coin-or/ (Homebrew's ipopt ships the
# coin-or/ layout). There's no env var in ipopt-sys to override the search path
# directly, so this creates a local shim: a self-referencing `coin` symlink inside
# a copy of the include path, exposed via a generated pkg-config file that points
# PKG_CONFIG_PATH (set in .cargo/config.toml) at the shim instead of the real
# Homebrew path. Idempotent -- safe to rerun after `brew upgrade ipopt`.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SHIM_DIR="$REPO_ROOT/.ipopt-shim"

if ! command -v brew >/dev/null 2>&1; then
    echo "error: homebrew not found. This setup script assumes a Homebrew-installed ipopt." >&2
    exit 1
fi

if ! brew list ipopt >/dev/null 2>&1; then
    echo "error: ipopt is not installed. Run 'brew install ipopt' first." >&2
    exit 1
fi

IPOPT_PREFIX="$(brew --prefix ipopt)"
IPOPT_INCLUDE="$IPOPT_PREFIX/include/coin-or"

if [ ! -f "$IPOPT_INCLUDE/IpIpoptApplication.hpp" ]; then
    echo "error: expected header not found at $IPOPT_INCLUDE/IpIpoptApplication.hpp" >&2
    echo "ipopt's installed layout may have changed; this script needs updating." >&2
    exit 1
fi

mkdir -p "$SHIM_DIR"
ln -sfn "$IPOPT_INCLUDE" "$SHIM_DIR/coin"

cat > "$SHIM_DIR/ipopt.pc" <<EOF
prefix=$IPOPT_PREFIX
libdir=\${prefix}/lib
includedir=$SHIM_DIR

Name: ipopt
Description: COIN-OR Ipopt (local shim exposing coin-or/ headers under a coin/ symlink for ipopt-sys compatibility)
Version: $(brew list --versions ipopt | awk '{print $2}')
Libs: -L\${libdir} -lipopt
Cflags: -I\${includedir}
EOF

echo "ipopt shim ready at $SHIM_DIR"
echo "cargo build should now pick it up via PKG_CONFIG_PATH in .cargo/config.toml"
