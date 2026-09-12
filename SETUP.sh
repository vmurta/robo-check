#!/usr/bin/env bash
# =============================================================================
# SETUP.sh -- prerequisite checker/installer for robo-check
#
# Checks for everything the build needs and installs whatever is missing:
#   * apt packages   : git, build-essential (g++/make), cmake >= 3.14, m4,
#                      libeigen3-dev
#   * CUDA toolkit   : nvcc (prompts before installing; it is a big download)
#   * libccd + FCL   : built from source into /usr/local (for -lfcl -lccd)
#   * googletest     : git submodules used by the CMake/gtest builds
#
# Usage:
#   ./SETUP.sh [--yes] [--debug]
#     --yes      auto-confirm large installs (CUDA toolkit)
#     --debug    verbose debug output (same as DEBUG=1 ./SETUP.sh)
#
# The script may ask for your sudo password when installing packages.
# =============================================================================
set -uo pipefail

# ------------------------------------------------------------------ flags ----
YES=0
DEBUG="${DEBUG:-0}"
NO_TTY=0
[ -t 1 ] || NO_TTY=1

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPS_DIR="$REPO_DIR/build/deps"

usage() {
    cat <<'EOF'
Usage: ./SETUP.sh [--yes] [--debug] [--help]

Checks for every build dependency of robo-check and installs what's missing:

  apt packages   git, build-essential (g++/make), cmake (>= 3.14), m4,
                 libeigen3-dev
  CUDA toolkit   nvcc -- prompts before installing (big download)
  libccd, FCL    built from source into /usr/local (for -lfcl -lccd)
  googletest     git submodules

Options:
  --yes          auto-confirm large installs (CUDA toolkit)
  --debug        verbose debug output (same as DEBUG=1 ./SETUP.sh)
  --help         show this help
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --yes|-y)   YES=1 ;;
        --debug|-d) DEBUG=1 ;;
        --help|-h)  usage; exit 0 ;;
        *)          echo "SETUP.sh: unknown argument '$1' (try --help)" >&2; exit 2 ;;
    esac
    shift
done

# ----------------------------------------------------------------- colors ----
if [ "$NO_TTY" -eq 0 ]; then
    BOLD=$'\033[1m'; RED=$'\033[31m'; GREEN=$'\033[32m'
    YELLOW=$'\033[33m'; BLUE=$'\033[34m'; CYAN=$'\033[36m'; NC=$'\033[0m'
else
    BOLD=""; RED=""; GREEN=""; YELLOW=""; BLUE=""; CYAN=""; NC=""
fi

# ---------------------------------------------------------------- helpers ----
info()  { printf '%b\n' "${CYAN}[info]${NC}  $*"; }
warn()  { printf '%b\n' "${YELLOW}[warn]${NC}  $*" >&2; }
ok()    { printf '%b\n' "${GREEN}[ ok ]${NC}  $*"; }
miss()  { printf '%b\n' "${RED}[MISS]${NC}  $*"; }
fail()  { printf '%b\n' "${RED}[fail]${NC}  $*" >&2; }
debug() { [ "$DEBUG" -eq 1 ] && printf '%b\n' "${BLUE}[dbg ]${NC}  $*" >&2 || true; }
die()   { printf '%b\n' "${RED}[fatal]${NC} $*" >&2; exit 1; }

have() { command -v "$1" >/dev/null 2>&1; }

asroot() { if have sudo; then sudo "$@"; else "$@"; fi; }

FAILS=0

# spinner around a long-running command
SPIN_CHARS='-\|/'
run() {
    local label="$1"; shift
    local log; log="$(mktemp)"
    debug "run: $label :: $*"
    local rc
    if [ "$NO_TTY" -eq 1 ]; then
        info "  $label ..."
        "$@" >"$log" 2>&1
        rc=$?
    else
        "$@" >"$log" 2>&1 &
        local pid=$! i=0
        while kill -0 "$pid" 2>/dev/null; do
            printf '\r  %s %s' "${SPIN_CHARS:i%4:1}" "$label"
            i=$((i + 1))
            sleep 0.1
        done
        wait "$pid"
        rc=$?
        printf '\r\033[2K'
    fi
    if [ "$rc" -ne 0 ]; then
        fail "$label (exit $rc) -- last 15 log lines:"
        tail -n 15 "$log" | sed 's/^/      /' >&2
        rm -f "$log"
        return "$rc"
    fi
    rm -f "$log"
    return 0
}

# determinate progress bar: bar <cur> <total> <label>
BAR_WIDTH=30
bar() {
    [ "$NO_TTY" -eq 1 ] && return 0
    local cur=$1 tot=$2 label=$3
    local fill="" empty="" i
    for ((i = 0; i < cur * BAR_WIDTH / tot; i++)); do fill+="#"; done
    for ((i = cur * BAR_WIDTH / tot; i < BAR_WIDTH; i++)); do empty+="."; done
    printf '\r  [%s%s] %3d%% %s' "$fill" "$empty" "$((cur * 100 / tot))" "$label"
}
bar_done() { [ "$NO_TTY" -eq 1 ] || printf '\r\033[2K'; }

# ----------------------------------------------------------------- checks ----
NEED_APT=()
NEED_CCD=0
NEED_FCL=0
NEED_CUDA=0
NEED_SUB=0

header_found() { # $1 = relative path under /usr[/local]/include
    local base
    for base in /usr/include /usr/local/include; do
        if [ -f "$base/$1" ]; then
            debug "found header $base/$1"
            return 0
        fi
    done
    debug "header not found: $1"
    return 1
}

lib_known() { # $1 = grep -E pattern for lib names
    local cache; cache="$(ldconfig -p 2>/dev/null)"
    if [ -n "$cache" ] && grep -Eq "$1" <<<"$cache"; then
        return 0
    fi
    ls /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib 2>/dev/null | grep -Eq "^$1"
}

cmake_ver() {
    cmake --version 2>/dev/null | head -n1 | grep -oE '[0-9]+\.[0-9]+(\.[0-9]+)?' | head -n1
}

ver_ge() { # $1 a >= $2 b ?
    [ -n "$1" ] || return 1
    [ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | tail -n1)" = "$1" ]
}

check_line() { # $1 = ok|miss|warn, $2 = text
    case "$1" in
        ok)   ok "$2" ;;
        miss) miss "$2"; FAILS=$((FAILS + 1)) ;;
        warn) warn "$2"; FAILS=$((FAILS + 1)) ;;
    esac
}

CHECK_TOTAL=10
CHECK_CUR=0
step() {
    CHECK_CUR=$((CHECK_CUR + 1))
    bar "$CHECK_CUR" "$CHECK_TOTAL" "checking $1"
    [ "$NO_TTY" -eq 0 ] && sleep 0.03
    bar_done
}

check_all() {
    step "git"
    if have git; then check_line ok "git $(git --version)"; else check_line miss "git not found"; NEED_APT+=(git); fi

    step "g++"
    if have g++; then check_line ok "g++ $(g++ -dumpversion 2>/dev/null)"; else check_line miss "g++ not found"; NEED_APT+=(build-essential); fi

    step "make"
    if have make; then check_line ok "make $(make --version | head -n1)"; else check_line miss "make not found"; NEED_APT+=(build-essential); fi

    step "cmake"
    local cver; cver="$(cmake_ver)"
    if [ -z "$cver" ]; then
        check_line miss "cmake not found (FCL needs >= 3.14)"
        NEED_APT+=(cmake)
    elif ver_ge "$cver" "3.14"; then
        check_line ok "cmake $cver (>= 3.14 required)"
    else
        check_line warn "cmake $cver is too old (need >= 3.14); will try to upgrade via apt"
        NEED_APT+=(cmake)
    fi

    step "m4"
    if have m4; then check_line ok "m4 $(m4 --version | head -n1)"; else check_line miss "m4 not found (needed for libccd)"; NEED_APT+=(m4); fi

    step "Eigen3"
    if header_found "eigen3/Eigen/Dense"; then check_line ok "Eigen3 headers"; else check_line miss "Eigen3 not found"; NEED_APT+=(libeigen3-dev); fi

    step "libccd"
    if header_found "ccd/ccd.h" && lib_known 'libccd\.so'; then
        check_line ok "libccd"
    else
        check_line miss "libccd (will build from source)"
        NEED_CCD=1
    fi

    step "FCL"
    if header_found "fcl/fcl.h" && lib_known 'libfcl\.so'; then
        check_line ok "FCL"
    else
        check_line miss "FCL (will build v0.7.0 from source)"
        NEED_FCL=1
    fi

    step "CUDA toolkit"
    if have nvcc; then
        check_line ok "CUDA: $(nvcc --version | grep -oE 'release [0-9]+\.[0-9]+' | head -n1)"
    else
        check_line miss "CUDA toolkit (nvcc) not found"
        NEED_CUDA=1
    fi

    step "googletest submodules"
    if [ -f "$REPO_DIR/googletest/CMakeLists.txt" ]; then
        check_line ok "googletest submodule present"
    elif git -C "$REPO_DIR" rev-parse --git-dir >/dev/null 2>&1; then
        check_line miss "git submodules not initialized"
        NEED_SUB=1
    else
        check_line miss "googletest missing (not a git checkout)"
        NEED_SUB=1
    fi
}

# --------------------------------------------------------------- installers --
install_apt() {
    [ "${#NEED_APT[@]}" -eq 0 ] && { ok "all required apt packages present"; return 0; }
    info "missing apt packages: ${NEED_APT[*]}"
    if ! have sudo; then
        warn "sudo not available -- install manually with:"
        echo "      sudo apt-get update && sudo apt-get install -y ${NEED_APT[*]}"
        FAILS=$((FAILS + 1))
        return 0
    fi
    run "apt-get update" sudo apt-get update || return 1
    run "apt-get install -y ${NEED_APT[*]}" sudo apt-get install -y "${NEED_APT[@]}" || return 1
    local cver; cver="$(cmake_ver)"
    if [ -n "$cver" ] && ! ver_ge "$cver" "3.14"; then
        warn "cmake is still $cver (< 3.14). See https://robots.uc3m.es/installation-guides/install-cmake.html"
    fi
}

install_libccd() {
    info "libccd: building from source into /usr/local"
    mkdir -p "$DEPS_DIR"
    local tot=4 cur

    cur=1; bar "$cur" "$tot" "cloning libccd"
    if [ ! -d "$DEPS_DIR/libccd/.git" ]; then
        [ -e "$DEPS_DIR/libccd" ] && rm -rf "$DEPS_DIR/libccd"
        run "clone libccd" git clone --depth 1 https://github.com/danfis/libccd.git "$DEPS_DIR/libccd" || return 1
    else
        debug "libccd already cloned at $DEPS_DIR/libccd"
    fi
    bar_done

    cur=2; bar "$cur" "$tot" "generating ccd/config.h (m4)"
    run "generate ccd/config.h" bash -c "cd '$DEPS_DIR/libccd/src' && m4 -DUSE_DOUBLE ccd/config.h.m4 > ccd/config.h" || return 1
    bar_done

    cur=3; bar "$cur" "$tot" "cmake configure + build"
    run "cmake configure libccd" cmake -S "$DEPS_DIR/libccd" -B "$DEPS_DIR/libccd/build" \
        -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON || return 1
    run "build libccd (-j$(nproc))" cmake --build "$DEPS_DIR/libccd/build" -j"$(nproc)" || return 1
    bar_done

    cur=4; bar "$cur" "$tot" "installing libccd"
    run "install libccd to /usr/local" asroot make -C "$DEPS_DIR/libccd/build" install || return 1
    bar_done
    ok "libccd installed"
}

install_fcl() {
    info "FCL: building v0.7.0 from source into /usr/local"
    mkdir -p "$DEPS_DIR"
    local tot=4 cur

    cur=1; bar "$cur" "$tot" "cloning FCL (v0.7.0)"
    if [ ! -d "$DEPS_DIR/fcl/.git" ]; then
        [ -e "$DEPS_DIR/fcl" ] && rm -rf "$DEPS_DIR/fcl"
        run "clone FCL" git clone --depth 1 --branch 0.7.0 \
            https://github.com/flexible-collision-library/fcl.git "$DEPS_DIR/fcl" || return 1
    else
        debug "FCL already cloned at $DEPS_DIR/fcl"
    fi
    bar_done

    cur=2; bar "$cur" "$tot" "cmake configure FCL"
    run "cmake configure FCL" cmake -S "$DEPS_DIR/fcl" -B "$DEPS_DIR/fcl/build" \
        -DCMAKE_BUILD_TYPE=Release -DFCL_BUILD_TESTS=OFF || return 1
    bar_done

    cur=3; bar "$cur" "$tot" "building FCL (-j$(nproc))"
    run "build FCL (-j$(nproc))" cmake --build "$DEPS_DIR/fcl/build" -j"$(nproc)" || return 1
    bar_done

    cur=4; bar "$cur" "$tot" "installing FCL"
    run "install FCL to /usr/local" asroot make -C "$DEPS_DIR/fcl/build" install || return 1
    bar_done
    ok "FCL installed"
}

ensure_submodules() {
    if git -C "$REPO_DIR" rev-parse --git-dir >/dev/null 2>&1; then
        run "git submodule update --init --recursive" \
            git -C "$REPO_DIR" submodule update --init --recursive || return 1
    else
        mkdir -p "$DEPS_DIR"
        run "clone googletest" git clone --depth 1 https://github.com/google/googletest.git "$REPO_DIR/googletest" || return 1
        [ -e "$REPO_DIR/test/googletest" ] || ln -s ../googletest "$REPO_DIR/test/googletest"
    fi
    ok "googletest available"
}

ensure_cuda() {
    if have nvcc; then ok "CUDA toolkit present"; return 0; fi
    info "CUDA toolkit (nvcc) not found -- the CUDA builds need it"
    if [ "$YES" -ne 1 ]; then
        if [ "$NO_TTY" -eq 1 ]; then
            warn "non-interactive mode and --yes not given; skipping CUDA install"
            FAILS=$((FAILS + 1))
            return 0
        fi
        printf '  Install via apt now (nvidia-cuda-toolkit, large download)? [y/N] '
        read -r ans
        if [ "${ans,,}" != "y" ]; then
            warn "skipped; install the CUDA toolkit matching your GPU driver manually"
            FAILS=$((FAILS + 1))
            return 0
        fi
    fi
    if ! have sudo; then
        warn "sudo not available -- install manually: sudo apt-get install -y nvidia-cuda-toolkit"
        FAILS=$((FAILS + 1))
        return 0
    fi
    run "apt install nvidia-cuda-toolkit (large download, please wait)" \
        sudo apt-get install -y nvidia-cuda-toolkit || return 1
    have nvcc && ok "CUDA toolkit installed" || warn "nvcc still not on PATH -- check your CUDA install"
}

refresh_ldconfig() {
    if lib_known 'libfcl\.so'; then
        debug "libfcl.so is visible to the dynamic linker"
        return 0
    fi
    info "running ldconfig so the linker can find /usr/local/lib"
    run "ldconfig" asroot ldconfig || warn "ldconfig failed"
    if ! lib_known 'libfcl\.so'; then
        warn "/usr/local/lib is not in the linker path. Add to ~/.bashrc:"
        echo "      export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib"
        FAILS=$((FAILS + 1))
    fi
}

verify() {
    local tmp; tmp="$(mktemp -d)"
    debug "verify tmp dir: $tmp"

    cat > "$tmp/fcl_check.cpp" <<'EOF'
#include <fcl/fcl.h>
#include <Eigen/Dense>
#include <memory>
int main() {
    fcl::Transform3<double> tf = fcl::Transform3<double>::Identity();
    std::shared_ptr<fcl::Box<double>> box = std::make_shared<fcl::Box<double>>(1.0, 1.0, 1.0);
    fcl::CollisionObject<double> obj(box, tf);
    return obj.getObjectType() == fcl::OT_GEOM ? 0 : 1;
}
EOF
    if run "FCL + Eigen compile/link check" g++ -std=c++17 "$tmp/fcl_check.cpp" -o "$tmp/fcl_check" -lfcl -lccd \
        && run "run FCL smoke test" "$tmp/fcl_check"; then
        ok "FCL + Eigen: compiles and links"
    else
        fail "FCL + Eigen check failed"
        FAILS=$((FAILS + 1))
    fi

    if have nvcc; then
        printf '#include <cstdio>\nint main() { return 0; }\n' > "$tmp/nvcc_check.cu"
        if run "nvcc compile check" nvcc "$tmp/nvcc_check.cu" -o "$tmp/nvcc_check"; then
            ok "nvcc: compiles"
        else
            fail "nvcc check failed"
            FAILS=$((FAILS + 1))
        fi
    fi

    rm -rf "$tmp"
}

# -------------------------------------------------------------------- main ---
main() {
    local banner; banner="=================================================="
    printf '%b\n' "${BOLD}${banner}${NC}"
    printf '%b\n' "${BOLD}  robo-check environment setup${NC}"
    printf '%b\n' "${BOLD}${banner}${NC}"
    debug "repo dir : $REPO_DIR"
    debug "deps dir : $DEPS_DIR"
    debug "shell    : $BASH_VERSION"
    debug "yes      : $YES, debug: $DEBUG, tty: $((1 - NO_TTY))"

    # ask for sudo up front (if we will need it) so prompts are not hidden
    if have sudo && { [ "${#NEED_APT[@]}" -gt 0 ] || [ "$NEED_CCD" -eq 1 ] || [ "$NEED_FCL" -eq 1 ] || [ "$NEED_CUDA" -eq 1 ]; }; then
        info "requesting sudo access (may prompt for your password)"
        sudo -v || warn "sudo -v failed; install steps may prompt later"
    fi

    info "checking prerequisites"
    check_all
    bar_done

    info "installing apt packages"
    install_apt || die "apt install step failed"

    info "installing libraries"
    if [ "$NEED_CCD" -eq 1 ]; then
        install_libccd || die "libccd install failed"
    else
        ok "libccd already installed, skipping"
    fi
    if [ "$NEED_FCL" -eq 1 ]; then
        install_fcl || die "FCL install failed"
    else
        ok "FCL already installed, skipping"
    fi
    if [ "$NEED_SUB" -eq 1 ]; then
        ensure_submodules || die "submodule initialization failed"
    else
        ok "submodules already initialized, skipping"
    fi
    if [ "$NEED_CUDA" -eq 1 ]; then
        ensure_cuda || die "CUDA install failed"
    fi

    refresh_ldconfig

    # misses discovered during checks are (hopefully) resolved by now
    FAILS=0

    info "verifying the toolchain"
    verify

    echo
    if [ "$FAILS" -gt 0 ]; then
        fail "$FAILS problem(s) remain -- see messages above"
        exit 1
    fi
    ok "setup complete"
    printf '%b\n' "${BOLD}Next steps:${NC}"
    echo "  make                 # build the library + benchmark binaries"
    echo "  ./run-bvh.sh         # run the BVH benchmark"
    echo "  ./run-benchmarks.sh  # run the benchmark suite"
    echo "If linking fails at runtime, ensure /usr/local/lib is in your library path:"
    echo "  export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib   # add to ~/.bashrc"
}

main "$@"
