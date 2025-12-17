#!/usr/bin/env bash
set -euo pipefail

script_name="$(basename "$0")"
WS_NAME="remote_ws"

DRY_RUN=0
CONFIRM=0
CLEAN_ALL=0
SOFT_ONLY=0
VERBOSE=0
WITH_LOG=0
# Initialize as empty array to avoid "unbound variable" under set -u
PKGS=()

usage() {
  cat <<EOF
Usage: ${script_name} [options]

Clean ROS2 workspace artifacts in the repository root.

Options:
  -y, --yes         Proceed without confirmation
  -a, --all         Also remove caches: __pycache__, *.pyc, .pytest_cache, .mypy_cache, .ruff_cache
      --soft        Only remove top-level build/, install/, log/
      --pkg NAME    Clean only the specified package (repeatable or comma-separated)
      --with-log    Also remove top-level log/ during package clean
      --dry-run     Show what would be removed
      --verbose     Print paths as they are processed
  -h, --help        Show this help

Defaults: removes top-level build/, install/, log/. Use --pkg for package-specific clean. Use --all for extra caches.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -y|--yes) CONFIRM=1 ;;
    -a|--all) CLEAN_ALL=1 ;;
    --soft) SOFT_ONLY=1 ;;
    --pkg)
      if [[ $# -lt 2 ]]; then echo "Error: --pkg requires a name" >&2; exit 1; fi
      IFS=',' read -r -a __arr <<< "$2"; PKGS+=("${__arr[@]}"); shift ;;
    --with-log) WITH_LOG=1 ;;
    --dry-run) DRY_RUN=1 ;;
    --verbose) VERBOSE=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
  shift
done

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${script_dir}/../${WS_NAME}" && pwd)"

info() { echo "$*"; }

delete_path() {
  local p="$1"
  if [[ -e "$p" ]]; then
    if (( DRY_RUN )); then
      info "[dry-run] rm -rf $p"
    else
      rm -rf "$p"
      if (( VERBOSE )); then info "Removed: $p"; fi
    fi
  else
    if (( VERBOSE )); then info "Skip missing: $p"; fi
  fi
}

ensure_workspace() {
  if [[ ! -d "${ROOT_DIR}/src" ]]; then
    echo "Error: src/ not found under ${ROOT_DIR}. Aborting for safety." >&2
    exit 1
  fi
}

plan_summary() {
  info "Workspace: ${ROOT_DIR}"
  if (( ${#PKGS[@]} > 0 )); then
    info "Package clean: ${PKGS[*]}"
    info "Actions: remove build/<pkg>, install/<pkg>"
    if (( WITH_LOG )); then info "Also: remove top-level log/"; fi
  else
    info "Actions: remove top-level build/, install/, log/"
  fi
  if (( CLEAN_ALL )) && (( ! SOFT_ONLY )); then
    info "Extra: remove caches (__pycache__, *.pyc, .pytest_cache, .mypy_cache, .ruff_cache)"
  fi
}

confirm_or_exit() {
  if (( DRY_RUN )); then return 0; fi
  if (( CONFIRM )); then return 0; fi
  read -r -p "Proceed with cleanup? [y/N] " ans
  case "$ans" in
    y|Y|yes|YES) ;;
    *) echo "Aborted."; exit 1 ;;
  esac
}

clean_top_level() {
  delete_path "${ROOT_DIR}/build"
  delete_path "${ROOT_DIR}/install"
  delete_path "${ROOT_DIR}/log"
}
clean_selected_packages() {
  for pkg in "${PKGS[@]}"; do
    delete_path "${ROOT_DIR}/build/${pkg}"
    delete_path "${ROOT_DIR}/install/${pkg}"
  done
  if (( WITH_LOG )); then
    delete_path "${ROOT_DIR}/log"
  fi
}


clean_caches() {
  if (( SOFT_ONLY )); then return 0; fi
  if (( CLEAN_ALL )); then
    if (( DRY_RUN )); then
      find "${ROOT_DIR}" -type d -name "__pycache__" -print
      find "${ROOT_DIR}" -type f -name "*.pyc" -print
      find "${ROOT_DIR}" -type d \( -name ".pytest_cache" -o -name ".mypy_cache" -o -name ".ruff_cache" \) -print
    else
      find "${ROOT_DIR}" -type d -name "__pycache__" -exec rm -rf {} +
      find "${ROOT_DIR}" -type f -name "*.pyc" -delete
      find "${ROOT_DIR}" -type d \( -name ".pytest_cache" -o -name ".mypy_cache" -o -name ".ruff_cache" \) -exec rm -rf {} +
    fi
  fi
}

main() {
  ensure_workspace
  plan_summary
  confirm_or_exit
  if (( ${#PKGS[@]} > 0 )); then
    clean_selected_packages
  else
    clean_top_level
  fi
  clean_caches
  info "Cleanup complete."
}

main "$@"
