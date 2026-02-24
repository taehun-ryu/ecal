#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash docker/run_checkerboard_docker.sh /absolute/path/to/data [h5_file_name]

Behavior:
  1) Builds Docker image: ecal-checkerboard:latest
     (target: checkerboard in docker/Dockerfile)
  2) Runs container with:
     -v /absolute/path/to/data:/data
     -v <repo_root>:/workspace
  3) Maps selected H5 file to /data/input.h5 in container
  4) Disables GUI by default in container (ECAL_ENABLE_GUI=0)
  5) Sets working directory to /workspace
  6) Executes ecal_checkerboard_calibrate with /app/config/checkerboard.docker.yaml

Expected host files:
  /absolute/path/to/data/<h5_file_name>

Output:
  <repo_root>/results/checkerboard/run_YYYYMMDD_HHMMSS/

Notes:
  - If h5_file_name is omitted, defaults to input.h5
  - Override GUI behavior with: ECAL_ENABLE_GUI=1 bash docker/run_checkerboard_docker.sh ...
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
  exit 1
fi

DATA_DIR_IN="$1"
if [[ ! -d "$DATA_DIR_IN" ]]; then
  echo "[error] data directory not found: $DATA_DIR_IN" >&2
  exit 1
fi

if command -v realpath >/dev/null 2>&1; then
  DATA_DIR="$(realpath "$DATA_DIR_IN")"
else
  DATA_DIR="$(cd "$DATA_DIR_IN" && pwd)"
fi

H5_FILE="${2:-input.h5}"
if [[ "$H5_FILE" == /* ]]; then
  echo "[error] h5_file_name must be a path relative to data directory: $H5_FILE" >&2
  exit 1
fi

H5_HOST_PATH="$DATA_DIR/$H5_FILE"
if [[ ! -f "$H5_HOST_PATH" ]]; then
  echo "[error] expected input file not found: $H5_HOST_PATH" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE_TAG="${ECAL_CHECKERBOARD_IMAGE_TAG:-ecal-checkerboard:latest}"

mkdir -p "$REPO_ROOT/results/checkerboard"

docker build -f "$SCRIPT_DIR/Dockerfile" --target checkerboard -t "$IMAGE_TAG" "$REPO_ROOT"
docker run --rm -it \
  -v "$REPO_ROOT:/workspace" \
  -v "$DATA_DIR:/data" \
  -v "$H5_HOST_PATH:/data/input.h5:ro" \
  -w /workspace \
  -e ECAL_ENABLE_GUI="${ECAL_ENABLE_GUI:-0}" \
  "$IMAGE_TAG" /app/config/checkerboard.docker.yaml
