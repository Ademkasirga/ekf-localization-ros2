#!/usr/bin/env bash
set -euo pipefail

WS="/home/ademkas/ekf_ws"
CSV="/tmp/ekf_gazebo_benchmark.csv"
DURATION="${1:-90}"
MAX_MEAN_EKF_ERR="${2:-0.22}"

cd "$WS"
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
else
  echo "ROS setup.bash not found" >&2
  exit 1
fi
# shellcheck disable=SC1091
source install/setup.bash

export LIBGL_ALWAYS_SOFTWARE=1

rm -f "$CSV"

ros2 launch vehicle_bringup gazebo_ekf_benchmark.launch.py output_csv:=${CSV} &
LAUNCH_PID=$!

cleanup() {
  kill "$LAUNCH_PID" 2>/dev/null || true
  sleep 2
  pkill -f gzserver 2>/dev/null || true
  pkill -f gzclient 2>/dev/null || true
}
trap cleanup EXIT

echo "Waiting for Gazebo + EKF (${DURATION}s)..."
sleep "$DURATION"

python3 - <<'PY' "$CSV" "$MAX_MEAN_EKF_ERR"
import sys
import csv
import math
from statistics import mean

path, max_mean = sys.argv[1], float(sys.argv[2])
rows = []
with open(path, newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        try:
            ekf = float(row['ekf_err'])
            gps = float(row['gps_err'])
        except (KeyError, ValueError):
            continue
        if math.isnan(ekf) or math.isnan(gps):
            continue
        rows.append((ekf, gps))

if len(rows) < 50:
    print(f"FAIL: only {len(rows)} valid samples in {path}")
    sys.exit(1)

ekf_errs = [r[0] for r in rows]
gps_errs = [r[1] for r in rows]
m_ekf = mean(ekf_errs)
m_gps = mean(gps_errs)
p95_ekf = sorted(ekf_errs)[int(0.95 * len(ekf_errs)) - 1]

print(f"samples={len(rows)} mean_ekf_err={m_ekf:.4f} mean_gps_err={m_gps:.4f} p95_ekf={p95_ekf:.4f}")

ok = m_ekf < max_mean and m_ekf < m_gps * 0.85
if ok:
    print("PASS: EKF beats noisy GPS and mean error is within threshold.")
    sys.exit(0)
print("FAIL: EKF not good enough yet.")
sys.exit(2)
PY
