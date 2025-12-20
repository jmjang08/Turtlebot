#!/usr/bin/env bash
set -euo pipefail

############### Argument Parsing ###############

NAME_SPACE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -n|--n|--ns|--namespace)
      ROBOT_ID="$2"
      NAME_SPACE="/robot${ROBOT_ID}"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: ./prep_checker --n <robot_number>"
      exit 1
      ;;
  esac
done

# Default value if no option is provided
if [[ -z "${NAME_SPACE}" ]]; then
  NAME_SPACE="/robot1"
fi

echo "Using namespace: ${NAME_SPACE}"

############### Configuration ###############

ODOM_FRAME=${ODOM_FRAME:-odom}
BASE_LINK_FRAME=${BASE_LINK_FRAME:-base_link}
LASER_FRAME=${LASER_FRAME:-rplidar_link}
SCAN_TOPIC=${SCAN_TOPIC:-/scan}
MAP_TOPIC=${MAP_TOPIC:-/map}
TIMEOUT=${TIMEOUT:-10}

##############################################

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
No_Color='\033[0m'

function ok() {
  echo -e "${GREEN}[OK]${No_Color} $1"
}

function warn() {
  echo -e "${YELLOW}[WARN]${No_Color} $1"
}

function error() {
  echo -e "${RED}[ERROR]${No_Color} $1"
  exit 1
}

echo "===================================================="
echo "  Robot Readiness Checker: ${NAME_SPACE}"
echo "===================================================="

# 1. Check Map Topic (Global Map Server Status)
echo "1. Checking Global Map Server..."
if ! ros2 topic list | grep -q "^${MAP_TOPIC}$"; then
  error "Map topic ${MAP_TOPIC} not found. Please check map_server/nav2."
fi
ok "Map topic ${MAP_TOPIC} is active."

# 2. Check Odometry (odom -> base_link)
echo "2. Checking Odometry TF..."
echo "  - Looking for: ${NAME_SPACE}/${ODOM_FRAME} -> ${NAME_SPACE}/${BASE_LINK_FRAME}"

TMP_TF=$(mktemp)
set +e
timeout "${TIMEOUT}" \
  ros2 run tf2_ros tf2_echo "${NAME_SPACE}/${ODOM_FRAME}" "${NAME_SPACE}/${BASE_LINK_FRAME}" \
  --ros-args -r tf:=${NAME_SPACE}/tf -r tf_static:=${NAME_SPACE}/tf_static \
  | head -n 20 > "${TMP_TF}" 2>&1
set -e

if ! grep -q "At time" "${TMP_TF}"; then
  warn "TF [${ODOM_FRAME} -> ${BASE_LINK_FRAME}] not found within ${TIMEOUT}s. Check robot driver/bringup."
else
  ok "TF [${ODOM_FRAME} -> ${BASE_LINK_FRAME}] is healthy."
fi
rm -f "${TMP_TF}"

# 3. Check LiDAR (LaserScan)
echo "3. Checking LiDAR status..."

# 3-1) Check topic existence
if ! ros2 topic list | grep -q "^${NAME_SPACE}${SCAN_TOPIC}$"; then
  error "LaserScan topic ${NAME_SPACE}${SCAN_TOPIC} not found. Check LiDAR driver."
fi
ok "LaserScan topic ${NAME_SPACE}${SCAN_TOPIC} detected."

# 3-2) Check topic type
FULL_SCAN_TOPIC="${NAME_SPACE}${SCAN_TOPIC}"
SCAN_TYPE=$(ros2 topic type "${FULL_SCAN_TOPIC}" 2>/dev/null)

if [[ "${SCAN_TYPE}" != "sensor_msgs/msg/LaserScan" ]]; then
  warn "Topic ${FULL_SCAN_TOPIC} has wrong type: ${SCAN_TYPE}"
else
  ok "Topic ${FULL_SCAN_TOPIC} is valid sensor_msgs/msg/LaserScan."
fi

# 3-3) Check base_link -> laser TF
echo "  - Checking TF: ${BASE_LINK_FRAME} -> ${LASER_FRAME}..."
TMP_BASE_LASER_TF=$(mktemp)
set +e
timeout "${TIMEOUT}" \
  ros2 run tf2_ros tf2_echo "${BASE_LINK_FRAME}" "${LASER_FRAME}" \
    --ros-args -r tf:=${NAME_SPACE}/tf -r tf_static:=${NAME_SPACE}/tf_static \
  | head -n 20 > "${TMP_BASE_LASER_TF}" 2>&1
set -e

if ! grep -q "At time" "${TMP_BASE_LASER_TF}"; then
  warn "TF [${BASE_LINK_FRAME} -> ${LASER_FRAME}] not found. Check static_tf_publisher or URDF."
else
  ok "TF [${BASE_LINK_FRAME} -> ${LASER_FRAME}] is healthy."
fi
rm -f "${TMP_BASE_LASER_TF}"

echo "===================================================="
ok "Pre-flight check completed for ${NAME_SPACE}."
echo "===================================================="