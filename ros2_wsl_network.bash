# ROS 2 network profiles for WSL2.
#
# Usage:
#   source ~/ros2_ws/ros2_wsl_network.bash
#   ros2_wsl_local
#   ros2_wsl_lan
#   ros2_wsl_peers 192.168.0.10 192.168.0.20
#   ros2_wsl_fix_loopback

ros2_wsl_base() {
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
}

ros2_wsl_local() {
  ros2_wsl_base
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces></General></Domain></CycloneDDS>'
  echo "ROS 2 WSL mode: local only (CycloneDDS on lo, multicast disabled)"
}

ros2_wsl_lan() {
  ros2_wsl_base
  local iface="${1:-eth1}"
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${iface}\" multicast=\"true\"/></Interfaces></General></Domain></CycloneDDS>"
  echo "ROS 2 WSL mode: LAN subnet (CycloneDDS on ${iface}, multicast enabled)"
}

ros2_wsl_peers() {
  ros2_wsl_base
  local iface="${ROS2_WSL_IFACE:-eth1}"
  local peers_xml=""
  local peer

  for peer in "$@"; do
    peers_xml="${peers_xml}<Peer Address=\"${peer}\"/>"
  done

  if [ -z "${peers_xml}" ]; then
    echo "usage: ros2_wsl_peers <peer-ip> [peer-ip ...]" >&2
    return 2
  fi

  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${iface}\" multicast=\"false\"/></Interfaces></General><Discovery><Peers>${peers_xml}</Peers></Discovery></Domain></CycloneDDS>"
  echo "ROS 2 WSL mode: explicit peers on ${iface} (${*})"
}

ros2_wsl_show() {
  printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID}"
  printf 'ROS_AUTOMATIC_DISCOVERY_RANGE=%s\n' "${ROS_AUTOMATIC_DISCOVERY_RANGE}"
  printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION}"
  printf 'CYCLONEDDS_URI=%s\n' "${CYCLONEDDS_URI}"
}

ros2_wsl_fix_loopback() {
  local route
  route="$(ip route get 127.0.0.1 2>/dev/null || true)"

  if printf '%s\n' "${route}" | grep -q ' dev loopback0 '; then
    echo "WSL mirrored networking is routing 127.0.0.1 through loopback0."
    echo "Adding a higher-priority local route rule for ROS 2 daemon..."
    sudo ip rule add to 127.0.0.0/8 lookup local priority 0 2>/dev/null || true
  fi

  ip route get 127.0.0.1
}
