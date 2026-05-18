# ROS 2 / WSL2 network handoff notes

This document records the ROS 2 network work done in this workspace so another
AI assistant or future maintainer can understand the current state without
repeating the investigation.

## Project context

- Workspace: `/home/ray/ros2_ws`
- ROS distro: Jazzy
- Default RMW after this fix: `rmw_cyclonedds_cpp`
- Main issue: ROS 2 DDS discovery was hanging in WSL2 network conditions.
- User-visible symptoms:
  - `ros2 node list` could hang.
  - `ros2 node list --no-daemon` could hang.
  - `ros2 daemon stop/start/status` could hang.
  - Restarting WSL could make a previously working setup fail again.

## Original problem statement

The user summarized the starting point as:

- Root cause: WSL2 network isolation; multicast packets do not reliably get out,
  causing DDS node discovery to fail.
- Symptoms:
  - `ros2 node list` hangs because the daemon connection times out.
  - `ros2 daemon stop/start` hangs.
  - `--no-daemon` also hangs because DDS initialization itself can block.
- Why it seemed to work before:
  - A previous `ros2 daemon stop && ros2 daemon start` happened to succeed once.
  - After WSL was restarted, the problem came back.

This session started from that diagnosis and turned it into persistent shell
configuration plus explicit network-mode commands.

## Root cause summary

ROS 2 graph discovery relies on DDS discovery traffic. In this WSL2 setup,
multicast/subnet discovery is not reliable enough to use as the default path.
When ROS 2 tried to discover over the WSL network, DDS initialization or the ROS
2 CLI daemon path could block.

This work does not make WSL2 multicast universally reliable. Instead, it creates
a stable default for local WSL development and explicit switchable modes for LAN
communication.

## Session timeline

1. Inspected the active ROS/DDS environment and found:
   - current shell still had `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`
   - `.bashrc` contained conflicting FastDDS and CycloneDDS settings
   - fresh shells could be made stable with CycloneDDS bound to `lo`
2. Verified that direct graph queries return when using local-only CycloneDDS:
   - `ros2 node list --no-daemon`
   - `ros2 topic list --no-daemon`
3. Replaced conflicting `.bashrc` exports with a stable WSL2 default.
4. Verified ROS 2 daemon commands return:
   - `ros2 daemon status`
   - `ros2 daemon stop`
   - `ros2 daemon start`
5. Checked WSL networking:
   - Windows `.wslconfig` already uses `networkingMode=mirrored`
   - WSL has LAN interface `eth1` with `192.168.0.181/24`
6. Added switchable helper commands for local, LAN multicast, and explicit-peer
   modes instead of hard-coding only one DDS configuration.
7. Expanded this markdown file into a handoff note for future AI assistants.

## Files changed by Codex

### `/home/ray/.bashrc`

The old `.bashrc` had conflicting ROS settings:

- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`
- another later override to `rmw_cyclonedds_cpp`
- a deprecated CycloneDDS `NetworkInterfaceAddress` XML snippet

Codex replaced that with this WSL2 default block:

```bash
# ROS 2 Jazzy defaults for WSL2.
# WSL2 multicast can be unreliable for DDS discovery. Default to localhost for
# stable single-WSL work; run ros2_wsl_lan or ros2_wsl_peers when needed.
if [ -f "$HOME/ros2_ws/ros2_wsl_network.bash" ]; then
  source "$HOME/ros2_ws/ros2_wsl_network.bash"
  ros2_wsl_local >/dev/null
fi
export TURTLEBOT3_MODEL=burger
```

New terminals now default to a stable local-only ROS 2 mode.

### `/home/ray/ros2_ws/ros2_wsl_network.bash`

Codex added this helper script. It defines the ROS 2 network profiles:

- `ros2_wsl_local`
- `ros2_wsl_lan`
- `ros2_wsl_peers`
- `ros2_wsl_show`

Source it manually if needed:

```bash
source ~/ros2_ws/ros2_wsl_network.bash
```

New bash shells source it automatically through `.bashrc`.

### `/home/ray/ros2_ws/ROS2_WSL_NETWORK.md`

This file is the handoff document.

## Network profiles

### Default: local WSL only

Command:

```bash
ros2_wsl_local
```

Environment:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces></General></Domain></CycloneDDS>'
```

Use this for:

- Gazebo/simulation running inside this WSL distro.
- ROS 2 nodes that all run inside this WSL distro.
- Avoiding hangs from WSL2 multicast discovery.

This is the default mode for new terminals.

### LAN multicast mode

Command:

```bash
ros2_wsl_lan
```

or explicitly:

```bash
ros2_wsl_lan eth1
```

Environment:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth1" multicast="true"/></Interfaces></General></Domain></CycloneDDS>'
```

Use this only when the WSL/Windows/LAN path actually forwards DDS multicast
reliably. It is provided as a switchable mode, not as the default.

### Explicit peer mode

Command:

```bash
ros2_wsl_peers 192.168.0.10 192.168.0.20
```

Environment shape:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
CYCLONEDDS_URI='<CycloneDDS>...<Discovery><Peers><Peer Address="192.168.0.10"/></Peers></Discovery>...</CycloneDDS>'
```

Use this when another ROS 2 host is on the LAN but multicast discovery is not
reliable. Add the IP addresses of the other ROS 2 hosts. Those hosts should use:

- the same `ROS_DOMAIN_ID`
- compatible CycloneDDS peer settings
- firewall rules that allow DDS UDP traffic

If the WSL LAN interface changes:

```bash
export ROS2_WSL_IFACE=eth1
ros2_wsl_peers 192.168.0.10
```

## Useful commands

Show the active ROS 2 network profile:

```bash
ros2_wsl_show
```

Return to safe local-only mode:

```bash
ros2_wsl_local
```

Try LAN multicast mode:

```bash
ros2_wsl_lan eth1
```

Use explicit peers:

```bash
ros2_wsl_peers <other-ros-host-ip>
```

Verify ROS graph without daemon:

```bash
ros2 node list --no-daemon
ros2 topic list --no-daemon
```

Verify daemon commands:

```bash
ros2 daemon status
ros2 daemon stop
ros2 daemon start
```

If daemon commands hang but `--no-daemon` graph queries work, check whether WSL
mirrored networking is intercepting localhost:

```bash
ip route get 127.0.0.1
```

If the route says `dev loopback0`, repair the current WSL session with:

```bash
ros2_wsl_fix_loopback
ros2 daemon start
ros2 daemon status
```

## Verification performed

Codex verified that a fresh shell loads the new default:

```bash
ROS_DOMAIN_ID=0
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces></General></Domain></CycloneDDS>
```

Codex verified these commands returned instead of hanging:

```bash
ros2 node list --no-daemon
ros2 topic list --no-daemon
ros2 daemon status
ros2 daemon stop
ros2 daemon start
```

Observed local topics:

```text
/parameter_events
/rosout
```

Codex also tested:

```bash
ros2_wsl_lan eth1
ros2_wsl_peers 192.168.0.181
```

Both modes returned without hanging. The explicit peer test printed a CycloneDDS
message about disabling multicast on `eth1`; this is expected in peer mode
because peer mode intentionally does not rely on multicast.

## Current machine notes

- Windows WSL version observed: WSL 2.6.3.0.
- Distro: `Ubuntu`, WSL version 2.
- Windows config file exists at `C:\Users\chaon\.wslconfig`.
- That file contains:

```ini
[wsl2]
networkingMode=mirrored
```

- WSL LAN interface observed:

```text
eth1 192.168.0.181/24
```

- Other observed interface:

```text
eth0 100.116.67.4/32
```

- Default route observed through:

```text
192.168.0.1 dev eth1
```

## What was solved

- Stopped relying on FastDDS/subnet multicast as the default.
- Removed conflicting `.bashrc` ROS/DDS settings.
- Replaced deprecated CycloneDDS XML with the `Interfaces` form.
- Made new terminals start in stable local-only mode.
- Added named commands to switch between local, LAN multicast, and explicit
  peer modes.
- Verified ROS 2 graph commands and daemon commands return in the fixed setup.

## What was not solved

- WSL2 multicast itself was not made universally reliable.
- Automatic DDS discovery across Windows, WSL, and LAN hosts is not guaranteed.
- FastDDS multicast/subnet discovery was not repaired; the setup moved to
  CycloneDDS because it is easier to constrain safely in WSL2.

## Guidance for future AI assistants

Do not reintroduce `rmw_fastrtps_cpp` plus `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`
as the default unless the user explicitly wants to test FastDDS again.

Do not remove the `.bashrc` call to `ros2_wsl_local` unless replacing it with an
equally stable default.

For local WSL simulation bugs, keep the environment in `ros2_wsl_local`.

For cross-machine ROS 2 communication, prefer `ros2_wsl_peers <peer-ip>` over
assuming multicast discovery will work.

If testing commands that previously hung, wrap them with `timeout`, for example:

```bash
timeout 8s bash -ic 'ros2 node list --no-daemon'
```

If a currently open terminal still shows old environment values such as
`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, run:

```bash
source ~/.bashrc
```

or open a new terminal.
