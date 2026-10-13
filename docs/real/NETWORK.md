# 實車網路

更新時間：2026-10-14。IP 都是 DHCP 拿到的，路由器綁定之前可能會變。

## 1. 裝置

| 裝置 / 介面 | MAC | IP（2026-10-14） | 說明 |
|---|---|---|---|
| Orin 有線 `enP8p1s0` | `3c:6d:66:76:88:57` | 192.168.0.14 | 預設路由（metric 100）；車子開走後不會接線 |
| Orin Wi-Fi `wlP1p1s0` | `58:02:05:de:1e:7d` | **192.168.0.15** | 目前 SSH / Foxglove 用這個；車上只剩這條 |
| Orin NetBird `wt0` | — | 100.80.15.177 | `112ab-jno.netbird.cloud`，見第 3 節 |
| Orin `l4tbr0` | — | 192.168.55.1 | USB 裝置模式用，目前 DOWN |
| Orin `can0` | — | — | DOWN（R550 若用 CAN 才會用到） |
| WAVE ROVER ESP32（Wi-Fi STA） | `30:76:F5:7D:DD:44` | 192.168.0.16 | HTTP 80 埠 |
| Ray 的 Windows 電腦（乙太網路） | — | 192.168.0.181 | |
| Ray 的 WSL | — | 172.21.173.167/20 | `networkingMode=nat`，在 Windows 後面，見第 5 節 |
| 路由器 | — | 192.168.0.1 | 型號 UNKNOWN |

連線測試（Windows，`Test-NetConnection`，2026-10-14）：

| 目標 | 結果 |
|---|---|
| 192.168.0.15:22（Orin SSH） | TCP 成功（ping 不通，但 TCP 通） |
| 192.168.0.16:80（ESP32 HTTP） | TCP 成功（ping 不通） |
| 192.168.0.15:8765（Foxglove） | 失敗：`foxglove_bridge` 還沒裝，見第 4 節 |

用到的埠：SSH 22、ESP32 HTTP 80、Foxglove 8765、ROS 2 DDS（UDP 7400 起，依 `ROS_DOMAIN_ID` 而定）。

## 2. 路由器 DHCP 綁定（Ray 自己做）

路由器型號未知，選單名稱可能不同（常見：「DHCP 保留」、「靜態 IP 分配」、「Address Reservation」）。

1. 用瀏覽器開 `http://192.168.0.1`，登入管理頁面。
2. 找到 DHCP 的「保留位址 / 綁定」頁面，新增三筆：

   | MAC | 綁定 IP |
   |---|---|
   | `58:02:05:de:1e:7d`（Orin Wi-Fi） | 192.168.0.15 |
   | `3c:6d:66:76:88:57`（Orin 有線） | 192.168.0.14 |
   | `30:76:F5:7D:DD:44`（ESP32） | 192.168.0.16 |

3. 儲存。裝置要重新取得 IP 才會生效（重開機或重新連線）。
4. 在 Windows 確認：`Test-NetConnection 192.168.0.15 -Port 22`、`Test-NetConnection 192.168.0.16 -Port 80`。

不要改 Orin 本身的網路設定（不要改成靜態 IP），讓路由器負責綁定。

## 3. NetBird 現況

Orin 上（2026-10-14 唯讀檢查 `netbird status`）：Daemon 0.79.0、Management / Signal 都已連線、FQDN `112ab-jno.netbird.cloud`、NetBird IP 100.80.15.177、**Peers 0/3 connected**。

- Orin 在**組員的 NetBird 帳號**下。Ray 的電腦還沒加入，要請組員從 NetBird 管理頁面邀請 Ray（或給一把 setup key），Ray 在自己電腦安裝 NetBird 並登入後，才能在不同網路下用 `112ab-jno.netbird.cloud` 連 Orin。
- 這次沒有改任何 NetBird 設定。

## 4. Foxglove（建議的視覺化方式）

`wro2026_real` 的 bringup 加了 `foxglove:=true`：

```bash
ros2 launch wro2026_real bringup.launch.py transport:=http host:=192.168.0.16 max_speed:=0.0 foxglove:=true
```

- 需要 `ros-jazzy-foxglove-bridge`（**目前沒裝**，啟動時會顯示略過；見 2026-10-13 R550 準備報告的 B2）。
- 連線：Foxglove（桌面版或網頁版）→ Open connection → Foxglove WebSocket → `ws://192.168.0.15:8765`。
- **只能讀取**：`capabilities` 只開 `connectionGraph`、`assets`，沒有 `clientPublish`、`parameters`、`services`，另外 `client_topic_whitelist` 設成不符合任何 topic。所以從 Foxglove 介面**不能**發 `/cmd_vel`、不能改 `max_speed` 之類的參數、不能呼叫 service。設定在 `src/wro2026_real/wro2026_real/foxglove_config.py`，有 pytest 檢查。參數名稱依 ROS 2 版 foxglove_bridge 的文件（[foxglove-sdk](https://github.com/foxglove/foxglove-sdk) 的 `ros/src/foxglove_bridge/README.md`）。
- 2026-10-14 在 Orin 用上面的指令啟動：driver 與 robot_state_publisher 正常、`/cmd_vel` Publisher 0、foxglove_bridge 因未安裝而略過、8765 沒有在聽；關閉時 `shutdown stop sent: ok`。

## 5. WSL 看 Orin 的 topic

2026-10-14 測試：Orin 上 driver 在跑時，在 WSL 執行

```bash
wsl -e bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic list"
```

結果只有 `/parameter_events`、`/rosout`（WSL 自己的），**看不到 Orin 的 topic**。沒有改任何 WSL 設定。可能的原因：

1. **WSL 是 NAT 模式**：`%USERPROFILE%\.wslconfig` 目前是 `networkingMode=nat`，WSL 的 IP 是 172.21.173.167，和 Orin 不在同一個網段；DDS 用 multicast 找其他節點，過不了 NAT，Orin 也連不回 WSL 的 UDP 埠。（`docs/sim/ROS2_WSL_NETWORK.md` 寫的是 `mirrored`，和現在的設定不一致，那份文件可能是舊的。）
2. WSL 的互動 shell 可能套用了 `ros2_wsl_local` 設定（CycloneDDS 只用 loopback），見 `docs/sim/ROS2_WSL_NETWORK.md`；這次用的是非互動 shell，環境是 `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`、`ROS_DOMAIN_ID` 未設定（= 0），和 Orin 相同，所以不是 domain 不同造成的。
3. 就算改成 mirrored 模式，Windows / Hyper-V 防火牆與 multicast 轉送也可能擋住 DDS。

建議：**用 Foxglove 看資料**（只需要 TCP 8765，經過 NAT 或 NetBird 都沒問題，而且是唯讀的）。真的需要 WSL 的 RViz 時，可以考慮：改成 mirrored 模式、用 `ROS_STATIC_PEERS=192.168.0.15`、或在 Orin 跑 Fast DDS discovery server；這些都會改 WSL / 網路設定，要 Ray 自己決定。

## 6. `ROS_DOMAIN_ID` 建議

- 現況：Orin 和 WSL 都沒設定（= 0），`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`。
- 建議：在學校或共用網路上，其他人的 ROS 2 機器也可能用 0，會互相看到 topic，甚至收到別人的 `/cmd_vel`。選一個全隊共用、不是 0 的 ID（要在 0–101 之間，避免 Linux 上的埠號衝突），寫進 Orin 和所有要看車子資料的電腦的 `~/.bashrc`：

```bash
export ROS_DOMAIN_ID=<全隊共用的數字>
```

- 這次**沒有**改 Orin 的 `~/.bashrc`：換 domain 會讓還沒換的電腦看不到 Orin，要全隊一起換。Foxglove 不受影響（bridge 跑在 Orin 上，和 driver 同一個 domain 即可）。
