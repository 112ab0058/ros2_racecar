# 2026-10-13 文件整理 + WAVE ROVER 實車底層：結果報告

分支：`real/wave-rover-base`（從 `origin/real/h0-hardware-inventory` 開出）。執行環境：Windows `%USERPROFILE%\ros2_racecar`（編輯 / commit / push）、Jetson `user2@192.168.0.15` 的 `~/racecar_ws`（build / test / 只讀實機驗證）、WSL（只讀檢查與暫存驗證）。
全程沒有讓輪子轉：送給 ESP32 的只有 `{"T":1,"L":0,"R":0}`、`{"T":130}`、`{"T":126}`。沒有用 sudo、沒有 apt install、沒有改網路 / NetBird / 電源模式。

## 1. 各 Phase 狀態

| 項目 | 狀態 | 數據 |
|---|---|---|
| P0.1 clone、開分支、LF | DONE | `git clone -c core.autocrlf=false`；`core.autocrlf=false`；新分支的 upstream 原本被自動設成 h0 分支，已取消，改 push 到 `origin/real/wave-rover-base` |
| P0.2 WSL 唯讀檢查 | DONE | 見第 2 節；WSL 有 Jazzy + Gazebo |
| P0.3 讀文件 | DONE | README、CLAUDE、AGENTS、docs 全部、handoff、SKILL |
| P0.4 `line_follower.py` SHA | DONE | `de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2`；4315 行、279 個模組層級常數（自己數過，與描述一致） |
| P1 文件搬移 | DONE | 全部 `git mv`（19 個檔案 rename）；封存檔只加一行標記 + 空行，`tail -n +3` 與原檔 byte 相同；handoff 的 7 個 .md 去掉標記後仍符合 `SHA256SUMS.txt` |
| P1 STATUS / 索引 / README | DONE | `docs/STATUS.md` 55 行；`docs/README.md` 索引；README 最上方加 Project Status（英文） |
| P1 AGENTS / CLAUDE / SKILL | DONE | 見 commit `60211a8` |
| P1 連結檢查腳本 | DONE | `tools/check_md_links.py`：最後 30 個 md、89 個引用、0 WARN、0 ERROR |
| P1 `map_server.yaml` | DONE | launch 參數 `map`（預設 `get_package_share_directory('wro2026_sim')/maps/candidates/phase7_nav2_field_map_7dr_candidate.yaml`）；WSL 暫存驗證通過，沒有啟動 Gazebo |
| P1 `fix_lines.py` | DONE | `--sdf`，預設 repo 相對路徑；`--help` exit 0；SDF SHA 前後相同 `232ac111…` |
| P2.1 底盤 package | DONE | `src/wro2026_base_waverover`；先 commit 原樣搬入（`05efd25`）再重構（`4fd576a`） |
| P2.2 odometry | DONE | `/odom` + TF，`pwm_to_mps` 預設 0，啟動 log 有警告 |
| P2.3 測試 | DONE | Windows 46 + 16 passed（各 2 skipped）；Jetson `colcon test` 66 項 0 失敗（見第 5 節） |
| P2.4 `wro2026_real` | DONE | URDF、bringup、line detector（numpy-only） |
| P2.5 實機只讀驗證 | DONE | 見第 4 節；有一個流程失誤（SIGINT 送錯 PID），無安全影響 |
| P3 實車文件 | DONE | RUNBOOK、REAL_CAR_HARDWARE、STATUS、README、SYSTEM_ARCHITECTURE、CLAUDE.md |
| P4.1 檢查清單 | DONE | 見第 5 節 |
| P4.2 獨立審查 | DONE | 20 項發現（無違反規則），17 項已修正、3 項說明不改，修正後重新測試並在實機只讀驗證，見第 7 節 |
| P4.3 draft PR | BLOCKED | Windows 沒有安裝 `gh`，依 prompt 條件不開 PR；分支已 push，見第 11 節 |
| P4.4 報告 | DONE | 本檔 |

## 2. WSL repo 檢查結果（只讀，沒有動）

```text
$ wsl -e bash -lc "cd ~/ros2_racecar && git status --short --branch && git log --oneline -5 && git log origin/main..HEAD --oneline"
## main...origin/main
?? docs/HARDWARE_INVENTORY.md
c9061f5 Add CLAUDE.md and 2026-10-09 handoff package
6f77c24 Add Phase 7 nav2/SLAM config, launch files, maps and world
f316b08 Update line_follower to accepted controller (sha256 de79e41c…)
06725db docs: sync Phase 7 spec, state and evidence docs
a0120ed Add AI project sync handoff docs
(origin/main..HEAD 空)
```

**請你先處理：**

1. WSL repo 裡的未追蹤檔案 docs/HARDWARE_INVENTORY.md（3676 bytes，2026-10-09 18:21，H0 草稿，內容已被 `docs/real/REAL_CAR_HARDWARE.md` 取代）。
2. `stash@{0}: On main: backup before 160cm update`。
3. 本機分支 `backup/local-main-20261009` 有 2 個 GitHub 上沒有的 commit：`c7938cd Tune return-to-line exit for one-obstacle test`、`bd189d0 Tune startup and cornering for 160cm field`（訊息和 main 上的 `5f65dbc`、`02983ad` 相同，SHA 不同，可能是 rebase 前的版本）。
4. WSL 的 `main` 和 GitHub `main` 同一個 commit（`c9061f5`）；沒有未 commit 的修改。

環境：Ubuntu 24.04.4、`/opt/ros/jazzy`、`/usr/bin/gz`、`ros_gz`、`ros_gz_bridge`、`nav2_map_server`、`robot_state_publisher` 都在。

## 3. 判斷和原因

文件整理

1. 封存標記 = 指定的那一行 + 一個空行，讓內文從第 3 行開始、可以用 `tail -n +3` 驗證和原檔 byte 相同。原本就在 `docs/archive/` 的 `NEXT_PROMPT_phase7g.md` 也加了，保持一致。
2. handoff 的 `SHA256SUMS.txt` 不改：它記錄原始檔案；去掉標記後仍相符（已驗證 7/7）。
3. repo 裡原本沒有任何 `[文字](路徑)` 形式的連結，引用都是反引號路徑，所以連結檢查腳本同時檢查 markdown 連結和反引號裡的文件路徑（`docs/`、`.agents/`、`*.md`）。`docs/archive/` 的內文凍結、本來就指向搬移前的路徑，所以略過（報告略過數量）；`src/`、`tools/` 路徑只給 WARN。
4. `docs/sim/SPEC.md` 第 10 節只改路徑字串，規則內容不動；`WORKSPACE_MAP.md` 只改 docs 路徑，`/home/ray` 保留。
5. `map_server.yaml` 的 `yaml_filename` 改成註解，由 launch 以 `{"yaml_filename": LaunchConfiguration("map")}` 覆寫（grep 確認只有 `phase7_saved_map_localization.launch.py` 用它）。
6. AGENTS.md 只保留路由規則和原本 4 條模擬規則（路徑更新）。CLAUDE.md 除了路徑，在 Phase 3 也更新了「Package 規劃」三行（`wro2026_real` 已建立、新增 `wro2026_base_waverover`、`~/racecar_ws`），因為原文「待建立 / 之後要搬進 src」已經不正確；安全規則一字未改。
7. `SYSTEM_ARCHITECTURE.md` 的實車內容等 Phase 2 程式存在後才寫（Phase 3），避免描述不存在的東西；原本 2026-07-05 的規劃全部保留。
8. README 維持英文；舊的 Next Work 保留並標為歷史。

實車程式

9. 第一個 Phase 2 commit 是 Jetson driver 的原樣複製（只改名），重構放在下一個 commit，方便對照已驗證的版本。node 名稱維持 `wave_rover_driver`。
10. 零速一律編成整數 0，停止指令就是字面上的 `{"T":1,"L":0,"R":0}`（舊版送 `0.0`）。
11. 回授監看：收到第一筆 T:130 回應之前也拒絕非零指令；遺失後要等回應恢復才放行。`feedback_timeout:=0` 可關閉（`poll_base:=false` 時必須關，否則永遠不能動，啟動時會警告）。
12. odom：送出的指令為零時位置和角度都不動（`odom_hold_when_stopped`），避免陀螺儀雜訊讓 odom 漂移；IMU 超過 0.5 s 沒更新時角速度當 0；IMU z 軸方向未知，加 `imu_yaw_rate_sign` 參數。
13. `/cmd_vel` 的語意維持舊 driver：`linear.x`、`angular.z` 直接當 PWM 比例換算（「功能和參數完全保留」）。
14. 新、舊 driver 同時啟動的防護：新 driver 和 `test_motion` 啟動時掃 `/proc`，找到舊的或另一個新的 driver 就拒絕啟動。
15. Jetson 沒有 OpenCV / cv_bridge，line detector 核心只用 numpy（HSV、開閉運算、8-連通最大色塊都自己寫），節點自己轉 rgb8/bgr8/rgba8/bgra8；在 Windows 用 OpenCV 交叉比對（HSV 差 ≤ 1、形態學完全相同、偵測結果與模擬版演算法一致）。
16. URDF 的根是 `base_link`（driver 發佈 `odom → base_link`），沒有加 `base_footprint`，避免 `base_link` 有兩個 parent。
17. `wro2026_real` 的 bringup 不包含 line detector（另一個 launch），也不發佈 `/cmd_vel`。
18. Windows 的系統 Python 沒有 pytest：在 session 暫存區用 uv 建一個獨立 venv（Python 3.12、numpy 1.26.4 與 Jetson 相同、pytest、flake8、pydocstyle、OpenCV 4.10 只供交叉比對），沒有動系統 Python。numpy 2.5 + OpenCV 4.14/5.0 會讓 `cv2.erode` 丟 "Unknown exception"，所以釘在 numpy 1.26.4。
19. ament_pep257 要求多行 docstring 的摘要從第二行開始（D213），ament_flake8 要求 google 風格 import 排序；從 Jetson 搬來的 `test_motion.py`、`check_serial.py` 也只做了換行與 import 排序（行為不變）。
20. maintainer 沿用 `wro2026_sim` 的 `WRO Team <team@wro2026.com>`，不放個人信箱。

實機驗證

21. 2.5 的 bringup 加 `max_speed:=0.0` 當保險：就算有東西發佈 `/cmd_vel`，也只會送出停止。
22. 新 package 的 `test_motion`（即使 `--startup-only`）沒有在實機上跑：prompt 沒要求，保守起見留給你明天依 RUNBOOK 跑。

## 4. 實機只讀驗證（Phase 2.5）

Jetson log：`~/wave_rover_logs/verify_2_5_20261013_022804/`。指令：

```bash
ros2 launch wro2026_real bringup.launch.py transport:=http host:=192.168.0.16 max_speed:=0.0
```

| 檢查 | 結果 |
|---|---|
| 啟動前 | 沒有任何 driver 在跑；`/cmd_vel` 不存在；T:130 `{"T":1001,"L":0,"R":0,...,"v":12.1167}` |
| node | `/robot_state_publisher`、`/wave_rover_driver` |
| `/cmd_vel` | 啟動中和結束前都是 Publisher count 0、Subscription count 1 |
| 剛啟動只送零速 | `/wave_rover/sent_cmd` 170 筆，170 筆都是 `{"T":1,"L":0,"R":0}`、`gate` 全是 `idle`、`ok` 全是 true（約 1.7 Hz） |
| `/battery` | 4.805 Hz，12.1167 V，frame `base_link` |
| `/imu/data` | 4.772 Hz，frame `imu_link` |
| `/diagnostics` | 1.000 Hz，level OK，message `ok`，feedback_age 0.15 s |
| `/odom` | 20.000 Hz；t0 與 t0 + 33.7 s 兩筆都是位置 (0, 0, 0)、orientation (0, 0, 0, 1)；pose covariance[0] = 100 |
| `/wave_rover/feedback` | 414 筆 T:1001 + 414 筆 T:1002，沒有其他回覆 |
| TF | `odom → base_link` identity；`base_link → imu_link` identity；`base_link → camera_link` (0.1, 0, 0.1)；`odom → camera_link` 可查 |
| 關閉 | 對 `ros2 launch` 送 SIGINT：`[driver-1] shutdown stop sent: ok`、兩個 process `finished cleanly`、沒有殘留；之後直接讀 T:130 為 `L=0, R=0`、12.1166 V |

流程失誤（無安全影響）：驗證腳本把 SIGINT 送給包在外面的 bash subshell（背景函式），`ros2 launch` 沒收到，所以 driver 多跑了約 40 秒（全程只送停止、`max_speed=0.0`、沒有 `/cmd_vel` publisher）。之後直接對 `ros2 launch` 的 PID 送 SIGINT，正常關閉。這個陷阱已寫進 RUNBOOK 第 2 節；Jetson 上的臨時腳本已刪除。

另外直接讀 30 筆 T:126（靜止）：gz 平均 0.048 deg/s、標準差 5.122、範圍 -14.712～+13.913；gx 標準差 0.177、gy 0.214；韌體 yaw 約 3 秒內 -24.774 → -24.617。gz 雜訊大，已記進 REAL_CAR_HARDWARE.md。

舊 driver：`~/ros2_ws/src/wave_rover_driver` `git status` 乾淨、HEAD `543a4fc`，沒有修改，整個驗證期間沒有啟動。

### 審查修正後的第二次只讀驗證（2026-10-13 10:46，+08:00）

因為改了收送指令的排程（第 7 節 B2），用同樣的條件（`max_speed:=0.0`、沒有 `/cmd_vel` publisher）再跑一次，log 在 Jetson `~/wave_rover_logs/verify_review_fix_20261013_104651/`：

| 檢查 | 結果 |
|---|---|
| `/cmd_vel` | Publisher 0、Subscription 1 |
| 頻率 | `/battery` 4.884 Hz、`/imu/data` 4.803 Hz、`/odom` 19.997 Hz；`/diagnostics` level OK、message ok |
| `/odom` | 位置 (0, 0, 0) |
| 送出的指令 | 62 筆，全部 `{"T":1,"L":0,"R":0}` |
| 關閉 | 這次 SIGINT 直接送到 `ros2 launch`：`shutdown stop sent: ok`、兩個 process `finished cleanly`、沒有殘留；之後 T:130 為 `L=0, R=0` |
| **電池** | **11.08 V**（凌晨 02:28 是 12.12 V，車子整晚開著）。已接近 11.0 V 警告門檻，請先充電再做任何測試 |

## 5. 檢查清單（指令輸出）

```text
$ sha256sum src/wro2026_sim/scripts/line_follower.py
de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2   (與 Phase 0 相同)
$ git diff --stat origin/main...HEAD -- src/wro2026_sim/scripts/
(空，0 行)
$ git grep -l /home/ray
docs/archive/handoff_2026-10-09/{KEEP_ARCHIVE_DELETE,KNOWN_PATHS_AND_EVIDENCE,OPEN_QUESTIONS,PROJECT_HANDOFF,README,START_HERE_NEXT_AI}.md
docs/archive/sim_phases/{CURRENT_STATE,NEXT_PROMPT_phase7g,TEST_EVIDENCE}.md
docs/sim/ROS2_WSL_NETWORK.md
docs/sim/WORKSPACE_MAP.md
src/wro2026_sim/config/phase7/localization_validation_route.yaml
(map_server.yaml 已改成功，不在清單內)
$ git ls-files --eol
crlf in index: 0  crlf in worktree: 0  mixed: 0（其餘為 i/lf、4 個空檔 i/none、2 個 .pgm binary）
$ python3 tools/check_md_links.py
31 markdown files, 107 references checked, 25 inline references in docs/archive/ skipped (frozen), 0 WARN, 0 ERROR
$ (Windows) python -m pytest   # Python 3.12.13, numpy 1.26.4, OpenCV 4.10, pytest 9.1.1
wro2026_base_waverover: 46 passed, 2 skipped (ament flake8/pep257 不在 Windows)
wro2026_real:           16 passed, 2 skipped
$ (Jetson, 乾淨 build) colcon build && colcon test && colcon test-result --all
build/wro2026_base_waverover/pytest.xml: 48 tests, 0 errors, 0 failures, 0 skipped
build/wro2026_real/pytest.xml: 18 tests, 0 errors, 0 failures, 3 skipped (OpenCV 交叉比對，Jetson 沒有 OpenCV)
Summary: 66 tests, 0 errors, 0 failures, 3 skipped
（以上是審查修正後的數字；修正前是 41 passed 與 61 tests，同樣 0 失敗）
```

## 6. Git

最後這個文件 commit 之前的狀態（這個 commit 本身再多一個）：

```text
$ git log --oneline origin/main..HEAD
c0bcf2a real: fix independent-review findings in the base driver
8ca051c docs(report): fix link-check error for WSL-only file
8672fc2 docs: report for 2026-10-13 cleanup and WAVE ROVER base
123dc7e docs(real): RUNBOOK, verified bringup, status/architecture update
e85c83f real: add wro2026_real (bringup, URDF, parameterised line detector)
c039545 real: hardware-free tests for wro2026_base_waverover
4fd576a real: split WAVE ROVER driver into core / transport / node; add odom
05efd25 real: import Jetson wave_rover_driver as wro2026_base_waverover
60211a8 docs: reorganize into STATUS/real/sim/archive; remove hard-coded paths
958f004 docs: H0 hardware inventory for WAVE ROVER + Orin; fix stale Humble note
$ git diff --stat origin/main...HEAD | tail -1
 74 files changed, 4401 insertions(+), 52 deletions(-)
```

`958f004` 是上一個分支 `real/h0-hardware-inventory` 的 commit（還沒進 main）。

## 7. 獨立審查

由 subagent 執行，輸入只有 `git diff -M origin/main...HEAD` 和這份任務 prompt。結論：**沒有違反規則的地方**；找到 8 個程式問題（B）和 12 個文件問題（C）。

| # | 問題 | 嚴重度 | 處理 |
|---|---|---|---|
| B1 | `/cmd_vel` 收到 NaN 時，`clamp()` 會變成 +max_speed（全速前進） | 中 | **已修**：NaN / inf 一律當停止，`set_cmd_vel` 回傳 False，node log warn；加測試（舊程式會失敗） |
| B2 | HTTP 模式下指令一直變動時，輪速指令永遠優先，T:130 輪詢被擠掉 → 2 s 後判定回授遺失 → 停停走走 | 中 | **已修**：送完一次輪速後，若有到期的輪詢就先輪詢一次；新的停止指令永遠立刻送。加測試（模擬每個請求 0.09 s、指令每圈都變；舊程式會失敗），並在實機重新只讀驗證 |
| B3 | `test_motion --startup-only` 仍用 `max_speed` 0.05 啟動 driver，若有別的程式發 `/cmd_vel` 仍會動 | 低～中 | **已修**：`--startup-only` 時 driver 的 `max_speed` 設 0.0 |
| B4 | UART 讀取一直出錯時（拔線），讀取迴圈沒有 sleep，CPU 100 % | 低 | **已修**：出錯時 sleep 0.1 s，計數 `read_errors` |
| B5 | 送失敗的停止指令要等 `cmd_period`（HTTP 0.5 s）才重送 | 低 | **已修**：送失敗後 0.1 s 重送；加測試。odom 在送失敗時仍用「最後成功送出的指令」，不改（實際狀態未知，ESP32 3 s 會自停，odom 本來就是低品質） |
| B6 | `map_server.yaml` 拿掉 `yaml_filename` 可能影響其他 launch；預設地圖是否有安裝 | 低（未確認） | **不改**：Phase 1 已用 grep 確認只有一個 launch 使用；WSL 驗證用 `evaluate_parameters` 解析出路徑且 `os.path.isfile` 為 True（第 4 節之前的 Phase 1 紀錄） |
| B7 | 新舊 driver 互斥只在新 driver 這一側；舊執行檔名稱未確認 | 低 | **文件已補**：RUNBOOK、package README、STATUS 寫明「先開新的再開舊的擋不住」。舊執行檔名已由舊 `setup.py`（`driver = wave_rover_driver.driver_node:main`）確認 |
| B8 | bringup 說「每個參數都能覆寫」，但少了 4 個 odom covariance | 低 | **已修**：補上 |
| C1 | `docs/sim/SPEC.md` 第 10 節仍要新對話讀封存的 CURRENT_STATE、過期的 NEXT_PROMPT，與 AGENTS.md 矛盾 | 中 | **已修**：改成 AGENTS.md → STATUS.md → SPEC.md，封存文件只在需要背景時讀，NEXT_PROMPT 標為過期；保留一行說明原清單 |
| C2 | handoff 的 `SHA256SUMS.txt` 與加了封存標記的檔案不符 | 低 | **已補說明**：STATUS 已知問題寫明去掉前兩行後相符 |
| C3 | AGENTS.md 寫「原文保留」，實際改了路徑 | 低 | **已修**：「原規則保留，路徑已更新」 |
| C4 | RUNBOOK 寫停止送 `/cmd_vel`「0.3 s 內停」，新 driver HTTP 下可能到約 0.6 s，且未實測 | 低 | **已修** |
| C5 | RUNBOOK 沒有檢查 `teleop_twist_keyboard`、`python3-serial` | 低 | **已修**：加檢查指令；兩者在 Jetson 上都有（2026-10-13 已確認），不需要安裝 |
| C6 | `/cmd_vel` 當 PWM 比例、`/odom` 是 m/s，上層必須知道底盤單位 | 低 | **記為已知問題**（STATUS）。程式不改：prompt 要求功能和參數完全保留；`pwm_to_mps` 量好後再改成 m/s 換算 |
| C7 | 硬體文件寫「2026-10-13 晚」，實際是 02:28 | 低 | **已修**：改成 02:28（+08:00） |
| C8 | CLAUDE.md 用 `user2@112ab-jno`，RUNBOOK 用 IP | 低 | **不改**：該行來自上一個分支的 commit；主機名稱和 IP 都是有效的連線方式（IP 是 DHCP 會變） |
| C9 | STATUS 寫「61 項 0 失敗」，其實含 3 項略過 | 低 | **已修**：STATUS 改連到本報告的數字 |
| C10 | `docs/README.md` 沒有 reports 索引 | 低 | 報告 commit 時已加 |
| C11 | real README 說 RSP 沒裝會略過，但 package.xml 宣告為依賴 | 低 | **已修**：說明兩者並存的原因 |
| C12 | RUNBOOK「兩邊都是 3.3 V 邏輯」在 repo 裡沒有來源，接錯可能燒壞 Orin | 低（未確認） | **已修**：註明來源只有前一輪 Jetson 報告，接線前先用電表量車頂 TX 對 GND ≤ 3.3 V |

## 8. 需要你裝的 apt 套件

目前的功能**不需要**再裝任何套件（`robot_state_publisher`、`diagnostic_msgs`、`tf2_ros_py`、`tf2_tools`、`python3-serial`、numpy、pytest、ament_flake8/pep257、`teleop_twist_keyboard` 都已經在 Jetson 上）。之後視需要：

| 套件 | 原因 | 何時需要 |
|---|---|---|
| 相機 driver，例如 `ros-jazzy-v4l2-camera` 或 `ros-jazzy-usb-cam` | Jetson 沒有任何相機 package，也沒有 `/dev/video*`；line detector 需要影像 topic | 相機型號確定後（型號 UNKNOWN，先不要猜） |
| `ros-jazzy-cv-bridge`、`python3-opencv` | 相機如果只輸出 yuv422 / mono8 / compressed，line detector 需要它們轉換；rgb8/bgr8 不需要 | 視相機輸出格式 |

## 8.1 其他需要的工具（Windows）

| 工具 | 原因 |
|---|---|
| GitHub CLI `gh`（並 `gh auth login`） | 沒有安裝，所以這次沒有開 draft PR |

## 9. 需要你親自確認或測試的項目

0. **電池充電**：10:46 量到 11.08 V（凌晨 12.12 V），接近 11.0 V 警告門檻；低於 10.5 V 時 driver 會拒絕所有非零指令。
1. **先處理 WSL repo**（第 2 節的未追蹤檔、stash、本機分支）。
2. **輪子目視測試**：新 driver 還沒讓輪子轉過。架空後照 `docs/real/RUNBOOK.md` 第 3 節跑 `test_motion`，確認四輪方向和停止。
3. 明天 UART 接線：照第 10 節。接線前再對一次 Waveshare 腳位圖、用電表量車頂 TX 電壓（≤ 3.3 V），不要接 5V。
4. 量測：輪距、輪徑、`pwm_to_mps`、IMU z 軸方向（RUNBOOK 第 5 節），填進 `config/wave_rover.yaml`。
5. 確認我做的兩個安全取捨是否符合你的預期：收到第一筆 T:130 前拒絕非零指令；回授遺失後要等回授恢復才放行。
6. IMU gz 靜止雜訊大（標準差 5.1 deg/s）：要不要改用韌體 yaw（T:126 的 `y`）做 odom 的角度。
7. 路由器替 ESP32（MAC `30:76:F5:7D:DD:44`）設定固定 IP，避免 `192.168.0.16` 變動。
8. 看過 PR diff 後自己開 draft PR（第 11 節），或裝 `gh` 後由我開。

## 10. 明天接完 UART 要跑的指令（Jetson，照順序）

接線前 Orin 和車子都關電源：Orin pin 8（TX）→ 車頂 RX、pin 10（RX）← 車頂 TX、pin 6 GND ↔ GND，不接 5V。完整說明在 `docs/real/RUNBOOK.md` 第 4 節。任何一步失敗就停。

```bash
source /opt/ros/jazzy/setup.bash && source ~/racecar_ws/install/setup.bash
pgrep -af 'lib/wave_rover_driver/driver|lib/wro2026_base_waverover/driver' || echo "沒有 driver 在跑"
```

1. UART 唯讀檢查（要 0 FAIL）：

```bash
ros2 run wro2026_base_waverover check_serial
```

2. 不動輪子的 serial 測試：

```bash
ros2 run wro2026_base_waverover test_motion --transport serial --port /dev/ttyTHS1 --startup-only
```

3. 確認 B1 的 `t130_count` > 0；是 0 就檢查 TX/RX，不要往下做。
4. 不動輪子的 bringup（另開終端機看 `ros2 topic echo --once /diagnostics` 要 OK、`ros2 topic hz /battery` 有資料），Ctrl-C 結束：

```bash
ros2 launch wro2026_real bringup.launch.py max_speed:=0.0
```

5. ⚠️ 架空低速動作測試（目視四輪方向、每次都停住）：

```bash
ros2 run wro2026_base_waverover test_motion --transport serial --port /dev/ttyTHS1 --speed 0.05
```

6. ⚠️ 架空遙控（兩個終端機；`k` 或 Ctrl-C 停）：

```bash
ros2 launch wro2026_real bringup.launch.py max_speed:=0.05 cmd_timeout:=0.5
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 11. PR

沒有開：Windows 上沒有 `gh`（`Get-Command gh` 找不到），prompt 的條件是「有 `gh` 而且已登入才開」。分支已 push，可以從這裡開 draft PR（base `main`，不要 merge）：

https://github.com/112ab0058/ros2_racecar/compare/main...real/wave-rover-base?expand=1

建議標題：`Docs reorganization + WAVE ROVER base driver (wro2026_base_waverover, wro2026_real)`；內文可直接貼本報告第 1 節和第 7 節。
