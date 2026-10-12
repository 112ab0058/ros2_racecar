> 已封存（2026-10-13）。目前狀態見 docs/STATUS.md。

# Test Evidence

Last updated: 2026-07-22

## Build Evidence

Command environment:

```text
which python3
/usr/bin/python3

python3 --version
Python 3.12.3
```

Build:

```text
colcon build --symlink-install --packages-select wro2026_sim
Starting >>> wro2026_sim
Finished <<< wro2026_sim [0.31s]
Summary: 1 package finished [0.54s]
```

## Topic Probe

```text
topics=/camera/camera_info /camera/image_raw /clock /cmd_vel /line_detection /odom /parameter_events /rosout /scan /tf
clock=(20, 350000000)
scan_count=51 scan_hz=6.59
camera_count=61 camera_hz=8.65
cmd_vel=(0.035, 0.2)
odom_count=223 pose=(0.44999375144088943, 0.41583963082677994, 2.160820451481072)
```

## Verified No-Obstacle Regression

Before this run, runtime world was checked:

```text
NO_OBSTACLE_WORLD_CHECK
obstacle_yellow_mid pose = 0.00 0.00 0.075 0 0 0
is_obstacle_active = no for B-point obstacle
```

Summary:

```text
mode=no_obstacle
timed_out=True
sample_count=515
top_x_min=-0.530 top_x_max=+0.428
top_y_min=+0.350 top_y_max=+0.499
top_yaw_min=-3.141 top_yaw_max=+3.141
top_avg_x=+0.090
y42_pose=x=-0.530 y=+0.421 yaw=+1.584
avoid_start=x=+0.135 y=+0.383 yaw=+3.141 front=0.1991715133190155 dx=+0.135 dy=-0.037 dist=0.140
avoid_start_count=1
min_front=0.11999999731779099
left_upper_pose=None
bottom_pose=None
recover=True
recover_failed=False
```

This confirms the clean no-obstacle run still false-triggered.

## Previous No-Obstacle Regression

Runtime-only setup:

- `obstacle_yellow_mid` temporarily moved to center holding position.
- Formal repo world was not changed.
- Runtime world was restored to B position afterward with `rsync`.

Summary:

```text
mode=no_obstacle
timed_out=True
sample_count=520
top_x_min=-0.531 top_x_max=+0.425
top_y_min=+0.350 top_y_max=+0.499
top_yaw_min=-3.142 top_yaw_max=+3.141
top_avg_x=+0.091
y42_pose=x=-0.531 y=+0.420 yaw=+1.563
avoid_start=x=+0.117 y=+0.395 yaw=+3.141 front=0.1956753134727478 dx=+0.117 dy=-0.025 dist=0.120
avoid_start_count=1
min_front=0.11999999731779099
left_upper_pose=None
bottom_pose=None
recover=True
recover_failed=False
```

Key cluster debug at false trigger:

```text
AVOID cluster_check front=0.962 front_min=0.948 LF=0.256 RF=0.526 LS=0.403 RS=0.387
near_front=False diag_near=True
cluster_count=3 cluster_min=0.196 cluster_center=58.269 cluster_width=30.068 cluster_count_points=30
cluster_obstacle=True side_clear=True obstacle_asymmetry=True narrow_geometry=False lockout=False trigger=True
x=+0.117 y=+0.395 yaw=+3.141
```

Conclusion:

Partial. The no-obstacle regression failed because the conservative `0.215 m` cluster min-range gate is still too permissive for normal upper-field geometry.

## Runtime Restore

Runtime world restored to:

```text
obstacle_yellow_mid <pose>0.00 0.42 0.075 0 0 0</pose>
```

## Phase 3P — C500 Post-Recover-Hold Validation

Status: **Done**

- Production controller SHA-256：`b897f792b4e287e8ef8f9b4fa44db80b562402d588b3887ff0c66816cd5983cc`
- Final build exit：0；formal/workspace/installed `cmp=0`；`git diff --check=0`。
- A0：12/12 valid，4 次 `D2_PASS_B`。
- C500：12/12 cap-specific，12/12 `SAFE_FIRST_RECOVER`。
- C500 active upper-RECOVER forward command count：0。
- C500 PASS 最低 full-body clearance：`41.371 mm`。
- C500 RECOVER-entry 最低 clearance：`41.371 mm`。
- C500 stop/reverse/reverse-hold minima：`40.367 / 40.384 / 80.690 mm`。
- Production no-obstacle regression：PASS，抵達 `no_obstacle_bottom_pose`。
- Production B smoke：PASS；PASS/entry `72.828 / 71.711 mm`，stop/reverse/hold `71.651 / 72.130 / 117.859 mm`，forward=0。

Evidence root：

```text
/home/ray/ros2_evidence/phase3p_c500_post_recover_hold_validation_20260715_171210
```

Read-only archive：

```text
/home/ray/ros2_evidence/phase3p_c500_post_recover_hold_validation_20260715_171210.tar.gz
SHA-256: d1b89185ce40d13d384065a8e3517b5b2758b8ff0c718af4abc37e0990a0bbd0
mode: 0444
independent extraction verification: 970/970 manifest records, 0 failures
```

## Phase 4M — REVERSE_EAST_STAGE Level 1 Screening

Status: **Blocked**

- Production controller SHA-256：`b897f792b4e287e8ef8f9b4fa44db80b562402d588b3887ff0c66816cd5983cc`
- 三項 preflight：全部通過；A0/candidate no-obstacle activation 都為 0，A0 B-short 等價窗口通過。
- Campaign：7 個有效 slots；2 個 protocol/setup-failure attempts 保存後只重跑原 slot；B08 在 candidate 立即拒絕後未執行。
- A0：B01 重現 `CLEARANCE_LT_5MM`，minimum clearance `4.203 mm`。
- Candidate：4/4 有效 route exposures，command、target、order、Segment 2 path source 均正確。
- B02/B03/B06 route durations：`5.240 / 5.350 / 5.510 s`，均小於 `5.681 s`。
- B07 `ALIGN_REVERSE_EAST` duration：`1.350 s`，超過 segment timeout `1.204 s`。
- B07 route duration：`5.840 s`，亦超過 route hard fail-safe `5.681 s`。
- B07 effective earliest terminal：`SEGMENT_TIMEOUT_RETROSPECTIVE`；exact candidate rejected。
- Candidate route minima：`170.467 / 182.188 / 107.401 / 172.076 mm`。
- Candidate handoff+1.5 s minima：`186.856 / 192.832 / 157.771 / 186.394 mm`。
- 無 candidate contact/<5 mm、RECOVER_FAILED、exception 或新 state loop；clearance safety 不覆蓋 timeout rejection。
- Temporary selector、route 與 diagnostics 已完整回退；formal/workspace/installed SHA 相同、`cmp=0`、syntax/build/`git diff --check` 通過，Git status 與開始前一致，無殘留 process。
- 本輪不保留 production 修改、不建立 archive。

Evidence root：

```text
/home/ray/ros2_evidence/phase4m_reverse_east_stage_screening_20260716_194804
```

## Phase 5C — Integrated Survivor Level 1 Validation

Status: **C1 INTEGRATED_SURVIVOR**

- Fixed campaign：`12/12` valid runs，required order preserved，normal valid reruns `0`。
- I1：`6/6` formal full-route successes；extension `6/6`。
- Integrated route duration：`5.225–5.375 s`，全部 `<=5.896 s`。
- Scan-confirm activation：`5/6`；clear indices `3 / 1 / 2 / 3 / 2`。
- I1 whole-route minimum clearance：`39.114927 mm`。
- Scan accounting、generic forward、RECOVER_FAILED、timeout、exception、state loop：全部 `0`。
- A0：`6/6` 重現 `<5 mm` baseline failure，最低 `4.186361 mm`。
- Production modification：未保留；controller 已回復當時 production SHA。

Evidence root：

```text
/home/ray/ros2_evidence/phase5c_integrated_survivor_validation_20260717_073653
```

## Phase 5D — Integrated Production Retention

Status: **D1 PRODUCTION_RETENTION_COMPLETE**

- Controller SHA before：`b897f792b4e287e8ef8f9b4fa44db80b562402d588b3887ff0c66816cd5983cc`
- Controller SHA after：`fbd871401ae0fb77250103f371321c9fbf1ded7a650792d26b07bd0a26a39e15`
- Syntax、`git diff --check`、package build：全部 exit `0`。
- formal/workspace/installed SHA 相同，兩組 `cmp=0`。
- No-obstacle production preflight：`2/2` valid，extension/confirmation activation 全部 `0`。
- B-short sanity：`B_SHORT_PRE5_BIT_EXACT`，route `5.250 s`，`5.000 s` 前 confirmation activation `0`，production selector 不存在。
- Production B regression：`8/8` valid formal full-route successes；normal valid reruns `0`。
- Extension completion：`8/8` odom path progress；duration `2.355–2.385 s`。
- ALIGN_DOWN duration：`2.875–3.020 s`。
- Route duration：`5.240–5.385 s`，全部 `<=5.896 s`。
- Scan-confirm activation：`7/8`；clear indices `1 / 3 / 1 / 3 / 3 / 3 / 1`，最大 fresh scan index `3`。
- Whole-route minimum clearance：`39.505547 mm`（`B03_PRODUCTION`）。
- Entry、duplicate、freshness、target、command、reset errors：全部 `0`。
- Generic forward、RECOVER_FAILED、timeout、exception、state loop：全部 `0`。
- 一次 PF1 setup failure 與三次 PF3 monitor protocol failures 均保存，修正 setup/monitor 後只重跑原 slot；controller 未修改。
- Phase 4Z guard 未保留；bottom timeout 仍為 `5.000 s`；front threshold 仍為 `0.45 m`。

Level 2 evidence root：

```text
/home/ray/ros2_evidence/phase5d_integrated_production_retention_20260717_132037
```

Manifest：`complete_sha256.txt` 全部通過，staging 與 persistent archive 一致；唯讀 tar archive 與獨立解壓驗證結果記錄於 Phase 5D evidence。

## Phase 5E — Final Acceptance Regression

Status: **E2 FINAL_ACCEPTANCE_REGRESSION**

- Production controller SHA：`fbd871401ae0fb77250103f371321c9fbf1ded7a650792d26b07bd0a26a39e15`；本 phase 未修改 controller。
- No-obstacle `2/2` 完成；Production B `G1_B01` 成功，`G1_B02` 是正常有效 regression，依 protocol 立即停止且未重跑。
- `G1_B02` extension activation `1`、odom path `0.112130685 m`、ALIGN_DOWN 完成、route bit-exact、route duration `5.245 s`。
- Confirmation 四個 fresh front：`0.414454 / 0.433182 / 0.438742 / 0.449266 m`；scan 4 仍低於 `0.45 m`，budget exhaustion 後進入 `RECOVER_FAILED`。
- 最低 clearance `40.614 mm`；contact／<10 mm、route timeout、generic forward、accounting error、exception、state loop 均為 `0`。
- Evidence：`/home/ray/ros2_evidence/phase5e_final_acceptance_20260717_142733`

## Phase 5F — G1_B02 Offline Root Cause

Status: **F3 SCAN_COUNT_PROGRESS_NOT_BOUNDED**（SPEC Level 0）

- Entry baseline 未計數；四個 scan identity `390/391/392/393` 與 stamp `50.750/50.875/51.000/51.125 s` 皆 strictly fresh 且各計一次。
- 實作順序正確：front update → clear check → budget exhaustion；scan 4 shortfall 僅 `0.733715 mm`，但仍是合法 non-clear。
- Fresh scan identity 不約束 pre-entry displacement、callback 間 motion 或 front geometry；fixed scan-count family 正式關閉，不能改試 count 5。
- Acceptance-blocked patch 只有 `BOTTOM_POST_TIMEOUT_SCAN_CONFIRM_4`；`RECOVER_EGRESS_EXTENSION` 仍有效。
- Evidence：`/home/ray/ros2_evidence/phase5f_g1_b02_offline_review_20260717_150446`

## Phase 5G — Scan-confirm Production Rollback

Status: **G1 PRODUCTION_SCAN_CONFIRM_ROLLBACK_COMPLETE**（SPEC Level 2）

- Controller SHA before：`fbd871401ae0fb77250103f371321c9fbf1ded7a650792d26b07bd0a26a39e15`
- Controller SHA after：`3866302660a95e92cece184b1e859a9bc73b8fe92888f388eb62b3b851c46a25`
- 精確移除 scan-confirm constant、6 個 fields、freshness／entry／tick／reset methods、scan callback identity tracking、confirmation routing 與 4-scan exhaustion；bottom timeout 恢復原本 `5.000 s` timeout-first failure flow，threshold 維持 `0.45 m`。
- `RECOVER_EGRESS_EXTENSION` methods rollback 前後 byte hash 全相同；path `0.107710342260001 m`、timeouts `2.730 / 3.651 / 5.896 s` 與 commands 均未變。
- Syntax、`git diff --check`、package build 全部 exit `0`；formal／workspace／installed 三份 SHA 相同且兩組 `cmp=0`。
- No-obstacle `2/2` 有效通過；extension／scan-confirm activation `0/0`，最低 clearance `62.279 mm`。
- B upper-route `4/4` 有效通過；extension、odom completion、ALIGN_DOWN、downward handoff、bottom pose 與 bottom RECOVER entry 均為 `4/4`。
- Route duration `5.250–5.375 s`；全組最低 clearance `38.665124 mm`（`B02_UPPER_ROUTE`）。
- Scan-confirm activation、generic forward、extension timeout、RECOVER_FAILED、exception、state loop 全部為 `0`。
- N01 前兩次 simulation 前 setup failures 已保存；只重跑同一 N01 slot，所有正常有效 run 均未重跑。
- Level 2 evidence：`/home/ray/ros2_evidence/phase5g_scan_confirm_production_rollback_20260717_152924`
- Manifest、staging/persistent、唯讀 tar 與獨立解壓驗證結果記錄於 Phase 5G evidence。

## Phase 5Q — Bottom Direct Reorient Odom Handoff Screening

Status: **Q1 CANDIDATE_SURVIVES_LEVEL1**（SPEC Level 1）

- Starting production SHA：`3866302660a95e92cece184b1e859a9bc73b8fe92888f388eb62b3b851c46a25`。
- Temporary controller SHA：`12b988ea69d33b60ecda2f4978a01387891c9f1e17bbd6706f42cb038f3679a6`。
- Candidate：`BOTTOM_DIRECT_REORIENT_ODOM_HANDOFF`；DIRECT `+pi/4`、DIAGONAL REVERSE cumulative planar odom path、RESTORE saved target、direct POST_TURN handoff。
- `6/6` 有效完整 B route 成功；fallback activation `5/6`，其餘一 run front-clear priority 正常生效。
- 所有 activation 完成三個 segment，沒有 timeout、route hard fail-safe、generic RECOVER forward、RECOVER_FAILED、exception 或 state loop。
- Level 1 完成後 production 已 rollback；Phase 5R 使用保存的 temporary diff 重建，未重新設計。
- Evidence：`/home/ray/ros2_evidence/phase5q_bottom_direct_reorient_odom_handoff_screening_20260718_140225`

## Phase 5R — Bottom Direct Reorient Production Retention

Status: **R1 RETAINED**（SPEC Level 2）

- Controller SHA before：`3866302660a95e92cece184b1e859a9bc73b8fe92888f388eb62b3b851c46a25`
- Controller SHA after：`12b988ea69d33b60ecda2f4978a01387891c9f1e17bbd6706f42cb038f3679a6`
- Reapplied source 與 Phase 5Q `controller_temporary.py` byte-identical，controller SHA 精確符合凍結值；沒有重新設計或調參。
- Syntax、`git diff --check`、package build 全部 exit `0`；formal／workspace／installed 三份 SHA 相同，formal↔workspace 與 workspace↔installed `cmp=0`。
- No-obstacle：`2/2` valid completed；fallback activation、extension activation、RECOVER_FAILED、timeout、exception、state loop 全為 `0`。
- Production B：`8/8` valid formal full-route successes；normal valid rerun `0`，setup/protocol failure `0`。
- Fallback activation：`7/8`；`R_B05` 在第一個可處理的 post-5 callback front=`0.4664533734321594 m`，front-clear priority 正常生效而 activation=`0`，仍完整成功。
- 7 次 entry 全為 bottom-context、elapsed `5.000–5.115 s`、front non-clear；第一筆 command `(0,+0.600)`、same-callback、無 STOP dwell。
- DIRECT：duration `2.625–2.875 s` `<3.490 s`；displacement `0.020761952–0.030288665 m`；最低 clearance `137.892655 mm`；最大實際 angular velocity `0.6065385893151476 rad/s`；overshoot 全部 `0`。
- DIAGONAL REVERSE：duration `0.755–0.885 s` `<2.350 s`；formal cumulative planar odom path `0.028406352–0.032994570 m`，全部達 `0.027969393301043588 m`；最低 clearance `154.793751 mm`。
- RESTORE：duration `1.610–1.640 s` `<3.505 s`；displacement `0.009447186–0.014498762 m`；最低 clearance `148.830342 mm`；overshoot 全部 `0`。
- Candidate route duration `4.990–5.375 s`；相對 `8.380 s` hard fail-safe 最小 margin `3.005 s`；正常 completion 全部由 segment predicate，無 timeout completion。
- Candidate route 最低 clearance `137.892655 mm`（`R_B08`，`base_collision:island_wall_south`）。
- Handoff：`7/7` 直接進 `POST_TURN`，first POST_TURN command 均有效，之後 `2.360–2.390 s` 進 `STRAIGHT`；handoff+2.0 s 最低 clearance `160.457350 mm`。
- `RECOVER_EGRESS_EXTENSION`：B `8/8` activation、odom completion、ALIGN_DOWN、downward handoff與 route bit-exact；route duration `5.120–5.375 s`。
- B 全程最低 clearance `38.073101 mm`（`R_B03`，`base_collision:obstacle_yellow_mid`）；contact／<10 mm、candidate semantic error、generic RECOVER forward、RECOVER_FAILED、timeout、exception、unexpected state loop 全部為 `0`。
- Level 2 evidence root：`/home/ray/ros2_evidence/phase5r_direct_reorient_level2_retention_20260719_141424`
- Read-only archive：`/home/ray/ros2_evidence/phase5r_direct_reorient_level2_retention_20260719_141424.tar.gz`；SHA-256：`69729a5581502df4aa7ba86e730bbaf61a12b67055a06eb08f1f90bd3b8b7f6c`；mode `0444`。
- 第一次 Python `tarfile extractall(filter=data)` 驗證未回復最外層 extracted root 的 archive `0555` mode，分類為 verifier protocol failure；原 archive 未重建，失敗紀錄保存於 `.tar.gz.verification_attempt_01.txt`。
- 同一唯讀 archive 改由 GNU tar 獨立解壓後，`48` files／`4` directories／`0` symlinks 的 paths、types、sizes、SHA、symlink targets 全部相同；files `0444`、directories `0555`，verification failures `0`。
- `complete_sha256.txt`、staging↔persistent equality、唯讀 modes與獨立解壓驗證全部通過；final record：`.tar.gz.verification.txt`，SHA sidecar：`.tar.gz.sha256`。

Phase 5R 是指定 production patch 的 Level 2 retention，**不等於整體專案 final acceptance**。

## Phase 6F — Reset-race Root Cause

Status: **OLD-EPOCH ODOM CONTINUATION CONFIRMED**

- reset-detecting odom callback 在 time rewind reset 後繼續處理觸發 reset 的 old-epoch odom，可重新污染 cache、pose、`_straight_target_yaw` 與第一筆非零 command。
- Phase 6F 保存的 stale record 後續由 Phase 6H／6I offline gate 重播；本輪沒有重新掃描 Phase 6F raw evidence。

## Phase 6H — Old-Epoch Odom Quarantine Level 1

Status: **H1 OLD_EPOCH_ODOM_QUARANTINE_LEVEL1_PASS**（SPEC Level 1）

- Semantic candidate SHA：`de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2`。
- Runtime diagnostic SHA：`89cecb19e694736e9ad988dd7124bb6b37d0093e95160589d2c0d6b29e7dd32d`；quarantine core 與 semantic candidate AST match。
- 4/4 groups、12/12 valid slots；normal valid rerun `0`。
- No-obstacle `8/8`、B01 `4/4` PASS；8 resets、2 odom barrier activations、2/2 fresh repopulation。
- Barrier nonzero publish、stale cache/target leak、control/safety regression均為 `0`。
- Evidence：`/home/ray/ros2_evidence/phase6h_setup_repair_odom_quarantine_20260722_001814`
- Manifest SHA：`58520aa6b2218f3cfd6909742cda02a1aab560dd69bf730a305659fa3e20dbe9`。

## Phase 6I — Old-Epoch Odom Quarantine Level 2 Retention

Status: **I1 OLD_EPOCH_ODOM_QUARANTINE_LEVEL2_RETAINED**（SPEC Level 2）

- Controller SHA before：`12b988ea69d33b60ecda2f4978a01387891c9f1e17bbd6706f42cb038f3679a6`。
- Controller SHA after：`de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2`。
- Runtime 前 Phase 6H manifest與完整 offline gate全部 PASS；candidate identity、scope及 semantic/diagnostic core match通過。
- 固定 campaign：2/2 cold-launch groups、8/8 valid slots；I_G1 `NO01→NO02→B01→B02`，I_G2 `B01→NO01→B02→NO02`。
- Group PID：I_G1 `6836`、I_G2 `7877`；組內不變、組間不同。Normal valid rerun `0`。
- I_G1 attempt 01 因 shell 未 source workspace，launch 找不到 `wro2026_sim`；在第一個 slot 前停止且沒有 ROS clock/controller PID/topic或有效 slot。紀錄保存後只使用規格允許的唯一 setup retry。
- No-obstacle `4/4` 抵達 `NO_OBSTACLE_BOTTOM_POSE`；B `4/4` 抵達 `FIRST_BLUE_ACCEPT_PLUS_0P5`。
- Reset `6`；odom-detected reset `4`。4/4 reset callback STOP/immediate return、empty cache、fresh acceptance/repopulation、fresh target與 first command ordering全通過。
- Runtime rejected old-epoch odom `0`（沒有訊息落入 barrier queue）；offline gate 的 multiple old headers、missing header與 suppressed nonzero command replay全通過。
- 全組最低 clearance `52.847743 mm`；contact、`<10 mm`、exception、state loop、generic RECOVER forward、stale leak與barrier nonzero publish全部為 `0`。
- Production smoke：semantic controller PID建立，`/clock`、`/odom`、`/scan`、`/camera/image_raw`、`/cmd_vel` 均存在且各取得一筆訊息。唯一 traceback/exit `-2` 發生於 runner 主動 SIGINT cleanup，三個 Python ROS nodes 同步呈現相同 Jazzy shutdown artifact；運行期沒有 exception，未重跑 smoke。
- Diagnostic controller及runner hooks已移除；formal／workspace／installed SHA一致、兩組 `cmp=0`，syntax、final build、`git diff --check`與cleanup全部通過。
- Evidence：`/home/ray/ros2_evidence/phase6i_old_epoch_odom_quarantine_level2_retention_20260722_214929`

Phase 6I 是指定 quarantine candidate 的 Level 2 retention，**不等於整體專案 final acceptance**。

## Phase 6J — Final Production Acceptance Audit

Status: **J1 FINAL_ACCEPTANCE_PASS；FINAL ACCEPTED**

- Production controller SHA：`de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2`；本 phase 未修改 controller、runner語意、world、model、launch、QoS或threshold。
- 開始 gate：formal／workspace／installed SHA一致、兩組 `cmp=0`；Phase 6I archive SHA與獨立解壓 `125` records／`0` failures通過；Git狀態保存且無殘留runtime process。
- 固定單一 cold-launch sequence：`NO01→reset→B01→reset→NO02→reset→B02`；`4/4` valid，controller PID全程 `22996`；normal valid rerun `0`，setup retry `0`。
- NO `2/2` 抵達 `NO_OBSTACLE_BOTTOM_POSE`；B `2/2` 抵達 `FORMAL_FULL_ROUTE_SUCCESS_PLUS_0P5`，fallback activation `2/2`且成功 `2/2`。
- Reset `3`，old-epoch odom `0`。其中 `1` 次由 odom callback偵測：STOP/immediate return、empty cache、fresh acceptance/repopulation、fresh start-pose target、barrier nonzero command `0`與first-command ordering全部PASS。
- 另 `2` 次為 scan/image callback偵測；被動 `/clock`、`/odom`、`/cmd_vel` probe確認clock rewind、第一筆start odom先於第一筆非零command，沒有old-epoch odom。
- 全組最低 clearance `54.794775 mm`；contact、`<10 mm`、RECOVER_FAILED、exception、unexpected state loop、generic RECOVER forward與新增regression全部為 `0`。
- 最終 formal／workspace／installed SHA維持一致、兩組 `cmp=0`；syntax、`git diff --check`、shutdown及cleanup全部通過。
- SPEC lightweight retention保留worst NO與worst B full traces，其他高容量stream已抽取為reset／ordering evidence後移除。
- Evidence root：`/home/ray/ros2_evidence/phase6j_final_production_acceptance_20260722_224955`
- 唯讀 final acceptance archive、SHA sidecar與獨立解壓驗證記錄位於同名 `.tar.gz`、`.tar.gz.sha256`、`.tar.gz.verification.txt`。

Phase 6J 通過所有最終驗收 gate；專案狀態正式封為 **FINAL ACCEPTED**，不建立後續 Phase。
