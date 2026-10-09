# WRO ROS2 Phase Execution Specification

## 1. 基本安全規則

* 不得 commit、push、pull、reset 或 stash，除非使用者明確授權。
* 不得修改 world、model、launch 或 docs，除非本輪明確要求。
* Temporary selector、diagnostic code 和 test-only state 必須在本輪結束前完整回退。
* 不得留下 ROS、Gazebo、controller 或測試 process。

## 2. Controller 同步驗證

凡本輪有修改或測試 controller，結束時必須確認：

* formal、workspace、installed 三份 controller SHA-256 一致。
* formal ↔ workspace `cmp=0`。
* formal ↔ installed `cmp=0`。
* syntax check 通過。
* build exit code 為 0。
* `git diff --check` 通過。
* Temporary／diagnostic 修改必須完整回退。
* Git status 中只能保留本輪明確授權的 production、docs 或 evidence 變更。
* 不得出現未說明的新修改。

只檢查 controller，不得遞迴掃描整個 workspace。

## 3. 禁止大量掃描

除非本輪明確要求，禁止遞迴掃描：

* build/
* install/
* log/
* archive/
* 舊的 evidence folders
* 整個 repository

不得對以上目錄執行大型 `find`、`du`、`rg`、全文索引或逐檔 SHA。

搜尋程式時只搜尋本輪相關檔案，例如：

`src/wro2026_sim/scripts/line_follower.py`

## 4. 增量分析

* 優先使用上一輪已完成的 CSV、JSON 和 summary。
* 只分析本輪新增 runs。
* 已經分類完成的歷史 log 不重新解析。
* 只有資料矛盾或缺少必要欄位時，才回讀原始 log。

## 5. Run 資料保存

所有 runs 只保存輕量 summary，包括：

* run id
* arm/candidate
* terminal classification
* PASS 是否進入
* completion source
* minimum clearance
* RECOVER entry clearance
* limiting pair
* timeout、exception、D2、D4
* camera、scan、control rate summary

完整 callback trace 只保存：

* 所有失敗 runs
* candidate 第一次真正 branch activation
* 每個 arm 最差 margin run
* baseline 最差 margin run
* 每個 arm 最多一個代表性 SAFE run
* 分類有爭議的 run

不得讓每個正常 run 都輸出完整 callback trace。

## 6. Evidence 文件

一般 Phase 只需要：

* implementation_review.md
* validation_review.md
* run_inventory.csv
* per_run_summary.csv
* classification.json
* phase_summary.md
* rollback_verification.md
* complete_sha256.txt

不要重複建立內容相同的 review 文件。

## 7. Archive 規則

一般 screening Phase 不建立完整 archive。

只有以下情況才建立唯讀 archive：

* production patch 正式保留
* 重要根因支線正式 Done
* 原始資料未來可能無法重現
* 準備 commit/push
* 最終驗收

建立 archive 時才執行：

* sync
* tar.gz
* chmod 0444
* archive SHA-256
* 獨立解壓驗證
* path/type/size/SHA/symlink target 驗證

## 8. 驗證等級

### Level 0：離線分析

* 不 build
* 不 launch
* 不 simulation
* 不 archive
* 只分析指定 evidence 和 summary

### Level 1：Screening

* temporary patch
* syntax/build
* no-obstacle 或必要 preflight
* 固定 run campaign
* 輕量 summary
* 事件式完整 trace
* 完整 rollback
* 不建立 archive

### Level 2：Milestone

* production patch 正式保留
* 完整 regression
* 完整 evidence
* 建立唯讀 archive

## 9. Phase Prompt 原則

每輪 Prompt 只需要描述：

* Phase 名稱
* 本輪目標
* 唯一 candidate
* 允許修改的範圍
* run schedule
* 驗收和拒絕條件
* 本輪輸出
* Done / Partial / Blocked 規則

不得在每輪 Prompt 重複貼上整份固定規格。

## 10. 新對話與專案交接

新 Codex 對話開始時，依序讀取：

1. AGENTS.md
2. docs/SPEC.md
3. docs/CURRENT_STATE.md
4. docs/NEXT_PROMPT.md
5. docs/TEST_EVIDENCE.md

以目前 working tree、CURRENT_STATE.md 與 controller SHA 為準。

不得重新執行已標記 Done 的歷史 Phase。
不得為理解專案而重新掃描全部舊 evidence 或原始 logs。
只有 CURRENT_STATE.md 缺少必要資訊或資料互相矛盾時，才讀取指定的歷史 summary。

新對話第一步只能摘要：

- 目前 Phase 狀態
- controller SHA
- 已保留的 production 修改
- 已關閉或拒絕的方向
- 唯一下一步

在使用者確認摘要前，不得修改檔案、build、launch 或執行 simulation。
