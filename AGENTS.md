# Project Instructions

新對話先讀 `docs/STATUS.md`（唯一的目前狀態），再依工作內容選規則：

- 模擬線（`src/wro2026_sim`）：照 `docs/sim/SPEC.md`。
- 實車線（`src/wro2026_base_waverover`、`src/wro2026_real` 等非模擬 package）：照 `CLAUDE.md`。

模擬線的規則（原文保留）：

- 以目前 working tree、`docs/STATUS.md` 與 controller SHA 為準。
- 不得重新執行已完成的歷史 Phase。
- 遵守 `docs/sim/SPEC.md` 的效能限制、增量分析及 Level 0/1/2 制度。
- 未經使用者確認目前狀態前，不得修改程式或執行 simulation。

歷史 Phase 的狀態與證據摘要已封存在 `docs/archive/sim_phases/`（`CURRENT_STATE.md`、`TEST_EVIDENCE.md`、`NEXT_PROMPT.md`），需要背景時才讀。
