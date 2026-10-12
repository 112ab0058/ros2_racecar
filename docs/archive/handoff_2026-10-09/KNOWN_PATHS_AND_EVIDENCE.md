> 已封存（2026-10-13）。目前狀態見 docs/STATUS.md。

# 已知路徑與證據索引（均須在本機核對）

## Repo/docs
- Repo：`/home/ray/ros2_racecar`
- `docs/SPEC.md`
- `docs/CURRENT_STATE.md`
- `docs/TEST_EVIDENCE.md`
- `docs/NEXT_PROMPT.md`
- 禁止寫入：`/home/ray/ros2_ws`

## Production identity / map
- Historical accepted controller SHA：`de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2`
- Historical map：`/home/ray/ros2_racecar/src/wro2026_sim/maps/phase7_nav2_field_map.yaml` 與同名 `.pgm`

## Wheel dataset / AE
- Dataset root：`/home/ray/ros2_evidence/phase7eady_measurement_repaired_dataset_20260807_004700/`
- Dataset file：`measurement_repaired_local_wheel_odom_calibration_dataset.yaml`
- SHA：`2e650de7f3667fa2d335b14ffc44134e0ec9aa6b667721f36490577cb306f94e`
- AE evidence：`/home/ray/ros2_evidence/phase7eae_calibrated_wheel_odom_candidate_20260807_021029/`，manifest SHA `d4e7daebef15c485fdc4a992ea2e4f1ec9ea0388f9972acb79f50f8a9b57837e`
- AE-R：`/home/ray/ros2_evidence/phase7eaer_nominal_geometry_adequacy_20260807_025606/`，freeze SHA `a0dcdbcf6b8b3df4f6bc10b3a2e1674fed2b56f447863abb11b94a0f98f2ca9d`
- AE-S-R：`/home/ray/ros2_evidence/phase7eaesr_identifiability_contract_repair_20260807_050037/`，manifest SHA `aa9423884446a71a44553e1302428e68921e9c28eeba1d1dc1cfb935eaa5ca64`

## Campaigns
- Architecture reassessment：`/home/ray/ros2_evidence/phase7e_autonomous_completion_campaign_20260807_053800/`；classification `PROVEN_IRREDUCIBLE_ARCHITECTURE_BLOCKER`；manifest SHA `c36307d0f4a98274075f3535f4102563c582f73855d294319b1dbc3f4827a09c`
- Residual odom：`/home/ray/ros2_evidence/phase7g_residual_odom_campaign_20260807_062907/`；classification `PROVEN_RESIDUAL_ODOM_ARCHITECTURE_BLOCKER`；manifest SHA `0de44b3359f618b4e94b7077f62fbd95750767e296765943465fced4f697793a`

## 查閱順序
先讀 `campaign_final_report.md`/`phase_summary.md`，再讀 `classification.json`，驗證 `complete_sha256.txt`，最後才讀特定 raw data。不要把整批 rosbag/raw images 全部載入 AI context。
