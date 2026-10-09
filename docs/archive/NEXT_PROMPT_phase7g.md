執行以下長期自主完成目標。

Phase 7G Autonomous State-Dependent Residual Odom Campaign
— From Non-Geometric Error Modeling to Final Localization Acceptance

==================================================
Campaign總目標
==================================================

目前constant rL／rR／B geometry calibration路線已正式證明：

PROVEN_IRREDUCIBLE_ARCHITECTURE_BLOCKER

不得重新開啟、放寬gate或再次搜尋constant geometry。

本Campaign明確授權一條全新的唯一架構：

STATE-DEPENDENT RESIDUAL SE(2) ODOM CORRECTION

它不再嘗試從安全body-Twist excitation中分離：

- rL
- rR
- B

而是以固定nominal geometry：

- rL=0.033 m
- rR=0.033 m
- B=0.160 m

產生nominal wheel-kinematic increment，再以training-only學習的、
低維、可解釋、受約束之state-dependent residual model修正：

- Local forward increment
- Local lateral increment
- Signed yaw increment
- State-dependent covariance

Runtime不得使用truth。

本Campaign持續推進以下完整鏈：

1. 封存constant geometry blocker。
2. 分解既有wheel／truth residual的state dependence。
3. 建立固定且唯一的殘差模型architecture family。
4. 使用既有Y training資料進行training-only model selection。
5. Freeze後使用既有Y holdout作Generation A獨立驗證。
6. 若Generation A已充分，直接建立residual odom candidate。
7. 若Generation A因feature coverage不足而失敗，
   自主設計新的residual-coverage excitation sequence。
8. 實作並qualification新的coverage controller。
9. 取得全新的Generation B training／holdout dataset。
10. 使用Generation B training重新完成一次正式模型推導。
11. 使用Generation B holdout獨立驗證。
12. 建立state-dependent covariance contract。
13. 實作ResidualSE2Odom correction layer。
14. 完成unit tests及offline replay。
15. 完成單一runtime qualification。
16. 建立獨立AMCL calibrated-odom candidate。
17. 完成Localization Level 1。
18. 完成Phase 7F final localization acceptance。
19. 最終通過後才promote正式package及文件。
20. 中途出現唯一且安全的implementation repair時，
    自主建立repair phase繼續，不得每一小段停下詢問。

成功終點：

FINAL_STATE_DEPENDENT_ODOM_LOCALIZATION_ACCEPTANCE_PASS

合法失敗終點：

PROVEN_RESIDUAL_ODOM_ARCHITECTURE_BLOCKER

不得因一般build、script、overlay、QoS、serialization、
validator或report問題提前停止。

==================================================
目前權威狀態
==================================================

Latest autonomous campaign evidence：

/home/ray/ros2_evidence/phase7e_autonomous_completion_campaign_20260807_053800

Campaign classification：

PROVEN_IRREDUCIBLE_ARCHITECTURE_BLOCKER

Stop condition：

STOP_ARCHITECTURE_BLOCKER

Campaign manifest：

60/60 PASS

Campaign manifest SHA：

c36307d0f4a98274075f3535f4102563c582f73855d294319b1dbc3f4827a09c

Campaign完成：

- Phase 7E-AE-S2
- Phase 7E-AE-S3

S2 classification：

7EAES2_2 —
NO_FEASIBLE_WHEEL_BASIS_HIGH_INFORMATION_DESIGN

S3 classification：

7EAES3_2 —
SAFE_BODY_TWIST_ENVELOPE_INSUFFICIENT_DIRECT_WHEEL_EQUIVALENT

S2 evidence：

/home/ray/ros2_evidence/phase7e_autonomous_completion_campaign_20260807_053800/01_phase7eaes2

S3 evidence：

/home/ray/ros2_evidence/phase7e_autonomous_completion_campaign_20260807_053800/02_repair_1

S2 manifest SHA：

e463ea6c849de28cf62d6d5fb6e6711ecbd7a1f19972284b595e82bc34aac458

S3 manifest SHA：

c0f1895dfcf8d7836ac5282dd1a1277ce2f131ea1c0538bf137778748ce01ad0

S2：

- Search execution count=1
- Evaluated points=2304
- Feasible points=0
- Minimum original condition=231.9081
- Minimum original correlation=0.9995249
- Minimum transformed condition=91.7704
- Minimum transformed correlation=0.9975922

Frozen gates：

Original：

- Condition≤100
- Correlation<0.98

Transformed：

- Condition≤75
- Correlation<0.98

S3：

- Production-safe command envelope assessed
- Evaluated points=3645
- Feasible points=0
- Direct wheel command與body Twist為可逆映射
- Direct-wheel API不增加新的運動基底
- Frozen plugin沒有額外wheel command interface

Constant geometry路線正式封存。

不得：

- 重跑S2
- 重跑S3
- 修改S2／S3 search space
- 放寬identifiability gates
- 重新估計constant rL／rR／B
- 以direct-wheel API作為新的資訊來源
- 追認任何舊candidate

==================================================
Production Identity
==================================================

Production controller expected SHA：

de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2

Campaign開始時必須從：

- Formal copy
- Workspace copy
- Installed copy
- Final Accepted evidence

重新取得及驗證完整SHA。

要求：

- Formal／workspace／installed三份SHA完全相同
- cmp=0
- Campaign全程不得修改

其他frozen artifacts：

- World
- Robot geometry
- Spawn
- Phase 7D-R map
- Phase 7D-R route
- Raw odom publisher
- Existing odom→base_footprint TF owner
- Baseline AMCL config
- Frozen localization thresholds
- docs/CURRENT_STATE.md
- docs/TEST_EVIDENCE.md
- 所有既有evidence

/home/ray/ros2_ws：

永久禁止寫入。

==================================================
既有高品質Dataset
==================================================

Phase 7E-AD-Y evidence：

/home/ray/ros2_evidence/phase7eady_measurement_repaired_dataset_20260807_004700

Classification：

7EADY1 —
MEASUREMENT_ASSOCIATION_REPAIR_AND_FRESH_DATASET_ACQUISITION_PASS

Dataset：

/home/ray/ros2_evidence/phase7eady_measurement_repaired_dataset_20260807_004700/measurement_repaired_local_wheel_odom_calibration_dataset.yaml

Dataset SHA：

2e650de7f3667fa2d335b14ffc44134e0ec9aa6b667721f36490577cb306f94e

Status：

VERSIONED_MEASUREMENT_ASSOCIATION_REPAIRED_LOCAL_WHEEL_ODOM_CALIBRATION_DATASET

Training：

E_WHEEL_ODOM_QUALITY_REPAIR_TRAIN_01

Holdout：

E_WHEEL_ODOM_QUALITY_REPAIR_HOLDOUT_01

Training及holdout：

- Independent cold start
- Cross-runtime pairing=0
- Raw SHA overlap=0
- Joint observer loss=0
- Writer drop=0
- Exact-stamp TF／odom一致
- Safety PASS
- Rank=3
- Existing geometry fitting未通過identifiability

本Campaign允許將這份dataset用作：

Generation A

固定partition保持：

- Y training只能供Generation A training／model selection
- Y holdout只能供Generation A final independent evaluation

不得：

- 合併training及holdout
- 隨機重切
- 使用Y holdout選feature、model order或regularization
- 在看到Y holdout結果後修改Generation A candidate
- 將舊失敗runtime混入
- 將qualification資料混入

若Generation A失敗且進入Generation B：

Y dataset只能作：

- Feature-range參考
- Noise contract
- Regression fixture
- Execution bound
- Model architecture診斷

不得加入Generation B正式training或holdout。

==================================================
本Campaign一次性完整授權
==================================================

Evidence：

- 建立一個全新campaign evidence root
- 建立每個phase獨立subroot
- 寫入分析程式、configs、raw evidence、reports及manifest
- 唯讀載入全部列明的既有evidence
- 建立campaign state、decision log及runtime inventory
- 執行report-only finalizer
- 建立每個phase及整體SHA manifest

Repository-local：

- 建立phase-scoped temporary package
- 編輯本Campaign所建立的temporary source
- 建立model trainer、replay engine、observer、validator、
  correction node、launch及tests
- 執行repo-local build
- 執行unit、offline replay及pre-launch tests
- 在runtime lock前持續修復temporary implementation
- 建立candidate configs
- 建立temporary AMCL candidate config
- 最終7F PASS後執行一次正式promotion

Runtime：

- 執行本Prompt明確列出的qualification、dataset、
  correction-layer及localization runtimes
- 使用Python supervisor
- 收集JointState、odom、TF、truth、clock、command及clearance
- Truth只能作observer、training target及evaluation
- Truth不得作runtime node或controller input
- 每個runtime使用新的ID及新evidence directory
- 依規格使用pre-lock setup retry

Autonomous repair：

允許自行修復：

- Syntax
- Import
- Build
- Package metadata
- Overlay
- AMENT prefix
- QoS
- Callback metadata
- Writer queue
- Serialization
- Exact timestamp pairing
- File path
- Model serialization
- Replay mismatch
- TF ownership implementation
- Reset／epoch handling
- Covariance serialization
- Launch integration
- Validator
- Manifest
- Cleanup

在唯一root cause且不放寬gate時，
可自行建立repair phase及新runtime ID。

不得每一phase結束就要求使用者授權。

==================================================
絕對禁止事項
==================================================

不得：

- 修改production controller
- 修改world
- 修改robot geometry
- 修改spawn
- 修改map
- 修改route
- 修改raw odom publisher
- 修改existing odom→base_footprint TF owner
- 覆寫baseline AMCL config
- 寫入/home/ray/ros2_ws
- 修改或刪除既有evidence
- 重新開啟constant geometry calibration
- 修改frozen rL／rR／B identifiability結論
- 使用holdout調參
- 使用Phase 7F結果回頭調模型
- 使用truth作runtime correction
- 使用truth作controller feedback
- 使用AMCL pose作residual model input
- 使用route state作model input
- 使用cmd_vel作主要odometry source
- 以nearest／latest pairing冒充same-time evidence
- 缺失資料插值後冒充raw evidence
- 放寬frozen localization gate
- 為消耗額度重複已完成的分析
- 重跑有效runtime挑結果
- 使用多個互斥模型architecture人工挑選
- 在runtime後修改candidate再追認同一runtime
- 使用git reset --hard
- 使用無範圍git clean
- 刪除未知path

==================================================
唯一新Architecture
==================================================

Architecture ID：

STATE_DEPENDENT_RESIDUAL_SE2_ODOM_V1

Future node：

StateDependentResidualSE2Odom

Future package：

wro2026_residual_odom

Nominal input increment由：

- /joint_states
- Nominal rL=0.033
- Nominal rR=0.033
- Nominal B=0.160

使用exact SE(2)積分建立。

Raw /odom只允許作：

- Initial epoch alignment
- Existing raw TF correction reference
- Reset／timestamp consistency
- Runtime diagnostic

Raw /odom不得作training truth target。

Training target：

Gazebo truth relative SE(2) increment

Runtime允許輸入：

- /joint_states
- /odom
- /clock

Runtime禁止輸入：

- Gazebo truth
- AMCL pose
- map→odom
- Scan
- Camera
- Route
- Waypoint
- cmd_vel
- Particle cloud
- Contact
- Clearance
- Ground-truth residual

Runtime輸出：

- /odom_residual_calibrated
- TF odom_residual_calibrated→odom

Existing raw source保持：

- /odom
- TF odom→base_footprint

Future AMCL唯一發布：

map→odom_residual_calibrated

不得發布：

- odom_residual_calibrated→base_footprint
- map→odom
- map→odom_residual_calibrated，由correction node發布
- odom→base_footprint
- /cmd_vel

==================================================
Residual Model Target
==================================================

每個固定window建立nominal local increment：

u_nominal =

- Δx_nominal
- Δy_nominal
- Δyaw_nominal

Truth local increment：

u_truth

Residual target：

r = log_SE2(
  inverse(T_nominal) × T_truth
)

至少輸出：

- residual_forward
- residual_lateral
- residual_yaw

不得直接使用global x／y差。

Windowing初始contract：

- 0.100 sim-s
- 不跨runtime
- 不跨epoch
- 不跨primitive
- 不跨invalid JointState bracket
- Joint boundary interpolation
- Truth boundary interpolation
- Publisher-side stage interval
- Terminal partial <0.050 s排除
- Terminal partial ≥0.050 s獨立保存

Generation A分析可確認該contract，
但不得使用holdout修改。

==================================================
固定Feature Architecture
==================================================

只能使用以下runtime可取得feature。

Base features：

- 1
- Δs_nominal
- Δyaw_nominal
- |Δs_nominal|
- |Δyaw_nominal|
- dt
- v_nominal=Δs/dt
- ω_nominal=Δyaw/dt
- |v_nominal|
- |ω_nominal|
- v_nominal×ω_nominal
- |v_nominal|×|ω_nominal|
- signed curvature
- absolute curvature
- left wheel increment
- right wheel increment
- average wheel increment
- right-minus-left increment
- stationary／motion indicator
- straight／turn motion-regime indicator

Curvature必須使用事前固定epsilon及clip。

禁止features：

- Truth
- Truth residual history
- AMCL
- Route
- Waypoint
- Global position
- World region
- Obstacle distance
- Clearance
- Contact
- Scan
- Camera
- cmd_vel
- Primitive名稱直接作模型feature
- Runtime ID
- Training／holdout label

Stage及primitive只允許：

- Grouped CV
- Diagnostics
- Evaluation

不得作runtime feature。

==================================================
固定Model Family
==================================================

只允許一個architecture family：

SYMMETRY_CONSTRAINED_REGULARIZED_RESIDUAL_MODEL

包含預先固定的四個nested levels：

R0：

Nominal identity，無residual correction。

R1：

Linear residual basis。

R2：

Linear＋固定二階interaction basis。

R3：

R2＋固定curvature hinge basis。

R4：

R3＋固定motion-regime covariance分層，
但mean correction basis不得再增加。

R1至R4是同一nested architecture，
不是多個互斥架構。

不得新增：

- Neural network
- Random forest
- Gaussian process
- SVM
- High-order unrestricted polynomial
- Lookup table byroute
- Runtime truth observer
- Online adaptive parameter fitting
- Reinforcement learning
- Kalman filter withtruth update

Mean model使用：

- Ridge regression
- 固定標準化
- Training-only scaler
- Fixed intercept contract
- Fixed symmetry augmentation或等價線性constraints
- No arbitrary feature deletion after holdout

每個輸出：

- forward residual
- lateral residual
- yaw residual

可使用共享feature matrix及獨立output coefficients。

==================================================
Symmetry Contract
==================================================

在left／right mirror下：

- Swap left／right wheel increments
- Δyaw sign反轉
- Curvature sign反轉

期望：

- Forward residual保持偶對稱主體
- Lateral residual反轉
- Yaw residual反轉

必須以：

- Hard coefficient constraints
- 或training-only mirrored augmentation

建立。

不得以holdout檢查結果修改symmetry方法。

輸出必須報告：

- Symmetry residual
- Mirror consistency
- Left／right bias
- Constraint violations

==================================================
Training-Only Model Selection
==================================================

Generation A只使用Y training。

建立blocked grouped CV。

固定fold至少包含：

1. Time-block folds。
2. Leave-one-motion-regime-out diagnostics。
3. Left／right mirrored groups隔離。
4. Stop windows獨立處理。
5. 不得將相鄰window隨機分至train／validation。

Regularization grid必須在看到CV結果前freeze。

建議固定log grid：

lambda ∈
{
1e-8,
1e-7,
1e-6,
1e-5,
1e-4,
1e-3,
1e-2,
1e-1,
1,
10,
100
}

不得在看到Y holdout後修改。

Model selection順序：

1. R0至R4依複雜度排序。
2. 每個level內選擇CV weighted loss最低lambda。
3. 使用one-standard-error rule選最簡單level。
4. Candidate必須通過training-only stability gates。
5. Freeze candidate後才能載入Y holdout。

Loss：

- Forward residual normalized squared error
- Lateral residual normalized squared error
- Yaw residual normalized squared error
- Primitive／regime equal-total weighting
- Training-only robust scales
- No holdout weighting

==================================================
Generation A Training Gates
==================================================

Candidate freeze前要求：

- Feature matrix finite
- Rank合理
- No duplicated feature column
- Standardized condition number≤1e6
- Coefficients finite
- CV fold success=100%
- No leakage
- Symmetry gate通過
- CV weighted SSE優於R0
- CV yaw RMSE改善≥10%
- CV translation RMSE改善≥5%
- 任一fold yaw不得惡化>10%
- 任一fold translation不得惡化>10%
- Coefficient norm受regularization控制
- Effective degrees of freedom≤40
- Feature support完整記錄
- OOD guard可定義
- Candidate serialization round-trip PASS

若Generation A training gates失敗：

先分類：

GA_TRAIN_A —
FEATURE_COVERAGE_INSUFFICIENT

GA_TRAIN_B —
MODEL_FAMILY_INADEQUATE

GA_TRAIN_C —
NUMERICAL_OR_IMPLEMENTATION_DEFECT

C可自主repair。

A進入Generation B coverage dataset設計。

B停止為：

PROVEN_RESIDUAL_MODEL_FAMILY_BLOCKER

==================================================
Generation A Candidate Freeze
==================================================

Training gates通過後建立：

generation_a_candidate_freeze.yaml

包含：

- Dataset SHA
- Training runtime ID
- Feature contract SHA
- Feature scaler SHA
- Model level
- Lambda
- Coefficients SHA
- Symmetry contract
- CV metrics
- Support bounds
- OOD contract
- Code SHA
- Config SHA
- Timestamp
- No-holdout-access declaration

Freeze前不得載入Y holdout residual values。

Freeze後：

- Model level不可修改
- Lambda不可修改
- Features不可修改
- Scaler不可修改
- Coefficients不可修改
- Gates不可修改

==================================================
Generation A Holdout Gates
==================================================

Freeze後才允許載入：

E_WHEEL_ODOM_QUALITY_REPAIR_HOLDOUT_01

比較：

A. Raw plugin odom  
B. Independent nominal wheel odom  
C. Frozen Generation A residual odom

至少計算：

- Per-window translation RMSE
- Per-window yaw RMSE
- P95／P99／max
- Primitive endpoint error
- Integrated final position error
- Integrated final yaw error
- Left／right symmetry
- Straight／turn performance
- Stop drift
- Out-of-support count
- OOD fallback count
- Correction magnitude

Generation A holdout全部要求：

- No holdout refit
- Weighted SSE優於nominal
- Yaw RMSE改善≥15%
- Translation RMSE改善≥10%
- Translation不得相對raw odom惡化>5%
- Integrated final position改善≥10%
- Integrated final yaw改善≥20%
- 任一motion regime translation不得惡化>10%
- 任一motion regime yaw不得惡化>10%
- Left／right方向無明顯偏差
- OOD fraction≤5%
- Correction finite
- No correction explosion
- Residual mean明顯下降
- Residual autocorrelation下降
- Candidate不依賴truth或stage runtime input

若全部通過：

進入state-dependent covariance及implementation。

若失敗，分類：

GA_HOLDOUT_A —
COVERAGE_OR_SUPPORT_GAP

GA_HOLDOUT_B —
GENERALIZATION_FAILURE

GA_HOLDOUT_C —
IMPLEMENTATION_OR_REPLAY_DEFECT

C可自主repair但不得修改freeze candidate。

A進入Generation B。

B停止為：

PROVEN_RESIDUAL_MODEL_GENERALIZATION_BLOCKER

不得使用Y holdout調整model後再次測試同一holdout。

==================================================
OOD Guard
==================================================

Runtime必須有training-only固定support guard。

至少包含：

- Per-feature min／max support
- Robust quantile support
- Mahalanobis或標準化距離
- Motion-regime availability
- Maximum correction magnitude
- Maximum correction derivative

不得：

- 使用truth判斷OOD
- 使用AMCL error判斷OOD
- 使用holdout重新設定support

Out-of-support時固定行為：

- Mean correction平滑衰減至identity
- 不允許突然跳變
- Covariance提高
- Diagnostic標記DEGRADED_SUPPORT
- 不得停止發布odom
- 不得用last correction永久保持

所有blend公式及threshold必須在candidate freeze前固定。

==================================================
Generation B觸發條件
==================================================

只有以下情況進入Generation B：

- Generation A training分類GA_TRAIN_A
- Generation A holdout分類GA_HOLDOUT_A
- Existing Y dataset缺少必要的速度／曲率／regime support
- Existing Y sequence的feature convex hull不足

Generation B不得因：

- 想取得更好分數
- 想重試不同seed
- 想修改holdout結果
- 想重新選lambda

而啟動。

Generation B包含：

1. Residual coverage sequence specification。
2. Controller implementation及qualification。
3. Fresh training／holdout dataset。
4. 一次正式model derivation。
5. 一次holdout evaluation。

==================================================
Generation B Coverage Design
==================================================

這不是geometry identifiability search。

目標是覆蓋runtime-relevant feature space：

- 多個forward speed
- 多個angular speed
- 多個positive／negative curvature
- Straight
- Gentle arc
- Medium arc
- Tight safe arc
- In-place rotation
- Stop
- Acceleration／deceleration transition
- Left／right symmetry

只允許forward linear command及in-place rotation。

不得使用negative linear.x。

Coverage sequence必須：

- 在spawn-connected safe area
- 不修改world／robot／spawn
- Conservative clearance≥0.060 m
- Negative clearance=0
- Collision overlap=0
- 不需要truth feedback
- Controller只訂閱/odom及/clock
- Qualification資料永久排除

固定速度候選：

linear.x：

{
0.015,
0.025,
0.035,
0.045,
0.050
} m/s

angular.z magnitude：

{
0.08,
0.12,
0.18,
0.24,
0.30
} rad/s

Curvature combinations必須受安全geometry限制。

Coverage hard gates：

- Straight speed levels≥4
- Positive curvature bins≥4
- Negative curvature bins≥4
- Angular speed levels≥4
- Stop events≥6
- Left／right mirrored coverage imbalance≤5%
- Feature standardized condition≤1e5
- Feature convex-hull coverage明顯高於Generation A
- Nominal minimum clearance≥0.100 m
- Conservative minimum clearance≥0.060 m

只產生一個sequence candidate。

設計search在結果前freeze，
只允許一次search execution。

==================================================
Generation B Controller Qualification
==================================================

Temporary controller：

ResidualCoverageExcitationController

只允許訂閱：

- /odom
- /clock

只允許發布：

- /cmd_vel
- Phase-scoped state evidence

禁止：

- Truth
- JointState
- AMCL
- Scan
- Camera
- Contact
- Clearance feedback
- Route feedback

Qualification runtime：

G_RESIDUAL_COVERAGE_QUALIFICATION_01

只允許：

- 1 initial setup attempt
- 1 pre-lock setup retry
- 1 valid runtime
- Valid後不得重跑

要求：

- Fixed sequence完整
- Timeout=0
- Skip／repeat=0
- Negative linear=0
- Effective stop nonzero=0
- Abort=0
- Controller subscription allowlist PASS
- Unique /cmd_vel owner
- Observer drop=0
- Writer exception=0
- Dynamic／static TF完整
- Exact-stamp TF／odom一致
- Clearance≥0.060 m
- Collision=0
- Feature coverage達到candidate gate
- Qualification資料標記：
  RESIDUAL_COVERAGE_CONTROL_QUALIFICATION_ONLY_NOT_DATASET

PASS後進入Generation B dataset acquisition。

==================================================
Generation B Dataset
==================================================

全新runtime IDs：

Training：

G_RESIDUAL_ODOM_TRAIN_01

Holdout：

G_RESIDUAL_ODOM_HOLDOUT_01

兩個runtime必須：

- Independent cold start
- Same controller binary
- Same sequence
- Same observer
- Same queue config
- Same TF config
- Same writer
- Same environment
- No training-derived change applied toholdout
- No cross-runtime pairing
- No old samples

每個slot：

- 1 initial setup attempt
- 最多1次pre-lock setup retry
- 一個valid runtime後不得重跑

Training PASS後才啟動holdout。

Raw evidence：

- JointState
- Raw odom
- Dynamic TF
- Static TF
- Truth
- Clock
- Command
- Controller state
- Clearance
- Queue metrics

Quality gates：

- Observer callback exception=0
- Writer exception=0
- Queue overflow=0
- Drop=0
- Enqueued=written
- Raw readback PASS
- Joint mapping=100%
- Timestamp regression=0
- Canonical coverage≥98%
- Valid rows≥1500
- Exact-stamp TF coverage≥98%
- TF／odom exact disagreement≤1e-12
- Minimum clearance≥0.060 m
- Collision overlap=0
- Required feature coverage完整
- Left／right mirrored imbalance≤5%
- Training／holdout feature-envelope overlap≥90%
- Holdout仍保留獨立noise及trajectory差異

建立：

state_dependent_residual_odom_dataset.yaml

Status：

VERSIONED_STATE_DEPENDENT_RESIDUAL_ODOM_DATASET

固定partition不得交換或重切。

==================================================
Generation B Model Derivation
==================================================

使用完全相同的：

- Feature architecture
- R0至R4 model family
- Lambda grid
- Grouped CV
- Symmetry contract
- Selection rule
- Training gates
- OOD guard derivation

只使用：

G_RESIDUAL_ODOM_TRAIN_01

完成model selection及freeze。

Holdout：

G_RESIDUAL_ODOM_HOLDOUT_01

只作freeze後validation。

Generation B holdout gates：

- Weighted SSE優於nominal
- Yaw RMSE改善≥20%
- Translation RMSE改善≥15%
- Integrated final position改善≥15%
- Integrated final yaw改善≥25%
- 任一regime不得惡化>10%
- Left／right symmetry通過
- OOD fraction≤3%
- Correction magnitude bounded
- Residual autocorrelation下降
- No truth runtime dependency

若通過建立：

state_dependent_residual_odom_candidate.yaml

Status：

DERIVED_STATE_DEPENDENT_RESIDUAL_ODOM_CANDIDATE_UNIMPLEMENTED

若仍失敗：

不得切換新model family。

停止為：

PROVEN_RESIDUAL_MODEL_GENERALIZATION_BLOCKER

==================================================
State-Dependent Covariance
==================================================

只使用正式candidate的training residual。

Covariance不得為全零。

建立固定motion regimes：

- STOP
- STRAIGHT_LOW_SPEED
- STRAIGHT_HIGH_SPEED
- TURN_LOW_CURVATURE
- TURN_MEDIUM_CURVATURE
- TURN_HIGH_CURVATURE
- IN_PLACE_ROTATION
- DEGRADED_SUPPORT

每個regime建立：

- Residual mean audit
- 3×3 increment covariance
- PSD projection
- Positive numerical floor
- Cross-correlation
- Sample count
- Support bounds

不得使用holdout估計covariance。

Holdout只驗證：

- Finite
- Symmetric
- PSD
- Positive diagonal
- 95% predictive coverage合理
- 99% predictive coverage≥95%
- No extreme interval inflation
- OOD covariance大於in-support covariance
- Stop covariance不為0

輸出：

- residual_increment_covariance.npy
- residual_covariance_contract.yaml
- covariance_regime_inventory.csv
- holdout_covariance_validation.csv

==================================================
Implementation Phase
==================================================

建立temporary package：

/home/ray/ros2_racecar/src/wro2026_phase7g_residual_odom_tmp/

Node：

StateDependentResidualSE2Odom

固定runtime流程：

1. 收到JointState。
2. Name-based取得左右輪position。
3. 處理unwrap。
4. 檢查timestamp。
5. 建立nominal differential-drive increment。
6. 建立runtime features。
7. 執行support guard。
8. 執行frozen residual model。
9. Blend correction。
10. Compose corrected SE(2) increment。
11. Propagatepose。
12. Propagate covariance。
13. 對齊raw odom correction reference。
14. 發布/odom_residual_calibrated。
15. 發布odom_residual_calibrated→odom。

必須完成unit tests：

- Joint names
- Ordering
- Position unwrap
- Duplicate timestamp
- Out-of-order timestamp
- Missing wheel
- Missing velocity
- Clock rewind
- New epoch
- Reset
- Straight
- Left rotation
- Right rotation
- Left arc
- Right arc
- Stop
- Feature computation
- Curvature clipping
- Symmetry
- Model serialization
- Candidate SHA
- OOD identity fallback
- OOD covariance inflation
- Blend continuity
- Maximum correction
- Finite output
- Covariance PSD
- No zero covariance
- TF correction identity
- No duplicate child
- No TF loop
- Forbidden subscription audit
- Training replay
- Holdout replay
- Rollback toidentity model

Offline replay必須重現candidate freeze metrics。

不得修改candidate coefficients。

==================================================
TF Architecture
==================================================

Raw：

odom→base_footprint

Residual correction：

odom_residual_calibrated→odom

AMCL future：

map→odom_residual_calibrated

完整chain：

map
→ odom_residual_calibrated
→ odom
→ base_footprint

Correction transform：

T_residual_odom_to_odom
=
T_corrected_base × inverse(T_raw_base)

要求：

- base_footprint仍只有raw odom parent
- odom只有residual correction parent
- odom_residual_calibrated由AMCL作global child
- Duplicate child=0
- TF loop=0
- Timestamp一致
- Clock rewind建立新epoch
- Reset清空state
- Old epoch output=0
- Missing JointState停止更新
- No stale correction
- Raw odom source不受影響

==================================================
Runtime Qualification
==================================================

Runtime ID：

G_RESIDUAL_ODOM_RUNTIME_QUALIFICATION_01

只允許：

- 1 initial setup attempt
- 1 pre-lock setup retry
- 1 valid runtime
- Valid後不得重跑

不啟動AMCL。

驗證：

- Candidate SHA正確
- Model binary／config SHA正確
- Inputs只含/joint_states、/odom、/clock
- Forbidden subscriptions=0
- Outputs正確
- TF ownership正確
- Duplicate child=0
- TF loop=0
- map-related TF=0
- Timestamp regression=0
- Epoch contamination=0
- Queue drop=0
- Exception=0
- Coverage≥98%
- Covariance finite／PSD／nonzero
- Offline／runtime replay差在固定numerical tolerance
- Truth只作observer
- Runtime improvement符合candidate預測範圍
- OOD behavior可驗證
- Correction magnitude不爆炸

Qualification資料永久排除於AMCL tuning及dataset。

PASS後進入AMCL Level 1。

==================================================
AMCL Level 1
==================================================

建立temporary AMCL candidate config。

只修改：

odom_frame_id: odom_residual_calibrated

保持：

- global_frame_id=map
- Existing base frame
- Baseline alpha1–alpha5
- 其他baseline AMCL參數
- Map
- Sensor config
- Particle settings
- Frozen thresholds

Baseline alpha不得重新擬合。

TF ownership：

AMCL唯一發布：

map→odom_residual_calibrated

Residual node唯一發布：

odom_residual_calibrated→odom

Raw source唯一發布：

odom→base_footprint

Runtime ID：

G_RESIDUAL_ODOM_AMCL_LEVEL1_01

只允許：

- 1 initial setup attempt
- 1 pre-lock setup retry
- 1 valid runtime
- Valid後不得重跑

Frozen localization gates：

- Truth／estimate coverage≥95%
- Position error≤0.15 m的samples≥95%
- Yaw error≤0.30 rad的samples≥95%
- Maximum position error≤0.25 m
- Maximum yaw error≤0.500 rad
- Truth interruption=0
- Estimate outside map／field=0
- Final stop 2 s後position≤0.10 m
- Final stop 2 s後yaw≤0.20 rad

另外要求：

- Particle depletion=0
- TF timestamp regression=0
- Correction regression=0
- Old epoch transform=0
- Duplicate owner=0
- TF loop=0
- Truth runtime subscription=0
- Route unchanged
- Production controller unchanged

若PASS：

立即執行Phase 7F。

若FAIL分類：

G_AMCL_FAIL_A —
RESIDUAL_ODOM_IMPLEMENTATION_DEFECT

G_AMCL_FAIL_B —
TF_OR_TIMESTAMP_DEFECT

G_AMCL_FAIL_C —
SUPPORT_GUARD_OR_COVARIANCE_DEFECT

G_AMCL_FAIL_D —
RESIDUAL_MODEL_GENERALIZATION_DEFECT

G_AMCL_FAIL_E —
AMCL_MOTION_MODEL_ARCHITECTURE_BLOCKER

A／B／C存在唯一repair時可自主建立新repair phase及新runtime ID。

D：

不得使用該localization runtime回頭重新訓練同一candidate。

若存在Generation B未使用的正式training-only資料與
預先允許的固定repair，才可繼續；
否則停止。

E：

不得修改alpha或放寬gate。

停止為：

PROVEN_RESIDUAL_ODOM_LOCALIZATION_ARCHITECTURE_BLOCKER

==================================================
Phase 7F Final Acceptance
==================================================

只有AMCL Level 1全部通過才可執行。

Runtime ID：

G_RESIDUAL_ODOM_FINAL_ACCEPTANCE_01

只允許：

- 1 initial setup attempt
- 1 pre-lock setup retry
- 1 valid runtime
- Valid後不得重跑

使用byte-identical：

- Residual model candidate
- Feature scaler
- OOD guard
- Covariance
- Correction node
- AMCL candidate
- World
- Robot
- Map
- Route
- Controller

Level 1至7F之間不得修改：

- Model level
- Lambda
- Coefficients
- Feature scaler
- Support guard
- Covariance
- AMCL alpha
- Localization thresholds

Final gates：

Identity：

- Candidate SHA正確
- Model artifact SHA正確
- Covariance SHA正確
- Binary／config SHA正確
- World／robot／map／route SHA正確
- Production controller SHA正確
- Baseline AMCL未覆寫

Runtime：

- Single cold start
- No valid rerun
- Setup retry合法
- Process／zombie最終0/0
- No exception
- No observer drop
- No TF loop
- No duplicate TF owner
- No old epoch data
- No truth runtime correction

Localization：

- Coverage≥95%
- Position≤0.15 m至少95%
- Yaw≤0.30 rad至少95%
- Max position≤0.25 m
- Max yaw≤0.500 rad
- Final stop 2 s position≤0.10 m
- Final stop 2 s yaw≤0.20 rad
- Outside-map estimate=0
- Truth interruption=0
- Pose teleport=0

Route／safety：

- Frozen route完整
- No waypoint skip
- Controller abort=0
- Collision overlap=0
- Minimum clearance符合route gate
- Unauthorized command owner=0

通過分類：

7F_RESIDUAL1 —
FINAL_STATE_DEPENDENT_ODOM_LOCALIZATION_ACCEPTANCE_PASS

==================================================
Final Promotion
==================================================

只有7F_RESIDUAL1成立才執行。

允許新增正式package：

/home/ray/ros2_racecar/src/wro2026_residual_odom/

允許新增：

- Node source
- Feature contract
- Model artifact
- Scaler
- OOD guard config
- Covariance
- Package metadata
- Unit tests
- Replay tests
- Launch integration
- Independent AMCL candidate config
- Documentation

不得覆寫baseline AMCL。

Promotion後：

- Clean repo-local build
- Unit tests
- Offline training replay
- Offline holdout replay
- TF tests
- Covariance tests
- Launch parse
- Forbidden subscription audit
- Candidate identity
- git diff --check
- Process／zombie=0

最後才允許更新一次：

- docs/CURRENT_STATE.md
- docs/TEST_EVIDENCE.md
- docs/NEXT_PROMPT.md

CURRENT_STATE標記：

FINAL_STATE_DEPENDENT_ODOM_LOCALIZATION_ACCEPTED

不得刪除既有constant geometry blocker歷史。

==================================================
Campaign Phases
==================================================

建立Campaign root：

/home/ray/ros2_evidence/phase7g_residual_odom_campaign_<YYYYMMDD_HHMMSS>

預定subroots：

01_7ga_residual_forensic/
02_7gb_generation_a_model/
03_7gc_generation_a_holdout/
04_7gd_coverage_design/
05_7ge_controller_qualification/
06_7gf_generation_b_dataset/
07_7gg_generation_b_model/
08_7gh_residual_implementation/
09_7gi_runtime_qualification/
10_7gj_amcl_level1/
11_7f_final_acceptance/
12_final_promotion/

僅實際執行的phase需要建立。

Repair：

<phase>_repair_<n>/

每個subroot至少：

- phase_contract.md
- phase_summary.md
- classification.json
- source_identity.md
- source_manifest_validation.txt
- decision_log.csv
- offline_gate_results.csv
- runtime_attempt_inventory.csv，如適用
- cleanup_verification.md
- final_verification.json
- complete_sha256.txt

Campaign root至少：

- campaign_goal.md
- campaign_authorization.md
- campaign_state.yaml
- campaign_phase_inventory.csv
- campaign_decision_log.csv
- campaign_runtime_inventory.csv
- campaign_model_inventory.csv
- campaign_dataset_inventory.csv
- campaign_candidate_inventory.csv
- campaign_frozen_artifact_inventory.csv
- campaign_git_before.txt
- campaign_git_after.txt
- campaign_final_report.md
- campaign_final_classification.json
- campaign_complete_sha256.txt

==================================================
持續執行及Repair規則
==================================================

每個phase完成後：

1. 寫phase summary。
2. 寫classification。
3. 建manifest。
4. Process／zombie確認。
5. Cleanup。
6. 更新campaign state。
7. 根據本Master Goal立即進入下一Phase。

不要在中間phase停下詢問。

只有以下情況停止：

STOP_FINAL_PASS

STOP_CONSTANT_MODEL_FAMILY_BLOCKER

STOP_RESIDUAL_GENERALIZATION_BLOCKER

STOP_LOCALIZATION_ARCHITECTURE_BLOCKER

STOP_IDENTITY_FAILURE

STOP_FROZEN_CHANGE_REQUIRED

STOP_NO_UNIQUE_REPAIR

STOP_RUNTIME_BUDGET_EXHAUSTED

一般implementation問題不得停止。

==================================================
Runtime Lock及Retry
==================================================

每個runtime slot：

- 1 initial setup attempt
- 最多1次pre-lock setup retry

Pre-lock條件：

以下事件均尚未發生：

- 正式dataset acquisition
- 第一筆正式raw sample
- 第一筆正式pairing
- 第一筆正式controller state
- 第一個nonzero command
- 第一筆正式localization evaluation

任一發生：

retry_allowed=false

Valid runtime不得重跑。

不得：

- 為改善指標重跑
- 為挑seed重跑
- 修改candidate後重跑同一ID
- 修改threshold後重跑
- 刪除不利sample
- 交換training／holdout
- 將setup evidence混入正式dataset

==================================================
Cleanup及Rollback
==================================================

每個phase結束：

- 單次正常SIGINT
- Process=0
- Zombie=0
- 移除phase-scoped hooks
- 移除temporary source
- 移除phase build／install／log
- 驗證/home/ray/ros2_ws未變
- 驗證production controller cmp=0
- 驗證frozen artifact SHA
- git diff --check=0
- 建立complete_sha256
- sha256sum -c PASS

Final promotion前：

Repository必須回到Campaign開始狀態，
除了本Master Goal所在的NEXT_PROMPT.md。

若停止：

- 不更新CURRENT_STATE.md
- 不更新TEST_EVIDENCE.md
- 不promotion
- 清除temporary implementation
- 保留全部persistent evidence

==================================================
Campaign最終必須回答
==================================================

1. Campaign final classification。
2. 停止條件。
3. 完成哪些phase。
4. 每個phase classification。
5. 每個phase evidence root。
6. 每個manifest結果及SHA。
7. Constant geometry blocker是否保持封存。
8. Production controller是否保持。
9. World／robot／spawn／map／route是否保持。
10. Generation A feature contract。
11. Generation A selected model level及lambda。
12. Generation A CV結果。
13. Generation A holdout結果。
14. 是否進入Generation B。
15. Generation B觸發原因。
16. Coverage sequence candidate path及SHA。
17. Qualification runtime結果。
18. Qualification資料是否排除。
19. Generation B training／holdout IDs。
20. Generation B dataset path及SHA。
21. Model architecture。
22. Feature scaler SHA。
23. Coefficient artifact SHA。
24. OOD guard contract。
25. Holdout translation改善。
26. Holdout yaw改善。
27. Integrated final pose改善。
28. Symmetry結果。
29. Regime-specific結果。
30. Candidate path及SHA。
31. Covariance path及SHA。
32. Covariance holdout coverage。
33. Implementation tests。
34. Offline replay結果。
35. Runtime qualification結果。
36. TF ownership結果。
37. AMCL Level 1結果。
38. Phase 7F結果。
39. Frozen 0.500 rad gate是否保持。
40. AMCL alpha是否未修改。
41. Truth是否未作runtime input。
42. /home/ray/ros2_ws是否未變。
43. Temporary artifacts是否清除。
44. Process／zombie是否0/0。
45. Final build結果。
46. Git狀態及authorized diff。
47. Promoted files。
48. CURRENT_STATE／TEST_EVIDENCE是否更新。
49. 最終可用能力。
50. 尚存限制。
51. Rollback方式。
52. 若停止，唯一不可繼續的精確阻塞。
53. Campaign manifest是否100% PASS。

==================================================
立即執行
==================================================

不要只寫計畫。

立即：

1. 建立Phase 7G campaign evidence root。
2. 驗證最新Campaign、Y dataset及production identities。
3. 封存constant geometry blocker。
4. 封存repository及frozen artifacts開始狀態。
5. 執行7G-A residual forensic。
6. 執行Generation A training-only model selection。
7. Freeze後執行Generation A holdout。
8. 依固定分類自動選擇直接implementation或Generation B。
9. 持續執行至runtime、AMCL及Phase 7F。
10. 遇到唯一implementation repair時自行修正並繼續。
11. 不要在中間Phase完成後等待使用者。
12. 不得為耗額度重複無效工作。
13. 不得放寬gate或重跑valid runtime。
14. 最後只提交一次完整Campaign報告。

除本Prompt列明的合法停止條件外，持續往前執行。