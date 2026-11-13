# wro2026_imitation

AI 方向 B（模仿學習）：人遙控開車，記錄「光達掃描 → 左／直／右」，訓練小型神經網路照著開。純 numpy，不需要 torch。

這個 package **不碰串口**。只有 `bc_node` 加上 `publish_cmd:=true` 會發 `/cmd_vel`，而且要 `base_node` 開 `enable_commands:=true` 馬達才會轉，base_node 的限速、0.3 s 逾時、限時照樣有效。

| 檔案 | 用途 |
|---|---|
| `features.py` | 光達掃描變成網路的輸入：前方 ±110°，每 5° 一格，共 44 格，每格是最近障礙物距離 ÷ 3 m |
| `dataset.py` | 每一圈掃描對到當下的指令，得到標籤；依時間切訓練和驗證；錄影時的速度和轉彎半徑 |
| `model.py` | 小型神經網路（44-32-16-3）、Adam 訓練、存讀檔 |
| `extract.py` | `bc_extract`：從 `orin_map.sh` 的 rosbag 取出訓練資料 |
| `train.py` | `bc_train`：訓練並印出準確率；模型檔裡記下錄影時的速度和半徑 |
| `policy.py` | 安全規則 + 兩種策略：`model`（網路）、`center`（走中間，對照組） |
| `bc_node.py` | `bc_node`：讀 `/scan`，發 `bc/decision`；`publish_cmd:=true` 才發 `/cmd_vel` |

```bash
ros2 run wro2026_imitation bc_extract ~/r550_map/<time>
ros2 run wro2026_imitation bc_train ~/r550_map/*/dataset.npz -o ~/r550_map/model.npz
```

**資料切法**：相鄰的掃描幾乎一樣，隨機切會把幾乎相同的資料同時放進訓練和驗證，分數會虛高。所以依時間切：每 3 秒一段，每 5 段取 1 段當驗證，訓練資料要離驗證段 0.5 秒以上。

**鏡像**：只有訓練資料左右翻轉一份（輸入反過來、左右標籤對調），驗證資料不翻。光達遮掉自己車子的角度在 168–243°，在 ±110° 之外，所以不會進到輸入。

## bc_node

```bash
# 只看它會怎麼開（預設，不發 /cmd_vel）；另一個終端機要先開 sensors.launch.py
ros2 run wro2026_imitation bc_node --ros-args -p model:=$HOME/r550_map/model.npz
ros2 run wro2026_imitation bc_node --ros-args -p policy:=center
```

在 Foxglove 看 `/bc/decision`：-1 停、0 左、1 直、2 右。`/bc/probs` 是網路對左／直／右的機率。

每 0.05 s 依序檢查，第一個成立的決定結果：

1. **太近（too_close）**：任何光達點進到車身外框再加 4 cm 內（車頭在光達前 0.11 m、半寬 0.09 m）→ 停，**鎖住**到重開 bc_node。
2. **沒有掃描（no_scan）**：最新掃描超過 0.3 s → 停。
3. **掃描格式不對（bad_scan）**：不是從 0° 開始轉一整圈 → 停。
4. 都沒事 → 問策略：左／直／右 → `vx = 速度`、`wz = ±速度 ÷ 半徑`。

**速度和半徑**預設用模型檔裡記的（錄影時用的值），所以網路說「左」時開出來的彎跟它學的一樣。`center` 預設 0.1 m/s、0.5 m。速度超過 0.15 m/s 或半徑小於 0.35 m 會拒絕啟動（0.35 = 方向盤打到底，實際 0.40 m）。數字參數要寫小數點（`speed:=0.10`，不是 `1`）。

**走中間規則**：左邊（+30～+90°）和右邊（-90～-30°）各取最近的牆。前方 ±15° 比 0.5 m 近時，轉向比較空的一邊；不然哪邊的牆近 5 cm 以上就往另一邊轉，否則直走。

**停止**：Ctrl+C、`kill -INT`、`kill -TERM` 都會先送 5 個 0 再結束。程式自己裝了訊號處理，因為腳本在背景啟動的程式會繼承「忽略 SIGINT」（2026-11-10 在 WSL 量到）。

**已知限制**：比 `range_min`（0.1 m）更近的東西光達量不到，輸入會當成很遠；黑色物體看不到。
