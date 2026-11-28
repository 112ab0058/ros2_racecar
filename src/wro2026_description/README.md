# wro2026_description

R550 mini akm 的 URDF：記錄**感測器裝在車上的哪裡**，由 `robot_state_publisher` 發成 TF。這個 package 不碰任何硬體。

## 為什麼需要

光達量到的距離是以**光達自己**為原點（`laser_link`），里程計算出來的是**車子**的位置（`base_link`）。要把光達的點畫在地圖上，ROS 必須知道光達在車上的哪裡，也就是 `base_link → laser_link` 這個轉換。slam_toolbox、Nav2 都靠這棵 TF 樹。

```
odom ──(wro2026_base_r550 的 /odom，會動)──> base_link ──(這個 package，固定)──> laser_link
                                                       ├──> imu_link
                                                       └──> base_footprint
```

## 座標與數值

`base_link` = **後輪軸中心**（阿克曼車的慣例：轉彎時後軸中心不會側滑）。x 往前、y 往左、z 往上（REP-103）。

| frame | 相對 base_link（m） | 來源 |
|---|---|---|
| `base_footprint` | 0, 0, −0.035 | 輪子直徑 7 cm → 後軸離地 3.5 cm（2026-10-16 量） |
| `laser_link` | 0.13, 0, 0.115 | 光達中心在後軸前約 13 cm（2026-10-17 量）；左右偏移沒量，假設在中線；掃描面離地 15 cm − 3.5 cm |
| `imu_link` | 0, 0, 0 | **UNKNOWN**，暫放原點。2D 車只用 gyro z，位置影響很小；軸向假設和車子一致，**未驗證** |

`laser_link` 不需要旋轉：driver 已經把光達 0°（車頭）轉成 ROS 的 +x。

## 使用（Orin）

```bash
ros2 launch wro2026_description description.launch.py
ros2 run tf2_ros tf2_echo base_link laser_link    # 應該看到 Translation: [0.130, 0.000, 0.115]
```

改了量測值：只改 `urdf/r550.urdf` 和 `test/test_urdf.py` 裡對應的數字，重新 build。
