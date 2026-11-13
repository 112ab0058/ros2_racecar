# Orin 備份與還原

更新時間：2026-10-13。兩種備份：

| 種類 | 內容 | 誰做 | 還原後的狀態 |
|---|---|---|---|
| **狀態備份**（這份文件第 1–2 節） | 裝了哪些套件、apt 來源、系統版本、`~/.bashrc`、我們的原始碼 | AI 自動做，不需要 sudo | 要重新刷系統、照清單重裝，**不會** byte 相同 |
| **SD 卡映像**（第 3 節） | 整張 microSD 的完整拷貝 | Ray 自己做（要關機拔卡） | 和備份當下完全相同 |

裝大量套件、改系統設定之前，最好兩種都有。狀態備份只能告訴你「原本是什麼樣子」，真正能一步回到原狀的是 SD 卡映像。

## 1. 狀態備份的內容

2026-10-13 12:05 的備份（在 B2 安裝 ROS 套件之前）：

- Jetson：`~/orin_backup_20261013_120554/`、`~/orin_backup_20261013_120554.tar.gz`
- Windows：`%USERPROFILE%\orin_backups\orin_backup_20261013_120554.tar.gz`（不在 repo 裡）
- SHA-256：`19b50b5dc3473fe530db4ce1d348e25a8779245e2968628a3bd025530d4e1911`（兩邊一致）

| 檔案 | 來源指令 / 內容 |
|---|---|
| `dpkg_get_selections.txt` | `dpkg --get-selections`（2409 項） |
| `apt_mark_showmanual.txt` | `apt-mark showmanual`（487 個手動安裝的套件） |
| `apt_list_installed.txt` | `apt list --installed`（含版本） |
| `apt_sources/` | `/etc/apt/sources.list`、`/etc/apt/sources.list.d/` 底下可讀的檔案：`ubuntu.sources`、`ros2.sources`、`nvidia-l4t-apt-source.list`、`netbird.list`（裡面的 PGP 是公開金鑰） |
| `ros2_pkg_list.txt` | `ros2 pkg list`（198 個） |
| `nv_tegra_release.txt`、`uname_a.txt`、`os_release.txt` | L4T R39.2.1、kernel、Ubuntu 24.04 |
| `nvpmodel_q.txt` | `nvpmodel -q`：MAXN_SUPER（mode 2） |
| `lsblk.txt`、`df_h.txt` | 分區：microSD 58 GB、15 個分區，`/` 是 `mmcblk0p1`（56.5 GB） |
| `bashrc.txt` | `~/.bashrc` |
| `ros_env.txt`、`python_version.txt` | ROS 環境變數（`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`）、Python 版本 |
| `ros2_ws_src.tar.gz` | `~/ros2_ws/src`（舊 driver `wave_rover_driver`，含它的本機 git） |
| `racecar_ws_src.tar.gz` | `~/racecar_ws/src`（不含 build / install / log） |

**刻意不備份**：`~/.ssh`、NetworkManager 的連線設定（裡面有 Wi-Fi 密碼）、任何私鑰、build / install / log。建立時用 grep 檢查過**文字檔**，沒有 Wi-Fi 金鑰、密碼或私鑰的內容；兩個原始碼壓縮檔只檢查了檔名（沒有金鑰類檔名），沒有逐檔掃描內容（`ros2_ws_src.tar.gz` 含舊 driver 的本機 `.git`）。

再做一次（Jetson，不需要 sudo）：

```bash
source /opt/ros/jazzy/setup.bash
B=~/orin_backup_$(date +%Y%m%d_%H%M%S); mkdir -p $B/apt_sources; cd $B
dpkg --get-selections > dpkg_get_selections.txt
apt-mark showmanual > apt_mark_showmanual.txt
apt list --installed > apt_list_installed.txt
cp /etc/apt/sources.list /etc/apt/sources.list.d/* apt_sources/ 2>/dev/null
ros2 pkg list > ros2_pkg_list.txt
cp /etc/nv_tegra_release nv_tegra_release.txt; uname -a > uname_a.txt; cat /etc/os-release > os_release.txt
nvpmodel -q > nvpmodel_q.txt; lsblk > lsblk.txt; df -h > df_h.txt; cp ~/.bashrc bashrc.txt
printenv | grep -E '^(ROS_|RMW_|AMENT_PREFIX_PATH|COLCON)' > ros_env.txt; python3 --version > python_version.txt
tar czf ros2_ws_src.tar.gz -C ~ --exclude=__pycache__ ros2_ws/src
tar czf racecar_ws_src.tar.gz -C ~ --exclude=__pycache__ racecar_ws/src
cd ~ && tar czf $(basename $B).tar.gz $(basename $B) && sha256sum $(basename $B).tar.gz
```

Windows（PowerShell）拉回：`scp user2@192.168.0.15:orin_backup_<時間>.tar.gz $env:USERPROFILE\orin_backups\`。

## 2. 用狀態備份重建（只有在 SD 卡壞掉、又沒有映像時才用）

以下指令都是**範例，沒有執行過**；需要 sudo 的由 Ray 親自執行。

1. 用 NVIDIA 的方式重新刷 Jetson Orin Nano：L4T **R39.2.1**、Ubuntu 24.04（版本以 `nv_tegra_release.txt`、`os_release.txt` 為準）。NVIDIA 的套件（`nvidia-l4t-*`）由刷機帶進來，**不要**照清單用 apt 手動裝。
2. 建立使用者 `user2`、主機名稱 `112ab-jno`，重新設定網路與 NetBird（設定沒有備份，要重新加入）。
3. 還原 apt 來源：比對 `apt_sources/` 和新系統的 `/etc/apt/sources.list.d/`，把缺的（例如 `ros2.sources`、`netbird.list`）放回去，然後 `sudo apt-get update`。
4. 重裝手動安裝的套件（排除 NVIDIA 的套件）：

```bash
grep -vE '^(nvidia|cuda|libcuda|cudnn|libcudnn|libnv|tensorrt|libnvinfer|l4t|jetson|vpi|libvpi|nsight|linux-)' apt_mark_showmanual.txt > to_install.txt
apt-get -s install --no-upgrade $(cat to_install.txt) > sim.txt   # 先模擬，不需要 sudo
grep -c '^Remv' sim.txt                                            # 必須是 0
grep -E '^(Inst|Remv) (nvidia|cuda|tensorrt|libnv|linux-)' sim.txt # 必須沒有輸出
sudo apt-get install --no-upgrade $(cat to_install.txt)            # 上面兩項都通過才執行
```

   裝完用 `apt list --installed` 和 `apt_list_installed.txt` 比對版本；apt 會裝**現在的**版本，不一定和備份當下相同。
5. `cp bashrc.txt ~/.bashrc`。
6. 原始碼：`tar xzf ros2_ws_src.tar.gz -C ~`、`tar xzf racecar_ws_src.tar.gz -C ~`，然後在兩個 workspace 各自 `colcon build`。我們的程式以 GitHub 上的 repo 為準，`racecar_ws_src` 只是當時的快照。
7. `nvpmodel -q` 確認電源模式（原本 MAXN_SUPER）；**不要**為了符合而去改它，改電源模式要另外和 Ray 確認。

## 3. 完整 SD 卡映像（Ray 自己做；這裡只寫步驟，沒有執行過）

需要：讀卡機、Windows 上的可用空間至少要等於整張卡的位元組數（`lsblk` 顯示的 58 G 是 GiB，約 62.3 GB；建議預留 64 GB 以上；如果工具支援壓縮可以小很多）、映像工具 [USBImager](https://gitlab.com/bztsrc/usbimager) 或 [Win32 Disk Imager](https://sourceforge.net/projects/win32diskimager/)。

### 3.1 做映像

1. Jetson：`sudo shutdown -h now`，等它完全關機後拔電源，再取出 microSD。
2. 把卡插進 Windows 的讀卡機。**Windows 若跳出「需要格式化磁碟」一律按取消**（Jetson 的分區 Windows 讀不懂，格式化會毀掉系統）。
3. 開映像工具：
   - USBImager：選這張卡（看容量確認是 58 GB 左右那顆，不要選錯硬碟），按 **Read**，存成例如 `orin_sd_20261013.img`。
   - Win32 Disk Imager：Image File 填存檔路徑，Device 選這張卡的任一個磁碟代號（它會讀整個裝置）；**不要勾** "Read Only Allocated Partitions"（Jetson 有 15 個分區，要整張讀）；按 **Read**。
4. 做完算雜湊並記下來：`certutil -hashfile orin_sd_20261013.img SHA256`（Windows 命令提示字元）。
5. 退出讀卡機，把卡插回 Jetson，開機確認正常。

### 3.2 還原

1. 準備一張**容量相同或更大**的 microSD（標示 64 GB 的卡實際容量不一定比 58 GB 大，要看實際位元組數）。
2. USBImager：選映像檔、選目標卡、按 **Write**（勾 Verify）；Win32 Disk Imager：選映像檔和卡、按 **Write**，再按 **Verify Only**。
3. 插回 Jetson 開機，確認 `cat /etc/nv_tegra_release`、`ros2 pkg list | wc -l` 和備份當時一致。

### 3.3 注意

- 映像裡有 Wi-Fi 密碼、SSH 金鑰等所有資料，**不要上傳到雲端或放進 repo**。
- 映像要在 Orin 關機狀態下做；開機中直接讀可能得到不一致的檔案系統。
