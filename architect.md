# ProblemD_SA_prime 架構說明

依據 `plan.txt` 與 ICCAD 2026 Problem D（`D_20260212.pdf`）。

## 1. 競賽目標（摘要）

- 固定 data path，僅能透過 clock tree **插入 buffer** 與 **調整 buffer 尺寸** 改善 timing。
- Setup 看 SS corner（WNS/TNS setup），Hold 看 FF corner（WNS/TNS hold）。
- 評分同時考慮 WNS、TNS、Area 相對初始設計的改善。
- **每測資 10 分鐘**；輸出 `modified_clk_tree.structure`。

## 2. 總體流程

```
讀檔 (Phase0 C)
    → 建立 branch 模型（每條 parent→child 邊）
    → 建立 SS/FF buffer-chain DP 表
    → Phase A: 多目標 LP（30s）→ 初始 delay 向量
    → Phase B: Path-heap SA（至總時限 600s）
    → DP 將 delay 映射為 cell → 寫出結果
```

執行檔：`sa_solver`  
環境變數：`SA_TIME_LIMIT=600`（總時限）、`LP_INIT_TIME_LIMIT=30`（LP 階段）

## 3. 資料結構

| 元件 | 說明 |
|------|------|
| `LpBranch` | 時鐘樹邊：ExistingBuf（既有 buffer）或 Insertable（parent→FF，可視為可插 buffer） |
| `d_ss[b]`, `d_ff[b]` | 邊 b 在 SS/FF corner 的等效延遲（連續決策變數） |
| `LpBufferChainDp` | 對 (fanout n, delay d) 查最小 area 與 buffer 鏈 |
| `SaPgCtx` | 前綴和 timing：每 FF 的 T_ss/T_ff、每 path 的 slack |

## 4. Phase A — 多目標 LP 暖啟動（`lp_mo_init.cpp`）

**變數**：每條 branch 的 `d_ss`、`d_ff`。

**約束**（投影）：
- `d_ss ∈ [d_ss_min, d_ss_max]`、`d_ff ∈ [d_ff_min, d_ff_max]`（由 buf.lib 在該 fanout 下 min/max 決定）
- Insertable 邊允許 delay = 0（不插入）

**目標**：最小化 WNS/TNS 違例（SS setup + FF hold 多目標）
- 對每條 violating path，依負 slack 大小加權梯度
- Capture 側分支：增加 delay → 改善 setup
- Launch 側分支：減少 delay（或增加 launch 延遲修 hold）→ 改善 hold

**演算法**：投影梯度法，牆鐘限時 `LP_INIT_TIME_LIMIT`（預設 30s）。

**失敗處理**：若 LP 未產出有效解，SA 初始狀態使用原始 clock tree 的 cell delay（`sa_init_from_design`）。

## 5. Phase B — Path-heap SA（`sa_path_solve.cpp`）

**初始狀態**：LP 輸出或原始 tree。

**Path 優先佇列**（最大堆）：
- 權重 `w(p) = max(0, -slack_ss(p)) + max(0, -slack_ff(p))`
- 僅 violating path 入堆

**每次迭代**：
1. 自堆取出 **10 個最差** path
2. 自 violating 集合再抽 **10 個隨機** path
3. 合併為候選池（最多 20），**均勻隨機選 1 條** path
4. 對該 path 做啟發式 move：
   - Setup violation → 在 capture→FF 路徑上選一邊，增大 `d_ss` 或略減 `d_ff`
   - Hold violation → 在 launch→FF 路徑上選一邊，增大 `d_ff` 或略減 `d_ss`
   - 新 delay 取自 DP 候選離散表（各 cell 延遲 + min/max）
5. Metropolis 接受（最大化競賽 Score）
6. 維護歷史 **best** 解

**時限**：`elapsed(wall_t0) < SA_TIME_LIMIT`（預設 600s），含 LP 已用時間。

## 6. Delay → Buffer（`sa_apply.cpp` + DP）

對每條 ExistingBuf 邊：
- 以 `d_ss` 查 `DP_SS(fanout, d)`，取鏈上 **最後一顆** cell 寫入節點
- 評分時 area/delay 由 DP 表給出

（尚未在 structure 物理插入 NEW_BUF 鏈；Insertable 邊 delay 參與評分但未改拓撲。）

## 7. 評分（`lp_score.cpp`）

與競賽一致（α=β=γ=1）：

```
Score = Σ (1 - 改善項/ori項)  over TNS_SS+, WNS_SS+, TNS_FF+, WNS_FF+, Area
```

SA/LP 內部以 `lp_compute_score` 為純量目標。

## 8. 輸出 `result.txt`

| 欄位 | 說明 |
|------|------|
| `time_limit_sec` | 總時限（600） |
| `lp_init_sec` / `lp_init_ok` | LP 階段耗時與是否成功 |
| `wall_elapsed_sec` | 程式總時間 |
| `sa_elapsed_sec` | SA 階段時間 |
| `sa_iterations` | SA 迭代次數 |
| baseline / after optimize | WNS、TNS、Area、Score |

## 9. 測試

```bash
bash scripts/run_timed_tc12.sh
```

- testcase1、testcase2
- 每案 `SA_TIME_LIMIT=600`，`timeout 615s`
- `wall_elapsed_sec > 600.5` 或 timeout → **FAIL**

## 10. 原始碼對照

| 檔案 | 職責 |
|------|------|
| `src/main.cpp` | 流程編排 |
| `src/lp_mo_init.cpp` | 30s 多目標 LP |
| `src/sa_path_solve.cpp` | Path-heap SA |
| `src/sa_eval.cpp` | Timing 前綴和、path 權重 |
| `src/lp_buffer_dp.cpp` | Buffer-chain DP |
| `src/lp_branch.cpp` | Branch 模型 |
| `src/sa_apply.cpp` | 寫回 cell |
| `src/lp_score.cpp` | 評分與 result.txt |

## 11. 後續擴充

- 在 structure 插入完整 NEW_BUF 鏈
- Insertable 邊的物理插入與 fanout 更新
- 無 violation 後的 area recovery
