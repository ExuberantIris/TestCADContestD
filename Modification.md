# Modifications - 0615

## 主要改動

### 1. 移除 SA Fallback 邏輯
- **移除**: SA degradation fallback to LP_init
- **原因**: SA 經常無法改善 LP_init 結果，反而多次使最終結果劣化
- **檔案**: `src/sa_path_solve.cpp`
- **影響**: SA 不再與 LP_init 比較，執行獨立優化

### 2. 新增 Greedy Post-LP Local Search
- **新檔案**:
  - `include/greedy_postlp.hpp` - 函數聲明
  - `src/greedy_postlp.cpp` - 實現檔案
- **集成位置**: `src/main.cpp` - LP_init 完成後、SA 開始前執行
- **輸出檔案**: `greedy_postlp_t540.txt` （不覆蓋 result.txt）

## Greedy Post-LP 策略詳解

### 執行流程
```
1. 從 LP_init 解開始
2. 迭代（每 pass 遍歷所有 branch）
3. 每 pass 內對每個 branch 嘗試 SS delay 和 FF delay 的候選值
4. 若找到改善就立即接受（first-fit greedy）
5. 時間限制 540 秒或不再改善則停止 (不硬性限制 pass 數)
```

### 核心約束 - Hold-Preserving
- **必須滿足**: `FF hold WNS >= LP_init baseline`
- **必須滿足**: `FF hold TNS >= LP_init baseline`
- 目的：確保 hold timing 不惡化，只改善 setup timing

### 優化指標（優先級）
1. **Setup Timing**（主要）: SS setup TNS 最小化
2. **Hold Timing**（約束）: FF hold WNS/TNS 不低於 LP_init
3. **面積**（次要）: 允許輕微增加以換取 timing 改善
4. **總分**: Score = setup_wns + setup_tns + hold_wns + hold_tns + area（加權）

### 候選值生成
- 每個 branch 維護一套 SS delay 和 FF delay 的候選集合
- 候選值來自 SA 分析（`sa_build_branch_opts`）
- 包含多個可能的延遲改變量

### 接受條件
```cpp
// 1. 不惡化 hold
if (trial_ctx.wns_ff < lp_init_metrics->wns_hold_ff ||
    trial_ctx.tns_ff < lp_init_metrics->tns_hold_ff)
    continue;  // 拒絕

// 2. 改善總分
if (trial_ctx.score > cur_score + 1e-12) {
    accept_and_break;  // 立即接受，移至下一個 branch
}
```

## 實驗結果範例 (testcase4)

| 指標 | Baseline | LP_init | After Greedy |
|-----|----------|---------|--------------|
| SS setup TNS | -2247.03 | -2247.03 | -1806.69 ↓ |
| FF hold WNS | -0.1178 | -0.1178 | -0.1178 (unchanged) |
| Total area | 10539.49 | 10539.49 | 10553.13 |
| **Score** | 0.0 | 0.01364 | **0.21965** ↑ |

**改善倍數**: 16.1x vs LP_init

## 時間配置

### 環境變數控制
- `SA_PHASE_TIME_LIMIT` (預設 0.11 sec) - 控制 SA 執行時間
- `LP_INIT_TIME_LIMIT` (預設 30 sec) - 控制 LP_init 時間
- Greedy 固定 30 秒時間限制（`main.cpp` 硬編碼）

### Makefile 支援
```bash
# 預設設定
make run-all

# 自訂 SA 時間
make SA_PHASE_TIME_LIMIT=60 run-all

# 單個 testcase
make SA_PHASE_TIME_LIMIT=1 TC=testcase2 run
```

## 管道執行順序
```
1. LP_init (0-30 sec)
   ↓
2. Greedy Post-LP (30 sec)
   ↓
3. SA Path-Solve (0.11 sec or configured)
   ↓
4. Output generation
```

## 檔案修改清單
- ✅ `src/greedy_postlp.cpp` - 新增
- ✅ `include/greedy_postlp.hpp` - 新增
- ✅ `src/main.cpp` - 新增 greedy 調用
- ✅ `Makefile` - 新增 SA_PHASE_TIME_LIMIT 變數及環境傳遞
- ✅ `include/greedy_postlp.hpp` - 函數聲明
