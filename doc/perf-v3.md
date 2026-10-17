# 性能优化报告 (v3)

对象: `include/slotsboxmalloc/slotsboxobj.h`（唯一的实现文件）。
环境: Intel Xeon w7-3455 (48 核), gcc 13.2, `-O3 -DNDEBUG`（与 CMake Release 相同）,
经共享库链接调用。数据区 128MB (`8*64^4`), meta 16MB。
所有数字为 5 次运行中位数; 吞吐按**成功**操作计数, 失败率单独列出。

## 基线 (v2) 的四个问题

1. **多线程假失败**: 128MB 数据区在 v2 只有一个 root slot（一把锁）, 且
   `sbo_alloc` 用 trylock, 抢不到锁直接返回 -1。8 线程时 94.5% 的分配
   在空间充足时失败（仓库自带 bench 只数成功数, 掩盖了这一点）。
2. **填充二次退化**: alloc-only 保持存活的场景下, 每次分配从下标 0
   线性扫描子 box 并逐个解引用, 50k 次分配平均 47µs/op 且随填充恶化。
3. **逐位扫描**: free bitmap 是 `uint8_t[8]`, 连续最大空闲、找连续 run、
   置/清位全是逐 bit 循环; `update_parent` 每次容量变化都重扫全部子
   box 并解引用各自指针; `packed` 结构体使每次成员访问都是不对齐访问。
4. **潜在越界**: `sbo_alloc_obj_slots` 在 `start==0` 时
   `obj_boundary = start-1` 下溢为 255, 随后 `sbo_alloc_box_slot` 会
   扫描并写入 slot 64 以外的内存（基线在压测中复现过段错误, 8 线程
   模型检查出现过 `allocated_size=0`）。

## v3 的改动

- `free_mask` 改为单个 `uint64_t`: 单段连续空闲走 `clz`/`ctz` O(1) 快速路径,
  碎片状态走 6 步 SWAR 倍增位移与（无分支循环）计算连续最大与连续 run。
- slot 状态改为一字节（紧邻 32B 热头布局在 `sbo_box_t` 前 96B 内）,
  `OBJ_START` 内嵌对象长度: free 与 `allocated_size` O(1) 取长, 校验低位对齐防止内部偏移误判。
- 父 box 缓存每个子 box 的 combined 容量（uint16 编码 `(level<<6)|mult`,
  `mult` 截断至 `SBO_MULT_MAX=63` 消除 `64<->63` 传播风暴与跨层虚报）,
  下降时先校验 `child_max_enc >= need_enc` 与 `child_cap[i]`；最大值用
  “持有者计数 + `child_mask` 定向重扫”精确维护, 父节点 combined 不变时立即停止传播。
- 每 shard 锁结构（64B cache line 填充区）内嵌 L1/L2/L3 活跃 box 直达索引
  (`hot_id1`/`hot_base`), `sbo_alloc` 与 `sbo_free` 命中热 box 时 O(1) 直达叶节点,
  无需从 root 逐层下探; 配合 `hint_child` roving 提示保证填充场景 O(depth)。
- 按需惰性回收空子 box（`sbo_reclaim_empty`）与深层 `blocks_alloc` 失败回滚:
  常规 churn 保持叶 box 温热零抖动, 当父层满或 meta 块耗尽时自动回收空子树。
- **分片**: 数据区 ≥ 64×2MB 时切为 64 个独立 shard（各自 TTAS 锁 + blockmalloc
  池 + 树根）, 线程用 `initial-exec` TLS 亲和（奇数步长 9 错开相邻线程 cache line）;
  `sbo_alloc` 先 trylock 轮转、再阻塞轮转, 瞬时竞争不盲目迁移 home shard,
  不再因竞争误报 OOM。
- box 经填充保证恒 8 字节对齐（块头 2/4 字节时加 pad, 步长取 8 的倍数）,
  去掉 `packed` 的不对齐访问与冗余 `memset`；ASan/UBSan/TSan 干净。

代价/兼容性:

- meta 格式不兼容 v2（magic 升级为 `slotsboxobj64v3`）, 需重新 `sbo_init`。
- 单个 box 元数据 343B → 480B（多了 child_cap 缓存等）, 相同 meta 区可放
  的 box 数约减少 29%。
- 单次分配上限约为 shard 大小的 63/64（128MB 数据区 → 约 1.97MB）,
  这是 v2“根 box 63 个 slot”同一规则在分片后的形式；可用
  `SBO_MIN_SHARD_BYTES` 调大 shard 下限换取更大单次上限（分片数随之减少）。
- 极小数据区（<512B）现在 `sbo_init` 直接拒绝（v2 接受但根 level 为 0
  实际不可用）；meta 区要求 8 字节对齐。

## 结果（自研严格基准, 成功吞吐, 中位数）

alloc+free 混合尺寸 {8,64,512,4096}B, 每线程 200k 次（`taskset -c 0-15`）:

| 线程 | v2 Mops/s | v2 失败率 | v3 (initial) Mops/s | v3 (optimized) Mops/s | 相比 v2 提升 |
|---:|---:|---:|---:|---:|---:|
| 1 | 3.27 | 0% | 16.93 | **20.33** | **6.2×** |
| 4 | 1.62 | 91.7% | 64.09 | **80.31** | **49.6×** |
| 8 | 0.85 | 95.0% | 119.53 | **162.55** | **191×** |
| 16 | 0.75 | 95.0% | 180.64 | **248.46** | **331×** |

其它场景（1 线程 / 16 线程）:

- 小尺寸 {8..64}B alloc+free: 3.11 → **20.42 Mops/s**（6.6×）/ 0.75 → **227.36 Mops/s**（303×）
- 随机 churn（2048 存活集, 8B–64KB）: 2.73 → **13.06 Mops/s**（4.8×）/ 0.83 → **112.74 Mops/s**（136×）
- alloc-only 填充 50k 不释放: 0.02 → **15.22 Mops/s**（761×, 消除二次退化）
- v3 在所有多线程场景失败率 0（空间充足时）。

仓库自带基准:

- `08_bench_pure`: 单线程 **23.5M ops/s**; 8 线程全部完成 400000 次, **141.7M ops/s**。
- `15_bench_dedicated`: 8 线程 **159.5M ops/s**。

## 正确性验证

- `tutorial/18_model_check.c`（ctest 以 `4 10000` 运行）: 随机 churn
  下用逐 8B 单位的 claim 表验证分配永不重叠、首尾图案验证存活对象不被
  disturb、`sbo_allocated_size` 恒等于模型容量, 且多线程空间充足时零假失败;
  另含溢出尺寸、shard 上限、L1/L2 内部偏移与未对齐/双重 free、过小 `head_size`、
  meta 块耗尽回滚、空子 box 回收合并、错误 magic、小数据区与单 shard 多线程检查。
- `ctest` 10/10 与 `tutorial/test.py` 17/17 全部通过;
  ASan+UBSan 与 TSan（`-fno-sanitize-recover=all`）全部干净。

## 复现

```bash
make test
./build/tutorial/08_bench_pure 8
./build/tutorial/16_bench_malloc
./build/tutorial/18_model_check 16 100000
```
