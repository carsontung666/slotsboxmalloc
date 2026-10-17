# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目定位

slotsboxmalloc 是基于 N=64 伙伴树（left-box/right-obj）的存储分配器。不是通用 malloc 替代，设计目标覆盖 OS kernel 和 block 设备的存储分配场景。全部实现都在头文件 `include/slotsboxmalloc/slotsboxobj.h`（header-only，`src/slotsboxmalloc.c` 只是实例化），依赖 blockmalloc 管理 meta 区的 box 节点。性能优化细节与实验数据见 `doc/perf-v3.md`。

## 构建

```bash
make
```

**注意**：slotsboxmalloc 依赖 blockmalloc 共享库。需要先 `sudo make install` blockmalloc，或设置 `LD_LIBRARY_PATH`。运行时确保 `libblockmalloc.so.1` 在链接路径中。无 cmake 时可手动：

```bash
gcc -O3 -DNDEBUG -fPIC -shared -std=c11 -Iinclude -I<blockmalloc>/include \
    src/slotsboxmalloc.c -L<blockmalloc>/build -lblockmalloc -o libslotsboxmalloc.so
```

## 测试

```bash
python3 tutorial/test.py
python3 tutorial/test.py --filter 04          # 按名称过滤
python3 tutorial/test.py --skip-stress        # 跳过 03_stress（有无界循环）
cd build && ./tutorial/01_basic               # 直接运行单个 case
ASAN_OPTIONS=detect_leaks=1 ./tutorial/01_basic
```

tutorial case 清单（节选）：

| case | 说明 |
|------|------|
| `01_basic` | 基础 init/alloc/free |
| `02_bench` | alloc/write/read/free 计时 |
| `03_stress` | 分配至满（2MB 数据区），然后随机 free/alloc 循环（2s 限时）|
| `04_multithread` | 多线程并发 alloc/free |
| `05_multiprocess` / `06_multiprocess_multithread` | 共享内存多进程并发 |
| `08_bench_pure` / `16_bench_malloc` | 纯吞吐基准（16 含 glibc 对照）|
| `17_data_integrity` | 数据完整性 |
| `18_model_check` | 随机模型校验：不重叠/容量精确/多线程零假失败（ctest）|

## 架构 (v3)

### 64 叉树模型

数据区必须为 `8 × 64^k` 字节。最小分配单元 8 字节：L1=512B（64×8B）, L2=32KB, L3=2MB, L4=128MB…每 box 固定 64 个 slot，子 box 从左侧分配、对象从右侧分配（连续多 slot，首 slot 状态字节内嵌长度）。

- **level / multiple**：`sbo_align_to(n)` 把 8 字节单位数换成 `(level, multiple)`，multiple ∈ [1,63]；容量编码 `enc=(level<<6)|multiple` 可直接整数比较。
- **free_mask**：单个 `uint64_t`，bit=1 表示 slot 空闲。单段连续空闲走 `clz`/`ctz` O(1) 路径，碎片状态走 SWAR 词运算。

### 内存布局

meta 区与 data 区分离。`sbo_alloc` 返回 data 区内的**字节偏移**，调用者自行 `data + offset`。meta 布局：`[sbo_meta_t][N×blocks_meta_t][N×sbo_lock_t][shard0 池][shard1 池]...`，池内每个块 = blockmalloc 块头 + 对齐 pad + `sbo_box_t`（恒 8 字节对齐，步长为 8 的倍数）。

### 分片与并发

- 数据区 ≥ 64×2MB 时切成 64 个 shard（`SBO_MIN_SHARD_BYTES` 可调），每 shard 独立锁、独立 blockmalloc 池、独立树根；否则单 shard。单次分配上限 ≈ shard 大小的 63/64。
- `sbo_alloc`：TLS 亲和 home shard，先 trylock 轮转全部 shard、再阻塞轮转，**不会因锁竞争误报 OOM**。`sbo_free` 由 offset 移位定位 shard 后加锁。
- 容量 hint 与直达缓存：父 box 的 `child_cap[64]` 缓存各子 box 的 combined 容量，`child_max_enc` 用持有者计数 + `child_mask` 定向重扫维护，变化沿父链 O(1) 传播；每 shard 锁内缓存 L1/L2/L3 活跃 box ID 与基址，命中时 O(1) 直达叶节点。
- 锁是 per-shard TTAS spinlock（acquire/release），位于 meta 区内，SHM 多进程可用。box 内无锁，同一 shard 的所有操作由 shard 锁串行化。

### offset ↔ 树位置

从 shard 根起按 6-bit 逐层解码 unit offset（`sbo_find_obj_node`）：每层取 `(unit >> 6*(level-1)) & 63`，`child_mask` 命中则下降、`OBJ_START` 且低位对齐则命中。shard 边界与高位 6-bit 对齐，因此同一解码对分片/未分片都成立。

## 常见易错点

1. **offset 是相对 data 区的字节偏移**，不是指针；meta 格式为 v3（`slotsboxobj64v3`），与 v2 不兼容，旧 meta 必须重新 init。
2. **空子 box 按需惰性回收**：常规 free 保持空子 box 温热复用；当父层空间不足或 meta 块耗尽时由 `sbo_reclaim_empty` 自动回收全空子树。
3. **meta 太小会自动退回单 shard**（每 shard 池 < 32 个 box 时），多线程扩展性随之消失——压测时先确认 `sbo_meta_size` 给足。
4. **`sbo_box_t` 不是 packed**，靠 init 时的块填充保证对齐；改结构体后检查 `block_stride`/`box_off` 仍为 8 的倍数（UBSan alignment 会报）。
5. **shard 根恒为 block 0**，运行时据此 O(1) 定位；不要在 init 里先于根分配其它块。
6. `multiple==64` 不存在：恒进位为 `(level+1, 1)`，单 box 自身 `max_obj_cap` 上限为 63（`SBO_MULT_MAX`）；`enc==0` 表示无容量。
