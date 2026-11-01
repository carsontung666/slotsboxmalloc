/* N=64 buddy allocator, meta format v3. Meta memory must be trusted;
 * dead shard lock holders are detected via PID liveness and repaired on takeover. */

#ifndef SLOTSBOXOBJ_H
#define SLOTSBOXOBJ_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#ifndef _MSC_VER
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#if !defined(_POSIX_C_SOURCE) && !defined(_GNU_SOURCE) && !defined(_DEFAULT_SOURCE)
int kill(pid_t pid, int sig);
#endif
#endif

#define SBO_N         64
#define SBO_N_BITS    6
#define SBO_N_MASK    0x3F
#define SBO_MULT_MAX  63
#define SBO_MAGIC     "slotsboxobj64v3"

#ifndef SBO_MIN_SHARD_BYTES
#define SBO_MIN_SHARD_BYTES (8ULL * 64 * 64 * 64)
#endif
#define SBO_MAX_SHARDS 64

/* Capacity: 8 * 64^(level-1) * multiple bytes. */

typedef struct {
    uint8_t level;
    uint8_t multiple;
} sbo_usage_t;

static inline uint16_t sbo_enc(uint8_t level, uint8_t multiple) {
    return (uint16_t)(((uint16_t)level << 6) | multiple);
}
static inline uint16_t sbo_enc_of(sbo_usage_t u) { return sbo_enc(u.level, u.multiple); }

static inline uint64_t sbo_offset_raw(uint8_t level, uint8_t slot) {
    if (level == 0) return (uint64_t)slot << 3;
    return (uint64_t)slot << ((level - 1) * SBO_N_BITS + 3);
}

static inline sbo_usage_t sbo_align_to(uint64_t n_slots) {
    sbo_usage_t r = {0, 0};
    if (n_slots == 0) return r;
    if (n_slots <= SBO_MULT_MAX) { r.level = 1; r.multiple = (uint8_t)n_slots; return r; }
    uint64_t t = n_slots / SBO_MULT_MAX + (n_slots % SBO_MULT_MAX != 0);
    uint32_t k = (uint32_t)((64 - __builtin_clzll(t - 1) + SBO_N_BITS - 1) / SBO_N_BITS);
    uint32_t sh = k * SBO_N_BITS;
    r.level = (uint8_t)(k + 1);
    r.multiple = (uint8_t)((n_slots >> sh) + ((n_slots & ((1ULL << sh) - 1)) != 0));
    return r;
}

typedef struct {
    atomic_int lock;
    uint32_t   hot_id1[3];
    uint64_t   hot_base[3];
    uint32_t   grow_nid;   /* pending root-split block id during sbo_data_grow, 0 otherwise */
    uint8_t    _pad[20];
} sbo_lock_t;
_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "atomic_int must be lock-free");
_Static_assert(sizeof(sbo_lock_t) == 64, "sbo_lock_t must be 64 bytes");

static inline void sbo_cpu_relax(void) {
#ifdef __x86_64__
    __asm__ __volatile__("pause" ::: "memory");
#elif defined(__aarch64__)
    __asm__ __volatile__("yield" ::: "memory");
#endif
}

#ifdef _MSC_VER
static inline int sbo_self_pid(void) {
    int p = (int)GetCurrentProcessId();
    return p > 0 ? p : 1;
}
static inline bool sbo_pid_dead(int owner) {
    (void)owner;
    return false;
}
#else
static _Thread_local int sbo_tls_pid;
static pthread_once_t sbo_atfork_once = PTHREAD_ONCE_INIT;

static void sbo_reset_tls_pid(void) { sbo_tls_pid = 0; }
static void sbo_install_atfork(void) { (void)pthread_atfork(NULL, NULL, sbo_reset_tls_pid); }

static inline int sbo_self_pid(void) {
    int p = sbo_tls_pid;
    if (__builtin_expect(p <= 0, 0)) {
        (void)pthread_once(&sbo_atfork_once, sbo_install_atfork);
        p = (int)getpid();
        if (p <= 0) p = 1;
        sbo_tls_pid = p;
    }
    return p;
}

static inline bool sbo_pid_dead(int owner) {
    if (owner <= 0) return false;
    if (kill((pid_t)owner, 0) == -1 && errno == ESRCH) return true;
#if defined(__linux__)
    char path[64];
    int n = snprintf(path, sizeof(path), "/proc/%d/stat", owner);
    if (n > 0 && (size_t)n < sizeof(path)) {
#ifdef O_CLOEXEC
        int fd = open(path, O_RDONLY | O_CLOEXEC);
#else
        int fd = open(path, O_RDONLY);
#endif
        if (fd < 0) return errno == ENOENT;
        char buf[512];
        ssize_t r = read(fd, buf, sizeof(buf) - 1);
        close(fd);
        if (r > 0) {
            buf[r] = '\0';
            char *rp = strrchr(buf, ')');
            if (rp) {
                char *s = rp + 1;
                while (*s == ' ') s++;
                if (*s == 'Z' || *s == 'X') return true;
            }
        }
    }
#endif
    return false;
}
#endif

static inline bool sbo_lock_ex(sbo_lock_t *lk) {
    int me = sbo_self_pid();
    for (;;) {
        int e = 0;
        if (atomic_compare_exchange_weak_explicit(&lk->lock, &e, me,
                memory_order_acquire, memory_order_relaxed))
            return false;
        for (int spin = 0; spin < 4096; spin++) {
            sbo_cpu_relax();
            if (atomic_load_explicit(&lk->lock, memory_order_relaxed) == 0)
                break;
        }
        int owner = atomic_load_explicit(&lk->lock, memory_order_acquire);
        if (owner > 0 && owner != me && sbo_pid_dead(owner)) {
            if (atomic_compare_exchange_strong_explicit(&lk->lock, &owner, me,
                    memory_order_acq_rel, memory_order_relaxed))
                return true;
        }
    }
}

static inline void sbo_lock(sbo_lock_t *lk) {
    (void)sbo_lock_ex(lk);
}
static inline bool sbo_trylock(sbo_lock_t *lk) {
    if (atomic_load_explicit(&lk->lock, memory_order_relaxed) != 0) return false;
    int e = 0;
    int me = sbo_self_pid();
    return atomic_compare_exchange_weak_explicit(&lk->lock, &e, me,
               memory_order_acquire, memory_order_relaxed);
}
static inline void sbo_unlock(sbo_lock_t *lk) {
    atomic_store_explicit(&lk->lock, 0, memory_order_release);
}

/* Slot state: 0=free, 1=box, 0x40|(len-1)=object start. */
#define SBO_FREE 0
#define SBO_BOX  1
#define SBO_START(len)   ((uint8_t)(0x40 | ((len) - 1)))
#define SBO_IS_START(s)  ((unsigned)((s) - 0x40) < SBO_MULT_MAX)
#define SBO_START_LEN(s) ((int)((s) & 0x3F) + 1)

typedef struct {
    uint64_t free_mask;
    uint64_t child_mask;
    int32_t  parent;
    int32_t  self_id;
    uint16_t child_max_enc;
    uint8_t  objlevel;
    uint8_t  max_obj_cap;
    uint8_t  hint_child;
    uint8_t  parent_slot;
    uint8_t  child_max_count;
    uint8_t  _pad;
    uint8_t  states[SBO_N];
    uint16_t child_cap[SBO_N];
    int32_t  children[SBO_N];
} sbo_box_t;

typedef struct {
    uint8_t  magic[16];
    uint64_t data_size;
    uint64_t per_slot_meta;
    uint32_t next_shard;
    uint16_t block_stride;
    uint8_t  root_slots;
    uint8_t  shard_mask;
    uint8_t  slot_shift;
    uint8_t  box_off;
    uint32_t max_blocks;
    uint8_t  layout;     /* SBO_LAYOUT_LOCK64 when root_slots > 1, else 0 */
    uint8_t  _pad[1];
} sbo_meta_t;

/* Multi-shard metas place the lock array on a 64-byte boundary relative to
 * meta (pass a 64-byte aligned meta, e.g. from mmap) so each shard's lock and
 * hot cache own one cache line. Multi-shard metas from v0.1.7 (layout 0) are
 * rejected rather than misread; the single-shard layout is unchanged. */
#define SBO_LAYOUT_LOCK64 1

int      sbo_init(void *metaptr, size_t head_size, size_t data_size);
uint64_t sbo_alloc(void *metaptr, size_t size);
void     sbo_free(void *metaptr, uint64_t obj_offset);
uint64_t sbo_allocated_size(void *metaptr, uint64_t obj_offset);
void    *sbo_data_ptr(void *metaptr, void *data_base, uint64_t obj_offset);
size_t   sbo_meta_size(size_t data_size, size_t per_slot_pool);

#endif /* SLOTSBOXOBJ_H */

#ifdef SLOTSBOXMALLOC_IMPLEMENTATION

#include <blockmalloc/blockmalloc.h>

#define SBO_POOL_HDR (sizeof(blocks_meta_t) + sizeof(sbo_lock_t))

static inline blocks_meta_t *sbo_pool_meta(sbo_meta_t *meta, uint8_t si) {
    return (blocks_meta_t *)((uint8_t *)(meta + 1)) + si;
}
static inline size_t sbo_lock_off(uint8_t shards) {
    size_t off = sizeof(sbo_meta_t) + (size_t)shards * sizeof(blocks_meta_t);
    return shards > 1 ? (off + 63) & ~(size_t)63 : off;
}
static inline size_t sbo_header_size(uint8_t shards) {
    return sbo_lock_off(shards) + (size_t)shards * sizeof(sbo_lock_t);
}
static inline sbo_lock_t *sbo_pool_lock(sbo_meta_t *meta, uint8_t si) {
    return (sbo_lock_t *)((uint8_t *)meta + sbo_lock_off(meta->root_slots)) + si;
}
static inline uint8_t *sbo_pool_mem(sbo_meta_t *meta, uint8_t si) {
    return (uint8_t *)meta + sbo_header_size(meta->root_slots)
           + si * meta->per_slot_meta;
}

typedef struct {
    blocks_meta_t *pool_meta;
    sbo_lock_t *lk;
    uint8_t *pool;
    uint32_t stride;
    uint32_t head;
    uint32_t max_blocks;
    int32_t last_id;
} sbo_ctx_t;

static inline sbo_ctx_t sbo_ctx_for(sbo_meta_t *meta, uint8_t si, sbo_lock_t *lk) {
    sbo_ctx_t c;
    c.pool_meta = sbo_pool_meta(meta, si);
    c.lk = lk;
    c.pool = sbo_pool_mem(meta, si);
    c.stride = meta->block_stride;
    c.head = meta->box_off;
    c.max_blocks = meta->max_blocks;
    c.last_id = -1;
    return c;
}
static inline sbo_box_t *sbo_box_at(const sbo_ctx_t *c, int32_t id) {
    return (sbo_box_t *)(c->pool + (uint32_t)id * c->stride + c->head);
}

static inline uint8_t sbo_mask_contmax(uint64_t m) {
    if (!m) return 0;
    uint64_t u = m + (m & (~m + 1ULL));
    if (__builtin_expect((u & (u - 1ULL)) == 0, 1)) {
        int n = 64 - __builtin_clzll(m) - __builtin_ctzll(m);
        return (uint8_t)(n > SBO_MULT_MAX ? SBO_MULT_MAX : n);
    }
    uint64_t m2  = m   & (m   >> 1);
    uint64_t m4  = m2  & (m2  >> 2);
    uint64_t m8  = m4  & (m4  >> 4);
    uint64_t m16 = m8  & (m8  >> 8);
    uint64_t m32 = m16 & (m16 >> 16);
    uint8_t best = 1;
    uint64_t x = m, y;
    if (m32) { x = m32; best += 32; }
    y = (x >> 16) & m16; if (y) { x = y; best += 16; }
    y = (x >> 8)  & m8;  if (y) { x = y; best += 8;  }
    y = (x >> 4)  & m4;  if (y) { x = y; best += 4;  }
    y = (x >> 2)  & m2;  if (y) { x = y; best += 2;  }
    y = (x >> 1)  & m;   if (y) { best += 1; }
    return best > SBO_MULT_MAX ? SBO_MULT_MAX : best;
}

static inline int sbo_mask_find_run(uint64_t m, int need) {
    uint64_t x = m;
    int len = 1;
    while (len * 2 <= need) { x &= (x >> len); len *= 2; }
    if (len < need) x &= (x >> (need - len));
    return x ? (63 - __builtin_clzll(x)) : -1;
}

static inline uint16_t sbo_combined(const sbo_box_t *box) {
    uint16_t own = box->max_obj_cap ? sbo_enc(box->objlevel, box->max_obj_cap) : 0;
    return own > box->child_max_enc ? own : box->child_max_enc;
}

static inline void sbo_rescan_child_max(sbo_box_t *p) {
    uint16_t mx = 0;
    uint8_t cnt = 0;
    uint64_t mm = p->child_mask;
    while (mm) {
        int i = __builtin_ctzll(mm);
        mm &= mm - 1;
        uint16_t v = p->child_cap[i];
        if (v > mx) { mx = v; cnt = 1; }
        else if (v == mx) cnt++;
    }
    p->child_max_enc = mx;
    p->child_max_count = cnt;
}

static inline void sbo_propagate(const sbo_ctx_t *c, sbo_box_t *box) {
    uint16_t comb = sbo_combined(box);
    while (box->parent >= 0) {
        sbo_box_t *p = sbo_box_at(c, box->parent);
        uint16_t old = p->child_cap[box->parent_slot];
        if (old == comb) return;
        uint16_t p_old = sbo_combined(p);
        p->child_cap[box->parent_slot] = comb;
        if (comb > p->child_max_enc) {
            p->child_max_enc = comb;
            p->child_max_count = 1;
        } else if (comb == p->child_max_enc) {
            p->child_max_count++;
        } else if (old == p->child_max_enc && --p->child_max_count == 0) {
            sbo_rescan_child_max(p);
        }
        comb = sbo_combined(p);
        if (comb == p_old) return;
        box = p;
    }
}

static inline void sbo_after_mask_change(const sbo_ctx_t *c, sbo_box_t *box) {
    uint8_t old = box->max_obj_cap;
    uint8_t nc = sbo_mask_contmax(box->free_mask);
    if (nc != old) {
        box->max_obj_cap = nc;
        sbo_propagate(c, box);
    }
}

static inline void sbo_box_format(sbo_box_t *box, uint8_t objlevel,
                                   int32_t parent, uint8_t parent_slot,
                                   int32_t self_id) {
    box->free_mask = ~0ULL;
    box->child_mask = 0;
    box->parent = parent;
    box->self_id = self_id;
    box->child_max_enc = 0;
    box->objlevel = objlevel;
    box->max_obj_cap = SBO_MULT_MAX;
    box->hint_child = 0;
    box->parent_slot = parent_slot;
    box->child_max_count = 0;
    memset(box->states, 0, sizeof(box->states));
}

static bool sbo_reclaim_empty(const sbo_ctx_t *c, sbo_box_t *box) {
    bool any = false;
    uint64_t mm = box->child_mask;
    while (mm) {
        int i = __builtin_ctzll(mm);
        mm &= mm - 1;
        sbo_box_t *ch = sbo_box_at(c, box->children[i]);
        if (ch->child_mask) sbo_reclaim_empty(c, ch);
        if (ch->free_mask == ~0ULL && ch->child_mask == 0) {
            int32_t cid = ch->self_id;
            uint8_t lv = ch->objlevel;
            if (lv >= 1 && lv <= 3 && c->lk->hot_id1[lv - 1] == (uint32_t)cid + 1)
                c->lk->hot_id1[lv - 1] = 0;
            box->child_mask &= ~(1ULL << i);
            box->free_mask |= (1ULL << i);
            box->states[i] = SBO_FREE;
            box->child_cap[i] = 0;
            atomic_thread_fence(memory_order_release);
            blocks_free(c->pool_meta, c->pool, (uint64_t)cid);
            any = true;
        }
    }
    if (any) {
        sbo_rescan_child_max(box);
        box->hint_child = box->child_mask ? (uint8_t)__builtin_ctzll(box->child_mask) : 0;
        box->max_obj_cap = sbo_mask_contmax(box->free_mask);
        sbo_propagate(c, box);
    }
    return any;
}

static inline int sbo_alloc_obj_slots(sbo_ctx_t *c, sbo_box_t *box, uint8_t m) {
    uint64_t fm = box->free_mask;
    if (!fm) return -1;
    uint64_t u = fm + (fm & (~fm + 1ULL));
    if (__builtin_expect((u & (u - 1ULL)) == 0, 1)) {
        int hi = 64 - __builtin_clzll(fm);
        int k = hi - __builtin_ctzll(fm);
        if (k < (int)m) return -1;
        int s = hi - (int)m;
        box->free_mask = fm & ~(((1ULL << m) - 1ULL) << s);
        box->states[s] = SBO_START(m);
        c->last_id = box->self_id;
        int rem = k - (int)m;
        uint8_t nc = (uint8_t)(rem > SBO_MULT_MAX ? SBO_MULT_MAX : rem);
        if (nc != box->max_obj_cap) {
            box->max_obj_cap = nc;
            sbo_propagate(c, box);
        }
        return s;
    }
    int s = sbo_mask_find_run(fm, m);
    if (s < 0) return -1;
    box->free_mask = fm & ~(((1ULL << m) - 1ULL) << s);
    box->states[s] = SBO_START(m);
    c->last_id = box->self_id;
    sbo_after_mask_change(c, box);
    return s;
}

static inline void sbo_free_obj_slots(const sbo_ctx_t *c, sbo_box_t *box, uint8_t start) {
    int len = SBO_START_LEN(box->states[start]);
    if (start + len > SBO_N) return;
    box->states[start] = SBO_FREE;
    box->free_mask |= (((1ULL << len) - 1ULL) << start);
    sbo_after_mask_change(c, box);
}

#define SBO_ALLOC_FAIL ((uint64_t)-1)

static uint64_t sbo_find_alloc(sbo_ctx_t *c, sbo_box_t *box,
                               sbo_usage_t usage, uint16_t need_enc) {
    if (usage.level == box->objlevel) {
        int slot = sbo_alloc_obj_slots(c, box, usage.multiple);
        if (slot < 0 && box->child_mask && sbo_reclaim_empty(c, box))
            slot = sbo_alloc_obj_slots(c, box, usage.multiple);
        if (slot < 0) return SBO_ALLOC_FAIL;
        return sbo_offset_raw(box->objlevel, (uint8_t)slot);
    }
    if (usage.level > box->objlevel) return SBO_ALLOC_FAIL;

    if (box->child_max_enc >= need_enc) {
        int start = box->hint_child;
        for (int part = 0; part < 2; part++) {
            uint64_t mm = (part == 0)
                ? (box->child_mask & (~0ULL << start))
                : (box->child_mask & ((start == 0) ? 0ULL : ((1ULL << start) - 1)));
            while (mm) {
                int i = __builtin_ctzll(mm);
                mm &= mm - 1;
                if (box->child_cap[i] < need_enc) continue;
                uint64_t sub = sbo_find_alloc(c, sbo_box_at(c, box->children[i]),
                                              usage, need_enc);
                if (sub != SBO_ALLOC_FAIL) {
                    if (box->hint_child != (uint8_t)i) box->hint_child = (uint8_t)i;
                    return sbo_offset_raw(box->objlevel, (uint8_t)i) + sub;
                }
            }
        }
    }

    uint32_t need_blk = (uint32_t)(box->objlevel - usage.level);
    if (!box->free_mask || c->max_blocks - (uint32_t)c->pool_meta->used_blocks < need_blk) {
        sbo_reclaim_empty(c, sbo_box_at(c, 0));
        if (!box->free_mask || c->max_blocks - (uint32_t)c->pool_meta->used_blocks < need_blk)
            return SBO_ALLOC_FAIL;
    }

    int64_t cid = blocks_alloc(c->pool_meta, c->pool);
    if (cid < 0) return SBO_ALLOC_FAIL;
    int slot = __builtin_ctzll(box->free_mask);
    sbo_box_t *ch = sbo_box_at(c, (int32_t)cid);
    sbo_box_format(ch, (uint8_t)(box->objlevel - 1), box->self_id,
                   (uint8_t)slot, (int32_t)cid);
    uint16_t ch_comb = sbo_combined(ch);
    box->children[slot] = (int32_t)cid;
    box->child_cap[slot] = ch_comb;
    atomic_thread_fence(memory_order_release);
    box->states[slot] = SBO_BOX;
    box->free_mask &= ~(1ULL << slot);
    box->child_mask |= (1ULL << slot);
    if (ch_comb > box->child_max_enc) {
        box->child_max_enc = ch_comb;
        box->child_max_count = 1;
    } else if (ch_comb == box->child_max_enc) {
        box->child_max_count++;
    }
    box->max_obj_cap = sbo_mask_contmax(box->free_mask);

    uint64_t sub = sbo_find_alloc(c, ch, usage, need_enc);
    if (sub == SBO_ALLOC_FAIL) {
        box->child_mask &= ~(1ULL << slot);
        box->free_mask |= (1ULL << slot);
        box->states[slot] = SBO_FREE;
        box->child_cap[slot] = 0;
        atomic_thread_fence(memory_order_release);
        blocks_free(c->pool_meta, c->pool, (uint64_t)cid);
        sbo_rescan_child_max(box);
        box->max_obj_cap = sbo_mask_contmax(box->free_mask);
        sbo_propagate(c, box);
        return SBO_ALLOC_FAIL;
    }
    if (box->hint_child != (uint8_t)slot) box->hint_child = (uint8_t)slot;
    return sbo_offset_raw(box->objlevel, (uint8_t)slot) + sub;
}

static inline sbo_box_t *sbo_find_obj_node(const sbo_ctx_t *c, sbo_box_t *root,
                                           uint64_t obj_offset, uint8_t *out_slot) {
    if (root->objlevel == 0 || root->objlevel > 10) return NULL;
    uint64_t unit = obj_offset >> 3;
    sbo_box_t *node = root;
    int level = node->objlevel;
    while (level >= 1) {
        uint32_t sh = (uint32_t)(level - 1) * SBO_N_BITS;
        uint8_t slot = (uint8_t)((unit >> sh) & SBO_N_MASK);
        if ((node->child_mask >> slot) & 1ULL) {
            node = sbo_box_at(c, node->children[slot]);
            level--;
            continue;
        }
        uint8_t st = node->states[slot];
        if (SBO_IS_START(st) && (unit & ((1ULL << sh) - 1)) == 0
            && slot + SBO_START_LEN(st) <= SBO_N) {
            *out_slot = slot;
            return node;
        }
        return NULL;
    }
    return NULL;
}

static inline uint8_t sbo_shards_for_data(size_t data_size) {
    return (data_size / SBO_MAX_SHARDS >= SBO_MIN_SHARD_BYTES)
               ? (uint8_t)SBO_MAX_SHARDS : (uint8_t)1;
}

static inline bool sbo_magic_ok(const sbo_meta_t *meta) {
    uint64_t got0, got1, want0, want1;
    memcpy(&got0, meta->magic, 8);
    memcpy(&got1, meta->magic + 8, 8);
    memcpy(&want0, SBO_MAGIC, 8);
    memcpy(&want1, SBO_MAGIC + 8, 8);
    return ((got0 ^ want0) | (got1 ^ want1)) == 0;
}

static inline bool sbo_meta_base_ok(const sbo_meta_t *meta) {
    if (!sbo_magic_ok(meta)) return false;
    if (meta->root_slots != 1 && meta->root_slots != SBO_MAX_SHARDS) return false;
    if (meta->root_slots > 1 && meta->layout != SBO_LAYOUT_LOCK64) return false;
    if (meta->shard_mask != meta->root_slots - 1) return false;
    if ((meta->block_stride & 7) != 0 || (meta->box_off & 7) != 0) return false;
    if (meta->box_off > meta->block_stride
        || (size_t)(meta->block_stride - meta->box_off) < sizeof(sbo_box_t)) return false;
    if (meta->per_slot_meta < meta->block_stride) return false;
    return true;
}

static inline bool sbo_meta_ok(const sbo_meta_t *meta) {
    if (!sbo_meta_base_ok(meta)) return false;
    uint8_t sh = __atomic_load_n(&meta->slot_shift, __ATOMIC_ACQUIRE);
    uint64_t ds = __atomic_load_n(&meta->data_size, __ATOMIC_ACQUIRE);
    if (sh < 9 || sh >= 64) return false;
    if (ds == 0 || (ds >> sh) != meta->root_slots) return false;
    return true;
}

#ifndef SBO_NO_TLS
typedef struct { const void *meta; int16_t home; uint16_t miss; } sbo_tls_t;
static __thread sbo_tls_t sbo_tls __attribute__((tls_model("initial-exec"))) = {NULL, -1, 0};
#endif

static inline int sbo_next_home(sbo_meta_t *meta) {
    return (int)((__atomic_fetch_add(&meta->next_shard, 1u, __ATOMIC_RELAXED) * 9u)
                 & meta->shard_mask);
}

static inline int sbo_home_shard(sbo_meta_t *meta) {
    if (meta->root_slots == 1) return 0;
#ifdef SBO_NO_TLS
    return sbo_next_home(meta);
#else
    if (sbo_tls.meta != (const void *)meta || sbo_tls.home < 0
        || sbo_tls.home > meta->shard_mask) {
        sbo_tls.meta = (const void *)meta;
        sbo_tls.home = (int16_t)sbo_next_home(meta);
        sbo_tls.miss = 0;
    }
    return sbo_tls.home;
#endif
}

#define SBO_MIN_POOL_BLOCKS 32

int sbo_init(void *metaptr, size_t head_size, size_t data_size) {
    if (!metaptr || ((uintptr_t)metaptr & 7) != 0) return -1;
    if (head_size <= sizeof(sbo_meta_t) + SBO_POOL_HDR || (data_size & 7) != 0) return -1;
    sbo_meta_t *meta = (sbo_meta_t *)metaptr;
    if (sbo_magic_ok(meta)) return -1;

    uint64_t slots = data_size >> 3;
    if (slots < SBO_N || (slots & (slots - 1)) != 0) return -1;
    if (__builtin_ctzll(slots) % SBO_N_BITS != 0) return -1;

    uint8_t shards = sbo_shards_for_data(data_size);
    size_t header_base, per_slot_meta;
    for (;;) {
        header_base = sbo_header_size(shards);
        if (head_size <= header_base) return -1;
        per_slot_meta = ((head_size - header_base) / shards) & ~(size_t)7;
        if (shards == 1
            || per_slot_meta >= (size_t)SBO_MIN_POOL_BLOCKS * (sizeof(sbo_box_t) + 16))
            break;
        shards = 1;
    }
    if (per_slot_meta < sizeof(sbo_box_t) + 16) return -1;

    memset(meta, 0, sizeof(*meta));
    meta->data_size = data_size;
    meta->root_slots = shards;
    meta->shard_mask = (uint8_t)(shards - 1);
    meta->slot_shift = (uint8_t)__builtin_ctzll(data_size / shards);
    meta->per_slot_meta = per_slot_meta;
    meta->layout = shards > 1 ? SBO_LAYOUT_LOCK64 : 0;
    memset((uint8_t *)(meta + 1), 0, header_base - sizeof(*meta));

    uint8_t root_level =
        (uint8_t)(sbo_align_to(slots / shards).level - 1);
    for (uint8_t si = 0; si < shards; si++) {
        blocks_meta_t *pm = sbo_pool_meta(meta, si);
        uint8_t *pool = sbo_pool_mem(meta, si);
        uint64_t bs = sizeof(sbo_box_t);
        uint8_t head = 0, pad = 0;
        for (int it = 0; it < 4; it++) {
            blocks_init(pm, per_slot_meta, bs);
            head = (uint8_t)pm->sizeof_block_head;
            pad = (uint8_t)((8 - (head & 7)) & 7);
            uint64_t nbs = pad + ((sizeof(sbo_box_t) + 7) & ~(uint64_t)7);
            if (nbs == bs) break;
            bs = nbs;
        }
        if (si == 0) {
            meta->box_off = (uint8_t)(head + pad);
            meta->block_stride = (uint16_t)(head + bs);
            if ((meta->block_stride & 7) != 0 || (meta->box_off & 7) != 0)
                return -1;
            meta->max_blocks = (uint32_t)(per_slot_meta / meta->block_stride);
        }
        if (blocks_alloc(pm, pool) != 0) return -1;
        sbo_box_format((sbo_box_t *)(pool + meta->box_off), root_level, -1, 255, 0);
    }

#ifndef SBO_NO_TLS
    if (sbo_tls.meta == (const void *)meta) { sbo_tls.meta = NULL; sbo_tls.home = -1; sbo_tls.miss = 0; }
#endif
    memcpy(meta->magic, SBO_MAGIC, sizeof(SBO_MAGIC) - 1);
    return 0;
}

static bool sbo_try_hint(sbo_ctx_t *c, sbo_box_t *root, sbo_usage_t usage,
                         uint16_t need_enc, uint64_t *off_out) {
    if (usage.level >= 1 && usage.level <= 3) {
        uint32_t id1 = c->lk->hot_id1[usage.level - 1];
        if (id1 != 0) {
            sbo_box_t *hb = sbo_box_at(c, (int32_t)(id1 - 1));
            if (hb->objlevel == usage.level && hb->max_obj_cap >= usage.multiple) {
                int slot = sbo_alloc_obj_slots(c, hb, usage.multiple);
                if (slot >= 0) {
                    *off_out = c->lk->hot_base[usage.level - 1]
                               + sbo_offset_raw(usage.level, (uint8_t)slot);
                    return true;
                }
            }
        }
    }
    sbo_box_t *box = root;
    uint64_t base = 0;
    for (int guard = 0; guard < 16 && usage.level < box->objlevel; guard++) {
        uint8_t h = box->hint_child;
        if (h >= SBO_N) return false;
        if ((box->child_mask & (1ULL << h)) == 0) return false;
        if (box->child_cap[h] < need_enc || box->children[h] < 0) return false;
        uint8_t parent_level = box->objlevel;
        sbo_box_t *ch = sbo_box_at(c, box->children[h]);
        if ((uint8_t)(ch->objlevel + 1) != parent_level) return false;
        base += sbo_offset_raw(parent_level, h);
        box = ch;
    }
    if (usage.level != box->objlevel) return false;
    int slot = sbo_alloc_obj_slots(c, box, usage.multiple);
    if (slot < 0) return false;
    if (usage.level >= 1 && usage.level <= 3) {
        c->lk->hot_id1[usage.level - 1] = (uint32_t)box->self_id + 1;
        c->lk->hot_base[usage.level - 1] = base;
    }
    *off_out = base + sbo_offset_raw(box->objlevel, (uint8_t)slot);
    return true;
}

static void sbo_repair_box(const sbo_ctx_t *c, sbo_box_t *box, uint8_t level,
                           int32_t parent, uint8_t parent_slot, int32_t self_id,
                           uint64_t *vis, uint64_t mb) {
    if (vis && (uint64_t)self_id < mb)
        vis[(uint64_t)self_id >> 6] |= 1ULL << (self_id & 63);
    box->self_id = self_id;
    box->parent = parent;
    box->parent_slot = parent_slot;
    box->objlevel = level;
    uint64_t free_mask = ~0ULL;
    uint64_t child_mask = 0;
    memset(box->child_cap, 0, sizeof(box->child_cap));
    for (int slot = 0; slot < SBO_N; slot++) {
        uint8_t st = box->states[slot];
        if (SBO_IS_START(st)) {
            int len = SBO_START_LEN(st);
            if (slot + len <= SBO_N) {
                uint64_t run = ((1ULL << len) - 1ULL) << slot;
                free_mask &= ~run;
                for (int k = 1; k < len; k++)
                    box->states[slot + k] = SBO_FREE;
                slot += len - 1;
                continue;
            }
            box->states[slot] = SBO_FREE;
        } else if (st == SBO_BOX) {
            int32_t cid = box->children[slot];
            bool dup = vis && cid > 0 && (uint64_t)cid < mb &&
                       ((vis[(uint64_t)cid >> 6] >> (cid & 63)) & 1ULL) != 0;
            if (level > 1 && cid > 0 && (uint64_t)cid < mb && !dup) {
                free_mask &= ~(1ULL << slot);
                child_mask |= (1ULL << slot);
                sbo_box_t *ch = sbo_box_at(c, cid);
                sbo_repair_box(c, ch, (uint8_t)(level - 1), self_id,
                               (uint8_t)slot, cid, vis, mb);
                box->child_cap[slot] = sbo_combined(ch);
                continue;
            }
            box->states[slot] = SBO_FREE;
        } else {
            box->states[slot] = SBO_FREE;
        }
    }
    box->free_mask = free_mask;
    box->child_mask = child_mask;
    box->max_obj_cap = sbo_mask_contmax(free_mask);
    sbo_rescan_child_max(box);
    box->hint_child = child_mask ? (uint8_t)__builtin_ctzll(child_mask) : 0;
}

static inline void sbo_repair_shard(sbo_meta_t *meta, uint8_t si, sbo_lock_t *lk) {
    memset(lk->hot_id1, 0, sizeof(lk->hot_id1));
    memset(lk->hot_base, 0, sizeof(lk->hot_base));
    blocks_meta_t *pm = sbo_pool_meta(meta, si);
    if (meta->root_slots == 1) {
        if (pm->total_size > meta->per_slot_meta)
            meta->per_slot_meta = pm->total_size;
        meta->max_blocks = (uint32_t)(meta->per_slot_meta / meta->block_stride);
    }
    sbo_ctx_t c = sbo_ctx_for(meta, si, lk);
    spin_lock(&pm->lock);
    blocks_repair_pool(pm, c.pool);
    spin_unlock(&pm->lock);
    sbo_box_t *root = sbo_box_at(&c, 0);
    uint64_t mb = pm->malloc_blocks;
    if (meta->root_slots == 1) {
        uint8_t want_lvl =
            (uint8_t)(sbo_align_to((1ULL << meta->slot_shift) >> 3).level - 1);
        uint32_t gnid = __atomic_load_n(&lk->grow_nid, __ATOMIC_ACQUIRE);
        if (gnid > 0 && (uint64_t)gnid < mb && meta->slot_shift + SBO_N_BITS < 64) {
            sbo_box_t *gb = sbo_box_at(&c, (int32_t)gnid);
            if (gb->objlevel == want_lvl) {
                /* Complete an interrupted sbo_data_grow root split from journaled grow_nid. */
                uint8_t lvl = (uint8_t)(want_lvl + 1);
                sbo_box_format(root, lvl, -1, 255, 0);
                root->children[0] = (int32_t)gnid;
                root->states[0] = SBO_BOX;
                root->free_mask = ~1ULL;
                root->child_mask = 1ULL;
                meta->slot_shift = (uint8_t)(meta->slot_shift + SBO_N_BITS);
            }
        } else if (root->objlevel == (uint8_t)(want_lvl + 1)
            && meta->slot_shift + SBO_N_BITS < 64) {
            meta->slot_shift = (uint8_t)(meta->slot_shift + SBO_N_BITS);
        }
        if (meta->slot_shift >= 9 && meta->slot_shift < 64)
            __atomic_store_n(&meta->data_size, 1ULL << meta->slot_shift, __ATOMIC_RELEASE);
        __atomic_store_n(&lk->grow_nid, 0u, __ATOMIC_RELEASE);
    }
    uint8_t root_lvl =
        (uint8_t)(sbo_align_to((1ULL << meta->slot_shift) >> 3).level - 1);
    uint64_t stack_vis[64];
    uint64_t *vis = NULL;
    uint64_t nwords = (mb + 63ULL) >> 6;
    if (nwords > 0) {
        if (nwords <= 64) {
            memset(stack_vis, 0, (size_t)nwords * sizeof(uint64_t));
            vis = stack_vis;
        } else {
            vis = (uint64_t *)calloc((size_t)nwords, sizeof(uint64_t));
        }
    }
    sbo_repair_box(&c, root, root_lvl, -1, 255, 0, vis, mb);
    if (vis) {
        for (uint64_t id = 1; id < mb; id++) {
            if (((vis[id >> 6] >> (id & 63)) & 1ULL) == 0) {
                block64_t blk = getblock_t(pm, c.pool + block_offset(pm, id));
                if (blk.used == 1)
                    blocks_free(pm, c.pool, id);
            }
        }
        if (vis != stack_vis)
            free(vis);
    }
    sbo_reclaim_empty(&c, root);
}

static inline void sbo_lock_shard(sbo_meta_t *meta, uint8_t si) {
    sbo_lock_t *lk = sbo_pool_lock(meta, si);
    if (sbo_lock_ex(lk))
        sbo_repair_shard(meta, si, lk);
}

uint64_t sbo_alloc(void *metaptr, size_t size) {
    if (!metaptr || size == 0) return SBO_ALLOC_FAIL;
    sbo_meta_t *meta = (sbo_meta_t *)metaptr;
    if (!sbo_meta_ok(meta)) {
        if (!sbo_meta_base_ok(meta)) return SBO_ALLOC_FAIL;
        if (meta->root_slots == 1) {
            sbo_lock_shard(meta, 0);
            sbo_unlock(sbo_pool_lock(meta, 0));
        }
        if (!sbo_meta_ok(meta)) return SBO_ALLOC_FAIL;
    }
    uint8_t cur_shift = __atomic_load_n(&meta->slot_shift, __ATOMIC_ACQUIRE);
    uint64_t shard_bytes = 1ULL << cur_shift;
    if (size > shard_bytes - (shard_bytes >> SBO_N_BITS)) return SBO_ALLOC_FAIL;

    sbo_usage_t usage = sbo_align_to((size >> 3) + ((size & 7) != 0));
    uint16_t need_enc = sbo_enc_of(usage);
    int shards = meta->root_slots;
    int home = sbo_home_shard(meta);
    bool home_full = false;

    for (int pass = 0; pass < 2; pass++) {
        for (int t = 0; t < shards; t++) {
            int si = (home + t) & meta->shard_mask;
            sbo_lock_t *lk = sbo_pool_lock(meta, (uint8_t)si);
            if (pass == 0) { if (!sbo_trylock(lk)) continue; }
            else sbo_lock_shard(meta, (uint8_t)si);

            sbo_ctx_t c = sbo_ctx_for(meta, (uint8_t)si, lk);
            sbo_box_t *root = sbo_box_at(&c, 0);
            uint64_t off = SBO_ALLOC_FAIL;
            if (sbo_combined(root) < need_enc && root->child_mask)
                sbo_reclaim_empty(&c, root);
            if (sbo_combined(root) >= need_enc) {
                if (!sbo_try_hint(&c, root, usage, need_enc, &off)) {
                    off = sbo_find_alloc(&c, root, usage, need_enc);
                    if (off != SBO_ALLOC_FAIL && usage.level >= 1 && usage.level <= 3
                        && c.last_id >= 0) {
                        uint32_t bsh = (uint32_t)usage.level * SBO_N_BITS + 3;
                        lk->hot_id1[usage.level - 1] = (uint32_t)c.last_id + 1;
                        lk->hot_base[usage.level - 1] =
                            bsh < 64 ? (off & ~((1ULL << bsh) - 1)) : 0;
                    }
                }
            }
            sbo_unlock(lk);

            if (off != SBO_ALLOC_FAIL) {
#ifndef SBO_NO_TLS
                if (t == 0) {
                    sbo_tls.miss = 0;
                } else if (sbo_tls.meta == (const void *)meta
                           && (home_full || ++sbo_tls.miss >= 4)) {
                    sbo_tls.home = (int16_t)si;
                    sbo_tls.miss = 0;
                }
#endif
                return ((uint64_t)si << meta->slot_shift) + off;
            }
            if (t == 0) home_full = true;
        }
    }
    return SBO_ALLOC_FAIL;
}

static inline bool sbo_offset_ok(const sbo_meta_t *meta, uint64_t obj_offset) {
    return sbo_meta_ok(meta) && (obj_offset & 7) == 0
           && obj_offset < meta->data_size
           && (obj_offset >> meta->slot_shift) < meta->root_slots;
}

static inline sbo_box_t *sbo_lookup_locked(sbo_meta_t *meta, uint64_t obj_offset,
                                           sbo_ctx_t *c, sbo_lock_t **lk_out,
                                           uint8_t *slot) {
    uint8_t si = 0;
    if (meta->root_slots > 1) {
        if (meta->slot_shift < 9 || meta->slot_shift >= 64) {
            *lk_out = NULL;
            return NULL;
        }
        if ((obj_offset >> meta->slot_shift) >= meta->root_slots) {
            *lk_out = NULL;
            return NULL;
        }
        si = (uint8_t)(obj_offset >> meta->slot_shift);
    }
    sbo_lock_t *lk = sbo_pool_lock(meta, si);
    *lk_out = lk;
    sbo_lock_shard(meta, si);
    if (!sbo_offset_ok(meta, obj_offset))
        return NULL;
    *c = sbo_ctx_for(meta, si, lk);
    uint64_t local_off = obj_offset & ((1ULL << meta->slot_shift) - 1);
    for (uint8_t lv = 1; lv <= 2; lv++) {
        uint32_t id1 = lk->hot_id1[lv - 1];
        if (!id1) continue;
        uint32_t sh = (uint32_t)(lv - 1) * SBO_N_BITS + 3;
        uint64_t rel = local_off - lk->hot_base[lv - 1];
        if ((rel & ~((uint64_t)SBO_N_MASK << sh)) == 0) {
            uint8_t s = (uint8_t)(rel >> sh);
            sbo_box_t *hb = sbo_box_at(c, (int32_t)(id1 - 1));
            uint8_t st = hb->states[s];
            if (hb->objlevel == lv && SBO_IS_START(st)
                && s + SBO_START_LEN(st) <= SBO_N) {
                *slot = s;
                return hb;
            }
        }
    }
    return sbo_find_obj_node(c, sbo_box_at(c, 0), local_off, slot);
}

void sbo_free(void *metaptr, uint64_t obj_offset) {
    if (!metaptr || (obj_offset & 7) != 0) return;
    sbo_meta_t *meta = (sbo_meta_t *)metaptr;
    if (!sbo_meta_base_ok(meta)) return;
    sbo_ctx_t c;
    sbo_lock_t *lk = NULL;
    uint8_t slot = 0;
    sbo_box_t *node = sbo_lookup_locked(meta, obj_offset, &c, &lk, &slot);
    if (node) sbo_free_obj_slots(&c, node, slot);
    if (lk) sbo_unlock(lk);
}

uint64_t sbo_allocated_size(void *metaptr, uint64_t obj_offset) {
    if (!metaptr || (obj_offset & 7) != 0) return 0;
    sbo_meta_t *meta = (sbo_meta_t *)metaptr;
    if (!sbo_meta_base_ok(meta)) return 0;
    sbo_ctx_t c;
    sbo_lock_t *lk = NULL;
    uint8_t slot = 0;
    sbo_box_t *node = sbo_lookup_locked(meta, obj_offset, &c, &lk, &slot);
    uint64_t r = 0;
    if (node) r = sbo_offset_raw(node->objlevel, (uint8_t)SBO_START_LEN(node->states[slot]));
    if (lk) sbo_unlock(lk);
    return r;
}

void *sbo_data_ptr(void *metaptr, void *data_base, uint64_t obj_offset) {
    (void)metaptr;
    if (!data_base) return NULL;
    return (uint8_t *)data_base + obj_offset;
}

size_t sbo_meta_size(size_t data_size, size_t per_slot_pool) {
    uint8_t shards = sbo_shards_for_data(data_size);
    return sbo_header_size(shards) + (size_t)shards * per_slot_pool;
}

#endif /* SLOTSBOXMALLOC_IMPLEMENTATION */
