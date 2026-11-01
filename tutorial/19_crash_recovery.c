#define _POSIX_C_SOURCE 200809L
/* Crash recovery test: zombie takeover, ESRCH takeover, tree repair, and SIGKILL churn. */
#include <errno.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#define BLOCKMALLOC_IMPLEMENTATION
#include <blockmalloc/blockmalloc.h>
#define SLOTSBOXMALLOC_IMPLEMENTATION
#include <slotsboxmalloc/slotsboxobj.h>

#define DATA_SIZE (8ULL * 64 * 64 * 64) /* 2 MiB single-shard arena for deep tree splits */
#define POOL_SIZE (512 * 1024)
#define UNITS     (DATA_SIZE / 8)
#define FAIL64    ((uint64_t)-1)

typedef struct {
    _Atomic int ready;
    _Atomic uint64_t ops;
    _Atomic uint8_t claim[UNITS];
} shared_ctl_t;

static uint64_t expect_cap(size_t sz) {
    uint64_t n = (sz + 7) / 8;
    if (n <= 63) return n * 8;
    uint64_t base = 1;
    while (63 * base < n) base *= 64;
    return ((n + base - 1) / base) * base * 8;
}

static int test_zombie_and_corrupt_repair(void) {
    size_t ms = sbo_meta_size(DATA_SIZE, POOL_SIZE);
    size_t total = ((ms + 4095) & ~(size_t)4095) + sizeof(shared_ctl_t);
    uint8_t *shm = mmap(NULL, total, PROT_READ | PROT_WRITE,
                        MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (shm == MAP_FAILED) return 1;
    memset(shm, 0, total);
    shared_ctl_t *ctl = (shared_ctl_t *)(shm + ((ms + 4095) & ~(size_t)4095));
    if (sbo_init(shm, ms, DATA_SIZE) != 0) return 1;

    uint64_t keep0 = sbo_alloc(shm, 24);
    uint64_t keep1 = sbo_alloc(shm, 4096);
    if (keep0 == FAIL64 || keep1 == FAIL64) return 1;
    uint64_t used_before_orphan = sbo_pool_meta((sbo_meta_t *)shm, 0)->used_blocks;

    /* Phase 1: child acquires shard lock, corrupts cached masks/hot_id1, gets SIGKILLed
     * WITHOUT waitpid first (zombie 'Z' state). Parent must steal and repair! */
    pid_t pid = fork();
    if (pid < 0) return 1;
    if (pid == 0) {
        sbo_meta_t *m = (sbo_meta_t *)shm;
        sbo_lock_t *lk = sbo_pool_lock(m, 0);
        sbo_lock(lk);
        sbo_ctx_t c = sbo_ctx_for(m, 0, lk);
        sbo_box_t *root = sbo_box_at(&c, 0);
        /* Simulate crash mid-mutation: corrupt derived masks and hot cache while
         * authoritative states[] + children[] remain valid. */
        root->free_mask = 0;
        root->max_obj_cap = 0;
        root->child_max_enc = 0;
        root->child_max_count = 0;
        lk->hot_id1[0] = 999999;
        /* Also allocate an unlinked orphan block in sbo_pool to verify sweep reclaims it. */
        (void)blocks_alloc(c.pool_meta, c.pool);
        atomic_store_explicit(&ctl->ready, 1, memory_order_release);
        for (;;) pause();
    }

    while (atomic_load_explicit(&ctl->ready, memory_order_acquire) == 0)
        sbo_cpu_relax();
    kill(pid, SIGKILL);
    /* Wait briefly for child to enter zombie 'Z' state WITHOUT reaping via waitpid. */
    for (int i = 0; i < 500; i++) {
        if (sbo_pid_dead((int)pid)) break;
        struct timespec ts = {0, 1000000};
        nanosleep(&ts, NULL);
    }

    /* Allocating while holder is an unreaped zombie must steal lock and repair tree. */
    uint64_t after_z = sbo_alloc(shm, 64);
    if (after_z == FAIL64) {
        fprintf(stderr, "zombie takeover allocation failed\n");
        return 1;
    }
    int st = 0;
    waitpid(pid, &st, 0);

    if (sbo_pool_meta((sbo_meta_t *)shm, 0)->used_blocks != used_before_orphan) {
        fprintf(stderr, "orphan pool block leaked: used_blocks=%llu expected=%llu\n",
                (unsigned long long)sbo_pool_meta((sbo_meta_t *)shm, 0)->used_blocks,
                (unsigned long long)used_before_orphan);
        return 1;
    }

    if (sbo_allocated_size(shm, keep0) != expect_cap(24) ||
        sbo_allocated_size(shm, keep1) != expect_cap(4096) ||
        sbo_allocated_size(shm, after_z) != expect_cap(64)) {
        fprintf(stderr, "post-repair allocated_size mismatch\n");
        return 1;
    }
    sbo_free(shm, keep0);
    sbo_free(shm, keep1);
    sbo_free(shm, after_z);

    /* Full-arena allocation triggers lazy empty-box reclaim and must succeed if no capacity leaked. */
    size_t maxsz = DATA_SIZE / 64 * 63;
    uint64_t big = sbo_alloc(shm, maxsz);
    if (big == FAIL64 || sbo_allocated_size(shm, big) != expect_cap(maxsz)) {
        fprintf(stderr, "post-repair max allocation failed (leaked capacity)\n");
        return 1;
    }
    sbo_free(shm, big);
    munmap(shm, total);
    return 0;
}

static void churn_loop(uint8_t *meta, uint8_t *data, shared_ctl_t *ctl, int wid) {
    static const size_t szs[] = {8, 24, 64, 200, 512, 4096, 32768};
    uint64_t live[64];
    int nl = 0;
    uint64_t rs = 0x9e3779b97f4a7c15ULL ^ ((uint64_t)getpid() * 0xbf58476d1ce4e5b9ULL)
                  ^ (uint64_t)wid;
    for (;;) {
        rs ^= rs >> 12; rs ^= rs << 25; rs ^= rs >> 27;
        uint64_t r = rs * 2685821657736338717ULL;
        if (nl > 0 && (nl == 64 || (r & 1))) {
            int idx = (int)((r >> 8) % (uint64_t)nl);
            sbo_free(meta, live[idx]);
            live[idx] = live[--nl];
        } else {
            size_t req = szs[(r >> 16) % (sizeof(szs) / sizeof(szs[0]))];
            uint64_t off = sbo_alloc(meta, req);
            if (off != FAIL64) {
                memset(data + off, (int)(wid + 1), req < 64 ? req : 64);
                live[nl++] = off;
            }
        }
        atomic_fetch_add_explicit(&ctl->ops, 1, memory_order_relaxed);
    }
}

static int test_sigkill_churn(void) {
    size_t ms = sbo_meta_size(DATA_SIZE, POOL_SIZE);
    size_t meta_aligned = (ms + 4095) & ~(size_t)4095;
    size_t total = meta_aligned + DATA_SIZE + sizeof(shared_ctl_t);
    uint8_t *shm = mmap(NULL, total, PROT_READ | PROT_WRITE,
                        MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (shm == MAP_FAILED) return 1;
    memset(shm, 0, total);
    uint8_t *data = shm + meta_aligned;
    shared_ctl_t *ctl = (shared_ctl_t *)(data + DATA_SIZE);
    if (sbo_init(shm, ms, DATA_SIZE) != 0) return 1;

    /* Pre-allocate anchor objects owned by the parent that must survive all crashes. */
    uint64_t anchors[16];
    uint64_t anchor_caps[16];
    for (int i = 0; i < 16; i++) {
        size_t sz = (size_t)(8 * (i + 1) * ((i & 1) ? 17 : 1));
        anchors[i] = sbo_alloc(shm, sz);
        if (anchors[i] == FAIL64) return 1;
        anchor_caps[i] = expect_cap(sz);
        uint64_t tag = 0xA5A5000000000000ULL | (uint64_t)i;
        memcpy(data + anchors[i], &tag, 8);
        for (uint64_t u = anchors[i] / 8; u < (anchors[i] + anchor_caps[i]) / 8; u++)
            atomic_store_explicit(&ctl->claim[u], 1, memory_order_relaxed);
    }

    /* Repeatedly spawn 4 worker processes doing tight alloc/free churn and SIGKILL them. */
    for (int round = 0; round < 20; round++) {
        pid_t pids[4];
        uint64_t start_ops = atomic_load_explicit(&ctl->ops, memory_order_relaxed);
        for (int w = 0; w < 4; w++) {
            pids[w] = fork();
            if (pids[w] == 0) {
                churn_loop(shm, data, ctl, w + 1);
                _exit(0);
            }
        }
        while (atomic_load_explicit(&ctl->ops, memory_order_relaxed) - start_ops < 2000) {
            struct timespec ts = {0, 200000};
            nanosleep(&ts, NULL);
        }
        for (int w = 0; w < 4; w++)
            kill(pids[w], SIGKILL);
        /* Deliberately allocate BEFORE reaping half the workers to exercise zombie path too. */
        uint64_t probe = sbo_alloc(shm, 32);
        if (probe != FAIL64)
            sbo_free(shm, probe);
        for (int w = 0; w < 4; w++) {
            int st = 0;
            waitpid(pids[w], &st, 0);
        }
    }

    /* Verify all parent anchors remain intact and non-overlapping with new allocations. */
    for (int i = 0; i < 16; i++) {
        if (sbo_allocated_size(shm, anchors[i]) != anchor_caps[i]) {
            fprintf(stderr, "anchor %d allocated_size corrupted\n", i);
            return 1;
        }
        uint64_t tag = 0, exp = 0xA5A5000000000000ULL | (uint64_t)i;
        memcpy(&tag, data + anchors[i], 8);
        if (tag != exp) {
            fprintf(stderr, "anchor %d payload overwritten by churn!\n", i);
            return 1;
        }
    }

    uint64_t fresh[64];
    int nf = 0;
    for (int i = 0; i < 64; i++) {
        uint64_t off = sbo_alloc(shm, 64);
        if (off == FAIL64) break;
        uint64_t cap = sbo_allocated_size(shm, off);
        if (cap != 64) {
            fprintf(stderr, "fresh alloc bad cap=%lu\n", (unsigned long)cap);
            return 1;
        }
        for (uint64_t u = off / 8; u < (off + cap) / 8; u++) {
            uint8_t prev = atomic_exchange_explicit(&ctl->claim[u], 2, memory_order_relaxed);
            if (prev != 0) {
                fprintf(stderr, "overlap after crash recovery at unit=%lu prev=%u\n",
                        (unsigned long)u, prev);
                return 1;
            }
        }
        fresh[nf++] = off;
    }
    if (nf == 0) {
        fprintf(stderr, "unable to allocate after crash churn\n");
        return 1;
    }
    for (int i = 0; i < nf; i++) sbo_free(shm, fresh[i]);
    for (int i = 0; i < 16; i++) sbo_free(shm, anchors[i]);
    munmap(shm, total);
    return 0;
}

static int test_interrupted_grow_repair(void) {
    /* Use single-shard data size (2 MiB = 2^21 = 8 * 64^3, < SBO_MIN_SHARD_BYTES * 64 -> 1 shard). */
    const size_t ds1 = 2 * 1024 * 1024;
    size_t ms = sbo_meta_size(ds1, POOL_SIZE);
    uint8_t *shm = mmap(NULL, ms, PROT_READ | PROT_WRITE,
                        MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (shm == MAP_FAILED) return 1;
    memset(shm, 0, ms);
    if (sbo_init(shm, ms, ds1) != 0) return 1;

    uint64_t high = 0;
    sbo_meta_t *m = (sbo_meta_t *)shm;
    if (m->root_slots != 1) return 1;
    /* Simulate crash mid-sbo_repair_shard after slot_shift += 6 but before data_size updated. */
    blocks_meta_t *pm = sbo_pool_meta(m, 0);
    uint8_t *pool = sbo_pool_mem(m, 0);
    sbo_lock_t *lk = sbo_pool_lock(m, 0);
    uint64_t keep = sbo_alloc(shm, 64);
    if (keep == FAIL64) return 1;

    int64_t nid = blocks_alloc(pm, pool);
    if (nid <= 0) return 1;
    sbo_ctx_t c = sbo_ctx_for(m, 0, lk);
    sbo_box_t *root = sbo_box_at(&c, 0);
    sbo_box_t *nb = sbo_box_at(&c, (int32_t)nid);
    memcpy(nb, root, sizeof(*root));
    uint8_t want_lvl = nb->objlevel;
    sbo_box_format(root, (uint8_t)(want_lvl + 1), -1, 255, 0);
    root->children[0] = (int32_t)nid;
    root->states[0] = SBO_BOX;
    root->free_mask = ~1ULL;
    root->child_mask = 1ULL;
    m->slot_shift = (uint8_t)(m->slot_shift + SBO_N_BITS);
    /* Leave m->data_size at old ds1 and lock held by dead PID (e.g. crashed mid-repair). */
    pid_t pid = fork();
    if (pid == 0) {
        __atomic_store_n(&lk->lock, (uint32_t)getpid(), __ATOMIC_RELEASE);
        _exit(0);
    }
    int st = 0;
    waitpid(pid, &st, 0);

    /* sbo_allocated_size and sbo_alloc must both recover data_size = 1ULL << slot_shift cleanly. */
    if (sbo_allocated_size(shm, keep) != 64) {
        fprintf(stderr, "interrupted grow repair: sbo_allocated_size failed\n");
        return 1;
    }
    high = sbo_alloc(shm, ds1 * 2);
    if (high == FAIL64 || m->data_size != (1ULL << m->slot_shift)) {
        fprintf(stderr, "interrupted grow repair: post-repair large alloc failed\n");
        return 1;
    }
    sbo_free(shm, high);
    sbo_free(shm, keep);
    munmap(shm, ms);
    return 0;
}

int main(void) {
    if (test_zombie_and_corrupt_repair() != 0) return 1;
    if (test_interrupted_grow_repair() != 0) return 1;
    if (test_sigkill_churn() != 0) return 1;
    printf("19_crash_recovery PASS\n");
    return 0;
}
