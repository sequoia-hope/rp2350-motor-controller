// Why calibration left LittleFS (see src/cal_store.h). Power-cut simulation of
// the old save_calibration(), using the framework's own littlefs (v2.10) with
// arduino-pico's exact config and the exact call sequence its LittleFS wrapper
// makes (FS::open "w" with the default time callback, close() writing the
// 'c'/'t' attributes, begin() autoformat). `make -C firmware/host_test littlefs`
//
// Each trial: format, save OLD P times, then save NEW with a power cut injected
// at flash op i (before / partial / after the op). Then "reboot" exactly like
// load_calibration_and_init(): begin() (mount, autoformat on failure), open
// "calibration.txt" read-only, read a line. Each trial runs in a forked child
// so an assert (board: panic) or a memory fault (board: hardfault) is counted
// instead of killing the run. Same NOR power-cut model as cal_store_test.cpp.
//
// Result (prior saves 1..200, 16 seeds): 37% of cut points leave an EMPTY
// calibration.txt -- the wrapper's open() syncs, committing the truncation
// before any data -- and only 7% keep the old calibration. littlefs itself
// never asserted, faulted (ASan/UBSan clean) or failed to mount.
#define LFS_NAME_MAX 255
#define LFS_NO_DEBUG
#define LFS_NO_WARN
#define LFS_NO_ERROR
#include LFS_C

#include <setjmp.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#define BS 4096
#define NB 16
static uint8_t flash[NB * BS];

enum { CUT_BEFORE, CUT_PARTIAL, CUT_AFTER };
static long op_count, cut_at = -1;
static int cut_mode;
static uint32_t rng = 1;
static jmp_buf cut_jb;

static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

// glibc's assert() lands here; on the board it is a panic (hang).
void __assert_fail(const char *e, const char *f, unsigned l, const char *fn) {
    (void)f; (void)l; (void)fn;
    fprintf(stderr, "ASSERT %s\n", e);
    _exit(10);
}

static int bd_read(const struct lfs_config *c, lfs_block_t b, lfs_off_t off, void *dst, lfs_size_t n) {
    (void)c; memcpy(dst, flash + b * BS + off, n); return 0;
}
static int bd_prog(const struct lfs_config *c, lfs_block_t b, lfs_off_t off, const void *src, lfs_size_t n) {
    (void)c;
    uint8_t *p = flash + b * BS + off;
    const uint8_t *s = src;
    if (op_count++ == cut_at) {
        if (cut_mode == CUT_PARTIAL) {
            // NOR program only clears bits: some bytes done, one half-done.
            lfs_size_t k = rnd() % n;
            for (lfs_size_t i = 0; i < k; i++) p[i] &= s[i];
            p[k] &= (s[k] | (uint8_t)rnd());
        }
        if (cut_mode == CUT_AFTER) for (lfs_size_t i = 0; i < n; i++) p[i] &= s[i];
        longjmp(cut_jb, 1);
    }
    for (lfs_size_t i = 0; i < n; i++) p[i] &= s[i];
    return 0;
}
static int bd_erase(const struct lfs_config *c, lfs_block_t b) {
    (void)c;
    uint8_t *p = flash + b * BS;
    if (op_count++ == cut_at) {
        // An interrupted erase leaves cells part-way: bits drift toward 1.
        if (cut_mode == CUT_PARTIAL) for (int i = 0; i < BS; i++) p[i] |= (uint8_t)rnd();
        if (cut_mode == CUT_AFTER) memset(p, 0xff, BS);
        longjmp(cut_jb, 1);
    }
    memset(p, 0xff, BS);
    return 0;
}
static int bd_sync(const struct lfs_config *c) { (void)c; return 0; }

// arduino-pico LittleFSImpl: memset cfg, then these fields.
static struct lfs_config cfg;
static lfs_t lfs;
static void cfg_init(void) {
    memset(&cfg, 0, sizeof cfg);
    cfg.read = bd_read; cfg.prog = bd_prog; cfg.erase = bd_erase; cfg.sync = bd_sync;
    cfg.read_size = 256; cfg.prog_size = 256; cfg.block_size = BS; cfg.block_count = NB;
    cfg.block_cycles = 16; cfg.cache_size = 256; cfg.lookahead_size = 256;
}

static int formatted;  // begin() had to autoformat (calibration wiped)
static int begin(void) {  // LittleFSImpl::begin, autoFormat = true
    memset(&lfs, 0, sizeof lfs);
    if (lfs_mount(&lfs, &cfg) == 0) return 1;
    formatted = 1;
    if (lfs_format(&lfs, &cfg)) return 0;
    memset(&lfs, 0, sizeof lfs);
    return lfs_mount(&lfs, &cfg) == 0;
}

static const char *OLD = "1.234567,1,2.500000,-2.500000,2.500000";
static const char *NEW = "4.567890,-1,-2.500000,2.500000,2.500000";

// save_calibration(): begin(); open("w"); print x5 fields; close(); end().
static void save(const char *line) {
    if (!begin()) return;
    lfs_file_t f;
    time_t creation = 0;
    // FS::open with OM_CREATE and a time callback: probe whether it exists.
    if (lfs_file_open(&lfs, &f, "calibration.txt", LFS_O_RDONLY) == 0) lfs_file_close(&lfs, &f);
    else creation = 1234;
    if (lfs_file_open(&lfs, &f, "calibration.txt", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) == 0) {
        lfs_file_sync(&lfs, &f);  // the wrapper syncs right after open
        const char *p = line;
        while (*p) {  // one write per comma-separated field, like file.print()
            const char *q = strchr(p, ',');
            size_t n = q ? (size_t)(q - p + 1) : strlen(p);
            lfs_file_write(&lfs, &f, p, n);
            p += n;
        }
        lfs_file_close(&lfs, &f);
        time_t now = 5678;
        if (creation) lfs_setattr(&lfs, "calibration.txt", 'c', &creation, sizeof creation);
        lfs_setattr(&lfs, "calibration.txt", 't', &now, sizeof now);
    }
    lfs_unmount(&lfs);
}

enum { R_OLD, R_NEW, R_EMPTY, R_MISSING, R_WIPED, R_BADPARSE, R_ASSERT, R_CRASH, R_N };
#define RESAVE_FAILED 0x40
static const char *rname[R_N] = {"old cal kept", "new cal", "EMPTY file", "file MISSING",
                                 "FS WIPED (autoformat)", "garbage content", "ASSERT (panic)", "CRASH (hardfault)"};

// load_calibration_and_init(): begin(); open("r"); readStringUntil('\n').
static int load(void) {
    formatted = 0;
    if (!begin()) return R_MISSING;
    if (formatted) return R_WIPED;
    lfs_file_t f;
    char buf[128] = {0};
    if (lfs_file_open(&lfs, &f, "calibration.txt", LFS_O_RDONLY)) { lfs_unmount(&lfs); return R_MISSING; }
    lfs_ssize_t n = lfs_file_read(&lfs, &f, buf, sizeof buf - 1);
    lfs_file_close(&lfs, &f);
    lfs_unmount(&lfs);
    if (n <= 0) return R_EMPTY;
    if (!strcmp(buf, OLD)) return R_OLD;
    if (!strcmp(buf, NEW)) return R_NEW;
    return R_BADPARSE;
}

static int trial(int prior, long cut, int mode, uint32_t seed) {
    cfg_init();
    memset(flash, 0xff, sizeof flash);
    op_count = 0; cut_at = -1; rng = seed;
    lfs_format(&lfs, &cfg);
    for (int i = 0; i < prior; i++) save(OLD);
    op_count = 0; cut_at = cut; cut_mode = mode;
    if (setjmp(cut_jb) == 0) save(NEW);
    cut_at = -1;
    int r = load();
    // A board that came back must still be able to save and load.
    save(NEW);
    if (load() != R_NEW) r |= RESAVE_FAILED;
    return r;
}

int main(int argc, char **argv) {
    int max_prior = argc > 1 ? atoi(argv[1]) : 40;
    int seeds = argc > 2 ? atoi(argv[2]) : 4;
    long tally[R_N] = {0}, resave_fail = 0, total = 0;
    long first[R_N]; for (int i = 0; i < R_N; i++) first[i] = -1;
    char where[R_N][80] = {{0}};
    for (int prior = 1; prior <= max_prior; prior++) {
        // Count the flash ops in this save.
        cfg_init(); memset(flash, 0xff, sizeof flash); cut_at = -1; lfs_format(&lfs, &cfg);
        for (int i = 0; i < prior; i++) save(OLD);
        op_count = 0; save(NEW);
        long ops = op_count;
        for (long cut = 0; cut < ops; cut++)
            for (int mode = 0; mode < 3; mode++)
                for (int s = 0; s < (mode == CUT_PARTIAL ? seeds : 1); s++) {
                    fflush(stdout);
                    pid_t pid = fork();
                    if (pid == 0) _exit(trial(prior, cut, mode, 0x9e3779b9u * (s + 1) + cut));
                    int st; waitpid(pid, &st, 0);
                    int r;
                    if (WIFSIGNALED(st)) r = R_CRASH;
                    else if (WEXITSTATUS(st) == 10) r = R_ASSERT;
                    else {
                        r = WEXITSTATUS(st);
                        if (r & RESAVE_FAILED) { resave_fail++; r &= ~RESAVE_FAILED; }
                        if (r >= R_N) r = R_CRASH;  // ASan/UBSan abort exit code
                    }
                    tally[r]++; total++;
                    if (first[r] < 0) { first[r] = total; snprintf(where[r], 80, "prior=%d op=%ld/%ld mode=%d", prior, cut, ops, mode); }
                }
    }
    printf("%ld power-cut trials (prior saves 1..%d, every flash op, before/partial/after)\n", total, max_prior);
    for (int i = 0; i < R_N; i++)
        if (tally[i]) printf("  %-24s %6ld  (%5.1f%%)  e.g. %s\n", rname[i], tally[i], 100.0 * tally[i] / total, where[i]);
    printf("  board could not save+load afterwards: %ld\n", resave_fail);
    return 0;
}
