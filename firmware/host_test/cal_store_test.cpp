// Host test for src/cal_store.h: drives the real save/load code against a
// simulated NOR flash and cuts the power at every flash operation -- before
// it, part-way through it, or just after it -- then checks what a reboot
// would load. Run with `make -C firmware/host_test`.
//
// Power-cut model (NOR): program only clears bits; an interrupted program has
// done some bytes and half-done one; an interrupted erase leaves every cell
// part-way, bits drifting toward 1.
#include "../src/cal_store.h"

#include <stdio.h>
#include <stdlib.h>

static uint8_t sim[2][CAL_SECTOR_SIZE];
struct PowerCut {};
enum { CUT_BEFORE, CUT_PARTIAL, CUT_AFTER, CUT_MODES };
static long ops, cut_at = -1;
static int cut_mode;
static unsigned partial_k;  // bytes an interrupted program completed
static int stuck_bit = -1;  // a bit the next program fails to clear (worn cell)
static uint32_t rng = 1;
static uint32_t rnd() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

static void simErase(int slot) {
    uint8_t *p = sim[slot];
    if (ops++ == cut_at) {
        if (cut_mode == CUT_PARTIAL) for (unsigned i = 0; i < CAL_SECTOR_SIZE; i++) p[i] |= (uint8_t)rnd();
        if (cut_mode == CUT_AFTER) memset(p, 0xFF, CAL_SECTOR_SIZE);
        throw PowerCut();
    }
    memset(p, 0xFF, CAL_SECTOR_SIZE);
}
static void simProgram(int slot, const uint8_t *page) {
    uint8_t *p = sim[slot];
    if (ops++ == cut_at) {
        if (cut_mode == CUT_PARTIAL) {
            unsigned k = partial_k = rnd() % CAL_PAGE_SIZE;
            for (unsigned i = 0; i < k; i++) p[i] &= page[i];
            p[k] &= (uint8_t)(page[k] | rnd());
        }
        if (cut_mode == CUT_AFTER) for (unsigned i = 0; i < CAL_PAGE_SIZE; i++) p[i] &= page[i];
        throw PowerCut();
    }
    for (unsigned i = 0; i < CAL_PAGE_SIZE; i++) p[i] &= page[i];
    if (stuck_bit >= 0) p[stuck_bit / 8] |= (uint8_t)(1u << (stuck_bit % 8));
}
static const CalFlash flash = {{sim[0], sim[1]}, simErase, simProgram};

static long failures;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; if (failures <= 20) { \
    printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static CalData make(uint32_t n) {  // distinct, valid data per save
    CalData d = {};
    d.zero_electric_angle = 0.001f * (float)(n % 6000);
    d.sensor_direction = (n & 1) ? 1 : -1;
    d.gain_a = 2.5f; d.gain_b = (n & 2) ? -2.5f : 2.5f; d.gain_c = -2.5f;
    d.hall_a = 31; d.hall_b = 32; d.hall_c = 33;
    return d;
}
static bool same(const CalData &a, const CalData &b) { return memcmp(&a, &b, sizeof(a)) == 0; }

// What a reboot loads: 0 = nothing, else the data's tag n (make(n)).
static long loadTag(const long *tags, long ntags) {
    CalRecord rec; CalSlotState st[2];
    if (calNewest(flash, &rec, st) < 0) return 0;
    for (long i = 0; i < ntags; i++) if (same(rec.data, make((uint32_t)tags[i]))) return tags[i];
    return -1;  // a record nobody saved: corruption got through
}

static void fill(int how) {  // initial flash contents
    for (int s = 0; s < 2; s++)
        for (unsigned i = 0; i < CAL_SECTOR_SIZE; i++)
            sim[s][i] = how == 0 ? 0xFF : (uint8_t)rnd();  // erased, or old LittleFS/garbage
}

// Every cut point of one save, from every starting history.
static void exhaustiveCuts() {
    long trials = 0, kept_old = 0, got_new = 0;
    for (int start = 0; start < 2; start++)
        for (int prior = 0; prior <= 4; prior++)
            for (long cut = 0; cut < 2; cut++)  // a save is one erase + one program
                for (int mode = 0; mode < CUT_MODES; mode++)
                    for (int seed = 0; seed < (mode == CUT_PARTIAL ? 500 : 1); seed++) {
                        rng = 0x9E3779B9u * (uint32_t)(seed + 1) + (uint32_t)(cut * 7 + prior);
                        fill(start);
                        ops = 0; cut_at = -1;
                        for (int i = 1; i <= prior; i++) CHECK(calSave(flash, 1, make(i)) == (uint32_t)i, "prior save");
                        long tags[2] = {prior, prior + 1};
                        cut_at = cut; ops = 0; cut_mode = mode;
                        try { calSave(flash, 1, make(prior + 1)); CHECK(false, "cut did not fire"); } catch (PowerCut &) {}
                        cut_at = -1;
                        long got = loadTag(tags, 2);
                        trials++;
                        // Old calibration, or the new one once every byte of its record
                        // is programmed (the rest of the page is erased padding). An
                        // interrupted program that stopped on the record's last byte
                        // may or may not have finished it.
                        bool on_program = (cut == 1);
                        bool must_new = on_program && (mode == CUT_AFTER ||
                                        (mode == CUT_PARTIAL && partial_k >= sizeof(CalRecord)));
                        bool may_new = must_new || (on_program && mode == CUT_PARTIAL &&
                                                    partial_k == sizeof(CalRecord) - 1);
                        if (got == prior) kept_old++;
                        if (got == prior + 1) got_new++;
                        CHECK(got == (must_new ? prior + 1 : prior) || (may_new && got == prior + 1),
                              "start=%d prior=%d cut=%ld mode=%d seed=%d k=%u loaded %ld",
                              start, prior, cut, mode, seed, partial_k, got);
                        // The board must save cleanly afterwards.
                        long tags3[1] = {prior + 2};
                        CHECK(calSave(flash, 1, make(prior + 2)) != 0 && loadTag(tags3, 1) == prior + 2,
                              "resave after cut (start=%d prior=%d cut=%ld mode=%d)", start, prior, cut, mode);
                    }
    printf("exhaustive: %ld cut trials, old kept %ld, new loaded %ld (only once its record was complete)\n",
           trials, kept_old, got_new);
}

// Long random history: saves with random cuts, including cuts during the save
// that follows a cut. Invariant: a reboot loads the last completed save, or
// the cut one only if its record got fully programmed; never nothing once a save
// has completed, never a record nobody wrote.
static void fuzz() {
    rng = 12345;
    fill(1);
    long done = 0, next = 1, saves = 0, cuts = 0;
    for (long it = 0; it < 200000; it++) {
        bool cut = rnd() % 2;
        long tag = next++;
        ops = 0;
        cut_at = cut ? (long)(rnd() % 2) : -1;
        cut_mode = (int)(rnd() % CUT_MODES);
        bool completed = false;
        try { completed = calSave(flash, 1, make((uint32_t)tag)) != 0; } catch (PowerCut &) {}
        cut_at = -1;
        long tags[2] = {done, tag};
        long got = loadTag(tags, 2);
        if (completed) { CHECK(got == tag, "it=%ld completed save not loaded (got %ld)", it, got); done = tag; saves++; }
        else {
            cuts++;
            CHECK(got == done || got == tag, "it=%ld cut save: loaded %ld, want %ld or %ld", it, got, done, tag);
            if (got == tag) done = tag;  // record finished just before the cut
        }
        if (done) CHECK(got != 0, "it=%ld lost every calibration", it);
    }
    printf("fuzz: %ld saves, %ld cuts, 200000 iterations\n", saves, cuts);
}

static void corruption() {
    // A bit flipped in the newest record falls back to the older one.
    fill(0);
    calSave(flash, 1, make(1));
    calSave(flash, 1, make(2));  // slot 1, seq 2
    long tags[2] = {1, 2};
    for (int bit = 0; bit < (int)sizeof(CalRecord) * 8; bit++) {
        sim[1][bit / 8] ^= (uint8_t)(1u << (bit % 8));
        CHECK(loadTag(tags, 2) == 1, "bit %d flipped in newest: did not fall back", bit);
        sim[1][bit / 8] ^= (uint8_t)(1u << (bit % 8));
    }
    CHECK(loadTag(tags, 2) == 2, "restored newest not loaded");

    // Both slots damaged: nothing loads, and the states say why.
    sim[0][20] ^= 1; sim[1][20] ^= 1;
    CalRecord rec; CalSlotState st[2];
    CHECK(calNewest(flash, &rec, st) < 0 && st[0] == CAL_SLOT_BAD && st[1] == CAL_SLOT_BAD, "both bad");
    fill(0);
    CHECK(calNewest(flash, &rec, st) < 0 && st[0] == CAL_SLOT_ERASED && st[1] == CAL_SLOT_ERASED, "both erased");

    // A cell that will not program: the save reports failure, the old record stays.
    calSave(flash, 1, make(1));
    // gain_a = 2.5f (0x40200000): its low byte is 0x00, so every bit must clear.
    stuck_bit = 8 * (int)(offsetof(CalRecord, data) + offsetof(CalData, gain_a));
    CHECK(calSave(flash, 1, make(2)) == 0, "stuck bit not caught by read-back");
    stuck_bit = -1;
    CHECK(loadTag(tags, 2) == 1, "old record lost after a failed save");

    // Sequence numbers climb and saves alternate slots.
    fill(0);
    for (uint32_t i = 1; i <= 6; i++) {
        CHECK(calSave(flash, 1, make(i)) == i, "seq %u", i);
        CalRecord r0, r1;
        CHECK(calReadSlot(flash, (int)((i - 1) % 2), &r0) == CAL_SLOT_GOOD && r0.seq == i, "slot alternation %u", i);
        if (i > 1) CHECK(calReadSlot(flash, (int)(i % 2), &r1) == CAL_SLOT_GOOD && r1.seq == i - 1, "older kept %u", i);
    }
    printf("corruption: bit flips, double damage, stuck cell, slot alternation\n");
}

static void checks() {
    const int32_t halls[3] = {31, 32, 33};
    CalRecord r = {};
    r.motor_config = 2;
    r.data = make(3);
    CHECK(calCheck(r, 2, 2.5f, halls) == nullptr, "valid record rejected");
    CHECK(calCheck(r, 1, 2.5f, nullptr) != nullptr, "other motor config accepted");
    CHECK(calCheck(r, 2, 5.0f, halls) != nullptr, "other shunt accepted");
    CalRecord b = r; b.data.sensor_direction = 0;
    CHECK(calCheck(b, 2, 2.5f, halls) != nullptr, "direction 0 accepted");
    b = r; b.data.zero_electric_angle = NAN;
    CHECK(calCheck(b, 2, 2.5f, halls) != nullptr, "NaN angle accepted");
    b = r; b.data.zero_electric_angle = -0.1f;
    CHECK(calCheck(b, 2, 2.5f, halls) != nullptr, "negative angle accepted");
    b = r; b.data.zero_electric_angle = 6.28318f;  // 2pi itself is reachable
    CHECK(calCheck(b, 2, 2.5f, halls) == nullptr, "2pi rejected");
    b = r; b.data.gain_b = 0.0f;
    CHECK(calCheck(b, 2, 2.5f, halls) != nullptr, "zero gain accepted");
    b = r; b.data.gain_c = NAN;
    CHECK(calCheck(b, 2, 2.5f, halls) != nullptr, "NaN gain accepted");
    b = r; b.data.hall_b = 31;
    CHECK(calCheck(b, 2, 2.5f, halls) != nullptr, "repeated hall pin accepted");
    b = r; b.data.hall_a = 33; b.data.hall_c = 31;
    CHECK(calCheck(b, 2, 2.5f, halls) == nullptr, "hall permutation rejected");
    b = r; b.data.hall_a = -1; b.data.hall_b = -1; b.data.hall_c = -1;
    CHECK(calCheck(b, 2, 2.5f, nullptr) == nullptr, "non-hall build checked hall pins");
    printf("value checks: config, direction, angle, gains, hall order\n");
}

int main() {
    exhaustiveCuts();
    fuzz();
    corruption();
    checks();
    if (failures) { printf("%ld FAILURES\n", failures); return 1; }
    printf("all passed\n");
    return 0;
}
