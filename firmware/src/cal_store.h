// Calibration storage that survives a power cut at any point of a save.
//
// Two raw 4 KB flash sectors ("slots"). A save erases the slot that does NOT
// hold the newest good record and programs one 256-byte page into it, so a
// power cut anywhere leaves the other slot -- the previous calibration --
// untouched. A record counts only if its magic, version, size and CRC-32
// check out; of two good records the higher sequence number wins. Loading is
// a memcpy and a CRC, so a damaged slot can never hang the boot.
//
// This replaces calibration.txt on LittleFS. arduino-pico's File open("w")
// syncs straight away, committing the truncated (empty) file before a byte of
// data is written, so a cut during the rest of the save left an EMPTY file:
// 37% of cut points in host_test/littlefs_powercut.c, against 7% that kept
// the old calibration. littlefs itself never failed there (no assert, no bad
// read, 11.8k trials) -- the empty file is what bricked the demo, which then
// reported "no valid calibration" with the orange LED.
//
// No Arduino or pico-sdk dependencies: main.cpp supplies the flash access and
// host_test/cal_store_test.cpp drives the same code with simulated power cuts.
#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define CAL_MAGIC       0x314C4143u  // "CAL1", little-endian
#define CAL_VERSION     1
#define CAL_SECTOR_SIZE 4096u        // erase unit
#define CAL_PAGE_SIZE   256u         // program unit

// What a save captures: the alignment results initFOC() would otherwise have
// to measure again (sensor offset/direction, and driverAlign()'s phase-to-ADC
// mapping as the signed gains), plus the hall pin order on halls builds.
struct CalData {
    float   zero_electric_angle;
    int32_t sensor_direction;        // Direction::CW (1) or CCW (-1)
    float   gain_a, gain_b, gain_c;
    int32_t hall_a, hall_b, hall_c;  // -1 on builds without hall sensors
};

struct CalRecord {
    uint32_t magic;
    uint16_t version;
    uint16_t size;          // sizeof(CalRecord)
    uint32_t seq;           // 1 for the first save, +1 per save
    uint32_t motor_config;  // MOTOR_CONFIG of the build that saved it
    CalData  data;
    uint32_t crc;           // CRC-32 of every byte before this field
};
static_assert(sizeof(CalRecord) <= CAL_PAGE_SIZE, "a record must fit one flash page");

// Flash access. read[] is each slot's memory-mapped address; erase() clears
// one slot's sector and program() writes its first page.
struct CalFlash {
    const uint8_t *read[2];
    void (*erase)(int slot);
    void (*program)(int slot, const uint8_t *page);
};

enum CalSlotState { CAL_SLOT_GOOD, CAL_SLOT_ERASED, CAL_SLOT_BAD };

static inline uint32_t calCrc32(const uint8_t *p, size_t n) {
    uint32_t crc = 0xFFFFFFFFu;
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

static inline CalSlotState calReadSlot(const CalFlash &f, int slot, CalRecord *rec) {
    memcpy(rec, f.read[slot], sizeof(*rec));
    if (rec->magic == CAL_MAGIC && rec->version == CAL_VERSION && rec->size == sizeof(*rec) &&
        rec->crc == calCrc32((const uint8_t *)rec, offsetof(CalRecord, crc)))
        return CAL_SLOT_GOOD;
    for (size_t i = 0; i < sizeof(*rec); i++)
        if (f.read[slot][i] != 0xFF) return CAL_SLOT_BAD;
    return CAL_SLOT_ERASED;
}

// Index of the slot holding the newest good record (copied to *rec), or -1.
// state[] reports both slots so the caller can say why nothing loaded.
static inline int calNewest(const CalFlash &f, CalRecord *rec, CalSlotState state[2]) {
    CalRecord r[2];
    for (int s = 0; s < 2; s++) state[s] = calReadSlot(f, s, &r[s]);
    int best = -1;
    for (int s = 0; s < 2; s++)
        if (state[s] == CAL_SLOT_GOOD && (best < 0 || r[s].seq > r[best].seq)) best = s;
    if (best >= 0) *rec = r[best];
    return best;
}

// Writes a new record into the slot not holding the newest good one, then
// reads it back. Returns the new sequence number, or 0 if the read-back did
// not match (the previous calibration is still intact in the other slot).
static inline uint32_t calSave(const CalFlash &f, uint32_t motor_config, const CalData &d) {
    CalRecord prev;
    CalSlotState state[2];
    int newest = calNewest(f, &prev, state);
    int target = (newest == 0) ? 1 : 0;

    uint8_t page[CAL_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));  // unprogrammed bytes stay erased
    CalRecord rec;
    memset(&rec, 0, sizeof(rec));      // zero the padding so the CRC is repeatable
    rec.magic = CAL_MAGIC;
    rec.version = CAL_VERSION;
    rec.size = sizeof(rec);
    rec.seq = (newest >= 0) ? prev.seq + 1 : 1;
    rec.motor_config = motor_config;
    rec.data = d;
    rec.crc = calCrc32((const uint8_t *)&rec, offsetof(CalRecord, crc));
    memcpy(page, &rec, sizeof(rec));

    f.erase(target);
    f.program(target, page);

    CalRecord check;
    if (calReadSlot(f, target, &check) != CAL_SLOT_GOOD || memcmp(&check, &rec, sizeof(rec)) != 0)
        return 0;
    return rec.seq;
}

// Sanity checks a loaded record before it is trusted to run a motor. Returns
// nullptr when it is usable, else the reason. gain_mag is this build's
// 1/(shunt * amp gain): driverAlign() only swaps and negates the gains, so
// each one must have exactly that magnitude -- a record from a build with a
// different shunt fails here instead of mis-scaling every current reading.
// hall_pins is the build's three hall GPIOs, or nullptr on non-hall builds.
static inline const char *calCheck(const CalRecord &r, uint32_t motor_config, float gain_mag,
                                   const int32_t *hall_pins) {
    const CalData &d = r.data;
    if (r.motor_config != motor_config) return "saved by a build for a different motor config";
    if (d.sensor_direction != 1 && d.sensor_direction != -1) return "sensor direction is not CW/CCW";
    // initFOC() leaves the angle in [0, 2pi] (_normalizeAngle).
    if (!(d.zero_electric_angle >= 0.0f && d.zero_electric_angle <= 6.2832f))
        return "zero electric angle outside 0..2pi";
    const float g[3] = {d.gain_a, d.gain_b, d.gain_c};
    for (int i = 0; i < 3; i++)
        if (!(fabsf(fabsf(g[i]) - gain_mag) <= 0.01f * gain_mag))
            return "current sense gain does not match this build's shunt";
    if (hall_pins) {
        const int32_t h[3] = {d.hall_a, d.hall_b, d.hall_c};
        for (int i = 0; i < 3; i++) {
            int matches = 0;
            for (int j = 0; j < 3; j++) matches += (h[j] == hall_pins[i]);
            if (matches != 1) return "hall pin order is not a permutation of the hall GPIOs";
        }
    }
    return nullptr;
}
