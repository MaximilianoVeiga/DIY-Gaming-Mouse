/*
 * Copyright (c) 2024, wareya
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <SPI.h>
#include <mbed.h>
#include <kvstore_global_api.h>

REDIRECT_STDOUT_TO(Serial);

#include <PluggableUSBHID.h>

#if __has_include("srom_3360_0x05.h")
#include "srom_3360_0x05.h"
#define HAVE_REAL_SROM 1
#else
#include "srom_dummy_blank.h"
#endif
#include "relmouse_16.h"
USBMouse16 mouse(false);

#define REG_PRODUCT_ID (0x00)
#define REG_REVISION_ID (0x01)
#define REG_MOTION (0x02)
#define REG_DELTA_X_L (0x03)
#define REG_DELTA_X_H (0x04)
#define REG_DELTA_Y_L (0x05)
#define REG_DELTA_Y_H (0x06)
#define REG_SQUAL (0x07)
#define REG_RAW_DATA_SUM (0x08)
#define REG_MAX_RAW_DATA (0x09)
#define REG_MIN_RAW_DATA (0x0A)
#define REG_SHUTTER_LOWER (0x0B)
#define REG_SHUTTER_UPPER (0x0C)
#define REG_CONTROL (0x0D)

#define REG_CONFIG1 (0x0F)
#define REG_CONFIG2 (0x10)
#define REG_ANGLE_TUNE (0x11)
#define REG_FRAME_CAPTURE (0x12)
#define REG_SROM_ENABLE (0x13)
#define REG_RUN_DOWNSHIFT (0x14)
#define REG_REST1_RATE_LOWER (0x15)
#define REG_REST1_RATE_UPPER (0x16)
#define REG_REST1_DOWNSHIFT (0x17)
#define REG_REST2_RATE_LOWER (0x18)
#define REG_REST2_RATE_UPPER (0x19)
#define REG_REST2_DOWNSHIFT (0x1A)
#define REG_REST3_RATE_LOWER (0x1B)
#define REG_REST3_RATE_UPPER (0x1C)

#define REG_OBSERVATION (0x24)
#define REG_DATA_OUT_LOWER (0x25)
#define REG_DATA_OUT_UPPER (0x26)

#define REG_RAW_DATA_DUMP (0x29)
#define REG_SROM_ID (0x2A)
#define REG_MIN_SQ_RUN (0x2B)
#define REG_RAW_DATA_THRESHOLD (0x2C)

#define REG_CONFIG5 (0x2F)

#define REG_POWER_UP_RESET (0x3A)
#define REG_SHUTDOWN (0x3B)

#define REG_INVERSE_PRODUCT_ID (0x3F)

#define REG_LIFTCUTOFF_TUNE3 (0x41)
#define REG_ANGLE_SNAP (0x42)

#define REG_LIFTCUTOFF_TUNE1 (0x4A)

#define REG_MOTION_BURST (0x50)

#define REG_LIFTCUTOFF_TUNE_TIMEOUT (0x58)

#define REG_LIFTCUTOFF_TUNE_MIN_LENGTH (0x5A)

#define REG_SROM_LOAD_BURST (0x62)
#define REG_LIFT_CONFIG (0x63)
#define REG_RAW_DATA_BURST (0x64)
#define REG_LIFTOFF_TUNE2 (0x65)

#define CONFIG2_REST_ENABLED (0x30)
#define CONFIG2_REPORT_MODE (0x04)

// PMW3360 identity / Observation
#define PMW3360_PRODUCT_ID 0x42
#define PMW3360_INVERSE_PRODUCT_ID 0xBD
#define OBS_SROM_RUNNING (1u << 6)

// Pins
#define BUTTONS_ON 0
#define BUTTONS_OFF 1
#define BUTTON_M1 2
#define BUTTON_M2 3
#define BUTTON_M3 4
#define BUTTON_DPI 5
#define BUTTON_M4 6
#define BUTTON_M5 7
#define ENCODER_A 8
#define ENCODER_B 10
#define ENCODER_COM 9
#define PIN_NCS 17
#define PIN_MISO 16
#define PIN_MOSI 19
#define PIN_SCK 18

// Physical button bit indexes in `buttons`
enum PhysButton : uint8_t {
    PHYS_M1 = 0,
    PHYS_M2 = 1,
    PHYS_M3 = 2,
    PHYS_DPI = 3,
    PHYS_M4 = 4,
    PHYS_M5 = 5,
    PHYS_COUNT = 6
};

#define PHYS_BIT(i) (1u << (i))

// HID report button bits (5-button report in relmouse_16.h)
enum HidButton : uint8_t {
    HID_LEFT = 0,
    HID_RIGHT = 1,
    HID_MIDDLE = 2,
    HID_BACK = 3,
    HID_FORWARD = 4,
    HID_NONE = 0xFF,
    // M3 and DPI share middle-click; last edge wins (legacy which_m3 behavior)
    HID_MIDDLE_SHARED = 0xFE
};

// Compile-time physical -> HID map. Edit here to rebind buttons.
// Default matches stock firmware: M3/DPI share middle, M4=back, M5=forward.
static const uint8_t BUTTON_MAP[PHYS_COUNT] = {
    HID_LEFT,           // PHYS_M1
    HID_RIGHT,          // PHYS_M2
    HID_MIDDLE_SHARED,  // PHYS_M3
    HID_MIDDLE_SHARED,  // PHYS_DPI
    HID_BACK,           // PHYS_M4
    HID_FORWARD         // PHYS_M5
};

// Timing (us unless noted)
#define T_NCS_SCLK_US 1
#define T_SRAD_US 160
#define T_SCLK_NCS_WRITE_US 35
#define T_SRAD_MOTBR_US 35
#define T_SWW_SWR_US 180
#define T_SRW_SRR_US 20
#define T_BEXIT_US 1
#define T_SROM_BYTE_US 15
#define WHEEL_POLL_US 250
#define BUTTON_LATCH_LOOPS 8
#define BOOT_SERIAL_DELAY_MS 500
#define SETTINGS_FLUSH_MS 2000
#define SENSOR_FAIL_STREAK_LIMIT 8
#define SENSOR_RECOVERY_COOLDOWN_MS 5000

// DPI / LOD
#define DPI_DEFAULT 12   // hundreds: 1200
#define DPI_MIN 1
#define DPI_MAX 120
#define LOD_DEFAULT 2
#define LOD_MIN 2
#define LOD_MAX 3

#define CHORD_DPI (PHYS_BIT(PHYS_M1) | PHYS_BIT(PHYS_M2) | PHYS_BIT(PHYS_DPI) | PHYS_BIT(PHYS_M4))
#define CHORD_LOD (PHYS_BIT(PHYS_M1) | PHYS_BIT(PHYS_M2) | PHYS_BIT(PHYS_DPI) | PHYS_BIT(PHYS_M5))

#define SETTINGS_KEY "/kv/diymouse_cfg"
#define SETTINGS_MAGIC 0x44594D53u /* 'DYMS' */
#define SETTINGS_VERSION 1

SPISettings spisettings(2000000, MSBFIRST, SPI_MODE3);

struct MotionBurstData {
    uint8_t motion;
    uint8_t observation;
    int16_t x;
    int16_t y;
    uint8_t squal;
};

struct SettingsRecord {
    uint32_t magic;
    uint16_t version;
    uint8_t dpi;
    uint8_t lod;
    uint32_t crc;
};

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
    }
    return ~crc;
}

static uint32_t settings_payload_crc(const SettingsRecord &rec)
{
    uint8_t payload[4];
    payload[0] = (uint8_t)(rec.magic);
    payload[1] = (uint8_t)(rec.magic >> 8);
    payload[2] = (uint8_t)(rec.magic >> 16);
    payload[3] = (uint8_t)(rec.magic >> 24);
    uint32_t crc = crc32_update(0, payload, 4);
    uint8_t rest[4] = {
        (uint8_t)(rec.version),
        (uint8_t)(rec.version >> 8),
        rec.dpi,
        rec.lod
    };
    return crc32_update(crc, rest, 4);
}

// Forward decls
void spi_write(byte addr, byte data);
byte spi_read(byte addr);
MotionBurstData spi_read_motion_burst(bool do_update_wheel);
void pmw3360_boot();
void pmw3360_config();
void srom_upload();
void update_wheel();
void update_buttons();
bool sensor_srom_observation_ok();
void enter_limp_mode(const char *reason);
void try_sensor_recovery();
void settings_load();
void settings_mark_dirty();
void settings_flush_if_due();
bool settings_save();
int16_t sat_add_i16(int16_t a, int16_t b);
int8_t sat_add_i8(int8_t a, int8_t b);
uint8_t pack_hid_buttons(uint8_t phys);

bool mouse_inited = false;
bool sensor_degraded = false; // no real SROM, but tracking enabled
bool sensor_limp = false;
uint8_t sensor_fail_streak = 0;
uint32_t last_sensor_recovery_ms = 0;

int dpi = DPI_DEFAULT;
int lod = LOD_DEFAULT;
bool settings_dirty = false;
uint32_t settings_dirty_ms = 0;

uint32_t pins_state = 0;
uint8_t buttons = 0;
uint8_t which_m3 = 0; // 0 = physical M3 owns middle; 1 = DPI owns middle
uint8_t buttons_latch[PHYS_COUNT] = {0};

volatile uint32_t *gpio_oe_set = (uint32_t *)0xd0000024;
volatile uint32_t *gpio_oe_clr = (uint32_t *)0xd0000028;
volatile uint32_t *gpio_in = (uint32_t *)0xd0000004;

uint8_t wheel_state_a = 0;
uint8_t wheel_state_b = 0;
uint8_t wheel_state_output = 0;
int8_t wheel_progress = 0;

int16_t pending_x = 0;
int16_t pending_y = 0;
int8_t pending_wheel = 0;
uint8_t pending_buttons = 0;
uint32_t usb_fail_count = 0;
uint32_t usb_recover_count = 0;
bool usb_had_pending_fail = false;

void setup_buttons()
{
    pinMode(BUTTONS_OFF, INPUT);
    pinMode(BUTTONS_ON, INPUT);
    pinMode(BUTTON_M1, INPUT_PULLUP);
    pinMode(BUTTON_M2, INPUT_PULLUP);
    pinMode(BUTTON_M3, INPUT_PULLUP);
    pinMode(BUTTON_M4, INPUT_PULLUP);
    pinMode(BUTTON_M5, INPUT_PULLUP);
    pinMode(BUTTON_DPI, INPUT_PULLUP);

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    pinMode(ENCODER_COM, OUTPUT);

    digitalWrite(ENCODER_COM, LOW);
}

void setup()
{
    delay(BOOT_SERIAL_DELAY_MS);

    SPI.begin();
    pinMode(PIN_NCS, OUTPUT);

    digitalWrite(PIN_NCS, HIGH);
    delayMicroseconds(T_NCS_SCLK_US);
    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(T_NCS_SCLK_US);
    digitalWrite(PIN_NCS, HIGH);
    delayMicroseconds(T_NCS_SCLK_US);

    settings_load();
    pmw3360_boot();
    delay(10);
    pmw3360_config();
    setup_buttons();
}

void update_buttons()
{
    // Tri-state matrix: OE toggling is much faster than pinMode()
    *gpio_oe_clr = 1u << BUTTONS_OFF;
    *gpio_oe_set = 1u << BUTTONS_ON;

    digitalWrite(BUTTONS_ON, LOW);
    delayMicroseconds(1);
    uint32_t pins_on = ~*gpio_in;

    *gpio_oe_set = 1u << BUTTONS_OFF;
    *gpio_oe_clr = 1u << BUTTONS_ON;

    digitalWrite(BUTTONS_OFF, LOW);
    delayMicroseconds(1);
    uint32_t pins_off = ~*gpio_in;

    uint32_t pins_update = pins_on ^ pins_off;
    pins_state = (pins_state & (~pins_update)) | (pins_on & pins_update);

    uint8_t next_buttons =
          ((!!(pins_state & (1u << BUTTON_M1))) << PHYS_M1)
        | ((!!(pins_state & (1u << BUTTON_M2))) << PHYS_M2)
        | ((!!(pins_state & (1u << BUTTON_M3))) << PHYS_M3)
        | ((!!(pins_state & (1u << BUTTON_DPI))) << PHYS_DPI)
        | ((!!(pins_state & (1u << BUTTON_M4))) << PHYS_M4)
        | ((!!(pins_state & (1u << BUTTON_M5))) << PHYS_M5);

    uint8_t ok_mask = 0;
    for (uint8_t i = 0; i < PHYS_COUNT; i++)
        if (buttons_latch[i] == 0)
            ok_mask |= PHYS_BIT(i);

    next_buttons = (next_buttons & ok_mask) | (buttons & ~ok_mask);

    for (uint8_t i = 0; i < PHYS_COUNT; i++) {
        if (((next_buttons ^ buttons) >> i) & 1)
            buttons_latch[i] = BUTTON_LATCH_LOOPS;
        else if (buttons_latch[i])
            buttons_latch[i] -= 1;
    }

    if ((next_buttons & PHYS_BIT(PHYS_M3)) != (buttons & PHYS_BIT(PHYS_M3)))
        which_m3 = 0;
    if ((next_buttons & PHYS_BIT(PHYS_DPI)) != (buttons & PHYS_BIT(PHYS_DPI)))
        which_m3 = 1;

    buttons = next_buttons;
}

void update_wheel()
{
    uint8_t wheel_new_a = digitalRead(ENCODER_A);
    uint8_t wheel_new_b = digitalRead(ENCODER_B);

    if (wheel_new_a != wheel_state_a || wheel_new_b != wheel_state_b) {
        if (wheel_new_a == wheel_new_b && wheel_state_output != wheel_new_a) {
            // scrolling up: B changes first; scrolling down: A changes first
            if (wheel_new_b == wheel_state_b)
                wheel_progress += 1;
            // skip illegal 00<->11 jumps
            else if (wheel_new_a == wheel_state_a)
                wheel_progress -= 1;

            wheel_state_output = wheel_new_a;
        }

        wheel_state_a = wheel_new_a;
        wheel_state_b = wheel_new_b;
    }
}

uint8_t pack_hid_buttons(uint8_t phys)
{
    uint8_t hid = 0;
    bool m3_shared = false;
    bool dpi_shared = false;

    for (uint8_t i = 0; i < PHYS_COUNT; i++) {
        if (!(phys & PHYS_BIT(i)))
            continue;

        uint8_t map = BUTTON_MAP[i];
        if (map == HID_NONE)
            continue;
        if (map == HID_MIDDLE_SHARED) {
            if (i == PHYS_M3)
                m3_shared = true;
            else if (i == PHYS_DPI)
                dpi_shared = true;
            else
                hid |= (1u << HID_MIDDLE);
            continue;
        }
        if (map <= HID_FORWARD)
            hid |= (1u << map);
    }

    // M3/DPI share middle-click; last edge wins when both are held
    if (m3_shared && dpi_shared) {
        if (which_m3 ? dpi_shared : m3_shared)
            hid |= (1u << HID_MIDDLE);
    } else if (m3_shared || dpi_shared) {
        hid |= (1u << HID_MIDDLE);
    }

    return hid;
}

static int dpi_step_up(int value)
{
    if (value < 16)
        return 1;
    if (value < 32)
        return 2;
    if (value < 64)
        return 4;
    return 8;
}

static int dpi_step_down(int value)
{
    if (value <= 16)
        return 1;
    if (value <= 32)
        return 2;
    if (value <= 64)
        return 4;
    return 8;
}

void check_config_inputs()
{
    const bool dpi_chord = (buttons & CHORD_DPI) == CHORD_DPI;
    const bool lod_chord = (buttons & CHORD_LOD) == CHORD_LOD;

    if (dpi_chord && !lod_chord) {
        int olddpi = dpi;
        if (wheel_progress < 0)
            dpi += dpi_step_up(dpi);
        else if (wheel_progress > 0)
            dpi -= dpi_step_down(dpi);

        if (dpi > DPI_MAX)
            dpi = DPI_MAX;
        else if (dpi < DPI_MIN)
            dpi = DPI_MIN;

        if (olddpi != dpi) {
            spi_write(REG_CONFIG1, (byte)(dpi - 1));
            printf("new DPI: %d\n", dpi * 100);
            settings_mark_dirty();
        }
        wheel_progress = 0;
    } else if (lod_chord && !dpi_chord) {
        int oldlod = lod;
        if (wheel_progress < 0)
            lod += 1;
        else if (wheel_progress > 0)
            lod -= 1;

        if (lod > LOD_MAX)
            lod = LOD_MAX;
        else if (lod < LOD_MIN)
            lod = LOD_MIN;

        if (oldlod != lod) {
            spi_write(REG_LIFT_CONFIG, (byte)lod);
            printf("new LOD: %d mm\n", lod);
            settings_mark_dirty();
        }
        wheel_progress = 0;
    }
}

int16_t sat_add_i16(int16_t a, int16_t b)
{
    int32_t sum = (int32_t)a + (int32_t)b;
    if (sum > 32767)
        return 32767;
    if (sum < -32767)
        return -32767;
    return (int16_t)sum;
}

int8_t sat_add_i8(int8_t a, int8_t b)
{
    int16_t sum = (int16_t)a + (int16_t)b;
    if (sum > 127)
        return 127;
    if (sum < -127)
        return -127;
    return (int8_t)sum;
}

void settings_mark_dirty()
{
    settings_dirty = true;
    settings_dirty_ms = millis();
}

void settings_load()
{
    SettingsRecord rec;
    size_t actual = 0;
    int res = kv_get(SETTINGS_KEY, &rec, sizeof(rec), &actual);
    if (res != MBED_SUCCESS || actual != sizeof(rec)) {
        printf("settings: using defaults (no saved record)\n");
        dpi = DPI_DEFAULT;
        lod = LOD_DEFAULT;
        return;
    }

    if (rec.magic != SETTINGS_MAGIC || rec.version != SETTINGS_VERSION ||
        rec.crc != settings_payload_crc(rec) ||
        rec.dpi < DPI_MIN || rec.dpi > DPI_MAX ||
        rec.lod < LOD_MIN || rec.lod > LOD_MAX) {
        printf("settings: invalid record, using defaults\n");
        dpi = DPI_DEFAULT;
        lod = LOD_DEFAULT;
        return;
    }

    dpi = rec.dpi;
    lod = rec.lod;
    printf("settings: loaded DPI=%d LOD=%d\n", dpi * 100, lod);
}

bool settings_save()
{
    SettingsRecord rec;
    rec.magic = SETTINGS_MAGIC;
    rec.version = SETTINGS_VERSION;
    rec.dpi = (uint8_t)dpi;
    rec.lod = (uint8_t)lod;
    rec.crc = settings_payload_crc(rec);

    int res = kv_set(SETTINGS_KEY, &rec, sizeof(rec), 0);
    if (res != MBED_SUCCESS) {
        printf("settings: kv_set failed (%d)\n", res);
        return false;
    }
    printf("settings: saved DPI=%d LOD=%d\n", dpi * 100, lod);
    return true;
}

void settings_flush_if_due()
{
    if (!settings_dirty)
        return;
    if ((millis() - settings_dirty_ms) < SETTINGS_FLUSH_MS)
        return;

    if (settings_save())
        settings_dirty = false;
    else
        settings_dirty_ms = millis(); // retry later
}

bool sensor_srom_observation_ok()
{
    spi_write(REG_OBSERVATION, 0x00);
    delay(10);
    byte obs = spi_read(REG_OBSERVATION);
    printf("observation: 0x%02X\n", obs);
    return (obs & OBS_SROM_RUNNING) != 0;
}

void enter_limp_mode(const char *reason)
{
    if (!sensor_limp)
        printf("sensor limp: %s\n", reason);
    sensor_limp = true;
    mouse_inited = false;
}

void try_sensor_recovery()
{
    uint32_t now = millis();
    if ((now - last_sensor_recovery_ms) < SENSOR_RECOVERY_COOLDOWN_MS)
        return;
    if (buttons != 0 || wheel_progress != 0 || pending_x || pending_y || pending_wheel)
        return;

    last_sensor_recovery_ms = now;
    printf("sensor: attempting recovery\n");
    pmw3360_boot();
    delay(10);
    if (mouse_inited || sensor_degraded) {
        pmw3360_config();
        sensor_limp = false;
        sensor_fail_streak = 0;
        printf("sensor: recovery ok\n");
    }
}

void loop()
{
    update_wheel();
    delayMicroseconds(WHEEL_POLL_US);
    update_wheel();

    int16_t x = 0;
    int16_t y = 0;

    MotionBurstData data = spi_read_motion_burst(true);

    // Invalid SPI responses (all 0xFF) indicate a bus/sensor fault; zero SQUAL alone is normal when lifted
    bool burst_bus_fault = (data.motion == 0xFF && data.observation == 0xFF && data.squal == 0xFF);
    if (burst_bus_fault) {
        if (sensor_fail_streak < 255)
            sensor_fail_streak++;
        if (sensor_fail_streak >= SENSOR_FAIL_STREAK_LIMIT)
            enter_limp_mode("invalid motion burst");
    } else if (sensor_fail_streak) {
        sensor_fail_streak--;
    }

    if ((mouse_inited || sensor_degraded) && !sensor_limp && data.motion) {
        x = data.x;
        y = data.y;
    }

    update_wheel();
    update_buttons();
    check_config_inputs();
    settings_flush_if_due();

    if (sensor_limp)
        try_sensor_recovery();

    int8_t wheel = wheel_progress;
    wheel_progress = 0;

    pending_x = sat_add_i16(pending_x, x);
    pending_y = sat_add_i16(pending_y, y);
    pending_wheel = sat_add_i8(pending_wheel, wheel);
    pending_buttons = pack_hid_buttons(buttons);

    // mouse.update blocks until the ~1 ms HID interval; return false means drop/coalesce
    bool ok = mouse.update(pending_x, pending_y, pending_buttons, pending_wheel);
    if (ok) {
        if (usb_had_pending_fail) {
            usb_recover_count++;
            usb_had_pending_fail = false;
            printf("usb: recovered (fails=%lu recovers=%lu)\n",
                   (unsigned long)usb_fail_count, (unsigned long)usb_recover_count);
        }
        pending_x = 0;
        pending_y = 0;
        pending_wheel = 0;
    } else {
        usb_fail_count++;
        usb_had_pending_fail = true;
        // keep pending_* for next loop; newest buttons already stored
    }
}

void pmw3360_boot()
{
    mouse_inited = false;
    sensor_degraded = false;

    spi_write(REG_POWER_UP_RESET, 0x5A);
    delay(50);

    spi_read(REG_MOTION);
    spi_read(REG_DELTA_X_L);
    spi_read(REG_DELTA_X_H);
    spi_read(REG_DELTA_Y_L);
    spi_read(REG_DELTA_Y_H);

    byte product_id = spi_read(REG_PRODUCT_ID);
    byte inverse_product_id = spi_read(REG_INVERSE_PRODUCT_ID);
    printf("product id: %d (inverse: %d)\n", product_id, inverse_product_id);
    if (product_id != PMW3360_PRODUCT_ID || inverse_product_id != PMW3360_INVERSE_PRODUCT_ID) {
        printf("warning: unexpected PMW3360 product id; check SPI wiring\n");
        enter_limp_mode("bad product id");
        return;
    }

#ifdef HAVE_REAL_SROM
    srom_upload();
#else
    printf("warning: no real SROM header; tracking may glitch (spinouts/teleports)\n");
    printf("place srom_3360_0x05.h next to this sketch to enable SROM upload\n");
    sensor_degraded = true;
    mouse_inited = false;
    sensor_limp = false;
#endif
}

void pmw3360_config()
{
    spi_write(REG_CONFIG1, (byte)(dpi - 1));
    spi_write(REG_LIFT_CONFIG, (byte)lod);
}

void spi_begin()
{
    SPI.beginTransaction(spisettings);
}

void spi_end()
{
    SPI.endTransaction();
}

void spi_write(byte addr, byte data)
{
    spi_begin();

    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(T_NCS_SCLK_US);

    SPI.transfer(addr | 0x80);
    SPI.transfer(data);

    delayMicroseconds(T_SCLK_NCS_WRITE_US);
    digitalWrite(PIN_NCS, HIGH);

    spi_end();

    delayMicroseconds(T_SWW_SWR_US);
}

byte spi_read(byte addr)
{
    spi_begin();

    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(T_NCS_SCLK_US);

    SPI.transfer(addr & 0x7F);

    delayMicroseconds(T_SRAD_US);

    byte ret = SPI.transfer(0);

    delayMicroseconds(T_NCS_SCLK_US);
    digitalWrite(PIN_NCS, HIGH);

    spi_end();

    delayMicroseconds(T_SRW_SRR_US);

    return ret;
}

MotionBurstData spi_read_motion_burst(bool do_update_wheel)
{
    MotionBurstData ret = {0};

    // ~260us; sample wheel midway
    spi_write(REG_MOTION_BURST, 0x00);
    if (do_update_wheel)
        update_wheel();

    spi_begin();

    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(T_NCS_SCLK_US);

    SPI.transfer(REG_MOTION_BURST);

    delayMicroseconds(T_SRAD_MOTBR_US);

    ret.motion = SPI.transfer(0);
    ret.observation = SPI.transfer(0);
    ret.x = SPI.transfer(0);
    ret.x |= ((uint16_t)SPI.transfer(0)) << 8;
    ret.y = SPI.transfer(0);
    ret.y |= ((uint16_t)SPI.transfer(0)) << 8;
    ret.squal = SPI.transfer(0);
    digitalWrite(PIN_NCS, HIGH);
    delayMicroseconds(T_BEXIT_US);

    spi_end();

    return ret;
}

void srom_upload()
{
    spi_write(REG_CONFIG2, 0x00);
    spi_write(REG_SROM_ENABLE, 0x1D);
    delay(10);

    spi_write(REG_SROM_ENABLE, 0x18);

    spi_begin();

    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(T_NCS_SCLK_US);

    SPI.transfer(REG_SROM_LOAD_BURST | 0x80);
    delayMicroseconds(T_SROM_BYTE_US);

    for (size_t i = 0; i < SROM_LENGTH; i += 1) {
        SPI.transfer(srom[i]);
        delayMicroseconds(T_SROM_BYTE_US);
    }

    digitalWrite(PIN_NCS, HIGH);
    delayMicroseconds(T_NCS_SCLK_US);

    spi_end();

    delayMicroseconds(200);

    byte id = spi_read(REG_SROM_ID);
    bool id_ok = (id != 0 && id != 0xFF);
    bool obs_ok = sensor_srom_observation_ok();
    mouse_inited = id_ok && obs_ok;
    sensor_degraded = false;
    sensor_limp = !mouse_inited;

    printf("srom id: %d (%s)\n", id, mouse_inited ? "ok" : "failed");
    if (!mouse_inited)
        printf("sensor limp: SROM validation failed\n");

    spi_write(REG_CONFIG2, 0x00);
}
