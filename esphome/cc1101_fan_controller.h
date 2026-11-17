#pragma once
#include "esphome.h"
#include <CC1101_ESP_Arduino.h>

// ============================================================
// Fan RF Protocol Constants
// 29-bit OOK protocol: 20-bit device address + 9-bit command
// ============================================================
static const uint16_t FAN_CMD_SPEED1 = 0x1E8;  // 111101000
static const uint16_t FAN_CMD_SPEED2 = 0x1C8;  // 111001000
static const uint16_t FAN_CMD_SPEED3 = 0x1A9;  // 110101001
static const uint16_t FAN_CMD_SPEED4 = 0x189;  // 110001001
static const uint16_t FAN_CMD_SPEED5 = 0x16A;  // 101101010
static const uint16_t FAN_CMD_SPEED6 = 0x14A;  // 101001010
static const uint16_t FAN_CMD_TOGGLE = 0x191;  // 110010001
static const uint16_t FAN_CMD_LIGHT  = 0x0B5;  // 010110101
static const uint16_t FAN_CMD_INVERT = 0x12B;  // 100101011

// OOK timings (microseconds), estimated from the recorded 0x093 frames.
// Receiver captures show asymmetric mark/space intervals, so keep each
// symbol's mark and space duration separate rather than using complements.
static const int OOK_MARK_0_US  = 275;
static const int OOK_SPACE_0_US = 725;
static const int OOK_MARK_1_US  = 750;
static const int OOK_SPACE_1_US = 250;
static const int OOK_GAP_US     = 7580;
static const int OOK_TX_REPS    = 3;
static const int OOK_MSG_BITS = 29;  // 20-bit address + 9-bit command
// Captures show 33 symbols: 29 address/command bits, 3 zero trailer bits,
// then one trailing parity symbol. The captured 0x0B3 frame has even parity.

// ============================================================
// Hardware pin defaults — must match your wiring
// ============================================================
#ifndef CC1101_CS_PIN
#define CC1101_CS_PIN    5
#endif
#ifndef CC1101_GDO0_PIN
#define CC1101_GDO0_PIN  25
#endif
#ifndef CC1101_GDO2_PIN
#define CC1101_GDO2_PIN  26
#endif
#ifndef CC1101_SCK_PIN
#define CC1101_SCK_PIN   18
#endif
#ifndef CC1101_MISO_PIN
#define CC1101_MISO_PIN  19
#endif
#ifndef CC1101_MOSI_PIN
#define CC1101_MOSI_PIN  23
#endif

// Forward declaration for ISR
static void IRAM_ATTR rf_record_isr();

class CC1101FanController {
    CC1101 radio_;
    int gdo0_;
    int gdo2_;
    bool ready_ = false;
    portMUX_TYPE rf_mux_ = portMUX_INITIALIZER_UNLOCKED;

public:
    static constexpr int MAX_CHANGES = 512;
    static volatile int rec_timings_[MAX_CHANGES];
    static volatile int rec_levels_[MAX_CHANGES];
    static volatile int rec_initial_level_;
    static volatile int rec_count_;
    static volatile long rec_last_us_;
    static volatile bool rec_signal_detected_;
    static volatile bool rec_active_;

    CC1101FanController(int sck, int miso, int mosi, int cs, int gdo0, int gdo2)
        : radio_(sck, miso, mosi, cs, gdo0, gdo2), gdo0_(gdo0), gdo2_(gdo2) {}

    bool begin() {
        radio_.init();
        radio_.setMHZ(433.92);
        radio_.setTXPwr(TX_PLUS_10_DBM);
        radio_.setDataRate(2400);
        radio_.setRxBW(RX_BW_162_KHZ);
        radio_.setModulation(ASK_OOK);

        uint8_t partnum = radio_.getPartnum();
        uint8_t version = radio_.getVersion();

        if (version == 0x00 || version == 0xFF) {
            ESP_LOGE("cc1101", "CC1101 not detected (ver=0x%02X). Check wiring.", version);
            return false;
        }

        ready_ = true;
        ESP_LOGI("cc1101", "CC1101 OK  part=0x%02X  ver=0x%02X  433.92 MHz ASK/OOK", partnum, version);
        return true;
    }

    void send_command(uint32_t address, uint16_t command, bool odd_parity = false) {
        if (!ready_) {
            ESP_LOGW("cc1101", "Not initialised — skipping TX");
            return;
        }

        uint32_t msg = ((address & 0xFFFFF) << 9) | (command & 0x1FF);

        // The captures contain three zero symbols after the 29-bit payload.
        int ones = __builtin_popcount(msg);
        bool parity = odd_parity ? !(ones & 1) : (ones & 1);
        char tx_bits[34];
        for (int bit = 28; bit >= 0; bit--)
            tx_bits[28 - bit] = (msg & (1UL << bit)) ? '1' : '0';
        tx_bits[29] = '0';
        tx_bits[30] = '0';
        tx_bits[31] = '0';
        tx_bits[32] = parity ? '1' : '0';
        tx_bits[33] = '\0';
        ESP_LOGI("cc1101_tx", "addr=0x%05lX cmd=0x%03X frame33=%s parity=%s reps=%d gap=%dus",
                 (unsigned long)address, command, tx_bits, odd_parity ? "odd" : "even", OOK_TX_REPS, OOK_GAP_US);
        ESP_LOGI("cc1101_tx", "TX timing: bit1 mark/space=%d/%dus; bit0=%d/%dus",
                 OOK_MARK_1_US, OOK_SPACE_1_US, OOK_MARK_0_US, OOK_SPACE_0_US);

        radio_.setTx();
        pinMode(gdo0_, OUTPUT);
        delayMicroseconds(500);

        // Preamble: alternating pulses to wake the receiver's AGC
        for (int i = 0; i < 12; i++) {
            digitalWrite(gdo0_, HIGH);
            delayMicroseconds(OOK_MARK_1_US);
            digitalWrite(gdo0_, LOW);
            delayMicroseconds(OOK_SPACE_1_US);
        }
        delayMicroseconds(OOK_GAP_US);

        for (int rep = 0; rep < OOK_TX_REPS; rep++) {
            portENTER_CRITICAL(&rf_mux_);

            for (int b = OOK_MSG_BITS - 1; b >= 0; b--) {
                bool one = msg & (1UL << b);
                digitalWrite(gdo0_, HIGH);
                delayMicroseconds(one ? OOK_MARK_1_US : OOK_MARK_0_US);
                digitalWrite(gdo0_, LOW);
                delayMicroseconds(one ? OOK_SPACE_1_US : OOK_SPACE_0_US);
            }
            // Three captured zero trailer symbols follow the payload.
            for (int trailer_bit = 0; trailer_bit < 3; trailer_bit++) {
                digitalWrite(gdo0_, HIGH);
                delayMicroseconds(OOK_MARK_0_US);
                digitalWrite(gdo0_, LOW);
                delayMicroseconds(OOK_SPACE_0_US);
            }
            // Final parity symbol; default is even parity based on captured 0x0B3.
            digitalWrite(gdo0_, HIGH);
            delayMicroseconds(parity ? OOK_MARK_1_US : OOK_MARK_0_US);
            digitalWrite(gdo0_, LOW);

            portEXIT_CRITICAL(&rf_mux_);

            delayMicroseconds(OOK_GAP_US);
        }

        digitalWrite(gdo0_, LOW);
        radio_.setIdle();
        ESP_LOGD("cc1101", "TX  addr=0x%05X  cmd=0x%03X", address, command);
    }

    void send_cyclic_command(uint32_t address, uint16_t command_prefix, int &next_nibble, uint8_t step, int excluded_nibble = -1) {
        uint8_t nibble = next_nibble & 0x0F;
        if (excluded_nibble >= 0 && nibble == (excluded_nibble & 0x0F))
            nibble = (nibble + step) & 0x0F;
        const uint16_t command = (command_prefix & 0x1F0) | nibble;
        ESP_LOGI("cc1101_seq", "addr=0x%05lX prefix=0x%03X nibble=0x%X step=+%u",
                 (unsigned long)address, command_prefix, nibble, step);
        if (!ready_) {
            send_command(address, command);
            return;
        }
        send_command(address, command);
        uint8_t next = (nibble + step) & 0x0F;
        if (excluded_nibble >= 0 && next == (excluded_nibble & 0x0F))
            next = (next + step) & 0x0F;
        next_nibble = next;
    }
    void record_signal() {
        if (!ready_) {
            ESP_LOGW("cc1101", "Not initialised — skipping RX record");
            return;
        }

        ESP_LOGI("cc1101", "=== RECORDING: Put CC1101 in RX, waiting for signal (10s timeout) ===");

        rec_signal_detected_ = false;
        rec_count_ = 0;

        radio_.setRx();
        pinMode(gdo2_, INPUT);
        rec_initial_level_ = digitalRead(gdo2_);
        rec_last_us_ = micros();
        rec_active_ = true;
        attachInterrupt(digitalPinToInterrupt(gdo2_), rf_record_isr, CHANGE);

        unsigned long start = millis();
        while (!rec_signal_detected_ && (millis() - start < 10000)) {
            delay(1);
        }

        if (rec_signal_detected_) {
            delay(300);
        }

        rec_active_ = false;
        detachInterrupt(digitalPinToInterrupt(gdo2_));
        radio_.setIdle();

        int count = rec_count_;

        if (count < 20) {
            ESP_LOGW("cc1101", "Too few transitions (%d). No signal captured.", count);
            return;
        }

        ESP_LOGI("cc1101", "Captured %d transitions. Raw timings:", count);

        // Log timings in chunks so they fit in log buffers
        String timings_str = "[";
        for (int i = 0; i < count; i++) {
            if (i > 0) timings_str += ", ";
            timings_str += String(rec_timings_[i]);
            if (timings_str.length() > 400 || i == count - 1) {
                if (i < count - 1) timings_str += ",";
                ESP_LOGI("cc1101", "  %s", timings_str.c_str());
                timings_str = "";
            }
        }
        ESP_LOGI("cc1101", "]");

        log_timing_analysis(count);
        decode_signal(count);
    }

private:
    void log_timing_analysis(int count) {
        int high_bins[13] = {};
        int low_bins[13] = {};
        int level = rec_initial_level_;
        String edge_line;

        ESP_LOGI("rf_timing", "Intervals are index:level/duration; level is RX output during that interval.");
        for (int i = 0; i < count; i++) {
            const int duration = rec_timings_[i];
            if (duration >= 5000) {
                ESP_LOGI("rf_timing", "GAP index=%d duration=%dus", i, duration);
            } else {
                int bin = duration / 100;
                if (bin > 11) bin = 12;
                if (level) high_bins[bin]++;
                else low_bins[bin]++;
                edge_line += String(i) + ":" + String(level) + "/" + String(duration) + " ";
                if (edge_line.length() >= 220) {
                    ESP_LOGI("rf_timing", "%s", edge_line.c_str());
                    edge_line = "";
                }
            }
            level = rec_levels_[i];
        }
        if (edge_line.length()) ESP_LOGI("rf_timing", "%s", edge_line.c_str());

        for (int bin = 0; bin < 13; bin++) {
            const int low = bin * 100;
            if (bin == 12) {
                if (high_bins[bin]) ESP_LOGI("rf_timing", "RX level 1: >=1200 us=%d samples", high_bins[bin]);
                if (low_bins[bin]) ESP_LOGI("rf_timing", "RX level 0: >=1200 us=%d samples", low_bins[bin]);
            } else {
                const int high = low + 99;
                if (high_bins[bin]) ESP_LOGI("rf_timing", "RX level 1: %d-%d us=%d samples", low, high, high_bins[bin]);
                if (low_bins[bin]) ESP_LOGI("rf_timing", "RX level 0: %d-%d us=%d samples", low, high, low_bins[bin]);
            }
        }
    }

    void decode_signal(int count) {
        int frames = 0;
        uint32_t previous_in_capture = 0;
        bool have_previous_in_capture = false;
        uint32_t changed_in_capture = 0;
        uint32_t first_message = 0;
        static uint32_t previous_capture_message = 0;
        static bool have_previous_capture = false;

        // Use the next long gap as the end of a frame. Do not assume frame
        // length or that the trailer is a parity bit: captures contain more
        // symbols than the 29 address/command bits.
        for (int gap = 0; gap < count; gap++) {
            if (rec_timings_[gap] < 5000) continue;
            const int first_mark = gap + 1;
            int end = first_mark;
            while (end < count && rec_timings_[end] < 5000) end++;

            const int intervals = end - first_mark;
            const int symbols = (intervals + 1) / 2;
            if (symbols < OOK_MSG_BITS || symbols > 60) continue;

            char bits[61];
            uint32_t message29 = 0;
            bool valid = true;
            for (int bit = 0; bit < symbols; bit++) {
                const int mark_index = first_mark + bit * 2;
                if (mark_index >= end) { valid = false; break; }
                const bool one = rec_timings_[mark_index] > 500;
                bits[bit] = one ? '1' : '0';
                if (bit < OOK_MSG_BITS) message29 = (message29 << 1) | (one ? 1U : 0U);
            }
            if (!valid) continue;
            bits[symbols] = '\0';

            const uint32_t address = (message29 >> 9) & 0xFFFFF;
            const uint16_t command = message29 & 0x1FF;
            const char *suffix = symbols > OOK_MSG_BITS ? bits + OOK_MSG_BITS : "(none)";

            frames++;
            if (frames == 1) first_message = message29;
            if (have_previous_in_capture) changed_in_capture |= message29 ^ previous_in_capture;
            previous_in_capture = message29;
            have_previous_in_capture = true;

            ESP_LOGI("rf_decode", "Frame %d: symbols=%d addr=0x%05lX cmd=0x%03X suffix-after-29=%s bits=%s",
                     frames, symbols, (unsigned long)address, (unsigned int)command, suffix, bits);
        }

        if (frames == 0) {
            ESP_LOGW("rf_decode", "No frame with at least 29 symbols found.");
            return;
        }
        ESP_LOGI("rf_decode", "Capture summary: %d frames; changed bits in first 29 symbols=0x%08lX",
                 frames, (unsigned long)changed_in_capture);
        if (have_previous_capture) {
            const uint32_t delta = first_message ^ previous_capture_message;
            ESP_LOGI("rf_decode", "First 29 symbols vs previous Record press: XOR=0x%08lX",
                     (unsigned long)delta);
            ESP_LOGI("rf_decode", "Changed address bits=0x%05lX; changed command bits=0x%03lX",
                     (unsigned long)((delta >> 9) & 0xFFFFF),
                     (unsigned long)(delta & 0x1FF));
        } else {
            ESP_LOGI("rf_decode", "Record again after another remote-button press to compare.");
        }
        previous_capture_message = first_message;
        have_previous_capture = true;
    }
};

// Static member definitions
volatile int CC1101FanController::rec_timings_[CC1101FanController::MAX_CHANGES];
volatile int CC1101FanController::rec_levels_[CC1101FanController::MAX_CHANGES];
volatile int CC1101FanController::rec_initial_level_ = 0;
volatile int CC1101FanController::rec_count_ = 0;
volatile long CC1101FanController::rec_last_us_ = 0;
volatile bool CC1101FanController::rec_signal_detected_ = false;
volatile bool CC1101FanController::rec_active_ = false;

static void IRAM_ATTR rf_record_isr() {
    if (!CC1101FanController::rec_active_ ||
        CC1101FanController::rec_count_ >= CC1101FanController::MAX_CHANGES)
        return;
    long now = micros();
    int index = CC1101FanController::rec_count_;
    CC1101FanController::rec_timings_[index] =
        (int)(now - CC1101FanController::rec_last_us_);
    CC1101FanController::rec_levels_[index] = digitalRead(CC1101_GDO2_PIN);
    CC1101FanController::rec_count_ = index + 1;
    CC1101FanController::rec_last_us_ = now;
    CC1101FanController::rec_signal_detected_ = true;
}

static CC1101FanController fan_ctrl(
    CC1101_SCK_PIN,
    CC1101_MISO_PIN,
    CC1101_MOSI_PIN,
    CC1101_CS_PIN,
    CC1101_GDO0_PIN,
    CC1101_GDO2_PIN
);
