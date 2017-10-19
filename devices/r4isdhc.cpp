#include <cstring>
#include <algorithm>

#include "device.h"

namespace flashcart_core {
using ntrcard::sendCommand;
using ntrcard::BlowfishKey;
using platform::logMessage;
using platform::ioDelay;
using platform::showProgress;

namespace {
    union CmdBuf4 {
            uint32_t u32;
            uint8_t u8[4];
    };
    static_assert(sizeof(CmdBuf4) == 4, "Wrong union size");

    constexpr uint64_t norCmd(const uint8_t outlen, const uint8_t inlen, const uint8_t cmd, const uint32_t addr, const uint8_t d1 = 0, const uint8_t d2 = 0) {
        // DD DD AA AA AA CC XY 99
        return 0x99ull | ((outlen & 0xFull) << 12) | ((inlen & 0xFull) << 8) |
            (cmd << 16) |
            ((addr & 0xFF0000ull) << 8) | ((addr & 0xFF00ull) << 24) | ((addr & 0xFFull) << 40) |
            (static_cast<uint64_t>(d1) << 48) | (static_cast<uint64_t>(d2) << 56);
    }
    static_assert(norCmd(2, 5, 0x3B, 0xABCDEF, 0x12, 0x34) == 0x3412EFCDAB3B2599, "norCmd result is wrong");

    // i have no idea what else to name this...
    constexpr uint64_t norRaw(const uint8_t d1, const uint8_t d2, const uint8_t fpgaParam = 0) {
        // 00 00 00 00 D2 D1 XY 99
        return 0x99ull | (static_cast<uint64_t>(fpgaParam) << 8) |
            (static_cast<uint64_t>(d1) << 16) | (static_cast<uint64_t>(d2) << 24);
    }
    static_assert(norRaw(0x34, 0x56, 0x12) == 0x56341299, "norRaw result is wrong");

    uint32_t norRead(const uint32_t address) {
        CmdBuf4 buf;
        sendCommand(norCmd(2, 5, 0x3B, address), 4, buf.u8, 0x180000);
        logMessage(LOG_DEBUG, "R4ISDHC: NOR read at %X returned %X", address, buf.u32);
        return buf.u32;
    }

    void norWriteEnable() {
        sendCommand(norCmd(0, 1, 6, 0), 4, nullptr, 0x180000);
        ioDelay(0x60000);
    }

    uint32_t nor68Cmd(const char a1, const char a2, const char a3, uint32_t a4, uint16_t a5) {
        CmdBuf4 buf;
        uint8_t cmd[8];
        cmd[0] = 0x68;
        cmd[1] = (a2 & 0xF) | (a1 << 4);
        cmd[2] = a3;
        cmd[3] = a4 >> 16;
        cmd[4] = a4 >> 8;
        cmd[5] = a4;
        cmd[6] = a5 >> 8;
        cmd[7] = a5;
        sendCommand(cmd, 4, buf.u8, 0x180000);
        logMessage(LOG_DEBUG, "R4ISDHC: NOR 68 at %X returned %X", a4, buf.u32);
        return buf.u32;
    }

    uint32_t norRawCmd(const char outlen, const char inlen, const char cmd, const uint32_t addr, const uint16_t d) {
        CmdBuf4 buf;
        sendCommand(norCmd(outlen, inlen, cmd, addr, d >> 8, d), 4, buf.u8, 0x180000);
        logMessage(LOG_DEBUG, "R4ISDHC: NOR %X at %X returned %X", cmd, addr, buf.u32);
        return buf.u32;
    }

    void norErase4k(const uint32_t address) {
        norWriteEnable();
        sendCommand(norCmd(0, 4, 0x20, address), 4, nullptr, 0x180000);
        ioDelay(41000000);
    }

    void norWrite256(const uint32_t address, const uint8_t *bytes) {
        norWriteEnable();
        sendCommand(norCmd(0, 6, 2, address, bytes[0], bytes[1]), 4, nullptr, 0x180000);
        for (uint32_t cur = 2; cur < 0x100; cur += 2) {
            sendCommand(norRaw(bytes[cur], bytes[cur+1]), 4, nullptr, 0x180000);
        }
        sendCommand(norRaw(bytes[0], bytes[1], 0xF0), 4, nullptr, 0x180000);
        ioDelay(0x60000);
    }

    void norWrite4k(const uint32_t address, const uint8_t *bytes) {
        uint32_t cur = 0;
        while (cur < 0x1000) {
            norWrite256(address + cur, bytes + cur);
            cur += 0x100;
        }
    }

    bool readNor(const uint32_t address, const uint32_t length, uint8_t *const buffer, bool progress = false) {
        uint32_t cur = 0;

        while (cur < length) {
            uint32_t rd = norRead(address + cur);
            std::memcpy(buffer + cur, &rd, std::min<uint32_t>(length - cur, sizeof(rd)));
            cur += 4;
            if (progress && cur % 0x4000) {
                showProgress(cur, length, "Reading NOR");
            }
        }

        return true;
    }

    bool writeNor(const uint32_t dest_address, const uint32_t length, const uint8_t *const src, bool progress = false,
                    const char *const progress_str = "Writing NOR") {
        const uint32_t real_start = dest_address & ~0xFFF;
        const uint32_t first_page_offset = dest_address & 0xFFF;
        const uint32_t real_length = ((length + first_page_offset) + 0xFFF) & ~0xFFF;
        uint32_t cur = 0;
        uint8_t buf[0x1000];

        if (progress) {
                showProgress(cur, real_length, progress_str);
        }

        while (cur < real_length) {
            const uint32_t cur_addr = real_start + cur;
            const uint32_t buf_ofs = ((dest_address > cur_addr) ? (dest_address - cur_addr) : 0);
            const uint32_t src_ofs = ((cur > first_page_offset) ? (cur - first_page_offset) : 0);
            const uint32_t len = std::min<uint32_t>(0x1000 - buf_ofs, std::min<uint32_t>(length - src_ofs, 0x1000));

            if (!readNor(cur_addr, 0x1000, buf)) {
                logMessage(LOG_ERR, "writeNor: failed to read");
                return false;
            }

            // don't write if they're already identical
            if (std::memcmp(buf + buf_ofs, src + src_ofs, len)) {
                norErase4k(cur_addr);
                // now ideally if i could read the NOR status register, i'd do the memcpy here
                // while the NOR does the sector erase, then just wait on it at the end. BUT NOPE!
                // (and the datasheet doesn't say this chip can read while writing, so that's not an option)

                bool success = false;
                uint32_t retry = 0;
                while (retry < 10) {
                    // some sanity checks..
                    success = norRead(cur_addr) == 0xFFFFFFFF &&
                        norRead(cur_addr + 0x1000 - 4) == 0xFFFFFFFF;
                    if (success) {
                        break;
                    }

                    showProgress(cur, real_length, "Waiting for NOR erase to finish");
                    ++retry;
                    logMessage(LOG_WARN, "writeNor: start or end isn't FF");
                    ioDelay(41000000);
                }

                if (!success) {
                    if (progress) {
                        showProgress(0, 1, "NOR erase sanity check failed");
                    }
                    return false;
                }

                std::memcpy(buf + buf_ofs, src + src_ofs, len);

                norWrite4k(cur_addr, buf);
            }

            cur += 0x1000;
            if (progress) {
                showProgress(cur, real_length, progress_str);
            }
        }

        uint32_t t = norRead(dest_address);
        if (std::memcmp(&t, src, std::min<uint32_t>(length, 4))) {
            if (progress) {
                showProgress(0, 1, "NOR write start verification failed");
            }
            logMessage(LOG_ERR, "writeNor: start mismatch after write");
            return false;
        }

        if (length > 4) {
            t = norRead(dest_address + length - 4);
            if (std::memcmp(&t, src + length - 4, 4)) {
                if (progress) {
                    showProgress(0, 1, "NOR write end verification failed");
                }
                logMessage(LOG_ERR, "writeNor: end mismatch after write");
                return false;
            }
        }
        return true;
    }

    bool trySecureInit(BlowfishKey key) {
        union {
            uint32_t u32;
            uint8_t bytes[4];
        } buf;

        ntrcard::init();
        ntrcard::state.hdr_key1_romcnt = ntrcard::state.key1_romcnt = 0x81808F8;
        ntrcard::state.hdr_key2_romcnt = ntrcard::state.key2_romcnt = 0x416657;
        ntrcard::state.key2_seed = 0;
        if (!ntrcard::initKey1(key)) {
            logMessage(LOG_ERR, "r4isdhc: init key1 (key = %d) failed", static_cast<int>(key));
            return false;
        }
        if (!ntrcard::initKey2()) {
            logMessage(LOG_ERR, "r4isdhc: init key2 failed");
            return false;
        }
        sendCommand(0x66, 4, nullptr, 0x586000);
        sendCommand(0x40199, 4, buf.bytes, 0x180000);
        return buf.u32 != 0xFFFFFFFF;
    }
}

class R4iSDHC : Flashcart {
    bool old_cart;
public:
    // Name & Size of Flash Memory
    R4iSDHC() : Flashcart("R4iSDHC family", 0x200000), old_cart(false) { }

    const char* getAuthor() {
        return
                    "handsomematt, Rai-chan, Kitlith,\n"
            "        stuckpixel, angelsl, et al.";
    }

    const char* getDescription() {
        return
            "\n"
            "A family of DSTT clones. Tested with:\n"
            " * R4iSDHC RTS Lite (r4isdhc.com)\n"
            " * R4i-SDHC 3DS RTS (r4i-sdhc.com)\n"
            " * R4i-SDHC B9S (r4i-sdhc.com)";
    }

    uint8_t sub_20602468(const int addr, const int len, const uint8_t *buf) {
        uint8_t unk_u8[] = {
            0x8B, 0x2C, 0xBC, 0xD4, 0x32, 0x12, 0xEB, 0xBC,
            0x64, 0x84, 0xEC, 0xBF, 0xB9, 0x0F, 0xDB, 0x35,
            0x91, 0x8F, 0x40, 0xAB, 0x51, 0x89, 0xF8, 0x7F,
            0x4E, 0xAA, 0xF7, 0x48, 0xA2, 0x1F, 0x70, 0x1E,
        };

        // p should be 0, but orig code not check this
        uint32_t idx = nor68Cmd(unk_u8[22], unk_u8[23], unk_u8[24], unk_u8[25], unk_u8[26]);
        uint32_t *p = (uint32_t*)&buf[idx];
        for (int a = addr; a <= addr + len; a++, p++) {
            *p = norRawCmd(unk_u8[27], unk_u8[28], unk_u8[29], a, unk_u8[30]);
        }
        return 1;
    }

    uint8_t sub_20602630() {
        sendCommand(0x68, 4, nullptr, 0x180000);

        // step1.
        // sub_20602554
        // ret unknown addr?
        uint32_t v0 = norRead(0x7084);
        if ((v0 & 0xFF) != 0x32) {
            return 2;
        }

        // step2.
        uint32_t v22[7];
        v22[1] = 0x3BE32FA2;
        uint32_t v0_l = 0x02FE5462;
        uint32_t v0_h = 0x3BE32FA2 - 0x4700F435;
        v22[2] = 0xFC7C33B6;
        v22[3] = 0xE191AA40;
        v22[4] = 0x71F28D1D;
        v22[0] = 0xFFFF0030;
        *(uint64_t*)&v22[5] = ((uint64_t)(v0_h) << 32) | v0_l;

        uint8_t unk_v28[104];
        v0 = sub_20602468(0x2F00, 8, unk_v28);

        uint32_t v1 = 0;
        uint8_t *v2 = unk_v28;
        uint8_t v3 = 0xA2;
        uint8_t *v4 = (uint8_t*)&v22[1];

        for (int i = 0; i < 8; i++) {
             if (v2[i] == v3) {
                 v1 = 1;
             }
             v3 = (v4++)[1];
        }
        if (!v1) {
            return 1;
        }

        // step3.
        uint8_t v9 = 0x40;
        uint8_t *v10 = (uint8_t*)&v22[3];
        v1 = 0;

        for (int i = 0; i < 8; i++) {
             if (v2[i] != v9) {
                 v1 = 1;
             }
             v9 = (v10++)[1];
        }
        if (v1) {
            return 2;
        }

        // step4.
        sub_20602468(0x3000, 0x80, unk_v28);
        uint8_t v13 = 0x30;
        uint8_t *v14 = (uint8_t*)&v22[0];
        for (int i = 0; i < 4; i++) {
            if (v14[i] != v13) {
                v1 = 1;
            }

        }
        if (v1) {
            return 2;
        }

        // step5.
        uint8_t v16 = 0x62;
        uint8_t *v17 = &unk_v28[95];
        uint8_t *v18 = (uint8_t*)&v22[5];
        for (int i = 0; i < 8; i++) {
             if (v17[i] != v16) {
                 v1 = 1;
             }
             v16 = (v18++)[1];
        }
        if (v1 == 1) {
            return 2;
        }
        return 0;
    }

    bool initialize() {
        CmdBuf4 buf;
        // this is actually the NOR write disable command
        // the r4isdhc will respond to cart commands with 0xFFFFFFFF if
        // the "magic" command hasn't been sent, so we check for that
        sendCommand(0x40199, 4, buf.u8, 0x180000);
        if (buf.u32 != 0xFFFFFFFF) {
            return false;
        }

        //ntrcard::init();

        // type is 0~2
        uint8_t type = sub_20602630();
        if (type == 2) {
            return false;
        }

        // now it will return zeroes
        sendCommand(0x40199, 4, buf.u8, 0x180000);
        if (buf.u32 == 0) {
            return true;
        }

        old_cart = true;
        return true;
    }

    void shutdown() { }

    bool readFlash(const uint32_t address, const uint32_t length, uint8_t *const buffer) override {
        return readNor(address, length, buffer, true);
    }

    bool writeFlash(const uint32_t address, const uint32_t length, const uint8_t *const buffer) override {
        return writeNor(address, length, buffer, true);
    }

    bool injectNtrBoot(uint8_t *blowfish_key, uint8_t *firm, uint32_t firm_size) override {
        // FIRM is written at 0x7E00; blowfish key at 0x1F1000
        // N.B. this doesn't necessarily mean that the cart's ROM => NOR mapping will
        // allow a FIRM of this size (i.e. old carts), it's just so we don't overwrite
        // the blowfish key
        if (firm_size > (0x1F1000 - 0x7E00)) {
            showProgress(0, 1, "FIRM too big (max 2003456 bytes)");
            return false;
        }

        uint8_t map[0x100] = {0};
        // set the 2nd ROM map to some high value (0x7FFFFFFF in big-endian)
        map[4] = 0x7F; map[5] = 0xFF; map[6] = 0xFF; map[7] = 0xFF;
        return
            // 1:1 map the ROM <=> NOR (unless it's an "old" cart - those don't seem to have
            // a mapping in the NOR)
            (old_cart || writeNor(0x40, 0x100, map, true, "Writing ROM <=> NOR map")) &&
            writeNor(0x1000, 0x48, blowfish_key, true, "Writing Blowfish key (1)") && // blowfish P array
            writeNor(0x1F1000, 0x48, blowfish_key, true, "Writing Blowfish key (2)") &&
            writeNor(0x2000, 0x1000, blowfish_key+0x48, true, "Writing Blowfish key (3)") && // blowfish S boxes
            writeNor(0x1F2000, 0x1000, blowfish_key+0x48, true, "Writing Blowfish key (4)") &&
            writeNor(0x7E00, firm_size, firm, true, "Writing FIRM (1)") && // FIRM
            // older carts read 0x8000-0x10000 from 0x1F8000-0x200000 instead of from 0x8000
            writeNor(0x1F7E00, std::min<uint32_t>(firm_size, old_cart ? 0x8200 : 0x200), firm, true,
                "Writing FIRM (2)"); // FIRM header
    }
};

R4iSDHC r4isdhc;
}
