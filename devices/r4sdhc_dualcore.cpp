#include "device.h"

#include <cstring>
#include <algorithm>

#define BIT(n) (1 << (n))

namespace flashcart_core {
using ntrcard::sendCommand;
using platform::logMessage;
using platform::showProgress;

class R4SDHC_DualCore : Flashcart {
private:
    static const uint8_t cmdEraseFlash[8];
    static const uint8_t cmdWriteByteFlash[8];
    static const uint8_t cmdUnkC0[8];
    static const uint8_t cmdCartVersion[8];
    static const uint8_t cmdReadChipID[8];
    static const uint8_t cmdUnkD0AA[8];
    static const uint8_t cmdUnkD0[8];

    uint8_t decrypt(uint8_t enc) {
        uint8_t dec = 0;
        enc ^= 0x98;
        if (enc & BIT(0)) dec |= BIT(6);
        if (enc & BIT(1)) dec |= BIT(2);
        if (enc & BIT(2)) dec |= BIT(7);
        if (enc & BIT(3)) dec |= BIT(3);
        if (enc & BIT(4)) dec |= BIT(1);
        if (enc & BIT(5)) dec |= BIT(0);
        if (enc & BIT(6)) dec |= BIT(4);
        if (enc & BIT(7)) dec |= BIT(5);
        return dec;
    }

    void encrypt_memcpy(uint8_t *dst, uint8_t *src, uint32_t length)
    {
        for(int i = 0; i < (int)length; ++i)
            dst[i] = encrypt(src[i]);
    }

    uint8_t encrypt(uint8_t dec) {
        uint8_t enc = 0;
        if (dec & BIT(0)) enc |= BIT(6); // 64
        if (dec & BIT(1)) enc |= BIT(2); // 4
        if (dec & BIT(2)) enc |= BIT(7); // 128
        if (dec & BIT(3)) enc |= BIT(3); // 8
        if (dec & BIT(4)) enc |= BIT(1); // 2
        if (dec & BIT(5)) enc |= BIT(0); // 1
        if (dec & BIT(6)) enc |= BIT(4); // 16
        if (dec & BIT(7)) enc |= BIT(5); // 32
        enc ^= 0x2A;
        return enc;
    }

    uint16_t read_cart_version() {
        uint32_t version;
        sendCommand(cmdCartVersion, 4, (uint8_t*)(&version), 0x50);
        uint16_t ret = ((version << 8) & 0xFF00) | ((version >> 8) & 0xFF);
        logMessage(LOG_DEBUG, "R4SDHC: C5 %X(%X)", ret, version);
        return ret;
    }

    uint16_t read_cart_id() {
        uint32_t version;
        sendCommand(cmdReadChipID, 4, (uint8_t*)(&version), 0x50);

        uint16_t ret = ((version << 8) & 0xFF00) | ((version >> 8) & 0xFF);
        logMessage(LOG_DEBUG, "R4SDHC: B8 %X(%X)", ret, version);
        return ret;
    }

    bool unlock_cart() {
        uint32_t dummy;
        sendCommand(cmdUnkD0AA, 4, (uint8_t*)&dummy, 0x50);
        logMessage(LOG_DEBUG, "R4SDHC: D0AA %X", dummy);
        sendCommand(cmdUnkD0, 0, nullptr, 0x50);
        sendCommand(cmdUnkD0AA, 4, (uint8_t*)&dummy, 0x50);
        logMessage(LOG_DEBUG, "R4SDHC: D0AA %X", dummy);
    }

    void erase_cmd(uint32_t address) {
        uint8_t cmdbuf[8];
        logMessage(LOG_DEBUG, "R4SDHC: erase(0x%08x)", address);
        memcpy(cmdbuf, cmdEraseFlash, 8);
        cmdbuf[1] = (address >> 16) & 0xFF;
        cmdbuf[2] = (address >>  8) & 0xFF;
        cmdbuf[3] = (address >>  0) & 0xFF;

        // TODO: find IDB and get the latencies.
        sendCommand(cmdbuf, 0, nullptr, 0x50);
        sendCommand(cmdUnkC0, 0, nullptr, 0x50);
    }

    void write_cmd(uint32_t address, uint8_t value) {
        uint8_t cmdbuf[8];
        logMessage(LOG_DEBUG, "R4SDHC: write(0x%08x) = 0x%02x", address, value);
        memcpy(cmdbuf, cmdWriteByteFlash, 8);
        cmdbuf[1] = (address >> 16) & 0xFF;
        cmdbuf[2] = (address >>  8) & 0xFF;
        cmdbuf[3] = (address >>  0) & 0xFF;
        cmdbuf[4] = value;

        sendCommand(cmdbuf, 0, nullptr);
        sendCommand(cmdUnkC0, 0, nullptr, 0x50);
    }

public:
    R4SDHC_DualCore() : Flashcart("R4 SDHC Dual Core", 0x200000) { }

    bool initialize() {
        //uint8_t dummy[4];

        logMessage(LOG_INFO, "R4SDHC: Init");

        uint16_t version = read_cart_version();
        // TODO check version
        return false;
    }
    void shutdown() {
        logMessage(LOG_INFO, "R4SDHC: Shutdown");
    }

    // We don't have a read command...
    bool readFlash(uint32_t address, uint32_t length, uint8_t *buffer) {
        logMessage(LOG_ERR, "R4SDHC: readFlash not implemented!");
        return false;
    }

    bool writeFlash(uint32_t address, uint32_t length, const uint8_t *buffer) {
        logMessage(LOG_INFO, "R4SDHC: writeFlash(addr=0x%08x, size=0x%x)", address, length);
        for (uint32_t addr=0; addr < length; addr+=0x10000)
            erase_cmd(address + addr);

        for (uint32_t i=0; i < length; i++) {
            write_cmd(address + i, buffer[i]);
            showProgress(i,length, "Writing");
        }

        return true;
    }

    // Need to find offsets first.
    bool injectNtrBoot(uint8_t *blowfish_key, uint8_t *firm, uint32_t firm_size) {
        unlock_cart();
        const uint32_t blowfish_adr = 0x10000;
        const uint32_t firm_hdr_adr = 0x1BE00;
        const uint32_t firm_adr = 0x1C0000;

        logMessage(LOG_INFO, "R4iGold: Injecting ntrboot");
        uint8_t *chunk0 = (uint8_t *)malloc(0x10000);
        readFlash(blowfish_adr, 0x10000, chunk0);
        encrypt_memcpy(chunk0, blowfish_key, 0x1048);
        encrypt_memcpy(chunk0 + firm_hdr_adr, firm, 0x200);
        writeFlash(blowfish_adr, 0x10000, chunk0);
        free(chunk0);

        uint32_t buf_size = PAGE_ROUND_UP(firm_size - 0x200, 0x10000);
        uint8_t *firm_chunk = (uint8_t *)malloc(buf_size);
        readFlash(firm_adr, buf_size, firm_chunk);
        encrypt_memcpy(firm_chunk, firm + 0x200, firm_size);
        writeFlash(firm_adr, buf_size, firm_chunk);

        free(firm_chunk);

        return true;
    }
};


const uint8_t R4SDHC_DualCore::cmdUnkC0[8] = {0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t R4SDHC_DualCore::cmdCartVersion[8] = {0xC5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint8_t R4SDHC_DualCore::cmdUnkD0AA[8] = {0xD0, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t R4SDHC_DualCore::cmdUnkD0[8] = {0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//const uint8_t R4SDHC_DualCore::cmdReadFlash[8] = {0xB7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t R4SDHC_DualCore::cmdReadChipID[8] = {0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8, 0xB8};
const uint8_t R4SDHC_DualCore::cmdEraseFlash[8] = {0xD4, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
const uint8_t R4SDHC_DualCore::cmdWriteByteFlash[8] = {0xD4, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00};

// R4SDHC_DualCore r4sdhc_dualcore;
}
