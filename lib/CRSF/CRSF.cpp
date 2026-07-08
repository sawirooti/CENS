#include "CRSF.h"

// CRC8, полином 0xD5 (DVB-S2), используется в протоколе CRSF
uint8_t CRSF::crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc <<= 1;
        }
    }
    return crc;
}

void CRSF::begin(HardwareSerial &serial, uint32_t baud) {
    port = &serial;
    port->begin(baud);
}

void CRSF::update() {
    if (!port) return;

    while (port->available()) {
        uint8_t b = port->read();

        if (bufPos == 0) {
            // ждём байт адреса устройства (0xC8 - обычно для полётного контроллера)
            if (b == 0xC8 || b == 0xEE || b == 0xEA) {
                buf[bufPos++] = b;
            }
            continue;
        }

        if (bufPos == 1) {
            expectedLen = b; // длина = type + payload + crc
            if (expectedLen < 2 || expectedLen > CRSF_MAX_PACKET_SIZE - 2) {
                bufPos = 0; // мусор, сброс синхронизации
                continue;
            }
            buf[bufPos++] = b;
            continue;
        }

        buf[bufPos++] = b;

        if (bufPos == expectedLen + 2) {
            parsePacket();
            bufPos = 0;
        }

        if (bufPos >= CRSF_MAX_PACKET_SIZE) {
            bufPos = 0; // защита от переполнения буфера
        }
    }

    // если пакетов нет дольше 500 мс - считаем, что связь пропала
    if (linkUp && millis() - lastFrameTime > 500) {
        linkUp = false;
    }
}

void CRSF::parsePacket() {
    uint8_t type = buf[2];
    uint8_t receivedCrc = buf[expectedLen + 1];
    uint8_t calcCrc = crc8(&buf[2], expectedLen - 1); // crc считается от type до конца payload

    if (receivedCrc != calcCrc) {
        return; // битый пакет - игнорируем
    }

    if (type == 0x16) { // RC_CHANNELS_PACKED
        unpackChannels(&buf[3]);
        linkUp = true;
        lastFrameTime = millis();
    }
}

void CRSF::unpackChannels(const uint8_t *payload) {
    // 16 каналов по 11 бит, упакованы подряд в 22 байта (little-endian bitstream)
    uint32_t bitBuffer = 0;
    uint8_t bitCount = 0;
    uint8_t byteIndex = 0;

    for (uint8_t ch = 0; ch < CRSF_CHANNELS; ch++) {
        while (bitCount < 11) {
            bitBuffer |= (uint32_t)payload[byteIndex++] << bitCount;
            bitCount += 8;
        }
        channels[ch] = bitBuffer & 0x7FF;
        bitBuffer >>= 11;
        bitCount -= 11;
    }
}

uint16_t CRSF::getChannel(uint8_t ch) const {
    if (ch >= CRSF_CHANNELS) return 992;
    return channels[ch];
}

uint16_t CRSF::getChannelUs(uint8_t ch) const {
    uint16_t raw = getChannel(ch);
    return map(raw, 172, 1811, 988, 2012);
}
