#pragma once
#include <Arduino.h>

#define CRSF_CHANNELS 16
#define CRSF_MAX_PACKET_SIZE 64

// Простой парсер протокола CRSF (ExpressLRS / Crossfire) для HardwareSerial.
// Работает по неинвертированному UART, поэтому подходит для RX1/TX1 (Serial1) на Mega.
class CRSF {
public:
    void begin(HardwareSerial &serial, uint32_t baud = 420000);

    // Вызывать каждый loop() как можно чаще - вычитывает и парсит все доступные байты
    void update();

    // "Сырое" значение канала, диапазон CRSF: 172..1811, центр 992
    uint16_t getChannel(uint8_t ch) const;

    // То же самое, но пересчитано в привычные "servo" микросекунды 988..2012
    uint16_t getChannelUs(uint8_t ch) const;

    // Есть ли связь с приёмником (пакеты приходят регулярно)
    bool isLinkUp() const { return linkUp; }

private:
    HardwareSerial *port = nullptr;

    uint8_t buf[CRSF_MAX_PACKET_SIZE];
    uint8_t bufPos = 0;
    uint8_t expectedLen = 0;

    uint16_t channels[CRSF_CHANNELS] = {0};
    bool linkUp = false;
    unsigned long lastFrameTime = 0;

    static uint8_t crc8(const uint8_t *data, uint8_t len);
    void parsePacket();
    void unpackChannels(const uint8_t *payload);
};
