/*
 * HS1527Decoder.h
 *
 *  Created on: Nov 10, 2025
 *      Author: erlbriton
 */

#pragma once
#include "main.h"
#include <cstdint>

class HS1527Decoder {
public:
    HS1527Decoder(TIM_HandleTypeDef* tim_, UART_HandleTypeDef* uart_);

    void begin();                       // запуск Input Capture
    void processFront(uint32_t timestamp); // вызывается из ISR Input Capture

    uint32_t lastDelta;

private:
    TIM_HandleTypeDef* htim;
    UART_HandleTypeDef* huart;

    static constexpr size_t PACKET_SIZE = 25;
    static constexpr uint32_t PAUSE_THRESHOLD = 6800; // пауза между пачками (~7 мс)
    static constexpr uint32_t PAUSE_THRESHOLD_big = 7300;
    static constexpr uint32_t SHORT_MIN = 180;  // короткий импульс ~216 мкс
    static constexpr uint32_t SHORT_MAX = 250;
    static constexpr uint32_t LONG_MIN  = 600;  // длинный импульс ~680 мкс
    static constexpr uint32_t LONG_MAX  = 750;

    uint32_t currentPacket[PACKET_SIZE];
    size_t packetIndex;
    uint32_t lastTimestamp;

    uint8_t lastButtonCode;
    uint8_t repeatCount;

    void analyzePacket();      // превращает timestamps в биты и определяет кнопку
    void reportCode(uint8_t code);
};


