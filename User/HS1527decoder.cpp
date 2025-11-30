/*
 * HS1527decoder.cpp
 *
 *  Created on: Nov 10, 2025
 *      Author: erlbriton
 */

// HS1527Decoder.cpp
#include "HS1527Decoder.h"
#include <cstdio>
#include <cstring>

HS1527Decoder::HS1527Decoder(TIM_HandleTypeDef* tim_, UART_HandleTypeDef* uart_)
    : htim(tim_), huart(uart_), packetIndex(0), lastTimestamp(0),
      lastButtonCode(0xFF), repeatCount(0)
{
    memset(currentPacket, 0, sizeof(currentPacket));
}

void HS1527Decoder::begin() {
    // запуск Input Capture с прерыванием
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_4);
}
uint8_t asdewq = 0;
// вызывается из ISR Input Capture
void HS1527Decoder::processFront(uint32_t timestamp) {
	uint32_t delta = timestamp - lastTimestamp;
	lastDelta = delta;
	//if (lastDelta > 700) asdewq = 1;

    // проверка паузы между пачками
    if (delta > PAUSE_THRESHOLD && delta < PAUSE_THRESHOLD_big) {
        packetIndex = 0; // начало новой пачки
        asdewq =1;
    }

    if (packetIndex < PACKET_SIZE) {
        currentPacket[packetIndex++] = timestamp;
    }

    lastTimestamp = timestamp;

    // если пачка полная, анализируем
    if (packetIndex == PACKET_SIZE) {
        analyzePacket();
        packetIndex = 0;
    }
}

// анализ одной пачки из 25 фронтов
void HS1527Decoder::analyzePacket() {
    uint8_t bits[PACKET_SIZE] = {0};

    // вычисляем дельты и определяем короткий/длинный импульс
    for (size_t i = 0; i < PACKET_SIZE - 1; ++i) {
        uint32_t delta = currentPacket[i + 1] - currentPacket[i];
        if (delta >= LONG_MIN && delta <= LONG_MAX)
        	bits[i] = 1;
        else if (delta >= SHORT_MIN && delta <= SHORT_MAX)
        	bits[i] = 0;
        else return; // некорректный импульс → discard пачки
    }

    // последние 5 битов → код кнопки
    uint8_t code = 0;
    for (size_t i = PACKET_SIZE - 5; i < PACKET_SIZE; ++i) {
        code = (code << 1) | bits[i];
    }

    // проверка повторяемости
    if (code == lastButtonCode) {
        if (++repeatCount >= 3) { // три одинаковые пачки подряд
            reportCode(code);
            repeatCount = 0;
        }
    } else {
        lastButtonCode = code;
        repeatCount = 1;
    }
}

// вывод кода кнопки через UART
void HS1527Decoder::reportCode(uint8_t code) {
    char buf[32];
    int l = sprintf(buf, "HS1527 button: %02X\r\n", code);
    HAL_UART_Transmit(huart, (uint8_t*)buf, l, 100);
}





