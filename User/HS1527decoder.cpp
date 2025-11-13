/*
 * HS1527decoder.cpp
 *
 *  Created on: Nov 10, 2025
 *      Author: erlbriton
 */

#include "HS1527Decoder.h"
#include <cstdio>


HS1527Decoder::HS1527Decoder(TIM_HandleTypeDef* tim_, UART_HandleTypeDef* uart_)
    : state(IDLE), bufIdx(0), lastProcIdx(0), htim(tim_), huart(uart_)
{
    memset((void*)timestamps, 0, sizeof(timestamps));
}

void HS1527Decoder::begin() {
    if (HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_4, (uint32_t*)timestamps, IC_DMA_BUF_LEN) != HAL_OK) {
        const char t[] = "IC DMA start FAIL\r\n";
        HAL_UART_Transmit(huart, (uint8_t*)t, sizeof(t) - 1, 100);
    } else {
        const char t[] = "IC DMA started\r\n";
        HAL_UART_Transmit(huart, (uint8_t*)t, sizeof(t) - 1, 100);
    }
}

void HS1527Decoder::onDmaHalfComplete() {
    processBuffer(0, IC_DMA_BUF_LEN / 2);
}

void HS1527Decoder::onDmaComplete() {
    processBuffer(IC_DMA_BUF_LEN / 2, IC_DMA_BUF_LEN);
}

inline uint32_t HS1527Decoder::diffTicks(uint32_t a, uint32_t b) {
    if (b >= a) return b - a;
    return (0xFFFFFFFFu - a) + 1u + b;
}

void HS1527Decoder::processBuffer(size_t start, size_t end) {
    // Параметры под твои импульсы
    const uint32_t T_MIN   = 8;   // короткий импульс ≈ 10 тиков
    const uint32_t T_MAX   = 12;

    const uint32_t LONG_MIN = 18; // длинный импульс ≈ 20 тиков
    const uint32_t LONG_MAX = 22;

    const uint32_t SYNC_MIN = 80; // пауза между пакетами ≈ 90 тиков

    for (size_t i = start + 1; i < end; ++i) {
        uint32_t prev = timestamps[i - 1];
        uint32_t cur  = timestamps[i];
        if (prev == 0 || cur == 0) continue;

        uint32_t delta = diffTicks(prev, cur);

        if (delta >= SYNC_MIN) {
            // Обработка пакета
            size_t j = i + 1;
            const size_t needIntervals = 24 * 2;
            if (j + needIntervals > end) continue;

            uint32_t intervals[48];
            for (size_t k = 0; k < needIntervals; ++k) {
                uint32_t a = timestamps[i + k];
                uint32_t b = timestamps[i + k + 1];
                intervals[k] = (a && b) ? diffTicks(a, b) : 0xFFFFFFFFu;
            }

            uint32_t code = 0;
            bool ok = true;
            for (size_t bit = 0; bit < 24; ++bit) {
                uint32_t high = intervals[bit * 2];
                uint32_t low  = intervals[bit * 2 + 1];

                // Определяем короткий/длинный импульс по твоим тикам
                if (high >= T_MIN && high <= T_MAX && low >= LONG_MIN && low <= LONG_MAX) {
                    code = (code << 1);          // 0
                } else if (high >= LONG_MIN && high <= LONG_MAX && low >= T_MIN && low <= T_MAX) {
                    code = (code << 1) | 1u;    // 1
                } else {
                    ok = false;                 // неверный пакет
                    break;
                }
            }

            if (ok) {
                reportCode(code);
                i += needIntervals; // пропускаем уже обработанные интервалы
            }
        }
    }
}


void HS1527Decoder::reportCode(uint32_t code) {
    char buf[64];
    int l = sprintf(buf, "HS1527 code: %06lX\r\n", (unsigned long)code);
    HAL_UART_Transmit(huart, (uint8_t*)buf, l, 50);
}


