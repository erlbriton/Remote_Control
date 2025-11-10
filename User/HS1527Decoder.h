/*
 * HS1527decoder.h
 *
 *  Created on: Nov 10, 2025
 *      Author: erlbriton
 */

#pragma once
#include "main.h"
#include <cstdint>
#include <cstring>

#define IC_DMA_BUF_LEN 512   // длина буфера таймстемпов

class HS1527Decoder {
public:
    HS1527Decoder(TIM_HandleTypeDef* tim, UART_HandleTypeDef* uart);

    void begin();                   // старт таймера + DMA
    void onDmaHalfComplete();       // callback DMA half complete
    void onDmaComplete();           // callback DMA full complete

private:
    enum State { IDLE, RECEIVING } state;

    TIM_HandleTypeDef* htim;
    UART_HandleTypeDef* huart;

    volatile uint32_t timestamps[IC_DMA_BUF_LEN];
    size_t bufIdx;
    size_t lastProcIdx;

    void processBuffer(size_t start, size_t end);
    void reportCode(uint32_t code);
    inline uint32_t diffTicks(uint32_t a, uint32_t b);
};
