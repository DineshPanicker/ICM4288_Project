#pragma once
//
// hal_bus.hpp  —  STM32 HAL implementation of the SpiBus concept
//
// HalBus is the "real" bus.  It satisfies SpiBus because it provides
// assertCs(), releaseCs(), and transfer() with exactly the right signatures.
// No inheritance, no virtual functions — the concept check happens at
// compile time when Icm42688<HalBus> is instantiated.

#include <cstddef>
#include <span>
#include "stm32f4xx_hal.h"

namespace icm42688 {

struct HalBus {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef*      csPort;
    uint16_t           csPin;

    void assertCs()  noexcept {
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    }
    void releaseCs() noexcept {
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    }

    // transfer: if tx is non-empty, transmit those bytes;
    //           if rx is non-empty, clock bytes in.
    // Both tx-only, rx-only, and simultaneous are valid calls.
    void transfer(std::span<const std::byte> tx,
                  std::span<std::byte>       rx) noexcept {
        if (!tx.empty()) {
            // NOLINTNEXTLINE: HAL needs non-const uint8_t*; cast is safe.
            HAL_SPI_Transmit(hspi,
                const_cast<uint8_t*>(
                    reinterpret_cast<const uint8_t*>(tx.data())),
                static_cast<uint16_t>(tx.size()),
                HAL_MAX_DELAY);
        }
        if (!rx.empty()) {
            uint8_t dummy = 0;
            HAL_SPI_Receive(hspi, reinterpret_cast<uint8_t*>(rx.data()),
            		static_cast<uint16_t>(rx.size()),
					HAL_MAX_DELAY);
        }
    }
};

} // namespace icm42688
