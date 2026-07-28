#pragma once
//
// Minimal mock of the STM32 HAL SPI/GPIO surface used by icm42688.cpp,
// so the driver's register logic and unit conversions can be unit-tested
// on a desktop compiler without real hardware. NOT a real HAL -- swap
// this header out for CubeIDE's actual stm32f4xx_hal.h when building for
// the target.
//
#include <cstdint>
#include <cstring>
#include <map>

using HAL_StatusTypeDef = int;
constexpr HAL_StatusTypeDef HAL_OK = 0;
constexpr uint32_t HAL_MAX_DELAY = 0xFFFFFFFFu;

struct SPI_HandleTypeDef { int dummy_ = 0; };
struct GPIO_TypeDef      { int dummy_ = 0; };

constexpr int GPIO_PIN_RESET = 0;
constexpr int GPIO_PIN_SET   = 1;

// --- Fake sensor register file, so tests can pre-load expected data ---
namespace mock_sensor {
    inline std::map<uint8_t, uint8_t>& registers() {
        static std::map<uint8_t, uint8_t> regs;
        return regs;
    }
    // Tracks the "currently addressed" register across a CS-low burst,
    // mimicking the real chip's auto-increment behaviour.
    inline uint8_t& cursor() {
        static uint8_t c = 0;
        return c;
    }
    inline bool& addressPhase() {
        static bool a = true; // next TransmitReceive call is the address byte
        return a;
    }
}

inline void HAL_GPIO_WritePin(GPIO_TypeDef*, uint16_t, int level) {
    if (level == GPIO_PIN_RESET) {
        mock_sensor::addressPhase() = true; // CS just went low: next byte is address
    }
}

inline HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*, uint8_t* data, uint16_t size, uint32_t) {
    if (size == 1 && mock_sensor::addressPhase()) {
        uint8_t reg = data[0] & 0x7F;
        mock_sensor::cursor() = reg;
        mock_sensor::addressPhase() = false;
    } else if (size == 1) {
        // write path: data[0] is the value being written to cursor()
        mock_sensor::registers()[mock_sensor::cursor()] = data[0];
    }
    return HAL_OK;
}

inline HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*, uint8_t*, uint8_t* rxData, uint16_t size, uint32_t) {
    for (uint16_t i = 0; i < size; ++i) {
        rxData[i] = mock_sensor::registers()[mock_sensor::cursor()];
        mock_sensor::cursor()++;
    }
    return HAL_OK;
}

inline void HAL_Delay(uint32_t) {}
