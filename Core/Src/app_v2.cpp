// app_v2.cpp
//
// Application layer for the C++20 driver rewrite.
// The extern "C" bridge in app.hpp is unchanged — main.c still calls
// app_setup() and app_loop() without knowing anything is different.

#include "app.hpp"
#include "main.h"
#include "icm42688_v2.hpp"
#include "hal_bus.hpp"
#include <cstdio>

extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart2;

// platformDelay1ms: STM32 implementation of the platform hook in icm42688_v2.hpp
namespace icm42688 {
    void platformDelay1ms() noexcept { HAL_Delay(1); }
}

namespace {
    // HalBus satisfies SpiBus — verified by the compiler at instantiation.
    icm42688::Icm42688<icm42688::HalBus> g_imu{
        icm42688::HalBus{&hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin}
    };
}

void app_setup(void) {
    if (!g_imu.begin()) {
        char e[] = "begin() FAILED\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)e, sizeof(e)-1, HAL_MAX_DELAY);
        Error_Handler();
    }
}

void app_loop(void) {
    // std::optional<MotionData> — check before using.
    if (auto m = g_imu.read()) {
        char msg[128];
        // Integer-scaled output to avoid the newlib-nano float-printf issue.
        // milli-g, milli-dps, centi-Celsius — no float in the format string.
        int len = std::snprintf(msg, sizeof(msg),
            "accel_mg=(%d,%d,%d) gyro_mdps=(%d,%d,%d) T_cdeg=%d\r\n",
            static_cast<int>(m->accelX * 1000.0f),
            static_cast<int>(m->accelY * 1000.0f),
            static_cast<int>(m->accelZ * 1000.0f),
            static_cast<int>(m->gyroX  * 1000.0f),
            static_cast<int>(m->gyroY  * 1000.0f),
            static_cast<int>(m->gyroZ  * 1000.0f),
            static_cast<int>(m->temperatureC * 100.0f));
        if (len > 0 && static_cast<std::size_t>(len) < sizeof(msg)) {
            HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t*>(msg),
                              static_cast<uint16_t>(len), HAL_MAX_DELAY);
        }
    }
    HAL_Delay(100);
}
