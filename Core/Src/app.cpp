// app.cpp
//
// Real application logic, in real C++, called from CubeMX-generated
// main.c through the extern "C" bridge in app.hpp. See app.hpp for why
// this file exists instead of just renaming main.c to main.cpp.
//
// Requires the CS pin's CubeMX "User Label" to be set to IMU_CS, which is
// what makes CubeMX generate the IMU_CS_GPIO_Port / IMU_CS_Pin macros
// used below. main.h provides those defines plus the HAL type
// definitions (SPI_HandleTypeDef, UART_HandleTypeDef); the extern
// declarations for hspi1/huart2 themselves are given explicitly below
// since whether main.h re-declares them varies between CubeMX
// versions/settings.

#include "app.hpp"
#include "main.h"
#include "icm42688.hpp"
#include <cstdio>

// CubeMX defines these as ordinary (non-static) globals in main.c, but
// whether main.h re-declares them as extern varies between CubeMX
// versions/settings. Declaring them here directly is version-agnostic:
// it only requires that hspi1/huart2 exist as globals somewhere, which
// CubeMX always guarantees.
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

namespace {
    // Constructed once, at static-init time. This is safe specifically
    // because Icm42688's constructor only stores the handle/pin -- it
    // does no SPI/GPIO access until begin() is called from app_setup(),
    // by which point CubeMX's MX_SPI1_Init()/MX_GPIO_Init() have already
    // run in main().
    icm42688::Icm42688 g_imu(&hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin);
}

void app_setup(void) {
    if (!g_imu.begin()) {
        // WHO_AM_I mismatch: check MISO/MOSI/SCK/CS wiring and confirm
        // SPI1 is configured for Mode 0 before assuming the sensor itself
        // is faulty.
        Error_Handler();
    }
}

void app_loop(void) {
    icm42688::MotionData m;
    if (g_imu.read(m)) {
        char msg[128];
        int len = std::snprintf(msg, sizeof(msg),
            "accel[g]=(%.3f,%.3f,%.3f) gyro[dps]=(%.2f,%.2f,%.2f) T=%.1fC\r\n",
            m.accelX, m.accelY, m.accelZ, m.gyroX, m.gyroY, m.gyroZ, m.temperatureC);
        HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t*>(msg),
                           static_cast<uint16_t>(len), HAL_MAX_DELAY);
    }
    HAL_Delay(100);
}
