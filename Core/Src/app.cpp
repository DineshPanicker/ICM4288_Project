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
    // Step 1 - confirm UART works
    char h[] = "app_setup reached\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)h, sizeof(h)-1, HAL_MAX_DELAY);

    // Step 2 - read WHO_AM_I raw before begin()
    uint8_t who = g_imu.whoAmIRaw();
    char b[40];
    int n = snprintf(b, sizeof(b), "WHO_AM_I = 0x%02X (expect 0x47)\r\n", who);
    HAL_UART_Transmit(&huart2, (uint8_t*)b, n, HAL_MAX_DELAY);

    // Step 3 - try begin(), but DON'T call Error_Handler on failure
    // so the board keeps running and we can see output
    if (!g_imu.begin()) {
        char e[] = "begin() FAILED - continuing anyway for diagnostics\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)e, sizeof(e)-1, HAL_MAX_DELAY);
    } else {
        char ok[] = "begin() OK\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)ok, sizeof(ok)-1, HAL_MAX_DELAY);
    }
}

void app_loop(void) {
    icm42688::MotionData m;
    if (g_imu.read(m)) {
        char msg[128];
        int len = snprintf(msg, sizeof(msg),
            "accel_mg=(%d,%d,%d) gyro_mdps=(%d,%d,%d) T_cdeg=%d\r\n",
            (int)(m.accelX * 1000),
            (int)(m.accelY * 1000),
            (int)(m.accelZ * 1000),
            (int)(m.gyroX  * 1000),
            (int)(m.gyroY  * 1000),
            (int)(m.gyroZ  * 1000),
            (int)(m.temperatureC * 100));
        HAL_UART_Transmit(&huart2,
            reinterpret_cast<uint8_t*>(msg),
            static_cast<uint16_t>(len),
            HAL_MAX_DELAY);
    }
    HAL_Delay(100);
}
