// Host-side unit test: exercises Icm42688's register logic and unit
// conversions against the mock HAL in mock_hal/, with no real hardware
// or ARM toolchain required. Build/run with:
//
//   g++ -std=c++17 -I ../inc -I mock_hal test_icm42688.cpp ../src/icm42688.cpp -o test_icm42688
//   ./test_icm42688
//
#include "icm42688.hpp"
#include <cstdio>
#include <cmath>

namespace {
int failures = 0;

void expectNear(const char* label, float actual, float expected, float tol = 0.01f) {
    if (std::fabs(actual - expected) > tol) {
        std::printf("FAIL %s: got %f, expected %f\n", label, actual, expected);
        ++failures;
    } else {
        std::printf("PASS %s: %f\n", label, actual);
    }
}
} // namespace

int main() {
    auto& regs = mock_sensor::registers();

    // WHO_AM_I must read back 0x47 for begin() to succeed.
    regs[0x75] = 0x47;

    // Pre-load accel/gyro/temp data registers with known raw values.
    regs[0x1F] = 0x08; regs[0x20] = 0x00; // accelX raw = 2048  -> +1.000 g
    regs[0x21] = 0xF8; regs[0x22] = 0x00; // accelY raw = -2048 -> -1.000 g
    regs[0x23] = 0x00; regs[0x24] = 0x00; // accelZ raw = 0     ->  0.000 g
    regs[0x25] = 0x06; regs[0x26] = 0x68; // gyroX raw  = 1640  -> +100.0 dps
    regs[0x27] = 0xF9; regs[0x28] = 0x98; // gyroY raw  = -1640 -> -100.0 dps
    regs[0x29] = 0x00; regs[0x2A] = 0x00; // gyroZ raw  = 0     ->  0.0 dps
    regs[0x1D] = 0x00; regs[0x1E] = 0x00; // temp raw   = 0     ->  25.0 C

    SPI_HandleTypeDef hspi{};
    GPIO_TypeDef csPort{};

    icm42688::Icm42688 imu(&hspi, &csPort, 0);

    bool began = imu.begin();
    if (!began) {
        std::printf("FAIL begin(): WHO_AM_I check did not pass\n");
        return 1;
    }
    std::printf("PASS begin(): WHO_AM_I matched, sensor configured\n");

    icm42688::MotionData m;
    if (!imu.read(m)) {
        std::printf("FAIL read(): returned false\n");
        return 1;
    }

    expectNear("accelX_g", m.accelX, 1.0f);
    expectNear("accelY_g", m.accelY, -1.0f);
    expectNear("accelZ_g", m.accelZ, 0.0f);
    expectNear("gyroX_dps", m.gyroX, 100.0f, 0.1f);
    expectNear("gyroY_dps", m.gyroY, -100.0f, 0.1f);
    expectNear("gyroZ_dps", m.gyroZ, 0.0f, 0.1f);
    expectNear("temperatureC", m.temperatureC, 25.0f, 0.1f);

    if (failures > 0) {
        std::printf("\n%d check(s) FAILED\n", failures);
        return 1;
    }
    std::printf("\nAll checks passed.\n");
    return 0;
}
