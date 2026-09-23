// test_v2.cpp
//
// Host-side unit test for the C++20 driver rewrite.
//
// MockBus satisfies SpiBus exactly as HalBus does, but uses an in-memory
// register file instead of real hardware.  The concept check at
// Icm42688<MockBus> instantiation proves both implementations conform.
//
// Build and run:
//   g++ -std=c++20 -Wall -Wextra -I. test_v2.cpp -o test_v2 && ./test_v2

#include "icm42688_v2.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <map>
#include <span>

// ── Platform hook (no-op on the host) ────────────────────────────────────
namespace icm42688 {
    void platformDelay1ms() noexcept {}
}

// ── MockBus ───────────────────────────────────────────────────────────────
//
// Satisfies SpiBus: same three signatures as HalBus.
// Internally maintains a register file and simulates the ICM-42688 SPI
// framing (address byte with read bit, then data bytes).
//
struct MockBus {
    std::map<uint8_t, uint8_t> regs;   // fake register file
    bool     addressPhase{true};        // next byte is an address
    uint8_t  currentReg{0};
    bool     isRead{false};

    void assertCs()  noexcept { addressPhase = true; }
    void releaseCs() noexcept {}

    bool transfer(std::span<const std::byte> tx,
                  std::span<std::byte>       rx) noexcept {
        if (!tx.empty() && addressPhase) {
            uint8_t addr = std::to_integer<uint8_t>(tx[0]);
            isRead      = (addr & 0x80u) != 0;
            currentReg  = addr & 0x7Fu;
            addressPhase = false;
            return true;
        }
        if (!tx.empty() && !isRead) {
            // write path
            regs[currentReg] = std::to_integer<uint8_t>(tx[0]);
            ++currentReg;
            return true;
        }
        if (!rx.empty() && isRead) {
            // read path: fill the whole span, auto-incrementing like the chip
            for (auto& b : rx) {
                b = std::byte{regs[currentReg]};
                ++currentReg;
            }
        }
        return true;
    }
};

// Verify MockBus satisfies the concept — compile-time guarantee.
static_assert(icm42688::SpiBus<MockBus>, "MockBus must satisfy SpiBus");

// ── Test helpers ──────────────────────────────────────────────────────────
static int failures = 0;

void expectNear(const char* label, float actual, float expected, float tol = 0.01f) {
    if (std::fabs(actual - expected) > tol) {
        std::printf("FAIL  %s: got %.4f, expected %.4f\n", label, actual, expected);
        ++failures;
    } else {
        std::printf("PASS  %s: %.4f\n", label, actual);
    }
}

// ── Pre-load helpers ──────────────────────────────────────────────────────
void setReg16(MockBus& bus, uint8_t addr, int16_t val) {
    bus.regs[addr]     = static_cast<uint8_t>(val >> 8);
    bus.regs[addr + 1] = static_cast<uint8_t>(val & 0xFF);
}

// ── main ──────────────────────────────────────────────────────────────────
int main() {
    MockBus mock;

    // WHO_AM_I must return 0x47
    mock.regs[0x75] = 0x47;

    // Accel: X=+1g, Y=-1g, Z=0
    setReg16(mock, 0x1F, 2048);
    setReg16(mock, 0x21, -2048);
    setReg16(mock, 0x23, 0);

    // Gyro: X=+100 dps, Y=-100 dps, Z=0
    setReg16(mock, 0x25, 1640);
    setReg16(mock, 0x27, -1640);
    setReg16(mock, 0x29, 0);

    // Temp: raw 0 -> 25.0 C
    setReg16(mock, 0x1D, 0);

    icm42688::Icm42688<MockBus> imu{std::move(mock)};

    if (!imu.begin()) {
        std::printf("FAIL  begin(): WHO_AM_I check did not pass\n");
        return 1;
    }
    std::printf("PASS  begin(): WHO_AM_I matched\n");

    auto m = imu.read();
    if (!m) {
        std::printf("FAIL  read(): returned nullopt\n");
        return 1;
    }

    expectNear("accelX_g",      m->accelX,       1.0f);
    expectNear("accelY_g",      m->accelY,      -1.0f);
    expectNear("accelZ_g",      m->accelZ,       0.0f);
    expectNear("gyroX_dps",     m->gyroX,      100.0f, 0.1f);
    expectNear("gyroY_dps",     m->gyroY,     -100.0f, 0.1f);
    expectNear("gyroZ_dps",     m->gyroZ,        0.0f, 0.1f);
    expectNear("temperatureC",  m->temperatureC, 25.0f, 0.1f);

    if (failures > 0) {
        std::printf("\n%d check(s) FAILED\n", failures);
        return 1;
    }
    std::printf("\nAll checks passed.\n");
    return 0;
}
