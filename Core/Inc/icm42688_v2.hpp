#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>

namespace icm42688 {

// ── 1. SpiBus concept ────────────────────────────────────────────────────
//
// Any type T is a SpiBus if it provides these three operations with
// exactly these signatures.  Checked entirely at compile time; no vtable.
//
template<typename T>
concept SpiBus = requires(T& bus,
                          std::span<const std::byte> tx,
                          std::span<std::byte>       rx) {
    { bus.assertCs()       } -> std::same_as<void>;
    { bus.releaseCs()      } -> std::same_as<void>;
    { bus.transfer(tx, rx) } -> std::same_as<bool>;
};

// ── 2. Register map ──────────────────────────────────────────────────────
//
// enum class gives each name a distinct type: Register::WhoAmI is not
// an integer and cannot be accidentally passed where one is expected.
// The underlying type is uint8_t so it maps directly to the address byte.
//
enum class Register : uint8_t {
    TempDataH    = 0x1D,
    TempDataL    = 0x1E,
    AccelDataX1  = 0x1F,
    AccelDataX0  = 0x20,
    AccelDataY1  = 0x21,
    AccelDataY0  = 0x22,
    AccelDataZ1  = 0x23,
    AccelDataZ0  = 0x24,
    GyroDataX1   = 0x25,
    GyroDataX0   = 0x26,
    GyroDataY1   = 0x27,
    GyroDataY0   = 0x28,
    GyroDataZ1   = 0x29,
    GyroDataZ0   = 0x2A,
    PwrMgmt0     = 0x4E,
    GyroConfig0  = 0x4F,
    AccelConfig0 = 0x50,
    WhoAmI       = 0x75,
};

// ── 3. Sensor configuration ──────────────────────────────────────────────

// Strongly-typed ODR and full-scale selection replaces raw hex constants.
enum class AccelOdr : uint8_t { Hz1000 = 0x06 };
enum class GyroOdr  : uint8_t { Hz1000 = 0x06 };
enum class AccelFsr : uint8_t { G16    = 0x00 };  // +-16 g   -> 2048 LSB/g
enum class GyroFsr  : uint8_t { Dps2000= 0x00 };  // +-2000dps-> 16.4 LSB/dps

struct Config {
    AccelFsr accelFsr = AccelFsr::G16;
    AccelOdr accelOdr = AccelOdr::Hz1000;
    GyroFsr  gyroFsr  = GyroFsr::Dps2000;
    GyroOdr  gyroOdr  = GyroOdr::Hz1000;
};

// ── 4. MotionData ────────────────────────────────────────────────────────

struct MotionData {
    float accelX{}, accelY{}, accelZ{};   // g
    float gyroX{},  gyroY{},  gyroZ{};   // deg/s
    float temperatureC{};                  // °C
};

// ── Platform hook ────────────────────────────────────────────────────────
// Provided by each platform: HAL_Delay(1) on STM32, no-op on the host test.
// Declared before the template so it's visible inside begin().
void platformDelay1ms() noexcept;

// ── 5. Driver ────────────────────────────────────────────────────────────
//
// Icm42688 is now a class template.  The Bus type parameter must satisfy
// SpiBus — checked at instantiation.  Passing a non-conforming type
// produces a clear concept-violation error, not a linker mystery.
//
template<SpiBus Bus>
class Icm42688 {
public:
    // Constructor takes the bus by value (or move).
    // No I/O here — hardware may not be ready at static-init time.
    explicit Icm42688(Bus bus, Config cfg = {}) noexcept
        : bus_{std::move(bus)}, cfg_{cfg} {}

    // [[nodiscard]]: ignoring the return value is almost certainly a bug.
    // Verifies WHO_AM_I, enables accel + gyro, configures ranges and ODR.
    [[nodiscard]] bool begin() noexcept;

    // Returns the latest sample, or nullopt if begin() hasn't succeeded.
    // Caller: if (auto m = imu.read()) { use m->accelX; }
    [[nodiscard]] std::optional<MotionData> read() noexcept;

    [[nodiscard]] bool isReady() const noexcept { return ready_; }


private:
    // ── SPI helpers ──────────────────────────────────────────────────
    [[nodiscard]] bool readReg(Register reg, uint8_t& out) noexcept;
    [[nodiscard]] bool writeReg(Register reg, uint8_t value) noexcept;


    // Burst read: fills 'out' starting at 'startReg', auto-incrementing.
    [[nodiscard]] bool readRegs(Register startReg, std::span<std::byte> out) noexcept;

    // ── Conversion helpers ───────────────────────────────────────────
    static constexpr int16_t toInt16(std::byte hi, std::byte lo) noexcept {
        return static_cast<int16_t>(
            (static_cast<uint16_t>(std::to_integer<uint8_t>(hi)) << 8) |
             static_cast<uint16_t>(std::to_integer<uint8_t>(lo)));
    }
    static constexpr float accelSensitivity(AccelFsr fsr) noexcept{
    	switch(fsr){
    	case AccelFsr::G16: return 2048.0f;
    	}
    	return 0.0f;
    }

    static constexpr float gyroSensitivity(GyroFsr fsr) noexcept{
    	switch(fsr){
    	case GyroFsr::Dps2000: return 16.4f;
    	}
    	return 0.0f;
    }
    // ── State ────────────────────────────────────────────────────────
    Bus    bus_;
    Config cfg_;
    bool   ready_{false};

    // Sensitivities at the configured full-scale ranges (datasheet tables).
    // constexpr: evaluated at compile time, zero runtime storage.

    static constexpr float kTempSens  = 132.48f;
    static constexpr float kTempOff   = 25.0f;

    static constexpr uint8_t kWhoAmIExpected = 0x47u;
    static constexpr uint8_t kReadBit        = 0x80u;  // MSB set = read
    static constexpr uint8_t kPwrLowNoise    = 0x0Fu;  // gyro+accel low-noise
};

// ── Method definitions ───────────────────────────────────────────────────
//
// Defined in the header because this is a class template:
// the compiler needs the full definition at instantiation time.

template<SpiBus Bus>
bool Icm42688<Bus>::begin() noexcept {
    bus_.releaseCs();  // idle CS high before first transaction


    // The first SPI transaction after reset is consumed — isolated
    // experimentally (see bring-up notes); mechanism not confirmed in
    // DS-000347. Discard one read before the real WHO_AM_I check.
    uint8_t discard = 0;
    (void)readReg(Register::WhoAmI, discard);

    uint8_t id = 0;
    if (!readReg(Register::WhoAmI, id) || id != kWhoAmIExpected) {
        ready_ = false;
        return false;
    }

   if(!writeReg(Register::PwrMgmt0, kPwrLowNoise))
   {
	   ready_ = false;
	   return false;
   }

    // datasheet: 1 ms settling after power-mode change.
    // Platform-provided delay; on STM32 this is HAL_Delay(1).
    // On the test host it's a no-op inline.
    platformDelay1ms();

    // Pack FSR + ODR into the config byte (bits [7:5] FSR, [3:0] ODR).
    uint8_t gyroCfg = (static_cast<uint8_t>(cfg_.gyroFsr) << 5) |
                       static_cast<uint8_t>(cfg_.gyroOdr);
    uint8_t accCfg  = (static_cast<uint8_t>(cfg_.accelFsr) << 5) |
                       static_cast<uint8_t>(cfg_.accelOdr);

    if(!writeReg(Register::GyroConfig0,  gyroCfg) ||  !writeReg(Register::AccelConfig0, accCfg))
    {
    	ready_=false;
    	return false;
    }

    ready_ = true;
    return true;
}

template<SpiBus Bus>
std::optional<MotionData> Icm42688<Bus>::read() noexcept {
    if (!ready_) return std::nullopt;

    std::array<std::byte, 12> raw{};
    if(!readRegs(Register::AccelDataX1,raw)){
    	return std::nullopt;
    }
    std::array<std::byte, 2> tmp{};
    if (!readRegs(Register::TempDataH, tmp)) {
        return std::nullopt;
    }

    MotionData m;
    m.accelX = static_cast<float>(toInt16(raw[0], raw[1])) / accelSensitivity(cfg_.accelFsr);
    m.accelY = static_cast<float>(toInt16(raw[2], raw[3])) / accelSensitivity(cfg_.accelFsr);
    m.accelZ = static_cast<float>(toInt16(raw[4], raw[5])) / accelSensitivity(cfg_.accelFsr);
    m.gyroX  = static_cast<float>(toInt16(raw[6], raw[7]))  / gyroSensitivity(cfg_.gyroFsr);
    m.gyroY  = static_cast<float>(toInt16(raw[8], raw[9]))  / gyroSensitivity(cfg_.gyroFsr);
    m.gyroZ  = static_cast<float>(toInt16(raw[10],raw[11])) / gyroSensitivity(cfg_.gyroFsr);

    m.temperatureC = (static_cast<float>(toInt16(tmp[0], tmp[1])) / kTempSens)
                   + kTempOff;

    return m;
}

template<SpiBus Bus>
bool Icm42688<Bus>::readReg(Register reg, uint8_t& out) noexcept {
    std::array<std::byte, 1> tx{std::byte(static_cast<uint8_t>(reg) | kReadBit)};
    std::array<std::byte, 1> rx{};
    bus_.assertCs();
    bool ok = bus_.transfer(tx,{}) && bus_.transfer({},rx);
    bus_.releaseCs();
    if (ok) {
        out = std::to_integer<uint8_t>(rx[0]);
    }
    return ok;
}

template<SpiBus Bus>
bool Icm42688<Bus>::writeReg(Register reg, uint8_t value) noexcept {
    std::array<std::byte, 1> addrByte{std::byte(static_cast<uint8_t>(reg) & ~kReadBit)};
    std::array<std::byte, 1> valByte {std::byte(value)};
    bus_.assertCs();
    bool ok = bus_.transfer(addrByte, {}) && bus_.transfer(valByte, {});
    bus_.releaseCs();
    return ok;
}
template<SpiBus Bus>
bool Icm42688<Bus>::readRegs(Register startReg, std::span<std::byte> out) noexcept {
    std::array<std::byte, 1> tx{std::byte(static_cast<uint8_t>(startReg) | kReadBit)};
    bus_.assertCs();
    bool ok = bus_.transfer(tx, {}) && bus_.transfer({}, out);
    bus_.releaseCs();
    return ok;
}

} // namespace icm42688
