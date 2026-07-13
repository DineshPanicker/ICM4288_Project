#include "icm42688.hpp"

namespace icm42688 {

Icm42688::Icm42688(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin)
    : hspi_(hspi), csPort_(csPort), csPin_(csPin) {}

void Icm42688::csLow()  { HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET); }
void Icm42688::csHigh() { HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET); }

uint8_t Icm42688::readRegister(uint8_t reg) {
    uint8_t txByte = static_cast<uint8_t>(reg | kDirRead);
    uint8_t rxByte = 0;
    uint8_t dummy  = 0;

    csLow();
    HAL_SPI_Transmit(hspi_, &txByte, 1, HAL_MAX_DELAY);
    HAL_SPI_TransmitReceive(hspi_, &dummy, &rxByte, 1, HAL_MAX_DELAY);
    csHigh();

    return rxByte;
}

void Icm42688::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t txByte = static_cast<uint8_t>(reg & 0x7F); // MSB clear = write
    csLow();
    HAL_SPI_Transmit(hspi_, &txByte, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(hspi_, &value, 1, HAL_MAX_DELAY);
    csHigh();
}

void Icm42688::readRegisters(uint8_t startReg, uint8_t* buffer, uint16_t length) {
    uint8_t txByte = static_cast<uint8_t>(startReg | kDirRead);

    csLow();
    HAL_SPI_Transmit(hspi_, &txByte, 1, HAL_MAX_DELAY);
    // The sensor auto-increments its internal register pointer on each
    // clocked byte, so one CS-low burst reads all `length` bytes.
    for (uint16_t i = 0; i < length; ++i) {
        uint8_t dummy = 0;
        HAL_SPI_TransmitReceive(hspi_, &dummy, &buffer[i], 1, HAL_MAX_DELAY);
    }
    csHigh();
}

bool Icm42688::begin() {
    csHigh(); // idle high before the first transaction

    if (readRegister(kWhoAmIReg) != kWhoAmIExpected) {
        initialised_ = false;
        return false;
    }

    writeRegister(kPwrMgmt0Reg, kPwrGyroLnAccelLn);
    HAL_Delay(1); // datasheet: allow settling time after a power-mode change

    writeRegister(kGyroConfig0Reg, kGyroFs2000Odr1k);
    writeRegister(kAccelConfig0Reg, kAccelFs16gOdr1k);

    initialised_ = true;
    return true;
}

bool Icm42688::read(MotionData& out) {
    if (!initialised_) {
        return false;
    }

    uint8_t raw[12] = {0};
    readRegisters(kAccelDataX1Reg, raw, sizeof(raw));

    auto toInt16 = [](uint8_t hi, uint8_t lo) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    };

    const int16_t ax = toInt16(raw[0], raw[1]);
    const int16_t ay = toInt16(raw[2], raw[3]);
    const int16_t az = toInt16(raw[4], raw[5]);
    const int16_t gx = toInt16(raw[6], raw[7]);
    const int16_t gy = toInt16(raw[8], raw[9]);
    const int16_t gz = toInt16(raw[10], raw[11]);

    out.accelX = static_cast<float>(ax) / kAccelSensitivityLsbPerG;
    out.accelY = static_cast<float>(ay) / kAccelSensitivityLsbPerG;
    out.accelZ = static_cast<float>(az) / kAccelSensitivityLsbPerG;
    out.gyroX  = static_cast<float>(gx) / kGyroSensitivityLsbPerDps;
    out.gyroY  = static_cast<float>(gy) / kGyroSensitivityLsbPerDps;
    out.gyroZ  = static_cast<float>(gz) / kGyroSensitivityLsbPerDps;

    uint8_t tempRaw[2] = {0};
    readRegisters(kTempDataReg, tempRaw, sizeof(tempRaw));
    const int16_t t = toInt16(tempRaw[0], tempRaw[1]);
    out.temperatureC = (static_cast<float>(t) / kTempSensitivityLsbPerC) + kTempOffsetC;

    return true;
}

} // namespace icm42688
