# ICM-42688 IMU Driver — STM32F446RE (C++)

A small C++ SPI driver for the TDK InvenSense ICM-42688-P 6-axis IMU
(3-axis accelerometer + 3-axis gyroscope), running on an STM32F446RE
(Nucleo-F446RE) with STM32CubeIDE and the STM32 HAL.

Written from scratch against the public ICM-42688-P datasheet. The driver
reads accelerometer, gyroscope, and die-temperature data over SPI and
streams the converted values (g, deg/s, °C) over UART.

## Why SPI

The ICM-42688-P supports I2C, I3C, and SPI. This project uses SPI because
it runs up to 24 MHz (versus ~1 MHz for I2C on this part) and is the
interface used in the motion-tracking applications the chip is designed
for.

## Project layout

```
Core/
  Inc/
    icm42688.hpp   - driver interface (register map, MotionData struct)
    app.hpp        - extern "C" bridge between main.c and the C++ code
    main.h, ...    - CubeMX-generated headers
  Src/
    icm42688.cpp   - driver implementation (HAL SPI/GPIO calls)
    app.cpp        - application logic: owns the driver, reads + prints
    main.c         - CubeMX-generated; calls app_setup()/app_loop()
    ...            - other CubeMX-generated sources
  Startup/
    startup_stm32f446retx.s
README.md
```

## Design notes

**Driver as a class over the HAL.** `Icm42688` wraps the raw
`HAL_SPI_*` calls behind a small interface: `begin()` verifies the
sensor's WHO_AM_I identity register and configures the accel/gyro ranges
and output data rate; `read()` performs a burst read of the data
registers and converts the raw counts into physical units. Register
addresses and sensitivities come from the datasheet.

**C++ alongside CubeMX's generated `main.c`.** CubeMX always regenerates
`main.c` as plain C, so renaming it to `main.cpp` would be overwritten on
the next code generation. Instead, `main.c` stays untouched and calls two
`extern "C"` functions — `app_setup()` and `app_loop()` — declared in
`app.hpp` and implemented in `app.cpp`. All the real logic lives in C++,
and regenerating from the `.ioc` file never disturbs it.

**No exceptions, no dynamic allocation.** The driver returns `bool`
status rather than throwing, and allocates nothing on the heap — standard
practice for a resource-constrained microcontroller target.

## Hardware setup

SPI1 in Full-Duplex Master mode, **Mode 0** (CPOL = Low, CPHA = 1 Edge),
MSB first, clock ≤ 24 MHz.

| ICM-42688 pin | STM32 pin | Nucleo Arduino header |
|---|---|---|
| SCLK | PA5 | D13 |
| MISO | PA6 | D12 |
| MOSI | PA7 | D11 |
| CS   | PB6 | D10 |
| VCC  | 3V3 | — |
| GND  | GND | — |
| INT1 / INT2 | not connected | — |

The CS pin (PB6) is configured as `GPIO_Output` in CubeMX with the User
Label `IMU_CS`, which generates the `IMU_CS_GPIO_Port` / `IMU_CS_Pin`
macros the driver uses. INT1/INT2 are left unconnected because the driver
polls the sensor rather than using data-ready interrupts.

## Building and running

1. Open the project in STM32CubeIDE and build (it compiles the `.cpp`
   files with `arm-none-eabi-g++` and the generated `.c` files with
   `arm-none-eabi-gcc`).
2. Flash to the Nucleo-F446RE over the onboard ST-LINK.
3. Open a serial terminal on the ST-LINK virtual COM port at
   **115200 8N1**, e.g.:
   ```
   picocom -b 115200 -d 8 -y n -p 1 --flow n /dev/ttyACM0
   ```
4. With the sensor wired and powered, the board streams a line roughly
   every 100 ms:
   ```
   accel[g]=(0.01,-0.02,1.00) gyro[dps]=(0.1,-0.2,0.0) T=25.3C
   ```

### Note on floating-point `printf`

STM32 projects link newlib-nano by default, which disables
floating-point support in `printf`/`snprintf` to save flash. To print the
`%f` values, enable **"Use float with printf from newlib-nano"** under
Project Properties → C/C++ Build → Settings → MCU Settings (then rebuild).

## Possible extensions

- Interrupt-driven reads using the sensor's INT1 line (wired to an EXTI
  GPIO) instead of polling, to avoid SPI transactions between data-ready
  events.
- A FIFO-based read path for higher, more consistent sample rates.
- A hardware-independent bus interface to allow unit-testing the driver
  logic against a mock SPI backend on a host compiler.

## License

MIT.
