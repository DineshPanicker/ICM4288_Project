# ICM-42688-P SPI Driver — STM32F446RE (C++17 → C++20)

A register-level C++ SPI driver for the TDK InvenSense ICM-42688-P 6-axis IMU
(3-axis accelerometer + 3-axis gyroscope) on an STM32F446RE (Nucleo-F446RE),
written from scratch against the public datasheet (DS-000347).

The repository holds two versions of the same driver:

- **v1 (C++17)** — the original, hardware-validated. A conservative C++ slice:
  a class over the HAL, `bool` returns, a `static constexpr` register map.
- **v2 (C++20)** — a rewrite in idiomatic modern C++: a concept-constrained
  bus template, `enum class` register map, `std::span`, `std::optional`,
  `[[nodiscard]]`/`noexcept`. Also hardware-validated.

I wrote v1 first and got it working on real hardware, then rewrote it as v2 to
lean on modern C++ properly rather than writing C in a `.cpp` file. Both are kept
in the repository on purpose: v1 is the honest starting point and v2 is the step
forward, and seeing the two side by side is the clearest way to show what
changed and why. v1 lives at the `v1.0` tag and in the build-excluded sources;
`main` builds v2.

## What it does

`begin()` verifies the sensor's WHO_AM_I identity register (0x47) before
trusting any data, then configures the accelerometer and gyroscope full-scale
ranges and output data rate. `read()` performs a burst read of the data
registers and converts the raw counts into physical units — g, deg/s, °C. The
converted values stream over UART.

## v1 vs v2 at a glance

| | v1 (C++17) | v2 (C++20) |
|---|---|---|
| Bus abstraction | raw HAL pointers | `SpiBus` concept, class template |
| Register map | `static constexpr uint8_t` | `enum class Register : uint8_t` |
| Buffer passing | `uint8_t* + length` | `std::span<std::byte>` |
| `read()` return | `bool` + out-reference | `std::optional<MotionData>` |
| Error handling | HAL status ignored | `transfer()` returns `bool`, finite timeout, failure propagates |
| Host test | mock via include-path swap | `MockBus` satisfies the concept |

## Design notes

**Driver as a class over the HAL.** `Icm42688` wraps the raw `HAL_SPI_*` calls
behind a small interface. Register addresses, the power-up configuration
sequence, and the sensitivity constants all come from the datasheet
(DS-000347).

**C++ alongside CubeMX's generated `main.c`.** CubeMX regenerates `main.c` as
plain C every time the `.ioc` changes, so renaming it to `main.cpp` would be
silently overwritten on the next code generation. Instead `main.c` stays
untouched and calls two `extern "C"` functions — `app_setup()` and `app_loop()`
— declared in `app.hpp`. All the real logic lives in the `.cpp` files, which
CubeMX never touches, so regenerating from the `.ioc` never disturbs it. Only
three lines live inside the `USER CODE` markers.

**No exceptions, no dynamic allocation.** Built with `-fno-exceptions`; the
driver reports failure through return values (`bool` in v1,
`std::optional`/`bool` in v2) rather than throwing, and allocates nothing on the
heap — standard practice for a resource-constrained microcontroller target.

**v2 is a class template, so it's header-only.** `Icm42688<Bus>` is instantiated
in `app_v2.cpp`, and the compiler needs the method definitions at the point of
instantiation, so they live in the header rather than a separate `.cpp`.
`HalBus` (the real STM32 HAL bus) and `MockBus` (the host test) both satisfy the
`SpiBus` concept — checked at compile time, with no inheritance and no virtual
dispatch.

## Project layout

```
Core/
  Inc/
    icm42688.hpp      - v1 driver interface
    icm42688_v2.hpp   - v2 driver (concept, register map, full template)
    hal_bus.hpp       - v2 HalBus: STM32 HAL bus satisfying SpiBus
    app.hpp           - extern "C" bridge between main.c and the C++ code
    main.h, ...       - CubeMX-generated headers
  Src/
    icm42688.cpp      - v1 driver implementation
    app.cpp           - v1 application layer
    app_v2.cpp        - v2 application layer
    main.c            - CubeMX-generated; calls app_setup()/app_loop()
    ...               - other CubeMX-generated sources
  Startup/
    startup_stm32f446retx.s
test/
  test_v2.cpp         - host-side unit test for v2 (MockBus)
  test_icm42688.cpp   - host-side unit test for v1 (mock HAL)
README.md
```

## Hardware setup

SPI1 in Full-Duplex Master mode, **Mode 0** (CPOL = Low, CPHA = 1 Edge), MSB
first, 10.5 MHz (prescaler 8 on the 84 MHz APB2 clock — comfortably within the
sensor's 24 MHz maximum).

| ICM-42688 pin | STM32 pin | Nucleo Arduino header |
|---|---|---|
| SCLK | PA5 | D13 |
| MISO | PA6 | D12 |
| MOSI | PA7 | D11 |
| CS   | PB6 | D10 |
| VCC  | 3V3 | — |
| GND  | GND | — |
| INT1 / INT2 | not connected | — |

The CS pin (PB6) is configured as `GPIO_Output` in CubeMX with the User Label
`IMU_CS`, which generates the `IMU_CS_GPIO_Port` / `IMU_CS_Pin` macros the driver
uses. Its idle output level is set **High** so CS is not asserted at boot before
the driver runs. INT1/INT2 are left unconnected because the driver polls the
sensor rather than using data-ready interrupts.

## Building

The repository builds one version at a time. In STM32CubeIDE, exclude the other
version's sources from the build (right-click the file → Properties → C/C++ Build
→ Exclude resource from build):

- **v2 (default on `main`):** exclude `app.cpp` and `icm42688.cpp`. Set the C++
  dialect to ISO C++20 (`-std=gnu++20`).
- **v1:** exclude `app_v2.cpp`. Set the dialect to ISO C++17. v1 is also
  available at the `v1.0` tag.

`app.cpp` and `app_v2.cpp` both define `app_setup()`/`app_loop()`, so only one
application file can be in the build at a time; the same applies to the two
driver implementations.

CubeIDE compiles the `.cpp` files with `arm-none-eabi-g++` and the generated
`.c` files with `arm-none-eabi-gcc`. Flash to the Nucleo-F446RE over the onboard
ST-LINK.

## Running

Open a serial terminal on the ST-LINK virtual COM port at **115200 8N1**:

```
picocom -b 115200 /dev/ttyACM0
```

With the sensor wired and powered, the board streams a line roughly every
100 ms. The output is integer-scaled — milli-g, milli-dps, centi-°C — to avoid
the newlib-nano float-`printf` limitation described below:

```
accel_mg=(-84,661,-739) gyro_mdps=(-304,-243,-60) T_cdeg=2599
```

At rest the accelerometer vector magnitude equals 1 g regardless of the board's
orientation, so `(-84, 661, -739)` mg is ≈1.00 g total — the board is simply
tilted. That invariant is a quick correctness check: it validates the
sensitivity constant, the high/low byte order, and the two's-complement
reassembly all at once.

### Note on floating-point `printf`

STM32 projects link newlib-nano by default, which disables floating-point
support in `printf`/`snprintf` to save flash — so `%f` silently prints nothing.
This project sidesteps it by printing scaled integers. The alternative is to
enable **"Use float with printf from newlib-nano"** under Project Properties →
C/C++ Build → Settings → MCU Settings, which adds `-u _printf_float` to the
linker.

## Host-side tests

The driver logic is unit-tested on a desktop compiler with no hardware attached,
against an in-memory mock bus:

```
g++ -std=c++20 -Wall -Wextra -fsanitize=address,undefined -ICore/Inc test/test_v2.cpp -o test_v2 && ./test_v2
```

8/8 checks — WHO_AM_I, all six accelerometer/gyroscope axes, and temperature —
pass with tight tolerances, clean under AddressSanitizer and
UndefinedBehaviorSanitizer. In v2, `MockBus` satisfies the same `SpiBus` concept
as the real `HalBus`, proven at compile time by a `static_assert`, so the test
exercises the exact driver code that ships. A companion negative
`static_assert(!SpiBus<NotABus>)` proves the concept also rejects a
non-conforming type.

## Possible extensions

- Interrupt-driven reads using the sensor's INT1 line (wired to an EXTI GPIO)
  instead of polling, to read exactly on data-ready rather than on a fixed
  interval.
- A single 14-byte burst starting at the temperature registers (0x1D), which sit
  immediately before the accelerometer registers, to fetch temperature and all
  six motion axes in one CS transaction instead of two.
- A DMA-based burst read for higher, more consistent sample rates.
- `std::expected` (C++23) in place of `std::optional`, to carry a typed error
  reason rather than just "no value".

## License

MIT.
