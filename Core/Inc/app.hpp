#pragma once
//
// app.hpp
//
// C-linkage bridge between CubeMX-generated main.c (which stays plain C
// forever, so CubeMX regeneration never overwrites your work) and the
// C++ driver code in icm42688.hpp/.cpp.
//
// main.c only ever needs to:
//   1. #include "app.hpp"          in USER CODE BEGIN Includes
//   2. call app_setup();           in USER CODE BEGIN 2
//   3. call app_loop();            in USER CODE BEGIN 3 (inside while(1))
//
// Nothing else in main.c changes, so regenerating code from the .ioc
// file after a pinout tweak is safe -- CubeMX preserves the content of
// USER CODE markers, and there's only ever 3 lines living there.

#ifdef __cplusplus
extern "C" {
#endif

void app_setup(void);
void app_loop(void);

#ifdef __cplusplus
}
#endif
