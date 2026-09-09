# Core module layout

The firmware is organized by embedded-domain responsibility rather than an UI
architecture such as MVVM.

- `Inc/protocol/wisun`, `Src/protocol/wisun`: Wi-SUN framing, transport,
  routing, and polling orchestration.
- `Inc/services`, `Src/services`: lighting, RTC, measurement, snapshots,
  storage, and solar-time application services.
- `Inc/signal`, `Src/signal`: FFT and AI signal-processing support.
- `Inc/platform`, `Src/platform`: MCU-facing adapters and compatibility stubs.
- `Inc/utils`, `Src/utils`: shared CBOR, logging, and memory utilities.
- Files kept directly under `Inc` and `Src` are STM32Cube-generated startup,
  HAL, interrupt, and application-entry files.

Includes use paths relative to `Core/Inc` (for example,
`#include "protocol/wisun/wisun_frame.h"`). This keeps dependencies explicit
while requiring only the existing `Core/Inc` compiler include path.
