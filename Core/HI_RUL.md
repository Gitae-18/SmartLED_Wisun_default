# HI / RUL integration

This implementation is in `C:/Smaled_On/SmartLED_Wisun_default`.
`ON_TEST_v1` supplied the model settings and RUL equations; it was not edited.

## Runtime

`main.c` calls `AI_Runtime_Init()` once after model initialization. Each existing
scheduled AI sensor capture supplies `[frequency_kHz, adc_peak_counts, voltage_V,
current_A]` plus the measured temperature and its validity flag to
`AI_Runtime_Process()`. The existing gateway-controlled inference interval is
preserved (default 60 seconds); this does not enable continuous ADC acquisition.

The processing chain is:

```text
AI_Runtime_Process -> AI_infer -> Min-Max -> INT8 quantization
  -> AI_ModelRun -> dequantization -> MSE -> HI / anomaly decision
  -> hourly RUL_Update -> RUL_Storage_Save
```

The model settings are:

- Input/output: 4 INT8 values; scales `0.003766168` / `0.003898344`; zero points -128.
- Anomaly: MSE >= `AI_SMPS_MSE_THRESHOLD` (100 W default `0.011325965`).
- HI: clamp((MSE - `0.000452750`) / (`0.000475430` - `0.000452750`), 0, 1).
- Min-Max bounds are unchanged from the ON_TEST_v1 model migration.

The RUL formulas preserve ON_TEST_v1: HI EWMA alpha 2/25, damage multiplier
clamp(1 + 2 * smoothed_HI, 1, 3), and one hour of damage per RUL update.
Temperature life is 100,000 hours below 70 C, decreases by 5,500 hours/C through
80 C, then by 3,000 hours/C, with a 1,000-hour lower limit.

RUL updates require successful inference and a valid finite temperature. A due
update waits for the next valid measurement; missed hours are not replayed with
one measurement. After an update, the next interval starts from that measurement.
HI smoothing, like the reference project, runs at the hourly RUL update cadence.

UART6 emits `[AI_RUNTIME]` with MSE, prediction (0 normal / 1 anomaly), HI,
temperature validity, RUL percent, and whether the state was saved. The existing
Wi-SUN MSE/prediction fields continue to be populated. Compact SNAP V3 sends RUL
at bytes 45..46 as described below; HI is not a packet field.

## RUL availability and receiver integration (2026-09-10)

Transmission now uses `AI_Runtime_GetRULPercent()` rather than the raw damage
getter. It returns NaN when initialization/recovery is invalid, no hourly update
has completed, the latest attempted AI measurement has invalid input or failed
inference, or an hourly update is due but has not completed. An invalid temperature
blocks RUL even when AI succeeds. Missing FFT or failed voltage/current acquisition
also invalidates the attempted AI/RUL measurement. A valid recovered state with
completed updates can be reported after reboot, until a measurement fails or an
update becomes due. Invalid measurements never erase accumulated damage; the next
valid measurement restores availability (and performs a due update).

The initial zero-damage recovery record is still saved, but it is not reported as
100% before the first successful hourly calculation. A failed Flash save retains
the valid calculated RAM result and retries; it does not itself invalidate RUL.

Compact SNAP V3 remains 47 bytes. Bytes 45..46 are unsigned little endian:

| Wire value | Meaning |
| --- | --- |
| 0..10000 | Valid RUL, divide by 100 for percent |
| 65535 (`FF FF`) | RUL unavailable |
| 10001..65534 | Invalid/reserved; receiver must not display as a percentage |

The scaler maps NaN, infinity and out-of-range percentages to 65535. The actual
packet encoder preserves this sentinel instead of clamping it to 10000. Valid
percentages are truncated to two decimal places before transmission:
99.999% becomes 99.99% (9999), and 98.765% becomes 98.76% (9876).
Exactly 100% remains 100.00% (10000).

Receiver logic must check the raw integer BEFORE scaling or clamping. For JSON,
represent unavailable as `null` and optionally `rul_valid: false`:

```python
raw = int.from_bytes(body[45:47], "little") if len(body) == 47 else None
rul_valid = raw is not None and 0 <= raw <= 10000
rul_percent = raw / 100.0 if rul_valid else None
```

Without receiver support, a decoder that blindly divides may display 655.35%.
The receiver/JSON service has not been modified in this device workspace.

## SMPS threshold configuration

`ON_TEST_v1` has one default threshold plus an in-memory setter, with no
capacity-specific table or terminal-to-capacity mapping. Its `ai_config.h` records
10,000 normal samples with P99.9 = 0.008074203, while the configured threshold is
0.011325965. The exact calibration for that difference has not been located.
The default is preserved for the supplied 100 W model, not newly validated.

Build settings in `Core/Inc/signal/ai_config.h` now pair:

- `AI_SMPS_RATED_WATTS`: default 100; explicit rating for this firmware image.
- `AI_SMPS_MSE_THRESHOLD`: default 0.011325965 only when rating is 100.

For a 125 W build, define `AI_SMPS_RATED_WATTS=125` and define
`AI_SMPS_MSE_THRESHOLD` to the validated MSE threshold for the intended model and
preprocessing. Apply both to all translation units. A non-100 W build without an
explicit threshold fails compilation; a nonfinite/nonpositive threshold causes
inference to return invalid. Boot logs `[AI_CONFIG]` with the selected pair.

No 125 W threshold has been invented or deployed. This is build-time selection,
not automatic detection, a MID=25 override, or a gateway configuration command.
An unchanged default firmware on a 125 W device still uses the 100 W setting.
Supporting a single image with per-device selection requires an agreed device
configuration/protocol and validated profiles. Merely changing the threshold does
not validate the 100 W model/scaler or the fixed HI/RUL calibration on 125 W.

## Flash policy

| Address | Size | Purpose |
| --- | --- | --- |
| 0x081F8000 | 8 KB | RUL journal sector 124 (bank 2) |
| 0x081FA000 | 8 KB | RUL journal sector 125 (bank 2) |
| 0x081FC000 | 8 KB | Existing AI interval settings (unchanged) |
| 0x081FE000 | 8 KB | Existing node/MID settings (unchanged) |

Both linker scripts reserve these areas. RUL storage assumes the project's
existing unswapped bank mapping and refuses operations when banks are swapped.
No loadable firmware section occupies RULCFG; ordinary application programming
must retain it. A mass erase or a debugger/download setting that erases all Flash
will delete the saved state.

- An erased journal starts at 100% and writes an initial recovery record once.
- Each completed hourly update immediately saves `cumulative_damage`, `hi_smooth`
  and `sample_count`. Boot restores the newest valid record, including smoothing
  state; it does not reset saved life to 100%.
- A 32-byte record contains the state, sequence, version, magic and CRC32.
  Its commit quadword is programmed last, then the full record is read back.
- Records append into 256 slots per sector. When full, the other sector is erased
  and written while the latest record remains intact in the current sector.
- Torn or unreadable records are skipped. The NMI hook handles only an ECC fault
  at the exact RUL data quadword currently being read; unrelated NMI faults retain
  the existing fatal behavior. This follows the STM32H5 distinction between
  correctable ECC errors and double-error NMI described in
  [ST AN5342](https://www.st.com/resource/en/application_note/an5342--how-to-use-error-correction-code-ecc-management-for-internal-memories-protection-on-stm32-mcus-stmicroelectronics.pdf).
- Failed saves retain RAM state and retry from the main loop after 60 seconds.
- A nonempty journal with no valid record reports RUL unavailable (`nan`) and
  continues AI detection without silently restoring life to 100%.
- The current partial hour and powered-off elapsed time are not persisted or
  charged. Each reboot starts a new one-hour timer from the recovered state.
- Saving on a gateway OFF command or on a percent-change threshold is not enabled;
  the chosen automatic policy is after each hourly update. `AI_Runtime_SaveState()`
  can flush an outstanding completed update from main-loop context.

## Files and validation

- `Inc/signal`, `Src/signal`: `ai_inference`, `ai_runtime`, `rul`.
- `Inc/services`, `Src/services`: `rul_storage` journal and CRC validation.
- `Inc/platform`, `Src/platform`: `rul_flash` HAL access and narrow ECC recovery.
- `main.c`, `stm32h5xx_it.c`, `app_x-cube-ai.c/h`, `ai_config.h`, both linker
  scripts and CubeIDE include-path priority are updated.
- Older `run_inference` / `AI_Run_FromRaw` calls delegate to the same inference
  module so there is a single set of preprocessing, MSE and threshold logic.

Run `python tests/test_hi_rul.py --cc <native-gcc>` for host tests against the
production calculation, inference and journal sources. The fake hardware injects
torn writes at each record byte, unreadable records and interrupted sector erase.
Tests also cover CRC fallback, sector rotation, reboot restore, invalid sensor
input, temperature gating, save retries and tick wraparound. The 2026-09-10 tests
include the production SNAP encoder, RUL sentinel propagation/recovery, and
build-time threshold selection using synthetic thresholds (not calibration data).

Debug was fully rebuilt and linked successfully. Host tests simulate the Flash
medium and neural-network output; they do not validate actual Flash electrical
behavior, NMI timing or model accuracy. Board validation remains: check INIT/SAVE
logs, run through a real hourly update, power-cycle and compare the recovered
state, then test interrupted writes/sector rotation with controlled power cuts.

Pre-change files: `Debug/hi-rul-backup-20260909-121322`.
Build log: `Debug/hi-rul-build.log`.

2026-09-10 exception/configuration build log: `Debug/rul-exception-build.log`.
The firmware was compiled and linked; no board flashing or receiver end-to-end
test was performed for this update.
