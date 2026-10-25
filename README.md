H/W Black_Driver (STM32F103-RBx-CTX)
F/W STM32F103-RBX (using STM32F103-CTx)

## Model binary (Windows / STM32CubeIDE)

Set `SYSTEM_MODEL_NAME` in `Core/Inc/system_setup.h` (default:
`"STM32_SB100D_BJ"`). Use ASCII letters, digits, underscores and hyphens.

Both Debug and Release run `tools/export-model-bin.ps1` after building.
The script converts the linked ELF with the toolchain's
`arm-none-eabi-objcopy` and writes `<model>_<yyyyMMdd>.bin` beside the ELF,
for example `Debug/STM32_SB100D_BJ_20261025.bin`.
The date is the ELF's local last-write date (the link/build date), not the
export date of an old ELF. Rebuild after changing the model name.
Same-model, same-date exports replace the corresponding named binary;
other dated binaries are retained.

Existing ELF, HEX and default BIN outputs are unchanged. Windows PowerShell
and `arm-none-eabi-objcopy` must be available in the build environment.
Invalid model names, missing tools and conversion failures fail the
post-build step. Reload the project if CubeIDE is open while its build
configuration is changed externally.

## PCB selection

`PCB_TYPE` in `Core/Inc/system_setup.h` defaults to `DM_DRIVER`.
This board does not initialize or use the FND. PC0/PC1/PC2 are C+/C-/D+
pull-up inputs instead of FND outputs; PC3 remains the D- pull-up input
with the reference project's rising-edge EXTI configuration.
The pin mapping matches `ycb_dmdriver_103rbtx`.
Select `FND_DRIVER` to retain the previous FND board behavior.

Both PCB types publish standard CAN ID `0x5B9`, DLC 8, every 100 ms
while the drive CAN link is active, immediately after `0x5B8`.
This replaces the previous `0x5A1` frame; `0x5A0` remains unchanged.

| Byte | DM_DRIVER (1) | FND_DRIVER (2) |
|---|---|---|
| 0 | bits 0..7: A+/A-/B+/B-/C+/C-/D+/D- | bits 0..3: A+/A-/B+/B-, bit 7: D-; bits 4..6 zero |
| 1 | bits 0..4: RY3..RY7, bit 5 zero, bit 6 PB0, bit 7 PB1 | bits 0..5: RY3..RY8; bits 6..7 zero |
| 2 | ADC1 (`ladcValue[0]`), 0..255 | zero |
| 3 | ADC2 (`ladcValue[1]`), 0..255 | zero |
| 4 | 1 | 2 |
| 5..7 | reserved, zero | reserved, zero |

Input bits are LOW-active. Relay/PB0/PB1 bits report the output latch
(HIGH = 1), not logical brake engagement. ADC values are clamped to
0..4095 then shifted right by 4 for 256 levels.
FND_DRIVER initializes RY8 on PD0 LOW as a push-pull output and enables
PD0/PD1 remapping for the LQFP64 board using the existing HSI clock.
DM_DRIVER does not initialize or read PD0. No RY8 control command is added.
Inputs are reported only: they do not stop motors or override CAN commands.
Existing `0x5A0` and `0x5B8` layouts are unchanged.
For compatibility with `ycb_samboo_auto_lcd`, DM_DRIVER also reports C+
(collision) in `0x5B8` byte 7 bit 5 and C- (hose) in bit 6, LOW = active.
These populate the existing two-bit `limitSwitch` field regardless of the
drive command's `ioMsg`. Both active inputs produce `limitSwitch = 3`;
neither active produces 0. The LCD continues to make the stop decision;
no LCD source change is required for those two limits. `0x5B9` uses the same
input snapshot. The LCD needs a new `0x5B9` parser to use the extended data;
this change updates the driver only.

## Fixed model configuration

`SYSTEM_MODEL_NAME` also selects the firmware configuration. Currently only
`STM32_SB100D_BJ` is supported; other names fail a compile-time assertion.
Its header-only configuration class is
`class/STM32_SB100D_BJ/STM32_SB100D_BJ.h`, selected as `SystemModel` by
`Core/Inc/system_setup.h`. Add an explicit model selection and class before
using another name; renaming the BIN alone does not add model support.

This model is CAN-input-only from boot, without a PA8 strap or local fallback.
Existing drive IDs (0x701, 0x702, 0x7C1) and their command formats are retained.
CAN byte2 overrides the brake delay: 0 selects the model default, otherwise
raw x 5 ms clamped to 50..1275 ms. Byte3 selects voltage: 0 keeps the current
value, 24/48 select volts; other values reject the drive command and request
a stop. A voltage change requires a zero-speed command, an empty command
queue, stopped motors, no active fault and the applied mechanical brake.
Rejected changes request a stop and are logged; retry after stopping.
The model groups PID settings in the `PID_CONTROL_CONFIG` typedef:
`SystemModel::PID` contains `disable`, `Kp`, `Ki` and `Kd` in that order.
Set the first initializer to `true` to disable PID for the model
(default `false`, preserving existing behavior). The existing per-frame
PI disable bit remains supported. Effective disable is model disable OR
CAN disable: CAN cannot re-enable PID when the model disables it.
Disabling PID clears its correction/integrators but retains PWM ramps,
brake control, speed measurement and protection logic.

Fixed values: acceleration 2000 ms, deceleration 3000 ms, brake delay 300 ms,
Kp 0.8, Ki 0.1, Kd 0 (PI only), 24 V nominal with 21/30 V diagnostic limits,
120 A Hall-current limit, and normal polarity for both motors.
Voltage is not auto-detected; the master may explicitly select 48 V.
Selection updates BEMF nominal voltage, voltage diagnostic limits
(24 V: 21/30 V; 48 V: 42/60 V) and enabled shunt watchdog thresholds without
clearing a latched trip.
Existing short-circuit and thermal protections are retained. Verify board,
wiring, sensor and motor ratings before using the 120 A setting.
Model defaults are compiled into firmware, not loaded from NVS.
Brake/voltage overrides are RAM-only; reboot restores defaults.
CAN timeout restores the default brake delay but retains the selected voltage.
