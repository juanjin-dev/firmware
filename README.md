# meshkit modem firmware

This is a modified version of [meshtastic/firmware](https://github.com/meshtastic/firmware).

It builds a single target, `rak3172`, as an industrial LoRa mesh modem driven by a host MCU over
UART. Sensor drivers, fieldbus protocols and application logic live on the host; this firmware owns
the radio and nothing else.

Traffic on the air is unchanged and remains readable by existing Meshtastic gateways, apps and
tooling.

## What was changed

- Modules with no meaning on a headless industrial modem are excluded.
- A private UART wire protocol is served on UART1. The host sends values; the firmware builds the
  mesh messages.
- CI builds and tests only this target.

See [variants/stm32/rak3172/platformio.ini](variants/stm32/rak3172/platformio.ini) for the exact
build configuration.

## Protocol

The UART protocol is specified in [meshkit](https://github.com/juanjin-dev/meshkit), under
Apache-2.0, along with the host libraries. Implement from that specification, not from this source.

## Building

```bash
platformio run -e rak3172
platformio test -e coverage -f test_modem_frame
```

Requires [PlatformIO](https://platformio.org/). Output is a single application image in
`.pio/build/rak3172/`; the STM32WLE5 bootloader lives in ROM and is not part of the build.

## Reflashing

Devices are **not** shipped at RDP Level 2. The ROM bootloader is reachable over UART with BOOT0
asserted, and over SWD, and it does not verify manufacturer signatures - so any image, including one
you have modified, can be installed:

```bash
# Assert BOOT0, pulse NRST to enter the ROM bootloader, then:
stm32flash -w .pio/build/rak3172/firmware-rak3172-*.bin -v -g 0x08000000 /dev/ttyUSB0

# or over SWD with a CMSIS-DAP adapter
platformio run -e rak3172 -t upload
```

## Licence

GPL-3.0, inherited from upstream. See [LICENSE](LICENSE).

Bug reports and features that are not specific to this modem belong upstream at
[meshtastic/firmware](https://github.com/meshtastic/firmware).

Meshtastic® is a registered trademark of Meshtastic LLC. See the
[trademark policy](https://meshtastic.org/docs/legal/). This project is not affiliated with or
endorsed by Meshtastic LLC.
