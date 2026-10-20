# SVEA-PX4-Autopilot-MCU-Peripheral

ESP32-C6 firmware for a SVEA peripheral MCU that publishes PX4-uORB-aligned data over UART.

The current implemented example module is `encoder` (publishes wheel topics).

## Encoder Operation

- UART: 115200 baud, 8N1; both topics publish every 20 ms (nominally 50 Hz).
- `ENCODER_EMULATION_ENABLE=0`: real GPIO inputs, including at rest. No automatic emulation.
- `ENCODER_EMULATION_ENABLE=1`: synthetic test data; GPIO interrupts are disabled.

| Encoder | Upper A | Lower B |
| --- | --- | --- |
| Left | GPIO22 (D4) | GPIO23 (D5) |
| Right | GPIO2 (D2) | GPIO21 (D3) |

Two quadrature signals per wheel are required. GPIO23 is left B, not the right-wheel input used by the older single-channel firmware.

The *SVEA Main Platform* assembly guide (page 17) shows the same D-pin wiring on an ESP32-S3. Its GPIO numbers differ from the [XIAO ESP32-C6 pinout](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/). Pages 13 and 15 show different disc slot patterns without specifying the slot count.

`ENCODER_TICKS_PER_REV` is decoded counts per wheel revolution, including gearing. The current value is 80; a 40-cycle encoder with 4× decoding produces 160. Measure before changing it. Verify `ENCODER_WHEEL_DIAM_M` (0.115), `ENCODER_SPEED_SCALE` (0.93), and direction inversion against the car.

At rest, distances/angles stay constant and speeds are zero. Turning one wheel changes only that wheel's data. `wheel_encoders` orders right before left; `wheel_distance` has named left/right fields.

PX4 setup and verification: [Peripheral MCU Bridge](https://github.com/nilskiefer/SVEA-PX4-Autopilot/blob/main/docs/en/svea/peripheral-mcu-bridge.md).

Host regression checks (Python 3 and a C compiler):

```bash
python3 -m unittest discover -s tests -v
```

## Architecture

The project is intentionally split into layers:

- `main/core/`
  - Runtime context and shared counters/locks.
  - `peripheral_context.[ch]`
- `main/transport/`
  - Raw UART setup and byte writes only.
  - `peripheral_uart.[ch]`
- `main/protocol/`
  - Frame format, CRC, and topic-id multiplexing.
  - `peripheral_protocol.[ch]`
- `main/topics/`
  - uORB-aligned topic payload APIs (typed publish helpers).
  - `peripheral_topics.[ch]`
- `main/modules/`
  - Feature modules that produce data.
  - `encoder/encoder_module.[ch]`
- `main/main.c`
  - Boot, wiring, and module startup.

This keeps modules independent from framing/UART details.

## Current Wire Protocol (compatible with PX4 `svea_peripheral_mcu`)

Frame bytes:

- `magic0` = `0x53` (`'S'`)
- `magic1` = `0x45` (`'E'`)
- `version` = `2`
- `topic_id` = `uint8`
- `payload_len` = `uint8`
- `payload[payload_len]`
- `crc16_ccitt` over `[version, topic_id, payload_len, payload...]` (little-endian in frame)

Current topic IDs:

- `1` -> `wheel_distance`
- `2` -> `wheel_encoders`

## Encoder Example (Reference Module)

`modules/encoder/encoder_module.c` shows the expected pattern for adding modules:

1. Collect or synthesize sensor data.
2. Populate a typed payload struct from `topics/peripheral_topics.h`.
3. Publish through `peripheral_topic_publish_*()`.
4. Fail hard on publish errors.

It publishes both:

- `wheel_distance` payload (`sequence`, `time_ms`, left/right distance)
- `wheel_encoders` payload (`sequence`, `time_ms`, wheel speeds/angles)

## How To Add A New Module

1. Create `main/modules/<name>/<name>_module.[ch]`.
2. Add topic payload + publish function to `main/topics/peripheral_topics.[ch]`.
3. Assign a new `topic_id` in `main/protocol/peripheral_protocol.h`.
4. Use `peripheral_protocol_send()` only from topic-layer helpers.
5. Start module from `main/main.c`.
6. Add new source file(s) in `main/CMakeLists.txt`.

Note: This repo is protocol/uORB-aligned on MCU side. PX4 still needs matching decode+publish mapping for any brand-new topic class.

## Build

### Docker Compose

```bash
docker compose run --rm idf zsh -lc "idf.py build"
```

### Set target (first time)

```bash
docker compose run --rm idf zsh -lc "idf.py set-target esp32c6"
```

## Flash

Build first:

```bash
docker compose run --rm idf zsh -lc "idf.py build"
```

Create a venv for python and run (this can be done via `F1` + `Select Interpreter`)
```bash
pip install esptool
```

Then flash from host:

```bash
cd build
python -m esptool --chip esp32c6 --port /dev/serial/by-id/*Espressif* --baud 460800 write-flash @flash_args
cd ..
```

## Monitor 
```bash
pio device monitor --port /dev/serial/by-id/*Espressif* --baud 115200
```

## Wiring (PX4 Clicker4 STM32F7)

- PX4 `PB6` (`USART1_TX`, `/dev/ttyS0`) -> ESP RX (`D7` / GPIO17)
- PX4 `PB7` (`USART1_RX`, `/dev/ttyS0`) <- ESP TX (`D6` / GPIO16)
- Common GND
- 3.3V logic only

Do not use `PA2/PA3` for this peripheral link (NSH console path).

## Configuration

See `main/svea_config.h`:

- UART config (port/pins/baud/buffer sizes)
- Encoder GPIO and kinematic constants
- `ENCODER_EMULATION_ENABLE` (synthetic encoder ticks for bring-up)
