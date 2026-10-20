#pragma once

#include "driver/uart.h"

/* ESP32-C6 UART connected to PX4 USART1 (/dev/ttyS0):
 *   PX4 PB6 (USART1_TX) -> ESP RX
 *   PX4 PB7 (USART1_RX) <- ESP TX
 */
#define BRIDGE_UART_NUM UART_NUM_1
#define BRIDGE_UART_TX_GPIO 16 /* XIAO D6 */
#define BRIDGE_UART_RX_GPIO 17 /* XIAO D7 */
#define BRIDGE_UART_BAUDRATE 115200
#define BRIDGE_UART_RX_BUF 4096
#define BRIDGE_UART_TX_BUF 4096

/* Quadrature encoders: two photo-interrupters (A/B) per wheel, ~90 deg apart.
 * A channels keep the original pins; B channels are new -- update to match
 * your actual wiring.
 */
#define ENCODER_LEFT_A_GPIO 22 /* XIAO D4 */
#define ENCODER_LEFT_B_GPIO 23 /* XIAO D5 */
#define ENCODER_RIGHT_A_GPIO 2  /* XIAO D2 */
#define ENCODER_RIGHT_B_GPIO 21  /* XIAO D3 */

/* If forward driving produces negative speed on a wheel, set its invert to 1
 * instead of swapping wires.   
 */
#define ENCODER_LEFT_INVERT 0
#define ENCODER_RIGHT_INVERT 0

/* Decoded counts per wheel revolution. Measure after wiring A/B.
 * 4x decoding of a 40-cycle/revolution encoder gives 160 counts.
 */
#define ENCODER_TICKS_PER_REV 80.0f
#define ENCODER_WHEEL_DIAM_M 0.115f
#define ENCODER_WHEELBASE_M 0.32f
#define ENCODER_PUBLISH_MS 20
#define ENCODER_SPEED_SCALE 0.93f
#define ENCODER_MIN_SPEED_MPS 0.01f
#define ENCODER_MAX_SPEED_MPS 5.0f

/* 0: real GPIO inputs, including at rest. 1: synthetic test data only. */
#define ENCODER_EMULATION_ENABLE 0
/* Simple band-pass plausibility filter for encoder-derived wheel speed. */
#define ENCODER_MIN_SPEED_FILTER_ENABLE 1
/* Log raw encoder GPIO levels and counters once per second. */
#define ENCODER_DEBUG_LOG_ENABLE 1
#define ENCODER_DEBUG_LOG_MS 1000
/* Emulation motion profile is intentionally hardcoded in encoder_module.c for
 * communication/integration bringup validation.
 */