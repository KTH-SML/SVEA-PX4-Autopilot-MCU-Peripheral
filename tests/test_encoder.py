"""Run encoder mode, quadrature and UART framing checks without ESP hardware.

Usage: python3 -m unittest discover -s tests
"""
import binascii
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
STUB = r'''
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <setjmp.h>
typedef int portMUX_TYPE;
typedef void *SemaphoreHandle_t;
typedef int BaseType_t;
typedef int esp_err_t;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define portENTER_CRITICAL_ISR(x) ((void)(x))
#define portEXIT_CRITICAL_ISR(x) ((void)(x))
#define pdMS_TO_TICKS(x) (x)
#define pdPASS 1
#define ESP_OK 0
#define IRAM_ATTR
#define DRAM_ATTR
#define ESP_INTR_FLAG_IRAM 1
#define GPIO_MODE_INPUT 1
#define GPIO_PULLUP_ENABLE 1
#define GPIO_PULLDOWN_DISABLE 0
#define GPIO_INTR_ANYEDGE 1
#define UART_NUM_1 1
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
typedef struct {
    uint64_t pin_bit_mask;
    int mode, pull_up_en, pull_down_en, intr_type;
} gpio_config_t;
extern int levels[32], handlers, configurations;
extern uint32_t now_ms;
extern int iterations;
extern jmp_buf finished;
static int GPIO;
static inline int gpio_config(const gpio_config_t *cfg) { configurations++; return ESP_OK; }
static inline int gpio_get_level(int pin) { return levels[pin]; }
static inline int gpio_ll_get_level(const int *gpio, int pin) { return levels[pin]; }
static inline int gpio_install_isr_service(int flags) { return ESP_OK; }
static inline int gpio_isr_handler_add(int pin, void (*fn)(void *), void *arg) {
    handlers++; return ESP_OK;
}
static inline void vTaskDelay(int ticks) {
    if (iterations++ == 1) { longjmp(finished, 1); }
    now_ms += ticks;
}
static inline int xTaskCreate(void (*fn)(void *), const char *name, int size,
                             void *arg, int priority, void *handle) { return pdPASS; }
'''
HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include "stub.h"
#include "main/modules/encoder/encoder_module.c"
#include "main/protocol/peripheral_protocol.c"
#include "main/topics/peripheral_topics.c"
int levels[32], handlers, configurations, iterations;
uint32_t now_ms;
jmp_buf finished;
uint32_t peripheral_now_ms(void) { return now_ms; }
int peripheral_uart_write(peripheral_context_t *ctx, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) { printf("%02x", data[i]); }
    puts("");
    return (int)len;
}
static void publish_once(encoder_module_t *module) {
    iterations = 0;
    if (setjmp(finished) == 0) { encoder_publish_task(module); }
}
static void cycle(int a, int b, int wheel, const int *states) {
    for (int i = 0; i < 4; i++) {
        levels[a] = states[i] >> 1;
        levels[b] = states[i] & 1;
        encoder_gpio_isr((void *)(uintptr_t)wheel);
    }
}
int main(void) {
    peripheral_context_t ctx = {0};
    encoder_module_t module;
    encoder_module_init(&module, &ctx);
    assert(module.use_emulation == (ENCODER_EMULATION_ENABLE != 0));
    if (ENCODER_EMULATION_ENABLE) {
        assert(handlers == 0 && configurations == 0);
        now_ms = 1000;
        publish_once(&module);
    } else {
        assert(handlers == 4 && configurations == 1);
        now_ms = 1000; /* Still at rest well beyond the old 300 ms probe. */
        publish_once(&module);
        assert(module.left_ticks == 0 && module.right_ticks == 0);
        const int forward[] = {1, 3, 2, 0};
        const int reverse[] = {2, 3, 1, 0};
        cycle(ENCODER_LEFT_A_GPIO, ENCODER_LEFT_B_GPIO, ENCODER_WHEEL_LEFT, forward);
        cycle(ENCODER_RIGHT_A_GPIO, ENCODER_RIGHT_B_GPIO, ENCODER_WHEEL_RIGHT, reverse);
        assert(module.left_ticks == 4 && module.right_ticks == -4);
        publish_once(&module);
        cycle(ENCODER_LEFT_A_GPIO, ENCODER_LEFT_B_GPIO, ENCODER_WHEEL_LEFT, reverse);
        cycle(ENCODER_RIGHT_A_GPIO, ENCODER_RIGHT_B_GPIO, ENCODER_WHEEL_RIGHT, forward);
        assert(module.left_ticks == 0 && module.right_ticks == 0);
        levels[ENCODER_LEFT_A_GPIO] = 1;
        levels[ENCODER_LEFT_B_GPIO] = 1;
        encoder_gpio_isr((void *)(uintptr_t)ENCODER_WHEEL_LEFT);
        assert(module.left_ticks == 0 && module.rejected_left_edges == 1);
    }
    return 0;
}
'''


class EncoderTest(unittest.TestCase):
    def run_firmware(self, emulation):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp / 'stub.h').write_text(STUB)
            headers = ('driver/gpio.h', 'driver/uart.h', 'esp_attr.h', 'esp_err.h',
                       'esp_log.h', 'freertos/FreeRTOS.h', 'freertos/semphr.h',
                       'freertos/task.h', 'hal/gpio_ll.h', 'soc/gpio_struct.h')
            for name in headers:
                path = tmp / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('#include "stub.h"\n')
            config = (ROOT / 'main/svea_config.h').read_text()
            self.assertIn('#define ENCODER_EMULATION_ENABLE 0', config)
            (tmp / 'svea_config.h').write_text(config.replace(
                '#define ENCODER_EMULATION_ENABLE 0',
                f'#define ENCODER_EMULATION_ENABLE {int(emulation)}'))
            (tmp / 'test.c').write_text(HARNESS)
            includes = [tmp, ROOT, ROOT / 'main']
            includes += [ROOT / 'main' / p for p in
                         ('core', 'transport', 'protocol', 'topics', 'modules/encoder')]
            command = [shutil.which('cc'), '-std=gnu11']
            for path in includes:
                command += ['-I', str(path)]
            subprocess.run(command + [str(tmp / 'test.c'), '-lm', '-o', str(tmp / 'test')],
                           check=True, capture_output=True, text=True)
            result = subprocess.run([str(tmp / 'test')], check=True,
                                    capture_output=True, text=True)
        frames = [bytes.fromhex(line) for line in result.stdout.splitlines()]
        self.frames = frames
        payloads = []
        for frame in frames:
            self.assertEqual(frame[:3], b'SE\x02')
            self.assertEqual(len(frame), frame[4] + 7)
            self.assertEqual(int.from_bytes(frame[-2:], 'little'),
                             binascii.crc_hqx(frame[2:-2], 0xffff))
            fmt = '<IIff' if frame[3] == 1 else '<IIffff'
            self.assertEqual(frame[4], struct.calcsize(fmt))
            payloads.append(struct.unpack(fmt, frame[5:-2]))
        return payloads

    def test_real_stationary_and_quadrature(self):
        distance, encoders, moved_distance, moved_encoders = self.run_firmware(False)
        self.assertEqual(distance[2:], (0, 0))
        self.assertEqual(encoders[2:], (0, 0, 0, 0))
        self.assertGreater(moved_distance[2], 0)  # left
        self.assertLess(moved_distance[3], 0)  # right
        self.assertLess(moved_encoders[2], 0)  # right speed
        self.assertGreater(moved_encoders[3], 0)  # left speed
        self.assertLess(moved_encoders[4], 0)  # right angle
        self.assertGreater(moved_encoders[5], 0)  # left angle

    def test_emulation_requires_explicit_opt_in(self):
        distance, encoders = self.run_firmware(True)
        self.assertNotEqual(distance[2:], (0, 0))
        self.assertNotEqual(encoders[2:], (0, 0, 0, 0))


if __name__ == '__main__':
    unittest.main()
