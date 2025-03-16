// Copyright (c) 2021 Michael Stoops. All rights reserved.
// Portions copyright (c) 2021 Raspberry Pi (Trading) Ltd.
// 
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the 
// following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
//    disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
//    following disclaimer in the documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
//    products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 
// USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// SPDX-License-Identifier: BSD-3-Clause
//
// Example of an SPI bus slave using the PL022 SPI interface

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

#define BUF_LEN         0x100
#define BROADCAST_MSG 0x00
#define SINGLE_MSG 0x01

#define LED_PIN PICO_DEFAULT_LED_PIN
#define CS_PIN PICO_DEFAULT_SPI_CSN_PIN
#define MISO_PIN PICO_DEFAULT_SPI_TX_PIN
#define MOSI_PIN PICO_DEFAULT_SPI_RX_PIN
#define SCK_PIN PICO_DEFAULT_SPI_SCK_PIN

void printbuf(uint8_t buf[], size_t len) {
    size_t i;
    for (i = 0; i < len; ++i) {
        if (i % 16 == 15)
            printf("%02x\n", buf[i]);
        else
            printf("%02x ", buf[i]);
    }

    // append trailing newline if there isn't one
    if (i % 16) {
        putchar('\n');
    }
}

int pico_led_init(void) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    return PICO_OK;
}

void pico_set_led(bool led_on) {
    gpio_put(LED_PIN, led_on);
}

void spi_miso_highz(bool high_z) {
    // printf("high z: %d; current: %04X\n", high_z, io_bank0_hw->io[MISO_PIN].ctrl);
    gpio_set_oeover(MISO_PIN, high_z ? 0x2 : 0x0);
    // printf("new: %04X\n", io_bank0_hw->io[MISO_PIN].ctrl);
}


int main() {
    // Enable UART so we can print
    stdio_init_all();
    pico_led_init();
#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi/spi_slave example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#else

    printf("SPI slave example\n");

    // Enable SPI 0 at 1 MHz and connect to GPIOs
    spi_init(spi_default, 250000);
    spi_set_slave(spi_default, true);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);
    // gpio_init(CS_PIN);
    // gpio_set_dir(CS_PIN, GPIO_IN);
    // gpio_pull_up(CS_PIN);
    gpio_set_function(CS_PIN, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, GPIO_FUNC_SPI));
    spi_set_format(spi_default, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);  // TODO what is this line actually doing? Because it fixes everything
    sleep_ms(3000);
    // spi_miso_highz(true);
    // spi_miso_highz(false);
    spi_miso_highz(true);

    uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN] = {0};

    // Initialize output buffer
    for (size_t i = 0; i < BUF_LEN; ++i) {
        // bit-inverted from i. The values should be: {0xff, 0xfe, 0xfd...}
        out_buf[i] = ~i;
    }

    printf("SPI slave says: When reading from MOSI, the following buffer will be written to MISO:\n");
    printbuf(out_buf, BUF_LEN);

    bool led_state = false;
    
    char in_buffer[16];
    uint8_t msg_header[2];
    while (true) {
        led_state = !led_state;
        pico_set_led(led_state);

        bool is_active = !gpio_get(CS_PIN);
        if (is_active) {
            printf("%d", is_active);
            // spi_read_blocking(spi_default, 0x00, msg_header, 2);
            spi_write_read_blocking(spi_default, msg_header, msg_header, 2);
            uint8_t msg_type = msg_header[0];
            uint8_t msg_len = msg_header[1];
            // printf("msg_type: %d, ", msg_type);
            // printf("msg_len: %d\n", msg_len);
            switch (msg_type) {
                case BROADCAST_MSG:
                    printf("Broadcast\n");
                    spi_read_blocking(spi_default, 0x48, in_buffer, msg_len);
                    break;
                case SINGLE_MSG:
                    // printf("For me\n");
                    gpio_put(MISO_PIN, true);
                    spi_miso_highz(false);
                    spi_read_blocking(spi_default, 0xFF, in_buffer, msg_len);
                    spi_write_blocking(spi_default, "myresponse\0", 11);
                    gpio_put(MISO_PIN, true);
                    spi_miso_highz(true);
                    break;
                default:
                    printf("Error\n");
                    break;
            }
            in_buffer[15] = '\0';
            printf("Message: %s\n", in_buffer);
        }
        sleep_ms(10);
        // spi_read_blocking(spi_default, 0x00)
    }

    for (size_t i = 0; ; ++i) {
        led_state = !led_state;
        pico_set_led(led_state);
        // spi_read_blocking(spi_default, 0x21, in_buf, 1);  // consider changing 0 to 0xff. Is this actually writing?
        // Write the output buffer to MISO, and at the same time read from MOSI.
        spi_write_read_blocking(spi_default, out_buf, in_buf, 1);

        // Write to stdio whatever came in on the MOSI line.
        printf("SPI slave says: read page %d from the MOSI line:\n", i);
        printbuf(in_buf, BUF_LEN);
        out_buf[0] = in_buf[0];
        sleep_ms(100);
    }
#endif
}
