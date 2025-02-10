/*
 * Copyright (c) 2021 Valentin Milea <valentin.milea@gmail.com>
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <hardware/i2c.h>
#include <pico/i2c_slave.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

#define PARAM_ASSERTIONS_ENABLED_HARDWARE_I2C 0  // required to allow sending to address 0

static const uint I2C_SLAVE_ADDRESS = 0x17; // 0x00;
static const uint I2C_BAUDRATE = 100000; // 100 kHz

#ifdef i2c_default
// For this example, we run both the master and slave from the same board.
// You'll need to wire pin GP4 to GP6 (SDA), and pin GP5 to GP7 (SCL).
static const uint I2C_SLAVE_SDA_PIN = PICO_DEFAULT_I2C_SDA_PIN; // 4
static const uint I2C_SLAVE_SCL_PIN = PICO_DEFAULT_I2C_SCL_PIN; // 5
static const uint I2C_MASTER_SDA_PIN = 6;
static const uint I2C_MASTER_SCL_PIN = 7;

// The slave implements a 256 byte memory. To write a series of bytes, the master first
// writes the memory address, followed by the data. The address is automatically incremented
// for each byte transferred, looping back to 0 upon reaching the end. Reading is done
// sequentially from the current memory address.
static struct
{
    uint8_t mem[256];
    uint8_t mem_address;
    bool mem_address_written;
} context;

void enable_general_call_mask(i2c_inst_t *i2c) {
    printf("mask1: 0x%08x\n", i2c->hw->intr_mask);
    i2c->hw->intr_mask |= I2C_IC_INTR_MASK_M_GEN_CALL_BITS;
    // hw_write_masked(
    //     &i2c->hw->intr_mask,
    //     I2C_IC_INTR_MASK_M_GEN_CALL_VALUE_ENABLED << I2C_IC_INTR_MASK_M_GEN_CALL_LSB,
    //     I2C_IC_INTR_MASK_M_GEN_CALL_BITS
    // );
    printf("mask2: 0x%08x\n", i2c->hw->intr_mask);
}

void enable_general_call_ack(i2c_inst_t *i2c) {
    hw_write_masked(
        &i2c->hw->ack_general_call,
        I2C_IC_ACK_GENERAL_CALL_ACK_GEN_CALL_VALUE_ENABLED << I2C_IC_ACK_GENERAL_CALL_ACK_GEN_CALL_LSB,
        I2C_IC_ACK_GENERAL_CALL_ACK_GEN_CALL_BITS
    );
}

void set_i2c_rx_thres(i2c_inst_t *i2c, uint8_t size) {
    i2c->hw->rx_tl = size;
    printf("set to: %d\n", i2c->hw->rx_tl);
}

bool received_general_call(i2c_inst_t *i2c) {
    return i2c->hw->raw_intr_stat & I2C_IC_INTR_STAT_R_GEN_CALL_BITS;
}

bool received_general_call_clear(i2c_inst_t *i2c) {
    bool is_general_call = (bool)i2c->hw->clr_gen_call;
    printf("read and cleared: 0x%08x\t", is_general_call);
    return is_general_call;
}

bool stop_bit_detected(i2c_inst_t *i2c) {
    return i2c->hw->raw_intr_stat & I2C_IC_INTR_STAT_R_STOP_DET_BITS;
}

bool stop_bit_detected_clear(i2c_inst_t *i2c) {
    return i2c->hw->clr_stop_det;
}
// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    printf("received event: %d\t", event);
    static int step = 0;
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        printf("RECEIVE\n");
        if (!context.mem_address_written) {
            printf("got raw: 0x%08x\t", i2c->hw->raw_intr_stat);
            printf("got: 0x%08x\t", i2c->hw->intr_stat);
            received_general_call_clear(i2c);
            stop_bit_detected_clear(i2c);
            printf("got raw: 0x%08x\t", i2c->hw->raw_intr_stat);
            printf("got: 0x%08x\t", i2c->hw->intr_stat);
            // writes always start with the memory address
            context.mem_address = i2c_read_byte_raw(i2c);
            context.mem_address_written = true;
            printf("got raw: 0x%08x\t", i2c->hw->raw_intr_stat);
            printf("got: 0x%08x\t", i2c->hw->intr_stat);
            printf("received byte: 0x%02x\n", context.mem_address);
        } else {
            // save into memory
            context.mem[context.mem_address] = i2c_read_byte_raw(i2c);
            context.mem_address++;
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        // load from memory
        i2c_write_byte_raw(i2c, 'a' + step);
        step++;
        printf("REQUEST\n");
        printf("got raw: 0x%08x\t", i2c->hw->raw_intr_stat);
        printf("got: 0x%08x\t", i2c->hw->intr_stat);
        received_general_call_clear(i2c);
        stop_bit_detected_clear(i2c);
        printf("got raw: 0x%08x\t", i2c->hw->raw_intr_stat);
        printf("got: 0x%08x\n", i2c->hw->intr_stat);
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        context.mem_address_written = false;
        printf("DONE\n");
        break;
    default:
        break;
    }
}

static void setup_slave() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
    enable_general_call_mask(i2c0);
    enable_general_call_ack(i2c0);
    // set_i2c_rx_thres(i2c0, 255);  // max is actually 15 (+1=16)
}

static void run_master() {
    gpio_init(I2C_MASTER_SDA_PIN);
    gpio_set_function(I2C_MASTER_SDA_PIN, GPIO_FUNC_I2C);
    // pull-ups are already active on slave side, this is just a fail-safe in case the wiring is faulty
    gpio_pull_up(I2C_MASTER_SDA_PIN);

    gpio_init(I2C_MASTER_SCL_PIN);
    gpio_set_function(I2C_MASTER_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_MASTER_SCL_PIN);

    i2c_init(i2c1, I2C_BAUDRATE);

    for (uint8_t mem_address = 0;; mem_address = (mem_address + 32) % 256) {
        char msg[32];
        snprintf(msg, sizeof(msg), "Hello, I2C slave! - 0x%02X", mem_address);
        uint8_t msg_len = strlen(msg);

        uint8_t buf[32];
        buf[0] = 0x66;
        // memcpy(buf + 1, msg, msg_len);
        // uint8_t data[1];
        // data[0] = "a";
        // // write message at mem_address
        // printf("Write at 0x%02X: '%s'\n", mem_address, msg);
        // int count = i2c_write_blocking(i2c1, 0/*I2C_SLAVE_ADDRESS*/, data, sizeof(data)/* + msg_len*/, false);  // 
        // if (count < 0) {
        //     puts("Couldn't write to slave, please check your wiring!");
        //     return;
        // }
        // hard_assert(count == 1 + msg_len);

        // seek to mem_address
        int count = i2c_write_blocking(i2c1, I2C_SLAVE_ADDRESS, buf, 1, true);
        hard_assert(count == 1);
        // partial read
        uint8_t split = 5;
        count = i2c_read_blocking(i2c1, I2C_SLAVE_ADDRESS, buf, split, true);
        hard_assert(count == split);
        buf[count] = '\0';
        printf("Read  at 0x%02X: '%s'\n", mem_address, buf);
        // hard_assert(memcmp(buf, msg, split) == 0);
        // read the remaining bytes, continuing from last address
        // count = i2c_read_blocking(i2c1, I2C_SLAVE_ADDRESS, buf, msg_len - split, false);
        // hard_assert(count == msg_len - split);
        // buf[count] = '\0';
        // printf("Read  at 0x%02X: '%s'\n", mem_address + split, buf);
        // hard_assert(memcmp(buf, msg + split, msg_len - split) == 0);

        // puts("");
        sleep_ms(2000);
    }
}
#endif

int main() {
    stdio_init_all();
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c / slave_mem_i2c example requires a board with I2C pins
    puts("Default I2C pins were not defined");
    return 0;
#else
    sleep_ms(3000);
    puts("\nI2C slave example");

    setup_slave();
    run_master();
    for (;;) {}
#endif
}
