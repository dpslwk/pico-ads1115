/*
 * Copyright (c) 2021-2022 Antonio González
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ads1115.h"

static const int32_t ADS1115_POSITIVE_FULL_SCALE_COUNTS = 0x8000;

static int32_t ads1115_get_full_scale_range_millivolts(ads1115_adc_t *adc) {
    uint16_t pga = adc->config & ADS1115_PGA_MASK;
    switch (pga) {
        case ADS1115_PGA_6_144:
            return 6144;
        case ADS1115_PGA_4_096:
            return 4096;
        case ADS1115_PGA_2_048:
            return 2048;
        case ADS1115_PGA_1_024:
            return 1024;
        case ADS1115_PGA_0_512:
            return 512;
        case ADS1115_PGA_0_256:
            return 256;
        default:
            return 2048;
    }
}

void ads1115_init(i2c_inst_t *i2c_port, uint8_t i2c_addr,
                  ads1115_adc_t *adc) {
    adc->i2c_port = i2c_port;
    adc->i2c_addr = i2c_addr;
    ads1115_read_config(adc);
}

static void ads1115_write_register(ads1115_adc_t *adc, uint8_t reg, uint16_t value) {
    uint8_t src[3];
    src[0] = reg;
    src[1] = (uint8_t)(value >> 8);
    src[2] = (uint8_t)(value & 0xff);
    i2c_write_blocking(adc->i2c_port, adc->i2c_addr, src, 3,
                       false);
}

void ads1115_start_single_conversion(ads1115_adc_t *adc) {
    adc->config |= ADS1115_STATUS_MASK;
    ads1115_write_config(adc);
}

void ads1115_read_adc(uint16_t *adc_value, ads1115_adc_t *adc){
    // If mode is single-shot, set bit 15 to start the conversion.
    if ((adc->config & ADS1115_MODE_MASK) == ADS1115_MODE_SINGLE_SHOT) {
        ads1115_start_single_conversion(adc);

        // Wait until the conversion finishes before reading the value
        do {
            ads1115_read_config(adc);
        } while (adc->config & ADS1115_STATUS_MASK == ADS1115_STATUS_BUSY);
    }

    // Now read the value from last conversion
    ads1115_read_last_conversion(adc_value, adc);
}

void ads1115_read_last_conversion(uint16_t *adc_value, ads1115_adc_t *adc){
    uint8_t dst[2];
    i2c_write_blocking(adc->i2c_port, adc->i2c_addr,
                       &ADS1115_POINTER_CONVERSION, 1, true);
    i2c_read_blocking(adc->i2c_port, adc->i2c_addr, dst, 2,
                      false);
    *adc_value = (dst[0] << 8) | dst[1];
}

float ads1115_raw_to_volts(uint16_t adc_value, ads1115_adc_t *adc) {
    float full_scale_range_volts =
        (float)ads1115_get_full_scale_range_millivolts(adc) / 1000.0f;
    int32_t signed_adc_value = (int16_t)adc_value;
    return ((float)signed_adc_value * full_scale_range_volts) /
           (float)ADS1115_POSITIVE_FULL_SCALE_COUNTS;
}

int16_t ads1115_raw_to_millivolts(uint16_t adc_value, ads1115_adc_t *adc) {
    int32_t full_scale_range_mv = ads1115_get_full_scale_range_millivolts(adc);
    int32_t signed_adc_value = (int16_t)adc_value;
    return (int16_t)((signed_adc_value * full_scale_range_mv) /
                     ADS1115_POSITIVE_FULL_SCALE_COUNTS);
}

void ads1115_read_config(ads1115_adc_t *adc){
    // Default configuration after power up should be 34179.
    // Default config with bit 15 cleared is 1411
    uint8_t dst[2];
    i2c_write_blocking(adc->i2c_port, adc->i2c_addr,
                       &ADS1115_POINTER_CONFIGURATION, 1, true);
    i2c_read_blocking(adc->i2c_port, adc->i2c_addr, dst, 2,
                      false);
    adc->config = (dst[0] << 8) | dst[1];
}

void ads1115_write_config(ads1115_adc_t *adc) {
    uint16_t config_word = adc->config;
    ads1115_write_register(adc, ADS1115_POINTER_CONFIGURATION, config_word);
}

void ads1115_set_input_mux(enum ads1115_mux_t mux, ads1115_adc_t *adc) {
    adc->config &= ~ADS1115_MUX_MASK;
    adc->config |= mux;
}

void ads1115_set_pga(enum ads1115_pga_t pga, ads1115_adc_t *adc){
    adc->config &= ~ADS1115_PGA_MASK;
    adc->config |= pga;
}

void ads1115_set_operating_mode(enum ads1115_mode_t mode,
                                ads1115_adc_t *adc) {
    adc->config &= ~ADS1115_MODE_MASK;
    adc->config |= mode;
}

void ads1115_set_data_rate(enum ads1115_rate_t rate,
                           ads1115_adc_t *adc) {
    adc->config &= ~ADS1115_RATE_MASK;
    adc->config |= rate;
}

void ads1115_set_comparator_latching(bool latching, ads1115_adc_t *adc) {
    enum ads1115_comparator_latching_t comparator_latching = latching
        ? ADS1115_COMPARATOR_LATCHING
        : ADS1115_COMPARATOR_NONLATCHING;

    adc->config &= ~ADS1115_COMPARATOR_LATCHING_MASK;
    adc->config |= comparator_latching;
}

void ads1115_set_comparator_queue(enum ads1115_comparator_queue_t queue,
                                  ads1115_adc_t *adc) {
    adc->config &= ~ADS1115_COMPARATOR_QUEUE_MASK;
    adc->config |= queue;
}

void ads1115_set_comparator_mode(enum ads1115_comparator_mode_t mode,
                                 ads1115_adc_t *adc) {
    adc->config &= ~ADS1115_COMPARATOR_MODE_MASK;
    adc->config |= mode;
}

void ads1115_set_comparator_polarity(enum ads1115_comparator_polarity_t polarity,
                                     ads1115_adc_t *adc) {
    adc->config &= ~ADS1115_COMPARATOR_POLARITY_MASK;
    adc->config |= polarity;
}

void ads1115_set_comparator_window(uint16_t low_threshold,
                                   uint16_t high_threshold,
                                   ads1115_adc_t *adc) {
    ads1115_write_register(adc, ADS1115_POINTER_LO_THRESH, low_threshold);
    ads1115_write_register(adc, ADS1115_POINTER_HI_THRESH, high_threshold);
}

void ads1115_set_window_conversion_ready(enum ads1115_comparator_queue_t queue,
                                         ads1115_adc_t *adc) {
    ads1115_set_comparator_queue(queue, adc);
    ads1115_set_comparator_window(0x0000, 0x8000, adc);
}
