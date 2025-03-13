/*
 * THIS FILE IS AUTOMATICALLY GENERATED
 *
 * Generator:     sensirion-driver-generator 1.1.2
 * Product:       sen66
 * Model-Version: 1.6.0
 */

#include "sen66_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"
#include "sensirion_test_setup.h"
#include <inttypes.h>
#include <stdio.h>

#define sensirion_hal_sleep_us sensirion_i2c_hal_sleep_usec

void print_byte_array(uint8_t* array, uint16_t len) {
    uint16_t i = 0;
    printf("0x");
    for (; i < len; i++) {
        printf("%02x", array[i]);
    }
}

TEST_GROUP (SEN66_Tests) {
    void setup() {
        sen66_init(0x6B);
    }
};

TEST (SEN66_Tests, test_perform_forced_co2_recalibration1) {
    int16_t local_error = 0;
    uint16_t correction = 0;
    local_error = sen66_perform_forced_co2_recalibration(600, &correction);
    CHECK_EQUAL_ZERO_TEXT(local_error, "perform_forced_co2_recalibration");
    printf("correction: %u\n", correction);
}

TEST (SEN66_Tests, test_get_product_name1) {
    int16_t local_error = 0;
    int8_t product_name[32] = {0};
    local_error = sen66_get_product_name(product_name, 32);
    CHECK_EQUAL_ZERO_TEXT(local_error, "get_product_name");
    printf("product_name: %s\n", product_name);
}

TEST (SEN66_Tests, test_get_serial_number1) {
    int16_t local_error = 0;
    int8_t serial_number[32] = {0};
    local_error = sen66_get_serial_number(serial_number, 32);
    CHECK_EQUAL_ZERO_TEXT(local_error, "get_serial_number");
    printf("serial_number: %s\n", serial_number);
}

TEST (SEN66_Tests, test_device_reset1) {
    int16_t local_error = 0;
    local_error = sen66_device_reset();
    CHECK_EQUAL_ZERO_TEXT(local_error, "device_reset");
}

TEST (SEN66_Tests, test_start_fan_cleaning1) {
    int16_t local_error = 0;
    local_error = sen66_start_fan_cleaning();
    CHECK_EQUAL_ZERO_TEXT(local_error, "start_fan_cleaning");
}

TEST (SEN66_Tests, test_set_temperature_offset_parameters1) {
    int16_t local_error = 0;
    local_error = sen66_set_temperature_offset_parameters(1, 10, 1, 0);
    CHECK_EQUAL_ZERO_TEXT(local_error, "set_temperature_offset_parameters");
}

TEST (SEN66_Tests, test_set_voc_algorithm_tuning_parameters1) {
    int16_t local_error = 0;
    local_error =
        sen66_set_voc_algorithm_tuning_parameters(100, 12, 12, 180, 50, 230);
    CHECK_EQUAL_ZERO_TEXT(local_error, "set_voc_algorithm_tuning_parameters");
}

TEST (SEN66_Tests, test_get_voc_algorithm_tuning_parameters1) {
    int16_t local_error = 0;
    int16_t index_offset = 0;
    int16_t learning_time_offset_hours = 0;
    int16_t learning_time_gain_hours = 0;
    int16_t gating_max_duration_minutes = 0;
    int16_t std_initial = 0;
    int16_t gain_factor = 0;
    local_error = sen66_get_voc_algorithm_tuning_parameters(
        &index_offset, &learning_time_offset_hours, &learning_time_gain_hours,
        &gating_max_duration_minutes, &std_initial, &gain_factor);
    CHECK_EQUAL_ZERO_TEXT(local_error, "get_voc_algorithm_tuning_parameters");
    printf("index_offset: %i ", index_offset);
    printf("learning_time_offset_hours: %i ", learning_time_offset_hours);
    printf("learning_time_gain_hours: %i ", learning_time_gain_hours);
    printf("gating_max_duration_minutes: %i ", gating_max_duration_minutes);
    printf("std_initial: %i ", std_initial);
    printf("gain_factor: %i\n", gain_factor);
}

TEST (SEN66_Tests, test_set_nox_algorithm_tuning_parameters1) {
    int16_t local_error = 0;
    local_error =
        sen66_set_nox_algorithm_tuning_parameters(1, 12, 12, 720, 50, 230);
    CHECK_EQUAL_ZERO_TEXT(local_error, "set_nox_algorithm_tuning_parameters");
}

TEST (SEN66_Tests, test_get_nox_algorithm_tuning_parameters1) {
    int16_t local_error = 0;
    int16_t index_offset = 0;
    int16_t learning_time_offset_hours = 0;
    int16_t learning_time_gain_hours = 0;
    int16_t gating_max_duration_minutes = 0;
    int16_t std_initial = 0;
    int16_t gain_factor = 0;
    local_error = sen66_get_nox_algorithm_tuning_parameters(
        &index_offset, &learning_time_offset_hours, &learning_time_gain_hours,
        &gating_max_duration_minutes, &std_initial, &gain_factor);
    CHECK_EQUAL_ZERO_TEXT(local_error, "get_nox_algorithm_tuning_parameters");
    printf("index_offset: %i ", index_offset);
    printf("learning_time_offset_hours: %i ", learning_time_offset_hours);
    printf("learning_time_gain_hours: %i ", learning_time_gain_hours);
    printf("gating_max_duration_minutes: %i ", gating_max_duration_minutes);
    printf("std_initial: %i ", std_initial);
    printf("gain_factor: %i\n", gain_factor);
}

TEST (SEN66_Tests, test_set_temperature_acceleration_parameters1) {
    int16_t local_error = 0;
    local_error = sen66_set_temperature_acceleration_parameters(1, 1, 1, 1);
    CHECK_EQUAL_ZERO_TEXT(local_error,
                          "set_temperature_acceleration_parameters");
}

TEST (SEN66_Tests, test_get_voc_algorithm_state1) {
    int16_t local_error = 0;
    uint8_t state[8] = {0};
    local_error = sen66_get_voc_algorithm_state(state, 8);
    CHECK_EQUAL_ZERO_TEXT(local_error, "get_voc_algorithm_state");
    printf("state: ");
    print_byte_array(state, 8);
    printf("\n");
}

TEST (SEN66_Tests, test_set_co2_sensor_automatic_self_calibration1) {
    int16_t local_error = 0;
    local_error = sen66_set_co2_sensor_automatic_self_calibration(0);
    CHECK_EQUAL_ZERO_TEXT(local_error,
                          "set_co2_sensor_automatic_self_calibration");
}

TEST (SEN66_Tests, test_get_co2_sensor_automatic_self_calibration1) {
    int16_t local_error = 0;
    uint8_t padding = 0;
    bool status = false;
    local_error =
        sen66_get_co2_sensor_automatic_self_calibration(&padding, &status);
    CHECK_EQUAL_ZERO_TEXT(local_error,
                          "get_co2_sensor_automatic_self_calibration");
    printf("padding: %u ", padding);
    printf("status: %d\n", status);
}

TEST (SEN66_Tests, test_set_ambient_pressure1) {
    int16_t local_error = 0;
    local_error = sen66_set_ambient_pressure(1013);
    CHECK_EQUAL_ZERO_TEXT(local_error, "set_ambient_pressure");
}

TEST (SEN66_Tests, test_get_ambient_pressure1) {
    int16_t local_error = 0;
    uint16_t ambient_pressure = 0;
    local_error = sen66_get_ambient_pressure(&ambient_pressure);
    CHECK_EQUAL_ZERO_TEXT(local_error, "get_ambient_pressure");
    printf("ambient_pressure: %u\n", ambient_pressure);
}

TEST (SEN66_Tests, test_set_sensor_altitude1) {
    int16_t local_error = 0;
    local_error = sen66_set_sensor_altitude(0);
    CHECK_EQUAL_ZERO_TEXT(local_error, "set_sensor_altitude");
}

TEST (SEN66_Tests, test_get_sensor_altitude1) {
    int16_t local_error = 0;
    uint16_t altitude = 0;
    local_error = sen66_get_sensor_altitude(&altitude);
    CHECK_EQUAL_ZERO_TEXT(local_error, "get_sensor_altitude");
    printf("altitude: %u\n", altitude);
}

TEST (SEN66_Tests, test_activate_sht_heater1) {
    int16_t local_error = 0;
    local_error = sen66_activate_sht_heater();
    CHECK_EQUAL_ZERO_TEXT(local_error, "activate_sht_heater");
}

TEST (SEN66_Tests, test_read_device_status1) {
    int16_t local_error = 0;
    sen66_device_status device_status = {.value = 0u};
    local_error = sen66_read_device_status(&device_status);
    CHECK_EQUAL_ZERO_TEXT(local_error, "read_device_status");
    printf("device_status: %02x\n", device_status.value);
}

TEST (SEN66_Tests, test_read_and_clear_device_status1) {
    int16_t local_error = 0;
    sen66_device_status device_status = {.value = 0u};
    local_error = sen66_read_and_clear_device_status(&device_status);
    CHECK_EQUAL_ZERO_TEXT(local_error, "read_and_clear_device_status");
    printf("device_status: %02x\n", device_status.value);
}

TEST (SEN66_Tests, test_start_continuous_measurement1) {
    int16_t local_error = 0;
    uint8_t padding = 0;
    bool data_ready = false;
    uint16_t mass_concentration_pm1p0 = 0;
    uint16_t mass_concentration_pm2p5 = 0;
    uint16_t mass_concentration_pm4p0 = 0;
    uint16_t mass_concentration_pm10p0 = 0;
    int16_t ambient_humidity = 0;
    int16_t ambient_temperature = 0;
    int16_t voc_index = 0;
    int16_t nox_index = 0;
    uint16_t co2 = 0;
    uint16_t number_concentration_pm0p5 = 0;
    uint16_t number_concentration_pm1p0 = 0;
    uint16_t number_concentration_pm2p5 = 0;
    uint16_t number_concentration_pm4p0 = 0;
    uint16_t number_concentration_pm10p0 = 0;
    int16_t raw_humidity = 0;
    int16_t raw_temperature = 0;
    uint16_t raw_voc = 0;
    uint16_t raw_nox = 0;
    uint16_t raw_co2 = 0;
    uint8_t state[8] = {0};
    local_error = sen66_start_continuous_measurement();
    CHECK_EQUAL_ZERO_TEXT(local_error, "start_continuous_measurement");
    local_error = sen66_get_data_ready(&padding, &data_ready);
    CHECK_EQUAL_ZERO_TEXT(local_error, "get_data_ready");
    printf("padding: %u ", padding);
    printf("data_ready: %d\n", data_ready);
    local_error = sen66_read_measured_values_as_integers(
        &mass_concentration_pm1p0, &mass_concentration_pm2p5,
        &mass_concentration_pm4p0, &mass_concentration_pm10p0,
        &ambient_humidity, &ambient_temperature, &voc_index, &nox_index, &co2);
    CHECK_EQUAL_ZERO_TEXT(local_error, "read_measured_values_as_integers");
    printf("mass_concentration_pm1p0: %u ", mass_concentration_pm1p0);
    printf("mass_concentration_pm2p5: %u ", mass_concentration_pm2p5);
    printf("mass_concentration_pm4p0: %u ", mass_concentration_pm4p0);
    printf("mass_concentration_pm10p0: %u ", mass_concentration_pm10p0);
    printf("ambient_humidity: %i ", ambient_humidity);
    printf("ambient_temperature: %i ", ambient_temperature);
    printf("voc_index: %i ", voc_index);
    printf("nox_index: %i ", nox_index);
    printf("co2: %u\n", co2);
    local_error = sen66_read_number_concentration_values_as_integers(
        &number_concentration_pm0p5, &number_concentration_pm1p0,
        &number_concentration_pm2p5, &number_concentration_pm4p0,
        &number_concentration_pm10p0);
    CHECK_EQUAL_ZERO_TEXT(local_error,
                          "read_number_concentration_values_as_integers");
    printf("number_concentration_pm0p5: %u ", number_concentration_pm0p5);
    printf("number_concentration_pm1p0: %u ", number_concentration_pm1p0);
    printf("number_concentration_pm2p5: %u ", number_concentration_pm2p5);
    printf("number_concentration_pm4p0: %u ", number_concentration_pm4p0);
    printf("number_concentration_pm10p0: %u\n", number_concentration_pm10p0);
    local_error = sen66_read_measured_raw_values(
        &raw_humidity, &raw_temperature, &raw_voc, &raw_nox, &raw_co2);
    CHECK_EQUAL_ZERO_TEXT(local_error, "read_measured_raw_values");
    printf("raw_humidity: %i ", raw_humidity);
    printf("raw_temperature: %i ", raw_temperature);
    printf("raw_voc: %u ", raw_voc);
    printf("raw_nox: %u ", raw_nox);
    printf("raw_co2: %u\n", raw_co2);
    local_error = sen66_set_temperature_offset_parameters(1, 10, 1, 0);
    CHECK_EQUAL_ZERO_TEXT(local_error, "set_temperature_offset_parameters");
    local_error = sen66_get_voc_algorithm_state(state, 8);
    CHECK_EQUAL_ZERO_TEXT(local_error, "get_voc_algorithm_state");
    printf("state: ");
    print_byte_array(state, 8);
    printf("\n");
    local_error = sen66_stop_measurement();
    CHECK_EQUAL_ZERO_TEXT(local_error, "stop_measurement");
}
