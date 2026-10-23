
#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "vl53l5cx_api.h"

#define I2C_SDA_GPIO       GPIO_NUM_21
#define I2C_SCL_GPIO       GPIO_NUM_22
#define I2C_PORT           I2C_NUM_1
#define I2C_FREQUENCY_HZ   1000000

void app_main(void)
{
    esp_err_t err;
    uint8_t status = 0;
    uint8_t is_alive = 0;
    uint8_t is_ready = 0;
    uint8_t zone;

    VL53L5CX_Configuration Dev = {0};
    VL53L5CX_ResultsData Results = {0};

    printf("\nInicializando barramento I2C...\n");

    i2c_master_bus_config_t i2c_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_io_num = I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;

    err = i2c_new_master_bus(&i2c_config, &bus_handle);
    if (err != ESP_OK) {
        printf("Erro ao criar barramento I2C: %s\n",
               esp_err_to_name(err));
        return;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L5CX_DEFAULT_I2C_ADDRESS >> 1,
        .scl_speed_hz = I2C_FREQUENCY_HZ,
    };

    err = i2c_master_bus_add_device(
        bus_handle,
        &device_config,
        &Dev.platform.handle
    );

    if (err != ESP_OK) {
        printf("Erro ao adicionar dispositivo I2C: %s\n",
               esp_err_to_name(err));
        return;
    }

    Dev.platform.bus_config = i2c_config;
    Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
    Dev.platform.reset_gpio = GPIO_NUM_NC;

    printf("Verificando sensor VL53L5CX...\n");

    status = vl53l5cx_is_alive(&Dev, &is_alive);

    if (status != VL53L5CX_STATUS_OK || !is_alive) {
        printf("VL53L5CX nao detectado. Verifique as conexoes.\n");
        return;
    }

    printf("Sensor detectado!\n");
    printf("Inicializando driver...\n");

    status = vl53l5cx_init(&Dev);

    if (status != VL53L5CX_STATUS_OK) {
        printf("Falha ao inicializar o sensor. Status: %u\n", status);
        return;
    }

    status = vl53l5cx_set_resolution(
        &Dev,
        VL53L5CX_RESOLUTION_4X4
    );

    if (status != VL53L5CX_STATUS_OK) {
        printf("Erro ao configurar resolucao 4x4: %u\n", status);
        return;
    }

    status = vl53l5cx_set_ranging_frequency_hz(&Dev, 1);

    if (status != VL53L5CX_STATUS_OK) {
        printf("Erro ao configurar frequencia: %u\n", status);
        return;
    }

    status = vl53l5cx_start_ranging(&Dev);

    if (status != VL53L5CX_STATUS_OK) {
        printf("Erro ao iniciar medicao: %u\n", status);
        return;
    }

    printf("\nSensor iniciado. Lendo 16 zonas...\n");

    while (1) {
        is_ready = 0;

        status = vl53l5cx_check_data_ready(&Dev, &is_ready);

        if (status == VL53L5CX_STATUS_OK && is_ready) {
            status = vl53l5cx_get_ranging_data(&Dev, &Results);

            if (status == VL53L5CX_STATUS_OK) {
                printf("\nQuadro: %u\n", Dev.streamcount);

                for (zone = 0; zone < 16; zone++) {
                    uint16_t index =
                        VL53L5CX_NB_TARGET_PER_ZONE * zone;

                    printf(
                        "Zona %2u | Status: %u | Distancia: %d mm\n",
                        zone,
                        Results.target_status[index],
                        Results.distance_mm[index]
                    );
                }
            } else {
                printf("Erro ao obter dados: %u\n", status);
            }
        } else if (status != VL53L5CX_STATUS_OK) {
            printf("Erro ao verificar dados: %u\n", status);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
