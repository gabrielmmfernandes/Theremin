
#include <stdio.h> // logs
//#include <stdint.h>
//#include <stdlib.h>
#include <math.h> // dsp

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h" // PCM5102A
#include "vl53l5cx_api.h"

#include "definitions.h"

/*
ESP32 DEVKITV1    VL53L5CX 1    VL53L5CX 2    PCM5102A
3v3               vin           vin           vin,xsmt
gnd               gnd           gnd           gnd,sck,fmt,demp,flt
d21               sda
d22               scl
d18                             sda
d19                             scl
d27                                           din
d26                                           bck
d25                                           lck

PCM5102A XSMT mutes on LOW
*/

// VL53L5C_1
#define TOF1_I2C_SDA_GPIO       GPIO_NUM_21
#define TOF1_I2C_SCL_GPIO       GPIO_NUM_22
#define TOF1_I2C_PORT           I2C_NUM_0
#define TOF1_I2C_FREQUENCY_HZ   1000000

// VL53L5C_2
#define TOF2_I2C_SDA_GPIO       GPIO_NUM_18
#define TOF2_I2C_SCL_GPIO       GPIO_NUM_19
#define TOF2_I2C_PORT           I2C_NUM_1
#define TOF2_I2C_FREQUENCY_HZ   1000000

#define TOF_GRID_DIM   4 // 8
#define TOF_RESOLUTION VL53L5CX_RESOLUTION_4X4 //8x8

// PCM5102A
#define I2S_LCK_GPIO       GPIO_NUM_25
#define I2S_BCK_GPIO       GPIO_NUM_26
#define I2S_DIN_GPIO       GPIO_NUM_27

#define SAMPLE_RATE 44100
#define AMPLITUDE   5000
#define BUFFER_SIZE 512

#define DETECT_MM   150      // 15 cm
#define NUM_PITCHES 8

// from 0 to 3 for TOF1, and from 4 to 7 for TOF2
static volatile int note_active[NUM_PITCHES] = {0, 0, 0, 0, 0, 0, 0, 0};
static const float freq[NUM_PITCHES] = {C4, D4, E4, F4, G4, A4, B4, C5};

void pcm5102a(void *pv) {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
    I2S_NUM_0, I2S_ROLE_MASTER
  );
  chan_cfg.dma_desc_num = 8;
  chan_cfg.dma_frame_num = 256;
  chan_cfg.auto_clear = true;

  i2s_std_config_t std_cfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO
    ),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = I2S_BCK_GPIO,
      .ws   = I2S_LCK_GPIO,
      .dout = I2S_DIN_GPIO,
      .din  = I2S_GPIO_UNUSED,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv   = false,
      },
    },
  };

  i2s_chan_handle_t tx_chan;
  static int16_t buffer[BUFFER_SIZE];
  size_t buffer_len = sizeof(buffer);
  size_t written;

  if(
    i2s_new_channel(&chan_cfg, &tx_chan, NULL) != ESP_OK ||
    i2s_channel_init_std_mode(tx_chan, &std_cfg) != ESP_OK ||
    i2s_channel_preload_data(tx_chan, buffer, buffer_len, &written) != ESP_OK ||
    i2s_channel_enable(tx_chan) != ESP_OK
  ) {
    printf("Erro ao iniciar I2S\n");
    vTaskDelete(NULL);
  }

  float phase[NUM_PITCHES] = {0};
  float gain[NUM_PITCHES]  = {0};
  float step[NUM_PITCHES];
  for(int n = 0; n < NUM_PITCHES; n++) {
    step[n] = 2.0f * M_PI * freq[n] / SAMPLE_RATE;
  }
  const float ramp = 1.0f / (0.01f * SAMPLE_RATE);   // 10 ms fade in/out

  while(1) {
    float target[NUM_PITCHES];
    for(int n = 0; n < NUM_PITCHES; n++) {
      target[n] = note_active[n] ? 1.0f : 0.0f;
    }

    for(int i = 0; i < 256; i++) {
      float mix = 0.0f;
      float gsum = 0.0f;
      for(int n = 0; n < NUM_PITCHES; n++) {
        if(gain[n] < target[n]) {
          gain[n] += ramp;
          if(gain[n] > target[n]) gain[n] = target[n];
        } else if(gain[n] > target[n]) {
          gain[n] -= ramp;
          if(gain[n] < target[n]) gain[n] = target[n];
        }

        mix  += gain[n] * sinf(phase[n]);
        gsum += gain[n];
        phase[n] += step[n];
        if(phase[n] >= 2.0f * M_PI) phase[n] -= 2.0f * M_PI;
      }
      int16_t s = (int16_t)(AMPLITUDE * mix / (1.0f + gsum));
      buffer[2 * i] = buffer[2 * i + 1] = s;
    }
    i2s_channel_write(tx_chan, buffer, buffer_len, &written, portMAX_DELAY);
  }
}

typedef struct {
  uint8_t id;               // index into note_active[]
  gpio_num_t sda_gpio;
  gpio_num_t scl_gpio;
  i2c_port_t i2c_port;
  uint32_t i2c_frequency_hz;
  char name[32];
} vl53l5cx_platform_t;

static vl53l5cx_platform_t tof1_platform = {
  .sda_gpio = TOF1_I2C_SDA_GPIO,
  .scl_gpio = TOF1_I2C_SCL_GPIO,
  .i2c_port = TOF1_I2C_PORT,
  .i2c_frequency_hz = TOF1_I2C_FREQUENCY_HZ,
  .name = "TOF1",
  .id = 0
};

static vl53l5cx_platform_t tof2_platform = {
  .sda_gpio = TOF2_I2C_SDA_GPIO,
  .scl_gpio = TOF2_I2C_SCL_GPIO,
  .i2c_port = TOF2_I2C_PORT,
  .i2c_frequency_hz = TOF2_I2C_FREQUENCY_HZ,
  .name = "TOF2",
  .id = 1
};

static int zone_touched(
  const VL53L5CX_ResultsData *data_from_fov,
  uint16_t dist_limit_mm,
  int n_zones, int col_start, int col_end
) {
  for(int row = 0; row < n_zones; row++) {
    for(int col = col_start; col < col_end; col++) {
      int z = row * n_zones + col;
      uint16_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
      uint8_t st = data_from_fov->target_status[idx];
      uint16_t d = data_from_fov->distance_mm[idx];
      if((st == 5 || st == 9) && d > 0 && d < dist_limit_mm) return 1;
    }
  }
  return 0;
}

void vl53l5c(void *pvParameters) {
  vl53l5cx_platform_t *platform = (vl53l5cx_platform_t *)pvParameters;
  printf("\n[%s] Iniciando sensor VL53L5CX...\n", platform->name);

  esp_err_t err;
  uint8_t status = 0;
  uint8_t is_alive = 0;
  uint8_t is_ready = 0;
  uint8_t zone;

  VL53L5CX_Configuration *Dev = calloc(1, sizeof(VL53L5CX_Configuration));
  VL53L5CX_ResultsData *Results = calloc(1, sizeof(VL53L5CX_ResultsData));

  printf("\n[%s] Inicializando barramento I2C...\n", platform->name);

  i2c_master_bus_config_t i2c_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = platform->i2c_port,
    .scl_io_num = platform->scl_gpio,
    .sda_io_num = platform->sda_gpio,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
  };

  i2c_master_bus_handle_t bus_handle;

  err = i2c_new_master_bus(&i2c_config, &bus_handle);
  if(err != ESP_OK) {
    printf(
      "[%s] Erro ao criar barramento I2C: %s\n",
      platform->name,
      esp_err_to_name(err)
    );
    vTaskDelete(NULL);
  }

  i2c_device_config_t device_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = VL53L5CX_DEFAULT_I2C_ADDRESS >> 1,
    .scl_speed_hz = platform->i2c_frequency_hz,
  };

  err = i2c_master_bus_add_device(
    bus_handle,
    &device_config,
    &Dev->platform.handle
  );
  if(err != ESP_OK) {
    printf(
      "[%s] Erro ao adicionar dispositivo I2C: %s\n",
      platform->name,
      esp_err_to_name(err)
    );
    vTaskDelete(NULL);
  }

  Dev->platform.bus_config = i2c_config;
  Dev->platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
  Dev->platform.reset_gpio = GPIO_NUM_NC;

  printf("[%s] Verificando sensor VL53L5CX...\n", platform->name);

  status = vl53l5cx_is_alive(Dev, &is_alive);
  if(status != VL53L5CX_STATUS_OK || !is_alive) {
    printf(
      "[%s] VL53L5CX nao detectado. Verifique as conexoes.\n",
      platform->name
    );
    vTaskDelete(NULL);
  }

  printf("[%s] Sensor detectado!\n", platform->name);
  printf("[%s] Inicializando driver...\n", platform->name);

  status = vl53l5cx_init(Dev);
  if(status != VL53L5CX_STATUS_OK) {
    printf(
      "[%s] Falha ao inicializar o sensor. Status: %u\n",
      platform->name, status
    );
    vTaskDelete(NULL);
  }

  status = vl53l5cx_set_resolution(Dev, TOF_RESOLUTION);
  if(status != VL53L5CX_STATUS_OK) {
    printf(
      "[%s] Erro ao configurar resolucao 4x4: %u\n",
      platform->name, status);
    vTaskDelete(NULL);
  }

  status = vl53l5cx_set_ranging_frequency_hz(Dev, 30);
  if(status != VL53L5CX_STATUS_OK) {
    printf(
      "[%s] Erro ao configurar frequencia: %u\n",
      platform->name, status
    );
    vTaskDelete(NULL);
  }

  status = vl53l5cx_start_ranging(Dev);
  if(status != VL53L5CX_STATUS_OK) {
    printf("[%s] Erro ao iniciar medicao: %u\n", platform->name, status);
    vTaskDelete(NULL);
  }

  printf("\n[%s] Sensor iniciado. Lendo 16 zonas...\n", platform->name);
  while(1) {
    is_ready = 0;
    status = vl53l5cx_check_data_ready(Dev, &is_ready);

    if(status == VL53L5CX_STATUS_OK && is_ready) {
      status = vl53l5cx_get_ranging_data(Dev, Results);
      if(status == VL53L5CX_STATUS_OK) {        
        /*
        platform->id: 0 for TOF1 and 1 for TOF2 as defined in vl53l5cx_platform_t
        multiplies by 4 to achieve the correspondent index in note_active[], which are 0 for TOF1 and 4 for TOF2
        sums 1 to go from the first to the last notes of each TOF in note_active[]
        */
        int base_index = platform->id * 4; // index for the TOF first tone
        note_active[base_index++] = zone_touched(
          Results, DETECT_MM, TOF_GRID_DIM, 0, 1
        ); // left
        note_active[base_index++] = zone_touched(
          Results, DETECT_MM, TOF_GRID_DIM, 1, 2
        ); // mid-left
        note_active[base_index++] = zone_touched(
          Results, DETECT_MM, TOF_GRID_DIM, 2, 3
        ); // mid-right
        note_active[base_index] = zone_touched(
          Results, DETECT_MM, TOF_GRID_DIM, 3, 4
        ); // right

        // printf("\nQuadro: %u\n", Dev->streamcount);
        // for(zone = 0; zone < 16; zone++) {
        //   uint16_t index = VL53L5CX_NB_TARGET_PER_ZONE * zone;
        //   printf(
        //     "[%s] Zona %2u | Status: %u | Distancia: %d mm\n",
        //     platform->name,
        //     zone,
        //     Results->target_status[index],
        //     Results->distance_mm[index]
        //   );
        // }
      } else {
        printf("[%s] Erro ao obter dados: %u\n", platform->name, status);
      }
    } else if(status != VL53L5CX_STATUS_OK) {
      printf("[%s] Erro ao verificar dados: %u\n", platform->name, status);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void app_main(void) {
  xTaskCreatePinnedToCore(
    vl53l5c,
    "TOF1",
    8192,
    &tof1_platform,
    1,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    vl53l5c,
    "TOF2",
    8192,
    &tof2_platform,
    1,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    pcm5102a,
    "DAC",
    4096,
    NULL,
    5,
    NULL,
    1
  );
}
