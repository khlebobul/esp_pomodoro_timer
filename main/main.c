#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "bsp/esp-bsp.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_err.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_io_expander.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

#define QMI_ADDR_A 0x6A
#define QMI_ADDR_B 0x6B
#define QMI_WHO_AM_I 0x00
#define QMI_CTRL1 0x02
#define QMI_CTRL2 0x03
#define QMI_CTRL7 0x08
#define QMI_AX_L 0x35
#define EDGE_G 0.70f
#define SAMPLE_RATE 16000
#define BEEP_SAMPLES 1600

static const char *TAG = "pomodoro";
static i2c_master_dev_handle_t s_imu;
static lv_obj_t *s_time;
static lv_obj_t *s_state;
static lv_display_t *s_display;
static int64_t s_started_us;
static int s_duration;
static int s_remaining;
static bool s_running;
static bool s_finished;
static lv_disp_rotation_t s_rotation = LV_DISPLAY_ROTATION_0;
static esp_codec_dev_handle_t s_speaker;
static int16_t s_beep[BEEP_SAMPLES];

static void speaker_init(void)
{
    i2s_chan_handle_t tx = NULL;
    i2s_chan_handle_t rx = NULL;
    i2s_chan_config_t channel = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    channel.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&channel, &tx, &rx));
    const i2s_std_config_t i2s = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {.mclk = BSP_I2S_MCLK, .bclk = BSP_I2S_SCLK, .ws = BSP_I2S_LCLK,
                     .dout = BSP_I2S_DOUT, .din = BSP_I2S_DSIN},
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx, &i2s));
    ESP_ERROR_CHECK(i2s_channel_enable(tx));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx, &i2s));
    ESP_ERROR_CHECK(i2s_channel_enable(rx));
    audio_codec_i2s_cfg_t i2s_cfg = {.port = CONFIG_BSP_I2S_NUM, .tx_handle = tx, .rx_handle = rx};
    audio_codec_i2c_cfg_t i2c_cfg = {.port = BSP_I2C_NUM, .addr = ES8311_CODEC_DEFAULT_ADDR,
                                      .bus_handle = bsp_i2c_get_handle()};
    const audio_codec_if_t *codec = es8311_codec_new(&(es8311_codec_cfg_t){
        .ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg), .gpio_if = audio_codec_new_gpio(),
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC, .pa_pin = BSP_POWER_AMP_IO,
    });
    s_speaker = esp_codec_dev_new(&(esp_codec_dev_cfg_t){.dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = codec, .data_if = audio_codec_new_i2s_data(&i2s_cfg)});
    esp_codec_dev_open(s_speaker, &(esp_codec_dev_sample_info_t){
        .bits_per_sample = 16, .channel = 1, .sample_rate = SAMPLE_RATE, .mclk_multiple = 256});
    esp_codec_dev_set_out_vol(s_speaker, 90);
}

static void play_done(void)
{
    for (int i = 0; i < BEEP_SAMPLES; ++i) {
        const float fade = i < 160 ? (float)i / 160.f
                                    : (i > 1440 ? (float)(BEEP_SAMPLES - i) / 160.f : 1.f);
        s_beep[i] = (int16_t)(sinf(2.f * 3.14159265f * 660.f * i / SAMPLE_RATE) * 4200.f * fade);
    }
    for (int i = 0; i < 3; ++i) {
        esp_codec_dev_write(s_speaker, s_beep, sizeof(s_beep));
        vTaskDelay(pdMS_TO_TICKS(140));
    }
}

static esp_err_t imu_write(uint8_t reg, uint8_t value)
{
    const uint8_t bytes[] = {reg, value};
    return i2c_master_transmit(s_imu, bytes, sizeof(bytes), 50);
}

static bool imu_init(void)
{
    ESP_ERROR_CHECK(bsp_i2c_init());
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    uint8_t addr = QMI_ADDR_A;
    if (i2c_master_probe(bus, addr, 50) != ESP_OK) {
        addr = QMI_ADDR_B;
    }
    if (i2c_master_probe(bus, addr, 50) != ESP_OK) {
        ESP_LOGE(TAG, "QMI8658 not found");
        return false;
    }
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &cfg, &s_imu));
    uint8_t who;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(s_imu, (uint8_t[]){QMI_WHO_AM_I}, 1, &who, 1, 50));
    ESP_LOGI(TAG, "QMI8658 at 0x%02X, id 0x%02X", addr, who);
    ESP_ERROR_CHECK(imu_write(QMI_CTRL1, 0x60)); // auto-increment
    ESP_ERROR_CHECK(imu_write(QMI_CTRL2, 0x23)); // +/-8g, 250 Hz
    ESP_ERROR_CHECK(imu_write(QMI_CTRL7, 0x01)); // accelerometer on
    vTaskDelay(pdMS_TO_TICKS(20));
    return true;
}

static int edge_seconds(lv_disp_rotation_t *rotation)
{
    uint8_t raw[6];
    if (i2c_master_transmit_receive(s_imu, (uint8_t[]){QMI_AX_L}, 1, raw, sizeof(raw), 50) != ESP_OK) {
        return 0;
    }
    const int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
    const int16_t y = (int16_t)((raw[3] << 8) | raw[2]);
    const int edge = (int)(4096 * EDGE_G); // QMI_CTRL2 0x23: +/-8g
    if (abs(x) < edge && abs(y) < edge) {
        return 0;
    }
    if (abs(x) > abs(y)) {
        *rotation = x > 0 ? LV_DISPLAY_ROTATION_0 : LV_DISPLAY_ROTATION_180;
        return x > 0 ? 5 : 10;
    }
    *rotation = y > 0 ? LV_DISPLAY_ROTATION_270 : LV_DISPLAY_ROTATION_90;
    return y > 0 ? 30 : 60;
}

static void set_running(bool running)
{
    if (running == s_running) {
        return;
    }
    if (running) {
        s_started_us = esp_timer_get_time();
    } else {
        s_remaining -= (int)((esp_timer_get_time() - s_started_us) / 1000000LL);
    }
    s_running = running;
}

static void update_ui(lv_timer_t *timer)
{
    (void)timer;
    lv_disp_rotation_t rotation = s_rotation;
    const int duration = edge_seconds(&rotation);
    if (duration == 0) {
        s_running = false;
        s_finished = false;
        s_duration = 0;
        s_remaining = 0;
    } else if (rotation != s_rotation) {
        bsp_display_rotate(s_display, rotation);
        s_rotation = rotation;
    }
    if (duration != 0 && duration != s_duration) {
        s_duration = duration;
        s_remaining = duration;
        s_started_us = esp_timer_get_time();
        s_finished = false;
    }
    set_running(duration != 0 && !s_finished);
    int remaining = s_remaining;
    if (s_running) {
        remaining -= (int)((esp_timer_get_time() - s_started_us) / 1000000LL);
    }
    if (s_running && remaining <= 0) {
        s_remaining = 0;
        remaining = 0;
        s_finished = true;
        s_running = false;
        play_done();
    }
    char text[16];
    snprintf(text, sizeof(text), "%02d:%02d", remaining / 60, remaining % 60);
    lv_label_set_text(s_time, text);
    lv_obj_align(s_time, LV_ALIGN_CENTER, 0, 8);
    if (s_duration == 0) {
        lv_label_set_text(s_state, "PLACE ON EDGE");
        lv_label_set_text(s_time, "--:--");
    } else if (s_finished) {
        lv_label_set_text(s_state, "TIME IS UP");
    } else if (s_running) {
        lv_label_set_text_fmt(s_state, "%d SEC", s_duration);
    } else {
        lv_label_set_text_fmt(s_state, "%d SEC", s_duration);
    }
}

static void orientation_self_check(void)
{
    assert(EDGE_G > 0.0f && EDGE_G < 1.0f);
}

static void build_ui(void)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x0C0B09), LV_PART_MAIN);
    lv_obj_set_style_pad_all(screen, 0, LV_PART_MAIN);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    s_state = lv_label_create(screen);
    lv_obj_set_style_text_font(s_state, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_state, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(s_state, LV_ALIGN_TOP_MID, 0, 60);

    s_time = lv_label_create(screen);
    lv_obj_set_style_text_font(s_time, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_time, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(s_time, LV_ALIGN_CENTER, 0, 8);
}

static void release_v2_panel_reset(void)
{
    if (bsp_i2c_init() != ESP_OK || i2c_master_probe(bsp_i2c_get_handle(), BSP_IO_EXPANDER_I2C_ADDRESS, 50) != ESP_OK) {
        return;
    }
    esp_io_expander_handle_t expander = bsp_io_expander_init();
    if (expander == NULL) {
        return;
    }
    const uint32_t pins = IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1 | IO_EXPANDER_PIN_NUM_2;
    ESP_ERROR_CHECK(esp_io_expander_set_dir(expander, pins, IO_EXPANDER_OUTPUT));
    ESP_ERROR_CHECK(esp_io_expander_set_level(expander, pins, 1));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(esp_io_expander_set_level(expander, pins, 0));
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_ERROR_CHECK(esp_io_expander_set_level(expander, pins, 1));
}

void app_main(void)
{
    orientation_self_check();
    release_v2_panel_reset();
    if (!imu_init()) {
        return;
    }
    speaker_init();
    if ((s_display = bsp_display_start()) == NULL) {
        return;
    }
    ESP_ERROR_CHECK(bsp_display_backlight_on());
    if (!bsp_display_lock(1000)) {
        return;
    }
    build_ui();
    lv_timer_create(update_ui, 250, NULL);
    bsp_display_unlock();
}
