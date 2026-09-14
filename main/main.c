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

#define MIN_TOP 30
#define MIN_RIGHT 10
#define MIN_BOTTOM 5
#define MIN_LEFT 60

static const char *TAG = "pomodoro";
static i2c_master_dev_handle_t s_imu;
static lv_obj_t *s_phase;
static lv_obj_t *s_dur;
static lv_obj_t *s_time;
static lv_obj_t *s_progress;
static lv_obj_t *s_pill;
static lv_obj_t *s_pill_text;
static lv_obj_t *s_edge_t;
static lv_obj_t *s_edge_r;
static lv_obj_t *s_edge_b;
static lv_obj_t *s_edge_l;
static lv_obj_t *s_hint[4];
static lv_obj_t *s_btn;
static lv_obj_t *s_btn_text;
static lv_display_t *s_display;
static int64_t s_started_us;
static int s_duration;
static int s_remaining;
static bool s_running;
static bool s_finished;
static lv_disp_rotation_t s_rotation = LV_DISPLAY_ROTATION_0;
static esp_codec_dev_handle_t s_speaker;

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

static void play_note(float freq, float seconds)
{
    const int total = (int)(seconds * SAMPLE_RATE);
    const float tau = 0.20f;
    int16_t chunk[512];
    for (int pos = 0; pos < total;) {
        int cnt = total - pos;
        if (cnt > 512) {
            cnt = 512;
        }
        for (int i = 0; i < cnt; ++i) {
            const int n = pos + i;
            const float t = (float)n / SAMPLE_RATE;
            const float decay = expf(-t / tau);
            const float attack = fminf(1.0f, t / 0.002f);
            float v = sinf(2.f * 3.14159265f * freq * t);
            v += 0.40f * sinf(2.f * 3.14159265f * freq * 2.f * t);
            v += 0.15f * sinf(2.f * 3.14159265f * freq * 3.f * t);
            const float tail = total - n < 40 ? (float)(total - n) / 40.f : 1.f;
            chunk[i] = (int16_t)(v * 3000.f * decay * attack * tail);
        }
        esp_codec_dev_write(s_speaker, chunk, cnt * (int)sizeof(int16_t));
        pos += cnt;
    }
    vTaskDelay(pdMS_TO_TICKS(90));
}

static void play_done(void)
{
    play_note(1046.5f, 0.45f);
    play_note(784.0f, 0.80f);
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

static int edge_minutes(lv_disp_rotation_t *rotation)
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
        return x > 0 ? MIN_BOTTOM : MIN_TOP;
    }
    *rotation = y > 0 ? LV_DISPLAY_ROTATION_270 : LV_DISPLAY_ROTATION_90;
    return y > 0 ? MIN_LEFT : MIN_RIGHT;
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

static void reset_button_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    s_duration = 0;
    s_remaining = 0;
    s_finished = false;
    s_running = false;
}

static void show_place_hint(bool show)
{
    static const char *words[] = {"PLACE", "ON", "A", "SIDE"};
    for (int i = 0; i < 4; ++i) {
        if (show) {
            lv_obj_remove_flag(s_hint[i], LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(s_hint[i], words[i]);
            lv_obj_align(s_hint[i], LV_ALIGN_CENTER, 0, -30 + i * 20);
        } else {
            lv_obj_add_flag(s_hint[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void update_ui(lv_timer_t *timer)
{
    (void)timer;
    lv_disp_rotation_t rotation = s_rotation;
    const int duration = edge_minutes(&rotation);
    if (duration == 0) {
        if (s_finished || s_duration == 0) {
            s_duration = 0;
            s_remaining = 0;
            s_finished = false;
            s_running = false;
        } else {
            set_running(false);
        }
    } else if (rotation != s_rotation) {
        bsp_display_rotate(s_display, rotation);
        s_rotation = rotation;
    }
    if (duration != 0 && duration != s_duration) {
        s_duration = duration;
        s_remaining = duration * 60;
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
    const bool idle = (s_duration == 0);
    if (idle) {
        if (s_rotation != LV_DISPLAY_ROTATION_0) {
            bsp_display_rotate(s_display, LV_DISPLAY_ROTATION_0);
            s_rotation = LV_DISPLAY_ROTATION_0;
        }
    }
    if (idle) {
        show_place_hint(true);
        lv_obj_add_flag(s_phase, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_time, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_dur, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_progress, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_pill, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_edge_t, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_edge_r, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_edge_b, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_edge_l, LV_OBJ_FLAG_HIDDEN);
        lv_obj_align(s_edge_t, LV_ALIGN_TOP_MID, 0, 14);
        lv_obj_align(s_edge_r, LV_ALIGN_RIGHT_MID, -14, 0);
        lv_obj_align(s_edge_b, LV_ALIGN_BOTTOM_MID, 0, -14);
        lv_obj_align(s_edge_l, LV_ALIGN_LEFT_MID, 14, 0);
    } else {
        const int total = s_duration * 60;
        const int elapsed = total - remaining;
        char text[16];
        snprintf(text, sizeof(text), "%02d:%02d", remaining / 60, remaining % 60);
        lv_label_set_text(s_time, text);
        lv_obj_update_layout(s_time);
        lv_obj_set_style_transform_pivot_x(s_time, lv_obj_get_width(s_time) / 2, LV_PART_MAIN);
        lv_obj_set_style_transform_pivot_y(s_time, lv_obj_get_height(s_time) / 2, LV_PART_MAIN);
        lv_obj_set_style_text_color(s_time, lv_color_white(), LV_PART_MAIN);
        lv_label_set_text_fmt(s_dur, "%d MIN", s_duration);
        lv_obj_remove_flag(s_time, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_dur, LV_OBJ_FLAG_HIDDEN);
        show_place_hint(false);
        if (s_finished) {
            lv_obj_remove_flag(s_progress, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(s_pill, LV_OBJ_FLAG_HIDDEN);
            lv_bar_set_value(s_progress, 1000, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(s_pill, lv_color_hex(0x3D0F14), LV_PART_MAIN);
            lv_obj_set_style_text_color(s_pill_text, lv_color_hex(0xFF4252), LV_PART_MAIN);
            lv_label_set_text(s_pill_text, "TIME IS UP");
            lv_label_set_text(s_phase, "FOCUS");
            lv_obj_set_style_text_color(s_phase, lv_color_hex(0xFF4252), LV_PART_MAIN);
        } else if (!s_running) {
            lv_obj_add_flag(s_progress, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_pill, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(s_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_align(s_btn, LV_ALIGN_BOTTOM_MID, 0, -24);
            lv_obj_center(s_btn_text);
            lv_label_set_text(s_phase, "PAUSED");
            lv_obj_set_style_text_color(s_phase, lv_color_hex(0xFFD24D), LV_PART_MAIN);
        } else {
            lv_obj_remove_flag(s_progress, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(s_pill, LV_OBJ_FLAG_HIDDEN);
            const uint32_t pct = elapsed <= 0 ? 0U : (uint32_t)((uint64_t)elapsed * 1000 / total);
            lv_bar_set_value(s_progress, pct, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(s_pill, lv_color_hex(0x0E3B26), LV_PART_MAIN);
            lv_obj_set_style_text_color(s_pill_text, lv_color_hex(0x22D27F), LV_PART_MAIN);
            lv_label_set_text(s_pill_text, "RUNNING");
            lv_label_set_text(s_phase, "FOCUS");
            lv_obj_set_style_text_color(s_phase, lv_color_hex(0xFF4252), LV_PART_MAIN);
        }
        lv_obj_remove_flag(s_phase, LV_OBJ_FLAG_HIDDEN);
        lv_obj_align(s_phase, LV_ALIGN_TOP_MID, 0, 40);
        lv_obj_align(s_dur, LV_ALIGN_TOP_MID, 0, 76);
        lv_obj_align(s_time, LV_ALIGN_CENTER, 0, -24);
        lv_obj_align(s_progress, LV_ALIGN_BOTTOM_MID, 0, -78);
        lv_obj_align(s_pill, LV_ALIGN_BOTTOM_MID, 0, -28);
        lv_obj_center(s_pill_text);
        lv_obj_remove_flag(s_time, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_dur, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_edge_t, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_edge_r, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_edge_b, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_edge_l, LV_OBJ_FLAG_HIDDEN);
    }
}

static void orientation_self_check(void)
{
    assert(EDGE_G > 0.0f && EDGE_G < 1.0f);
}

static void build_ui(void)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_pad_all(screen, 0, LV_PART_MAIN);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    s_phase = lv_label_create(screen);
    lv_obj_set_style_text_font(s_phase, &lv_font_unscii_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_phase, lv_color_hex(0xFF4252), LV_PART_MAIN);
    lv_obj_set_style_text_align(s_phase, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(s_phase, "FOCUS");

    s_dur = lv_label_create(screen);
    lv_obj_set_style_text_font(s_dur, &lv_font_unscii_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_dur, lv_color_hex(0x888888), LV_PART_MAIN);
    lv_label_set_text(s_dur, "");

    s_time = lv_label_create(screen);
    lv_obj_set_style_text_font(s_time, &lv_font_unscii_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_time, lv_color_hex(0x555555), LV_PART_MAIN);
    lv_obj_set_style_transform_scale_x(s_time, 768, LV_PART_MAIN);
    lv_obj_set_style_transform_scale_y(s_time, 768, LV_PART_MAIN);
    lv_label_set_text(s_time, "--:--");
    lv_obj_update_layout(s_time);
    lv_obj_set_style_transform_pivot_x(s_time, lv_obj_get_width(s_time) / 2, LV_PART_MAIN);
    lv_obj_set_style_transform_pivot_y(s_time, lv_obj_get_height(s_time) / 2, LV_PART_MAIN);

    s_progress = lv_bar_create(screen);
    lv_obj_set_size(s_progress, 300, 10);
    lv_obj_remove_flag(s_progress, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(s_progress, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_progress, lv_color_hex(0x1A1A1A), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_progress, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_progress, lv_color_hex(0xFF4252), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s_progress, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_bar_set_range(s_progress, 0, 1000);

    s_pill = lv_obj_create(screen);
    lv_obj_remove_flag(s_pill, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(s_pill, 150, 36);
    lv_obj_set_style_radius(s_pill, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_pill, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_pill, lv_color_hex(0x1A1A1A), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_pill, LV_OPA_COVER, LV_PART_MAIN);

    s_pill_text = lv_label_create(s_pill);
    lv_obj_set_style_text_font(s_pill_text, &lv_font_unscii_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_pill_text, lv_color_hex(0xAAAAAA), LV_PART_MAIN);
    lv_label_set_text(s_pill_text, "READY");

    s_edge_t = lv_label_create(screen);
    lv_obj_set_style_text_font(s_edge_t, &lv_font_unscii_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_edge_t, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_label_set_text_fmt(s_edge_t, "%d", MIN_TOP);

    s_edge_r = lv_label_create(screen);
    lv_obj_set_style_text_font(s_edge_r, &lv_font_unscii_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_edge_r, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_label_set_text_fmt(s_edge_r, "%d", MIN_RIGHT);

    s_edge_b = lv_label_create(screen);
    lv_obj_set_style_text_font(s_edge_b, &lv_font_unscii_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_edge_b, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_label_set_text_fmt(s_edge_b, "%d", MIN_BOTTOM);

    s_edge_l = lv_label_create(screen);
    lv_obj_set_style_text_font(s_edge_l, &lv_font_unscii_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_edge_l, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_label_set_text_fmt(s_edge_l, "%d", MIN_LEFT);

    for (int i = 0; i < 4; ++i) {
        s_hint[i] = lv_label_create(screen);
        lv_obj_set_width(s_hint[i], lv_pct(100));
        lv_obj_set_style_text_font(s_hint[i], &lv_font_unscii_16, LV_PART_MAIN);
        lv_obj_set_style_text_color(s_hint[i], lv_color_hex(0xBBBBBB), LV_PART_MAIN);
        lv_obj_set_style_text_align(s_hint[i], LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    }

    s_btn = lv_button_create(screen);
    lv_obj_set_size(s_btn, 328, 60);
    lv_obj_set_style_radius(s_btn, 6, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_btn, lv_color_hex(0x3B3408), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_btn, lv_color_hex(0xFFD24D), LV_PART_MAIN);
    lv_obj_set_style_border_width(s_btn, 1, LV_PART_MAIN);
    lv_obj_set_style_border_opa(s_btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_add_flag(s_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_btn, reset_button_cb, LV_EVENT_CLICKED, NULL);

    s_btn_text = lv_label_create(s_btn);
    lv_obj_set_style_text_font(s_btn_text, &lv_font_unscii_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_btn_text, lv_color_hex(0xFFD24D), LV_PART_MAIN);
    lv_label_set_text(s_btn_text, "RESET");

    show_place_hint(true);
    lv_obj_add_flag(s_phase, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_time, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_dur, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_progress, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_pill, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_btn, LV_OBJ_FLAG_HIDDEN);
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
