#include "encoder.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdatomic.h>

static const char *TAG = "encoder";

static int s_clk_pin;
static int s_dt_pin;
static int s_sw_pin;

static atomic_int    s_steps     = 0;
static volatile bool s_raw_press = false;
static TickType_t    s_last_press_tick = 0;
#define DEBOUNCE_TICKS pdMS_TO_TICKS(50)

/* ── Gray-code state machine ─────────────────────────────────────────
 * Both CLK and DT trigger on ANY edge. State = (clk<<1)|dt (0–3).
 * Table indexed by (prev_state<<2)|curr_state:
 *   +1 at 0b10→0b11 (CW  complete, back to rest)
 *   -1 at 0b01→0b11 (CCW complete, back to rest)
 * Counting only at rest prevents double-counting on the same detent.
 * Post-step 50 ms debounce suppresses the return-bounce that would
 * otherwise trigger an immediate opposite step. */
static const int8_t s_enc_table[16] = {
    0,  0,  0,  0,   /* prev=00: 00→00, 00→01, 00→10, 00→11 */
    0,  0,  0, -1,   /* prev=01: 01→00, 01→01, 01→10, 01→11 */
    0,  0,  0,  1,   /* prev=10: 10→00, 10→01, 10→10, 10→11 */
    0,  0,  0,  0,   /* prev=11: 11→00, 11→01, 11→10, 11→11 */
};
static volatile uint8_t   s_enc_state = 3;  /* assume at rest (CLK=1, DT=1) */

/* Diagnostic: total enc_isr calls; read via encoder_get_isr_count(). */
static volatile uint32_t  s_enc_isr_count = 0;

/* Post-step debounce: ignore further steps for 50 ms after counting one. */
static volatile TickType_t s_last_step_tick = 0;
#define STEP_DEBOUNCE_TICKS pdMS_TO_TICKS(50)

static void IRAM_ATTR enc_isr(void *arg)
{
    s_enc_isr_count++;

    uint8_t curr = (uint8_t)(gpio_get_level(s_clk_pin) << 1) |
                   (uint8_t) gpio_get_level(s_dt_pin);
    uint8_t idx  = (uint8_t)((s_enc_state << 2) | curr);
    s_enc_state  = curr;

    int8_t delta = s_enc_table[idx & 0x0F];
    if (!delta) return;

    TickType_t now = xTaskGetTickCountFromISR();
    if ((now - s_last_step_tick) < STEP_DEBOUNCE_TICKS) return;
    s_last_step_tick = now;
    atomic_fetch_add(&s_steps, delta);
}

static void IRAM_ATTR sw_isr(void *arg)
{
    s_raw_press = true;
}

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t encoder_init(int clk_pin, int dt_pin, int sw_pin)
{
    s_clk_pin = clk_pin;
    s_dt_pin  = dt_pin;
    s_sw_pin  = sw_pin;
    s_enc_state = (uint8_t)((gpio_get_level(clk_pin) << 1) | gpio_get_level(dt_pin));

    gpio_config_t io = {
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << clk_pin) | (1ULL << dt_pin) | (1ULL << sw_pin),
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    gpio_set_intr_type(clk_pin, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(dt_pin,  GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(sw_pin,  GPIO_INTR_NEGEDGE);

    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service: 0x%x", ret);
        return ret;
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(clk_pin, enc_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(dt_pin,  enc_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(sw_pin,  sw_isr,  NULL));

    ESP_LOGI(TAG, "encoder ready: CLK=%d DT=%d SW=%d", clk_pin, dt_pin, sw_pin);
    return ESP_OK;
}

int encoder_get_steps(void)
{
    return atomic_exchange(&s_steps, 0);
}

bool encoder_get_press(void)
{
    if (!s_raw_press) return false;
    s_raw_press = false;
    TickType_t now = xTaskGetTickCount();
    if ((now - s_last_press_tick) < DEBOUNCE_TICKS) return false;
    s_last_press_tick = now;
    return true;
}

