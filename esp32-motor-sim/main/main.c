#include <stdio.h>
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"

#define PIN_A         4      /* sorties MCPWM */
#define PIN_B         5
#define PIN_IN_A      6      /* entrees PCNT, reliees par fil a 4 et 5 */
#define PIN_IN_B      7
#define RES_HZ        1000000
#define PERIOD_TICKS  200    /* 1 MHz / 200 = 5 kHz par voie */

static mcpwm_cmpr_handle_t new_cmp(mcpwm_oper_handle_t op, uint32_t val)
{
    mcpwm_cmpr_handle_t c;
    mcpwm_comparator_config_t cfg = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(op, &cfg, &c));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(c, val));
    return c;
}

static void start_quadrature(void)
{
    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t tcfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = RES_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = PERIOD_TICKS,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, &timer));

    mcpwm_oper_handle_t opA, opB;
    mcpwm_operator_config_t ocfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &opA));
    ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &opB));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(opA, timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(opB, timer));

    mcpwm_cmpr_handle_t cA  = new_cmp(opA, PERIOD_TICKS / 2);
    mcpwm_cmpr_handle_t cB1 = new_cmp(opB, PERIOD_TICKS / 4);
    mcpwm_cmpr_handle_t cB2 = new_cmp(opB, 3 * PERIOD_TICKS / 4);

    mcpwm_gen_handle_t genA, genB;
    mcpwm_generator_config_t gcA = { .gen_gpio_num = PIN_A };
    mcpwm_generator_config_t gcB = { .gen_gpio_num = PIN_B };
    ESP_ERROR_CHECK(mcpwm_new_generator(opA, &gcA, &genA));
    ESP_ERROR_CHECK(mcpwm_new_generator(opB, &gcB, &genB));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(genA,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(genA,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cA, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(genB,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cB1, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(genB,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cB2, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

static pcnt_unit_handle_t start_decoder(int pin_a, int pin_b)
{
    pcnt_unit_handle_t unit;
    pcnt_unit_config_t ucfg = { .high_limit = 30000, .low_limit = -30000 };
    ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &unit));

    pcnt_glitch_filter_config_t fcfg = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &fcfg));

    pcnt_chan_config_t c1 = { .edge_gpio_num = pin_a, .level_gpio_num = pin_b };
    pcnt_chan_config_t c2 = { .edge_gpio_num = pin_b, .level_gpio_num = pin_a };
    pcnt_channel_handle_t ch1, ch2;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &c1, &ch1));
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &c2, &ch2));

    /* Decodage x4 : chaque canal compte les deux fronts de son signal,
       l'autre signal inverse le sens quand il est bas. */
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch1,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch1,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch2,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch2,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
    return unit;
}

void app_main(void)
{
    start_quadrature();
    pcnt_unit_handle_t dec = start_decoder(PIN_IN_A, PIN_IN_B);
    printf("quadrature 5 kHz/voie : A=GPIO%d->GPIO%d, B=GPIO%d->GPIO%d\n",
           PIN_A, PIN_IN_A, PIN_B, PIN_IN_B);

    for (;;) {
        int count = 0;
        pcnt_unit_clear_count(dec);
        int64_t t0 = esp_timer_get_time();
        vTaskDelay(pdMS_TO_TICKS(1000));
        pcnt_unit_get_count(dec, &count);
        int64_t t1 = esp_timer_get_time();
        int64_t dt = t1 - t0;
        printf("count=%d  fenetre=%lld us  taux=%.1f comptes/s\n",
               count, (long long)dt, (double)count * 1e6 / (double)dt);
    }
}