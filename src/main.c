#include "stm32f10x.h"
#include "usart.h"
#include "i2c.h"
#include "tcs34725.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/* =========================================================
   SENSOR MAP
   - RIGHT SENSOR = I2C1
   - LEFT  SENSOR = I2C2

   MOTOR MAP (L298N)
   - PB0  -> ENA   (TIM3_CH3 PWM)
   - PB1  -> ENB   (TIM3_CH4 PWM)
   - PB12 -> IN1
   - PB13 -> IN2
   - PB14 -> IN3
   - PB15 -> IN4
   ========================================================= */

#define RIGHT_SENSOR_BUS        I2C1
#define LEFT_SENSOR_BUS         I2C2

#define DEBUG_UART              0

#define PWM_PERIOD              999U
#define PWM_FORWARD             600U
#define PWM_BRAKE               999U
#define BRAKE_PULSE_MS          60U

#define PWM_REVERSE_NORMAL      210U
#define PWM_REVERSE_HALF        200U

/* dùng chung cho lúc bị lệch và lúc recovery */
#define PWM_ALIGN_NORMAL        430U
#define PWM_ALIGN_SEEN_80       380U

#define MATCH_THRESHOLD         18000UL

#define SENSOR_LOOP_DELAY_MS    4U
#define PRINT_INTERVAL_MS       500U

/* =========================================================
   CÁC THÔNG SỐ ĐIỀU CHỈNH THỜI GIAN CUA (RẼ)
   ======================================================== */
#define TURN_PULSE_MS           3U
#define TRANSITION_TURN_MS      180U
#define POINT1_STOP_MS          200U
#define POINT3_STOP_MS          200U
#define POINT3_TURN_MS          200U

/* Chống nhiễu đoạn bám BLUE trước khi cho phép nhận điểm tiếp theo */
#define POINT2_GUARD_MS         3500U
#define POINT2_CONFIRM_COUNT    10U

typedef struct {
    int r;
    int g;
    int b;
    uint16_t c;
} ColorData;

typedef enum {
    COLOR_RED = 0,
    COLOR_BLUE,
    COLOR_GREEN,
    COLOR_WHITE,
    COLOR_UNKNOWN = 255
} ColorIdx;

typedef struct {
    ColorData raw;
    int16_t rn;
    int16_t gn;
    int16_t bn;
    uint32_t dist;
    uint8_t match;
    ColorIdx idx;
} SensorResult;

static volatile uint32_t g_ms_ticks = 0;

/* RIGHT SENSOR */
static const int16_t allowed_norm_right[4][3] = {
    {716, 138, 142},
    {328, 311, 357},
    {335, 363, 298},
    {466, 300, 230}
};

/* LEFT SENSOR */
static const int16_t allowed_norm_left[4][3] = {
    {694, 157, 147},
    {305, 326, 368},
    {324, 376, 297},
    {420, 327, 251}
};

void SysTick_Handler(void)
{
    g_ms_ticks++;
}

static uint32_t millis(void)
{
    return g_ms_ticks;
}

static void delay_ms_tick(uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms);
}

static void uart_send_len(const char *s, uint16_t len)
{
    USART_Send_bytes(s, len);
}

static void uart_printf(const char *fmt, ...)
{
#if DEBUG_UART
    char buf[256];
    int n;
    va_list ap;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n > (int)sizeof(buf)) {
            n = (int)sizeof(buf);
        }
        uart_send_len(buf, (uint16_t)n);
    }
#else
    (void)fmt;
#endif
}

static void gpio_set_output_pp_50mhz(GPIO_TypeDef *GPIOx, uint8_t pin)
{
    volatile uint32_t *reg;
    uint32_t shift;
    uint32_t val;

    if (pin < 8U) {
        reg = &GPIOx->CRL;
        shift = pin * 4U;
    } else {
        reg = &GPIOx->CRH;
        shift = (pin - 8U) * 4U;
    }

    val = *reg;
    val &= ~(0xFU << shift);
    val |=  (0x3U << shift);
    *reg = val;
}

static void gpio_set_af_pp_50mhz(GPIO_TypeDef *GPIOx, uint8_t pin)
{
    volatile uint32_t *reg;
    uint32_t shift;
    uint32_t val;

    if (pin < 8U) {
        reg = &GPIOx->CRL;
        shift = pin * 4U;
    } else {
        reg = &GPIOx->CRH;
        shift = (pin - 8U) * 4U;
    }

    val = *reg;
    val &= ~(0xFU << shift);
    val |=  (0xBU << shift);
    *reg = val;
}

static void motor_pwm_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    gpio_set_af_pp_50mhz(GPIOB, 0);
    gpio_set_af_pp_50mhz(GPIOB, 1);

    gpio_set_output_pp_50mhz(GPIOB, 12);
    gpio_set_output_pp_50mhz(GPIOB, 13);
    gpio_set_output_pp_50mhz(GPIOB, 14);
    gpio_set_output_pp_50mhz(GPIOB, 15);

    GPIOB->BRR = (1U << 12) | (1U << 13) | (1U << 14) | (1U << 15);

    TIM3->PSC = 71U;
    TIM3->ARR = PWM_PERIOD;

    TIM3->CCR3 = 0U;
    TIM3->CCR4 = 0U;

    TIM3->CCMR2 &= ~(
        TIM_CCMR2_OC3M |
        TIM_CCMR2_OC3PE |
        TIM_CCMR2_OC4M |
        TIM_CCMR2_OC4PE
    );

    TIM3->CCMR2 |=
        (6U << 4)  | TIM_CCMR2_OC3PE |
        (6U << 12) | TIM_CCMR2_OC4PE;

    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CR1  |= TIM_CR1_ARPE;
    TIM3->EGR   = TIM_EGR_UG;
    TIM3->CR1  |= TIM_CR1_CEN;
}

static void motor_set_left_pwm(uint16_t pwm)
{
    if (pwm > PWM_PERIOD) pwm = PWM_PERIOD;
    TIM3->CCR3 = pwm;
}

static void motor_set_right_pwm(uint16_t pwm)
{
    if (pwm > PWM_PERIOD) pwm = PWM_PERIOD;
    TIM3->CCR4 = pwm;
}

static void motor_forward(uint16_t pwm)
{
    GPIOB->BSRR = (1U << 12);
    GPIOB->BRR  = (1U << 13);

    GPIOB->BSRR = (1U << 14);
    GPIOB->BRR  = (1U << 15);

    motor_set_left_pwm(pwm);
    motor_set_right_pwm(pwm);
}

static void motor_forward_lr(uint16_t left_pwm, uint16_t right_pwm)
{
    if (left_pwm > PWM_PERIOD)  left_pwm = PWM_PERIOD;
    if (right_pwm > PWM_PERIOD) right_pwm = PWM_PERIOD;

    /* Bánh trái tiến: IN1=1, IN2=0 */
    GPIOB->BSRR = (1U << 12);
    GPIOB->BRR  = (1U << 13);

    /* Bánh phải tiến: IN3=1, IN4=0 */
    GPIOB->BSRR = (1U << 14);
    GPIOB->BRR  = (1U << 15);

    motor_set_left_pwm(left_pwm);
    motor_set_right_pwm(right_pwm);
}

static void motor_turn_left(uint16_t pwm)
{
    /* Bánh trái lùi, bánh phải tiến */
    GPIOB->BRR  = (1U << 12);
    GPIOB->BSRR = (1U << 13);

    GPIOB->BSRR = (1U << 14);
    GPIOB->BRR  = (1U << 15);

    motor_set_left_pwm(pwm);
    motor_set_right_pwm(pwm);
}

static void motor_turn_right(uint16_t pwm)
{
    /* Bánh trái tiến, bánh phải lùi */
    GPIOB->BSRR = (1U << 12);
    GPIOB->BRR  = (1U << 13);

    GPIOB->BRR  = (1U << 14);
    GPIOB->BSRR = (1U << 15);

    motor_set_left_pwm(pwm);
    motor_set_right_pwm(pwm);
}

/* Bánh phải tiến, bánh trái đứng yên */
static void motor_right_forward_left_stop(uint16_t pwm)
{
    /* Bánh trái đứng yên (IN1=0, IN2=0) */
    GPIOB->BRR  = (1U << 12) | (1U << 13);

    /* Bánh phải tiến (IN3=1, IN4=0) */
    GPIOB->BSRR = (1U << 14);
    GPIOB->BRR  = (1U << 15);

    motor_set_left_pwm(0U);
    motor_set_right_pwm(pwm);
}

/* Lùi với tốc độ riêng từng bánh */
static void motor_reverse_lr(uint16_t left_pwm, uint16_t right_pwm)
{
    if (left_pwm > PWM_PERIOD)  left_pwm = PWM_PERIOD;
    if (right_pwm > PWM_PERIOD) right_pwm = PWM_PERIOD;

    /* Bánh trái lùi: IN1=0, IN2=1 */
    GPIOB->BRR  = (1U << 12);
    GPIOB->BSRR = (1U << 13);

    /* Bánh phải lùi: IN3=0, IN4=1 */
    GPIOB->BRR  = (1U << 14);
    GPIOB->BSRR = (1U << 15);

    motor_set_left_pwm(left_pwm);
    motor_set_right_pwm(right_pwm);
}

static void motor_stop(void)
{
    GPIOB->BRR = (1U << 12) | (1U << 13) | (1U << 14) | (1U << 15);
    motor_set_left_pwm(0U);
    motor_set_right_pwm(0U);
}

static void motor_brake(uint16_t pwm)
{
    if (pwm > PWM_PERIOD) pwm = PWM_PERIOD;

    GPIOB->BSRR = (1U << 12) | (1U << 13) | (1U << 14) | (1U << 15);

    motor_set_left_pwm(pwm);
    motor_set_right_pwm(pwm);
}

static void motor_brake_pulse_then_stop(uint16_t pwm, uint32_t brake_ms)
{
    motor_brake(pwm);
    delay_ms_tick(brake_ms);
    motor_stop();
}

static void Sensor_Init_All(void)
{
    I2C_Peripheral_Init(RIGHT_SENSOR_BUS);
    delay_ms_tick(10);
    tcs3272_init(RIGHT_SENSOR_BUS);
    delay_ms_tick(10);

    I2C_Peripheral_Init(LEFT_SENSOR_BUS);
    delay_ms_tick(10);
    tcs3272_init(LEFT_SENSOR_BUS);
    delay_ms_tick(10);
}

static void read_sensor_raw(I2C_TypeDef *I2Cx, ColorData *d)
{
    getRGB(I2Cx, &d->r, &d->g, &d->b, &d->c);
}

static void calc_norm_rgb(const ColorData *src, int16_t *rn, int16_t *gn, int16_t *bn)
{
    int32_t sum = src->r + src->g + src->b;

    if (sum <= 0) {
        *rn = 0;
        *gn = 0;
        *bn = 0;
        return;
    }

    *rn = (int16_t)((src->r * 1000L) / sum);
    *gn = (int16_t)((src->g * 1000L) / sum);
    *bn = (int16_t)((src->b * 1000L) / sum);
}

static uint32_t dist_rgb3(int16_t a1, int16_t a2, int16_t a3,
                          int16_t b1, int16_t b2, int16_t b3)
{
    int32_t d1 = (int32_t)a1 - (int32_t)b1;
    int32_t d2 = (int32_t)a2 - (int32_t)b2;
    int32_t d3 = (int32_t)a3 - (int32_t)b3;

    return (uint32_t)(d1 * d1 + d2 * d2 + d3 * d3);
}

static void classify_color(const ColorData *raw,
                           const int16_t ref_norm[4][3],
                           SensorResult *out)
{
    uint8_t i;
    uint32_t best_dist = 0xFFFFFFFFUL;
    uint8_t best_idx = COLOR_UNKNOWN;

    out->raw = *raw;
    calc_norm_rgb(raw, &out->rn, &out->gn, &out->bn);

    for (i = 0; i < 4U; i++) {
        uint32_t d = dist_rgb3(out->rn, out->gn, out->bn,
                               ref_norm[i][0], ref_norm[i][1], ref_norm[i][2]);

        if (d < best_dist) {
            best_dist = d;
            best_idx = i;
        }
    }

    out->dist = best_dist;

    if (best_dist <= MATCH_THRESHOLD) {
        out->match = 1U;
        out->idx = (ColorIdx)best_idx;
    } else {
        out->match = 0U;
        out->idx = COLOR_UNKNOWN;
    }
}

static const char *color_to_str(ColorIdx idx)
{
    if (idx == COLOR_RED)   return "RED";
    if (idx == COLOR_BLUE)  return "BLUE";
    if (idx == COLOR_GREEN) return "GREEN";
    if (idx == COLOR_WHITE) return "WHITE";
    return "UNKNOWN";
}

static uint8_t is_target_color(ColorIdx idx, ColorIdx target)
{
    return (idx == target) ? 1U : 0U;
}

int main(void)
{
    ColorData right_raw, left_raw;
    SensorResult right_res, left_res;
    uint8_t was_moving = 0;

    uint8_t recovery_mode = 0U;
    int8_t recovery_first_lost_side = 0; /* -1 = LEFT, +1 = RIGHT */
    int8_t recovery_last_lost_side  = 0; /* -1 = LEFT, +1 = RIGHT */

    uint32_t last_left_on_ms  = 0U;
    uint32_t last_right_on_ms = 0U;

    uint8_t left_go;
    uint8_t right_go;
    uint16_t left_reverse_pwm;
    uint16_t right_reverse_pwm;

    ColorIdx current_target = COLOR_RED;

    /* Chống nhiễu đoạn bám BLUE */
    uint32_t blue_start_ms = 0U;
    uint8_t point2_seen_count = 0U;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);

    Usart_Int(115200);

    Sensor_Init_All();
    motor_pwm_init();
    motor_stop();

    last_left_on_ms  = millis();
    last_right_on_ms = millis();

    while (1) {
        read_sensor_raw(RIGHT_SENSOR_BUS, &right_raw);
        read_sensor_raw(LEFT_SENSOR_BUS,  &left_raw);

        classify_color(&right_raw, allowed_norm_right, &right_res);
        classify_color(&left_raw,  allowed_norm_left,  &left_res);

        /* =====================================================
           DIEM 1:
           Dang bam RED, neu thay BLUE hoac GREEN
           (1 cam bien hoac ca 2 cam bien) thi:
           - khoa banh dung 0.2s
           - quay bang 1 banh tien, 1 banh lui
           - chuyen sang bam BLUE
           ===================================================== */
        if (current_target == COLOR_RED) {
            uint8_t point1_detect = 0U;

            if ((left_res.idx == COLOR_BLUE)  || (right_res.idx == COLOR_BLUE) ||
                (left_res.idx == COLOR_GREEN) || (right_res.idx == COLOR_GREEN)) {
                point1_detect = 1U;
            }

            if (point1_detect) {
                motor_brake(PWM_BRAKE);
                delay_ms_tick(POINT1_STOP_MS);
                motor_stop();

                current_target = COLOR_BLUE;
                blue_start_ms = millis();
                point2_seen_count = 0U;

                motor_turn_left(PWM_FORWARD);
                delay_ms_tick(TRANSITION_TURN_MS);

                motor_stop();
                was_moving = 0U;
                recovery_mode = 0U;
                recovery_first_lost_side = 0;
                recovery_last_lost_side = 0;

                continue;
            }
        }

        /* =====================================================
           DIEM 3:
           Dang bam BLUE, neu thay RED hoac GREEN
           (1 cam bien hoac ca 2 cam bien) thi:
           - khoa banh dung 0.2s
           - quay bang 1 banh tien, 1 banh lui
           - chuyen sang bam GREEN

           Chong nhieu:
           - sau diem 1 phai cho 2s moi duoc nhan xanh la/do
           - sau do neu chi thay 1 ben thi phai lap lai nhieu lan
           ===================================================== */
        if (current_target == COLOR_BLUE) {
            uint8_t marker_left = 0U;
            uint8_t marker_right = 0U;
            uint8_t point3_detect = 0U;

            if ((millis() - blue_start_ms) >= POINT2_GUARD_MS) {

                if ((left_res.idx == COLOR_RED) || (left_res.idx == COLOR_GREEN)) {
                    marker_left = 1U;
                }

                if ((right_res.idx == COLOR_RED) || (right_res.idx == COLOR_GREEN)) {
                    marker_right = 1U;
                }

                if (marker_left && marker_right) {
                    point3_detect = 1U;
                    point2_seen_count = 0U;
                } else {
                    if (marker_left || marker_right) {
                        if (point2_seen_count < 255U) {
                            point2_seen_count++;
                        }
                    } else {
                        point2_seen_count = 0U;
                    }

                    if (point2_seen_count >= POINT2_CONFIRM_COUNT) {
                        point3_detect = 1U;
                        point2_seen_count = 0U;
                    }
                }
            } else {
                point2_seen_count = 0U;
            }

            if (point3_detect) {
                motor_brake(PWM_BRAKE);
                delay_ms_tick(POINT3_STOP_MS);
                motor_stop();

                current_target = COLOR_GREEN;

                motor_turn_left(PWM_FORWARD);
                delay_ms_tick(POINT3_TURN_MS);

                motor_stop();
                was_moving = 0U;
                recovery_mode = 0U;
                recovery_first_lost_side = 0;
                recovery_last_lost_side = 0;

                continue;
            }
        }

        left_go  = is_target_color(left_res.idx, current_target);
        right_go = is_target_color(right_res.idx, current_target);

        if (left_go)  last_left_on_ms  = millis();
        if (right_go) last_right_on_ms = millis();

        if (recovery_mode) {
            if (left_go && right_go) {
                recovery_mode = 0U;
                recovery_first_lost_side = 0;
                recovery_last_lost_side = 0;

                motor_forward(PWM_FORWARD);
                was_moving = 1U;
            }
            else if (left_go && !right_go) {
                /* LEFT thấy line, RIGHT chưa thấy */
                motor_forward_lr(PWM_ALIGN_SEEN_80, PWM_ALIGN_NORMAL);
                was_moving = 1U;
            }
            else if (!left_go && right_go) {
                /* RIGHT thấy line, LEFT chưa thấy */
                motor_forward_lr(PWM_ALIGN_NORMAL, PWM_ALIGN_SEEN_80);
                was_moving = 1U;
            }
            else {
                left_reverse_pwm  = PWM_REVERSE_NORMAL;
                right_reverse_pwm = PWM_REVERSE_NORMAL;

                if ((recovery_first_lost_side == -1) && (recovery_last_lost_side == +1)) {
                    left_reverse_pwm  = PWM_REVERSE_HALF;
                    right_reverse_pwm = PWM_REVERSE_NORMAL;
                } else if ((recovery_first_lost_side == +1) && (recovery_last_lost_side == -1)) {
                    left_reverse_pwm  = PWM_REVERSE_NORMAL;
                    right_reverse_pwm = PWM_REVERSE_HALF;
                }

                motor_reverse_lr(left_reverse_pwm, right_reverse_pwm);
                was_moving = 1U;
            }
        }
        else {
            if (left_go && right_go) {
                motor_forward(PWM_FORWARD);
                was_moving = 1U;
            }
            else if (!left_go && right_go) {
                /* RIGHT thấy line, LEFT chưa thấy */
                motor_forward_lr(PWM_ALIGN_NORMAL, PWM_ALIGN_SEEN_80);
                was_moving = 1U;
            }
            else if (left_go && !right_go) {
                /* LEFT thấy line, RIGHT chưa thấy */
                motor_forward_lr(PWM_ALIGN_SEEN_80, PWM_ALIGN_NORMAL);
                was_moving = 1U;
            }
            else {
                /* Cả 2 đều mất line -> vào recovery lùi */
                if (last_left_on_ms < last_right_on_ms) {
                    recovery_first_lost_side = -1; /* LEFT mất trước */
                    recovery_last_lost_side  = +1; /* RIGHT mất sau */
                    left_reverse_pwm  = PWM_REVERSE_HALF;
                    right_reverse_pwm = PWM_REVERSE_NORMAL;
                } else if (last_right_on_ms < last_left_on_ms) {
                    recovery_first_lost_side = +1; /* RIGHT mất trước */
                    recovery_last_lost_side  = -1; /* LEFT mất sau */
                    left_reverse_pwm  = PWM_REVERSE_NORMAL;
                    right_reverse_pwm = PWM_REVERSE_HALF;
                } else {
                    recovery_first_lost_side = 0;
                    recovery_last_lost_side  = 0;
                    left_reverse_pwm  = PWM_REVERSE_NORMAL;
                    right_reverse_pwm = PWM_REVERSE_NORMAL;
                }

                recovery_mode = 1U;
                motor_reverse_lr(left_reverse_pwm, right_reverse_pwm);
                was_moving = 1U;
            }
        }

        delay_ms_tick(SENSOR_LOOP_DELAY_MS);
    }
}