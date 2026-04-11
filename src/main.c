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
#define PWM_FORWARD             200U
#define PWM_BRAKE               999U
#define BRAKE_PULSE_MS          60U

#define SENSOR_LOOP_DELAY_MS    4U

/* =========================================================
   NHẬN MÀU DỰA TRÊN LOG THỰC TẾ
   - WHITE: C rất cao (đèn 12V làm trắng sáng mạnh)
   - RED/BLUE: kênh trội rõ
   - GREEN: G chỉ nhỉnh hơn B ít, nên nới điều kiện
   ======================================================== */
#define WHITE_CLEAR_MIN         15000U

#define RED_OVER_G_DELTA        100U
#define RED_OVER_B_DELTA        100U

#define BLUE_OVER_R_DELTA       120U
#define BLUE_OVER_G_DELTA       120U

#define GREEN_OVER_R_DELTA      120U
#define GREEN_OVER_B_DELTA      10U

#define MIN_CLEAR_CHANNEL       15U

/* =========================================================
   THỜI GIAN CUA / DỪNG Ở CÁC ĐIỂM
   ======================================================== */
#define TURN_PULSE_MS           3U

/* Điểm 1: RED -> BLUE */
#define TRANSITION_TURN_MS      370U
#define POINT1_STOP_MS          200U

/* Điểm 2: BLUE -> GREEN */
#define POINT2_STOP_MS          300U
#define POINT2_TURN_MS          55U

/* Điểm 3: GREEN -> BLUE */
#define POINT3_STOP_MS          900U
#define POINT3_TURN_MS          180U

/* Điểm 4: BLUE -> RED */
#define POINT4_STOP_MS          500U
#define POINT4_TURN_MS          300U

/* Chống nhiễu đoạn bám BLUE trước khi cho phép nhận điểm 2 */
#define POINT2_GUARD_MS         2000U
#define POINT2_CONFIRM_COUNT    10U

/* Chống nhiễu đoạn bám GREEN trước khi cho phép nhận điểm 3 */
#define POINT3_GUARD_MS         800U
#define POINT3_CONFIRM_COUNT    6U

/* Chống nhiễu đoạn bám BLUE trước khi cho phép nhận điểm 4 */
#define POINT4_GUARD_MS         600U
#define POINT4_CONFIRM_COUNT    6U

/* =========================================================
   PROFILE TỐC ĐỘ THEO TỪNG ĐOẠN
   =========================================================

   route_stage:
   0 = ĐIỂM BẮT ĐẦU  -> ĐIỂM 1  (bám RED)
   1 = ĐIỂM 1        -> ĐIỂM 2  (bám BLUE)
   2 = ĐIỂM 2        -> ĐIỂM 3  (bám GREEN)
   3 = ĐIỂM 3        -> ĐIỂM 4  (bám BLUE)

   Ý nghĩa:
   - REVERSE_NORMAL / HALF: tốc độ lùi khi mất cả 2 line
   - ALIGN_NORMAL / SEEN_80: tốc độ tiến khi chỉ còn 1 cảm biến thấy line

   NOTE:
   - ĐOẠN CHUNG dùng cho:
       + ĐIỂM BẮT ĐẦU -> ĐIỂM 1
       + SAU ĐIỂM 4 BÁM LẠI RED
   - ĐOẠN RIÊNG 1 dùng cho:
       + ĐIỂM 1 -> ĐIỂM 2
   - ĐOẠN RIÊNG 2 dùng cho:
       + ĐIỂM 2 -> ĐIỂM 3
   ======================================================== */

/* =========================
   ĐOẠN CHUNG:
   START -> POINT1
   SAU POINT4 -> RED
   ========================= */
#define PWM_REVERSE_COMMON_NORMAL      210U
#define PWM_REVERSE_COMMON_HALF        200U
#define PWM_ALIGN_COMMON_NORMAL        440U
#define PWM_ALIGN_COMMON_SEEN_80       380U

/* =========================
   ĐOẠN RIÊNG 1:
   POINT1 -> POINT2
   ========================= */
#define PWM_REVERSE_P1_P2_NORMAL       220U
#define PWM_REVERSE_P1_P2_HALF         150U
#define PWM_ALIGN_P1_P2_NORMAL         180U
#define PWM_ALIGN_P1_P2_SEEN_80        150U

/* =========================
   ĐOẠN RIÊNG 2:
   POINT2 -> POINT3
   ========================= */
#define PWM_REVERSE_P2_P3_NORMAL       200U
#define PWM_REVERSE_P2_P3_HALF         150U
#define PWM_ALIGN_P2_P3_NORMAL         200U
#define PWM_ALIGN_P2_P3_SEEN_80        180U

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

    GPIOB->BSRR = (1U << 12);
    GPIOB->BRR  = (1U << 13);

    GPIOB->BSRR = (1U << 14);
    GPIOB->BRR  = (1U << 15);

    motor_set_left_pwm(left_pwm);
    motor_set_right_pwm(right_pwm);
}

static void motor_turn_left(uint16_t pwm)
{
    GPIOB->BRR  = (1U << 12);
    GPIOB->BSRR = (1U << 13);

    GPIOB->BSRR = (1U << 14);
    GPIOB->BRR  = (1U << 15);

    motor_set_left_pwm(pwm);
    motor_set_right_pwm(pwm);
}

static void motor_turn_right(uint16_t pwm)
{
    GPIOB->BSRR = (1U << 12);
    GPIOB->BRR  = (1U << 13);

    GPIOB->BRR  = (1U << 14);
    GPIOB->BSRR = (1U << 15);

    motor_set_left_pwm(pwm);
    motor_set_right_pwm(pwm);
}

static void motor_reverse_lr(uint16_t left_pwm, uint16_t right_pwm)
{
    if (left_pwm > PWM_PERIOD)  left_pwm = PWM_PERIOD;
    if (right_pwm > PWM_PERIOD) right_pwm = PWM_PERIOD;

    GPIOB->BRR  = (1U << 12);
    GPIOB->BSRR = (1U << 13);

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

static int16_t max3_i16(int16_t a, int16_t b, int16_t c)
{
    int16_t m = a;
    if (b > m) m = b;
    if (c > m) m = c;
    return m;
}

static int16_t min3_i16(int16_t a, int16_t b, int16_t c)
{
    int16_t m = a;
    if (b < m) m = b;
    if (c < m) m = c;
    return m;
}

static void classify_color(const ColorData *raw, SensorResult *out)
{
    int16_t maxv, minv;
    int16_t spread;

    out->raw = *raw;
    calc_norm_rgb(raw, &out->rn, &out->gn, &out->bn);

    maxv = max3_i16(out->rn, out->gn, out->bn);
    minv = min3_i16(out->rn, out->gn, out->bn);
    spread = (int16_t)(maxv - minv);

    out->dist = (uint32_t)spread;
    out->match = 0U;
    out->idx = COLOR_UNKNOWN;

    if (raw->c >= WHITE_CLEAR_MIN) {
        out->match = 1U;
        out->idx = COLOR_WHITE;
        return;
    }

    if ((out->rn > out->gn) && (out->rn > out->bn) &&
        ((out->rn - out->gn) >= RED_OVER_G_DELTA) &&
        ((out->rn - out->bn) >= RED_OVER_B_DELTA) &&
        (out->gn >= MIN_CLEAR_CHANNEL) &&
        (out->bn >= MIN_CLEAR_CHANNEL)) {
        out->match = 1U;
        out->idx = COLOR_RED;
        return;
    }

    if ((out->gn > out->rn) && (out->gn > out->bn) &&
        ((out->gn - out->rn) >= GREEN_OVER_R_DELTA) &&
        ((out->gn - out->bn) >= GREEN_OVER_B_DELTA) &&
        (out->rn >= MIN_CLEAR_CHANNEL) &&
        (out->bn >= MIN_CLEAR_CHANNEL)) {
        out->match = 1U;
        out->idx = COLOR_GREEN;
        return;
    }

    if ((out->bn > out->rn) && (out->bn > out->gn) &&
        ((out->bn - out->rn) >= BLUE_OVER_R_DELTA) &&
        ((out->bn - out->gn) >= BLUE_OVER_G_DELTA) &&
        (out->rn >= MIN_CLEAR_CHANNEL) &&
        (out->gn >= MIN_CLEAR_CHANNEL)) {
        out->match = 1U;
        out->idx = COLOR_BLUE;
        return;
    }
}

static void get_motion_profile(uint8_t route_stage,
                               uint16_t *reverse_normal,
                               uint16_t *reverse_half,
                               uint16_t *align_normal,
                               uint16_t *align_seen80)
{
    if (route_stage == 1U) {
        *reverse_normal = PWM_REVERSE_P1_P2_NORMAL;
        *reverse_half   = PWM_REVERSE_P1_P2_HALF;
        *align_normal   = PWM_ALIGN_P1_P2_NORMAL;
        *align_seen80   = PWM_ALIGN_P1_P2_SEEN_80;
    }
    else if (route_stage == 2U) {
        *reverse_normal = PWM_REVERSE_P2_P3_NORMAL;
        *reverse_half   = PWM_REVERSE_P2_P3_HALF;
        *align_normal   = PWM_ALIGN_P2_P3_NORMAL;
        *align_seen80   = PWM_ALIGN_P2_P3_SEEN_80;
    }
    else {
        *reverse_normal = PWM_REVERSE_COMMON_NORMAL;
        *reverse_half   = PWM_REVERSE_COMMON_HALF;
        *align_normal   = PWM_ALIGN_COMMON_NORMAL;
        *align_seen80   = PWM_ALIGN_COMMON_SEEN_80;
    }
}

static uint8_t is_target_color(ColorIdx idx, ColorIdx target)
{
    return (idx == target) ? 1U : 0U;
}

int main(void)
{
    ColorData right_raw, left_raw;
    SensorResult right_res, left_res;

    uint8_t recovery_mode = 0U;
    int8_t recovery_first_lost_side = 0;
    int8_t recovery_last_lost_side  = 0;

    uint32_t last_left_on_ms  = 0U;
    uint32_t last_right_on_ms = 0U;

    uint8_t left_go;
    uint8_t right_go;
    uint16_t left_reverse_pwm;
    uint16_t right_reverse_pwm;

    uint16_t reverse_normal_cur;
    uint16_t reverse_half_cur;
    uint16_t align_normal_cur;
    uint16_t align_seen80_cur;

    ColorIdx current_target = COLOR_RED;

    /* route_stage:
       0 = START -> POINT1 / SAU POINT4 -> RED
       1 = POINT1 -> POINT2
       2 = POINT2 -> POINT3
       3 = POINT3 -> POINT4
    */
    uint8_t route_stage = 0U;

    uint32_t blue_start_ms = 0U;
    uint8_t point2_seen_count = 0U;

    uint32_t green_start_ms = 0U;
    uint8_t point3_seen_count = 0U;

    uint32_t point4_blue_start_ms = 0U;
    uint8_t point4_seen_count = 0U;

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

        classify_color(&right_raw, &right_res);
        classify_color(&left_raw,  &left_res);

        /* =========================
           ĐIỂM 1: RED -> BLUE
           ========================= */
        if ((route_stage == 0U) && (current_target == COLOR_RED)) {
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
                route_stage = 1U;
                blue_start_ms = millis();
                point2_seen_count = 0U;

                motor_turn_left(PWM_FORWARD);
                delay_ms_tick(TRANSITION_TURN_MS);

                motor_stop();
                recovery_mode = 0U;
                recovery_first_lost_side = 0;
                recovery_last_lost_side = 0;
                continue;
            }
        }

        /* =========================
           ĐIỂM 2: BLUE -> GREEN
           ========================= */
        if ((route_stage == 1U) && (current_target == COLOR_BLUE)) {
            uint8_t marker_left = 0U;
            uint8_t marker_right = 0U;
            uint8_t point2_detect = 0U;

            if ((millis() - blue_start_ms) >= POINT2_GUARD_MS) {

                if (left_res.idx == COLOR_GREEN) {
                    marker_left = 1U;
                }

                if (right_res.idx == COLOR_GREEN) {
                    marker_right = 1U;
                }

                if (marker_left && marker_right) {
                    point2_detect = 1U;
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
                        point2_detect = 1U;
                        point2_seen_count = 0U;
                    }
                }
            } else {
                point2_seen_count = 0U;
            }

            if (point2_detect) {
                motor_brake(PWM_BRAKE);
                delay_ms_tick(POINT2_STOP_MS);
                motor_stop();

                current_target = COLOR_GREEN;
                route_stage = 2U;
                green_start_ms = millis();
                point3_seen_count = 0U;

                motor_turn_left(PWM_FORWARD);
                delay_ms_tick(POINT2_TURN_MS);

                motor_stop();
                recovery_mode = 0U;
                recovery_first_lost_side = 0;
                recovery_last_lost_side = 0;
                continue;
            }
        }

        /* =========================
           ĐIỂM 3: GREEN -> BLUE
           ========================= */
        if ((route_stage == 2U) && (current_target == COLOR_GREEN)) {
            uint8_t marker_left = 0U;
            uint8_t marker_right = 0U;
            uint8_t point3_detect = 0U;

            if ((millis() - green_start_ms) >= POINT3_GUARD_MS) {

                if (left_res.idx == COLOR_BLUE) {
                    marker_left = 1U;
                }

                if (right_res.idx == COLOR_BLUE) {
                    marker_right = 1U;
                }

                if (marker_left && marker_right) {
                    point3_detect = 1U;
                    point3_seen_count = 0U;
                } else {
                    if (marker_left || marker_right) {
                        if (point3_seen_count < 255U) {
                            point3_seen_count++;
                        }
                    } else {
                        point3_seen_count = 0U;
                    }

                    if (point3_seen_count >= POINT3_CONFIRM_COUNT) {
                        point3_detect = 1U;
                        point3_seen_count = 0U;
                    }
                }
            } else {
                point3_seen_count = 0U;
            }

            if (point3_detect) {
                motor_brake(PWM_BRAKE);
                delay_ms_tick(POINT3_STOP_MS);
                motor_stop();

                current_target = COLOR_BLUE;
                route_stage = 3U;
                point4_blue_start_ms = millis();
                point4_seen_count = 0U;

                motor_turn_left(PWM_FORWARD);
                delay_ms_tick(POINT3_TURN_MS);

                motor_stop();
                recovery_mode = 0U;
                recovery_first_lost_side = 0;
                recovery_last_lost_side = 0;
                continue;
            }
        }

        /* =========================
           ĐIỂM 4: BLUE -> RED
           Điều kiện: có RED và GREEN
           Sau đó quay 0.5s rồi bám RED
           Dùng lại tốc độ đoạn START -> POINT1
           ========================= */
        if ((route_stage == 3U) && (current_target == COLOR_BLUE)) {
            uint8_t marker_left = 0U;
            uint8_t marker_right = 0U;
            uint8_t point4_detect = 0U;

            if ((millis() - point4_blue_start_ms) >= POINT4_GUARD_MS) {

                if ((left_res.idx == COLOR_RED) || (left_res.idx == COLOR_GREEN)) {
                    marker_left = 1U;
                }

                if ((right_res.idx == COLOR_RED) || (right_res.idx == COLOR_GREEN)) {
                    marker_right = 1U;
                }

                if (marker_left && marker_right) {
                    point4_detect = 1U;
                    point4_seen_count = 0U;
                } else {
                    if (marker_left || marker_right) {
                        if (point4_seen_count < 255U) {
                            point4_seen_count++;
                        }
                    } else {
                        point4_seen_count = 0U;
                    }

                    if (point4_seen_count >= POINT4_CONFIRM_COUNT) {
                        point4_detect = 1U;
                        point4_seen_count = 0U;
                    }
                }
            } else {
                point4_seen_count = 0U;
            }

            if (point4_detect) {
                motor_brake(PWM_BRAKE);
                delay_ms_tick(POINT4_STOP_MS);
                motor_stop();

                current_target = COLOR_RED;
                route_stage = 0U; /* dùng lại profile tốc độ của START -> POINT1 */

                motor_turn_left(PWM_FORWARD);
                delay_ms_tick(POINT4_TURN_MS);

                motor_stop();
                recovery_mode = 0U;
                recovery_first_lost_side = 0;
                recovery_last_lost_side = 0;
                continue;
            }
        }

        left_go  = is_target_color(left_res.idx, current_target);
        right_go = is_target_color(right_res.idx, current_target);

        get_motion_profile(route_stage,
                           &reverse_normal_cur,
                           &reverse_half_cur,
                           &align_normal_cur,
                           &align_seen80_cur);

        if (left_go)  last_left_on_ms  = millis();
        if (right_go) last_right_on_ms = millis();

        if (recovery_mode) {
            if (left_go && right_go) {
                recovery_mode = 0U;
                recovery_first_lost_side = 0;
                recovery_last_lost_side = 0;
                motor_forward(PWM_FORWARD);
            }
            else if (left_go && !right_go) {
                motor_forward_lr(align_seen80_cur, align_normal_cur);
            }
            else if (!left_go && right_go) {
                motor_forward_lr(align_normal_cur, align_seen80_cur);
            }
            else {
                left_reverse_pwm  = reverse_normal_cur;
                right_reverse_pwm = reverse_normal_cur;

                if ((recovery_first_lost_side == -1) && (recovery_last_lost_side == +1)) {
                    left_reverse_pwm  = reverse_half_cur;
                    right_reverse_pwm = reverse_normal_cur;
                } else if ((recovery_first_lost_side == +1) && (recovery_last_lost_side == -1)) {
                    left_reverse_pwm  = reverse_normal_cur;
                    right_reverse_pwm = reverse_half_cur;
                }

                motor_reverse_lr(left_reverse_pwm, right_reverse_pwm);
            }
        }
        else {
            if (left_go && right_go) {
                motor_forward(PWM_FORWARD);
            }
            else if (!left_go && right_go) {
                motor_forward_lr(align_normal_cur, align_seen80_cur);
            }
            else if (left_go && !right_go) {
                motor_forward_lr(align_seen80_cur, align_normal_cur);
            }
            else {
                if (last_left_on_ms < last_right_on_ms) {
                    recovery_first_lost_side = -1;
                    recovery_last_lost_side  = +1;
                    left_reverse_pwm  = reverse_half_cur;
                    right_reverse_pwm = reverse_normal_cur;
                } else if (last_right_on_ms < last_left_on_ms) {
                    recovery_first_lost_side = +1;
                    recovery_last_lost_side  = -1;
                    left_reverse_pwm  = reverse_normal_cur;
                    right_reverse_pwm = reverse_half_cur;
                } else {
                    recovery_first_lost_side = 0;
                    recovery_last_lost_side  = 0;
                    left_reverse_pwm  = reverse_normal_cur;
                    right_reverse_pwm = reverse_normal_cur;
                }

                recovery_mode = 1U;
                motor_reverse_lr(left_reverse_pwm, right_reverse_pwm);
            }
        }

        delay_ms_tick(SENSOR_LOOP_DELAY_MS);
    }
}