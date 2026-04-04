// bám line ngon nhưng chưa tối ưu tốc độ
#include "stm32f10x.h"
#include "usart.h"
#include "i2c.h"
#include "tcs34725.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

typedef enum {
    MOVE_STOP = 0,
    MOVE_FORWARD,
    MOVE_LEFT,
    MOVE_RIGHT
} MoveState;

typedef enum {
    PHASE_FOLLOW_RED = 0,
    PHASE_CHAM1_BRAKE,
    PHASE_TURN_RIGHT_AFTER_CHAM1,
    PHASE_TRANS_BLUE_TO_GREEN,
    PHASE_FOLLOW_GREEN,
    PHASE_CHAM3_BRAKE,
    PHASE_TURN_RIGHT_AFTER_CHAM3,
    PHASE_FOLLOW_BLUE_AFTER_CHAM3
} RunPhase;

typedef enum {
    LAST_SEEN_NONE = 0,
    LAST_SEEN_LEFT,
    LAST_SEEN_RIGHT
} LastSeenSide;

typedef struct {
    int8_t idx;
    uint8_t match;
    uint8_t valid;
} RemoteColor;

typedef struct {
    int r;
    int g;
    int b;
    uint16_t c;
} ColorData;

typedef struct {
    ColorData raw;
    int16_t rn;
    int16_t gn;
    int16_t bn;
    uint32_t dist;
    int8_t idx;
    uint8_t match;
    uint8_t valid;
} SensorResult;

static const int16_t allowed_norm[3][3] = {
    {744, 137, 118},   // 0 = RED
    {278, 343, 377},   // 1 = BLUE
    {292, 446, 261}    // 2 = GREEN
};

#define MATCH_THRESHOLD                 15000UL
#define SENSOR_LOOP_DELAY_MS            8U
#define DEBUG_PRINT_MS                  150U
#define DEBUG_UART                      1

#define COLOR_RED                       0
#define COLOR_BLUE                      1
#define COLOR_GREEN                     2

/* =========================================================
   REMOTE COLOR INPUT FROM SENDER BOARD
   sender:   PA0=BIT0, PA1=BIT1, PA2=VALID
   receiver: PA3=BIT0, PA4=BIT1, PA5=VALID
   ========================================================= */
#define REMOTE_PORT                     GPIOA
#define REMOTE_PIN_BIT0                 GPIO_Pin_3
#define REMOTE_PIN_BIT1                 GPIO_Pin_4
#define REMOTE_PIN_VALID                GPIO_Pin_5

/* =========================================================
   MOTOR MAP - L298N
   ENA = PB0 = TIM3_CH3  -> LEFT WHEEL PWM
   ENB = PB1 = TIM3_CH4  -> RIGHT WHEEL PWM
   IN1 = PB12
   IN2 = PB13
   IN3 = PB14
   IN4 = PB15
   ========================================================= */
#define PWM_PERIOD                      999U
#define SPEED_SCALE_PERCENT             30U
#define MIN_EFFECTIVE_PWM               140U
#define PWM_FORWARD                     850U
#define PWM_TURN_OUTER                  680U
#define PWM_TURN_INNER                  500U
#define BRAKE_PWM                       PWM_PERIOD
#define PWM_SPIN_RIGHT                  850U

/* ====== THOI GIAN CAC CHAM ====== */
#define CHAM1_BRAKE_MS                  150U
#define TURN_RIGHT_AFTER_CHAM1_MS       730U
#define CHAM3_ARM_DELAY_MS              500U
#define CHAM3_BRAKE_MS                  120U
#define TURN_RIGHT_AFTER_CHAM3_MS       300U

static volatile uint32_t g_ms_ticks = 0;
static uint32_t g_last_print_ms = 0;
static SensorResult g_local_sensor;
static int8_t g_last_remote_idx = -2;
static uint8_t g_last_remote_valid = 2;
static int8_t g_last_local_idx = -2;
static uint8_t g_last_local_valid = 2;
static int8_t g_last_phase = -1;

static RunPhase g_phase = PHASE_FOLLOW_RED;
static uint32_t g_phase_start_ms = 0;
static uint8_t g_seen_red = 0;
static uint8_t g_green_locked = 0;
static LastSeenSide g_last_seen_side = LAST_SEEN_NONE;

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
    char buf[128];
    int n;
    va_list ap;
    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        if (n > (int)sizeof(buf)) n = (int)sizeof(buf);
        uart_send_len(buf, (uint16_t)n);
    }
#else
    (void)fmt;
#endif
}

static void GPIO_Init_All(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    /* PB0, PB1 = AF push-pull for TIM3 PWM */
    GPIOB->CRL &= ~((0xFU << 0) | (0xFU << 4));
    GPIOB->CRL |=  ((0xBU << 0) | (0xBU << 4));

    /* PB12..PB15 = output push-pull */
    GPIOB->CRH &= ~((0xFU << 16) | (0xFU << 20) | (0xFU << 24) | (0xFU << 28));
    GPIOB->CRH |=  ((0x2U << 16) | (0x2U << 20) | (0x2U << 24) | (0x2U << 28));

    /* PA3..PA5 = input pull-up for remote color */
    GPIOA->CRL &= ~((0xFU << 12) | (0xFU << 16) | (0xFU << 20));
    GPIOA->CRL |=  ((0x8U << 12) | (0x8U << 16) | (0x8U << 20));
    GPIOA->ODR |= (1U << 3) | (1U << 4) | (1U << 5);

    GPIOB->ODR &= ~((1U << 12) | (1U << 13) | (1U << 14) | (1U << 15));
}

static void PWM_TIM3_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 71;
    TIM3->ARR = PWM_PERIOD;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    TIM3->CCMR2 = 0;
    TIM3->CCMR2 |= TIM_CCMR2_OC3PE | (6U << 4);
    TIM3->CCMR2 |= TIM_CCMR2_OC4PE | (6U << 12);
    TIM3->CCER  |= TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CR1   |= TIM_CR1_ARPE;
    TIM3->EGR   |= TIM_EGR_UG;
    TIM3->CR1   |= TIM_CR1_CEN;
}

static uint16_t clamp_pwm(uint16_t v)
{
    return (v > PWM_PERIOD) ? PWM_PERIOD : v;
}

static uint16_t apply_speed_scale(uint16_t v)
{
    uint32_t scaled = ((uint32_t)v * SPEED_SCALE_PERCENT) / 100U;
    if ((scaled > 0U) && (scaled < MIN_EFFECTIVE_PWM)) scaled = MIN_EFFECTIVE_PWM;
    if (scaled > PWM_PERIOD) scaled = PWM_PERIOD;
    return (uint16_t)scaled;
}

static void Motor_SetPWM(uint16_t left_pwm, uint16_t right_pwm)
{
    TIM3->CCR3 = apply_speed_scale(clamp_pwm(left_pwm));
    TIM3->CCR4 = apply_speed_scale(clamp_pwm(right_pwm));
}

static void Motor_SetBothForwardDirection(void)
{
    GPIOB->ODR |=  (1U << 12);
    GPIOB->ODR &= ~(1U << 13);
    GPIOB->ODR |=  (1U << 14);
    GPIOB->ODR &= ~(1U << 15);
}

static void Motor_SetSpinRightDirection(void)
{
    /* LEFT wheel backward, RIGHT wheel forward */
    GPIOB->ODR &= ~(1U << 12);
    GPIOB->ODR |=  (1U << 13);

    GPIOB->ODR |=  (1U << 14);
    GPIOB->ODR &= ~(1U << 15);
}

static void Motor_Stop(void)
{
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    GPIOB->ODR &= ~((1U << 12) | (1U << 13) | (1U << 14) | (1U << 15));
}

static void Motor_BrakeLock(void)
{
    GPIOB->ODR |=  (1U << 12);
    GPIOB->ODR |=  (1U << 13);
    GPIOB->ODR |=  (1U << 14);
    GPIOB->ODR |=  (1U << 15);

    TIM3->CCR3 = BRAKE_PWM;
    TIM3->CCR4 = BRAKE_PWM;
}

static void Motor_Forward(void)
{
    Motor_SetBothForwardDirection();
    Motor_SetPWM(PWM_FORWARD, PWM_FORWARD);
}

static void Motor_Bias_Left(void)
{
    Motor_SetBothForwardDirection();
    Motor_SetPWM(PWM_TURN_INNER, PWM_TURN_OUTER);
}

static void Motor_Bias_Right(void)
{
    Motor_SetBothForwardDirection();
    Motor_SetPWM(PWM_TURN_OUTER, PWM_TURN_INNER);
}

static void Motor_SpinRight(void)
{
    Motor_SetSpinRightDirection();
    Motor_SetPWM(PWM_SPIN_RIGHT, PWM_SPIN_RIGHT);
}

static void remote_read_color(RemoteColor *res)
{
    uint32_t idr = REMOTE_PORT->IDR;
    uint8_t valid = (idr & REMOTE_PIN_VALID) ? 1U : 0U;
    uint8_t bit0  = (idr & REMOTE_PIN_BIT0)  ? 1U : 0U;
    uint8_t bit1  = (idr & REMOTE_PIN_BIT1)  ? 1U : 0U;
    uint8_t code  = (uint8_t)((bit1 << 1) | bit0);

    if (!valid) {
        res->idx = -1;
        res->match = 0;
        res->valid = 0;
        return;
    }

    if (code <= 2U) {
        res->idx = (int8_t)code;
        res->match = 1;
        res->valid = 1;
    } else {
        res->idx = -1;
        res->match = 0;
        res->valid = 0;
    }
}

static void read_sensor_raw(I2C_TypeDef *I2Cx, ColorData *d)
{
    getRGB(I2Cx, &d->r, &d->g, &d->b, &d->c);
}

static void calc_norm_rgb(const ColorData *src, int16_t *rn, int16_t *gn, int16_t *bn)
{
    int32_t sum = src->r + src->g + src->b;
    if (sum <= 0) {
        *rn = 0; *gn = 0; *bn = 0;
        return;
    }
    *rn = (int16_t)((src->r * 1000L) / sum);
    *gn = (int16_t)((src->g * 1000L) / sum);
    *bn = (int16_t)((src->b * 1000L) / sum);
}

static int8_t classify_from_norm(int16_t rn, int16_t gn, int16_t bn, uint32_t *best_dist)
{
    uint8_t i;
    int8_t best_idx = -1;
    uint32_t min_dist = 0xFFFFFFFFUL;

    for (i = 0; i < 3U; i++) {
        int32_t dr = rn - allowed_norm[i][0];
        int32_t dg = gn - allowed_norm[i][1];
        int32_t db = bn - allowed_norm[i][2];
        uint32_t d = (uint32_t)(dr * dr + dg * dg + db * db);
        if (d < min_dist) {
            min_dist = d;
            best_idx = (int8_t)i;
        }
    }

    *best_dist = min_dist;
    return best_idx;
}

static void local_sensor_read_and_classify(SensorResult *res)
{
    read_sensor_raw(I2C1, &res->raw);

    if ((res->raw.r == 0) && (res->raw.g == 0) && (res->raw.b == 0) && (res->raw.c == 0)) {
        res->rn = 0;
        res->gn = 0;
        res->bn = 0;
        res->dist = 0xFFFFFFFFUL;
        res->idx = -1;
        res->match = 0;
        res->valid = 0;
        return;
    }

    calc_norm_rgb(&res->raw, &res->rn, &res->gn, &res->bn);
    res->idx = classify_from_norm(res->rn, res->gn, res->bn, &res->dist);
    res->match = ((res->idx >= 0) && (res->dist <= MATCH_THRESHOLD)) ? 1U : 0U;
    res->valid = 1;
}

static char color_char_idx(int8_t idx, uint8_t valid, uint8_t match)
{
    if (!valid) return 'E';
    if (!match) return 'N';
    switch (idx) {
        case COLOR_RED:   return 'R';
        case COLOR_BLUE:  return 'B';
        case COLOR_GREEN: return 'G';
        default:          return '?';
    }
}

static uint8_t local_is_color(const SensorResult *local, int8_t color_idx)
{
    return (uint8_t)(local->valid && local->match && (local->idx == color_idx));
}

static uint8_t remote_is_color(const RemoteColor *remote, int8_t color_idx)
{
    return (uint8_t)(remote->valid && remote->match && (remote->idx == color_idx));
}

static uint8_t local_is_blue_or_green(const SensorResult *local)
{
    return (uint8_t)(local->valid && local->match &&
           ((local->idx == COLOR_BLUE) || (local->idx == COLOR_GREEN)));
}

static uint8_t remote_is_blue_or_green(const RemoteColor *remote)
{
    return (uint8_t)(remote->valid && remote->match &&
           ((remote->idx == COLOR_BLUE) || (remote->idx == COLOR_GREEN)));
}

static uint8_t detect_cham1_from_red(const SensorResult *local, const RemoteColor *remote)
{
    uint8_t local_red  = local_is_color(local, COLOR_RED);
    uint8_t remote_red = remote_is_color(remote, COLOR_RED);

    if (local_red || remote_red) {
        g_seen_red = 1U;
    }

    if (g_seen_red && (local_is_blue_or_green(local) || remote_is_blue_or_green(remote))) {
        return 1U;
    }

    return 0U;
}

/* CHAM 3: chi kich hoat khi thay MAU XANH DUONG */
static uint8_t detect_cham3_blue_any(const SensorResult *local,
                                     const RemoteColor *remote)
{
    uint8_t local_blue  = local_is_color(local, COLOR_BLUE);
    uint8_t remote_blue = remote_is_color(remote, COLOR_BLUE);

    return (uint8_t)(local_blue || remote_blue);
}

static void update_last_seen_side(uint8_t local_target, uint8_t remote_target)
{
    if (local_target && !remote_target) {
        g_last_seen_side = LAST_SEEN_LEFT;
    } else if (remote_target && !local_target) {
        g_last_seen_side = LAST_SEEN_RIGHT;
    }
}

static MoveState decide_follow_or_recover_target(const SensorResult *local,
                                                 const RemoteColor *remote,
                                                 int8_t target_color,
                                                 MoveState fallback_when_never_seen)
{
    uint8_t local_target;
    uint8_t remote_target;

    local_target  = local_is_color(local, target_color);
    remote_target = remote_is_color(remote, target_color);

    update_last_seen_side(local_target, remote_target);

    if (local_target && remote_target) {
        return MOVE_FORWARD;
    }

    if (local_target) {
        return MOVE_RIGHT;
    }

    if (remote_target) {
        return MOVE_LEFT;
    }

    if (g_last_seen_side == LAST_SEEN_LEFT) {
        return MOVE_RIGHT;
    }

    if (g_last_seen_side == LAST_SEEN_RIGHT) {
        return MOVE_LEFT;
    }

    return fallback_when_never_seen;
}

static void apply_move(MoveState state)
{
    switch (state) {
        case MOVE_FORWARD: Motor_Forward();    break;
        case MOVE_LEFT:    Motor_Bias_Left();  break;
        case MOVE_RIGHT:   Motor_Bias_Right(); break;
        default:           Motor_Stop();       break;
    }
}

static void Sensor_Init_Local(void)
{
    I2C_Peripheral_Init(I2C1);
    delay_ms_tick(10);
    tcs3272_init(I2C1);
    delay_ms_tick(10);
}

static void process_remote_and_local(void)
{
    RemoteColor remote;
    MoveState state;
    uint32_t now;
    int debug_move;
    uint8_t local_green;
    uint8_t remote_green;

    state = MOVE_STOP;
    debug_move = 0;

    now = millis();

    remote_read_color(&remote);
    local_sensor_read_and_classify(&g_local_sensor);

    switch (g_phase) {
        case PHASE_FOLLOW_RED:
            if (detect_cham1_from_red(&g_local_sensor, &remote)) {
                g_phase = PHASE_CHAM1_BRAKE;
                g_phase_start_ms = now;
                Motor_BrakeLock();
                debug_move = -10;
            } else {
                state = decide_follow_or_recover_target(&g_local_sensor, &remote,
                                                        COLOR_RED, MOVE_STOP);
                apply_move(state);
                debug_move = (int)state;
            }
            break;

        case PHASE_CHAM1_BRAKE:
            Motor_BrakeLock();
            debug_move = -10;
            if ((now - g_phase_start_ms) >= CHAM1_BRAKE_MS) {
                g_phase = PHASE_TURN_RIGHT_AFTER_CHAM1;
                g_phase_start_ms = now;
            }
            break;

        case PHASE_TURN_RIGHT_AFTER_CHAM1:
            Motor_SpinRight();
            debug_move = -11;
            if ((now - g_phase_start_ms) >= TURN_RIGHT_AFTER_CHAM1_MS) {
                g_phase = PHASE_TRANS_BLUE_TO_GREEN;
                g_phase_start_ms = now;
                g_last_seen_side = LAST_SEEN_RIGHT;
            }
            break;

        case PHASE_TRANS_BLUE_TO_GREEN:
            local_green  = local_is_color(&g_local_sensor, COLOR_GREEN);
            remote_green = remote_is_color(&remote, COLOR_GREEN);

            if (local_green || remote_green) {
                g_phase = PHASE_FOLLOW_GREEN;
                g_phase_start_ms = now;
                g_green_locked = 1U;
                update_last_seen_side(local_green, remote_green);

                state = decide_follow_or_recover_target(&g_local_sensor, &remote,
                                                        COLOR_GREEN, MOVE_STOP);
                apply_move(state);
                debug_move = (int)state;
            } else {
                state = decide_follow_or_recover_target(&g_local_sensor, &remote,
                                                        COLOR_BLUE, MOVE_RIGHT);
                apply_move(state);
                debug_move = (int)state;
            }
            break;

        case PHASE_FOLLOW_GREEN:
            local_green  = local_is_color(&g_local_sensor, COLOR_GREEN);
            remote_green = remote_is_color(&remote, COLOR_GREEN);

            if (local_green || remote_green) {
                g_green_locked = 1U;
            }

            /* CHAM 3: bo do, chi thay xanh duong moi phanh + quay */
            if (g_green_locked &&
                ((now - g_phase_start_ms) >= CHAM3_ARM_DELAY_MS) &&
                detect_cham3_blue_any(&g_local_sensor, &remote)) {
                g_phase = PHASE_CHAM3_BRAKE;
                g_phase_start_ms = now;
                Motor_BrakeLock();
                debug_move = -13;
            } else {
                state = decide_follow_or_recover_target(&g_local_sensor, &remote,
                                                        COLOR_GREEN, MOVE_STOP);
                apply_move(state);
                debug_move = (int)state;
            }
            break;

        case PHASE_CHAM3_BRAKE:
            Motor_BrakeLock();
            debug_move = -13;
            if ((now - g_phase_start_ms) >= CHAM3_BRAKE_MS) {
                g_phase = PHASE_TURN_RIGHT_AFTER_CHAM3;
                g_phase_start_ms = now;
            }
            break;

        case PHASE_TURN_RIGHT_AFTER_CHAM3:
            Motor_SpinRight();
            debug_move = -17;
            if ((now - g_phase_start_ms) >= TURN_RIGHT_AFTER_CHAM3_MS) {
                g_phase = PHASE_FOLLOW_BLUE_AFTER_CHAM3;
                g_phase_start_ms = now;
                g_last_seen_side = LAST_SEEN_RIGHT;
            }
            break;

        case PHASE_FOLLOW_BLUE_AFTER_CHAM3:
            state = decide_follow_or_recover_target(&g_local_sensor, &remote,
                                                    COLOR_BLUE, MOVE_RIGHT);
            apply_move(state);
            debug_move = (int)state;
            break;

        default:
            g_phase = PHASE_CHAM3_BRAKE;
            Motor_BrakeLock();
            debug_move = -99;
            break;
    }

#if DEBUG_UART
    if ((now - g_last_print_ms) >= DEBUG_PRINT_MS) {
        char lc;
        char rc;

        lc = color_char_idx(g_local_sensor.idx, g_local_sensor.valid, g_local_sensor.match);
        rc = color_char_idx(remote.idx, remote.valid, remote.match);

        g_last_print_ms = now;

        if ((remote.idx != g_last_remote_idx) ||
            (remote.valid != g_last_remote_valid) ||
            (g_local_sensor.idx != g_last_local_idx) ||
            (g_local_sensor.valid != g_last_local_valid) ||
            ((int8_t)g_phase != g_last_phase)) {

            g_last_remote_idx = remote.idx;
            g_last_remote_valid = remote.valid;
            g_last_local_idx = g_local_sensor.idx;
            g_last_local_valid = g_local_sensor.valid;
            g_last_phase = (int8_t)g_phase;

            uart_printf("L:%c R:%c M:%d PH:%d SR:%d GL:%d LS:%d\r\n",
                        lc, rc, debug_move, (int)g_phase,
                        (int)g_seen_red, (int)g_green_locked,
                        (int)g_last_seen_side);
        }
    }
#endif

    delay_ms_tick(SENSOR_LOOP_DELAY_MS);
}

int main(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);

    Usart_Int(115200);
    uart_send_len("RX RUN\r\n", 8);

    GPIO_Init_All();
    PWM_TIM3_Init();
    Motor_Stop();
    Sensor_Init_Local();

    while (1)
    {
        process_remote_and_local();
    }
}