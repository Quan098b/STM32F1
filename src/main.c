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
    PHASE_TURN_RIGHT_500MS,
    PHASE_FOLLOW_BLUE,
    PHASE_CHAM2_LOCK
} RunPhase;

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

#define MATCH_THRESHOLD         15000UL
#define SENSOR_LOOP_DELAY_MS    10U
#define DEBUG_PRINT_MS          150U
#define DEBUG_UART              1

#define COLOR_RED               0
#define COLOR_BLUE              1
#define COLOR_GREEN             2

/* =========================================================
   REMOTE COLOR INPUT FROM SENDER BOARD
   sender:   PA0=BIT0, PA1=BIT1, PA2=VALID
   receiver: PA3=BIT0, PA4=BIT1, PA5=VALID
   ========================================================= */
#define REMOTE_PORT             GPIOA
#define REMOTE_PIN_BIT0         GPIO_Pin_3
#define REMOTE_PIN_BIT1         GPIO_Pin_4
#define REMOTE_PIN_VALID        GPIO_Pin_5

/* =========================================================
   MOTOR MAP - L298N
   ENA = PB0 = TIM3_CH3  -> LEFT WHEEL PWM
   ENB = PB1 = TIM3_CH4  -> RIGHT WHEEL PWM
   IN1 = PB12
   IN2 = PB13
   IN3 = PB14
   IN4 = PB15
   ========================================================= */
#define PWM_PERIOD              999U
#define SPEED_SCALE_PERCENT     30U
#define MIN_EFFECTIVE_PWM       140U
#define PWM_FORWARD             850U
#define PWM_TURN_OUTER          680U
#define PWM_TURN_INNER          500U
#define BRAKE_PWM               PWM_PERIOD

/* ====== TRINH TU CHAM 1 / CHAM 2 ====== */
#define CHAM1_BRAKE_MS          150U
#define TURN_RIGHT_MS           730U
#define PWM_SPIN_RIGHT          850U

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
    GPIOB->ODR |=  (1U << 12);   // IN1 = 1
    GPIOB->ODR &= ~(1U << 13);   // IN2 = 0
    GPIOB->ODR |=  (1U << 14);   // IN3 = 1
    GPIOB->ODR &= ~(1U << 15);   // IN4 = 0
}

static void Motor_SetSpinRightDirection(void)
{
    /* LEFT wheel backward, RIGHT wheel forward */
    GPIOB->ODR &= ~(1U << 12);   // LEFT IN1 = 0
    GPIOB->ODR |=  (1U << 13);   // LEFT IN2 = 1

    GPIOB->ODR |=  (1U << 14);   // RIGHT IN3 = 1
    GPIOB->ODR &= ~(1U << 15);   // RIGHT IN4 = 0
}

static void Motor_Stop(void)
{
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    GPIOB->ODR &= ~((1U << 12) | (1U << 13) | (1U << 14) | (1U << 15));
}

static void Motor_BrakeLock(void)
{
    /* Khóa / hãm chủ động */
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
    uint8_t valid = GPIO_ReadInputDataBit(REMOTE_PORT, REMOTE_PIN_VALID) ? 1U : 0U;
    uint8_t bit0  = GPIO_ReadInputDataBit(REMOTE_PORT, REMOTE_PIN_BIT0) ? 1U : 0U;
    uint8_t bit1  = GPIO_ReadInputDataBit(REMOTE_PORT, REMOTE_PIN_BIT1) ? 1U : 0U;
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

/* Chạm 1:
   Đang bám đỏ mà bắt đầu gặp xanh dương hoặc xanh lá */
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

/* Bám đúng 1 màu mục tiêu */
static MoveState decide_follow_target_color(const SensorResult *local,
                                            const RemoteColor *remote,
                                            int8_t target_color)
{
    uint8_t local_target  = local_is_color(local, target_color);
    uint8_t remote_target = remote_is_color(remote, target_color);

    if ((!local->valid) && (!remote->valid)) return MOVE_STOP;
    if (local_target && remote_target) return MOVE_FORWARD;

    /* Dao huong vi thuc te xe dang cua nguoc */
    if (local_target)  return MOVE_RIGHT;
    if (remote_target) return MOVE_LEFT;

    return MOVE_STOP;
}

/* Chạm 2:
   Hai cảm biến cùng nhận trong tập {RED, GREEN} và khác màu nhau.
   Tức là một bên đỏ, một bên xanh lá. */
static uint8_t detect_cham2_red_green_split(const SensorResult *local,
                                            const RemoteColor *remote)
{
    uint8_t local_rg  = (uint8_t)(local->valid && local->match &&
                        ((local->idx == COLOR_RED) || (local->idx == COLOR_GREEN)));
    uint8_t remote_rg = (uint8_t)(remote->valid && remote->match &&
                        ((remote->idx == COLOR_RED) || (remote->idx == COLOR_GREEN)));

    if (local_rg && remote_rg && (local->idx != remote->idx)) {
        return 1U;
    }

    return 0U;
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
    MoveState state = MOVE_STOP;
    uint32_t now = millis();
    int debug_move = 0;

    remote_read_color(&remote);
    local_sensor_read_and_classify(&g_local_sensor);

    switch (g_phase) {
        case PHASE_FOLLOW_RED:
            if (detect_cham1_from_red(&g_local_sensor, &remote)) {
                g_phase = PHASE_CHAM1_BRAKE;
                g_phase_start_ms = now;
                Motor_BrakeLock();
                debug_move = -10; /* CHAM1_BRAKE */
            } else {
                state = decide_follow_target_color(&g_local_sensor, &remote, COLOR_RED);
                apply_move(state);
                debug_move = (int)state;
            }
            break;

        case PHASE_CHAM1_BRAKE:
            Motor_BrakeLock();
            debug_move = -10; /* CHAM1_BRAKE */
            if ((now - g_phase_start_ms) >= CHAM1_BRAKE_MS) {
                g_phase = PHASE_TURN_RIGHT_500MS;
                g_phase_start_ms = now;
            }
            break;

        case PHASE_TURN_RIGHT_500MS:
            Motor_SpinRight();
            debug_move = -11; /* TURN_RIGHT */
            if ((now - g_phase_start_ms) >= TURN_RIGHT_MS) {
                g_phase = PHASE_FOLLOW_BLUE;
                g_phase_start_ms = now;
            }
            break;

        case PHASE_FOLLOW_BLUE:
            if (detect_cham2_red_green_split(&g_local_sensor, &remote)) {
                g_phase = PHASE_CHAM2_LOCK;
                g_phase_start_ms = now;
                Motor_BrakeLock();
                debug_move = -12; /* CHAM2_LOCK */
            } else {
                state = decide_follow_target_color(&g_local_sensor, &remote, COLOR_BLUE);
                apply_move(state);
                debug_move = (int)state;
            }
            break;

        case PHASE_CHAM2_LOCK:
            Motor_BrakeLock();
            debug_move = -12; /* CHAM2_LOCK */
            break;

        default:
            g_phase = PHASE_CHAM2_LOCK;
            Motor_BrakeLock();
            debug_move = -99;
            break;
    }

#if DEBUG_UART
    if ((now - g_last_print_ms) >= DEBUG_PRINT_MS) {
        char lc = color_char_idx(g_local_sensor.idx, g_local_sensor.valid, g_local_sensor.match);
        char rc = color_char_idx(remote.idx, remote.valid, remote.match);

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

            uart_printf("L:%c R:%c M:%d PH:%d SR:%d\r\n",
                        lc, rc, debug_move, (int)g_phase, (int)g_seen_red);
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