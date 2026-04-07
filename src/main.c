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

typedef enum {
    COLOR_RED    = 0,
    COLOR_BLUE   = 1,
    COLOR_GREEN  = 2,
    COLOR_YELLOW = 3
} ColorIdx;

/* ===== 4 MAU VUA LAY ===== */
static const int16_t allowed_norm[4][3] = {
    {735, 139, 124},  // RED
    {271, 344, 384},  // BLUE
    {287, 447, 263},  // GREEN
    {457, 393, 148}   // YELLOW
};

#define MATCH_THRESHOLD         15000UL
#define SENSOR_LOOP_DELAY_MS    10U
#define DEBUG_PRINT_MS          150U
#define DEBUG_UART              1

/* ===== logic ===== */
#define BRAKE_MS                120U
#define TURN_LEFT_MS            900U
#define TURN_POINT2_LEFT_MS     500U
#define TURN_POINT3_LEFT_MS     400U
#define TURN_POINT4_LEFT_MS     250U

#define POINT1_CONFIRM_COUNT    3U
#define POINT2_CONFIRM_COUNT    3U
#define POINT3_CONFIRM_COUNT    12U   /* Tăng lên 12 để tránh nhiễu điểm 3 */
#define POINT4_CONFIRM_COUNT    3U
#define POINT5_CONFIRM_COUNT    3U

/* Bỏ qua tín hiệu tìm điểm 3 trong 1 giây đầu sau khi xong điểm 2 */
#define POINT3_BLIND_TIME_MS    1000U 

/* =========================================================
   REMOTE COLOR INPUT FROM SENDER BOARD
   sender:   PA0=BIT0, PA1=BIT1, PA2=VALID
   receiver: PA3=BIT0, PA4=BIT1, PA5=VALID

   ma hoa 2 bit:
   00 = RED
   01 = BLUE
   10 = GREEN
   11 = YELLOW
   ========================================================= */
#define REMOTE_PORT             GPIOA
#define REMOTE_PIN_BIT0         GPIO_Pin_3
#define REMOTE_PIN_BIT1         GPIO_Pin_4
#define REMOTE_PIN_VALID        GPIO_Pin_5

/* =========================================================
   MOTOR MAP - L298N
   ENA = PB0 = TIM3_CH3
   ENB = PB1 = TIM3_CH4
   IN1 = PB12
   IN2 = PB13
   IN3 = PB14
   IN4 = PB15
   ========================================================= */
#define PWM_PERIOD              999U
#define SPEED_SCALE_PERCENT     30U
#define MIN_EFFECTIVE_PWM       140U

/* toc do chung */
#define PWM_FORWARD             850U
#define PWM_TURN_OUTER          680U
#define PWM_TURN_INNER          500U
#define PWM_SPIN                720U

/* toc do cham dung cho point4, va gio dung them cho doan point2 -> point3 */
#define PWM_FORWARD_RED_SLOW    580U
#define PWM_TURN_OUTER_RED_SLOW 600U
#define PWM_TURN_INNER_RED_SLOW 180U

static volatile uint32_t g_ms_ticks = 0;
static uint32_t g_last_print_ms = 0;
static SensorResult g_local_sensor;
static int8_t g_last_remote_idx = -2;
static uint8_t g_last_remote_valid = 2;
static int8_t g_last_local_idx = -2;
static uint8_t g_last_local_valid = 2;

/* bo nho huong tim line */
static MoveState g_last_seek_move = MOVE_FORWARD;
static uint8_t g_have_seek_memory = 0;
static MoveState g_last_move_state = MOVE_STOP;

/* logic diem */
static int8_t g_target_color = COLOR_RED;     /* luc dau bam line do */
static uint8_t g_point1_done = 0;
static uint8_t g_point2_done = 0;
static uint8_t g_point3_done = 0;
static uint8_t g_point4_done = 0;
static uint8_t g_point1_counter = 0;
static uint8_t g_point2_counter = 0;
static uint8_t g_point3_counter = 0;
static uint8_t g_point4_counter = 0;
static uint8_t g_point5_counter = 0;
static uint8_t g_stop_forever = 0;
static uint8_t g_after_point4_red_mode = 0;

/* Mốc thời gian để tính khoảng mù */
static uint32_t g_point2_done_ms = 0; 

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

static void Motor_SetForwardDirection(void)
{
    GPIOB->ODR |=  (1U << 12);
    GPIOB->ODR &= ~(1U << 13);
    GPIOB->ODR |=  (1U << 14);
    GPIOB->ODR &= ~(1U << 15);
}

static void Motor_Stop(void)
{
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    GPIOB->ODR &= ~((1U << 12) | (1U << 13) | (1U << 14) | (1U << 15));
}

static void Motor_Forward(void)
{
    Motor_SetForwardDirection();
    Motor_SetPWM(PWM_FORWARD, PWM_FORWARD);
}

static void Motor_Bias_Left(void)
{
    Motor_SetForwardDirection();
    Motor_SetPWM(PWM_TURN_INNER, PWM_TURN_OUTER);
}

static void Motor_Bias_Right(void)
{
    Motor_SetForwardDirection();
    Motor_SetPWM(PWM_TURN_OUTER, PWM_TURN_INNER);
}

/* ===== toc do rieng sau point4 de bam line do ===== */
static void Motor_Forward_RedSlow(void)
{
    Motor_SetForwardDirection();
    Motor_SetPWM(PWM_FORWARD_RED_SLOW, PWM_FORWARD_RED_SLOW);
}

static void Motor_Bias_Left_RedSlow(void)
{
    Motor_SetForwardDirection();
    Motor_SetPWM(PWM_TURN_INNER_RED_SLOW, PWM_TURN_OUTER_RED_SLOW);
}

static void Motor_Bias_Right_RedSlow(void)
{
    Motor_SetForwardDirection();
    Motor_SetPWM(PWM_TURN_OUTER_RED_SLOW, PWM_TURN_INNER_RED_SLOW);
}

/* Khoa banh / phanh cung */
static void Motor_Brake(uint32_t ms)
{
    TIM3->CCR3 = PWM_PERIOD;
    TIM3->CCR4 = PWM_PERIOD;

    /* active brake */
    GPIOB->ODR |= (1U << 12);
    GPIOB->ODR |= (1U << 13);
    GPIOB->ODR |= (1U << 14);
    GPIOB->ODR |= (1U << 15);

    delay_ms_tick(ms);

    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
}

/* Quay trai tai cho */
static void Motor_TurnLeft_InPlace(void)
{
    /* Left motor forward */
    GPIOB->ODR |=  (1U << 12);
    GPIOB->ODR &= ~(1U << 13);

    /* Right motor reverse */
    GPIOB->ODR &= ~(1U << 14);
    GPIOB->ODR |=  (1U << 15);

    Motor_SetPWM(PWM_SPIN, PWM_SPIN);
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

    /* NHAN CA YELLOW = 3 */
    if (code <= 3U) {
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

    for (i = 0; i < 4U; i++) {
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
        case COLOR_RED:    return 'R';
        case COLOR_BLUE:   return 'B';
        case COLOR_GREEN:  return 'G';
        case COLOR_YELLOW: return 'Y';
        default:           return '?';
    }
}

static char target_char(int8_t idx)
{
    switch (idx) {
        case COLOR_RED:    return 'R';
        case COLOR_BLUE:   return 'B';
        case COLOR_GREEN:  return 'G';
        case COLOR_YELLOW: return 'Y';
        default:           return '?';
    }
}

static uint8_t local_has_color(const SensorResult *local, int8_t color)
{
    return (local->valid && local->match && (local->idx == color)) ? 1U : 0U;
}

static uint8_t remote_has_color(const RemoteColor *remote, int8_t color)
{
    return (remote->valid && remote->match && (remote->idx == color)) ? 1U : 0U;
}

/* Diem 1: dang bam do ma thay xanh duong hoac xanh la */
static uint8_t detect_point1(const SensorResult *local, const RemoteColor *remote)
{
    uint8_t local_point  = local_has_color(local, COLOR_BLUE)  || local_has_color(local, COLOR_GREEN);
    uint8_t remote_point = remote_has_color(remote, COLOR_BLUE) || remote_has_color(remote, COLOR_GREEN);
    return (uint8_t)(local_point || remote_point);
}

/* Diem 2: dang bam xanh la, chi can thay XANH_DUONG la kich hoat */
static uint8_t detect_point2(const SensorResult *local, const RemoteColor *remote)
{
    uint8_t local_blue  = local_has_color(local, COLOR_BLUE);
    uint8_t remote_blue = remote_has_color(remote, COLOR_BLUE);

    return (uint8_t)(local_blue || remote_blue);
}

/* Diem 3: dang bam xanh duong, chi can thay XANH_LA la kich hoat */
static uint8_t detect_point3(const SensorResult *local, const RemoteColor *remote)
{
    uint8_t local_green  = local_has_color(local, COLOR_GREEN);
    uint8_t remote_green = remote_has_color(remote, COLOR_GREEN);

    return (uint8_t)(local_green || remote_green);
}

/* Diem 4: dang bam xanh la sau point3, thay DO hoac XANH_DUONG la kich hoat */
static uint8_t detect_point4(const SensorResult *local, const RemoteColor *remote)
{
    uint8_t local_red   = local_has_color(local, COLOR_RED);
    uint8_t remote_red  = remote_has_color(remote, COLOR_RED);
    uint8_t local_blue  = local_has_color(local, COLOR_BLUE);
    uint8_t remote_blue = remote_has_color(remote, COLOR_BLUE);

    return (uint8_t)(local_red || remote_red || local_blue || remote_blue);
}

/* Diem 5: đang bám Đỏ sau point4, gặp VÀNG (YELLOW) là kích hoạt */
static uint8_t detect_point5(const SensorResult *local, const RemoteColor *remote)
{
    uint8_t local_yellow  = local_has_color(local, COLOR_YELLOW);
    uint8_t remote_yellow = remote_has_color(remote, COLOR_YELLOW);

    return (uint8_t)(local_yellow || remote_yellow);
}

/* --- BỎ HÀM DỪNG, CHỈ QUAY TRỰC TIẾP --- */

static void handle_point1(void)
{
#if DEBUG_UART
    uart_printf("POINT1 -> LEFT -> FOLLOW GREEN\r\n");
#endif

    Motor_TurnLeft_InPlace();
    delay_ms_tick(TURN_LEFT_MS);

    g_target_color = COLOR_GREEN;
    g_point1_done = 1;
    g_after_point4_red_mode = 0;

    g_last_seek_move = MOVE_LEFT;
    g_have_seek_memory = 1;
}

static void handle_point2(void)
{
#if DEBUG_UART
    uart_printf("POINT2 -> LEFT 500 -> FOLLOW BLUE_SLOW\r\n");
#endif

    Motor_TurnLeft_InPlace();
    delay_ms_tick(TURN_POINT2_LEFT_MS);

    g_target_color = COLOR_BLUE;
    g_point2_done = 1;
    g_point2_done_ms = millis(); // LƯU MỐC THỜI GIAN ĐỂ CHỐNG NHIỄU Ở ĐIỂM 3
    g_after_point4_red_mode = 0;

    g_last_seek_move = MOVE_LEFT;
    g_have_seek_memory = 1;
}

static void handle_point3(void)
{
#if DEBUG_UART
    uart_printf("POINT3 -> LEFT 400 -> FOLLOW GREEN\r\n");
#endif

    Motor_TurnLeft_InPlace();
    delay_ms_tick(TURN_POINT3_LEFT_MS);

    g_target_color = COLOR_GREEN;
    g_point3_done = 1;
    g_after_point4_red_mode = 0;

    g_last_seek_move = MOVE_LEFT;
    g_have_seek_memory = 1;
    g_stop_forever = 0;
}

static void handle_point4(void)
{
#if DEBUG_UART
    uart_printf("POINT4 -> LEFT 400 -> FOLLOW RED_SLOW\r\n");
#endif

    Motor_TurnLeft_InPlace();
    delay_ms_tick(TURN_POINT4_LEFT_MS);

    g_target_color = COLOR_RED;
    g_point4_done = 1;
    g_after_point4_red_mode = 1;

    g_last_seek_move = MOVE_LEFT;
    g_have_seek_memory = 1;
    g_stop_forever = 0;
}

static void handle_point5(void)
{
#if DEBUG_UART
    uart_printf("POINT5 (YELLOW) -> BRAKE -> STOP FOREVER\r\n");
#endif

    /* Điểm 5 là điểm cuối nên giữ phanh cứng để dừng hẳn */
    Motor_Brake(BRAKE_MS);
    Motor_Stop();
    
    g_stop_forever = 1;
}

static MoveState decide_move(const SensorResult *local, const RemoteColor *remote)
{
    uint8_t local_target  = local_has_color(local, g_target_color);
    uint8_t remote_target = remote_has_color(remote, g_target_color);

    if (local_target && remote_target) {
        return MOVE_FORWARD;
    }

    if (local_target) {
        g_last_seek_move = MOVE_RIGHT;
        g_have_seek_memory = 1;
        return MOVE_RIGHT;
    }

    if (remote_target) {
        g_last_seek_move = MOVE_LEFT;
        g_have_seek_memory = 1;
        return MOVE_LEFT;
    }

    if (g_have_seek_memory) {
        return g_last_seek_move;
    }

    return MOVE_FORWARD;
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

static void apply_move_red_slow(MoveState state)
{
    switch (state) {
        case MOVE_FORWARD: Motor_Forward_RedSlow();    break;
        case MOVE_LEFT:    Motor_Bias_Left_RedSlow();  break;
        case MOVE_RIGHT:   Motor_Bias_Right_RedSlow(); break;
        default:           Motor_Stop();               break;
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
    uint32_t now = millis();

    if (g_stop_forever) {
        Motor_Stop();
        delay_ms_tick(SENSOR_LOOP_DELAY_MS);
        return;
    }

    remote_read_color(&remote);
    local_sensor_read_and_classify(&g_local_sensor);

    /* ===== pha 1: bam do, gap xanh duong/xanh la = diem 1 ===== */
    if ((!g_point1_done) && (g_target_color == COLOR_RED) && detect_point1(&g_local_sensor, &remote)) {
        if (g_point1_counter < 255U) g_point1_counter++;
    } else {
        g_point1_counter = 0;
    }

    if ((!g_point1_done) && (g_point1_counter >= POINT1_CONFIRM_COUNT)) {
        g_point1_counter = 0;
        handle_point1();
        g_last_move_state = MOVE_STOP;
        delay_ms_tick(SENSOR_LOOP_DELAY_MS);
        return;
    }

    /* ===== pha 2: dang bam xanh la, chi can gap xanh duong = diem 2 ===== */
    if (g_point1_done && (!g_point2_done) && (g_target_color == COLOR_GREEN) &&
        detect_point2(&g_local_sensor, &remote)) {
        if (g_point2_counter < 255U) g_point2_counter++;
    } else {
        g_point2_counter = 0;
    }

    if (g_point1_done && (!g_point2_done) && (g_point2_counter >= POINT2_CONFIRM_COUNT)) {
        g_point2_counter = 0;
        handle_point2();
        g_last_move_state = MOVE_STOP;
        delay_ms_tick(SENSOR_LOOP_DELAY_MS);
        return;
    }

    /* ===== pha 3: dang bam xanh duong, gap xanh la = diem 3 ===== */
    if (g_point2_done && (!g_point3_done) && (g_target_color == COLOR_BLUE)) {
        /* Chỉ dò tìm điểm 3 nếu xe đã rời khỏi điểm 2 được một thời gian (vượt khoảng mù) */
        if ((now - g_point2_done_ms) > POINT3_BLIND_TIME_MS) {
            if (detect_point3(&g_local_sensor, &remote)) {
                if (g_point3_counter < 255U) g_point3_counter++;
            } else {
                g_point3_counter = 0;
            }
        } else {
            g_point3_counter = 0; /* Đang trong thời gian mù, không cộng dồn */
        }
    }

    if (g_point2_done && (!g_point3_done) && (g_point3_counter >= POINT3_CONFIRM_COUNT)) {
        g_point3_counter = 0;
        handle_point3();
        g_last_move_state = MOVE_STOP;
        delay_ms_tick(SENSOR_LOOP_DELAY_MS);
        return;
    }

    /* ===== pha 4: dang bam xanh la sau point3, gap do hoac xanh duong = diem 4 ===== */
    if (g_point3_done && (!g_point4_done) && (g_target_color == COLOR_GREEN) &&
        detect_point4(&g_local_sensor, &remote)) {
        if (g_point4_counter < 255U) g_point4_counter++;
    } else {
        g_point4_counter = 0;
    }

    if (g_point3_done && (!g_point4_done) && (g_point4_counter >= POINT4_CONFIRM_COUNT)) {
        g_point4_counter = 0;
        handle_point4();
        g_last_move_state = MOVE_STOP;
        delay_ms_tick(SENSOR_LOOP_DELAY_MS);
        return;
    }

    /* ===== pha 5: sau diem 4 (dang bam DO), gap VANG = diem 5 DUNG HẲN ===== */
    if (g_point4_done && (!g_stop_forever) && (g_target_color == COLOR_RED) &&
        detect_point5(&g_local_sensor, &remote)) {
        if (g_point5_counter < 255U) g_point5_counter++;
    } else {
        g_point5_counter = 0;
    }

    if (g_point4_done && (!g_stop_forever) && (g_point5_counter >= POINT5_CONFIRM_COUNT)) {
        g_point5_counter = 0;
        handle_point5();
        g_last_move_state = MOVE_STOP;
        delay_ms_tick(SENSOR_LOOP_DELAY_MS);
        return;
    }

    state = decide_move(&g_local_sensor, &remote);

    /* TU POINT2 DEN POINT3 hoac SAU POINT4 dung toc do cham */
    if ((g_point2_done && !g_point3_done && (g_target_color == COLOR_BLUE)) ||
        (g_after_point4_red_mode && (g_target_color == COLOR_RED))) {
        apply_move_red_slow(state);
    } else {
        apply_move(state);
    }

#if DEBUG_UART
    if ((now - g_last_print_ms) >= DEBUG_PRINT_MS) {
        char lc = color_char_idx(g_local_sensor.idx, g_local_sensor.valid, g_local_sensor.match);
        char rc = color_char_idx(remote.idx, remote.valid, remote.match);

        g_last_print_ms = now;

        if ((remote.idx != g_last_remote_idx) || (remote.valid != g_last_remote_valid) ||
            (g_local_sensor.idx != g_last_local_idx) || (g_local_sensor.valid != g_last_local_valid) ||
            (state != g_last_move_state)) {

            g_last_remote_idx = remote.idx;
            g_last_remote_valid = remote.valid;
            g_last_local_idx = g_local_sensor.idx;
            g_last_local_valid = g_local_sensor.valid;
            g_last_move_state = state;

            uart_printf("T:%c L:%c R:%c M:%d MEM:%d P1:%d P2:%d P3:%d P4:%d RS:%d STOP:%d\r\n",
                        target_char(g_target_color),
                        lc, rc,
                        (int)state,
                        (int)g_last_seek_move,
                        (int)g_point1_done,
                        (int)g_point2_done,
                        (int)g_point3_done,
                        (int)g_point4_done,
                        (int)g_after_point4_red_mode,
                        (int)g_stop_forever);
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