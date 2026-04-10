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

#define DEBUG_UART              1

#define PWM_PERIOD              999U
#define PWM_FORWARD             200U
#define PWM_BRAKE       999U
#define BRAKE_PULSE_MS  60U

#define MATCH_THRESHOLD         18000UL

#define SENSOR_LOOP_DELAY_MS    8U
#define PRINT_INTERVAL_MS       250U

/* =========================================================
   CÁC THÔNG SỐ ĐIỀU CHỈNH THỜI GIAN CUA (RẼ)
   ========================================================= */
#define TURN_PULSE_MS           7U    /* Thời gian nhịp rẽ bám line bình thường (ms) */

/* <=== ĐIỂM 1: CHỈNH THỜI GIAN CUA KHI CHUYỂN TỪ ĐỎ SANG XANH DƯƠNG Ở ĐÂY ===> */
#define TRANSITION_TURN_MS      600U  /* (ms) - Tăng/giảm số này để xe cua vừa đủ vào line Xanh */


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

/* Thêm hàm: Bánh phải tiến, bánh trái đứng yên */
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

/* Hàm kiểm tra màu có khớp với mục tiêu đang theo dõi hay không */
static uint8_t is_target_color(ColorIdx idx, ColorIdx target)
{
    return (idx == target) ? 1U : 0U;
}

int main(void)
{
    ColorData right_raw, left_raw;
    SensorResult right_res, left_res;
    uint32_t last_print = 0;
    uint8_t was_moving = 0;

    /* Khởi tạo biến lưu trạng thái màu mục tiêu: Bắt đầu bám màu ĐỎ */
    ColorIdx current_target = COLOR_RED; 

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);

    Usart_Int(115200);
    uart_printf("\r\nSTART COLOR -> MOTOR TEST (FAST)\r\n");

    Sensor_Init_All();
    motor_pwm_init();
    motor_stop();

    uart_printf("Logic Line Following:\r\n");
    uart_printf("- Muc tieu: DO -> XANH DUONG\r\n");
    uart_printf("- 2 BEN NHAN MAU  => FORWARD\r\n");
    uart_printf("- TRAI KHONG, PHAI CO => RE PHAI (Banh trai tien, banh phai lui)\r\n");
    uart_printf("- TRAI CO, PHAI KHONG => RE TRAI (Banh trai lui, banh phai tien)\r\n");
    uart_printf("- 2 BEN KHONG NHAN MAU => STOP\r\n\r\n");

    while (1) {
        read_sensor_raw(RIGHT_SENSOR_BUS, &right_raw);
        read_sensor_raw(LEFT_SENSOR_BUS,  &left_raw);

        classify_color(&right_raw, allowed_norm_right, &right_res);
        classify_color(&left_raw,  allowed_norm_left,  &left_res);

        /* LOGIC CHUYỂN LINE: Nếu đang bám màu Đỏ mà 1 trong 2 cảm biến thấy màu Xanh Dương -> Chuyển sang bám Xanh Dương */
        if (current_target == COLOR_RED) {
            if (left_res.idx == COLOR_BLUE || right_res.idx == COLOR_BLUE) {
                current_target = COLOR_BLUE;
                uart_printf("\r\n=== DA CHUYEN MUC TIEU SANG LINE XANH DUONG ===\r\n");
                uart_printf(">>> Thuc hien re: Banh phai tien, Banh trai dung...\r\n");

                /* Thực thi yêu cầu: Bánh PHẢI quay, bánh TRÁI đứng yên trong khoảng thời gian TRANSITION_TURN_MS */
                motor_right_forward_left_stop(PWM_FORWARD);
                delay_ms_tick(TRANSITION_TURN_MS);
                
                motor_stop(); /* Tạm ngắt để ổn định trước khi bám line tiếp */
                was_moving = 0U;

                /* Bỏ qua vòng lặp bám line bên dưới, quay lên trên để cập nhật dữ liệu từ góc độ mới */
                continue;
            }
        }

        /* So sánh màu đọc được với mục tiêu hiện tại thay vì so sánh với tất cả các màu */
        uint8_t left_go = is_target_color(left_res.idx, current_target);
        uint8_t right_go = is_target_color(right_res.idx, current_target);
        const char *motor_state_str = "STOP";

        /* Logic bám line */
        if (left_go && right_go) {
            // Cả hai cảm biến đều trong line -> Đi thẳng
            motor_forward(PWM_FORWARD);
            was_moving = 1U;
            motor_state_str = "FORWARD";

        } else if (!left_go && right_go) {
            // Cảm biến phải nhận, trái không nhận -> Rẽ Phải nhịp TURN_PULSE_MS
            motor_turn_right(PWM_FORWARD);
            delay_ms_tick(TURN_PULSE_MS);
            motor_stop(); // Dừng lại sau nhịp rẽ để chống trượt
            was_moving = 1U;
            motor_state_str = "TURN_RIGHT_PULSE";

        } else if (left_go && !right_go) {
            // Cảm biến trái nhận, phải không nhận -> Rẽ Trái nhịp TURN_PULSE_MS
            motor_turn_left(PWM_FORWARD);
            delay_ms_tick(TURN_PULSE_MS);
            motor_stop(); // Dừng lại sau nhịp rẽ để chống trượt
            was_moving = 1U;
            motor_state_str = "TURN_LEFT_PULSE";

        } else {
            // Cả 2 đều trượt ra ngoài -> Phanh và Dừng
            if (was_moving) {
                motor_brake_pulse_then_stop(PWM_BRAKE, BRAKE_PULSE_MS);
                was_moving = 0U;
            } else {
                motor_stop();
            }
            motor_state_str = "STOP";
        }

        if ((millis() - last_print) >= PRINT_INTERVAL_MS) {
            last_print = millis();

            uart_printf("TARGET:%s | ", color_to_str(current_target));

            uart_printf("R:%s d=%lu C=%u | ",
                        color_to_str(right_res.idx),
                        (unsigned long)right_res.dist,
                        right_res.raw.c);

            uart_printf("L:%s d=%lu C=%u | ",
                        color_to_str(left_res.idx),
                        (unsigned long)left_res.dist,
                        left_res.raw.c);

            uart_printf("MOTOR=%s\r\n", motor_state_str);
        }

        delay_ms_tick(SENSOR_LOOP_DELAY_MS);
    }
}