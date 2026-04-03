# STM32F103 - Board nhận tín hiệu màu và điều khiển động cơ

## Mô tả
Project dùng STM32F103C8 với framework CMSIS/StdPeriph để:
- Nhận mã màu từ board gửi qua 3 chân GPIO
- Gửi log debug qua UART1
- Điều khiển động cơ theo màu nhận được

## Vai trò truyền thông màu
- Project này là **board nhận**
- Nhận tín hiệu từ board gửi bằng 3 chân:
  - `PA3` = BIT0
  - `PA4` = BIT1
  - `PA5` = VALID

## Mã màu nhận
- `00` = ĐỎ
- `01` = XANH DƯƠNG
- `10` = XANH LÁ
- Trường hợp `VALID = 0` thì coi như dữ liệu không hợp lệ

## Điều khiển động cơ theo màu
- `ĐỎ` -> đi thẳng
- `XANH DƯƠNG` -> lệch trái
- `XANH LÁ` -> lệch phải
- Không hợp lệ -> dừng

## Cấu hình chính
- Board: `genericSTM32F103C8`
- Framework: `cmsis`
- Upload: `stlink`
- UART debug: `USART1`
  - TX = `PA9`
  - RX = `PA10`
  - Baudrate = `115200`

## Sơ đồ kết nối UART debug
- CH340 TX -> STM32 PA10
- CH340 RX -> STM32 PA9
- GND CH340 -> GND STM32

## Chức năng mã nguồn
- `src/main.c`: nhận mã màu từ GPIO, giải mã màu, điều khiển xe
- `src/usart.c`: cấu hình UART1 và hàm gửi dữ liệu serial
- `src/motor_control.c`: điều khiển hướng chạy và xung tốc độ động cơ (nếu dùng)

## Ghi chú đấu nối với board gửi
- Board gửi xuất:
  - `PA0` = BIT0
  - `PA1` = BIT1
  - `PA2` = VALID
- Nối sang board nhận tương ứng:
  - `PA0 (gửi)` -> `PA3 (nhận)`
  - `PA1 (gửi)` -> `PA4 (nhận)`
  - `PA2 (gửi)` -> `PA5 (nhận)`
- Nhớ nối chung GND giữa hai board
