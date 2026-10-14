# Cấu hình STM32CubeMX cho project MPU6050

Ghi lại các thiết lập cần có trong `mpu6050.ioc` để code trong repo chạy đúng.
File `.ioc` đã chứa đủ các thiết lập này; tài liệu chỉ để tra cứu nhanh và để tự cấu hình lại nếu tạo project mới.

## 1. Thông tin chung

| Mục | Giá trị |
|---|---|
| STM32CubeMX | 6.18.1 |
| Firmware package | STM32Cube FW_F4 **V1.28.3** |
| MCU | **STM32F401CCU6** (UFQFPN48), board `custom` |
| Toolchain / IDE | **CMake**, compiler GCC (`arm-none-eabi-gcc`) |
| Tần số HSE | **25 MHz** (thạch anh ngoài) |

## 2. Pinout

| Chân | Chức năng | Mode trong CubeMX | Ghi chú |
|---|---|---|---|
| PH0 / PH1 | RCC_OSC_IN / OSC_OUT | HSE: Crystal/Ceramic Resonator | Thạch anh 25 MHz |
| PB6 | I2C1_SCL | I2C | Nối SCL của MPU6050 |
| PB7 | I2C1_SDA | I2C | Nối SDA của MPU6050 |
| PA2 | USART2_TX | Asynchronous | Hiện chưa dùng (printf đi qua USB CDC) |
| PA3 | USART2_RX | Asynchronous | Hiện chưa dùng |
| PA11 | USB_OTG_FS_DM | Device_Only | D- |
| PA12 | USB_OTG_FS_DP | Device_Only | D+ |
| PA13 / PA14 | SYS_JTMS-SWDIO / SYS_JTCK-SWCLK | Serial Wire | Nạp và debug SWD |

Đấu nối MPU6050: `VCC` → 3.3 V, `GND` → GND, `SCL` → PB6, `SDA` → PB7, `AD0` → GND (địa chỉ I2C `0x68`).
Chân I2C cần điện trở kéo lên (thường module GY-521 đã có sẵn; CubeMX để `No pull-up` vì giả định có pull-up ngoài).

## 3. Clock Configuration

Tab **Clock Configuration**, chọn `HSE = 25 MHz` → PLL Source Mux = HSE:

| Thông số | Giá trị | Kết quả |
|---|---|---|
| PLLM | `/25` | VCO input 1 MHz |
| PLLN | `x336` | VCO output 336 MHz |
| PLLP | `/4` | **SYSCLK = HCLK = 84 MHz** |
| PLLQ | `/7` | **USB 48 MHz** (bắt buộc đúng 48 MHz) |
| AHB prescaler | `/1` | HCLK 84 MHz |
| APB1 prescaler | `/2` | PCLK1 42 MHz (timer 84 MHz) |
| APB2 prescaler | `/1` | PCLK2 84 MHz |

CubeMX tự sinh Flash latency = 2 WS và voltage scale 2 tương ứng với 84 MHz.
Nếu đổi thạch anh thì phải chọn lại PLLM/PLLN sao cho nhánh PLLQ vẫn ra đúng 48 MHz, nếu không USB sẽ không enumerate.

## 4. Peripheral

### Connectivity → I2C1
- Mode: **I2C**
- I2C Speed Mode: `Standard Mode`, Clock Speed `100000` Hz
- Duty cycle: `Duty cycle Tlow/Thigh = 2`
- Addressing: 7-bit, không bật dual address, không bật general call, clock stretching bật (`No Stretch Mode = Disable`)
- GPIO Settings (mặc định): `Alternate Function Open Drain`, `No pull-up and no pull-down`, speed `Very High`, AF4

### Connectivity → USART2 (tùy chọn, chưa dùng)
- Mode: **Asynchronous**, `115200` baud, 8 bit, không parity, 1 stop bit, TX/RX, oversampling 16

### Connectivity → USB_OTG_FS
- Mode: **Device_Only** (không bật VBUS sensing, không bật SOF)

### Middleware → USB_DEVICE
- Class For FS IP: **Communication Device Class (Virtual Port Com)**
- Giữ mặc định: `USBD_MAX_NUM_INTERFACES = 1`, `APP_RX_DATA_SIZE = APP_TX_DATA_SIZE = 1024`, `Self powered = 1`
- Descriptor mặc định: VID `0x0483`, PID `0x5740`, product string `STM32 Virtual ComPort`

### System Core
- **SYS**: Debug = `Serial Wire`, Timebase Source = `SysTick`
- **RCC**: High Speed Clock = `Crystal/Ceramic Resonator`
- **NVIC**: Priority Group = `4 bits for pre-emption priority`; bật ngắt `USB On The Go FS global interrupt` (tự bật khi chọn USB_DEVICE); SysTick priority 15
- **GPIO**: không cấu hình chân nào (chỉ bật clock GPIOA/B/H)

## 5. Project Manager

| Mục | Giá trị |
|---|---|
| Project Name / Location | `mpu6050`, main nằm ở `Core/Src` |
| Toolchain / IDE | `CMake` |
| Stack size / Heap size | `0x400` / `0x200` |
| Code Generator → Copy library | `Copy all used libraries into the project folder` |
| Code Generator → Generate peripheral initialization as a pair of '.c/.h' files per peripheral | **Tắt** (mọi `MX_xxx_Init` nằm trong `main.c`) |
| Code Generator → Keep User Code when re-generating | **Bật** |
| Code Generator → Delete previously generated files when not re-generated | Bật |
| Enable Full Assert | Tắt |

## 6. Phần code tự viết, không do CubeMX quản lý

- `Drivers/MPU6050/` — driver MPU6050 (`mpu6050.c`, `mpu6050_test.c`, `mpu6050.h`, `mpu6050_regs.h`).
- `App/` — đóng gói frame nhị phân gửi qua USB (`frame.c`, `frame.h`).
- `tools/` — công cụ PC bằng Python (viewer, ghi dữ liệu, test).
- Code của `Core/Src/main.c` nằm trong các khối `USER CODE BEGIN/END`.
- `CMakeLists.txt` ở thư mục gốc **chỉ được CubeMX sinh một lần**, không bị ghi đè khi generate lại. Các dòng tự thêm:
  - `Drivers/MPU6050/Src/mpu6050.c` và `App/Src/frame.c` trong `target_sources`
  - `Drivers/MPU6050/Inc` và `App/Inc` trong `target_include_directories`
  - `Drivers/MPU6050/Src/mpu6050_test.c` trong `add_executable`
- `printf` được chuyển sang USB CDC bằng cách định nghĩa lại `_write()` trong `main.c`.

## 7. Khôi phục thư viện ST sau khi clone

Repo **không** chứa các thư mục do CubeMX copy vào (đã nằm trong `.gitignore`):

- `Drivers/CMSIS/`
- `Drivers/STM32F4xx_HAL_Driver/`
- `Middlewares/ST/STM32_USB_Device_Library/`

Để lấy lại:

1. Cài STM32CubeMX và firmware package **STM32Cube FW_F4 V1.28.3** (CubeMX → Help → Manage embedded software packages).
2. Mở `mpu6050.ioc`, giữ nguyên cấu hình, bấm **Generate Code**.
3. Chạy `git status` để kiểm tra: ba thư mục trên được tạo lại, còn code tự viết không bị thay đổi.
4. Build:
   ```bash
   cmake --preset Debug
   cmake --build --preset Debug
   ```
   (tên preset xem trong `CMakePresets.json`).

> Lưu ý: mỗi lần chỉnh `.ioc` rồi Generate Code, hãy `git status` / `git diff` để chắc là chỉ code do CubeMX sinh bị thay đổi.
