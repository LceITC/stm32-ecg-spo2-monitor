# STM32 ECG 与血氧监测项目

这是一个基于 STM32F103C8T6 的心电（ECG）与血氧（SpO2）监测项目。项目使用 STM32 标准外设库，完成 ECG 采集、滤波、心率检测、MAX30102 红光/红外光采集、手指检测、血氧计算、OLED 显示和串口数据上传。

## 功能概览

- 使用 AD8232 采集 ECG，并支持切换到 PA2 上的直通信号源进行算法测试。
- TIM2 以 100 Hz 触发 ADC1，DMA1 Channel1 将 ADC 结果搬运到软件采样队列。
- ECG 数据经过滤波后，通过自适应阈值和 R 峰检测算法计算心率。
- 使用 MAX30102 采集 RED/IR 数据，进行手指状态判断和 SpO2 计算。
- 使用 OLED 显示 ECG 数值、心率、SpO2、手指状态、传感器初始化状态和 PPG 波形页面。
- USART1 以 115200 8N1 上传 CSV 数据，并接收 ECG 采集源切换命令。
- 提供 ECG 心率算法和 MAX30102 SpO2 算法的主机端 CMake 测试。

## 硬件平台

- MCU：STM32F103C8T6，系统时钟 72 MHz
- ECG 模块：AD8232，输出连接 PA0
- 光学传感器：MAX30102
- 显示屏：SSD1306 OLED，软件 I2C
- 开发板：STM32F103C8T6 Blue Pill

## 主要引脚

| 引脚 | 功能 |
| --- | --- |
| PA0 | ADC1_IN0，AD8232 ECG 输入 |
| PA2 | ADC1_IN2，直通信号源输入 |
| PA6 | 按键输入，切换 OLED 页面 |
| PA9 | USART1_TX |
| PA10 | USART1_RX |
| PB8 | OLED 软件 I2C SCL |
| PB9 | OLED 软件 I2C SDA，OLED 地址为 0x3C |
| PB10 | MAX30102 软件 I2C SCL |
| PB11 | MAX30102 软件 I2C SDA |
| PB12 | MAX30102 INT，低电平表示数据就绪 |
| PA13 / PA14 | SWD 调试接口 |

OLED 和 MAX30102 使用两条独立的软件 I2C 总线，STM32 硬件 I2C 外设未使用。TIM2 的通道 2 只产生内部比较事件，不占用外部引脚。

## 数据流程

### ECG 与心率

    AD8232 / 直通信号源
            ↓
    ADC1（TIM2 CC2，100 Hz）
            ↓
    DMA1 Channel1
            ↓
    ECG 软件采样队列
            ↓
    ECG 滤波
            ↓
    自适应阈值与 R 峰检测
            ↓
    心率 BPM

心率算法以 100 Hz 为基础，使用 2 秒窗口估计信号幅度和阈值，并通过 RR 间隔有效性检查抑制噪声和误检。

### MAX30102 与 SpO2

    MAX30102 RED / IR FIFO
            ↓
    手指检测与有效样本筛选
            ↓
    500 点 SpO2 计算窗口
            ↓
    SpO2 百分比

MAX30102 配置为 100 Hz、18 位 SpO2 模式。手指检测、SpO2 计算和 ECG 心率检测相互独立。

## 串口协议

USART1 参数：115200 baud，8 数据位，1 停止位，无校验。

每个 ECG 样本发送一行 CSV：

    ecg_value,spo2_value,heart_rate,ir_value\r\n

示例：

    1840,0,72,714

其中：

- ecg_value：滤波后的 ECG ADC 值
- spo2_value：当前 SpO2，0 表示暂无有效结果
- heart_rate：当前心率 BPM，0 表示暂无有效结果
- ir_value：MAX30102 的 IR 原始值

可以向 USART1 发送以下命令切换 ECG 数据源：

    ECG_SRC,AD8232\n
    ECG_SRC,DIRECT\n

## 目录结构

    Drivers/                         STM32F10x 标准外设库
    Hardware/Inc、Hardware/Src       ADC、DMA、TIM2、ECG、MAX30102 和监测模块
    my_lib/Inc、my_lib/Src           OLED、延时和串口底层驱动
    start/                           启动文件和 CMSIS 核心文件
    user/                            main 和中断文件
    tests/                           主机端算法测试
    tmp/pdfs/                        项目参考文档及图片
    CMakeLists.txt                   固件工程 CMake 配置
    5_1OLED.ioc                      STM32CubeMX 工程文件
    STM32F103C8TX_FLASH.ld           链接脚本

## 主机端测试

主机端测试不需要 ARM 编译器。需要安装 CMake 和本机 C 编译器。

在项目根目录执行：

    cmake -S tests -B build-tests
    cmake --build build-tests
    ctest --test-dir build-tests --output-on-failure

测试包括：

- ecg_hr_tests：ECG 滤波和心率检测算法
- max30102_spo2_tests：MAX30102 SpO2 算法

## 固件构建

固件 CMake 配置使用以下 ARM GNU 工具链：

- arm-none-eabi-gcc
- arm-none-eabi-g++
- arm-none-eabi-objcopy
- arm-none-eabi-size

确认工具链已加入 PATH 后，可以在项目根目录执行：

    cmake -S . -B build-firmware -DCMAKE_BUILD_TYPE=Debug
    cmake --build build-firmware

构建后会生成 ELF、HEX 和 BIN 文件。项目同时保留了 Keil、STM32CubeMX 和 Eclipse 工程相关文件，具体使用哪个 IDE 取决于本地开发环境。

## 注意事项

1. 两路 I2C 都是 GPIO 软件模拟总线，请按上表连接，不能按 STM32 硬件 I2C 引脚连接。
2. ECG 采集源切换后，滤波器和心率检测器会重新初始化，需要等待新的 2 秒窗口建立阈值。
3. MAX30102 初始化失败时，OLED 和串口会报告初始化状态，但 ECG 采集仍可独立运行。
4. 本项目用于嵌入式开发、算法验证和实验数据采集，不用于医疗诊断。

