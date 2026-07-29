# STM32G4_banyunche 项目梳理

## 范围和假设

本文档梳理当前项目根目录下的业务代码、启动代码、外设配置、构建配置和工具脚本。`Drivers/`、`Middlewares/`、`build/`、`MDK-ARM/STM32G4_TEST/` 中包含大量厂商库、FreeRTOS/CMSIS/DSP 源码和编译产物，本文按目录职责汇总，不逐个厂商库函数展开。
仓库当前只保留 `STM32G4_banyunche` 作为主工程目录，旧的 `STM32G4_TEST - 啊克曼解算2` 中文目录已从本地和 Git 跟踪中移除。

当前 CMake 编译使用 `ackermann1.c` 和 `imu_660.c`；`ackermann.c`、`spi_imu660rc.c` 是备用/旧实现，文件仍然保留，但不是 CMake 的当前业务源列表。
本工程同时保留三套编译入口：Keil MDK 工程、VSCode 中的 EIDE 工程，以及另一个用于 Cline 的 CMake 编译入口；后续源文件或外设配置变更需要同步关注三套入口的源文件列表和说明文档。

## 项目总体用途

这是一个基于 STM32G491xx 的搬运车/移动底盘控制工程。核心任务是：

- 通过 USART2 接收 TBOP10 或上位定位模块输出的位置、速度、IMU yaw/gz 数据。
- 用 `navigation.c` 的导航控制器追踪一组目标点，输出车体坐标系速度 `cmd_vx/cmd_vy/cmd_w`。
- 用 `mecanum.c` 把导航速度解算为四个麦克纳姆轮电机的速度和方向。
- 用 FDCAN + EMM V5 协议控制 1~4 号闭环步进电机同步运动。
- 用 TIM3 PWM 输出舵机角度 `front_angle`，TIM3_CH1/CH2/CH3 均按 180 度舵机 PWM 配置。
- 用 UART3 接收简单命令并通过 `printf` 输出调试/录制数据。
- 用软件 I2C 驱动 OLED 显示定位、速度、yaw、标定和航点状态。

## 启动和运行流程

1. `startup_stm32g491xx.s` 复位后进入 `SystemInit()`，再调用 `main()`。
2. `main()` 调用 `HAL_Init()`，配置系统时钟到 PLL；随后初始化 GPIO、FDCAN1、SPI2、TIM2、TIM3、USART1、USART2、USART3。
3. `main()` 调用 `OLED_Init()` 初始化 OLED，再调用 `Hal_starte()` 开始业务外设：
   - `FDCAN1_UserInit()` 配置扩展帧过滤、启动 FDCAN 和 FIFO0 接收中断。
   - 启动 USART1/USART3 单字节中断接收。
   - `UART2_StartDMAReceive()` 启动 USART2 DMA + IDLE 接收。
   - 启动 TIM3 PWM，当前配置并启动 CH1/CH2/CH3。
   - 启动 TIM2 输入捕获中断，当前 TIM2 未看到后续业务使用。
4. 初始化 CMSIS-RTOS2/FreeRTOS 内核，`MX_FREERTOS_Init()` 创建任务。
5. `osKernelStart()` 后进入多任务运行：
   - `Navigation_TASK` 每 10ms 用定位数据更新导航，生成底盘电机命令。
   - `Send_motor` 每 1ms 设置舵机 PWM。
   - `Uart1M_task` 每 10ms 解析 UART3 命令。
   - `Uart3Yuyin_task` 在航点录制时每 50ms 打印 yaw/x/y。
   - `OLED_TASK` 每 10ms 更新航点录制、里程计标定和 OLED 显示。
   - `StartDefaultTask` 空转延时。

## 主要数据流

`USART2 DMA/IDLE -> UART2_FSM_Parse_Byte() -> TB_position/TB_speed/imu_gz/imu_yaw -> Navigation_TASK -> Nav_Update() -> Mecanum_Calc_Full() -> Send_commandmotor() -> Emm_V5_Vel_Control() -> can_SendCmd() -> FDCAN1`

`USART3 interrupt -> Uart3_deel() -> Data_uart3/FlagOFYuyin -> shell_print3() -> commands_detect3() -> WaypointNav_StartRecord()/WaypointNav_StopRecord()`

`OLED_TASK -> WaypointNav_Update()/Odometry_Calib_Update() -> OLED_Printf()/OLED_Update()`

## Core 目录

### Core/Src/main.c

工程入口和系统初始化文件。

- `Hal_starte()`：启动 FDCAN 用户配置、UART1/UART3 中断接收、UART2 DMA 接收、TIM3 PWM、TIM2 输入捕获。
- `fputc()` / `__io_putchar()` / `_ttywrch()`：把标准输出重定向到 USART3，所以 `printf` 走 UART3。
- `main()`：HAL、时钟、外设、OLED、业务外设和 FreeRTOS 的总入口。
- `SystemClock_Config()`：使用 HSI + PLL 配置系统时钟。
- `HAL_TIM_PeriodElapsedCallback()`：TIM6 中断中调用 `HAL_IncTick()`，作为 HAL tick。
- `Error_Handler()`：错误后关中断并死循环。

### Core/Src/app_freertos.c

FreeRTOS 任务创建和任务主循环。

- `MX_FREERTOS_Init()`：创建 `defaultTask`、`ctrl_motor`、`NAVIGATION`、`uart1_motor`、`Uart3_yuyin`、`OLED` 六个线程。
- `StartDefaultTask()`：默认空任务，仅 `osDelay(1)`。
- `Send_motor()`：循环调用 `Servo_SetAngle(front_angle)`，当前负责舵机 PWM 输出。
- `Navigation_TASK()`：初始化 `NavController` 和 `WaypointNav`，按硬编码坐标点循环导航；当里程计标定中会暂停导航输出，航点录制模式下不干预电机。
- `Uart1M_task()`：当前在非 `ni_he_mode` 下调用 `shell_print3()`，实际解析 UART3 命令。
- `Uart3Yuyin_task()`：航点录制模式下用 `printf` 输出 `imu_yaw,x,y`。
- `OLED_TASK()`：录制模式下记录航点，标定模式下更新标定状态机，并刷新 OLED 状态显示。
- `led()`：翻转 PC13 LED 后延时，当前未被任务调用。

### Core/Src/gpio.c / Core/Inc/gpio.h

GPIO 初始化文件。

- `MX_GPIO_Init()`：初始化 PC13 LED、PB12 SPI2_CS、PB6/PB7 OLED 软件 I2C SCL/SDA。

### Core/Src/fdcan.c / Core/Inc/fdcan.h

CubeMX 生成的 FDCAN1 底层初始化。

- `MX_FDCAN1_Init()`：FDCAN1 Classic CAN、Normal Mode、TX FIFO 模式、名义时序配置。
- `HAL_FDCAN_MspInit()`：配置 PA11/PA12 为 FDCAN1 RX/TX，并使能 `FDCAN1_IT0_IRQn`。
- `HAL_FDCAN_MspDeInit()`：反初始化 FDCAN1。

业务过滤器、启动和接收回调在 `hardware/can.c`。

### Core/Src/usart.c / Core/Inc/usart.h

USART1/2/3 初始化。

- `MX_USART1_UART_Init()`：USART1，115200 8N1，中断收发，用于电机/上位串口数据。
- `MX_USART2_UART_Init()`：USART2，115200 8N1，配置 DMA1_Channel1 作为 RX，用于 TBOP10 定位帧。
- `MX_USART3_UART_Init()`：USART3，115200 8N1，用于命令、调试输出和 `printf`。
- `HAL_UART_MspInit()`：配置 USART 引脚和中断；USART2 额外配置 DMA RX。
- `HAL_UART_MspDeInit()`：反初始化串口资源。

### Core/Src/tim.c / Core/Inc/tim.h

TIM2/TIM3 初始化。

- `MX_TIM2_Init()`：TIM2 32 位基础定时器，当前主流程启动了输入捕获但代码里未看到业务消费。
- `MX_TIM3_Init()`：TIM3 PWM，CH1/CH2/CH3 均为 PWM1，高电平有效，周期 19999，预分频 170，用于 180 度舵机 PWM。
- `HAL_TIM_MspPostInit()`：配置 PA6 为 TIM3_CH1、PA7 为 TIM3_CH2、PB0 为 TIM3_CH3。

### Core/Src/spi.c / Core/Inc/spi.h

SPI2 初始化。

- `MX_SPI2_Init()`：SPI2 主机模式，8 bit，CPOL low/CPHA 1edge，软件 NSS，预分频 32。
- `HAL_SPI_MspInit()`：配置 PB13/PB14/PB15 为 SPI2 SCK/MISO/MOSI。

SPI2 主要供 IMU660RC 文件使用，但 `main()` 中 IMU 初始化当前被注释。

### Core/Src/stm32g4xx_it.c / Core/Inc/stm32g4xx_it.h

中断入口文件。

- `FDCAN1_IT0_IRQHandler()`：转发给 HAL FDCAN 处理。
- `USART1_IRQHandler()` / `USART2_IRQHandler()` / `USART3_IRQHandler()`：转发给 HAL UART 处理。
- `TIM6_DAC_IRQHandler()`：HAL tick 定时器中断。
- `DMA1_Channel1_IRQHandler()`：处理 USART2 RX DMA。
- Fault handlers：异常后进入死循环。

### Core/Src/stm32g4xx_hal_msp.c

HAL MSP 全局初始化文件。

- `HAL_MspInit()`：开启 SYSCFG/PWR 时钟，设置 PendSV 中断优先级。

### Core/Src/stm32g4xx_hal_timebase_tim.c

用 TIM6 替代 SysTick 作为 HAL tick。

- `HAL_InitTick()`：配置 TIM6 每 1ms 触发中断。
- `HAL_SuspendTick()` / `HAL_ResumeTick()`：关闭/打开 TIM6 更新中断。

### Core/Src/system_stm32g4xx.c

STM32G4 系统文件。

- `SystemInit()`：复位后初始化时钟寄存器基础状态。
- `SystemCoreClockUpdate()`：根据 RCC 寄存器更新 `SystemCoreClock`。

### Core/Src/syscalls.c / Core/Src/sysmem.c

newlib/starm 运行时适配文件。

- `syscalls.c`：实现 `_write`、`_read`、`_sbrk` 等系统调用桩，支持标准库。
- `sysmem.c`：实现堆内存分配相关的 `_sbrk`。

### Core/Src/app_freertos_orig.c

原始 FreeRTOS 文件备份/占位，当前内容很少，不参与主流程。

### Core/Inc/FreeRTOSConfig.h

FreeRTOS 配置头：任务调度、tick、堆、断言、中断优先级、CMSIS-RTOS2 适配开关等。

### Core/Inc/stm32g4xx_hal_conf.h

HAL 模块配置头：开启 FDCAN/GPIO/SPI/TIM/UART/DMA/RCC/PWR/CORTEX 等 HAL 模块。

### Core/Inc/main.h

项目公共引脚定义。

- `SPI2_CS_Pin`：PB12。
- `I2C3_SCL_Pin`：PB6。
- `I2C3_SDA_Pin`：PB7。
- `Error_Handler()` 声明。

## hardware 目录

### hardware/Common_used.h

业务公共头文件，集中包含 FreeRTOS、串口、语音、电机、OLED、IMU、阿克曼、麦轮、TBOP、导航等模块，并声明跨文件全局变量和业务函数。

重要宏：

- `use_xing_che`：车型/参数选择开关。
- `ni_he_mode`：UART1 解析模式开关。
- `RX_BUF_SIZE`：UART1 接收缓冲大小。

### hardware/can.c / hardware/can.h

FDCAN 业务封装和电机响应读取。

- `fdcan_dlc_from_len()`：把数据长度转换为 FDCAN DLC。
- `FDCAN_WaitFreeTxFifo()`：等待 TX FIFO 有空位，防止发送队列满。
- `can_SendCmd()`：按 EMM 协议把命令分包为扩展帧发送。
- `HAL_FDCAN_RxFifo0Callback()`：接收 FIFO0 消息，保存到 `can_rx_header/can_rx_data` 并置位 `can_rx_flag`。
- `Emm_V5_Read_Status()`：读取单个电机状态。
- `Emm_V5_Is_Reached()`：判断电机是否到位。
- `FDCAN1_UserInit()`：配置扩展帧全接收、启动 FDCAN、打开 FIFO0 新消息通知。

### hardware/emm_v5.c / hardware/emm_5v.h

EMM V5 闭环步进电机 CAN 命令拼帧库。

- `Emm_V5_Reset_CurPos_To_Zero()`：当前位置清零。
- `Emm_V5_Reset_Clog_Pro()`：解除堵转保护。
- `Emm_V5_Read_Sys_Params()`：读取固件、电压、位置、速度、状态等系统参数。
- `Emm_V5_Modify_Ctrl_Mode()`：修改开环/闭环控制模式。
- `Emm_V5_En_Control()`：电机使能/失能。
- `Emm_V5_Vel_Control()`：速度模式控制，当前底盘控制主要使用它。
- `Emm_V5_Pos_Control()`：位置模式控制。
- `Emm_V5_Stop_Now()`：立即停止。
- `Emm_V5_Synchronous_motion()`：触发多机同步运动。
- `Emm_V5_Origin_*()` 系列：零点设置、回零参数、触发回零、中断回零。

### hardware/Send_motor.c

底盘/舵机输出和 UART1 电机命令解析。

- `Send_commandmotor()`：把 `MecanumResult` 映射到 1~4 号电机，发送速度命令后用广播地址 `0` 触发同步运动。
- `Servo_SetAngle()`：限制角度 0~180 度，并把角度转换成 TIM3_CH1 PWM 比较值。
- `commands_detect()`：`use_nanof` 分支下解析 UART1 的左右电机速度、加速度、方向和舵机角。
- `comamd_detect()`：非 `use_nanof` 分支下把 UART1 数据解析为 `motor_v/motor_w` 两个 float。
- `shell_print()`：UART1 收到完整帧后触发解析并清标志。

### hardware/mecanum.c / hardware/mecanum.h

麦克纳姆轮运动学、编码器读取和里程计标定。

- `Mecanum_ProcessWheel()`：把单轮原始速度转换为方向和无符号速度。
- `Mecanum_Calc()`：单轴 `v + w` 逆运动学，输出四轮速度。
- `Mecanum_Calc_Full()`：三自由度 `vx + vy + w` 逆运动学，当前导航任务使用。
- `malu_cm_topluse_s()`：按轮半径把厘米转换为脉冲数。
- `Mecanum_Read_Speed()`：通过 CAN 读取单电机转速。
- `Mecanum_Read_Position()`：通过 CAN 读取单电机累计位置。
- `Mecanum_Read_AllPositions()`：读取四轮编码器位置。
- `Odometry_Calib_Start()`：开始里程计标定，目标前进/右移各约 1m。
- `Odometry_Calib_Update()`：标定状态机，驱动车辆、用 TBOP 位移计算编码器比例。
- `Odometry_Is_Calibrated()`：判断标定完成。
- `Odometry_Apply_Calib()`：把编码器增量换算成 mm，未标定时用粗略轮径估算。

### hardware/navigation.c / hardware/navigation.h

位置环 + 航向角串级控制器。

- `norm_deg()`：把角度归一到 -180~180 度。
- `PosLoop_Update()`：根据目标位置和当前坐标计算 `cmd_vx/cmd_vy`。
- `YawLoop_Update()`：对角速度滤波，用角度环生成目标角速度，再用角速度环输出 `cmd_w`。
- `Nav_Init()`：初始化控制器、PID 参数、限幅、到达阈值和陀螺仪滤波参数。
- `Nav_SetTarget()`：设置目标点并清 PID。
- `Nav_Update()`：一次完整导航更新，包含位置环、航向环和到达判定。
- `Nav_Arrived()`：判断是否到达。

### hardware/waypoint.c / hardware/waypoint.h

航点录制/回放系统。

- `g_path`：全局环形航点缓冲。
- `g_waypoint_nav`：全局航点导航适配器。
- `waypoint_init()` / `waypoint_clear()`：初始化/清空航点缓冲。
- `waypoint_record()`：把当前 x/y/yaw 写入环形缓冲。
- `waypoint_count()`：返回当前航点数。
- `waypoint_get()` / `waypoint_get_target()`：按索引读取航点。
- `WaypointNav_Init()`：初始化航点系统和内部 `NavController`。
- `WaypointNav_StartRecord()` / `WaypointNav_StopRecord()`：开始/停止录制。
- `WaypointNav_StartPlayback()`：以当前航点快照开始回放。
- `WaypointNav_Update()`：录制模式按时间间隔记录；回放模式驱动内部导航器切换目标点。
- `WaypointNav_Arrived()`：判断回放是否完成。
- `waypoint_export()`：通过 UART3 以 CSV 形式导出航点。

当前 `commands_detect3()` 只绑定了录制开始 `'r'` 和停止 `'s'`，未绑定回放命令。

### hardware/Uart2_tbop10.c / hardware/uart2_tbop10.h

USART2 TBOP10 定位帧解析。

- `TB_position`：当前位置 x/y。
- `TB_speed`：速度 x/y。
- `imu_gz` / `imu_yaw`：来自帧内的角速度和航向角。
- `UART2_FSM_Parse_Byte()`：按帧头 `0xAA 0xCC`、数据区、帧尾 `0xBB 0xDD` 解析 28 字节帧。
- `HAL_UARTEx_RxEventCallback()`：USART2 DMA + IDLE 回调，把收到的数据逐字节送入状态机，并重启 DMA。
- `UART2_StartDMAReceive()`：清错误标志并启动 ReceiveToIdle DMA。
- `UART2_Send()` / `UART2_SendCode()`：阻塞式发送 USART2 数据。

### hardware/Uart1_motor.c

USART1/USART3 单字节中断接收回调。

- `Uart1_deel()`：按 `0x55 ... 0xFA` 帧格式解析 UART1；`ni_he_mode` 下直接写 `front_angle`，否则设置 `FlagOFMotor`。
- `HAL_UART_RxCpltCallback()`：USART1 收到字节后调用 `Uart1_deel()` 并续收；USART3 收到字节后翻转 PC13、调用 `Uart3_deel()` 并续收。
- `HAL_UART_ErrorCallback()`：清 UART 溢出/帧错误/噪声错误并重新开启接收。

### hardware/Uart3_yuyin.c

USART3 命令解析和语音路径表。

- `Uart3_deel()`：支持两种命令格式：`0x5F cmd 0xFB` 帧格式，或裸字节命令。
- `commands_detect3()`：解析 `Data_uart3[1]`；当前 `'r'` 开始航点录制，`'s'` 停止航点录制。
- `shell_print3()`：当 `FlagOFYuyin` 置位时调用命令解析。

### hardware/Send_yuyin.c / hardware/Send_yuyin.h

JQ8x00 语音模块驱动。

- `JQ8x00_Init()`：初始化忙脚/VPP，当前部分代码是旧标准库风格，是否生效由宏控制。
- `OneLine_SendData()` / `OneLine_ByteControl()` / `OneLine_ZHControl()`：一线串口模式发送控制。
- `JQ8x00_ZuHeBoFang()`：组合播放命令。
- `JQ8x00_Command()` / `JQ8x00_Command_Data()`：UART 模式发送基础命令。
- `JQ8x00_RandomPathPlay()`：按文件路径播放音频。
- `Send_commendyu()`：根据 `buffer_flag/buf` 播放对应音频文件。

### hardware/pid.c / hardware/pid.h

通用 PID 控制器。

- `PID_init()`：设置 PID 模式、Kp/Ki/Kd、输出限幅和积分限幅。
- `PID_calc()`：支持位置式、增量式、带积分分离的增量式 PID。
- `PID_clear()`：清空 PID 历史状态和输出。

### hardware/ackermann1.c / hardware/ackermann1.h

阿克曼底盘解算当前实现。当前项目主要跑麦轮，但该文件被 CMake 编译。

- `angle_to_phid()`：用三次拟合把转向角转换为舵机 PWM 角度/标定值，并对小角度左转做补偿。
- `Ackerman_Calc()`：根据线速度 `v` 和角速度 `w` 计算左右轮速度、方向和舵机值。

### hardware/ackermann.c / hardware/ackermann.h

旧版/备用阿克曼解算。

- `angle_to_phid()`：用二次拟合反解舵机值。
- `Ackerman_Calc()`：根据 `v/w` 计算左右轮速度和舵机值。

当前 CMake 注释明确它是备用实现，不在 CMake 当前源列表中。

### hardware/imu_660.c / hardware/imu660.h

IMU660RC SPI 驱动和姿态解算当前实现。

- `IMU660RC_Init()`：读取 WHO_AM_I 并配置加速度计、陀螺仪量程和输出频率。
- `IMU660RC_ReadAcc()`：读取加速度原始数据并换算。
- `IMU660RC_ReadGyro()`：读取陀螺仪原始数据并换算。
- `IMU660RC_WriteRegs()` / `IMU660RC_ReadRegs()` / `IMU660RC_ReadMultiRegs()`：SPI 寄存器读写。
- `quat_to_euler()`：四元数转 roll/pitch/yaw。
- `IMU660RC_AttitudeUpdate()`：Mahony 风格姿态更新，使用加速度和陀螺仪融合。
- `IMU660RC_AttitudeInit()`：姿态初始化和偏置估计。

注意：`main()` 中 `IMU660RC_Init()` 和 `IMU660RC_AttitudeInit()` 当前被注释，导航实际使用的是 USART2 帧里的 `imu_yaw/imu_gz`。

### hardware/spi_imu660rc.c / hardware/spi_imu660rc.h

IMU660RC 备用实现，函数名与 `imu_660.c` 基本相同。当前不在 CMake 源列表中，不能和 `imu_660.c` 同时编译，否则同名函数会冲突。

### hardware/Gu_dao.c

基于 IMU 加速度积分的惯导速度估计实验模块。

- `Guan_dao_Reset()`：清空积分速度。
- `Guan_dao()`：把车体加速度转到世界坐标，做死区处理后积分得到 `V_x/V_y`。
- `guan_init()`：初始化 IMU。

当前主流程未调用 `Guan_dao()`。

### hardware/oled.c / hardware/oled.h

OLED 软件 I2C 驱动和绘图库。

- `OLED_GPIO_Init()`：初始化 PB6/PB7 开漏输出。
- `OLED_I2C_Start()` / `OLED_I2C_Stop()` / `OLED_I2C_SendByte()`：软件 I2C 基础时序。
- `OLED_WriteCommand()` / `OLED_WriteData()`：向 SSD1306 类 OLED 写命令/数据。
- `OLED_Init()`：初始化 OLED 控制器。
- `OLED_Update()` / `OLED_UpdateArea()`：把显存刷新到屏幕。
- `OLED_Clear()` / `OLED_ClearArea()` / `OLED_Reverse()` / `OLED_ReverseArea()`：显存清屏和反色。
- `OLED_ShowChar()` / `OLED_ShowString()` / `OLED_ShowNum()` / `OLED_ShowSignedNum()` / `OLED_ShowHexNum()` / `OLED_ShowBinNum()` / `OLED_ShowFloatNum()` / `OLED_ShowChinese()` / `OLED_ShowImage()` / `OLED_Printf()`：文本、数字、图片显示。
- `OLED_DrawPoint()` / `OLED_GetPoint()` / `OLED_DrawLine()` / `OLED_DrawRectangle()` / `OLED_DrawTriangle()` / `OLED_DrawCircle()` / `OLED_DrawEllipse()` / `OLED_DrawArc()`：基础图形绘制。

### hardware/oled_data.c / hardware/oled_data.h

OLED 字库和图片数据。

- `OLED_F8x16`：8x16 ASCII 字库。
- `OLED_F6x8`：6x8 ASCII 字库。
- `OLED_CF16x16`：中文字符点阵表。
- `Diode`：示例图片/图标数据。

## 根目录和构建配置文件

工程可通过 Keil MDK、VSCode EIDE 和 Cline 使用的 CMake 入口编译。新增或删除业务源文件时，需要同步检查 `MDK-ARM/STM32G4_TEST.uvprojx`、EIDE 配置和 CMake 源文件列表。

### CMakeLists.txt

顶层 CMake 工程文件。

- 设置 C11、工程名 `STM32G4_TEST`。
- 添加 `cmake/stm32cubemx` 子目录。
- 手动加入业务源文件 `hardware/*.c`。
- 包含 `hardware` 头文件目录。
- 链接 `stm32cubemx` 接口库。
- 构建后生成 `.hex` 和 `.bin`。

### CMakePresets.json

CMake 预设配置，通常用于指定 toolchain、构建目录和 Debug/Release 配置。

### cmake/stm32cubemx/CMakeLists.txt

CubeMX 生成源、HAL、FreeRTOS、CMSIS 路径和库链接配置。

- `MX_Application_Src`：加入 `Core/Src` 和启动汇编。
- `STM32_Drivers_Src`：加入 STM32G4 HAL 驱动。
- `FreeRTOS_Src`：加入 FreeRTOS 内核和 CMSIS-RTOS2 适配。
- `MX_LINK_LIBS`：链接 ARM DSP 库、HAL 对象库和 FreeRTOS 对象库。

### cmake/gcc-arm-none-eabi.cmake

GCC Arm Embedded toolchain 配置。

### cmake/starm-clang.cmake

Arm clang/starm toolchain 配置。

### STM32G4_TEST.ioc

STM32CubeMX 工程配置文件，描述芯片、时钟、引脚、外设和 FreeRTOS 配置。修改外设时通常从这里重新生成 `Core/` 配置代码。

### STM32G491XX_FLASH.ld

GCC 链接脚本，定义 Flash/RAM 布局、段放置规则和入口符号。

### startup_stm32g491xx.s

GCC/ARM 启动汇编，定义中断向量表、复位入口、栈、默认中断处理函数，并跳转到 `main()`。

### daplink.cfg

OpenOCD/DAPLink 调试或下载配置。

### .mxproject

CubeMX 元数据文件。

### .gitignore

Git 忽略规则。
已按忽略规则停止跟踪历史提交过的构建产物和 IDE 临时文件；本地文件保留，但后续不再进入 Git 索引。

### tools/waypoint_plot.m

MATLAB/Octave 脚本，用于绘制或检查航点轨迹数据。

## MDK-ARM 和 IDE 文件

### MDK-ARM/STM32G4_TEST.uvprojx

Keil MDK 工程文件，保存源文件列表、目标芯片、编译选项和分组。

### MDK-ARM/STM32G4_TEST.uvoptx

Keil 用户选项文件，保存调试器、窗口、断点等本地配置。

### MDK-ARM/startup_stm32g491xx.s

Keil/ARMCC 风格启动汇编副本。

### MDK-ARM/EIDE/*

EIDE 插件的工作区和 RTE 组件配置。

### MDK-ARM/STM32G4_TEST/*

Keil 构建产物目录，包含 `.axf`、`.sct`、`.lnp`、`.dep`、构建日志等，不属于手写业务逻辑。

### .eide/ .idea/ .vscode/ .claude/

本地 IDE、编辑器或工具配置目录，不直接参与固件运行逻辑。

## 外部依赖目录

### Drivers/

STM32 HAL、CMSIS、CMSIS-DSP、CMSIS-NN 等厂商/ARM 库源码和头文件。项目直接依赖的重点是：

- `Drivers/STM32G4xx_HAL_Driver/Inc|Src`：GPIO、FDCAN、UART、SPI、TIM、DMA、RCC、PWR 等 HAL 驱动。
- `Drivers/CMSIS/Device/ST/STM32G4xx/Include`：STM32G4 设备寄存器定义。
- `Drivers/CMSIS/Include`：Cortex-M CMSIS 核心头。
- `Drivers/CMSIS/DSP` / `Middlewares/ST/ARM/DSP`：ARM DSP 库，项目中 `mecanum.c` 和 `ackermann1.c` 包含 `arm_math.h`。

### Middlewares/

第三方中间件。

- `Middlewares/Third_Party/FreeRTOS/Source`：FreeRTOS 内核、portable 层、heap_4 和 CMSIS-RTOS2 适配。
- `Middlewares/ST/ARM/DSP`：ST 打包的 ARM DSP 头/库。

### build/

CMake 构建输出目录，包含编译中间文件、最终 ELF/HEX/BIN、compile_commands 等产物。

## 当前代码中的关键注意点

- `Navigation_TASK` 已经在跑硬编码的 12 个目标点，航点回放逻辑存在但当前命令入口没有调用 `WaypointNav_StartPlayback()`。
- `Uart1M_task` 名字像 UART1 电机任务，但当前非 `ni_he_mode` 分支调用的是 `shell_print3()`，实际处理 UART3 命令。
- `main()` 中 IMU 初始化被注释，当前导航 yaw/gz 主要来自 USART2 的 TBOP 帧，而不是板载 IMU 驱动。
- `TIM3` 已在 CubeMX 和代码中配置 CH1/CH2/CH3，`Hal_starte()` 同步启动三路 PWM；后续外设配置变更需要同步更新本文档。
- `can.h` 中 `can_SendCmd(__IO uint8_t *cmd, uint8_t len)` 与 `can.c` 中 `can_SendCmd(uint8_t *cmd, uint8_t len)` 参数限定符不完全一致，通常可编译但建议统一。
- `mecanum.h` 中 `MEC_WHEEL_RADIUS` 注释写 m，但数值 `3.75f` 更像 cm 或其他单位；`Odometry_Apply_Calib()` 又按 mm 粗略换算使用，后续做里程计时需要统一单位。
- 多个中文注释存在编码异常，不影响编译，但影响后续维护阅读。
