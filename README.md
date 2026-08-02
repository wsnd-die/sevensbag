# 七包（Sevensbag）— STM32G4 麦轮小车控制系统

基于 STM32G4 系列 MCU + FreeRTOS 的麦克纳姆轮小车控制系统，支持 K230 视觉模块通信、路径点导航、语音控制、颜色识别等功能。

## 硬件平台

- **主控**：STM32G4 系列
- **通信**：K230 视觉模块（UART + DMA）
- **驱动**：麦克纳姆轮四电机（FDCAN）
- **外设**：语音识别模块、颜色传感器、OLED 显示屏

## 项目结构

```
STM32G4_banyunche/
├── Core/               # STM32CubeMX 生成的 HAL 层代码
│   ├── Inc/            # 头文件（外设初始化）
│   └── Src/            # 源文件（main.c、中断、DMA、FreeRTOS 任务）
├── app/                # 应用层
│   ├── Mecanum_Move.c/h      # 麦克纳姆轮运动学解算
│   ├── NavigationMecanum.c/h # 路径点导航 + 梯形速度斜坡控制
├── hardware/           # 硬件驱动层
│   ├── k230.c/h              # K230 视觉模块通信（DMA接收）
│   ├── Send_motor.c/h        # 电机 CAN 发送
│   ├── Uart3_yuyin.c/h       # 语音识别模块
│   └── Common_used.h         # 公共定义
├── Middlewares/        # FreeRTOS 中间件
└── MDK-ARM/            # Keil MDK 工程文件
```

## 主要功能

### 1. 麦克纳姆轮运动控制
- 全向移动：平移、斜移、原地旋转
- 运动学正/逆解算

### 2. 路径点导航
- 多点路径规划
- 梯形速度斜坡控制
- 实时位姿跟踪

### 3. K230 视觉模块通信
- UART DMA 接收，低延迟数据处理
- 目标识别信息解析

### 4. 语音控制
- 串口语音模块指令接收与解析

### 5. 颜色识别
- 颜色传感器数据采集与处理

## 更新日志

### 2026-08-02
- **K230 通信**：优化 DMA 接收与数据解析逻辑
- **导航函数**：完善路径点导航，增加梯形速度斜坡控制

## 开发环境

- **IDE**：Keil MDK / STM32CubeIDE
- **RTOS**：FreeRTOS（CMSIS-OS v2）
- **代码生成**：STM32CubeMX
- **编译器**：ARM Compiler 6

## 构建与烧录

1. 使用 Keil MDK 打开 `STM32G4_banyunche/MDK-ARM/STM32G4_TEST.uvprojx`
2. 编译（Build）工程
3. 通过 ST-Link / DAP-Link 烧录到目标板

## License

MIT License
