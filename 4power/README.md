# T3 四轮驱动小车（当前为速度闭环与循迹测试模式）

MCU 为 MSPM0G3507，工程由 TI Code Composer Studio、SysConfig 和 TI Clang 构建。

## 当前测试功能

- 上电后四路电机关闭，公共 STBY 拉低。
- 按下 KEY1 后，A/B/C/D 四轮均以同一目标速度运行 2 秒；编码器速度 PI 每 20 ms 更新一次。
- KEY1 测试期间 OLED 显示 A/B/C/D 四轮的 `T`（目标 pps）与 `M`（实测 pps）。
- 2 秒由 MSPM0 Cortex-M 的 SysTick 1 ms 时基计时，不依赖 WIT 陀螺仪数据频率。
- 时间到后四轮短路制动，公共 STBY 拉低；任意轮连续 500 ms 没有编码器脉冲也会保护停车。
- 按下 KEY2 后启动灰度循迹；循迹 PID 每 5 ms 更新一次，并使用同一个四轮速度闭环作为内环，不调用陀螺仪控制。
- 八路灰度连续 200 ms 没有检测到黑线后，KEY2 循迹自动刹车并进入待机。
- OLED 同时显示 KEY1/KEY2 状态、WIT 偏航角和八路灰度传感器。
- KEY3、KEY4 不绑定任何软件功能。

## 主要文件

| 文件 | 用途 |
| --- | --- |
| `empty.c` | KEY1 四轮 2 秒速度闭环测试、KEY2 循迹与丢线停车 |
| `motor.c/.h` | 四路电机方向和 PWM 驱动 |
| `encoder.c/.h` | 四组 AB 相编码器 4 倍频解码 |
| `speed_control.c/.h` | 四轮独立速度 PI 和堵转保护 |
| `control_straight.c/.h` | WIT 偏航角直行控制 |
| `gray.c/.h` | 八路灰度传感器采样 |
| `empty.syscfg` | MSPM0 外设和引脚配置 |
| `4POWER_PINOUT.md` | 完整引脚表、接线和方向调试说明 |

## 构建

在 CCS 中导入 `4power` 工程后构建。SysConfig 会根据 `empty.syscfg` 自动生成 `ti_msp_dl_config.c/.h`，输出固件为 `Debug/4power.out`。

首次上车前先架空车轮，逐轮确认：正电机指令使车轮向前，向前转动产生正编码器计数。方向修正宏的位置见 `4POWER_PINOUT.md`。
