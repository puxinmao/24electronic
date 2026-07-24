# T3 四轮驱动小车（当前为单轮测试模式）

MCU 为 MSPM0G3507，工程由 TI Code Composer Studio、SysConfig 和 TI Clang 构建。

## 当前测试功能

- 上电后四路电机关闭，公共 STBY 拉低。
- 按下 KEY1 后仅 A 左前轮以 PWM=1500 持续运行。
- B、C、D 三路保持关闭，便于测量一路电机电流。
- 当前测试模式不会因黑色、WIT 或编码器状态自动停车。
- 结束测试时需要复位单片机或断开电机电源。
- KEY2、KEY3、KEY4 不绑定任何软件功能。

## 主要文件

| 文件 | 用途 |
| --- | --- |
| `empty.c` | KEY1 启动 A 左前轮持续运行的临时测试主流程 |
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
