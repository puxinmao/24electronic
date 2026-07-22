# 编码器速度闭环调节说明

## 1. 控制结构

当前程序使用两层控制：

- 外环：`control_straight.c` 的偏航 PID，或 `control_line.c` 的巡线 PID，负责决定左右轮应该产生多大差速。
- 内环：`speed_control.c` 的左右轮独立 PI，负责让编码器实测速度跟随各自的目标速度。

速度内环暂时使用 PI 而不是完整 PID。编码器在 20 ms 内得到的是离散脉冲数，直接对这种速度做微分容易放大跳变和漏脉冲噪声；当前先用 P 提高响应、用 I 消除稳态误差，更容易稳定调好。

上层参数仍沿用原来的反向 PWM 标度：数值越小目标速度越快，数值越大目标速度越慢。因此 `STRAIGHT_SPEED_NORMAL=700`、`LINE_SPEED_NORMAL=700`、`LINE_SPEED_ENTRY=1000` 可以继续作为主要速度参数使用。

速度环单位为编码器 4 倍频脉冲/秒（pps），不是转/分钟。只有知道每圈脉冲数和轮径后，才需要换算成 RPM 或 m/s。

## 2. 速度闭环参数

以下宏位于 `speed_control.c` 顶部：

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `SPEED_CLOSED_LOOP_ENABLE` | `1` | `1` 开启闭环，`0` 保留原开环 PWM，便于对照 |
| `SPEED_CONTROL_PERIOD_MS` | `20` | 速度采样和 PI 周期，当前为 50 Hz |
| `SPEED_FULL_OUTPUT_PPS` | `4000` | 满输出时估计的编码器速度，是第一个需要实测校准的参数 |
| `SPEED_KP` | `0.08` | 速度误差的即时修正强度 |
| `SPEED_KI` | `0.15` | 消除持续速度误差，补偿电池和负载变化 |
| `SPEED_CORRECTION_LIMIT` | `350` | 速度 PI 最多能在原速度指令上补偿多少 PWM |
| `SPEED_FILTER_ALPHA` | `0.40` | 新速度样本权重；越小越平滑、响应越慢 |
| `SPEED_LEFT_TARGET_SCALE` | `1.0` | 左轮目标速度比例，通常保持 1 |
| `SPEED_RIGHT_TARGET_SCALE` | `1.0` | 右轮目标速度比例，通常保持 1 |
| `SPEED_STALL_PROTECTION_ENABLE` | `1` | `1` 启用单轮无脉冲停车，调试特殊低速动作时可暂时设为 `0` |
| `SPEED_STALL_TARGET_MIN_PPS` | `600` | 目标低于该值不判断堵转，避免急弯内侧低速轮误报 |
| `SPEED_STALL_TIMEOUT_MS` | `500` | 有较高目标速度但连续无编码器脉冲时的停车时间 |

## 3. 串口数据怎么看

运行时每 200 ms 输出一行：

```text
SPD TL:2600 ML:2510 PL:680 TR:2600 MR:2470 PR:665
```

- `TL/TR`：左右轮目标速度，单位 pps。
- `ML/MR`：左右轮滤波后的实测速度，单位 pps。
- `PL/PR`：速度环最终写入左右 PWM 的数值；仍然是数值越小驱动力越强。

如果抬起小车测试，轮子负载明显小于落地状态，测出的参数只能作为初值。最终参数应以车辆落地直行为准。

## 4. 推荐调节顺序

### 第一步：确认编码器脉冲

先架空车辆、确保 KEY2 可立即急停。启动后观察：

- 两轮转动时，`ML`、`MR` 都应持续为大于 0 的数。这里显示的是绝对速度，只能确认有没有脉冲，不能用来判断 A/B 相方向。
- 当前速度环不依赖编码器正负方向，所以 A/B 相方向相反不会导致速度环反向失控。以后若加入里程、位置或转向闭环，再单独检查并统一累计计数方向。
- 两轮都应持续有数据。如果出现 `ENC LEFT` 或 `ENC RIGHT`，先检查编码器接线和中断，不要先提高 PI。
- 若急弯时偶发 `ENC LEFT/RIGHT`，先确认该轮在弯中是否本来就接近停转；可增大 `SPEED_STALL_TARGET_MIN_PPS` 或 `SPEED_STALL_TIMEOUT_MS`，不要用增大 PI 来掩盖误报。

### 第二步：校准 `SPEED_FULL_OUTPUT_PPS`

把 `SPEED_CLOSED_LOOP_ENABLE` 暂时改为 `0`，保持 `LINE_SPEED_NORMAL` 或 `STRAIGHT_SPEED_NORMAL` 为 `700`，观察稳定后的 `ML/MR` 平均值。

当前映射公式为：

```text
目标速度 = (2000 - 速度指令) / 2000 * SPEED_FULL_OUTPUT_PPS
```

因此在速度指令为 `700` 时：

```text
SPEED_FULL_OUTPUT_PPS = 实测平均速度 * 2000 / 1300
```

例如左右轮落地平均约为 `2340 pps`：

```c
#define SPEED_FULL_OUTPUT_PPS 3600.0f
```

校准后把 `SPEED_CLOSED_LOOP_ENABLE` 改回 `1`。

### 第三步：只调 `Kp`

先临时设置：

```c
#define SPEED_KI 0.0f
```

从 `SPEED_KP=0.08` 开始：

- `ML/MR` 跟不上目标、变化很慢：每次增加 `0.02`。
- PWM 和速度快速来回跳动：每次减少 `0.02`。
- 推荐先在 `0.04～0.20` 范围内寻找稳定值。

### 第四步：加入 `Ki`

确定 `Kp` 后恢复 `SPEED_KI=0.15`：

- 实测速度长期低于目标：每次增加 `0.05`。
- 起步后明显越冲越快，或速度低频来回摆动：减小 `Ki`。
- 一般先在 `0.05～0.40` 范围内调节。

`Ki` 的作用是消除持续误差，不负责第一时间响应，因此不要用很大的 `Ki` 代替不足的 `Kp`。

### 第五步：处理左右轮差异

闭环稳定后，左右轮目标相同但某一轮始终难以达到目标，先检查机械阻力和编码器脉冲数。如果两侧硬件正常但编码器每圈脉冲数确实不同，再微调：

```c
#define SPEED_LEFT_TARGET_SCALE  1.00f
#define SPEED_RIGHT_TARGET_SCALE 0.98f
```

不要优先用该比例掩盖电机卡滞、轮胎摩擦或编码器漏脉冲。

## 5. 现象与参数对应

| 现象 | 优先处理 |
| --- | --- |
| 闭环开启后整体速度明显改变 | 校准 `SPEED_FULL_OUTPUT_PPS` |
| 加速和负载变化时跟随太慢 | 增大 `SPEED_KP` |
| 速度和 PWM 快速抖动 | 减小 `SPEED_KP`，或减小 `SPEED_FILTER_ALPHA` |
| 稳定后仍长期低于目标 | 增大 `SPEED_KI` |
| 低频忽快忽慢、越调越过头 | 减小 `SPEED_KI` |
| PI 已到极限仍跟不上 | 增大 `SPEED_CORRECTION_LIMIT`，或降低目标速度 |
| 弯道转不过去 | 先调巡线差速参数，不要先调速度 PI |
| 直线方向蛇形 | 先区分速度环抖动还是偏航/巡线外环抖动，再调对应环路 |

## 6. 与原参数的关系

- 想改变整车快慢：仍先调 `STRAIGHT_SPEED_NORMAL`、`LINE_SPEED_NORMAL` 和 `LINE_SPEED_ENTRY`。
- 想改变直线方向纠偏：调 `STRAIGHT_KP/KI/KD`。
- 想改变巡线转向：调 `LINE_KP/KI/KD`、`LINE_CORR_LIMIT` 和边缘转向参数。
- 想让实际轮速更稳定地跟住目标：调 `SPEED_KP/KI`。

调试时一次只调整一层。先让速度内环在直行状态稳定，再调偏航直行，最后调巡线；否则多个环路同时变化，很难判断问题来源。
