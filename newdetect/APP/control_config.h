#ifndef CONTROL_CONFIG_H_
#define CONTROL_CONFIG_H_

/* K230 图像宽度，单位：像素。 */
#define K230_IMAGE_WIDTH                  (640)
/* K230 图像高度，单位：像素；当前控制只使用横向 x 坐标。 */
#define K230_IMAGE_HEIGHT                 (360)
/* x 偏差映射后的最大位置，单位：0.01 cm，即正负 12.50 cm。 */
#define BALL_POSITION_LIMIT_CENTI_CM      (1250)
/* K230 识别置信度低于此值时，视为未可靠检测到球。 */
#define BALL_MIN_SCORE                    (35U)

/* 球进入此范围且速度较低时停止电机，单位：cm。 */
#define BALL_DEAD_ZONE_ENTER_CM           (0.80f)
/* 已停止后，球超出此范围才重新追球；必须大于 ENTER，单位：cm。 */
#define BALL_DEAD_ZONE_EXIT_CM            (1.50f)
/* 判定球在中心稳定的最大速度，单位：cm/s。 */
#define BALL_HOLD_VELOCITY_CM_S           (1.00f)
/* 已进入中心保持后，速度超过此值立即恢复制动，必须略大于进入阈值，单位：cm/s。 */
#define BALL_HOLD_VELOCITY_EXIT_CM_S      (1.50f)
/* 位置预测时间，单位：s；根据当前球速度预测未来位置后送入位置 PID。 */
#define BALL_PREDICTION_TIME_S            (0.15f)
/* 预测位移绝对上限，单位：cm；防止视觉速度噪声造成误动作。 */
#define BALL_PREDICTION_MAX_CM            (2.00f)
/* 球正朝中心运动时的最小安全制动距离，单位：cm。 */
#define BALL_BRAKE_SAFETY_DISTANCE_CM     (1.20f)
/* 制动距离模型中的可实现减速度，单位：cm/s^2；取小可更早制动。 */
#define BALL_BRAKE_DECELERATION_CM_S2     (2.00f)
/* 进入制动区后要求的反向目标速度，单位：cm/s。 */
#define BALL_BRAKE_TARGET_VELOCITY_CM_S   (0.00f)
/* 低于此球速时不进入主动制动，避免视觉噪声触发换向。 */
#define BALL_BRAKE_MIN_VELOCITY_CM_S      (0.60f)
/* 球在该范围内且已基本静止时，锁定朝向中心的低速自适应微调。 */
#define BALL_FINE_TRIM_ZONE_CM             (4.00f)
#define BALL_FINE_TRIM_VELOCITY_CM_S       (0.40f)
/* 精调锁存后允许的速度噪声；超过该值才交回动态制动控制。 */
#define BALL_FINE_TRIM_RELEASE_VELOCITY_CM_S (1.20f)

/* 位置内环比例系数；增大追回更快，也更容易冲过中心。 */
#define POSITION_PID_KP                   (0.60f)
/* 位置内环积分系数；消除长期静差，过大容易缓慢摆动。 */
#define POSITION_PID_KI                   (0.0f)
/* 位置内环微分系数；根据球速度提前刹车，减小冲过头。 */
#define POSITION_PID_KD                   (1.80f)
/* 位置内环积分累积的绝对上限，防止积分饱和。 */
#define POSITION_PID_INTEGRAL_LIMIT       (12.0f)
/* 位置内环输出的最大期望球速度，单位：cm/s。 */
#define BALL_TARGET_VELOCITY_LIMIT_CM_S   (4.0f)

/* 速度外环比例系数；将速度误差转换为电机输出强度。 */
#define VELOCITY_PID_KP                   (4.0f)
/* 速度外环积分系数；修正持续存在的速度误差。 */
#define VELOCITY_PID_KI                   (0.0f)
/* 速度外环微分系数；抑制速度误差突变导致的冲击。 */
/* 先关闭速度环微分：视觉速度存在帧间量化噪声，KD 过大会造成电机方向频繁翻转。 */
#define VELOCITY_PID_KD                   (0.0f)
/* 速度外环积分累积的绝对上限，防止积分饱和。 */
#define VELOCITY_PID_INTEGRAL_LIMIT       (20.0f)
/* PID 输出达到该值时使用最高转速，单位：pulse/s；用于把 PID 输出映射为实际 RPM。 */
#define MOTOR_PULSE_RATE_LIMIT_PPS        (80.0f)
/* 输出低于此阈值时停止电机，避免最低 1 RPM 在中心附近持续推球。 */
#define MOTOR_PULSE_RATE_STOP_THRESHOLD_PPS (0.5f)
/* 旧版短脉冲控制的最小脉冲数；连续速度控制中不使用，保留兼容。 */
#define MOTOR_COMMAND_MIN_PULSES          (2U)
/* 旧版短脉冲控制的最大脉冲数；连续速度控制中不使用，保留兼容。 */
#define MOTOR_COMMAND_MAX_PULSES          (96U)

/* 球速度低通滤波系数，范围 0 到 1；越小越平滑但反应越慢。 */
#define BALL_VELOCITY_FILTER_ALPHA        (0.35f)
/* 首帧或异常时采用的控制周期，单位：ms。 */
#define CONTROL_DT_DEFAULT_MS             (33U)
/* 超过此帧间隔认为控制周期异常，单位：ms。 */
#define CONTROL_DT_MAX_MS                 (120U)
/* 超过此时间未收到有效视觉数据就停止电机，单位：ms。 */
#define VISION_LOST_TIMEOUT_MS            (250U)
/* CAN 控制命令发送失败后的重试间隔，单位：ms；防止毫秒级重复发送挤占总线。 */
#define MOTOR_CAN_COMMAND_RETRY_MS        (5U)
/* 单次电机修正的最短和最长持续时间，随后必须停机观察球的响应。 */
#define MOTOR_BURST_MIN_MS                (80U)
#define MOTOR_BURST_MAX_MS                (180U)
/* 每次短动作后的观察时间，避免水管倾角连续累积导致来回过冲。 */
#define MOTOR_BURST_SETTLE_MS             (120U)
/* 中心附近静止微调从小脉冲开始；若仍未移动，逐次增强但不超过上限。 */
#define MOTOR_FINE_TRIM_BURST_MS           (180U)
#define MOTOR_FINE_TRIM_STEP_MS            (60U)
#define MOTOR_FINE_TRIM_MAX_MS             (420U)
#define MOTOR_FINE_TRIM_SETTLE_MS          (250U)
/* 球停在精调区以外时使用较强的回归启动脉冲。 */
#define MOTOR_RECOVERY_BURST_MIN_MS        (140U)
#define MOTOR_RECOVERY_BURST_MAX_MS        (240U)
#define MOTOR_RECOVERY_SETTLE_MS            (80U)
#define MOTOR_RECOVERY_HIGH_SPEED_ZONE_CM  (4.00f)

/* 是否通过 UART0(PA10) 输出 PID 调试数据：1=输出，0=关闭。 */
#define PID_DEBUG_OUTPUT_ENABLED          (1U)
/* PID 调试数据输出周期，单位：ms；115200 波特率下建议不小于 100 ms。 */
#define PID_DEBUG_PERIOD_MS               (100U)

/* 张大头电机的 CAN 地址。 */
#define ZDT_MOTOR_ID                      (1U)
/* 张大头 CAN 协议的固定校验字节。 */
#define ZDT_MOTOR_CHECKSUM                (0x6BU)
/* 连续追球时的最低转速，单位：RPM；小误差使用低速，实现平滑微调。 */
#define ZDT_MOTOR_MIN_SPEED_RPM           (1U)
/* 连续追球时的最高转速，单位：RPM；偏差较大时用高速快速回中。 */
#define ZDT_MOTOR_MAX_SPEED_RPM           (20U)
/* 动态回中和制动采用连续控制的最高转速。 */
#define ZDT_MOTOR_DYNAMIC_MAX_SPEED_RPM   (10U)
/* 将电机速度控制转换为受限的管道倾角控制，单位为 0.001 度。 */
#define PIPE_TILT_DYNAMIC_LIMIT_MDEG      (4000)
#define PIPE_TILT_FINE_MIN_MDEG            (600)
#define PIPE_TILT_FINE_MAX_MDEG           (1500)
#define PIPE_TILT_RECOVERY_MIN_MDEG       (3000)
#define PIPE_TILT_RECOVERY_MAX_MDEG       (8000)
#define PIPE_TILT_ABSOLUTE_LIMIT_MDEG     (9000)
#define PIPE_TILT_POSITION_TOLERANCE_MDEG   (80)
#define PIPE_TILT_FAST_MOVE_ERROR_MDEG     (300)
/* 若实际机构存在减速比导致倾角变化小于理论值，请按比例减小本常数。
 * 减小本值 → 电机需转更久才停 → 物理倾角更大。 */
#define PIPE_TILT_MDEG_PER_RPM_MS             (2)
/* ZDT F6 命令的加速度等级；加大起转更快，也更容易冲过头。 */
#define ZDT_MOTOR_ACCEL_LEVEL             (10U)
/* 是否在上电后执行左右各一次测试，0=关闭，1=开启。 */
#define ZDT_STARTUP_TEST_ENABLED          (0U)
/* 上电测试每个方向的运行时间，单位：ms。 */
#define ZDT_STARTUP_TEST_DURATION_MS      (300U)

/* F6 方向字节：电机向右转，水管上移。 */
#define ZDT_DIRECTION_RIGHT_RAISE         (0x01U)
/* F6 方向字节：电机向左转，水管下移。 */
#define ZDT_DIRECTION_LEFT_LOWER          (0x00U)

/* 球位置坐标与电机方向的对应关系：1=反转控制方向。
 * 当前机构实测方向与 PID 输出一致，因此保持 0；如果以后更换皮带或电机接线后
 * 发现球总是远离中心，再改为 1。 */
#define BALL_CONTROL_REVERSE_DIRECTION    (0U)

#endif
