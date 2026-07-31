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
#define BALL_DEAD_ZONE_ENTER_CM           (0.25f)
/* 已停止后，球超出此范围才重新追球；必须大于 ENTER，单位：cm。 */
#define BALL_DEAD_ZONE_EXIT_CM            (0.40f)
/* 判定球在中心稳定的最大速度，单位：cm/s。 */
#define BALL_HOLD_VELOCITY_CM_S           (0.50f)
/* 超限预测补偿时间，单位：s；球快速靠近中心时提前反向制动。 */
#define BALL_OVERSHOOT_COMPENSATION_S     (0.12f)
/* 预测补偿位移绝对上限，单位：cm；防止视觉速度噪声导致误动作。 */
#define BALL_OVERSHOOT_COMPENSATION_MAX_CM (1.50f)
/* 预测位置进入该中心区域时开始主动反向制动，单位：cm。 */
#define BALL_OVERSHOOT_BRAKE_ZONE_CM      (0.80f)
/* 开始制动所需的最小球速度，单位：cm/s；低于此值不触发反向制动。 */
#define BALL_OVERSHOOT_BRAKE_MIN_CM_S     (0.30f)
/* 主动制动时要求球具有的反向速度，单位：cm/s；越大制动越强。 */
#define BALL_OVERSHOOT_BRAKE_VELOCITY_CM_S (0.80f)
/* 主动制动阶段的电机最高转速，单位：RPM；限制反向摆动幅度。 */
#define BALL_OVERSHOOT_BRAKE_MAX_RPM      (2U)

/* 位置内环比例系数；增大追回更快，也更容易冲过中心。 */
#define POSITION_PID_KP                   (2.00f)
/* 位置内环积分系数；消除长期静差，过大容易缓慢摆动。 */
#define POSITION_PID_KI                   (0.08f)
/* 位置内环微分系数；根据球速度提前刹车，减小冲过头。 */
#define POSITION_PID_KD                   (0.75f)
/* 位置内环积分累积的绝对上限，防止积分饱和。 */
#define POSITION_PID_INTEGRAL_LIMIT       (12.0f)
/* 位置内环输出的最大期望球速度，单位：cm/s。 */
#define BALL_TARGET_VELOCITY_LIMIT_CM_S   (12.0f)

/* 速度外环比例系数；将速度误差转换为电机输出强度。 */
#define VELOCITY_PID_KP                   (8.0f)
/* 速度外环积分系数；修正持续存在的速度误差。 */
#define VELOCITY_PID_KI                   (0.0f)
/* 速度外环微分系数；抑制速度误差突变导致的冲击。 */
#define VELOCITY_PID_KD                   (10.0f)
/* 速度外环积分累积的绝对上限，防止积分饱和。 */
#define VELOCITY_PID_INTEGRAL_LIMIT       (20.0f)
/* PID 输出的最大等效脉冲率，用于归一化计算转速，单位：pulse/s。 */
#define MOTOR_PULSE_RATE_LIMIT_PPS        (2000.0f)
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

/* 张大头电机的 CAN 地址。 */
#define ZDT_MOTOR_ID                      (1U)
/* 张大头 CAN 协议的固定校验字节。 */
#define ZDT_MOTOR_CHECKSUM                (0x6BU)
/* 连续追球时的最低转速，单位：RPM；过大时中心附近会冲得太快。 */
#define ZDT_MOTOR_MIN_SPEED_RPM           (2U)
/* 连续追球时的最高转速，单位：RPM；反应过快时优先降低此值。 */
#define ZDT_MOTOR_MAX_SPEED_RPM           (3U)
/* ZDT F6 命令的加速度等级；加大起转更快，也更容易冲过头。 */
#define ZDT_MOTOR_ACCEL_LEVEL             (5U)
/* 是否在上电后执行左右各一次测试，0=关闭，1=开启。 */
#define ZDT_STARTUP_TEST_ENABLED          (0U)
/* 上电测试每个方向的运行时间，单位：ms。 */
#define ZDT_STARTUP_TEST_DURATION_MS      (300U)

/* F6 方向字节：电机向右转，水管上移。 */
#define ZDT_DIRECTION_RIGHT_RAISE         (0x01U)
/* F6 方向字节：电机向左转，水管下移。 */
#define ZDT_DIRECTION_LEFT_LOWER          (0x00U)

#endif
