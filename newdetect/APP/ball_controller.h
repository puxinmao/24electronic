#ifndef BALL_CONTROLLER_H_
#define BALL_CONTROLLER_H_

#include <stdint.h>

#include "ball_protocol.h"
#include "zdt_motor.h"

typedef enum {
    BALL_CONTROL_STOP = 0,
    BALL_CONTROL_MOVE
} BallControlAction;

typedef struct {
    BallControlAction action;
    ZdtPipeDirection direction;
    uint16_t speed_rpm;
    uint16_t burst_duration_ms;
    uint8_t fine_trim;
    uint8_t recovery;
    uint8_t braking;
    int16_t target_tilt_milli_degree;
    int16_t position_centi_cm;
} BallControlCommand;

/* PID 调试数据：所有带 _centi 的数值均放大 100 倍，便于串口查看。 */
typedef struct {
    int16_t position_centi_cm;
    int16_t predicted_position_centi_cm;
    int16_t filtered_velocity_centi_cm_s;
    int16_t position_error_centi_cm;
    int16_t desired_velocity_centi_cm_s;
    int16_t velocity_error_centi_cm_s;
    int16_t pulse_rate;
    int8_t direction;
    uint16_t speed_rpm;
    uint16_t burst_duration_ms;
    uint8_t control_mode;
    uint8_t holding_center;
    uint8_t vision_active;
} BallControllerTelemetry;

void BallController_Init(void);
void BallController_Reset(void);
BallControlCommand BallController_Update(const BallSample *sample,
                                         uint32_t now_ms);
int16_t BallController_MapPixels(int16_t cx_pixels);
void BallController_GetTelemetry(BallControllerTelemetry *telemetry);

#endif
