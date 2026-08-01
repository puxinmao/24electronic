#ifndef ZDT_MOTOR_H_
#define ZDT_MOTOR_H_

#include <stdint.h>

typedef enum {
    ZDT_PIPE_LOWER = -1,
    ZDT_PIPE_RAISE = 1
} ZdtPipeDirection;

void ZdtMotor_Init(void);
void ZdtMotor_Enable(void);
void ZdtMotor_SetSpeed(ZdtPipeDirection direction, uint16_t speed_rpm);
void ZdtMotor_Stop(void);
void ZdtMotor_Poll(void);
uint8_t ZdtMotor_IsReady(void);

extern volatile uint32_t g_zdt_can_tx_success_count;
extern volatile uint32_t g_zdt_can_tx_failure_count;
extern volatile uint32_t g_zdt_can_last_extended_id;

#endif
