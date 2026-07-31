#ifndef ZDT_MOTOR_H_
#define ZDT_MOTOR_H_

#include <stdint.h>

typedef enum {
    ZDT_PIPE_LOWER = -1,
    ZDT_PIPE_RAISE = 1
} ZdtPipeDirection;

typedef struct {
    uint8_t tx_error_count;
    uint8_t rx_error_count;
    uint8_t error_passive;
    uint8_t warning;
    uint8_t bus_off;
    uint8_t last_error_code;
} ZdtCanStatus;

void ZdtMotor_Init(void);
uint8_t ZdtMotor_Enable(void);
uint8_t ZdtMotor_SetSpeed(ZdtPipeDirection direction, uint16_t speed_rpm);
uint8_t ZdtMotor_Stop(void);
void ZdtMotor_Poll(void);
uint8_t ZdtMotor_IsReady(void);
void ZdtMotor_GetCanStatus(ZdtCanStatus *status);

extern volatile uint32_t g_zdt_can_tx_success_count;
extern volatile uint32_t g_zdt_can_tx_failure_count;
extern volatile uint32_t g_zdt_can_last_extended_id;

#endif
