#ifndef APP_UART_H_
#define APP_UART_H_

#include <stdint.h>

#include "ball_controller.h"

void AppUart_InitK230(void);
void AppUart_SendPidTelemetry(const BallControllerTelemetry *telemetry,
                              uint32_t can_tx_success,
                              uint32_t can_tx_failure,
                              const ZdtCanStatus *can_status);

#endif
