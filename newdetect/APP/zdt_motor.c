#include "zdt_motor.h"

#include "control_config.h"
#include "ti_msp_dl_config.h"

#define ZDT_CAN_TX_BUFFER       (0U)
#define ZDT_CAN_TIMEOUT_LOOPS   (200000U)
#define ZDT_CAN_READY_LOOPS     (50000U)

static uint8_t g_enabled;
static uint8_t g_moving;
static uint8_t g_can_ready;
volatile uint32_t g_zdt_can_tx_success_count;
volatile uint32_t g_zdt_can_tx_failure_count;
volatile uint32_t g_zdt_can_last_extended_id;

static uint8_t send_can_frame(uint32_t extended_id,
                              const uint8_t *data,
                              uint8_t length)
{
    DL_MCAN_TxBufElement message = {0};
    uint32_t timeout;
    uint32_t buffer_mask = 1UL << ZDT_CAN_TX_BUFFER;
    uint8_t index;

    if ((g_can_ready == 0U) || (data == 0) ||
        (length == 0U) || (length > 8U)) {
        return 0U;
    }

    if ((DL_MCAN_getTxBufReqPend(MCAN0_INST) & buffer_mask) != 0U) {
        /* 上一帧仍在发送时不可取消它，否则会丢失关键的换向/停止命令。 */
        return 0U;
    }

    message.id = extended_id;
    message.rtr = 0U;
    message.xtd = 1U;
    message.esi = 0U;
    message.dlc = length;
    message.brs = 0U;
    message.fdf = 0U;
    message.efc = 0U;
    message.mm = 0U;
    for (index = 0U; index < length; index++) {
        message.data[index] = data[index];
    }

    DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF,
                        ZDT_CAN_TX_BUFFER, &message);
    if (DL_MCAN_TXBufAddReq(MCAN0_INST, ZDT_CAN_TX_BUFFER) != 0) {
        g_zdt_can_tx_failure_count++;
        return 0U;
    }
    g_zdt_can_last_extended_id = extended_id;

    for (timeout = 0U; timeout < ZDT_CAN_TIMEOUT_LOOPS; timeout++) {
        if ((DL_MCAN_getTxBufReqPend(MCAN0_INST) & buffer_mask) == 0U) {
            if ((DL_MCAN_getTxBufTransmissionStatus(MCAN0_INST) &
                 buffer_mask) != 0U) {
                g_zdt_can_tx_success_count++;
                return 1U;
            }
            break;
        }
    }
    (void)DL_MCAN_txBufCancellationReq(MCAN0_INST, ZDT_CAN_TX_BUFFER);
    g_zdt_can_tx_failure_count++;
    return 0U;
}

static uint8_t send_command(const uint8_t *command, uint8_t length)
{
    if ((command == 0) || (length == 0U) || (length > 8U)) {
        return 0U;
    }
    return send_can_frame(((uint32_t)ZDT_MOTOR_ID << 8U), command, length);
}

void ZdtMotor_Init(void)
{
    uint32_t timeout;

    g_enabled = 0U;
    g_moving = 0U;
    g_can_ready = 0U;
    g_zdt_can_tx_success_count = 0U;
    g_zdt_can_tx_failure_count = 0U;
    g_zdt_can_last_extended_id = 0U;
    for (timeout = 0U; timeout < ZDT_CAN_READY_LOOPS; timeout++) {
        if (DL_MCAN_getOpMode(MCAN0_INST) ==
            DL_MCAN_OPERATION_MODE_NORMAL) {
            g_can_ready = 1U;
            break;
        }
    }
}

uint8_t ZdtMotor_Enable(void)
{
    const uint8_t command[5] = {
        0xF3U, 0xABU, 0x01U, 0x00U, ZDT_MOTOR_CHECKSUM
    };

    if (send_command(command, sizeof(command)) != 0U) {
        g_enabled = 1U;
        return 1U;
    }
    return 0U;
}

uint8_t ZdtMotor_SetSpeed(ZdtPipeDirection direction, uint16_t speed_rpm)
{
    uint8_t command[7];

    if (speed_rpm == 0U) {
        return ZdtMotor_Stop();
    }
    if (g_enabled == 0U) {
        (void)ZdtMotor_Enable();
    }
    if (g_enabled == 0U) {
        return 0U;
    }

    command[0] = 0xF6U;
    command[1] = (direction == ZDT_PIPE_RAISE) ?
        ZDT_DIRECTION_RIGHT_RAISE : ZDT_DIRECTION_LEFT_LOWER;
    command[2] = (uint8_t)(speed_rpm >> 8);
    command[3] = (uint8_t)(speed_rpm & 0xFFU);
    command[4] = ZDT_MOTOR_ACCEL_LEVEL;
    command[5] = 0x00U;
    command[6] = ZDT_MOTOR_CHECKSUM;

    if (send_command(command, sizeof(command)) != 0U) {
        g_moving = 1U;
        return 1U;
    }
    return 0U;
}

uint8_t ZdtMotor_IsReady(void)
{
    return (g_can_ready != 0U) ? 1U : 0U;
}

void ZdtMotor_GetCanStatus(ZdtCanStatus *status)
{
    DL_MCAN_ErrCntStatus error_counters;
    DL_MCAN_ProtocolStatus protocol_status;

    if (status == 0) {
        return;
    }

    DL_MCAN_getErrCounters(MCAN0_INST, &error_counters);
    DL_MCAN_getProtocolStatus(MCAN0_INST, &protocol_status);
    status->tx_error_count = (uint8_t)error_counters.transErrLogCnt;
    status->rx_error_count = (uint8_t)error_counters.recErrCnt;
    status->error_passive = (uint8_t)protocol_status.errPassive;
    status->warning = (uint8_t)protocol_status.warningStatus;
    status->bus_off = (uint8_t)protocol_status.busOffStatus;
    status->last_error_code = (uint8_t)protocol_status.lastErrCode;
}

uint8_t ZdtMotor_Stop(void)
{
    const uint8_t command[4] = {
        0xFEU, 0x98U, 0x00U, ZDT_MOTOR_CHECKSUM
    };

    if (g_moving != 0U) {
        if (send_command(command, sizeof(command)) != 0U) {
            g_moving = 0U;
            return 1U;
        }
        return 0U;
    }
    return 1U;
}

void ZdtMotor_Poll(void)
{
    if ((g_can_ready == 0U) &&
        (DL_MCAN_getOpMode(MCAN0_INST) == DL_MCAN_OPERATION_MODE_NORMAL)) {
        g_can_ready = 1U;
        g_enabled = 0U;
    }
}
