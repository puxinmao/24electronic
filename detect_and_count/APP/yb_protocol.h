#ifndef _YB_PROTOCOL_H_
#define _YB_PROTOCOL_H_

#include "stdint.h"

#define MAX_BALLS                 (20)
#define PTO_BUF_LEN_MAX           (256)
#define PTO_HEAD                  (0x24)   /* '$' */

/* 单个钢球信息。cx 是相对图像中心的 X 偏差；cy 是原始图像 Y 坐标。 */
typedef struct {
    int cx;
    int cy;
    int w;
    int score;   /* K230 $BALL 帧的 score，范围 0..100 */
} BallInfo;

/* RX 中断写入、主循环读取；new_data 必须在所有字段写完后最后置位。 */
typedef struct {
    volatile uint8_t count;
    volatile uint8_t new_data;
    volatile BallInfo balls[MAX_BALLS];
} BallDetectResult;

void Pto_Data_Receive(uint8_t Rx_Temp);
void Pto_Data_Parse(uint8_t *data_buf, uint8_t num);
void Pto_Clear_CMD_Flag(void);
void Pto_Loop(void);
volatile BallDetectResult *Pto_Get_Ball_Result(void);

#endif
