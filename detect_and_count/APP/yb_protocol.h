#ifndef _YB_PROTOCOL_H_
#define _YB_PROTOCOL_H_
#include "stdint.h"

/* 最大支持的钢球数量 */
#define MAX_BALLS                 (20)

/* 协议缓冲区最大长度（帧头帧尾 + 帧长/ID/数量 + 每球4字段 + 逗号） */
/* 最坏情况：15球×4字段×3位数 + 逗号 ≈ 256 bytes，留足余量 */
#define PTO_BUF_LEN_MAX           (256)

/* 帧界定符 */
#define PTO_HEAD                  (0x24)   /* '$' */
#define PTO_TAIL                  (0x23)   /* '#' */

/* 功能 ID */
#define PTO_FUNC_ID_BALL_DETECT   (1)      /* 钢球检测 */


/* 单个钢球信息 */
typedef struct {
    int cx;    /* 中心 X（模型坐标 320×320） */
    int cy;    /* 中心 Y */
    int w;     /* 宽度 */
    int h;     /* 高度 */
} BallInfo;

/* 钢球检测结果 */
typedef struct {
    uint8_t count;                       /* 检测到的钢球数量 */
    uint8_t new_data;                    /* 新数据标志：1=有新数据待读取 */
    BallInfo balls[MAX_BALLS];           /* 钢球列表 */
} BallDetectResult;


void Pto_Data_Receive(uint8_t Rx_Temp);
void Pto_Data_Parse(uint8_t *data_buf, uint8_t num);
void Pto_Clear_CMD_Flag(void);
void Pto_Loop(void);

/* 获取最新的钢球检测结果 */
BallDetectResult* Pto_Get_Ball_Result(void);

#endif
