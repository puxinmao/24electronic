#include "yb_protocol.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "usart.h"


uint8_t RxBuffer[PTO_BUF_LEN_MAX];
/* 接收数据下标 */
uint8_t RxIndex = 0;
/* 接收状态机 */
uint8_t RxFlag = 0;
/* 新命令接收标志 */
uint8_t New_CMD_flag = 0;
/* 新命令数据长度 */
uint8_t New_CMD_length = 0;

char print_buf[256] = {0};

/* 全局钢球检测结果 */
static BallDetectResult g_ball_result = {0};

/* 获取最新的钢球检测结果 */
BallDetectResult* Pto_Get_Ball_Result(void)
{
    return &g_ball_result;
}

/* 清除命令数据和相关标志 */
void Pto_Clear_CMD_Flag(void)
{
    memset(RxBuffer, 0, PTO_BUF_LEN_MAX);
    memset(print_buf, 0, sizeof(print_buf));
    New_CMD_length = 0;
    New_CMD_flag = 0;
}


/* 接收数据进入协议缓存 */
void Pto_Data_Receive(uint8_t Rx_Temp)
{
    switch (RxFlag)
    {
    case 0:
        if (Rx_Temp == PTO_HEAD)
        {
            RxBuffer[0] = PTO_HEAD;
            RxFlag = 1;
            RxIndex = 1;
        }
        break;

    case 1:
        RxBuffer[RxIndex] = Rx_Temp;
        RxIndex++;
        if (Rx_Temp == PTO_TAIL)
        {
            New_CMD_flag = 1;
            New_CMD_length = RxIndex;
            RxFlag = 0;
            RxIndex = 0;
        }
        else if (RxIndex >= PTO_BUF_LEN_MAX)
        {
            /* 缓冲区溢出，丢弃 */
            New_CMD_flag = 0;
            New_CMD_length = 0;
            RxFlag = 0;
            RxIndex = 0;
            Pto_Clear_CMD_Flag();
        }

        if (New_CMD_flag > 0)
        {
            Pto_Data_Parse((uint8_t*)RxBuffer, New_CMD_length);
            Pto_Clear_CMD_Flag();
        }
        break;

    default:
        break;
    }
}

/* 将字符串数字转成数字。示例："12"->12 */
static int Pto_Char_To_Int(char* data)
{
    return atoi(data);
}

/**
 * @Brief: 数据分析 — 钢球检测帧
 * @Note:  帧格式: $<len>,<id>,<count>[,<cx>,<cy>,<w>,<h>]*#
 *         例如检测到2个球: $36,1,2,100,150,20,20,200,180,22,22#
 *         例如检测到0个球: $7,1,0#
 *         长度字段 = 帧总字节数（含$、长度本身、逗号、#）
 * @Parm:  data_buf — 接收到的完整帧数据
 *         num — 帧数据长度（字节数）
 */
void Pto_Data_Parse(uint8_t *data_buf, uint8_t num)
{
    uint8_t pto_head = data_buf[0];
    uint8_t pto_tail = data_buf[num - 1];

    /* 帧头帧尾校验 */
    if (!(pto_head == PTO_HEAD && pto_tail == PTO_TAIL))
    {
        sprintf(print_buf, "pto error: head=0x%02x, tail=0x%02x\r\n", pto_head, pto_tail);
        uart0_send_string(print_buf);
        return;
    }

    /* ---- 第1步：将逗号替换为 NUL，记录各字段起始位置 ---- */
    /* static 避免大数组在栈上（栈仅 512 字节，256+1024=1280 会溢出） */
    static uint8_t field_index[PTO_BUF_LEN_MAX];
    static int values[PTO_BUF_LEN_MAX];
    uint8_t data_index = 1;  /* field_index[0]=0 已预留 */
    uint8_t i;

    /* field_index[0] 始终为 0，其余在循环中覆盖 */
    field_index[0] = 0;
    for (i = 1; i < num - 1; i++)
    {
        if (data_buf[i] == ',')
        {
            data_buf[i] = 0;                    /* 逗号 -> 字符串终止符 */
            field_index[data_index] = i;         /* 记录逗号位置 */
            data_index++;
        }
    }

    /* ---- 第2步：将各字段转为整数值 ---- */
    for (i = 0; i < data_index; i++)
    {
        /* data_buf + field_index[i] + 1 指向该字段起始字节 */
        values[i] = Pto_Char_To_Int((char*)data_buf + field_index[i] + 1);
    }

    /* values 布局:
     *   values[0] = 帧总长度
     *   values[1] = 功能 ID
     *   values[2] = 检测到的钢球数量 (count)
     *   values[3 + k*4 + 0] = ball[k].cx
     *   values[3 + k*4 + 1] = ball[k].cy
     *   values[3 + k*4 + 2] = ball[k].w
     *   values[3 + k*4 + 3] = ball[k].h
     */

    /* ---- 第3步：校验帧长度 ---- */
    uint8_t pto_len = (uint8_t)values[0];
    if (pto_len != num)
    {
        sprintf(print_buf, "len err: expect=%d, actual=%d\r\n", pto_len, num);
        uart0_send_string(print_buf);
        return;
    }

    /* ---- 第4步：校验功能 ID ---- */
    uint8_t pto_id = (uint8_t)values[1];
    if (pto_id != PTO_FUNC_ID_BALL_DETECT)
    {
        sprintf(print_buf, "id err: got=%d, expect=%d\r\n", pto_id, PTO_FUNC_ID_BALL_DETECT);
        uart0_send_string(print_buf);
        return;
    }

    /* ---- 第5步：解析钢球数据 ---- */
    uint8_t count = (uint8_t)values[2];

    /* 校验数据字段数量是否足够 */
    uint8_t expected_fields = 3 + count * 4;
    if (data_index < expected_fields)
    {
        sprintf(print_buf, "field err: need=%d, got=%d\r\n", expected_fields, data_index);
        uart0_send_string(print_buf);
        return;
    }

    /* 限制最大球数 */
    if (count > MAX_BALLS)
    {
        count = MAX_BALLS;
    }

    /* 存入全局结果 */
    g_ball_result.count = count;
    for (i = 0; i < count; i++)
    {
        g_ball_result.balls[i].cx = values[3 + i * 4 + 0];
        g_ball_result.balls[i].cy = values[3 + i * 4 + 1];
        g_ball_result.balls[i].w  = values[3 + i * 4 + 2];
        g_ball_result.balls[i].h  = values[3 + i * 4 + 3];
    }
    g_ball_result.new_data = 1;

    /* ---- 第6步：通过调试串口打印结果 ---- */
    sprintf(print_buf, "Balls: %d\r\n", count);
    uart0_send_string(print_buf);

    for (i = 0; i < count; i++)
    {
        sprintf(print_buf, "  [%d] cx=%d, cy=%d, w=%d, h=%d\r\n",
                i + 1,
                g_ball_result.balls[i].cx,
                g_ball_result.balls[i].cy,
                g_ball_result.balls[i].w,
                g_ball_result.balls[i].h);
        uart0_send_string(print_buf);
    }
}

/* 主循环调用：处理收到的数据 */
void Pto_Loop(void)
{
    /* 当前在 ISR 中已完成解析，这里可用于后续扩展（如 LED 指示等） */
}
