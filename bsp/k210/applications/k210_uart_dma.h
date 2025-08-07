/*
 * K210 UART DMA + 环形缓冲区头文件
 * 借鉴 mm32car.c 设计思路，适配 K210 硬件特性
 */

#ifndef __K210_UART_DMA_H__
#define __K210_UART_DMA_H__

#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 配置宏定义 ==================== */
#define K210_UART_DMA_VERSION   "1.0.0"

/* ==================== 数据结构 ==================== */

// UART统计信息结构体
typedef struct {
    uint32_t total_received;      // 总接收字节数
    uint32_t dma_transfers;       // DMA传输次数
    uint32_t timeout_events;      // 超时事件次数
    uint32_t buffer_overflows;    // 缓冲区溢出次数
} k210_uart_stats_t;

/* ==================== 函数声明 ==================== */

/**
 * @brief 初始化K210 UART DMA系统
 * @return RT_EOK: 成功, 其他: 失败
 */
rt_err_t k210_uart_dma_init(void);

/**
 * @brief 获取UART统计信息
 * @param stats 统计信息结构体指针
 */
void k210_uart_get_stats(k210_uart_stats_t *stats);

/**
 * @brief 发送数据
 * @param data 数据指针
 * @param len 数据长度
 * @return RT_EOK: 成功, 其他: 失败
 */
rt_err_t k210_uart_send_data(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __K210_UART_DMA_H__ */
