/*
 * K210 UART DMA + 环形缓冲区实现
 * 借鉴 mm32car.c 设计思路，适配 K210 硬件特性
 * 
 * 技术特点：
 * 1. DMA + 定时器替代IDLE中断，实现不定长数据接收
 * 2. 双缓冲 + 环形缓冲区设计，提高数据吞吐和可靠性  
 * 3. 线程安全的数据访问，支持多任务环境
 * 4. 完整的错误处理和统计功能
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "dmalock.h"
#include "../packages/K210-SDK-latest/lib/drivers/include/uart.h"
#include "../packages/K210-SDK-latest/lib/drivers/include/dmac.h"
#include "../packages/K210-SDK-latest/lib/drivers/include/sysctl.h"

/* ==================== 配置参数 ==================== */
#define K210_UART_CHANNEL       UART_DEVICE_1          // 使用UART1
#define K210_UART_BAUDRATE      115200                 // 波特率
#define UART_DMA_BUF_SIZE       128                    // DMA缓冲区大小
#define RING_BUFFER_SIZE        512                    // 环形缓冲区大小  
#define UART_TIMEOUT_MS         10                     // 接收超时时间(ms)
#define UART_BATCH_SIZE         16                     // 批处理大小

/* ==================== 数据结构定义 ==================== */

// 环形缓冲区结构体
typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t overflow_count;   // 溢出计数
    rt_mutex_t mutex;                   // 互斥锁保护
} k210_ring_buffer_t;

// DMA接收状态
typedef struct {
    uint8_t dma_buf[2][UART_DMA_BUF_SIZE];   // 双缓冲区
    volatile uint8_t current_buf;             // 当前DMA写入缓冲区
    volatile uint16_t last_pos;               // 上次处理位置
    dmac_channel_number_t dma_ch;             // DMA通道号
    rt_timer_t timeout_timer;                 // 超时定时器
    rt_event_t data_event;                    // 数据事件
} k210_uart_dma_t;

// 统计信息
typedef struct {
    uint32_t total_received;      // 总接收字节数
    uint32_t dma_transfers;       // DMA传输次数
    uint32_t timeout_events;      // 超时事件次数
    uint32_t buffer_overflows;    // 缓冲区溢出次数
} k210_uart_stats_t;

/* ==================== 全局变量 ==================== */
static k210_ring_buffer_t g_ring_buffer = {0};
static k210_uart_dma_t g_uart_dma = {0};
static k210_uart_stats_t g_uart_stats = {0};

// 事件标志定义
#define EVENT_UART_DATA_READY   (1<<0)
#define EVENT_UART_TIMEOUT      (1<<1)

/* ==================== 环形缓冲区操作函数 ==================== */

// 初始化环形缓冲区
static rt_err_t ring_buffer_init(void)
{
    g_ring_buffer.head = 0;
    g_ring_buffer.tail = 0;
    g_ring_buffer.overflow_count = 0;
    
    g_ring_buffer.mutex = rt_mutex_create("uart_rb_mutex", RT_IPC_FLAG_PRIO);
    if (!g_ring_buffer.mutex) {
        rt_kprintf("Failed to create ring buffer mutex\n");
        return -RT_ERROR;
    }
    
    return RT_EOK;
}

// 获取缓冲区可用空间
static inline uint16_t ring_buffer_space(void)
{
    return (g_ring_buffer.tail - g_ring_buffer.head - 1) & (RING_BUFFER_SIZE - 1);
}

// 获取缓冲区数据量
static inline uint16_t ring_buffer_count(void)
{
    return (g_ring_buffer.head - g_ring_buffer.tail) & (RING_BUFFER_SIZE - 1);
}

// 写入单个字节到环形缓冲区
static uint8_t ring_buffer_put(uint8_t data)
{
    uint16_t next_head = (g_ring_buffer.head + 1) & (RING_BUFFER_SIZE - 1);
    
    if (next_head == g_ring_buffer.tail) {
        g_ring_buffer.overflow_count++;
        g_uart_stats.buffer_overflows++;
        return 0;  // 缓冲区满
    }
    
    g_ring_buffer.buffer[g_ring_buffer.head] = data;
    g_ring_buffer.head = next_head;
    return 1;
}

// 批量写入数据到环形缓冲区
static uint16_t ring_buffer_put_batch(const uint8_t *data, uint16_t len)
{
    rt_mutex_take(g_ring_buffer.mutex, RT_WAITING_FOREVER);
    
    uint16_t written = 0;
    uint16_t space = ring_buffer_space();
    
    if (len > space) {
        len = space;  // 限制写入长度
        g_ring_buffer.overflow_count++;
    }
    
    for (uint16_t i = 0; i < len; i++) {
        if (ring_buffer_put(data[i])) {
            written++;
        } else {
            break;
        }
    }
    
    rt_mutex_release(g_ring_buffer.mutex);
    return written;
}

// 从环形缓冲区读取单个字节
static uint8_t ring_buffer_get(uint8_t *data)
{
    if (g_ring_buffer.head == g_ring_buffer.tail) {
        return 0;  // 缓冲区空
    }
    
    *data = g_ring_buffer.buffer[g_ring_buffer.tail];
    g_ring_buffer.tail = (g_ring_buffer.tail + 1) & (RING_BUFFER_SIZE - 1);
    return 1;
}

// 批量从环形缓冲区读取数据
static uint16_t ring_buffer_get_batch(uint8_t *data, uint16_t max_len)
{
    rt_mutex_take(g_ring_buffer.mutex, RT_WAITING_FOREVER);
    
    uint16_t read = 0;
    uint16_t available = ring_buffer_count();
    uint16_t to_read = (max_len < available) ? max_len : available;
    
    for (uint16_t i = 0; i < to_read; i++) {
        if (ring_buffer_get(&data[i])) {
            read++;
        } else {
            break;
        }
    }
    
    rt_mutex_release(g_ring_buffer.mutex);
    return read;
}

// 获取缓冲区统计信息
static void ring_buffer_get_stats(uint16_t *used, uint16_t *free, uint16_t *overflows)
{
    rt_mutex_take(g_ring_buffer.mutex, RT_WAITING_FOREVER);
    *used = ring_buffer_count();
    *free = ring_buffer_space();
    *overflows = g_ring_buffer.overflow_count;
    rt_mutex_release(g_ring_buffer.mutex);
}

/* ==================== DMA处理函数 ==================== */

// DMA完成回调函数
static int dma_complete_callback(void *ctx)
{
    // DMA传输完成，切换缓冲区
    g_uart_dma.current_buf = 1 - g_uart_dma.current_buf;
    g_uart_stats.dma_transfers++;
    
    // 重新启动DMA接收到新缓冲区
    dmac_set_single_mode(
        g_uart_dma.dma_ch,
        (void *)(&uart[K210_UART_CHANNEL]->RBR),
        g_uart_dma.dma_buf[g_uart_dma.current_buf],
        DMAC_ADDR_NOCHANGE,
        DMAC_ADDR_INCREMENT,
        DMAC_MSIZE_1,
        DMAC_TRANS_WIDTH_32,
        UART_DMA_BUF_SIZE
    );
    
    // 发送数据就绪事件
    rt_event_send(g_uart_dma.data_event, EVENT_UART_DATA_READY);
    
    return 0;
}

// 超时定时器回调函数
static void timeout_timer_callback(void *parameter)
{
    g_uart_stats.timeout_events++;
    // 发送超时事件，处理不定长数据
    rt_event_send(g_uart_dma.data_event, EVENT_UART_TIMEOUT);
}

// 处理DMA接收的数据
static void process_dma_data(void)
{
    uint8_t inactive_buf = 1 - g_uart_dma.current_buf;
    
    // 计算接收到的数据长度
    uint16_t current_pos = UART_DMA_BUF_SIZE - dmac_data_number_get(DMA1, g_uart_dma.dma_ch);
    uint16_t data_len = 0;
    
    if (current_pos > g_uart_dma.last_pos) {
        data_len = current_pos - g_uart_dma.last_pos;
    } else if (current_pos < g_uart_dma.last_pos) {
        // DMA缓冲区循环
        data_len = UART_DMA_BUF_SIZE - g_uart_dma.last_pos + current_pos;
    }
    
    if (data_len > 0) {
        // 将数据写入环形缓冲区
        uint16_t written = ring_buffer_put_batch(
            &g_uart_dma.dma_buf[inactive_buf][g_uart_dma.last_pos],
            data_len
        );
        
        g_uart_stats.total_received += written;
        g_uart_dma.last_pos = current_pos;
        
        // 重启超时定时器
        rt_timer_stop(g_uart_dma.timeout_timer);
        rt_timer_start(g_uart_dma.timeout_timer);
    }
}

/* ==================== 数据处理线程 ==================== */

static void uart_data_thread(void *parameter)
{
    rt_uint32_t event;
    
    while (1) {
        // 等待数据事件
        if (rt_event_recv(g_uart_dma.data_event, 
                         EVENT_UART_DATA_READY | EVENT_UART_TIMEOUT,
                         RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                         RT_WAITING_FOREVER, &event) == RT_EOK) {
            
            if (event & EVENT_UART_DATA_READY) {
                // DMA完成事件
                process_dma_data();
            }
            
            if (event & EVENT_UART_TIMEOUT) {
                // 超时事件，处理不定长数据
                process_dma_data();
            }
        }
        
        // 简单的数据处理示例
        uint8_t temp_buf[UART_BATCH_SIZE];
        uint16_t read_len = ring_buffer_get_batch(temp_buf, UART_BATCH_SIZE);
        
        if (read_len > 0) {
            // 在这里添加具体的数据处理逻辑
            // 例如：协议解析、命令处理等
            
            // 示例：回显接收到的数据
            for (uint16_t i = 0; i < read_len; i++) {
                if (temp_buf[i] == 'f') {
                    rt_kprintf("收到发车指令!\n");
                }
            }
        }
    }
}

/* ==================== 初始化和控制函数 ==================== */

// 初始化K210 UART DMA系统
rt_err_t k210_uart_dma_init(void)
{
    rt_err_t result;
    
    rt_kprintf("初始化K210 UART DMA系统...\n");
    
    // 1. 初始化环形缓冲区
    result = ring_buffer_init();
    if (result != RT_EOK) {
        rt_kprintf("环形缓冲区初始化失败\n");
        return result;
    }
    
    // 2. 创建事件对象
    g_uart_dma.data_event = rt_event_create("uart_event", RT_IPC_FLAG_PRIO);
    if (!g_uart_dma.data_event) {
        rt_kprintf("创建UART事件失败\n");
        return -RT_ERROR;
    }
    
    // 3. 创建超时定时器
    g_uart_dma.timeout_timer = rt_timer_create("uart_timer",
                                              timeout_timer_callback,
                                              RT_NULL,
                                              rt_tick_from_millisecond(UART_TIMEOUT_MS),
                                              RT_TIMER_FLAG_ONE_SHOT);
    if (!g_uart_dma.timeout_timer) {
        rt_kprintf("创建超时定时器失败\n");
        return -RT_ERROR;
    }
    
    // 4. 分配DMA通道
    result = dmalock_take(&g_uart_dma.dma_ch, RT_WAITING_FOREVER, "uart_rx");
    if (result != RT_EOK) {
        rt_kprintf("分配DMA通道失败\n");
        return result;
    }
    
    // 5. 配置UART
    uart_configure(K210_UART_CHANNEL, K210_UART_BAUDRATE, 
                  UART_BITWIDTH_8BIT, UART_STOP_1, UART_PARITY_NONE);
    
    // 6. 配置DMA
    sysctl_dma_select((sysctl_dma_channel_t)g_uart_dma.dma_ch, 
                     SYSCTL_DMA_SELECT_UART1_RX_REQ + K210_UART_CHANNEL * 2);
    
    dmac_irq_register(g_uart_dma.dma_ch, dma_complete_callback, NULL, 1);
    
    // 7. 启动DMA接收
    g_uart_dma.current_buf = 0;
    g_uart_dma.last_pos = 0;
    
    dmac_set_single_mode(
        g_uart_dma.dma_ch,
        (void *)(&uart[K210_UART_CHANNEL]->RBR),
        g_uart_dma.dma_buf[g_uart_dma.current_buf],
        DMAC_ADDR_NOCHANGE,
        DMAC_ADDR_INCREMENT,
        DMAC_MSIZE_1,
        DMAC_TRANS_WIDTH_32,
        UART_DMA_BUF_SIZE
    );
    
    // 8. 创建数据处理线程
    rt_thread_t thread = rt_thread_create("uart_data",
                                         uart_data_thread,
                                         RT_NULL,
                                         2048,
                                         15,
                                         10);
    if (thread) {
        rt_thread_startup(thread);
    } else {
        rt_kprintf("创建UART数据处理线程失败\n");
        return -RT_ERROR;
    }
    
    rt_kprintf("K210 UART DMA系统初始化完成\n");
    rt_kprintf("DMA通道: %d, 环形缓冲区: %d bytes\n", 
              g_uart_dma.dma_ch, RING_BUFFER_SIZE);
    
    return RT_EOK;
}

// 获取UART统计信息
void k210_uart_get_stats(k210_uart_stats_t *stats)
{
    if (stats) {
        *stats = g_uart_stats;
    }
}

// 发送数据
rt_err_t k210_uart_send_data(const uint8_t *data, uint16_t len)
{
    // 使用阻塞方式发送数据
    for (uint16_t i = 0; i < len; i++) {
        uart_channel_putc(data[i], K210_UART_CHANNEL);
    }
    return RT_EOK;
}

/* ==================== 调试命令 ==================== */

// 显示UART状态信息
static void uart_status(int argc, char **argv)
{
    k210_uart_stats_t stats;
    uint16_t used, free, overflows;
    
    k210_uart_get_stats(&stats);
    ring_buffer_get_stats(&used, &free, &overflows);
    
    rt_kprintf("=== K210 UART DMA状态 ===\n");
    rt_kprintf("总接收: %d bytes\n", stats.total_received);
    rt_kprintf("DMA传输: %d 次\n", stats.dma_transfers);
    rt_kprintf("超时事件: %d 次\n", stats.timeout_events);
    rt_kprintf("缓冲区溢出: %d 次\n", stats.buffer_overflows);
    rt_kprintf("环形缓冲区使用: %d/%d bytes\n", used, RING_BUFFER_SIZE);
    rt_kprintf("当前DMA通道: %d\n", g_uart_dma.dma_ch);
}
MSH_CMD_EXPORT(uart_status, show uart dma status);

// 测试发送数据
static void uart_test_send(int argc, char **argv)
{
    const char *test_data = "Hello K210 UART DMA!\r\n";
    k210_uart_send_data((const uint8_t *)test_data, strlen(test_data));
    rt_kprintf("发送测试数据完成\n");
}
MSH_CMD_EXPORT(uart_test_send, send test data via uart);
