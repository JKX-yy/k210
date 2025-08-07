/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */
/*

UART（通用异步收发传输器，Universal Asynchronous Receiver/Transmitter）和**高速UART（High-Speed UART）**的主要区别体现在以下几个方面：

✅ 1. 波特率（Baud Rate）
普通UART：

常见波特率为：9600、19200、38400、57600、115200 bps。

一般最高支持到 115200 或 1Mbps，超过后误码率会上升明显。

高速UART：

支持更高的波特率，如 3Mbps、4Mbps、甚至20Mbps以上。

一些芯片（如Qualcomm、STM32F7/H7）支持 4.5Mbps~12Mbps甚至更高。

一般在硬件和驱动层做了优化，能保持较低误码率。

✅ 2. 硬件支持与FIFO缓冲
普通UART：

FIFO一般是 16字节或32字节（硬件缓冲区小），容易丢包。

高速UART：

更大的FIFO缓存（例如128字节或更多），可提高稳定性。

支持 DMA传输（更适合高速、大量数据）。

有的高速UART带有 分帧、错误检测等功能。

✅ 3. 系统时钟与精度要求
高速UART需要更高的系统时钟频率来确保足够精度，比如：

16倍、8倍采样（oversampling）。

时钟精度要求 < 2%，否则容易丢数据或帧错误。

普通UART对时钟要求较低。

✅ 4. 应用场景
场景	UART	高速UART
调试串口、控制信号	✅ 常用	✅ 可用
摄像头串口输出、蓝牙高速通信（如HC-05、ESP32）	❌ 波特率不够	✅ 推荐使用
文件传输、大数据帧	❌ 容易丢包	✅ 支持DMA和高吞吐

✅ 5. 芯片差异
一些MCU（如STM32F1、AVR）UART速率受限；

一些高端芯片（如STM32H7、i.MX RT、ESP32）提供高速UART接口；

Android平台中的HS-UART（High-Speed UART）用于与Wi-Fi/BT模块通信，常跑在3Mbps以上。

🧠 小结
项目	普通UART	高速UART
波特率	≤115200~1Mbps	≥3Mbps，最高几十Mbps
FIFO	小（16~64字节）	大（128字节以上）
DMA支持	视芯片而定	通常支持
串口数量	多数MCU多个	少数支持
适用场景	低速通信、控制	高频数据传输
*/

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "board.h"
#include "drv_uart.h"

#include <stdio.h>
#include <sysctl.h>

#include "../packages/K210-SDK-latest/lib/drivers/include/uart.h" ///
#include "../packages/K210-SDK-latest/lib/drivers/include/uarths.h"
#include "plic.h"

#define UART_DEFAULT_BAUDRATE               115200
#define  BSP_USING_UART1
#define  RT_SERIAL_USING_DMA
static volatile uarths_t *const _uarths = (volatile uarths_t *)UARTHS_BASE_ADDR;

struct device_uart //保存UART的硬件基地址和中断号
{
    rt_uint32_t hw_base;
    rt_uint32_t irqno;
};

static rt_err_t  rt_uarths_configure(struct rt_serial_device *serial, struct serial_configure *cfg);
static rt_err_t  uarths_control(struct rt_serial_device *serial, int cmd, void *arg);
static int       drv_uarths_putc(struct rt_serial_device *serial, char c);
static int       drv_uarths_getc(struct rt_serial_device *serial);

static void     uarths_irq_handler(int irqno, void *param);

static rt_err_t  rt_uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg);
static rt_err_t  uart_control(struct rt_serial_device *serial, int cmd, void *arg);
static int       drv_uart_putc(struct rt_serial_device *serial, char c);
static int       drv_uart_getc(struct rt_serial_device *serial);

static void     uart_irq_handler(int irqno, void *param);

const struct rt_uart_ops _uart_hs_ops =
{
    rt_uarths_configure,
    uarths_control,
    drv_uarths_putc,
    drv_uarths_getc,
    RT_NULL
};

const struct rt_uart_ops _uart_ops =
{
    rt_uart_configure,//串口配置函数
    uart_control, //串口控制函数
    drv_uart_putc,//发送字符
    drv_uart_getc,//接收字符
    //TODO: add DMA support 预留DMA支持
    RT_NULL//暂无DMA相关操作
};

/* START ported from kendryte standalone sdk uart.c */
#define __UART_BRATE_CONST  16  //// 波特率相关常量波特率相关的常量，用于分频计算。

//数组   保存3个UART寄存器基地质
volatile uart_t* const  _uart[3] =
{
    (volatile uart_t*)UART1_BASE_ADDR,//UART1
    (volatile uart_t*)UART2_BASE_ADDR,
    (volatile uart_t*)UART3_BASE_ADDR
};
//// UART初始化函数，参数为通道号  初始化指定UART通道，主要是使能时钟和复位硬件。
void _uart_init(uart_device_number_t channel)
{
    sysctl_clock_enable(SYSCTL_CLOCK_UART1 + channel);//使能UART时钟 123
    sysctl_reset(SYSCTL_RESET_UART1 + channel);//复位UART
}

/* END ported from kendryte standalone sdk uart.c  根据基地址获取UART通道号*/
static inline uart_device_number_t _get_uart_channel(rt_uint32_t addr)
{
    switch (addr)
    {
        case UART1_BASE_ADDR:
            return UART_DEVICE_1; //0
        case UART2_BASE_ADDR:
            return UART_DEVICE_2;//1
        case UART3_BASE_ADDR:
            return UART_DEVICE_3;//2
        default:
            return UART_DEVICE_MAX;
    }
}

/*
 * UART Initiation
 */
int rt_hw_uart_init(void)
{
    struct rt_serial_device *serial;//串行设备
    struct device_uart      *uart;//基地址和中断号
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;//默认配置

#ifdef BSP_USING_UART_HS
    {
        static struct rt_serial_device  serial_hs;
        static struct device_uart       uart_hs;

        serial  = &serial_hs;
        uart    = &uart_hs;

        serial->ops              = &_uart_hs_ops;
        serial->config           = config;
        serial->config.baud_rate = 115200;

        uart->hw_base   = UARTHS_BASE_ADDR;
        uart->irqno     = IRQN_UARTHS_INTERRUPT;

        rt_hw_serial_register(serial,
                              "uarths",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              uart);
    }
#endif

#ifdef BSP_USING_UART1
    {
        static struct rt_serial_device  serial1;
        static struct device_uart       uart1;

        serial  = &serial1;
        uart    = &uart1;

        serial->ops              = &_uart_ops;
        serial->config           = config;
        serial->config.baud_rate = UART_DEFAULT_BAUDRATE;

        uart->hw_base   = UART1_BASE_ADDR;
        uart->irqno     = IRQN_UART1_INTERRUPT;
        //UART初始化函数，参数为通道号  初始化指定UART通道，主要是使能时钟和复位硬件。
        _uart_init(UART_DEVICE_1);
        //
        rt_hw_serial_register(serial,
                              "uart1",//下面没有启动DMA中断  只是启动普通中断
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              uart);
    }
#endif

#ifdef BSP_USING_UART2
    {
        static struct rt_serial_device  serial2;
        static struct device_uart       uart2;

        serial  = &serial2;
        uart    = &uart2;

        serial->ops              = &_uart_ops;
        serial->config           = config;
        serial->config.baud_rate = UART_DEFAULT_BAUDRATE;

        uart->hw_base   = UART2_BASE_ADDR;
        uart->irqno     = IRQN_UART2_INTERRUPT;

        _uart_init(UART_DEVICE_2);

        rt_hw_serial_register(serial,
                              "uart2",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              uart);
    }
#endif

#ifdef BSP_USING_UART3
    {
        static struct rt_serial_device  serial3;
        static struct device_uart       uart3;

        serial  = &serial3;
        uart    = &uart3;

        serial->ops              = &_uart_ops;
        serial->config           = config;
        serial->config.baud_rate = UART_DEFAULT_BAUDRATE;

        uart->hw_base   = UART3_BASE_ADDR;
        uart->irqno     = IRQN_UART3_INTERRUPT;

        _uart_init(UART_DEVICE_3);

        rt_hw_serial_register(serial,
                              "uart3",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              uart);
    }
#endif

    return 0;
}

/*
 * UARTHS interface
 */
static rt_err_t rt_uarths_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct device_uart *uart;
    uint32_t freq_hs = sysctl_clock_get_freq(SYSCTL_CLOCK_CPU);
    uint16_t div_hs = freq_hs / cfg->baud_rate - 1;

    RT_ASSERT(serial != RT_NULL);
    serial->config = *cfg;

    uart = serial->parent.user_data;
    RT_ASSERT(uart != RT_NULL);

    if (uart->hw_base == UARTHS_BASE_ADDR)
    {
        _uarths->div.div = div_hs;
        _uarths->txctrl.txen = 1;
        _uarths->rxctrl.rxen = 1;
        _uarths->txctrl.txcnt = 0;
        _uarths->rxctrl.rxcnt = 0;
        _uarths->ip.txwm = 1;
        _uarths->ip.rxwm = 1;
        _uarths->ie.txwm = 0;
        _uarths->ie.rxwm = 1;
    }
    else
    {
        return (-1);
        /* other uart */
    }

    return (RT_EOK);
}

static rt_err_t uarths_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct device_uart *uart;

    uart = serial->parent.user_data;

    RT_ASSERT(uart != RT_NULL);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* Disable the UART Interrupt */
        rt_hw_interrupt_mask(uart->irqno);
        break;

    case RT_DEVICE_CTRL_SET_INT:
        /* install interrupt */
        rt_hw_interrupt_install(uart->irqno, uarths_irq_handler,
                                serial, serial->parent.parent.name);
        rt_hw_interrupt_umask(uart->irqno);
        break;
    }

    return (RT_EOK);
}


static int drv_uarths_putc(struct rt_serial_device *serial, char c)
{
    struct device_uart *uart = serial->parent.user_data;
    RT_ASSERT(uart->hw_base == UARTHS_BASE_ADDR);

    while (_uarths->txdata.full);
    _uarths->txdata.data = (uint8_t)c;

    return (1);
}

static int drv_uarths_getc(struct rt_serial_device *serial)
{
    struct device_uart *uart = serial->parent.user_data;
    RT_ASSERT(uart->hw_base == UARTHS_BASE_ADDR);

    uarths_rxdata_t recv = _uarths->rxdata;
    if (recv.empty)
        return EOF;
    else
        return (recv.data & 0xff);
    /* Receive Data Available */

    return (-1);
}

/* UARTHS ISR */
static void uarths_irq_handler(int irqno, void *param)
{
    struct rt_serial_device *serial = (struct rt_serial_device *)param;
    struct device_uart *uart = serial->parent.user_data;
    RT_ASSERT(uart->hw_base == UARTHS_BASE_ADDR);

    /* read interrupt status and clear it */
    if (_uarths->ip.rxwm)
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
}

/*
 * UART interface  普通UART接口
 该函数用于配置普通UART的硬件参数，包括波特率、数据位、停止位、校验位等。
通过分频计算设置波特率，通过寄存器配置完成硬件初始化。
包含多项断言，保证参数合法性和硬件安全。
最终返回 RT_EOK 表示配置成功。
 */
static rt_err_t rt_uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct device_uart *uart;
    uart_bitwidth_t data_width = (uart_bitwidth_t)cfg->data_bits ;
    uart_stopbit_t stopbit = (uart_stopbit_t)cfg->stop_bits;
    uart_parity_t parity = (uart_parity_t)cfg->parity;

    uint32_t freq = sysctl_clock_get_freq(SYSCTL_CLOCK_APB0);//// 获取APB0总线时钟频率
    uint32_t divisor = freq / (uint32_t)cfg->baud_rate;// // 计算分频系数，用于设置波特率
    uint8_t dlh = divisor >> 12;//// 分频高位
    uint8_t dll = (divisor - (dlh << 12)) / __UART_BRATE_CONST;//// 分频低位
    uint8_t dlf = divisor - (dlh << 12) - dll * __UART_BRATE_CONST;//  // 分频小数部分
    // // 保存配置到串口设备结构体
    RT_ASSERT(serial != RT_NULL);
    serial->config = *cfg;
    //// 获取底层硬件信息结构体
    uart = serial->parent.user_data;
    RT_ASSERT(uart != RT_NULL);
    //// 根据硬件基地址获取通道号
    uart_device_number_t channel = _get_uart_channel(uart->hw_base);
    RT_ASSERT(channel != UART_DEVICE_MAX);
    //// 断言通道号有效
    RT_ASSERT(data_width >= 5 && data_width <= 8);
    // // 数据位宽必须在5~8之间
    if (data_width == 5)
    {
        RT_ASSERT(stopbit != UART_STOP_2);
    }
    else
    {
        RT_ASSERT(stopbit != UART_STOP_1_5);
    }
    //// 停止位寄存器值，1位为0，2位为1
    uint32_t stopbit_val = stopbit == UART_STOP_1 ? 0 : 1;
    uint32_t parity_val;
    switch (parity)
    {
        case UART_PARITY_NONE:
            parity_val = 0;//// 无校验
            break;
        case UART_PARITY_ODD:
            parity_val = 1;//// 奇校验
            break;
        case UART_PARITY_EVEN:
            parity_val = 3;// 偶校验
            break;
        default:
            RT_ASSERT(!"Invalid parity");
            break;
    }

    _uart[channel]->LCR |= 1u << 7; // 使能分频寄存器写入（DLAB位）
    _uart[channel]->DLH = dlh;  // 设置分频高位
    _uart[channel]->DLL = dll; // 设置分频低位
    _uart[channel]->DLF = dlf;// 设置分频小数部分
    _uart[channel]->LCR = 0; // 清空LCR寄存器
    _uart[channel]->LCR = (data_width - 5) |
                          (stopbit_val << 2) |
                          (parity_val << 3);
                          // 设置数据位、停止位、校验位
    _uart[channel]->LCR &= ~(1u << 7); // 关闭DLAB位，恢复正常访问
    _uart[channel]->IER |= 0x80; /* THRE */ // 使能发送缓冲区空中断
    _uart[channel]->FCR = UART_RECEIVE_FIFO_1 << 6 |
                          UART_SEND_FIFO_8 << 4 |
                          0x1 << 3 |
                          0x1;
                          // 配置FIFO控制寄存器，设置接收/发送FIFO深度，复位FIFO，启用FIFO

    return (RT_EOK);
}

static rt_err_t uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct device_uart *uart;

    uart = serial->parent.user_data;//// 获取底层硬件信息结构体
    /*
    🔹 STM32 系列（例如 STM32F4）
    提供多个 UART/USART 外设：
    USART1、USART2、USART3、UART4、UART5USART6 等
    实际上每个都是一个独立的 UART 通道（编号）。
    编程中可以用 USARTx 寄存器组（如 USART1->SR, USART2->DR 等）来控制。
    */
    uart_device_number_t channel = _get_uart_channel(uart->hw_base);//获取通道号 uart1

    RT_ASSERT(uart != RT_NULL);
    RT_ASSERT(channel != UART_DEVICE_MAX);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:// 清除中断
        /* Disable the UART Interrupt */
        rt_hw_interrupt_mask(uart->irqno);//// 屏蔽中断
        _uart[channel]->IER &= ~0x1; // 关闭接收中断
        break;

    case RT_DEVICE_CTRL_SET_INT:
        /* install interrupt */
        rt_hw_interrupt_install(uart->irqno, uart_irq_handler,
                                serial, serial->parent.parent.name);//static rt_err_t uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct device_uart *uart;

    uart = serial->parent.user_data; // 获取底层硬件信息结构体
    uart_device_number_t channel = _get_uart_channel(uart->hw_base); // 获取通道号

    RT_ASSERT(uart != RT_NULL); // 断言硬件信息结构体不为空
    RT_ASSERT(channel != UART_DEVICE_MAX); // 断言通道号有效

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* Disable the UART Interrupt */
        rt_hw_interrupt_mask(uart->irqno); // 屏蔽中断
        _uart[channel]->IER &= ~0x1;       // 禁用接收中断
        break;

    case RT_DEVICE_CTRL_SET_INT:
        /* install interrupt */
        rt_hw_interrupt_install(uart->irqno, uart_irq_handler,
                                serial, serial->parent.parent.name); // 安装中断服务函数
        rt_hw_interrupt_umask(uart->irqno); // 使能中断
        _uart[channel]->IER |= 0x1;         // 使能接收中断
        break;
    }

    return (RT_EOK); // 返回成功
}
        rt_hw_interrupt_umask(uart->irqno);
        _uart[channel]->IER |= 0x1;
        break;
    }

    return (RT_EOK);
}

static int drv_uart_putc(struct rt_serial_device *serial, char c)
{
    struct device_uart *uart = serial->parent.user_data;
    uart_device_number_t channel = _get_uart_channel(uart->hw_base);
    RT_ASSERT(channel != UART_DEVICE_MAX);

    while (_uart[channel]->LSR & (1u << 5));
    _uart[channel]->THR = c;

    return (1);
}

static int drv_uart_getc(struct rt_serial_device *serial)
{
    struct device_uart *uart = serial->parent.user_data;
    uart_device_number_t channel = _get_uart_channel(uart->hw_base);
    RT_ASSERT(channel != UART_DEVICE_MAX);

    if (_uart[channel]->LSR & 1)
        return (char)(_uart[channel]->RBR & 0xff);
    else
        return EOF;
    /* Receive Data Available */

    return (-1);
}

/* UART ISR */
static void uart_irq_handler(int irqno, void *param)
{
    struct rt_serial_device *serial = (struct rt_serial_device *)param;
    struct device_uart *uart = serial->parent.user_data;
    uart_device_number_t channel = _get_uart_channel(uart->hw_base);
    RT_ASSERT(channel != UART_DEVICE_MAX);

    /* read interrupt status and clear it */
    if (_uart[channel]->LSR)
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
}

/* WEAK for SDK 0.5.6 */

rt_weak void uart_debug_init(uart_device_number_t uart_channel)
{

}

