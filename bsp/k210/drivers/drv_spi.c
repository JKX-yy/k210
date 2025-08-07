/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-18     ZYH          first version
 */
/*
主要包括 
1.头文件配置
2.数据结构定义
3.SPI配置函数
4.SPI数据传输函数
5.SPI初始化
*/
// #ifndef RT_USING_SPI
// #error "RT_USING_SPI not defined!"
// #endif

#define RT_USING_SPI 
#define BSP_USING_SPI1
#define BSP_SPI1_USING_SS0

#include <rtthread.h>  // RT-Thread操作系统核心头文件
#include <rtdevice.h> // RT-Thread设备驱动框架头文件
#include "../packages/K210-SDK-latest/lib/drivers/include/spi.h"   //K210  spi相关硬件配置
#include <rtconfig.h>

#ifdef RT_USING_SPI
#include "drv_spi.h" //本驱动的头文件
#include <drv_io_config.h>  // IO引脚配置
#include <drivers/dev_spi.h>  //SPI设备定义
#include "dmalock.h"  //DMA通道锁管理
#include <sysctl.h> //系统控制相关
#include <gpiohs.h>  //高速GPIO操作
#include <string.h> 
#include "utils.h" //实用工具
#include  "../packages/K210-SDK-latest/lib/bsp/include/platform.h"
#include  "../packages/K210-SDK-latest/lib/drivers/include/sysctl.h"
#define DRV_SPI_DEVICE(spi_bus)    (struct drv_spi_bus *)(spi_bus)// 类型转换宏

#define MAX_CLOCK   (40000000UL)  // SPI最大时钟频率40MHz

// SPI总线数据结构
struct drv_spi_bus
{
    struct rt_spi_bus parent;    // RT-Thread标准的SPI总线结构
    spi_device_num_t spi_instance;  // SPI实例编号(0,1,2,3)
    dmac_channel_number_t dma_send_channel;// 发送DMA通道
    dmac_channel_number_t dma_recv_channel; // 接收DMA通道
    struct rt_completion dma_completion;  // DMA完成信号量
};
/*
// struct rt_spi_bus
// {
//     struct rt_device parent;
//     rt_uint8_t mode;
//     const struct rt_spi_ops *ops;

// #ifdef RT_USING_DM
//     rt_base_t cs_pins[RT_SPI_CS_CNT_MAX];
//     rt_uint8_t cs_active_vals[RT_SPI_CS_CNT_MAX];
//     rt_bool_t slave;
//     int num_chipselect;
// #endif  //RT_USING_DM 
    // struct rt_mutex lock;
    // struct rt_spi_device *owner;
// };

struct rt_spi_ops
{
    rt_err_t (*configure)(struct rt_spi_device *device, struct rt_spi_configuration *configuration);
    rt_ssize_t (*xfer)(struct rt_spi_device *device, struct rt_spi_message *message);
};

*/

// 片选(CS)引脚数据结构
struct drv_cs
{
    int cs_index;  // 片选索引号
    int cs_pin;    // 片选引脚号
};
// SPI硬件寄存器指针数组  spi_t  各种寄存器
static volatile spi_t *const spi_instance[4] =
{
    (volatile spi_t *)SPI0_BASE_ADDR, //SPI0
    (volatile spi_t *)SPI1_BASE_ADDR, //SPI1
    (volatile spi_t *)SPI_SLAVE_BASE_ADDR,//从SPI
    (volatile spi_t *)SPI3_BASE_ADDR  //SPI3
};

//SPI配置函数
static rt_err_t drv_spi_configure(struct rt_spi_device *device,
                                  struct rt_spi_configuration *configuration)
{
    rt_err_t ret = RT_EOK;
    int freq = 0;
    // 获取总线数据和片选引脚信息
    struct drv_spi_bus *bus = DRV_SPI_DEVICE(device->bus);
    struct drv_cs * cs = (struct drv_cs *)device->parent.user_data;
    // 断言总线不为空
    RT_ASSERT(bus != RT_NULL);
    // 配置片选引脚为输出模式，并初始化为高电平(不选中)
    gpiohs_set_drive_mode(cs->cs_pin, GPIO_DM_OUTPUT);
    gpiohs_set_pin(cs->cs_pin, GPIO_PV_HIGH);

#ifdef BSP_USING_SPI1_AS_QSPI
    /* Todo:QSPI*/
#else
 // 初始化SPI硬件
    spi_init(bus->spi_instance, configuration->mode & RT_SPI_MODE_3, SPI_FF_STANDARD, configuration->data_width, 0);
/*
    spi_init(bus->spi_instance,             // SPI实例号
            configuration->mode & RT_SPI_MODE_3, // SPI模式(0-3)
            SPI_FF_STANDARD,                // 标准SPI格式
            configuration->data_width,      // 数据位宽
            0);                             // 保留参数*/
#endif
// 设置SPI时钟频率(不超过最大频率)
    freq = spi_set_clk_rate(bus->spi_instance, configuration->max_hz > MAX_CLOCK ? MAX_CLOCK : configuration->max_hz);
    rt_kprintf("set spi freq %d\n", freq); // 打印设置的频率
    return ret;
}


//当然可以！你提供的这个函数是 K210 的 SPI 控制器配置函数之一，函数名是：
/*
用来设置SPI的模式  SPIn  收  发 还是收发同时
它的作用是：设置某个 SPI 控制器的 TMOD 模式（传输模式），例如：
只发送（transmit-only）
只接收（receive-only）
同时收发（transmit & receive，full-duplex）
EEPROM 模式（很少用）
*/
void __spi_set_tmod(uint8_t spi_num, uint32_t tmod)
{   
    //断言检查，spi_num 是否是合法编号（0~3）。
    RT_ASSERT(spi_num < SPI_DEVICE_MAX);
    volatile spi_t *spi_handle = spi[spi_num]; //获取对应 SPI 控制器的寄存器结构体指针（比如 spi[0] 就是 SPI0）
    //找到SPI寄存器地址 此处是SPI1
    //spi_t是spi寄存器，其中contrl0是用于控制模式选择的
    uint8_t tmod_offset = 0;
    switch(spi_num)
    {
        case 0:
        case 1:
        case 2:
            tmod_offset = 8; //SPI0/SPI1/SPI2 的 TMOD 在 ctrlr0 的第 8:9 位
            break;
        case 3:
        default:
            tmod_offset = 10;//SPI3 的 TMOD 在 ctrlr0 的第 10:11 位
            break;
    }
    //set_bit() 是一个宏或函数，用来清除并设置某几位
    set_bit(&spi_handle->ctrlr0, 3 << tmod_offset, tmod << tmod_offset);
    //spi_handle->ctrlr0 = (spi_handle->ctrlr0 & ~(3 << tmod_offset)) | (tmod << tmod_offset);
    //与上一句等效
}
// 简单的DMA完成回调函数
/*
你调用 spi_transfer_dma(buffer, 512)，准备收 512 字节数据；
SPI 控制器开始工作，同时配置 DMA，让它在 SPI 数据到来时自动写入 buffer；
CPU 线程进入 rt_completion_wait()，等待 DMA 完成；
当 512 字节全部传输完毕，DMA 触发中断；
中断处理函数（也就是你说的 dma_irq_callback）被触发；
在中断中调用 rt_completion_done()，唤醒刚才等待的线程；
线程继续执行，处理收好的数据。
*/
int dma_irq_callback(void *ctx)
{
    struct rt_completion * cmp = ctx; // 获取完成信号量
    if(cmp)
    {
        rt_completion_done(cmp); // 通知DMA传输完成
    }
}

/*作用：驱动层的 SPI 数据传输函数。支持 DMA 模式。根据 message 中的信息发送/接收数据，可能是：
仅发送、
仅接收、
同时收发（full-duplex）。
*/
static rt_uint32_t drv_spi_xfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    /*
    struct rt_spi_message
{
    const void *send_buf;
    void *recv_buf;
    rt_size_t length;
    struct rt_spi_message *next;

    unsigned cs_take    : 1;
    unsigned cs_release : 1;
};*/
    struct drv_spi_bus *bus = DRV_SPI_DEVICE(device->bus);  //获取 SPI 总线对象 bus（包含 DMA 通道等信息）。
    struct drv_cs * cs = (struct drv_cs *)device->parent.user_data; //获取 cs（片选对象，控制 GPIO 片选）。
    struct rt_spi_configuration *cfg = &device->config;  //cfg：SPI 配置，如数据宽度、极性等。
    uint32_t * tx_buff = RT_NULL;    //tx_buff / rx_buff：为 DMA 分配的临时缓存。
    uint32_t * rx_buff = RT_NULL;
    int i;
    rt_ubase_t dummy = 0xFFFFFFFFU;  //dummy：SPI 发数据时占位。 rt_uint64_t   
    if(cfg->data_width != 8)  //本驱动只支持 8-bit 数据宽度，如果不是则退出。
    {
        return 0;
    }

    RT_ASSERT(bus != RT_NULL); 

    if(message->cs_take)   //如果需要拉低片选，先设置 GPIO 低电平，选中 SPI 外设（如 SD 卡）。
    //什么时候需要设置片选  我SPI0的总线上只有一个外设
    {
        gpiohs_set_pin(cs->cs_pin, GPIO_PV_LOW);
    }

    //开始DMA传输逻辑
    if(message->length)
    {
        bus->dma_send_channel = DMAC_CHANNEL_MAX; //初始化 发送 DMA 通道为无效值；
        bus->dma_recv_channel = DMAC_CHANNEL_MAX;//初始化 接收 DMA 通道为无效值；
        //初始化完成量对象 dma_completion，用于等待 DMA 结束。
        rt_completion_init(&bus->dma_completion); //置零
        /*
            占用 DMA 接收通道；
            选择 SPI 接收为 DMA 源；
            为接收数据分配临时缓冲区（32bit对齐）；
            分配失败直接跳转退出。
            const void *send_buf;
            void *recv_buf;
        */
        if(message->recv_buf) 
        {
            dmalock_sync_take(&bus->dma_recv_channel, RT_WAITING_FOREVER);//自动分配一个通道 利用  dmalock_sync_take
            /*这段函数 sysctl_dma_select 是用于在 K210 SoC 中**将 DMA 通道与特定外设请求源绑定（映射）**的函数，目的是指定某个 DMA 通道监听哪个外设（如 SSI、UART、I2C、ADC 等）的 DMA 请求。*/
            sysctl_dma_select(bus->dma_recv_channel, SYSCTL_DMA_SELECT_SSI0_RX_REQ + bus->spi_instance * 2);//第二个参数是位移从SPI0开始计算  得到RX TX
            //RT-Thread 中实现的一个标准内存分配函数 rt_calloc，以及一次调用它的例子
            /*
            这是 RT-Thread 中模仿标准 C 库的 calloc() 函数写的：
            count: 要分配多少个元素；
            size: 每个元素的大小；
            返回值：返回一块总大小为 count * size 的内存，并且所有字节初始化为 0；
            rt_weak: 表示这是一个弱定义函数，如果用户定义了同名强符号函数，则可以覆盖它。
            */
            rx_buff = rt_calloc(message->length * 4, 1);
            if(!rx_buff)
            {
                goto transfer_done;
            }
        }
        /*
            占用 DMA 发送通道；
            选择 SPI 发送为 DMA 源；
            为发送数据申请缓冲区；
            将原始 uint8_t 数据写入 32-bit 对齐缓冲区。
        */
        if(message->send_buf)
        {
            dmalock_sync_take(&bus->dma_send_channel, RT_WAITING_FOREVER);
            sysctl_dma_select(bus->dma_send_channel, SYSCTL_DMA_SELECT_SSI0_TX_REQ + bus->spi_instance * 2);
            tx_buff = rt_malloc(message->length * 4);
            if(!tx_buff)
            {
                goto transfer_done;
            }
            for(i = 0; i < message->length; i++)
            {
                tx_buff[i] = ((uint8_t *)message->send_buf)[i];
            }
        }

        if(message->send_buf && message->recv_buf)
        {
            /*
            设置 SPI 为发送 + 接收模式（全双工）；
            同时注册 发送和接收 DMA 中断回调；
            启用 SPI + DMA；
            设置 DMA：
            接收：SPI_DR ➜ rx_buff；
            发送：tx_buff ➜ SPI_DR。
                注册 DMA 接收中断回调；
                设置 SPI 为 收发模式；
                使能 DMA 发送 + 接收；
                启用 SPI 模块。
            */
           //注册中断函数
            dmac_irq_register(bus->dma_recv_channel, dma_irq_callback, &bus->dma_completion, 1);
            //设置spi寄存器  收发模式
            __spi_set_tmod(bus->spi_instance, SPI_TMOD_TRANS_RECV);
            //启 DMA 接收 + DMA 发送（位0 + 位1）。
            spi_instance[bus->spi_instance]->dmacr = 0x3;  // 寄存器/位  dmacr = 3 含义  使能DMA发送和接收
            //启用 SPI 模块。
            spi_instance[bus->spi_instance]->ssienr = 0x01; //寄存器/位  ssienr = 1 含义 使能SPI  
            
            /*设置 DMA 接收：SPI 接收寄存器 ➜ rx_buff；设置 DMA 发送：tx_buff ➜ SPI 发送寄存器。*/  
            //    /* SPI Data Register 0-36    (0x60 -- 0xec)      volatile uint32_t dr[36]; */
            //通道   源地址  目的地址
            /*
            void dmac_set_single_mode(
                dmac_channel_number_t channel,       // DMA 通道号
                void *src,                           // 源地址
                void *dst,                           // 目的地址
                dmac_addr_increment_t src_inc,      // 源地址是否自增
                dmac_addr_increment_t dst_inc,      // 目的地址是否自增
                dmac_msize_t dmac_msize,            // 总线突发大小（一次搬几个）
                dmac_transfer_width_t trans_width,  // 传输单位宽度（字节/半字/字）
                size_t block_size                   // 总传输数量（单位个数，不是字节）
            );
            */
            dmac_set_single_mode(bus->dma_recv_channel, (void *)(&spi_instance[bus->spi_instance]->dr[0]), rx_buff, DMAC_ADDR_NOCHANGE, DMAC_ADDR_INCREMENT,
                           DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, message->length);
            dmac_set_single_mode(bus->dma_send_channel, tx_buff, (void *)(&spi_instance[bus->spi_instance]->dr[0]), DMAC_ADDR_INCREMENT, DMAC_ADDR_NOCHANGE,
                           DMAC_MSIZE_4, DMAC_TRANS_WIDTH_32, message->length);
        
                        }
        else if(message->send_buf)
        {
            /*
                设置为 SPI 发送模式；
                只注册发送 DMA；
                配置 DMA 将 tx_buff 发往 SPI。
            */
            dmac_irq_register(bus->dma_send_channel, dma_irq_callback, &bus->dma_completion, 1);
            __spi_set_tmod(bus->spi_instance, SPI_TMOD_TRANS);
            spi_instance[bus->spi_instance]->dmacr = 0x2;
            spi_instance[bus->spi_instance]->ssienr = 0x01;
            dmac_set_single_mode(bus->dma_send_channel, tx_buff, (void *)(&spi_instance[bus->spi_instance]->dr[0]), DMAC_ADDR_INCREMENT, DMAC_ADDR_NOCHANGE,
                           DMAC_MSIZE_4, DMAC_TRANS_WIDTH_32, message->length);
        }
        else if(message->recv_buf)
        {
            /*
                SPI 设置为接收模式；
                告知控制器即将接收多少字节；
                dr[0] = 0xFF：触发一次 dummy write（SPI 为主发，必须写点什么才收）；
                设置 DMA 从 SPI ➜ rx_buff。
                这段代码是 SPI “仅接收模式”+ DMA 传输配置的完整流程，在 K210（或其他 SPI 主机架构中）非常典型 —— 你希望仅接收数据，但因为 SPI 是全双工接口，主机必须“发东西”才能“收东西”，因此就需要一个技巧：Dummy Write（伪写）。

            */
            dmac_irq_register(bus->dma_recv_channel, dma_irq_callback, &bus->dma_completion, 1);
            __spi_set_tmod(bus->spi_instance, SPI_TMOD_RECV);
            spi_instance[bus->spi_instance]->ctrlr1 = message->length - 1;
            /*
             设置要接收的数据长度（控制器接收计数器）
            这是 SPI 控制器的 CTRL1 寄存器；
            写入 N-1，表示接收 N 个数据（单位为 word）；
            控制器会自动接收完这 N 个后停止（且触发 DMA 完成中断）。
            */
            spi_instance[bus->spi_instance]->dmacr = 0x1; //所以 0x1 表示只开启接收。
            spi_instance[bus->spi_instance]->ssienr = 0x01; //ssienr = 1 启动 SPI（SSI Enable）； 不写不会启动
            spi_instance[bus->spi_instance]->dr[0] = 0xFF;
            /*
             Dummy Write：必须触发 SCLK 才能接收数据
            即使你设置了“只接收”，SPI 是主机，也得先“动起来”，而动起来的方式就是先写一个字；
            这会产生时钟（SCLK）；
            从机才能在 SCLK 上送出第一字节。
            ⚠️ 注意：
            后续的数据是自动由 DMA 从 rx_fifo 读出来的；
            但这个 Dummy Write 是启动的关键“第一脚”。
            */
            dmac_set_single_mode(bus->dma_recv_channel, (void *)(&spi_instance[bus->spi_instance]->dr[0]), rx_buff, DMAC_ADDR_NOCHANGE, DMAC_ADDR_INCREMENT,
                           DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, message->length);
        /*
        
        [ SPI 收模式开启 ]
        │
        ├─► 设置接收长度 (ctrlr1 = N-1)
        ├─► 启用 DMA 接收
        ├─► 启动 SPI 模块 (ssienr=1)
        ├─► Dummy write dr[0]=0xFF
        └─► DMA 从 SPI_DR ➜ rx_buff
                     ↑
           SPI 硬件接收从 MISO

           */
        
                        }
        else
        {
            goto transfer_done;
        }
        //启动传输
        //
        spi_instance[bus->spi_instance]->ser = 1U << cs->cs_index;//使能片选。

        rt_completion_wait(&bus->dma_completion, RT_WAITING_FOREVER);//阻塞等待中断回调唤醒线程（DMA 完成）；
        //清理&后处理   注销中断。
        /*
        收完或发完后，注销中断回调，避免残留回调影响后续；
        dmac_irq_unregister(channel) 会将通道对应的中断注册项清除。
        */
        if(message->recv_buf)
            dmac_irq_unregister(bus->dma_recv_channel);
        else
            dmac_irq_unregister(bus->dma_send_channel);

        // wait until all data has been transmitted  等待 SPI 状态寄存器：发完、收完；0x05 是 busy + fifo not empty。
        /*
        这是一个状态轮询等待 SPI 传输完成的判断语句。
        📌 解释：
        sr 是 SPI 的 Status Register；
        bit[0]: TFNF（Transmit FIFO Not Full）
        bit[2]: TFE（Transmit FIFO Empty）
        所以 0x05 表示 bit0 和 bit2；
        == 0x04 表示：
        TFE = 1：发送 FIFO 空；
        TFNF = 0：不能再写入了（FIFO 满或禁用）；
        ✅ 作用：
        确保 FIFO 中数据都发送完了；
        通常用于 SPI 主机在关闭片选信号前 清空发送缓存，避免截断数据。
*/
        while ((spi_instance[bus->spi_instance]->sr & 0x05) != 0x04)
            ;
            /*取消片选；禁用 SPI 控制器。*/
        spi_instance[bus->spi_instance]->ser = 0x00;//失能片选
        spi_instance[bus->spi_instance]->ssienr = 0x00;//禁止SPI控制器
        //把 rx_buff 中数据拷贝回用户 buffer。
        if(message->recv_buf)
        {
            for(i = 0; i < message->length; i++)
            {
                ((uint8_t *)message->recv_buf)[i] = (uint8_t)rx_buff[i];
            }
        }
    //清理资源
    /*
    释放锁；释放 malloc 分配的临时 buffer。
    */
transfer_done:
        dmalock_release(bus->dma_send_channel); //释放通道信号两
        dmalock_release(bus->dma_recv_channel);
        if(tx_buff)
        {
            rt_free(tx_buff); //释放临时buffer
        }
        if(rx_buff)
        {
            rt_free(rx_buff);
        }
    }
        //如果需要自动释放片选，则拉高 GPIO。
    if(message->cs_release)
    {
        gpiohs_set_pin(cs->cs_pin, GPIO_PV_HIGH);
    }
        //返回传输字节数。
    return message->length;
}

const static struct rt_spi_ops drv_spi_ops =
{
    drv_spi_configure,
    drv_spi_xfer
};

int rt_hw_spi_init(void)
{
    rt_err_t ret = RT_EOK;

#ifdef BSP_USING_SPI1
    {
        static struct drv_spi_bus spi_bus1;
        spi_bus1.spi_instance = SPI_DEVICE_1;
        ret = rt_spi_bus_register(&spi_bus1.parent, "spi1", &drv_spi_ops);

#ifdef BSP_SPI1_USING_SS0
        {
            static struct rt_spi_device spi_device10;
            static struct drv_cs cs10 =
            {
                .cs_index = SPI_CHIP_SELECT_0, //	SPI 控制器内部的 片选编号（SPI0/1 支持 SS0 ~ SS3），控制 ser 寄存器的 bit
                .cs_pin = SPI1_CS0_PIN  //SPI1_CS0_PIN 是一个枚举值（整数索引）；
                //	实际连接到 CS 的引脚（GPIO 口）— 用于手动拉低/拉高选中/释放外设
            };
            //创建spi10 绑定到spi1
            rt_spi_bus_attach_device(&spi_device10, "spi10", "spi1", (void *)&cs10);
        }
#endif

#ifdef BSP_SPI1_USING_SS1
        {
            static struct rt_spi_device spi_device11;
            static struct drv_cs cs11 =
            {
                .cs_index = SPI_CHIP_SELECT_1,
                .cs_pin = SPI1_CS1_PIN
            };
            rt_spi_bus_attach_device(&spi_device11, "spi11", "spi1", (void *)&cs11);
        }
#endif

#ifdef BSP_SPI1_USING_SS2
        {
            static struct rt_spi_device spi_device12;
            static struct drv_cs cs12 =
            {
                .cs_index = SPI_CHIP_SELECT_2,
                .cs_pin = SPI1_CS2_PIN
            };
            rt_spi_bus_attach_device(&spi_device12, "spi12", "spi1", (void *)&cs12);
        }
#endif

#ifdef BSP_SPI1_USING_SS3
        {
            static struct rt_spi_device spi_device13;
            static struct drv_cs cs13 =
            {
                .cs_index = SPI_CHIP_SELECT_2,
                .cs_pin = SPI1_CS2_PIN
            };
            rt_spi_bus_attach_device(&spi_device13, "spi13", "spi1", (void *)&cs13);
        }
#endif
    }
#endif
    return ret;
}
INIT_DEVICE_EXPORT(rt_hw_spi_init);
#endif
