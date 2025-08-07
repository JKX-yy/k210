/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-04-17     Bernard      first version.
 * 2010-07-15     aozima       Modify read/write according new block driver interface.
 * 2012-02-01     aozima       use new RT-Thread SPI drivers.
 * 2012-04-11     aozima       get max. data transfer rate from CSD[TRAN_SPEED].
 * 2012-05-21     aozima       update MMC card support.
 * 2018-03-09     aozima       fixed CSD Version 2.0 sector count calc.
 */

#include <string.h>
#include "dev_spi_msd.h"

//#define MSD_TRACE

#ifdef MSD_TRACE
    #define MSD_DEBUG(...)         rt_kprintf("[MSD] %d ", rt_tick_get()); rt_kprintf(__VA_ARGS__);
#else
    #define MSD_DEBUG(...)
#endif /* #ifdef MSD_TRACE */

#define DUMMY                 0xFF

#define CARD_NCR_MAX          9

#define CARD_NRC              1
#define CARD_NCR              1

static struct msd_device  _msd_device;

/* function define */
static rt_bool_t rt_tick_timeout(rt_tick_t tick_start, rt_tick_t tick_long);

//note: 这个函数的作用是确保当前 SPI 设备成为 SPI 总线的“所有者”，并进行必要的配置。
static rt_err_t MSD_take_owner(struct rt_spi_device *spi_device);

//等待返回token  或等待返回DUMMY
static rt_err_t _wait_token(struct rt_spi_device *device, uint8_t token);
static rt_err_t _wait_ready(struct rt_spi_device *device);//recv == DUMMY
//它会在 RT-Thread 设备框架中被调用。具体来说，它是在调用 rt_device_init() 函数时被触发的。
/*
msd_init 函数会注册一个名为 "sd0" 的 MSD 设备，并将其与 SPI 设备 "spi10" 绑定。
在注册过程中，rt_device_register 会将 MSD 设备的初始化函数 rt_msd_init 绑定到设备的 init 回调中。
*/
static rt_err_t  rt_msd_init(rt_device_t dev);

static rt_err_t  rt_msd_open(rt_device_t dev, rt_uint16_t oflag);
static rt_err_t  rt_msd_close(rt_device_t dev);

//
static rt_ssize_t rt_msd_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size);
static rt_ssize_t rt_msd_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size);

static rt_ssize_t rt_msd_sdhc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size);
static rt_ssize_t rt_msd_sdhc_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size);

static rt_err_t rt_msd_control(rt_device_t dev, int cmd, void *args);


/*
这个函数 MSD_take_owner 的作用是确保当前 SPI 设备成为 SPI 总线的“所有者”并进行必要的配置。具体流程如下：

尝试获取 SPI 总线的互斥锁，保证线程安全，避免多个设备同时访问总线。
判断当前总线的 owner 是否为本设备（spi_device）。如果不是，说明需要重新配置总线。
调用总线的 configure 方法，用当前设备的配置参数重新配置 SPI 总线。
配置成功后，将 owner 设置为当前设备，表示总线现在归这个设备所有。
最后返回操作结果（rt_err_t 类型）。
常见场景：
在多设备共享同一个 SPI 总线时，每个设备通信前都需要确保总线已按自己的参数配置，并且自己是 owner，防止配置冲突。
*/
static rt_err_t MSD_take_owner(struct rt_spi_device *spi_device)
{
    rt_err_t result;

    result = rt_mutex_take(&(spi_device->bus->lock), RT_WAITING_FOREVER);
    if (result == RT_EOK)
    {
        if (spi_device->bus->owner != spi_device)
        {
            /* not the same owner as current, re-configure SPI bus */
            result = spi_device->bus->ops->configure(spi_device, &spi_device->config);
            if (result == RT_EOK)
            {
                /* set SPI bus owner */
                spi_device->bus->owner = spi_device;
            }
        }
    }

    return result;
}

static rt_bool_t rt_tick_timeout(rt_tick_t tick_start, rt_tick_t tick_long)
{
    rt_tick_t tick_end = tick_start + tick_long;
    rt_tick_t tick_now = rt_tick_get();
    rt_bool_t result = RT_FALSE;

    if (tick_end >= tick_start)
    {
        if (tick_now >= tick_end)
        {
            result = RT_TRUE;
        }
        else
        {
            result = RT_FALSE;
        }
    }
    else
    {
        if ((tick_now < tick_start) && (tick_now >= tick_end))
        {
            result = RT_TRUE;
        }
        else
        {
            result = RT_FALSE;
        }
    }

    return result;
}

static uint8_t crc7(const uint8_t *buf, int len)
{
    unsigned char   i, j, crc, ch, ch2, ch3;

    crc = 0;

    for (i = 0; i < len; i ++)
    {
        ch = buf[i];

        for (j = 0; j < 8; j ++, ch <<= 1)
        {
            ch2 = (crc & 0x40) ? 1 : 0;
            ch3 = (ch & 0x80) ? 1 : 0;

            if (ch2 ^ ch3)
            {
                crc ^= 0x04;
                crc <<= 1;
                crc |= 0x01;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}


/*
定义一个静态函数 _send_cmd，用于向 SD 卡发送命令并接收响应。
参数说明：
device：SPI 设备指针。
cmd：命令字节。
arg：命令参数。
crc：CRC 校验码。
type：响应类型。
response：用于存放响应数据的缓冲区。

是的，SD 卡内部确实有一个控制器芯片，它能够根据接收到的命令和参数（如扇区地址）自动定位到相应的扇区位置，并执行读写操作。

以下是详细解释：

---

### 1. **SD 卡内部的控制器芯片**

SD 卡内部包含一个微控制器（控制芯片），它负责管理存储介质（如 NAND 闪存）的操作。控制器芯片的功能包括：
- 解析主机发送的命令（如读写命令）。
- 根据命令中的参数（如扇区地址）定位到存储介质上的具体位置。
- 执行读写操作，并返回结果。

---

### 2. **`_send_cmd` 的作用**

在 dev_spi_msd.c 文件中，`_send_cmd` 函数会将命令和参数（如扇区地址）打包成符合 SD 卡协议的格式，通过 SPI 接口发送给 SD 卡。

#### **代码片段：`_send_cmd`**
```c
cmd_buffer[2] = (uint8_t)(arg >> 24);   // 高字节
cmd_buffer[3] = (uint8_t)(arg >> 16);
cmd_buffer[4] = (uint8_t)(arg >> 8);
cmd_buffer[5] = (uint8_t)(arg);         // 低字节
```
- `arg` 是扇区的起始地址（以字节为单位）。
- 它被分成 4 个字节，填入命令缓冲区 `cmd_buffer` 中。
- 这个缓冲区通过 SPI 接口发送给 SD 卡。

---

### 3. **SD 卡如何处理命令**

SD 卡的控制器芯片会解析主机发送的命令，并根据命令类型和参数执行相应的操作。例如：

#### **读操作**
- 主机发送 `READ_SINGLE_BLOCK` 命令，并附带扇区地址。
- SD 卡控制器根据扇区地址定位到存储介质上的具体位置。
- SD 卡控制器读取数据，并通过 SPI 接口返回给主机。

#### **写操作**
- 主机发送 `WRITE_BLOCK` 命令，并附带扇区地址。
- 主机随后通过 SPI 接口发送数据块。
- SD 卡控制器将数据写入到指定的扇区。

---

### 4. **SD 卡协议的作用**

SD 卡协议定义了主机与 SD 卡之间的通信规则，包括：
- 命令格式（如 `CMD17` 表示读单块，`CMD24` 表示写单块）。
- 参数格式（如扇区地址）。
- 响应格式（如成功或失败的状态码）。

主机通过 SPI 接口发送符合协议的命令，SD 卡控制器根据协议解析命令并执行操作。

---

### 5. **驱动程序的作用**

驱动程序（如 dev_spi_msd.c）的作用是：
- 将高层的读写请求（如逻辑块号）转换为符合 SD 卡协议的命令。
- 通过 SPI 接口与 SD 卡通信。
- 处理 SD 卡返回的响应。

驱动程序不需要关心 SD 卡内部的具体实现，只需要按照协议发送命令和接收数据即可。

---

### 6. **总结**

- **SD 卡内部有控制器芯片**：它能够根据主机发送的命令和参数（如扇区地址）自动定位到存储介质上的具体位置，并执行读写操作。
- **驱动程序的作用**：负责将高层的读写请求转换为符合 SD 卡协议的命令，通过 SPI 接口与 SD 卡通信。
- **主机与 SD 卡的交互**：主机通过 SPI 接口发送命令，SD 卡控制器解析命令并执行相应的操作。
*/
static rt_err_t _send_cmd(
    struct rt_spi_device *device,
    uint8_t cmd,
    uint32_t arg,
    uint8_t crc,
    response_type type,
    uint8_t *response
)
{
    struct rt_spi_message message;  //定义 SPI 消息结构体 message。
    uint8_t cmd_buffer[8];    //定义命令缓冲区 cmd_buffer，用于存放要发送的命令和参数
    uint8_t recv_buffer[sizeof(cmd_buffer)]; //定义接收缓冲区 recv_buffer，用于接收数据。
    uint32_t i; //定义循环变量 i

    //构造SD卡命令包
    cmd_buffer[0] = DUMMY; // 第 0 字节为 DUMMY（通常为 0xFF，用于时钟填充）。
    cmd_buffer[1] = (cmd | 0x40); // 第 1 字节为命令字节，加上 0x40 标识这是命令而不是数据。
    //_send_cmd 函数会将 arg 参数（即扇区的起始地址）打包到 SD 卡命令中，并通过 SPI 接口发送给 SD 卡。
    cmd_buffer[2] = (uint8_t)(arg >> 24);   //第 2~5 字节为命令参数（高字节在前，低字节在后）。
    cmd_buffer[3] = (uint8_t)(arg >> 16);
    cmd_buffer[4] = (uint8_t)(arg >> 8);
    cmd_buffer[5] = (uint8_t)(arg);
    //CRC 校验处理
    /*
    如果 CRC 为 0，则自动计算 CRC7 校验码，并设置最后一位为 1（符合 SD 卡协议）。
    第 6 字节为 CRC 校验码
*/
    if (crc == 0x00)
    {
        crc = crc7(&cmd_buffer[1], 5);
        crc = (crc << 1) | 0x01;
    }
    cmd_buffer[6] = (crc);

    cmd_buffer[7] = DUMMY;  //命令包最后加一个 DUMMY 字节（时钟填充）。

    /* initial message 初始化 SPI 消息  设置 SPI 消息结构体，准备发送命令包。 */
    message.send_buf = cmd_buffer;
    message.recv_buf = recv_buffer;
    message.length = sizeof(cmd_buffer);
    message.cs_take = message.cs_release = 0;//	表示你手动控制 CS，系统不会自动管它
    //等待 SD 卡准备好，避免在卡忙时发送命令。
    _wait_ready(device);

    /* transfer message  通过 SPI 总线发送命令包。 */
    device->bus->ops->xfer(device, &message);
    //等待响应  循环发送 DUMMY 字节，等待 SD 卡返回有效响应（最高位为 0 ）响应存入 response 缓冲区。
    for (i = CARD_NCR; i < (CARD_NCR_MAX + 1); i++)
    {
        uint8_t send = DUMMY;

        /* initial message */
        message.send_buf = &send;
        message.recv_buf = response;
        message.length = 1;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
        //响应存入 response 缓冲区。
        if (0 == (response[0] & 0x80)) 
        {
            break;
        }
    } /* wait response */

    //响应超时处理
    if ((CARD_NCR_MAX + 1) == i)
    {
        return -RT_ERROR;//fail 如果超过最大等待次数还没收到响应，则返回错误
    }

    //recieve other byte  根据响应类型处理后续字节  
    //R1 类型只需一个字节，直接返回成功。
    if (type == response_r1)
    {
        return RT_EOK;
    }
    else if (type == response_r1b)  //R1b 响应（带忙状态）
    {
        rt_tick_t tick_start = rt_tick_get();
        uint8_t recv;

        while (1)
        {
            /* initial message   R1B类型需等待SD卡忙状态结束（收到DUMMY） 超时则返回错误*/
            message.send_buf = RT_NULL;
            message.recv_buf = &recv;
            message.length = 1;
            message.cs_take = message.cs_release = 0;

            /* transfer message */
            device->bus->ops->xfer(device, &message);

            if (recv == DUMMY)
            {
                return RT_EOK;
            }

            if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(2000)))
            {
                return -RT_ETIMEOUT;
            }
        }
    }
    else if (type == response_r2) //R2 类型需再接收一个字节，存入响应缓冲区。
    {
        /* initial message */
        /* Prevent non-aligned address access, use recv_buffer to receive data */
        message.send_buf = RT_NULL;
        message.recv_buf = recv_buffer;
        message.length = 1;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
        response[1] = recv_buffer[0];
    }
    else if ((type == response_r3) || (type == response_r7)) //R3/R7 类型需再接收 4 个字节，存入响应缓冲区。
    {
        /* initial message */
        message.send_buf = RT_NULL;
        message.recv_buf = recv_buffer;
        message.length = 4;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
        response[1] = recv_buffer[0];
        response[2] = recv_buffer[1];
        response[3] = recv_buffer[2];
        response[4] = recv_buffer[3];
    }
    else
    {
        return -RT_ERROR; // unknow type? 如果响应类型未知，返回错误。
    }

    return RT_EOK;
}
/*
用途：用于等待 SD 卡返回特定的令牌（token），表示某种特定的状态或操作开始。
触发条件：等待 SD 卡返回指定的 token，例如数据起始标志（MSD_TOKEN_READ_START）。
典型场景：在读取数据块时，等待 SD 卡返回数据起始标志。
*/
static rt_err_t _wait_token(struct rt_spi_device *device, uint8_t token)
{  //等待返回tokn
    struct rt_spi_message message;
    rt_tick_t tick_start;
    uint8_t send, recv;

    tick_start = rt_tick_get();

    /* wati token */
    /* initial message */
    send = DUMMY;
    message.send_buf = &send;
    message.recv_buf = &recv;
    message.length = 1;
    message.cs_take = message.cs_release = 0;

    while (1)
    {
        /* transfer message */
        device->bus->ops->xfer(device, &message);

        if (recv == token)
        {
            return RT_EOK;
        }

        if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(CARD_WAIT_TOKEN_TIMES)))
        {
            MSD_DEBUG("[err] wait data start token timeout!\r\n");
            return -RT_ETIMEOUT;
        }
    } /* wati token */
}

/*
这段代码是一个等待 SPI 设备准备好的函数。它的主要作用是轮询 SPI 设备，直到设备返回一个“准备好”的信号，或者超时为止。

详细解释
函数名：_wait_ready
参数：struct rt_spi_device *device —— SPI 设备对象。
返回值：rt_err_t —— 操作结果，成功返回 RT_EOK，超时返回 -RT_ETIMEOUT。
步骤说明
初始化消息结构体
构造一个 SPI 消息，只发送一个字节（DUMMY），并接收一个字节。

记录起始时间
用于后续判断是否超时。

轮询设备状态
通过 device->bus->ops->xfer 发送消息并接收设备返回的数据。

判断设备是否准备好
如果收到的数据等于 DUMMY，说明设备已经准备好，函数返回成功。

超时处理
如果等待超过 1000 毫秒，打印调试信息并返回超时错误。

关键点
轮询机制：通过不断发送和接收数据，判断设备状态。
超时保护：防止死循环，保证系统稳定。
SPI 通信：利用 SPI 总线的 xfer 操作进行数据交换。

用途：用于等待 SD 卡准备好，确保 SD 卡可以接收新的命令或数据。
触发条件：等待 SD 卡返回一个 DUMMY 字节（通常是 0xFF），表示 SD 卡已经准备好。
典型场景：在发送命令或数据之前，确保 SD 卡不再忙碌。
*/
static rt_err_t _wait_ready(struct rt_spi_device *device)
{//等待返回 DUMMY
    struct rt_spi_message message;
    rt_tick_t tick_start;
    uint8_t send, recv;

    tick_start = rt_tick_get();

    send = DUMMY;
    /* initial message */
    message.send_buf = &send;
    message.recv_buf = &recv;
    message.length = 1;
    message.cs_take = message.cs_release = 0;

    while (1)
    {
        /* transfer message */
        device->bus->ops->xfer(device, &message);

        if (recv == DUMMY)
        {
            return RT_EOK;
        }

        if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(1000)))
        {
            MSD_DEBUG("[err] wait ready timeout!\r\n");
            return -RT_ETIMEOUT;
        }
    }
}

//读单个块 存到buffer中
static rt_err_t _read_block(struct rt_spi_device *device, void *buffer, uint32_t block_size)
{
    struct rt_spi_message message;
    rt_err_t result;

    /* wati token // 等待 S 卡准备好*/
    result = _wait_token(device, MSD_TOKEN_READ_START);
    if (result != RT_EOK)
    {
        return result;
    }

    /* read data */
    {
        /* initial message */
        message.send_buf = RT_NULL;
        message.recv_buf = buffer;
        message.length = block_size;
        message.cs_take = message.cs_release = 0;

        /* transfer message  // 通过 SPI 接口读取数据*/
        device->bus->ops->xfer(device, &message);
    } /* read data */

    /* get crc */
    {
        uint8_t recv_buffer[2];

        /* initial message */
        message.send_buf = RT_NULL;
        message.recv_buf = recv_buffer;
        message.length = 2;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
    } /* get crc */

    return RT_EOK;
}
//写单个块 存到buffer中
static rt_err_t _write_block(struct rt_spi_device *device, const void *buffer, uint32_t block_size, uint8_t token)
{
    struct rt_spi_message message;
    uint8_t send_buffer[16];

    rt_memset(send_buffer, DUMMY, sizeof(send_buffer));
    send_buffer[sizeof(send_buffer) - 1] = token;

    /* send start block token */
    {
        /* initial message */
        message.send_buf = send_buffer;
        message.recv_buf = RT_NULL;
        message.length = sizeof(send_buffer);
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
    }

    /* send data */
    {
        /* initial message */
        message.send_buf = buffer;
        message.recv_buf = RT_NULL;
        message.length = block_size;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
    }

    /* put crc and get data response */
    {
        uint8_t recv_buffer[3];
        uint8_t response;

        /* initial message */
        message.send_buf = send_buffer;
        message.recv_buf = recv_buffer;
        message.length = sizeof(recv_buffer);
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);

//        response = 0x0E & recv_buffer[2];
        response = MSD_GET_DATA_RESPONSE(recv_buffer[2]);
        if (response != MSD_DATA_OK)
        {
            MSD_DEBUG("[err] write block fail! data response : 0x%02X\r\n", response);
            return -RT_ERROR;
        }
    }

    /* wati ready */
    return _wait_ready(device);
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops msd_ops =
{
    rt_msd_init,
    rt_msd_open,
    rt_msd_close,
    rt_msd_read,
    rt_msd_write,
    rt_msd_control
};

const static struct rt_device_ops msd_sdhc_ops =
{
    rt_msd_init,
    rt_msd_open,
    rt_msd_close,
    rt_msd_sdhc_read,
    rt_msd_sdhc_write,
    rt_msd_control
};
#endif

/* RT-Thread Device Driver Interface */
/*  SD卡设备的init    yinjiao duiying    kuaishebei  spi  读写 sd
这个时候才会真正执行 rt_msd_init() 的全部初始化流程（例如识别 SD 卡、初始化 SPI 协议、建立扇区信息）。
*/
static rt_err_t rt_msd_init(rt_device_t dev) //定义 SD 卡块设备的初始化函数，参数是 RT-Thread 设备指针。
{
    struct msd_device *msd = (struct msd_device *)dev; //将传入的设备指针强制类型转换为 MSD 设备结构体指针，方便后续访问。
    uint8_t response[MSD_RESPONSE_MAX_LEN];//定义一个数组用于存放 SD 卡命令响应数据。
    rt_err_t result = RT_EOK;//定义并初始化返回值变量，初始为成功。
    rt_tick_t tick_start; //定义变量用于计时
    uint32_t OCR;  //（操作电压范围）寄存器值。
    //调用init时已经 有spi_device了 如果 SPI 设备指针为空，打印错误信息并返回错误码。
    if (msd->spi_device == RT_NULL)
    {
        MSD_DEBUG("[err] the SPI SD device has no SPI!\r\n");
        return -RT_EIO;
    }

    /* config spi   配置spi  刚开是的时候只是注册了一个空的spi_device */ 
    { //配置 SPI 设备为 8 位数据宽度、模式 0、最高速率 400kHz，用于 SD 卡初始化。
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
        cfg.max_hz = 1000 * 400; /* 400kbit/s */
        rt_spi_configure(msd->spi_device, &cfg);  //SPI10 配置SPI设备
    } /* config spi */

    /* init SD card  声明 SPI 消息结构体，尝试获取 SPI 总线所有权，失败则跳转到退出，成功后释放 SPI 设备。*/
    {
        struct rt_spi_message message;

        result = MSD_take_owner(msd->spi_device);//spi10  尝试获取 SPI 总线所有权

        if (result != RT_EOK)
        {
            goto _exit;
        }

        rt_spi_release(msd->spi_device); //拉高篇选

        /* The host shall supply power to the card so that the voltage is reached to Vdd_min within 250ms and
           start to supply at least 74 SD clocks to the SD card with keeping CMD line to high.
           In case of SPI mode, CS shall be held to high during 74 clock cycles.
            /* 发送 74 个时钟周期，保证 SD 卡进入 SPI 模式 */
            
        {
            uint8_t send_buffer[100]; /* 100byte > 74 clock   /* 100字节 > 74时钟 */ 

            /* initial message */
            rt_memset(send_buffer, DUMMY, sizeof(send_buffer));
            message.send_buf = send_buffer;
            message.recv_buf = RT_NULL;
            message.length = sizeof(send_buffer);
            message.cs_take = message.cs_release = 0;

            /* transfer message   sd发送数据利用的是bus总线下面的  xfer函数*/
            msd->spi_device->bus->ops->xfer(msd->spi_device, &message);
        } /* send 74 clock 发送 100 字节的 DUMMY 数据，确保 SD 卡进入 SPI 模式。*/

        /* Send CMD0 (GO_IDLE_STATE) to put MSD in SPI mode 发送 CMD0，令 SD 卡进入 IDLE 状态  */
        {
            tick_start = rt_tick_get();

            while (1)  //循环发送 CMD0 命令，直到 SD 卡进入 IDLE 状态或超时。
            {
                rt_spi_take(msd->spi_device);
                result = _send_cmd(msd->spi_device, GO_IDLE_STATE, 0x00, 0x95, response_r1, response);
                rt_spi_release(msd->spi_device);

                if ((result == RT_EOK) && (response[0] == MSD_IN_IDLE_STATE))
                {
                    break;
                }

                if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(CARD_TRY_TIMES)))
                {
                    MSD_DEBUG("[err] SD card goto IDLE mode timeout!\r\n");
                    result = -RT_ETIMEOUT;
                    goto _exit;
                }
            }

            MSD_DEBUG("[info] SD card goto IDLE mode OK!\r\n");
        } /* Send CMD0 (GO_IDLE_STATE) to put MSD in SPI mode */

        /* CMD8  发送 CMD8，检测 SD 卡版本和电压兼容性 */
        {
            tick_start = rt_tick_get();

            do
            {
                rt_spi_take(msd->spi_device);
                result = _send_cmd(msd->spi_device, SEND_IF_COND, 0x01AA, 0x87, response_r7, response);
                rt_spi_release(msd->spi_device);

                if (result == RT_EOK)
                {
                    MSD_DEBUG("[info] CMD8 response : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n",
                              response[0], response[1], response[2], response[3], response[4]);
                    //这段代码有问题   并没有检测到其他类型的SD卡
                    if (response[0] & (1 << 2))
                    {
                        /* illegal command, SD V1.x or MMC card */
                        MSD_DEBUG("[info] CMD8 is illegal command.\r\n");
                        MSD_DEBUG("[info] maybe Ver1.X SD Memory Card or MMC card!\r\n");
                        msd->card_type = MSD_CARD_TYPE_SD_V1_X;
                        break;
                    }
                    else
                    {
                        /* SD V2.0 or later or SDHC or SDXC memory card! */
                        MSD_DEBUG("[info] Ver2.00 or later or SDHC or SDXC memory card!\r\n");
                        msd->card_type = MSD_CARD_TYPE_SD_V2_X;
                    }

                    if ((0xAA == response[4]) && (0x00 == response[3]))
                    {
                        /* SD2.0 not support current voltage */
                        MSD_DEBUG("[err] VCA = 0, SD2.0 not surpport current operation voltage range\r\n");
                        result = -RT_ERROR;
                        goto _exit;
                    }
                }
                else
                {
                    if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(200)))
                    {
                        MSD_DEBUG("[err] CMD8 SEND_IF_COND timeout!\r\n");
                        result = -RT_ETIMEOUT;
                        goto _exit;
                    }
                }
            }
            while (0xAA != response[4]);
        } /* CMD8   发送 CMD8 命令，判断卡类型（V1.x 或 V2.x），并检测电压兼容性。*/

        /* Ver1.X SD Memory Card or MMC card 根据卡类型进一步初始化 SD 卡或 MMC 卡 */
        if (msd->card_type == MSD_CARD_TYPE_SD_V1_X)
        {
            rt_bool_t is_sd_v1_x = RT_FALSE;
            rt_tick_t tick_start;

            /* try SD Ver1.x */
            while (1)
            {
                rt_spi_take(msd->spi_device);

                result = _send_cmd(msd->spi_device, READ_OCR, 0x00, 0x00, response_r3, response);
                if (result != RT_EOK)
                {
                    rt_spi_release(msd->spi_device);
                    MSD_DEBUG("[info] It maybe SD1.x or MMC But it is Not response to CMD58!\r\n");
                    goto _exit;
                }

                if (0 != (response[0] & 0xFE))
                {
                    rt_spi_release(msd->spi_device);
                    MSD_DEBUG("[info] It look CMD58 as illegal command so it is not SD card!\r\n");
                    break;
                }
                rt_spi_release(msd->spi_device);

                OCR = response[1];
                OCR = (OCR << 8) + response[2];
                OCR = (OCR << 8) + response[3];
                OCR = (OCR << 8) + response[4];
                MSD_DEBUG("[info] OCR is 0x%08X\r\n", OCR);

                if (0 == (OCR & (0x1 << 15)))
                {
                    MSD_DEBUG(("[err] SD 1.x But not surpport current voltage\r\n"));
                    result = -RT_ERROR;
                    goto _exit;
                }

                /* --Send ACMD41 to make card ready */
                tick_start = rt_tick_get();

                /* try CMD55 + ACMD41 */
                while (1)
                {
                    if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(CARD_TRY_TIMES_ACMD41)))
                    {
                        rt_spi_release(msd->spi_device);
                        MSD_DEBUG("[info] try CMD55 + ACMD41 timeout! mabey MMC card!\r\n");
                        break;
                    }

                    rt_spi_take(msd->spi_device);

                    /* CMD55 APP_CMD */
                    result = _send_cmd(msd->spi_device, APP_CMD, 0x00, 0x00, response_r1, response);
                    if (result != RT_EOK)
                    {
                        rt_spi_release(msd->spi_device);
                        continue;
                    }

                    if (0 != (response[0] & 0xFE))
                    {
                        rt_spi_release(msd->spi_device);
                        MSD_DEBUG("[info] Not SD card2 , may be MMC\r\n");
                        break;
                    }

                    /* ACMD41 SD_SEND_OP_COND */
                    result = _send_cmd(msd->spi_device, SD_SEND_OP_COND, 0x00, 0x00, response_r1, response);
                    if (result != RT_EOK)
                    {
                        rt_spi_release(msd->spi_device);
                        continue;
                    }

                    if (0 != (response[0] & 0xFE))
                    {
                        rt_spi_release(msd->spi_device);
                        MSD_DEBUG("[info] Not SD card4 , may be MMC\r\n");
                        break;
                    }

                    if (0 == (response[0] & 0xFF))
                    {
                        rt_spi_release(msd->spi_device);
                        is_sd_v1_x = RT_TRUE;
                        MSD_DEBUG("[info] It is Ver1.X SD Memory Card!!!\r\n");
                        break;
                    }
                } /* try CMD55 + ACMD41 */

                break;
            } /* try SD Ver1.x */

            /* try MMC */
            if (is_sd_v1_x != RT_TRUE)
            {
                uint32_t i;

                MSD_DEBUG("[info] try MMC card!\r\n");
                rt_spi_release(msd->spi_device);

                /* send dummy clock */
                {
                    uint8_t send_buffer[100];

                    /* initial message */
                    rt_memset(send_buffer, DUMMY, sizeof(send_buffer));
                    message.send_buf = send_buffer;
                    message.recv_buf = RT_NULL;
                    message.length = sizeof(send_buffer);
                    message.cs_take = message.cs_release = 0;

                    for (i = 0; i < 10; i++)
                    {
                        /* transfer message */
                        msd->spi_device->bus->ops->xfer(msd->spi_device, &message);
                    }
                } /* send dummy clock */

                /* send CMD0 goto IDLE state */
                tick_start = rt_tick_get();
                while (1)
                {
                    rt_spi_take(msd->spi_device);
                    result = _send_cmd(msd->spi_device, GO_IDLE_STATE, 0x00, 0x95, response_r1, response);
                    rt_spi_release(msd->spi_device);

                    if ((result == RT_EOK) && (response[0] == MSD_IN_IDLE_STATE))
                    {
                        break;
                    }

                    if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(CARD_TRY_TIMES)))
                    {
                        MSD_DEBUG("[err] SD card goto IDLE mode timeout!\r\n");
                        result = -RT_ETIMEOUT;
                        goto _exit;
                    }
                } /* send CMD0 goto IDLE stat */

                /* send CMD1 */
                tick_start = rt_tick_get();
                while (1)
                {
                    rt_spi_take(msd->spi_device);
                    result = _send_cmd(msd->spi_device, SEND_OP_COND, 0x00, 0x00, response_r1, response);
                    rt_spi_release(msd->spi_device);

                    if ((result == RT_EOK) && (response[0] == MSD_RESPONSE_NO_ERROR))
                    {
                        MSD_DEBUG("[info] It is MMC card!!!\r\n");
                        msd->card_type = MSD_CARD_TYPE_MMC;
                        break;
                    }

                    if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(CARD_TRY_TIMES)))
                    {
                        MSD_DEBUG("[err] SD card goto IDLE mode timeout!\r\n");
                        result = -RT_ETIMEOUT;
                        goto _exit;
                    }
                } /* send CMD1 */
            } /* try MMC */
        }
        else if (msd->card_type == MSD_CARD_TYPE_SD_V2_X)
        {
            rt_spi_take(msd->spi_device);//拉底片选

            result = _send_cmd(msd->spi_device, READ_OCR, 0x00, 0x00, response_r3, response);
            if (result != RT_EOK)
            {
                rt_spi_release(msd->spi_device);
                MSD_DEBUG("[err] It maybe SD2.0 But it is Not response to CMD58!\r\n");
                goto _exit;
            }

            if ((response[0] & 0xFE) != 0)
            {
                rt_spi_release(msd->spi_device);
                MSD_DEBUG("[err] It look CMD58 as illegal command so it is not SD card!\r\n");
                result = -RT_ERROR;
                goto _exit;
            }

            rt_spi_release(msd->spi_device);//拉高片选

            OCR = response[1];
            OCR = (OCR << 8) + response[2];
            OCR = (OCR << 8) + response[3];
            OCR = (OCR << 8) + response[4];
            MSD_DEBUG("[info] OCR is 0x%08X\r\n", OCR);

            if (0 == (OCR & (0x1 << 15)))
            {
                MSD_DEBUG(("[err] SD 1.x But not surpport current voltage\r\n"));
                result = -RT_ERROR;
                goto _exit;
            }

            /* --Send ACMD41 to make card ready */
            tick_start = rt_tick_get();

            /* try CMD55 + ACMD41 */
            do
            {
                rt_spi_take(msd->spi_device);
                if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(CARD_TRY_TIMES_ACMD41)))
                {
                    rt_spi_release(msd->spi_device);
                    MSD_DEBUG("[err] SD Ver2.x or later try CMD55 + ACMD41 timeout!\r\n");
                    result = -RT_ERROR;
                    goto _exit;
                }

                /* CMD55 APP_CMD */
                result = _send_cmd(msd->spi_device, APP_CMD, 0x00, 0x65, response_r1, response);
//                if((result != RT_EOK) || (response[0] == 0x01))
                if (result != RT_EOK)
                {
                    rt_spi_release(msd->spi_device);
                    continue;
                }

                if ((response[0] & 0xFE) != 0)
                {
                    rt_spi_release(msd->spi_device);
                    MSD_DEBUG("[err] Not SD ready!\r\n");
                    result = -RT_ERROR;
                    goto _exit;
                }

                /* ACMD41 SD_SEND_OP_COND */
                result = _send_cmd(msd->spi_device, SD_SEND_OP_COND, 0x40000000, 0x77, response_r1, response);
                if (result != RT_EOK)
                {
                    rt_spi_release(msd->spi_device);
                    MSD_DEBUG("[err] ACMD41 fail!\r\n");
                    result = -RT_ERROR;
                    goto _exit;
                }

                if ((response[0] & 0xFE) != 0)
                {
                    rt_spi_release(msd->spi_device);
                    MSD_DEBUG("[info] Not SD card4 , response : 0x%02X\r\n", response[0]);
//                    break;
                }
            }
            while (response[0] != MSD_RESPONSE_NO_ERROR);
            rt_spi_release(msd->spi_device);
            /* try CMD55 + ACMD41 */

            /* --Read OCR again */
            rt_spi_take(msd->spi_device);
            result = _send_cmd(msd->spi_device, READ_OCR, 0x00, 0x00, response_r3, response);
            if (result != RT_EOK)
            {
                rt_spi_release(msd->spi_device);
                MSD_DEBUG("[err] It maybe SD2.0 But it is Not response to 2nd CMD58!\r\n");
                goto _exit;
            }

            if ((response[0] & 0xFE) != 0)
            {
                rt_spi_release(msd->spi_device);
                MSD_DEBUG("[err] It look 2nd CMD58 as illegal command so it is not SD card!\r\n");
                result = -RT_ERROR;
                goto _exit;
            }
            rt_spi_release(msd->spi_device);

            OCR = response[1];
            OCR = (OCR << 8) + response[2];
            OCR = (OCR << 8) + response[3];
            OCR = (OCR << 8) + response[4];
            MSD_DEBUG("[info] OCR 2nd read is 0x%08X\r\n", OCR);

            if ((OCR & 0x40000000) != 0)
            {
                MSD_DEBUG("[info] It is SD2.0 SDHC Card!!!\r\n");
                msd->card_type = MSD_CARD_TYPE_SD_SDHC;
            }
            else
            {
                MSD_DEBUG("[info] It is SD2.0 standard capacity Card!!!\r\n");
            }
        } /* MSD_CARD_TYPE_SD_V2_X */
        else
        {
            MSD_DEBUG("[err] SD card type unkonw!\r\n");
            result = -RT_ERROR;
            goto _exit;
        }
    } /* init SD card */
    ///* 根据卡类型设置读写函数指针 */
    if (msd->card_type == MSD_CARD_TYPE_SD_SDHC)
    {
#ifdef RT_USING_DEVICE_OPS
        dev->ops   = &msd_sdhc_ops;
#else
        dev->read  = rt_msd_sdhc_read;// 如果我的SD卡选的是32G的SD卡  那么就会调用这个函数
        dev->write = rt_msd_sdhc_write;// 如果我的SD卡选的是32G的SD卡  那么就会调用这个函数
#endif
    }
    else
    {
#ifdef RT_USING_DEVICE_OPS
        dev->ops   = &msd_ops;//两种操作接口
        /*
        在 msd_init 函数中，_msd_device.parent.read 和 write 被设置为 RT_NULL，而不是直接设置为 rt_msd_read 和 rt_msd_write，这是因为 RT-Thread 提供了两种设备操作接口的实现方式：传统接口 和 设备操作表（ops）接口。具体原因如下
        */
#else
        dev->read  = rt_msd_read;
        dev->write = rt_msd_write;
#endif
    }

    /* set CRC */
    {
        rt_spi_release(msd->spi_device);
        rt_spi_take(msd->spi_device);
#ifdef MSD_USE_CRC
        result = _send_cmd(msd->spi_device, CRC_ON_OFF, 0x01, 0x83, response_r1, response);
#else
        result = _send_cmd(msd->spi_device, CRC_ON_OFF, 0x00, 0x91, response_r1, response);
#endif
        rt_spi_release(msd->spi_device);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] CMD59 CRC_ON_OFF fail! response : 0x%02X\r\n", response[0]);
            result = -RT_ERROR;
            goto _exit;
        }
    } /* set CRC */

    /* CMD16 SET_BLOCKLEN  扇区大小在初始化时通过 CMD16 命令设置为 SECTOR_SIZE，通常为 512 字节：*/
    {
        rt_spi_release(msd->spi_device);
        rt_spi_take(msd->spi_device);
        result = _send_cmd(msd->spi_device, SET_BLOCKLEN, SECTOR_SIZE, 0x00, response_r1, response);
        rt_spi_release(msd->spi_device);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] CMD16 SET_BLOCKLEN fail! response : 0x%02X\r\n", response[0]);
            result = -RT_ERROR;
            goto _exit;
        }
        msd->geometry.block_size = SECTOR_SIZE;
        msd->geometry.bytes_per_sector = SECTOR_SIZE;
    }

    /* read CSD */
    {
        uint8_t CSD_buffer[MSD_CSD_LEN];

        rt_spi_take(msd->spi_device);
//        result = _send_cmd(msd->spi_device, SEND_CSD, 0x00, 0xAF, response_r1, response);
        result = _send_cmd(msd->spi_device, SEND_CSD, 0x00, 0x00, response_r1, response);
        //_read_block 和 _write_block 函数负责实际的数据传输，但它们并不直接指定扇区号。扇区号已经在调用这些函数之前，通过 _send_cmd 函数发送给了 SD 卡。
        if (result != RT_EOK)
        {
            rt_spi_release(msd->spi_device);
            MSD_DEBUG("[err] CMD9 SEND_CSD timeout!\r\n");
            goto _exit;
        }

        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            rt_spi_release(msd->spi_device);
            MSD_DEBUG("[err] CMD9 SEND_CSD fail! response : 0x%02X\r\n", response[0]);
            result = -RT_ERROR;
            goto _exit;
        }

        result = _read_block(msd->spi_device, CSD_buffer, MSD_CSD_LEN);
        rt_spi_release(msd->spi_device);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[err] read CSD fail!\r\n");
            goto _exit;
        }

        /* Analyze CSD */
        {
            uint8_t  CSD_STRUCTURE;
            uint32_t C_SIZE;
            uint32_t card_capacity;

            uint8_t  tmp8;
            uint16_t tmp16;
            uint32_t tmp32;

            /* get CSD_STRUCTURE */
            tmp8 = CSD_buffer[0] & 0xC0; /* 0b11000000 */
            CSD_STRUCTURE = tmp8 >> 6;

            /* MMC CSD Analyze. */
            if (msd->card_type == MSD_CARD_TYPE_MMC)
            {
                uint8_t C_SIZE_MULT;
                uint8_t READ_BL_LEN;

                if (CSD_STRUCTURE > 2)
                {
                    MSD_DEBUG("[err] bad CSD Version : %d\r\n", CSD_STRUCTURE);
                    result = -RT_ERROR;
                    goto _exit;
                }

                if (CSD_STRUCTURE == 0)
                {
                    MSD_DEBUG("[info] CSD version No. 1.0\r\n");
                }
                else if (CSD_STRUCTURE == 1)
                {
                    MSD_DEBUG("[info] CSD version No. 1.1\r\n");
                }
                else if (CSD_STRUCTURE == 2)
                {
                    MSD_DEBUG("[info] CSD version No. 1.2\r\n");
                }

                /* get TRAN_SPEED 8bit [103:96] */
                tmp8 = CSD_buffer[3];
                tmp8 &= 0x03; /* [2:0] transfer rate unit.*/
                if (tmp8 == 0)
                {
                    msd->max_clock = 100 * 1000; /* 0=100kbit/s. */
                }
                else if (tmp8 == 1)
                {
                    msd->max_clock = 1 * 1000 * 1000; /* 1=1Mbit/s. */
                }
                else if (tmp8 == 2)
                {
                    msd->max_clock = 10 * 1000 * 1000; /* 2=10Mbit/s. */
                }
                else if (tmp8 == 3)
                {
                    msd->max_clock = 100 * 1000 * 1000; /* 3=100Mbit/s. */
                }
                if (tmp8 == 0)
                {
                    MSD_DEBUG("[info] TRAN_SPEED: 0x%02X, %dkbit/s.\r\n", tmp8, msd->max_clock / 1000);
                }
                else
                {
                    MSD_DEBUG("[info] TRAN_SPEED: 0x%02X, %dMbit/s.\r\n", tmp8, msd->max_clock / 1000 / 1000);
                }

                /* get READ_BL_LEN 4bit [83:80] */
                tmp8 = CSD_buffer[5] & 0x0F; /* 0b00001111; */
                READ_BL_LEN = tmp8;          /* 4 bit */
                MSD_DEBUG("[info] CSD : READ_BL_LEN : %d %dbyte\r\n", READ_BL_LEN, (1 << READ_BL_LEN));

                /* get C_SIZE 12bit [73:62] */
                tmp16 = CSD_buffer[6] & 0x03; /* get [73:72] 0b00000011 */
                tmp16 = tmp16 << 8;
                tmp16 += CSD_buffer[7];       /* get [71:64] */
                tmp16 = tmp16 << 2;
                tmp8 = CSD_buffer[8] & 0xC0;  /* get [63:62] 0b11000000 */
                tmp8 = tmp8 >> 6;
                tmp16 = tmp16 + tmp8;
                C_SIZE = tmp16;             //12 bit
                MSD_DEBUG("[info] CSD : C_SIZE : %d\r\n", C_SIZE);

                /* get C_SIZE_MULT 3bit [49:47] */
                tmp8 = CSD_buffer[9] & 0x03;//0b00000011;
                tmp8 = tmp8 << 1;
                tmp8 = tmp8 + ((CSD_buffer[10] & 0x80/*0b10000000*/) >> 7);
                C_SIZE_MULT = tmp8;         // 3 bit
                MSD_DEBUG("[info] CSD : C_SIZE_MULT : %d\r\n", C_SIZE_MULT);

                /* memory capacity = BLOCKNR * BLOCK_LEN */
                /* BLOCKNR = (C_SIZE+1) * MULT */
                /* MULT = 2^(C_SIZE_MULT+2) */
                /* BLOCK_LEN = 2^READ_BL_LEN */
                card_capacity = (1 << READ_BL_LEN) * ((C_SIZE + 1) * (1 << (C_SIZE_MULT + 2)));
                msd->geometry.sector_count = card_capacity / msd->geometry.bytes_per_sector;
                MSD_DEBUG("[info] card capacity : %d Mbyte\r\n", card_capacity / (1024 * 1024));
            }
            else /* SD CSD Analyze. */
            {
                if (CSD_STRUCTURE == 0)
                {
                    uint8_t C_SIZE_MULT;
                    uint8_t READ_BL_LEN;

                    MSD_DEBUG("[info] CSD Version 1.0\r\n");

                    /* get TRAN_SPEED 8bit [103:96] */
                    tmp8 = CSD_buffer[3];
                    if (tmp8 == 0x32)
                    {
                        msd->max_clock = 1000 * 1000 * 10; /* 10Mbit/s. */
                    }
                    else if (tmp8 == 0x5A)
                    {
                        msd->max_clock = 1000 * 1000 * 50; /* 50Mbit/s. */
                    }
                    else
                    {
                        msd->max_clock = 1000 * 1000 * 1; /* 1Mbit/s default. */
                    }
                    MSD_DEBUG("[info] TRAN_SPEED: 0x%02X, %dMbit/s.\r\n", tmp8, msd->max_clock / 1000 / 1000);

                    /* get READ_BL_LEN 4bit [83:80] */
                    tmp8 = CSD_buffer[5] & 0x0F; /* 0b00001111; */
                    READ_BL_LEN = tmp8;          /* 4 bit */
                    MSD_DEBUG("[info] CSD : READ_BL_LEN : %d %dbyte\r\n", READ_BL_LEN, (1 << READ_BL_LEN));

                    /* get C_SIZE 12bit [73:62] */
                    tmp16 = CSD_buffer[6] & 0x03; /* get [73:72] 0b00000011 */
                    tmp16 = tmp16 << 8;
                    tmp16 += CSD_buffer[7];       /* get [71:64] */
                    tmp16 = tmp16 << 2;
                    tmp8 = CSD_buffer[8] & 0xC0;  /* get [63:62] 0b11000000 */
                    tmp8 = tmp8 >> 6;
                    tmp16 = tmp16 + tmp8;
                    C_SIZE = tmp16;             //12 bit
                    MSD_DEBUG("[info] CSD : C_SIZE : %d\r\n", C_SIZE);

                    /* get C_SIZE_MULT 3bit [49:47] */
                    tmp8 = CSD_buffer[9] & 0x03;//0b00000011;
                    tmp8 = tmp8 << 1;
                    tmp8 = tmp8 + ((CSD_buffer[10] & 0x80/*0b10000000*/) >> 7);
                    C_SIZE_MULT = tmp8;         // 3 bit
                    MSD_DEBUG("[info] CSD : C_SIZE_MULT : %d\r\n", C_SIZE_MULT);

                    /* memory capacity = BLOCKNR * BLOCK_LEN */
                    /* BLOCKNR = (C_SIZE+1) * MULT */
                    /* MULT = 2^(C_SIZE_MULT+2) */
                    /* BLOCK_LEN = 2^READ_BL_LEN */
                    card_capacity = (1 << READ_BL_LEN) * ((C_SIZE + 1) * (1 << (C_SIZE_MULT + 2)));
                    msd->geometry.sector_count = card_capacity / msd->geometry.bytes_per_sector;
                    MSD_DEBUG("[info] card capacity : %d Mbyte\r\n", card_capacity / (1024 * 1024));
                }
                else if (CSD_STRUCTURE == 1)
                {
                    MSD_DEBUG("[info] CSD Version 2.0\r\n");

                    /* get TRAN_SPEED 8bit [103:96] */
                    tmp8 = CSD_buffer[3];
                    if (tmp8 == 0x32)
                    {
                        msd->max_clock = 1000 * 1000 * 10; /* 10Mbit/s. */
                    }
                    else if (tmp8 == 0x5A)
                    {
                        msd->max_clock = 1000 * 1000 * 50; /* 50Mbit/s. */  // 320 * 240 * 16 =40.69   
                    }
                    else if (tmp8 == 0x0B)
                    {
                        msd->max_clock = 1000 * 1000 * 100; /* 100Mbit/s. */
                        /* UHS50 Card sets TRAN_SPEED to 0Bh (100Mbit/sec), */
                        /* for both SDR50 and DDR50 modes. */
                    }
                    else if (tmp8 == 0x2B)
                    {
                        msd->max_clock = 1000 * 1000 * 200; /* 200Mbit/s. */
                        /* UHS104 Card sets TRAN_SPEED to 2Bh (200Mbit/sec). */
                    }
                    else
                    {
                        msd->max_clock = 1000 * 1000 * 1; /* 1Mbit/s default. */
                    }
                    MSD_DEBUG("[info] TRAN_SPEED: 0x%02X, %dMbit/s.\r\n", tmp8, msd->max_clock / 1000 / 1000);

                    /* get C_SIZE 22bit [69:48] */
                    tmp32 = CSD_buffer[7] & 0x3F; /* 0b00111111 */
                    tmp32 = tmp32 << 8;
                    tmp32 += CSD_buffer[8];
                    tmp32 = tmp32 << 8;
                    tmp32 += CSD_buffer[9];
                    C_SIZE = tmp32;
                    MSD_DEBUG("[info] CSD : C_SIZE : %d\r\n", C_SIZE);

                    /* memory capacity = (C_SIZE+1) * 512K byte */
                    card_capacity = (C_SIZE + 1) / 2; /* unit : Mbyte */
                    msd->geometry.sector_count = (C_SIZE + 1) * 1024; /* 512KB = 1024sector */
                    MSD_DEBUG("[info] card capacity : %d.%d Gbyte\r\n", card_capacity / 1024, (card_capacity % 1024) * 100 / 1024);
                    MSD_DEBUG("[info] sector_count : %d\r\n", msd->geometry.sector_count);
                }
                else
                {
                    MSD_DEBUG("[err] bad CSD Version : %d\r\n", CSD_STRUCTURE);
                    result = -RT_ERROR;
                    goto _exit;
                }
            } /* SD CSD Analyze. */
        } /* Analyze CSD */

    } /* read CSD */

    /* config spi to high speed */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
        cfg.max_hz = msd->max_clock;
        rt_spi_configure(msd->spi_device, &cfg);
    } /* config spi */

_exit:
    rt_spi_release(msd->spi_device);
    rt_mutex_release(&(msd->spi_device->bus->lock));
    return result;
}

static rt_err_t rt_msd_open(rt_device_t dev, rt_uint16_t oflag)
{
//    struct msd_device * msd = (struct msd_device *)dev;
    return RT_EOK;
}

static rt_err_t rt_msd_close(rt_device_t dev)
{
//    struct msd_device * msd = (struct msd_device *)dev;
    return RT_EOK;
}

//main函数中通过fprint  fget等函数本质上会调用 MSD驱动的的read函数，然后调用rt_msd_read->根据单块还是多块调用_read_block(),多块->read_blocks()
static rt_ssize_t rt_msd_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{//pos是逻辑块号  是FATS文件系统计算出来给的
    struct msd_device *msd = (struct msd_device *)dev;
    uint8_t response[MSD_RESPONSE_MAX_LEN];
    rt_err_t result = RT_EOK;
    //在多设备共享同一个 SPI 总线时，每个设备通信前都需要确保总线已按自己的参数配置，并且自己是 owner，防止配置冲突。
    result = MSD_take_owner(msd->spi_device);

    if (result != RT_EOK)
    {
        goto _exit;
    }

    /* SINGLE_BLOCK?读单个块 */
    if (size == 1)
    {
        //拉低片选引脚
        rt_spi_take(msd->spi_device);
        //发送读单块命令   pos（逻辑块号）被直接乘以 msd->geometry.bytes_per_sector，从而转换为物理扇区的起始地址（以字节为单位）。这是因为 SD 卡的操作是以扇区为单位的，而每个扇区的大小由 msd->geometry.bytes_per_sector 指定。
        result = _send_cmd(msd->spi_device, READ_SINGLE_BLOCK, pos * msd->geometry.bytes_per_sector, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] read SINGLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
            goto _exit;
        }
        //从哪个位置开始读 ？ 
        result = _read_block(msd->spi_device, buffer, msd->geometry.bytes_per_sector);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[err] read SINGLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
        }
    }
    else if (size > 1)
    {
        uint32_t i;

        rt_spi_take(msd->spi_device);

        result = _send_cmd(msd->spi_device, READ_MULTIPLE_BLOCK, pos * msd->geometry.bytes_per_sector, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] read READ_MULTIPLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
            goto _exit;
        }

        for (i = 0; i < size; i++)
        {
            result = _read_block(msd->spi_device,
                                 (uint8_t *)buffer + msd->geometry.bytes_per_sector * i,
                                 msd->geometry.bytes_per_sector);
            if (result != RT_EOK)
            {
                MSD_DEBUG("[err] read READ_MULTIPLE_BLOCK #%d fail!\r\n", pos);
                size = i;
                break;
            }
        }

        /* send CMD12 stop transfer */
        result = _send_cmd(msd->spi_device, STOP_TRANSMISSION, 0x00, 0x00, response_r1b, response);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[err] read READ_MULTIPLE_BLOCK, send stop token fail!\r\n");
        }
    } /* READ_MULTIPLE_BLOCK */

_exit:
    /* release and exit */
    rt_spi_release(msd->spi_device);
    rt_mutex_release(&(msd->spi_device->bus->lock));

    return size;
}

//高速卡read  write  2G~32G 它用于通过 SPI 方式从 SDHC 卡读取数据块。下面是逐行解释：参数分别为设备指针、起始块号、数据缓冲区和要读取的块数，返回实际读取的块数。
static rt_ssize_t rt_msd_sdhc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    //将设备指针强制类型转换为 MSD 设备结构体指针，方便后续访问设备信息。
    struct msd_device *msd = (struct msd_device *)dev;
    //定义用于存放 SD 卡响应的缓冲区和操作结果变量。
    uint8_t response[MSD_RESPONSE_MAX_LEN];
    rt_err_t result = RT_EOK;
    //在多设备共享同一个 SPI 总线时，每个设备通信前都需要确保总线已按自己的参数配置，并且自己是 owner，防止配置冲突。
    result = MSD_take_owner(msd->spi_device);

    if (result != RT_EOK)
    {
        goto _exit;
    }

    /* SINGLE_BLOCK? 如果只读一个块，先拉低片选，发送读单块命令，检查响应是否正常，然后读取数据块到缓冲区。如果失败则 size 置为 0。*/
    if (size == 1)
    {
        rt_spi_take(msd->spi_device);  //rt_spi_take 用于开始一次 SPI 设备的通信前，拉低片选信号，确保后续的数据传输针对目标设备。

        result = _send_cmd(msd->spi_device, READ_SINGLE_BLOCK, pos, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] read SINGLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
            goto _exit;
        }
        //然后读取数据块到缓冲区。
        result = _read_block(msd->spi_device, buffer, msd->geometry.bytes_per_sector);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[err] read SINGLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
        }
    }
    //如果读的块数大于1，则需要使用多块读取。
    else if (size > 1)
    {
        uint32_t i;
        //拉低片选
        rt_spi_take(msd->spi_device);
        //发送读多块的指令
        result = _send_cmd(msd->spi_device, READ_MULTIPLE_BLOCK, pos, 0x00, response_r1, response);
        //检查响应。
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] read READ_MULTIPLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
            goto _exit;
        }
        //循环读取每个块的数据到缓冲区。
        for (i = 0; i < size; i++)
        {
            result = _read_block(msd->spi_device,
                                 (uint8_t *)buffer + msd->geometry.bytes_per_sector * i,
                                 msd->geometry.bytes_per_sector);
                                 //如果某块读取失败，size 记录已成功读取的块数并跳出循环。
            if (result != RT_EOK)
            {
                MSD_DEBUG("[err] read READ_MULTIPLE_BLOCK #%d fail!\r\n", pos);
                size = i;
                break;
            }
        }

        /* send CMD12 stop transfer  最后发送停止传输命令（CMD12），结束多块读取。 */
        result = _send_cmd(msd->spi_device, STOP_TRANSMISSION, 0x00, 0x00, response_r1b, response);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[err] read READ_MULTIPLE_BLOCK, send stop token fail!\r\n");
        }
    } /* READ_MULTIPLE_BLOCK */

_exit:
    /* release and exit */
    rt_spi_release(msd->spi_device);//拉高片选引脚
    rt_mutex_release(&(msd->spi_device->bus->lock)); //释放 SPI 总线锁，允许其他设备访问。

    return size;
}

static rt_ssize_t rt_msd_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    struct msd_device *msd = (struct msd_device *)dev;
    uint8_t response[MSD_RESPONSE_MAX_LEN];
    rt_err_t result;

    result = MSD_take_owner(msd->spi_device);

    if (result != RT_EOK)
    {
        MSD_DEBUG("[err] get SPI owner fail!\r\n");
        goto _exit;
    }


    /* SINGLE_BLOCK? */
    if (size == 1)
    {
        rt_spi_take(msd->spi_device);
        result = _send_cmd(msd->spi_device, WRITE_BLOCK, pos * msd->geometry.bytes_per_sector, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] CMD WRITE_BLOCK fail!\r\n");
            size = 0;
            goto _exit;
        }

        result = _write_block(msd->spi_device, buffer, msd->geometry.bytes_per_sector, MSD_TOKEN_WRITE_SINGLE_START);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[err] write SINGLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
        }
    }
    else if (size > 1)
    {
        struct rt_spi_message message;
        uint32_t i;

        rt_spi_take(msd->spi_device);

#ifdef MSD_USE_PRE_ERASED
        if (msd->card_type != MSD_CARD_TYPE_MMC)
        {
            /* CMD55 APP_CMD */
            result = _send_cmd(msd->spi_device, APP_CMD, 0x00, 0x00, response_r1, response);
            if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
            {
                MSD_DEBUG("[err] CMD55 APP_CMD fail!\r\n");
                size = 0;
                goto _exit;
            }

            /* ACMD23 Pre-erased */
            result = _send_cmd(msd->spi_device, SET_WR_BLK_ERASE_COUNT, size, 0x00, response_r1, response);
            if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
            {
                MSD_DEBUG("[err] ACMD23 SET_BLOCK_COUNT fail!\r\n");
                size = 0;
                goto _exit;
            }
        }
#endif

        result = _send_cmd(msd->spi_device, WRITE_MULTIPLE_BLOCK, pos * msd->geometry.bytes_per_sector, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] CMD WRITE_MULTIPLE_BLOCK fail!\r\n");
            size = 0;
            goto _exit;
        }

        /* write all block */
        for (i = 0; i < size; i++)
        {
            result = _write_block(msd->spi_device,
                                  (const uint8_t *)buffer + msd->geometry.bytes_per_sector * i,
                                  msd->geometry.bytes_per_sector,
                                  MSD_TOKEN_WRITE_MULTIPLE_START);
            if (result != RT_EOK)
            {
                MSD_DEBUG("[err] write SINGLE_BLOCK #%d fail!\r\n", pos);
                size = i;
                break;
            }
        } /* write all block */

        /* send stop token */
        {
            uint8_t send_buffer[18];

            rt_memset(send_buffer, DUMMY, sizeof(send_buffer));
            send_buffer[sizeof(send_buffer) - 1] = MSD_TOKEN_WRITE_MULTIPLE_STOP;

            /* initial message */
            message.send_buf = send_buffer;
            message.recv_buf = RT_NULL;
            message.length = sizeof(send_buffer);
            message.cs_take = message.cs_release = 0;

            /* transfer message */
            msd->spi_device->bus->ops->xfer(msd->spi_device, &message);
        }

        /* wait ready */
        result = _wait_ready(msd->spi_device);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[warning] wait WRITE_MULTIPLE_BLOCK stop token ready timeout!\r\n");
        }
    } /* size > 1 */

_exit:
    /* release and exit */
    rt_spi_release(msd->spi_device);
    rt_mutex_release(&(msd->spi_device->bus->lock));

    return size;
}

//高内存卡写函数   
static rt_ssize_t rt_msd_sdhc_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{//定义 SDHC 卡写函数，参数为设备指针、写入起始块号、数据缓冲区和块数量，返回实际写入的块数。
    struct msd_device *msd = (struct msd_device *)dev;
    //定义用于存放 SD 卡响应的缓冲区和操作结果变量。
    uint8_t response[MSD_RESPONSE_MAX_LEN];
    rt_err_t result;
    //获取 SPI 设备的所有权，确保在多设备共享同一 SPI 总线时，当前设备可以安全地进行通信。
    result = MSD_take_owner(msd->spi_device);

    if (result != RT_EOK)
    {
        //如果获取总线失败，则跳转到退出处理，释放资源。
        goto _exit;
    }

    /* SINGLE_BLOCK?  写一个快*/
    if (size == 1)
    {
        ////拉低引脚
        rt_spi_take(msd->spi_device); 
        //发送写单块命令
        result = _send_cmd(msd->spi_device, WRITE_BLOCK, pos, 0x00, response_r1, response);
        //检查响应是否正常
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] CMD WRITE_BLOCK fail!\r\n");
            size = 0;
            goto _exit;
        }
        //然后写入数据块
        result = _write_block(msd->spi_device, buffer, msd->geometry.bytes_per_sector, MSD_TOKEN_WRITE_SINGLE_START);
        //如果失败则 size 置为 0。
        if (result != RT_EOK)
        {
            MSD_DEBUG("[err] write SINGLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
        }
    }
    else if (size > 1) //写多个块
    {
        struct rt_spi_message message;
        uint32_t i;

        rt_spi_take(msd->spi_device); //拉低

#ifdef MSD_USE_PRE_ERASED
        /* CMD55 APP_CMD  如果定义了 MSD_USE_PRE_ERASED，先发送预擦除相关命令（CMD55 和 ACMD23），提高写入效率。失败则 size 置为 0 并退出。 */
        result = _send_cmd(msd->spi_device, APP_CMD, 0x00, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] CMD55 APP_CMD fail!\r\n");
            size = 0;
            goto _exit;
        }

        /* ACMD23 Pre-erased   如果定义了 MSD_USE_PRE_ERASED，先发送预擦除相关命令（CMD55 和 ACMD23），提高写入效率。失败则 size 置为 0 并退出。*/
        result = _send_cmd(msd->spi_device, SET_WR_BLK_ERASE_COUNT, size, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] ACMD23 SET_BLOCK_COUNT fail!\r\n");
            size = 0;
            goto _exit;
        }
#endif
        //发送写多块命令
        result = _send_cmd(msd->spi_device, WRITE_MULTIPLE_BLOCK, pos, 0x00, response_r1, response);
        //检查响应是否正常
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] CMD WRITE_MULTIPLE_BLOCK fail!\r\n");
            size = 0;
            goto _exit;
        }

        /* write all block  循环写入每个数据块到 SD 卡。如果某块写入失败，size 记录已成功写入的块数并跳出循环。*/
        for (i = 0; i < size; i++)
        {
            result = _write_block(msd->spi_device,
                                  (const uint8_t *)buffer + msd->geometry.bytes_per_sector * i,
                                  msd->geometry.bytes_per_sector,
                                  MSD_TOKEN_WRITE_MULTIPLE_START);
            if (result != RT_EOK)
            {
                MSD_DEBUG("[err] write MULTIPLE_BLOCK #%d fail!\r\n", pos);
                size = i;
                break;
            }
        } /* write all block */

        /* send stop token  写完所有块后，发送停止令牌（stop token），通知 SD 卡结束多块写入。*/
        {
            uint8_t send_buffer[18];

            rt_memset(send_buffer, DUMMY, sizeof(send_buffer));
            send_buffer[sizeof(send_buffer) - 1] = MSD_TOKEN_WRITE_MULTIPLE_STOP;

            /* initial message */
            message.send_buf = send_buffer;
            message.recv_buf = RT_NULL;
            message.length = sizeof(send_buffer);
            message.cs_take = message.cs_release = 0;

            /* transfer message */
            msd->spi_device->bus->ops->xfer(msd->spi_device, &message);
        }
        //等待 SD 卡写入完成并准备好，超时则打印警告。
        result = _wait_ready(msd->spi_device);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[warning] wait WRITE_MULTIPLE_BLOCK stop token ready timeout!\r\n");
        }
    } /* size > 1 */

_exit:
    /* release and exit 退出处理，释放 SPI 设备和总线锁，返回实际写入的块数。*/
    rt_spi_release(msd->spi_device);
    rt_mutex_release(&(msd->spi_device->bus->lock));

    return size;
}

//rt_msd_control 函数，它是 MSD（SPI SD卡）设备的控制接口，主要用于获取设备的几何信息。
static rt_err_t rt_msd_control(rt_device_t dev, int cmd, void *args)
{ //定义 MSD 设备的控制函数，参数为设备指针、命令类型和参数指针

    struct msd_device *msd = (struct msd_device *)dev;

    RT_ASSERT(dev != RT_NULL);//断言设备指针不为空，防止后续访问空指针导致程序崩溃。

    if (cmd == RT_DEVICE_CTRL_BLK_GETGEOME) //判断命令类型是否为获取块设备几何信息（如扇区大小、块大小、总扇区数）。
    {
        struct rt_device_blk_geometry *geometry;//声明一个块设备几何结构体指针。

        geometry = (struct rt_device_blk_geometry *)args;//将参数指针强制类型转换为块设备几何结构体指针。
        if (geometry == RT_NULL) return -RT_ERROR;
        /*
        将 MSD 设备的扇区字节数、块大小和扇区总数赋值给传入的结构体，用于外部查询设备容量和参数。
        */
        geometry->bytes_per_sector = msd->geometry.bytes_per_sector;
        geometry->block_size = msd->geometry.block_size;
        geometry->sector_count = msd->geometry.sector_count;
    }

    return RT_EOK;
}


//这个函数并没有定义 sd0所有的函数  只有调用  rt_devic_init("std0")时才会调用这个函数  而下面的函数只是说注册一个msd设备使用spi10
//具体的初始化函数还是得调用  rt_device_init(dev);这个时候才会真正执行 rt_msd_init() 的全部初始化流程（例如识别 SD 卡、初始化 SPI 协议、建立扇区信息）。
rt_err_t msd_init(const char *sd_device_name, const char *spi_device_name)
{
    rt_err_t result = RT_EOK;
    struct rt_spi_device *spi_device;//声明SPI设备指针
    /*通过SPI设备名  sd0查找SPI设备   ，并强制类型转换为 rt_spi_device */
    spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if (spi_device == RT_NULL)
    {
        MSD_DEBUG("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }
    /*将 _msd_device 结构体的所有成员清零，初始化 MSD 设备结构体。 */
    rt_memset(&_msd_device, 0, sizeof(_msd_device));
    //将找到的 SPI 设备指针赋值给 MSD 设备结构体的 spi_device 字段。
    _msd_device.spi_device = spi_device;

    /* register sdcard device  设置 MSD 设备的类型为块设备（Block Device）。*/
    _msd_device.parent.type    = RT_Device_Class_Block;
    /*初始化 MSD 设备的几何参数（扇区字节数、扇区数量、块大小）为 0*/
    _msd_device.geometry.bytes_per_sector = 0;
    _msd_device.geometry.sector_count = 0;
    _msd_device.geometry.block_size = 0;

#ifdef RT_USING_DEVICE_OPS
    _msd_device.parent.ops     = &msd_ops;
#else
/*
也就是说：你定义了一个“SPI SD 卡设备”，它自己实现了 init/open/close/control 等接口，而不是去使用 spi_device 的。
❓那为啥 spi_device 里也有 parent，却不用它的 open/read？
因为：

🧩 spi_device 是“底层通信设备”
它是 SPI 总线上的一个从设备，提供 低层通信能力；

它的 parent（也就是 rt_device）只是用来注册 SPI 总线设备，不表示存储设备。

你用它的 open/read 是发字节，不是读文件扇区。    
    _msd_device.parent.init    = rt_msd_init;
    _msd_device.parent.open    = rt_msd_open;
    _msd_device.parent.close   = rt_msd_close;
    _msd_device.parent.read    = RT_NULL;
    _msd_device.parent.write   = RT_NULL;
    _msd_device.parent.control = rt_msd_control;  
    这个parent是rt_device   
我不理解 为什么是parent中的open read等函数，rt_spi_device 中 也有parent  为什么不用它里面的open  read  等函数

_msd_device  是 spi_msd类型的设备  static struct msd_device  _msd_device;
struct msd_device
{
    struct rt_device                parent;    // // 是一个“块设备”类型  含有 rt_device(open read write parent) 
    struct rt_device_blk_geometry   geometry;    
    struct rt_spi_device *          spi_device; // SPI 总线设备（本质上也是一个 rt_device）  //也含有 rt_device（open/read/write），rt_spi_bus（config/xtransfer），config，cs_pin,user_data
    msd_card_type                   card_type;   
    uint32_t                        max_clock;   
};  
🧱 msd_device 是“块设备”
它是一种 逻辑上的块设备（磁盘）；
它内部通过 SPI 驱动控制 SD 卡，但对外表现为 /dev/sd0、/ 挂载点；
它必须实现 init/open/close/control 等函数以支持 dfs_mount、read(), write() 这样的操作。
🔍 所以它是高层块设备，必须用你 msd_device 中 parent 的函数表。
*/
    _msd_device.parent.init    = rt_msd_init;
    _msd_device.parent.open    = rt_msd_open;
    _msd_device.parent.close   = rt_msd_close;
    _msd_device.parent.read    = RT_NULL;
    _msd_device.parent.write   = RT_NULL;
    _msd_device.parent.control = rt_msd_control;
#endif

    /* no private, no callback */
    _msd_device.parent.user_data = RT_NULL;
    _msd_device.parent.rx_indicate = RT_NULL;
    _msd_device.parent.tx_complete = RT_NULL;


    /*
        //在注册过程中，rt_device_register 会将 MSD 设备的初始化函数 rt_msd_init 绑定到设备的 init 回调中。
        rt_device_register 将 MSD 设备注册到 RT-Thread 的设备管理框架中。
        此时，rt_msd_init 函数并不会立即被调用。
        rt_msd_init 会在调用 rt_device_init() 时被触发。rt_device_init() 是 RT-Thread 设备框架中用于初始化设备的函数。
    */
    result = rt_device_register(&_msd_device.parent, sd_device_name,
                                RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE);
            //设备是可移除的，比如 SD 卡、U 盘等，系统会认为它不是永久固定的设备。
            //设备是独立的，不依赖其他设备（比如不是挂载在某个父设备下），通常用于单独工作的设备。
    return result;
}
