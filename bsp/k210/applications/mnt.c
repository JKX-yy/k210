/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */
#define RT_USING_DFS_ELMFAT
#define RT_USING_SPI_MSD
#include <rtthread.h>
#if defined(RT_USING_SPI_MSD) && defined(RT_USING_DFS_ELMFAT)
#include <dev_spi_msd.h>
#include <dfs_fs.h>
#include "../drivers/dev_spi.h"
#define SD_SPI_BUS "spi1"
#define SD_DEVICE_NAME "spi10"
#define SD_CS_INDEX 0 //SPI片选索引
#define SD_CS_PIN 5 //gpiohs5
// int mnt_init(void)
// {
//     /*调用 MSD（SPI SD卡）初始化函数 msd_init，参数 "sd0" 表示注册的 SD 卡设备名，"spi10" 表示底层使用的 SPI 设备名。
// 这行代码的作用是：通过 SPI 总线 "spi10" 初始化 SD 卡，并将其注册为 RT-Thread 系统中的 "sd0" 块设备，后续可以通过 "sd0" 进行文件系统挂载和读写操作。 
// 你的理解完全正确 ✅——你现在的 mnt_init() 函数中直接调用 msd_init("sd0", "spi10")，但并没有事先创建并注册 "spi10" 这个 SPI 设备，所以在执行：rt_device_find("spi10")报错
// */

//     msd_init("sd0", "spi10");
//     if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
//     {
//         rt_kprintf("Mount \"/dev/sd0\" on \"/\"\n");
//     }
//     return 0;
// }
// INIT_ENV_EXPORT(mnt_init);
// #endif





/*修改版本*/
int  mnt_init(void)
{
    //第一步创建spi10 SPI设备  假设已经 注册了spi1总线  BSP_USING_SPI1
    
    if(rt_hw_spi_device_attach(SD_SPI_BUS, SD_DEVICE_NAME, SD_CS_INDEX,SD_CS_PIN) != RT_EOK)
    {
        rt_kprintf("Failed to attach SPI device spi10\n");
        return -RT_ERROR;
    }
    //第二部初始化 SD卡设备（名字为sd0,挂载在spi10上）
    msd_init("sd0", SD_DEVICE_NAME);
    //第三步挂载文件系统
    if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
    { //在这个过程中，RT-Thread DFS 会先调用 rt_device_find("sd0") 找设备，然后自动调用 rt_device_init(dev)，此时也会执行：
        rt_kprintf("Mount \"/dev/sd0\" on \"/\"\n");
    }
    return 0;

}
#endif
//实现  rt_hw_spi_device_attach(SD_SPI_BUS, SD_DEVICE_NAME, SD_CS_PIN) 函数



/*

rt_err_t msd_init(const char *sd_device_name, const char *spi_device_name)  创建msd卡时为什么需要两个名字，一个sd卡的名字，一个spi_device_name，这个spi_device_name是什么， msd_init("sd0", "spi10"); "spi10" 表示底层使用的 SPI 设备名。参数 "sd0" 表示注册的 SD 卡设备名，为什么不能直接挂在spi1总线上，需要一个spi0的spi_device 

*/
