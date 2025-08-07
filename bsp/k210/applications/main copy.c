/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018/09/30     Bernard      The first version
 */

// #include <rtthread.h>
// #include <stdio.h>

// int main(void)
// {
//     rt_kprintf("Hello, world\n");

//     return 0;
// }

#include <rtthread.h>
#include <stdio.h>
#include <dev_spi_msd.h>
#include <dfs_fs.h>
#include <string.h>


// 定义线程栈大小和优先级
#define THREAD_STACK_SIZE 2048
#define THREAD_PRIORITY 10
#define THREAD_TIMESLICE 10

// 定义 SD 卡挂载点
#define SD_SPI_BUS "spi1"       // SPI 总线名称
#define SD_DEVICE_NAME "spi10"  // SPI 设备名称
#define SD_CS_PIN 5             // 片选引脚
#define SD_MOUNT_POINT "/sd"    // 挂载点


// 定义全局变量
static rt_mutex_t data_mutex; // 用于线程间同步的互斥锁
static char detected_garbage[64]; // 保存检测到的垃圾类型
static char gps_location[64]; // 保存 GPS 位置信息
static rt_device_t serial;
#define SAMPLE_UART_NAME       "uart2"
static struct rt_semaphore rx_sem;
#define RX_BUF_SIZE 32



 static rt_err_t uart_callback(rt_device_t dev, rt_size_t size)
 {

     rt_sem_release(&rx_sem);

     return RT_EOK;
 }
 /*  接收uint8类型的一字节    确定好
 mode 1(事先 存好数据点在SD卡中)text.txt
 mode 2
 */

 static void uart_rx_thread(void *parameter)
{
    uint8_t rx_buf[RX_BUF_SIZE];
    int rx_len = 0;
    while (1)
    {
        // 等待接收信号
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);

        // 循环读取所有可用字节
        rx_len = 0;
        int ch;
        while ((ch = rt_device_read(serial, 0, &rx_buf[rx_len], 1)) == 1)
        {
            rx_len++;
            if (rx_len >= RX_BUF_SIZE) break;
        }
        if (rx_len > 0)
        {
            // 假设 float 数据是4字节，且APP直接发送原始float字节流
            if (rx_len >= 4)
            {
                float value;
                memcpy(&value, rx_buf, sizeof(float));
                rt_kprintf("Received float: %f\n", value);
            }
            else
            {
                rx_buf[rx_len] = '\0';
                rt_kprintf("Received string: %s\n", rx_buf);
            }
        }
    }
}

// 模拟摄像头采集图像
void camera_thread_entry(void *parameter)
{
    while (1)
    {
        rt_kprintf("[Camera] Capturing image...\n");
        rt_thread_mdelay(1000); // 模拟图像采集延迟
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        strcpy(detected_garbage, "Plastic Bottle"); // 模拟检测到的垃圾类型
        rt_mutex_release(data_mutex);
        rt_kprintf("[Camera] Image captured and processed.\n");
    }
}

// 模拟 GPS 获取位置信息   使用uart2
void gps_thread_entry(void *parameter)
{
    gps_init();
    while (1)
    {
        //根据手机选好的模式或者 默认模式巡检

        if(gps_tau1201_flag)
		{
			gps_tau1201_flag = 0;
			gps_data_parse();           //开始解析数据

		}

        rt_kprintf("[GPS] Fetching location...\n");
        rt_thread_mdelay(2000); // 模拟 GPS 获取延迟
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        strcpy(gps_location, "Lat: 39.9042, Lon: 116.4074"); // 模拟 GPS 位置信息
        rt_mutex_release(data_mutex);
        rt_kprintf("[GPS] Location fetched.\n");
    }
}

// 保存数据到 SD 卡
// 保存数据到 SD 卡 
/*
在 RT-Thread 中，fprintf、fclose、fgets 等标准 C 文件操作函数是通过文件系统（如 FAT 文件系统）与 MSD 驱动结合起来的。文件系统会调用 MSD 驱动提供的接口（如 read、write 等），而 MSD 驱动则通过 SPI 接口与 SD 卡通信，完成底层的块读写操作。
文件系统调用 MSD 驱动的 read 和 write 接口时，会调用 rt_msd_read 和 rt_msd_write。
这些函数会根据块号和块大小，调用 _read_block 和 _write_block 函数，通过 SPI 接口与 SD 卡通信。

*/
void sdcard_thread_entry(void *parameter)
{
    while (1)
    {
        rt_thread_mdelay(3000); // 模拟数据保存间隔
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);

                /*
        在 RT-Thread 中，fprintf 和 fgets 等标准 C 文件操作函数通过文件系统（如 FAT 文件系统）与 MSD 驱动结合起来，完成对 SD 卡的读写操作。SD 卡的操作是以扇区为单位的，而文件系统负责将文件的逻辑操作（如写入字符串、读取行）映射到 SD 卡的物理扇区操作上。
        FATS  负责将C语言函数的操作映射到 SD卡的物理扇区操作上
        fprintf:
        1. fprintf将字符串“  ”写入到缓冲区，文件缓冲区是由文件系统管理的
        2. 文件系统将缓冲数据写入SD卡：
            缓冲区满了或者调用fclose时，文件系统将缓冲区的数据写入SD卡
            文件系统会根据文件的元信息（起始扇区号，文件大小等），计算出需要写入的磁盘区号和偏移量
        3.文件系统调用驱动的WRITE接口：
            文件系统调用 MSD 驱动的 rt_msd_write 函数，将数据写入到指定的扇区。
            MSD 驱动会根据扇区号和数据大小，调用 _write_block 函数，通过 SPI 接口与 SD 卡通信。
        4.MSD驱动通过SPI接口写入数据
            _write_block 使用 SPI 驱动的 xfer 接口，将数据块发送到 SD 卡。
            SD 卡接收数据后，将其写入到物理存储介质。
        */

        // 写入数据到 SD 卡  文件系统会将文件的元信息（如文件名、大小、起始块号等）加载到内存中
        FILE *file = fopen(SD_MOUNT_POINT"/garbage_log.txt", "a");
        if (file)
        {
            //会将数据写入文件缓冲区。
            //写图片

            fprintf(file, "Garbage: %s, Location: %s\n", detected_garbage, gps_location);

            struct payload {
                uint8_t mode; // 模式 0 1 2 3
                float param[N]; N 
            };

            write(file, &payload, sizeof(payload));


            //当缓冲区满或调用 fclose 时，文件系统会将缓冲区中的数据写入到 SD 卡的物理块中。
            fclose(file); //会确保所有缓冲区数据写入到 SD 卡，并关闭文件。
            rt_kprintf("[SDCard] Data saved: Garbage: %s, Location: %s\n", detected_garbage, gps_location);
        }
        else
        {
            rt_kprintf("[SDCard] Failed to open file for writing.\n");
        }

        rt_mutex_release(data_mutex);

        // 读取数据并打印确认

        file = fopen(SD_MOUNT_POINT"/garbage_log.txt", "r");
        if (file)
        {
            char line[128];
            rt_kprintf("[SDCard] Reading data from SD card:\n");
            while (fgets(line, sizeof(line), file))
            {
                rt_kprintf("%s", line); // 打印每一行内容
            }
            fclose(file);
        }
        else
        {
            rt_kprintf("[SDCard] Failed to open file for reading.\n");
        }
    }
}

// 主函数
int main(void)
{
    rt_kprintf("System initializing...\n");

    // 初始化互斥锁
    data_mutex = rt_mutex_create("data_mutex", RT_IPC_FLAG_FIFO);
    if (data_mutex == RT_NULL)
    {
        rt_kprintf("[Error] Failed to create mutex.\n");
        return -1;
    }
    //zai drv_spi.cshixianshixasss实现  通过宏定义   BSP_USING_SPI1   BSP_USING_SPI1_SS0片选
     // // 1. 注册 SPI 设备
    // if (rt_hw_spi_device_attach(SD_SPI_BUS, SD_DEVICE_NAME, 0, SD_CS_PIN) != RT_EOK)
    // {
    //     rt_kprintf("Failed to attach SPI device\n");
    //     return -RT_ERROR;
    // }

    // 2. 初始化 MSD 设备
    if (msd_init("sd0", SD_DEVICE_NAME) != RT_EOK)
    {
        rt_kprintf("Failed to initialize MSD device\n");
        return -RT_ERROR;
    }
    //3. 挂载 SD 卡
    /*
    dfs_mount 会调用 rt_device_find("sd0") 查找 MSD 设备。
    如果设备未初始化，会调用 rt_device_init("sd0")，从而触发 rt_msd_init。
    */
    if (dfs_mount("sd0", SD_MOUNT_POINT, "elm", 0, 0) == 0)
    {
        rt_kprintf("[SDCard] Mounted successfully at %s.\n", SD_MOUNT_POINT);
    }
    else
    {
        rt_kprintf("[SDCard] Failed to mount.\n");
        return -1;
    }

    char uart_name[RT_NAME_MAX];

    serial = rt_device_find(uart_name);


     // /配置串口参数
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = 9600; // 设置波特率为9600
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

     //初始化信号量
     rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
     //// 打开串口，使用中断接收
     rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);//open的时候初始化
     // // 设置接收回调
     rt_device_set_rx_indicate(serial, uart_callback);//回调函数 发送信号量 读取



    Servo_init();
    Servo__Stright_PID_Init();
    Turn_set=0;

    Motor_Init();
    Motor_Control(Stright_Speed);

    rt_sem_release(Car_sem);
    rt_sem_release(MPU_sem);

    //创建手机遥控线程
     //创建接收线程
    rt_thread_t thread = rt_thread_create("serial", uart_rx_thread, RT_NULL, 1024, 25, 10);


    // 创建摄像头线程
    rt_thread_t camera_thread = rt_thread_create("camera",
                                                 camera_thread_entry,
                                                 RT_NULL,
                                                 THREAD_STACK_SIZE,
                                                 THREAD_PRIORITY,
                                                 THREAD_TIMESLICE);
    if (camera_thread != RT_NULL)
        rt_thread_startup(camera_thread);

    // 创建 GPS 线程
    rt_thread_t gps_thread = rt_thread_create("gps",
                                              gps_thread_entry,
                                              RT_NULL,
                                              THREAD_STACK_SIZE,
                                              THREAD_PRIORITY,
                                              THREAD_TIMESLICE);
    if (gps_thread != RT_NULL)
        rt_thread_startup(gps_thread);

    // 创建 SD 卡线程
    rt_thread_t sdcard_thread = rt_thread_create("sdcard",
                                                 sdcard_thread_entry,
                                                 RT_NULL,
                                                 THREAD_STACK_SIZE,
                                                 THREAD_PRIORITY,
                                                 THREAD_TIMESLICE);
    if (sdcard_thread != RT_NULL)
        rt_thread_startup(sdcard_thread);

    rt_kprintf("System initialized.\n");

    return 0;
}