#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>
#define SAMPLE_UART_NAME       "uart2"
static struct rt_semaphore rx_sem;
static rt_device_t serial;

 static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
 {

     rt_sem_release(&rx_sem);

     return RT_EOK;
 }
#define RX_BUF_SIZE 32
 static void serial_thread_entry(void parameter)
 {
     uint8_t rx_buf[RX_BUF_SIZE];

     while (1)
     {
        
         while (rt_device_read(serial, -1, &ch, 1) != 1)
         {

             rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
         }

         ch = ch + 1;
         rt_device_write(serial, 0, &ch, 1);
     }
 }

 static int uart_sample(int argc, char argv[])
 {
     rt_err_t ret = RT_EOK;
     char uart_name[RT_NAME_MAX];
     char str[] = "hello RT-Thread!\r\n";

     if (argc == 2)
     {
         rt_strncpy(uart_name, argv[1], RT_NAME_MAX);
     }
     else
     {
         rt_strncpy(uart_name, SAMPLE_UART_NAME, RT_NAME_MAX);
     }


     serial = rt_device_find(uart_name);
     if (!serial)
     {
         rt_kprintf("find %s failed!\n", uart_name);
         return -RT_ERROR;
     }

     // /配置串口参数
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = 9600; // 设置波特率为9600
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

     //初始化信号量
     rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
     //// 打开串口，使用中断接收
     rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);//open的时候初始化
     // // 设置接收回调
     rt_device_set_rx_indicate(serial, uart_input);//回调函数 发送信号量 读取

    //  rt_device_write(serial, 0, str, (sizeof(str) - 1));

     //创建接收线程
     rt_thread_t thread = rt_thread_create("serial", serial_thread_entry, RT_NULL, 1024, 25, 10);

     if (thread != RT_NULL)
     {
         rt_thread_startup(thread);
     }
     else
     {
         ret = -RT_ERROR;
     }

     return ret;
 }

 MSH_CMD_EXPORT(uart_sample, uart device sample);