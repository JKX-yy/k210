/*
 * Copyright (c) 2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-01-07     Developer    CUAV Neo3 GPS驱动实现 - UBlox协议
 */

#include "drv_cuav_neo3.h"
#include <string.h>
#include <math.h>

/* 全局GPS设备实例 */
static cuav_neo3_device_t *g_cuav_neo3_dev = RT_NULL;

/* UART接收回调函数 - 基于RT-Thread FIFO机制 */
static rt_err_t cuav_neo3_uart_callback(rt_device_t dev, rt_size_t size)
{
    cuav_neo3_device_t *gps_dev = g_cuav_neo3_dev;
    
    if (gps_dev == RT_NULL) {
        return RT_ERROR;
    }
    
    /* 通过信号量通知解析线程处理数据 */
    rt_sem_release(gps_dev->data_sem);
    
    return RT_EOK;
}

/* UBlox校验和计算函数 */
uint16_t ublox_calculate_checksum(const uint8_t *data, rt_size_t len)
{
    uint8_t ck_a = 0, ck_b = 0;
    
    for (rt_size_t i = 0; i < len; i++) {
        ck_a = ck_a + data[i];
        ck_b = ck_b + ck_a;
    }
    
    return (ck_b << 8) | ck_a;
}

/* UBlox字节解析状态机 */
rt_err_t ublox_parse_byte(cuav_neo3_device_t *dev, uint8_t byte)
{
    switch (dev->parse_state) {
        case UBLOX_PARSE_STATE_IDLE:
        case UBLOX_PARSE_STATE_SYNC1:
            if (byte == UBLOX_SYNC_CHAR_1) {
                dev->parse_state = UBLOX_PARSE_STATE_SYNC2;
                dev->current_msg.sync_char_1 = byte;
                dev->checksum_a = 0;
                dev->checksum_b = 0;
            } else {
                dev->parse_state = UBLOX_PARSE_STATE_IDLE;
            }
            break;
            
        case UBLOX_PARSE_STATE_SYNC2:
            if (byte == UBLOX_SYNC_CHAR_2) {
                dev->parse_state = UBLOX_PARSE_STATE_CLASS;
                dev->current_msg.sync_char_2 = byte;
            } else {
                dev->parse_state = UBLOX_PARSE_STATE_IDLE;
                dev->error_count++;
            }
            break;
            
        case UBLOX_PARSE_STATE_CLASS:
            dev->current_msg.class = byte;
            dev->checksum_a += byte;
            dev->checksum_b += dev->checksum_a;
            dev->parse_state = UBLOX_PARSE_STATE_ID;
            break;
            
        case UBLOX_PARSE_STATE_ID:
            dev->current_msg.id = byte;
            dev->checksum_a += byte;
            dev->checksum_b += dev->checksum_a;
            dev->parse_state = UBLOX_PARSE_STATE_LENGTH1;
            break;
            
        case UBLOX_PARSE_STATE_LENGTH1:
            dev->current_msg.length = byte;
            dev->checksum_a += byte;
            dev->checksum_b += dev->checksum_a;
            dev->parse_state = UBLOX_PARSE_STATE_LENGTH2;
            break;
            
        case UBLOX_PARSE_STATE_LENGTH2:
            dev->current_msg.length |= (uint16_t)byte << 8;
            dev->checksum_a += byte;
            dev->checksum_b += dev->checksum_a;
            
            if (dev->current_msg.length > UBLOX_MAX_PAYLOAD_LEN) {
                dev->parse_state = UBLOX_PARSE_STATE_IDLE;
                dev->error_count++;
                return RT_ERROR;
            }
            
            dev->payload_index = 0;
            if (dev->current_msg.length == 0) {
                dev->parse_state = UBLOX_PARSE_STATE_CHECKSUM1;
            } else {
                dev->parse_state = UBLOX_PARSE_STATE_PAYLOAD;
            }
            break;
            
        case UBLOX_PARSE_STATE_PAYLOAD:
            dev->current_msg.payload[dev->payload_index++] = byte;
            dev->checksum_a += byte;
            dev->checksum_b += dev->checksum_a;
            
            if (dev->payload_index >= dev->current_msg.length) {
                dev->parse_state = UBLOX_PARSE_STATE_CHECKSUM1;
            }
            break;
            
        case UBLOX_PARSE_STATE_CHECKSUM1:
            dev->current_msg.checksum_a = byte;
            dev->parse_state = UBLOX_PARSE_STATE_CHECKSUM2;
            break;
            
        case UBLOX_PARSE_STATE_CHECKSUM2:
            dev->current_msg.checksum_b = byte;
            dev->parse_state = UBLOX_PARSE_STATE_IDLE;
            
            /* 校验和验证 */
            if (dev->current_msg.checksum_a == dev->checksum_a && 
                dev->current_msg.checksum_b == dev->checksum_b) {
                /* 校验成功，处理消息 */
                ublox_process_message(dev, &dev->current_msg);
                dev->message_count++;
                return RT_EOK;
            } else {
                /* 校验失败 */
                dev->checksum_errors++;
                return RT_ERROR;
            }
            break;
            
        default:
            dev->parse_state = UBLOX_PARSE_STATE_IDLE;
            break;
    }
    
    return RT_EOK;
}

/* UBlox消息处理函数 */
rt_err_t ublox_process_message(cuav_neo3_device_t *dev, ublox_message_t *msg)
{
    rt_err_t result = RT_EOK;
    
    rt_mutex_take(dev->data_mutex, RT_WAITING_FOREVER);
    
    switch (msg->class) {
        case UBLOX_CLASS_NAV:
            switch (msg->id) {
                case UBLOX_ID_NAV_POSLLH:
                    /* 解析位置信息 */
                    if (msg->length >= 28) {
                        dev->gps_data.time_of_week = *(uint32_t*)&msg->payload[0];
                        dev->gps_data.longitude = *(int32_t*)&msg->payload[4] * 1e-7;
                        dev->gps_data.latitude = *(int32_t*)&msg->payload[8] * 1e-7;
                        dev->gps_data.altitude = *(int32_t*)&msg->payload[12];
                        dev->gps_data.horizontal_accuracy = *(uint32_t*)&msg->payload[20];
                        dev->gps_data.vertical_accuracy = *(uint32_t*)&msg->payload[24];
                        dev->gps_data.valid = 1;
                        dev->gps_data.last_update = rt_tick_get();
                    }
                    break;
                    
                case UBLOX_ID_NAV_STATUS:
                    /* 解析状态信息 */
                    if (msg->length >= 16) {
                        dev->gps_data.fix_type = msg->payload[4];
                        dev->gps_data.flags = msg->payload[5];
                    }
                    break;
                    
                case UBLOX_ID_NAV_DOP:
                    /* 解析DOP信息 */
                    if (msg->length >= 18) {
                        dev->gps_data.position_dop = *(uint16_t*)&msg->payload[12];
                        dev->gps_data.time_dop = *(uint16_t*)&msg->payload[14];
                    }
                    break;
                    
                case UBLOX_ID_NAV_SOL:
                    /* 解析解算信息 */
                    if (msg->length >= 52) {
                        dev->gps_data.satellites = msg->payload[47];
                    }
                    break;
                    
                case UBLOX_ID_NAV_VELNED:
                    /* 解析速度信息 */
                    if (msg->length >= 36) {
                        dev->gps_data.velocity_north = *(int32_t*)&msg->payload[8];
                        dev->gps_data.velocity_east = *(int32_t*)&msg->payload[12];
                        dev->gps_data.velocity_down = *(int32_t*)&msg->payload[16];
                        dev->gps_data.ground_speed = *(uint32_t*)&msg->payload[20];
                        dev->gps_data.heading = *(uint32_t*)&msg->payload[24];
                        dev->gps_data.speed_accuracy = *(uint32_t*)&msg->payload[28];
                    }
                    break;
                    
                case UBLOX_ID_NAV_TIMEUTC:
                    /* 解析UTC时间信息 */
                    if (msg->length >= 20) {
                        dev->gps_data.year = *(uint16_t*)&msg->payload[12];
                        dev->gps_data.month = msg->payload[14];
                        dev->gps_data.day = msg->payload[15];
                        dev->gps_data.hour = msg->payload[16];
                        dev->gps_data.minute = msg->payload[17];
                        dev->gps_data.second = msg->payload[18];
                    }
                    break;
                    
                default:
                    break;
            }
            break;
            
        default:
            break;
    }
    
    rt_mutex_release(dev->data_mutex);
    
    return result;
}

/* GPS数据解析线程 - 基于RT-Thread UART FIFO */
static void cuav_neo3_parse_thread_entry(void *parameter)
{
    cuav_neo3_device_t *dev = (cuav_neo3_device_t *)parameter;
    uint8_t rx_buffer[128];
    rt_size_t read_len;
    
    rt_kprintf("[CUAV_Neo3] Parse thread started, using UART FIFO mechanism\n");
    
    while (1) {
        /* 等待UART数据到达信号量 */
        if (rt_sem_take(dev->data_sem, RT_WAITING_FOREVER) == RT_EOK) {
            
            /* 从UART FIFO读取数据 */
            read_len = rt_device_read(dev->uart_device, 0, rx_buffer, sizeof(rx_buffer));
            
            if (read_len > 0) {
                /* 逐字节解析UBlox协议 */
                for (rt_size_t i = 0; i < read_len; i++) {
                    ublox_parse_byte(dev, rx_buffer[i]);
                }
            }
        }
    }
}

/* UBlox配置消息生成函数 */
rt_err_t ublox_generate_cfg_rate(uint8_t *buffer, uint16_t meas_rate, uint16_t nav_rate)
{
    uint8_t payload[6] = {
        (uint8_t)(meas_rate & 0xFF),       // 测量速率低字节
        (uint8_t)(meas_rate >> 8),         // 测量速率高字节
        (uint8_t)(nav_rate & 0xFF),        // 导航速率低字节
        (uint8_t)(nav_rate >> 8),          // 导航速率高字节
        0x01, 0x00                         // 时间参考：GPS时间
    };
    
    /* 构造UBlox消息 */
    buffer[0] = UBLOX_SYNC_CHAR_1;         // 同步字符1
    buffer[1] = UBLOX_SYNC_CHAR_2;         // 同步字符2
    buffer[2] = 0x06;                      // CFG类别
    buffer[3] = 0x08;                      // RATE消息ID
    buffer[4] = 0x06;                      // 长度低字节
    buffer[5] = 0x00;                      // 长度高字节
    
    /* 复制载荷 */
    memcpy(&buffer[6], payload, 6);
    
    /* 计算校验和 */
    uint16_t checksum = ublox_calculate_checksum(&buffer[2], 10);
    buffer[12] = checksum & 0xFF;          // 校验和A
    buffer[13] = (checksum >> 8) & 0xFF;   // 校验和B
    
    return RT_EOK;
}

/* 发送UBlox命令函数 */
rt_err_t cuav_neo3_send_command(const uint8_t *cmd, rt_size_t len)
{
    if (g_cuav_neo3_dev == RT_NULL || g_cuav_neo3_dev->uart_device == RT_NULL) {
        return RT_ERROR;
    }
    
    rt_size_t written = rt_device_write(g_cuav_neo3_dev->uart_device, 0, cmd, len);
    
    return (written == len) ? RT_EOK : RT_ERROR;
}

/* 获取GPS数据接口 */
const cuav_neo3_data_t* cuav_neo3_get_data(void)
{
    if (g_cuav_neo3_dev == RT_NULL) {
        return RT_NULL;
    }
    
    return &g_cuav_neo3_dev->gps_data;
}

/* 打印GPS状态信息 */
void cuav_neo3_print_status(void)
{
    if (g_cuav_neo3_dev == RT_NULL) {
        rt_kprintf("[CUAV_Neo3] Device not initialized\n");
        return;
    }
    
    const cuav_neo3_data_t *data = &g_cuav_neo3_dev->gps_data;
    
    rt_kprintf("\n[CUAV_Neo3] GPS Status Report:\n");
    rt_kprintf("  Position: %.6f, %.6f, Alt: %d mm\n", 
               data->latitude, data->longitude, data->altitude);
    rt_kprintf("  Fix Type: %d, Satellites: %d, Valid: %d\n",
               data->fix_type, data->satellites, data->valid);
    rt_kprintf("  Speed: %d cm/s, Heading: %.2f°\n",
               data->ground_speed, data->heading * 1e-5);
    rt_kprintf("  Accuracy: H=%d mm, V=%d mm\n",
               data->horizontal_accuracy, data->vertical_accuracy);
    rt_kprintf("  Statistics: Msg=%d, Err=%d, ChkErr=%d\n",
               g_cuav_neo3_dev->message_count, g_cuav_neo3_dev->error_count, 
               g_cuav_neo3_dev->checksum_errors);
    rt_kprintf("  Last Update: %d ms ago\n",
               (rt_tick_get() - data->last_update) * 1000 / RT_TICK_PER_SECOND);
}

/* CUAV Neo3 GPS驱动初始化函数 */
rt_err_t cuav_neo3_init(const char *uart_name)
{
    rt_err_t result = RT_EOK;
    
    /* 检查是否已经初始化 */
    if (g_cuav_neo3_dev != RT_NULL) {
        rt_kprintf("[CUAV_Neo3] Already initialized\n");
        return RT_EBUSY;
    }
    
    /* 分配设备结构体内存 */
    g_cuav_neo3_dev = (cuav_neo3_device_t *)rt_malloc(sizeof(cuav_neo3_device_t));
    if (g_cuav_neo3_dev == RT_NULL) {
        rt_kprintf("[CUAV_Neo3] Failed to allocate device memory\n");
        return RT_ENOMEM;
    }
    
    /* 初始化设备结构体 */
    memset(g_cuav_neo3_dev, 0, sizeof(cuav_neo3_device_t));
    g_cuav_neo3_dev->parse_state = UBLOX_PARSE_STATE_IDLE;
    
    /* 查找UART设备 */
    g_cuav_neo3_dev->uart_device = rt_device_find(uart_name);
    if (g_cuav_neo3_dev->uart_device == RT_NULL) {
        rt_kprintf("[CUAV_Neo3] Cannot find UART device: %s\n", uart_name);
        result = RT_ERROR;
        goto cleanup;
    }
    
    /* 配置UART参数 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = CUAV_NEO3_BAUDRATE;
    config.data_bits = CUAV_NEO3_DATA_BITS;
    config.stop_bits = CUAV_NEO3_STOP_BITS;
    config.parity = CUAV_NEO3_PARITY;
    
    rt_device_control(g_cuav_neo3_dev->uart_device, RT_DEVICE_CTRL_CONFIG, &config);
    
    /* 打开UART设备（中断接收 + 读写模式） */
    result = rt_device_open(g_cuav_neo3_dev->uart_device, 
                           RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_RDWR);
    if (result != RT_EOK) {
        rt_kprintf("[CUAV_Neo3] Failed to open UART device\n");
        goto cleanup;
    }
    
    /* 设置UART接收回调函数 */
    rt_device_set_rx_indicate(g_cuav_neo3_dev->uart_device, cuav_neo3_uart_callback);
    
    /* 创建数据互斥锁 */
    g_cuav_neo3_dev->data_mutex = rt_mutex_create("gps_mutex", RT_IPC_FLAG_FIFO);
    if (g_cuav_neo3_dev->data_mutex == RT_NULL) {
        rt_kprintf("[CUAV_Neo3] Failed to create data mutex\n");
        result = RT_ERROR;
        goto cleanup;
    }
    
    /* 创建数据信号量 */
    g_cuav_neo3_dev->data_sem = rt_sem_create("gps_sem", 0, RT_IPC_FLAG_FIFO);
    if (g_cuav_neo3_dev->data_sem == RT_NULL) {
        rt_kprintf("[CUAV_Neo3] Failed to create data semaphore\n");
        result = RT_ERROR;
        goto cleanup;
    }
    
    /* 创建解析线程 */
    g_cuav_neo3_dev->parse_thread = rt_thread_create("gps_parse",
                                                     cuav_neo3_parse_thread_entry,
                                                     g_cuav_neo3_dev,
                                                     2048,
                                                     12,
                                                     20);
    if (g_cuav_neo3_dev->parse_thread == RT_NULL) {
        rt_kprintf("[CUAV_Neo3] Failed to create parse thread\n");
        result = RT_ERROR;
        goto cleanup;
    }
    
    /* 启动解析线程 */
    rt_thread_startup(g_cuav_neo3_dev->parse_thread);
    
    /* 发送配置命令：设置10Hz更新率 */
    uint8_t cfg_rate_cmd[14];
    ublox_generate_cfg_rate(cfg_rate_cmd, 100, 1);  // 100ms测量间隔, 1:1导航比率
    cuav_neo3_send_command(cfg_rate_cmd, sizeof(cfg_rate_cmd));
    
    rt_kprintf("[CUAV_Neo3] GPS driver initialized successfully\n");
    rt_kprintf("[CUAV_Neo3] UART: %s, Baudrate: %d, Protocol: UBlox\n", 
               uart_name, CUAV_NEO3_BAUDRATE);
    rt_kprintf("[CUAV_Neo3] Update Rate: 10Hz, FIFO-based reception\n");
    
    return RT_EOK;
    
cleanup:
    if (g_cuav_neo3_dev->data_sem) {
        rt_sem_delete(g_cuav_neo3_dev->data_sem);
    }
    if (g_cuav_neo3_dev->data_mutex) {
        rt_mutex_delete(g_cuav_neo3_dev->data_mutex);
    }
    if (g_cuav_neo3_dev->uart_device) {
        rt_device_close(g_cuav_neo3_dev->uart_device);
    }
    if (g_cuav_neo3_dev) {
        rt_free(g_cuav_neo3_dev);
        g_cuav_neo3_dev = RT_NULL;
    }
    
    return result;
}

/* CUAV Neo3 GPS驱动去初始化函数 */
rt_err_t cuav_neo3_deinit(void)
{
    if (g_cuav_neo3_dev == RT_NULL) {
        return RT_EOK;
    }
    
    /* 删除解析线程 */
    if (g_cuav_neo3_dev->parse_thread) {
        rt_thread_delete(g_cuav_neo3_dev->parse_thread);
    }
    
    /* 删除同步对象 */
    if (g_cuav_neo3_dev->data_sem) {
        rt_sem_delete(g_cuav_neo3_dev->data_sem);
    }
    if (g_cuav_neo3_dev->data_mutex) {
        rt_mutex_delete(g_cuav_neo3_dev->data_mutex);
    }
    
    /* 关闭UART设备 */
    if (g_cuav_neo3_dev->uart_device) {
        rt_device_close(g_cuav_neo3_dev->uart_device);
    }
    
    /* 释放设备内存 */
    rt_free(g_cuav_neo3_dev);
    g_cuav_neo3_dev = RT_NULL;
    
    rt_kprintf("[CUAV_Neo3] GPS driver deinitialized\n");
    
    return RT_EOK;
}

/* MSH命令：GPS状态查看 */
static void cmd_gps_status(int argc, char **argv)
{
    cuav_neo3_print_status();
}
MSH_CMD_EXPORT(cmd_gps_status, Show CUAV Neo3 GPS status);

/* MSH命令：GPS重新初始化 */
static void cmd_gps_init(int argc, char **argv)
{
    const char *uart_name = (argc > 1) ? argv[1] : CUAV_NEO3_UART_NAME;
    
    cuav_neo3_deinit();
    rt_err_t result = cuav_neo3_init(uart_name);
    
    if (result == RT_EOK) {
        rt_kprintf("GPS initialized on %s\n", uart_name);
    } else {
        rt_kprintf("GPS initialization failed\n");
    }
}
MSH_CMD_EXPORT(cmd_gps_init, Initialize CUAV Neo3 GPS driver);
