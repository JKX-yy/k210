/*
 * Copyright (c) 2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-01-07     Developer    CUAV Neo3 GPS驱动头文件 - UBlox协议
 */

#ifndef __DRV_CUAV_NEO3_H__
#define __DRV_CUAV_NEO3_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/serial.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CUAV Neo3 GPS模块配置 */
#define CUAV_NEO3_UART_NAME         "uart2"        // GPS使用UART2
#define CUAV_NEO3_BAUDRATE          38400          // CUAV Neo3默认波特率
#define CUAV_NEO3_DATA_BITS         DATA_BITS_8    // 8位数据位
#define CUAV_NEO3_STOP_BITS         STOP_BITS_1    // 1位停止位
#define CUAV_NEO3_PARITY            PARITY_NONE    // 无校验

/* UBlox协议相关定义 */
#define UBLOX_SYNC_CHAR_1           0xB5           // UBlox同步字符1
#define UBLOX_SYNC_CHAR_2           0x62           // UBlox同步字符2
#define UBLOX_HEADER_LEN            6              // UBlox消息头长度
#define UBLOX_CHECKSUM_LEN          2              // UBlox校验和长度
#define UBLOX_MAX_PAYLOAD_LEN       512            // 最大载荷长度

/* UBlox消息类别和ID */
#define UBLOX_CLASS_NAV             0x01           // 导航消息类别
#define UBLOX_ID_NAV_POSLLH         0x02           // 位置消息ID
#define UBLOX_ID_NAV_STATUS         0x03           // 状态消息ID  
#define UBLOX_ID_NAV_DOP            0x04           // DOP消息ID
#define UBLOX_ID_NAV_SOL            0x06           // 解算消息ID
#define UBLOX_ID_NAV_VELNED         0x12           // 速度消息ID
#define UBLOX_ID_NAV_TIMEUTC        0x21           // UTC时间消息ID
#define UBLOX_ID_NAV_SVINFO         0x30           // 卫星信息消息ID

/* NMEA消息类别（兼容NMEA模式） */
#define NMEA_MAX_LEN                82             // NMEA最大句子长度
#define NMEA_START_CHAR             '$'            // NMEA起始字符

/* GPS数据结构定义 */
typedef struct {
    /* 位置信息 */
    double latitude;                               // 纬度 (度)
    double longitude;                              // 经度 (度)
    int32_t altitude;                              // 海拔高度 (mm)
    uint32_t horizontal_accuracy;                  // 水平精度 (mm)
    uint32_t vertical_accuracy;                    // 垂直精度 (mm)
    
    /* 速度信息 */
    int32_t velocity_north;                        // 北向速度 (cm/s)
    int32_t velocity_east;                         // 东向速度 (cm/s)  
    int32_t velocity_down;                         // 下向速度 (cm/s)
    uint32_t speed_accuracy;                       // 速度精度 (cm/s)
    uint32_t ground_speed;                         // 地面速度 (cm/s)
    uint32_t heading;                              // 航向角 (1e-5 度)
    
    /* 时间信息 */
    uint32_t time_of_week;                         // 周内时间 (ms)
    uint16_t year;                                 // 年
    uint8_t month;                                 // 月
    uint8_t day;                                   // 日
    uint8_t hour;                                  // 时
    uint8_t minute;                                // 分
    uint8_t second;                                // 秒
    
    /* 状态信息 */
    uint8_t fix_type;                              // 定位类型
    uint8_t flags;                                 // 状态标志
    uint8_t satellites;                            // 可见卫星数
    uint16_t position_dop;                         // 位置DOP
    uint16_t time_dop;                             // 时间DOP
    
    /* 系统状态 */
    uint8_t valid;                                 // 数据有效标志
    uint32_t last_update;                          // 最后更新时间戳
} cuav_neo3_data_t;

/* UBlox消息结构 */
typedef struct {
    uint8_t sync_char_1;                           // 同步字符1 (0xB5)
    uint8_t sync_char_2;                           // 同步字符2 (0x62)
    uint8_t class;                                 // 消息类别
    uint8_t id;                                    // 消息ID
    uint16_t length;                               // 载荷长度
    uint8_t payload[UBLOX_MAX_PAYLOAD_LEN];        // 载荷数据
    uint8_t checksum_a;                            // 校验和A
    uint8_t checksum_b;                            // 校验和B
} ublox_message_t;

/* UBlox解析状态机 */
typedef enum {
    UBLOX_PARSE_STATE_IDLE = 0,                    // 空闲状态
    UBLOX_PARSE_STATE_SYNC1,                       // 等待同步字符1
    UBLOX_PARSE_STATE_SYNC2,                       // 等待同步字符2
    UBLOX_PARSE_STATE_CLASS,                       // 等待消息类别
    UBLOX_PARSE_STATE_ID,                          // 等待消息ID
    UBLOX_PARSE_STATE_LENGTH1,                     // 等待长度低字节
    UBLOX_PARSE_STATE_LENGTH2,                     // 等待长度高字节
    UBLOX_PARSE_STATE_PAYLOAD,                     // 等待载荷数据
    UBLOX_PARSE_STATE_CHECKSUM1,                   // 等待校验和A
    UBLOX_PARSE_STATE_CHECKSUM2                    // 等待校验和B
} ublox_parse_state_t;

/* GPS驱动结构体 */
typedef struct {
    rt_device_t uart_device;                       // UART设备句柄
    rt_thread_t parse_thread;                      // 解析线程
    rt_mutex_t data_mutex;                         // 数据互斥锁
    rt_sem_t data_sem;                             // 数据信号量
    
    /* 接收缓冲区 */
    uint8_t rx_buffer[256];                        // 接收缓冲区
    rt_size_t rx_index;                           // 接收索引
    
    /* UBlox解析状态 */
    ublox_parse_state_t parse_state;               // 解析状态机
    ublox_message_t current_msg;                   // 当前消息
    uint16_t payload_index;                        // 载荷索引
    uint8_t checksum_a;                            // 校验和A
    uint8_t checksum_b;                            // 校验和B
    
    /* GPS数据 */
    cuav_neo3_data_t gps_data;                     // GPS数据
    
    /* 统计信息 */
    uint32_t message_count;                        // 消息计数
    uint32_t error_count;                          // 错误计数
    uint32_t checksum_errors;                      // 校验错误计数
} cuav_neo3_device_t;

/* 函数声明 */
rt_err_t cuav_neo3_init(const char *uart_name);
rt_err_t cuav_neo3_deinit(void);
const cuav_neo3_data_t* cuav_neo3_get_data(void);
rt_err_t cuav_neo3_send_command(const uint8_t *cmd, rt_size_t len);
void cuav_neo3_print_status(void);

/* UBlox协议函数 */
rt_err_t ublox_parse_byte(cuav_neo3_device_t *dev, uint8_t byte);
rt_err_t ublox_process_message(cuav_neo3_device_t *dev, ublox_message_t *msg);
uint16_t ublox_calculate_checksum(const uint8_t *data, rt_size_t len);

/* 配置命令生成函数 */
rt_err_t ublox_generate_cfg_rate(uint8_t *buffer, uint16_t meas_rate, uint16_t nav_rate);
rt_err_t ublox_generate_cfg_msg(uint8_t *buffer, uint8_t class, uint8_t id, uint8_t rate);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_CUAV_NEO3_H__ */
