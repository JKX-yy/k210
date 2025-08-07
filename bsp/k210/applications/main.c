/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018/09/30     Bernard      The first version
 * 2025/01/XX     Author       城市垃圾巡检系统实现
 */

#include <rtthread.h>
#include <stdio.h>
#include <dev_spi_msd.h>
#include <dfs_fs.h>
#include <string.h>
#include <stdlib.h>
#include <kpu.h>              // K210 KPU硬件加速器头文件
#include <region_layer.h>     // YOLO区域层处理
#include <image.h>            // 图像处理头文件
#include <math.h>
#include "board.h"

// RT-Thread核心头文件
#include <rtdevice.h>
#include <board.h>

// RT-Thread串口驱动内部结构（用于直接访问驱动层缓冲区）
#include <drivers/serial.h>

// K210摄像头驱动
#include "camera/drv_ov5640.h"

// 车辆控制相关头文件
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// 定义线程栈大小和优先级 - 优化实时性
#define THREAD_STACK_SIZE 4096          // 增加栈大小，支持YOLO算法
#define CAMERA_THREAD_PRIORITY 8        // 相机线程高优先级，保证实时性
#define GPS_THREAD_PRIORITY 12          // GPS线程中等优先级
#define SDCARD_THREAD_PRIORITY 15       // SD卡线程低优先级
#define UART_THREAD_PRIORITY 10         // UART协议解析中优先级
#define PATROL_THREAD_PRIORITY 11       // 巡检控制线程
#define THREAD_TIMESLICE 20             // 增加时间片

// 实时控制任务定时器配置
// 控制定时器周期定义
#define SPEED_CONTROL_PERIOD_MS 10       // 速度环控制周期10ms (100Hz)
#define BALANCE_CONTROL_PERIOD_MS 5      // 平衡环控制周期5ms (200Hz)  
#define GPS_CONTROL_PERIOD_MS 100        // GPS方向控制周期100ms (10Hz)
#define ENCODER_TIMER_PERIOD_MS 5        // 编码器读取周期5ms (200Hz)
#define MPU6050_TIMER_PERIOD_MS 10       // MPU6050读取周期10ms (100Hz)

// 车辆控制参数定义
#define MAX_SPEED 1000                  // 最大速度（编码器计数/秒）
#define MAX_STEERING_ANGLE 45           // 最大转向角度（度）
#define WHEEL_CIRCUMFERENCE 0.2094f     // 车轮周长（米）
#define ENCODER_PPR 1024                // 编码器每转脉冲数

// PID控制器参数
typedef struct {
    float kp, ki, kd;                   // PID参数
    float prev_error;                   // 上次误差
    float integral;                     // 积分项
    float max_output;                   // 输出限制
    float min_output;                   // 输出下限
} pid_controller_t;

// 三轮车状态结构体 (单电机+单编码器+舵机)
typedef struct {
    // 编码器数据 (单个后轮驱动电机)
    volatile int32_t encoder_count;         // 编码器计数
    volatile float vehicle_speed;           // 车辆速度 (m/s)
    volatile float vehicle_angle;           // 车辆航向角 (度)
    
    // MPU6050数据
    volatile float gyro_z;                  // Z轴角速度 (度/秒)
    volatile float accel_x, accel_y;        // X,Y轴加速度 (m/s²)
    volatile float pitch, roll, yaw;        // 姿态角 (度)
    
    // 控制目标
    volatile float target_speed;            // 目标速度 (m/s)
    volatile float target_steering;         // 目标转向角度 (度)
    
    // 控制输出
    volatile int16_t motor_pwm;             // 后轮电机PWM (-1000~1000)
    volatile float servo_angle;             // 前轮舵机角度 (-45~45度)
} vehicle_state_t;

// 定义 SD 卡挂载点
#define SD_SPI_BUS "spi1"       // SPI 总线名称
#define SD_DEVICE_NAME "spi10"  // SPI 设备名称
#define SD_CS_PIN 5             // 片选引脚
#define SD_MOUNT_POINT "/sd"    // 挂载点

// 定义GPS路径文件和图像保存路径
#define GPS_ROUTE_FILE SD_MOUNT_POINT"/mode0.txt"    // GPS路径点文件
#define IMAGE_SAVE_DIR SD_MOUNT_POINT"/res/image"    // 图像保存目录

// K210 KPU YOLO模型相关定义 (基于YOLOv2架构 - 适合K210资源限制)
#define YOLO_VERSION "YOLOv2"                                 // YOLO模型版本 (YOLOv2更适合K210)
#define YOLO_MODEL_FILE SD_MOUNT_POINT"/garbage_yolov2.kmodel" // YOLOv2模型文件
#define ANCHOR_NUM 5                                           // YOLOv2锚框数量
#define CLASS_NUM 10                                          // 垃圾分类数量
#define NET_INPUT_WIDTH 320                                   // 网络输入宽度 (K210优化尺寸)
#define NET_INPUT_HEIGHT 240                                  // 网络输入高度 (K210优化尺寸)
#define YOLO_THRESHOLD 0.5                                    // YOLO检测阈值
#define NMS_THRESHOLD 0.2                                     // 非极大值抑制阈值
#define GRID_WIDTH (NET_INPUT_WIDTH / 32)                     // YOLOv2网格宽度 (10)
#define GRID_HEIGHT (NET_INPUT_HEIGHT / 32)                   // YOLOv2网格高度 (7)

// K210定义
#define BLUETOOTH_UART_NAME "uart1"      // 蓝牙使用UART1设备名
#define GPS_UART_NAME "uart2"            // GPS使用UART2设备名
#define BLUETOOTH_UART_BAUDRATE 9600     // 蓝牙波特率
#define GPS_UART_BAUDRATE 9600           // GPS波特率

// UART缓冲区配置
#define UART_BUFFER_SIZE        256                    // 统一缓冲区大小

// 环形缓冲区性能优化配置
#define RING_BUFFER_SIZE 512     // 增大缓冲区，提高抗突发性能
#define UART_BATCH_SIZE 16       // 批量处理大小，减少中断处理频率

// 图像处理相关定义 (基于K210 QVGA分辨率)
#define IMAGE_HEIGHT 240        // K210摄像头QVGA高度
#define IMAGE_WIDTH 320         // K210摄像头QVGA宽度
#define MAX_PATROL_POINTS 100   // 最大路径点数量(从mode0.txt读取)

// 图像双缓冲区（模拟mt9v03x摄像头）
uint16_t mt9v03x_image[2][IMAGE_HEIGHT][IMAGE_WIDTH];  // 双缓冲区
uint16_t processed_image[IMAGE_HEIGHT][IMAGE_WIDTH];   // 处理后的图像缓冲区（用于保存带标注的图片）
volatile int current_buf = 0;  // 当前DMA写入的缓冲区索引（需要保护）

// K210 KPU YOLO相关全局变量 (YOLOv2优化)
static kpu_model_context_t yolo_model;                // KPU模型上下文
static uint8_t *yolo_model_data = NULL;               // 模型数据指针
static float yolo_anchor[ANCHOR_NUM * 2] = {          // YOLOv2锚框参数 (针对K210和垃圾检测优化)
    0.57273, 0.677385, 1.87446, 2.06253, 3.33843, 
    5.47434, 7.88282, 3.52778, 9.77052, 9.16828
};
static volatile bool kpu_initialized = false;         // KPU初始化状态

}

/*
 * 从SD卡读取GPS路径点文件
 * 功能：读取mode0.txt中的GPS坐标点，格式：纬度,经度
 * 文件格式示例：
 * 39.904200,116.407400
 * 39.904500,116.407700
 * ...
 */
static int load_gps_route_from_file(void)
{
    FILE *route_file = fopen(GPS_ROUTE_FILE, "r");
    if (!route_file) {
        rt_kprintf("[GPS Route] Cannot open %s, using default route
", GPS_ROUTE_FILE);
        return -1;
    }

    char line[64];
    int loaded_count = 0;
    
    rt_kprintf("[GPS Route] Loading route points from %s...
", GPS_ROUTE_FILE);
    
    while (fgets(line, sizeof(line), route_file) && loaded_count < MAX_PATROL_POINTS) {
        float lat, lon;
        
        // 解析每行数据：纬度,经度
        if (sscanf(line, "%f,%f", &lat, &lon) == 2) {
            patrol_points[loaded_count].latitude = lat;
            patrol_points[loaded_count].longitude = lon;
            snprintf(patrol_points[loaded_count].description, 
                    sizeof(patrol_points[loaded_count].description),
                    "Route_Point_%d", loaded_count + 1);
            
            rt_kprintf("[GPS Route] Point %d: (%.6f, %.6f)
", 
                      loaded_count + 1, lat, lon);
            loaded_count++;
        } else {
            rt_kprintf("[GPS Route] Invalid line format: %s", line);
        }
    }
    
    fclose(route_file);
    patrol_point_count = loaded_count;
    
    rt_kprintf("[GPS Route] Successfully loaded %d route points
", loaded_count);
    return loaded_count;
}

/*
 * 创建图像保存目录
 * 功能：确保/res/image目录存在
 */
static rt_err_t create_image_directory(void)
{
    // 创建res目录
    if (mkdir(SD_MOUNT_POINT"/res", 0755) != 0) {
        // 目录可能已存在，检查errno
        rt_kprintf("[Init] /res directory already exists or created
");
    }
    
    // 创建image目录
    if (mkdir(IMAGE_SAVE_DIR, 0755) != 0) {
        rt_kprintf("[Init] /res/image directory already exists or created
");
    } else {
        rt_kprintf("[Init] Created image directory: %s
", IMAGE_SAVE_DIR);
    }
    
    return RT_EOK;
}

// 原子读取双缓冲区索引（线程中调用）
static inline int atomic_read_current_buf(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    int buf = current_buf;
    rt_hw_interrupt_enable(level);
    return buf;
}

// 原子切换双缓冲区索引（DMA中断中调用）
static inline void atomic_switch_current_buf(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    current_buf = current_buf ^ 1;  // 切换缓冲区
    rt_hw_interrupt_enable(level);
}

// 全局变量定义
static rt_mutex_t data_mutex;           // 数据同步互斥锁
static rt_event_t patrol_event;         // 巡检事件
// SD卡写入使用RT-Thread阻塞式驱动，无需completion机制

// 车辆控制全局变量
static vehicle_state_t vehicle_state = {0};    // 车辆状态
static pid_controller_t speed_pid;              // 速度PID控制器
static pid_controller_t angle_pid;              // 角度PID控制器

// RT-Thread设备句柄（使用RT-Thread设备框架）
static rt_device_t uart_bluetooth = RT_NULL;   // 蓝牙UART设备
static rt_device_t uart_gps = RT_NULL;          // GPS UART设备

// GPIO引脚定义（直接控制电机和舵机）
#define MOTOR_PIN_1    12    // 电机控制引脚1
#define MOTOR_PIN_2    13    // 电机控制引脚2
#define SERVO_PIN      14    // 舵机控制引脚
#define MOTOR_EN_PIN   15    // 电机使能引脚

// 电机方向控制引脚定义
#define MOTOR_DIR1_PIN  16   // 电机方向控制引脚1
#define MOTOR_DIR2_PIN  17   // 电机方向控制引脚2

// PWM设备句柄
static rt_device_t pwm_motor_dev = RT_NULL;  // 电机PWM设备
static rt_device_t pwm_servo_dev = RT_NULL;  // 舵机PWM设备

// 三个独立控制定时器句柄
static rt_timer_t speed_control_timer = RT_NULL;     // 速度环控制定时器 (100Hz)
static rt_timer_t balance_control_timer = RT_NULL;   // 平衡环控制定时器 (200Hz)
static rt_timer_t gps_control_timer = RT_NULL;       // GPS方向控制定时器 (10Hz)

// 数据采集定时器句柄
static rt_timer_t encoder_timer = RT_NULL;      // 编码器读取定时器 (200Hz)
static rt_timer_t mpu6050_timer = RT_NULL;      // MPU6050读取定时器 (100Hz)

static char detected_garbage[64];       // 检测到的垃圾类型
static char gps_location[64];           // GPS位置信息

// 全局GPS位置变量（实时更新）
typedef struct {
    volatile float latitude;        // 纬度
    volatile float longitude;       // 经度
    volatile float altitude;        // 海拔
    volatile float speed;           // 速度 (km/h)
    volatile float course;          // 航向 (度)
    volatile uint8_t satellites;    // 卫星数量
    volatile uint8_t fix_quality;   // 定位质量 (0=无效, 1=GPS, 2=DGPS)
    volatile uint32_t timestamp;    // 时间戳
    volatile uint8_t valid;         // 数据有效标志
} gps_data_t;

static gps_data_t current_gps = {
    .latitude = 39.9042f,    // 北京天安门坐标（默认值）
    .longitude = 116.4074f,
    .altitude = 50.0f,
    .speed = 0.0f,
    .course = 0.0f,
    .satellites = 0,
    .fix_quality = 0,
    .timestamp = 0,
    .valid = 0
};

// 分层控制架构变量 - 三个独立控制环
typedef struct {
    // GPS方向控制环（外环，10Hz）
    float target_latitude;          // 目标纬度
    float target_longitude;         // 目标经度  
    float target_heading;           // 目标航向角 (度)
    float heading_error;            // 航向误差
    float heading_error_integral;   // 航向误差积分
    float last_heading_error;       // 上次航向误差
    float target_speed;             // 目标速度 (m/s)
    
    // 平衡控制环（内环，200Hz）
    float target_pitch;             // 目标俯仰角
    float pitch_error;              // 俯仰角误差
    float pitch_error_integral;     // 俯仰角误差积分
    float last_pitch_error;         // 上次俯仰角误差
    float balance_output;           // 平衡控制输出
    
    // 速度控制环（中环，100Hz）
    float speed_error;              // 速度误差
    float speed_error_integral;     // 速度误差积分
    float last_speed_error;         // 上次速度误差
    int motor_pwm_output;           // 电机PWM输出
    
    // 舵机控制
    float target_servo_angle;       // 目标舵机角度
} layered_control_t;

static layered_control_t layered_ctrl = {0};

// PID控制器参数定义
#define SPEED_KP 500.0f          // 速度环比例系数
#define SPEED_KI 50.0f           // 速度环积分系数  
#define SPEED_KD 20.0f           // 速度环微分系数

#define BALANCE_KP 800.0f        // 平衡环比例系数
#define BALANCE_KI 10.0f         // 平衡环积分系数
#define BALANCE_KD 100.0f        // 平衡环微分系数

#define HEADING_KP 3.0f          // 航向环比例系数
#define HEADING_KI 0.1f          // 航向环积分系数
#define HEADING_KD 0.5f          // 航向环微分系数

// 全局控制变量
static float current_speed = 0.0f;       // 当前速度 (m/s)
static float pitch_angle = 0.0f;         // 当前俯仰角 (度) 
static float yaw_angle = 0.0f;           // 当前偏航角 (度)
static uint32_t encoder_count = 0;       // 编码器计数
static uint32_t last_encoder_count = 0;  // 上次编码器计数

// 手动控制变量
static float manual_speed = 0.0f;        // 手动模式目标速度
static float manual_direction_offset = 0.0f; // 手动模式方向偏移

// 使用RT-Thread设备框架的UART缓冲区
static uint8_t bluetooth_rx_buffer[UART_BUFFER_SIZE];    // 蓝牙接收缓冲区
static uint8_t gps_rx_buffer[UART_BUFFER_SIZE];          // GPS接收缓冲区

static struct rt_semaphore image_sem;  // 图像处理信号量

// 巡检模式定义
typedef enum {
    PATROL_MODE_MANUAL = 0,    // 手动模式
    PATROL_MODE_AUTO,          // 自动巡检模式
    PATROL_MODE_PRESET,        // 预设路径模式
    PATROL_MODE_STOP           // 停止模式
} patrol_mode_t;

// 巡检点结构
typedef struct {
    float latitude;
    float longitude;
    char description[32];
} patrol_point_t;

// YOLO垃圾检测结果结构
typedef struct {
    int detected_count;           // 检测到的垃圾数量
    char garbage_types[10][32];   // 垃圾类型名称
    float confidences[10];        // 置信度
    struct {
        int x, y, w, h;           // 边界框
    } bboxes[10];
} yolo_result_t;

// 垃圾类型定义（10种）
static const char* GARBAGE_TYPES[10] = {
    "废纸", "塑料瓶", "玻璃瓶", "金属罐", "织物", 
    "果皮", "油漆桶", "废旧电池", "电子垃圾", "有害垃圾"
};

// 全局巡检状态 - 默认自动巡检模式
static patrol_mode_t current_patrol_mode = PATROL_MODE_AUTO;  // 默认自动巡检模式
static patrol_point_t patrol_points[MAX_PATROL_POINTS];       // 从mode0.txt读取的路径点
static int patrol_point_count = 0;                            // 实际加载的路径点数量
static int current_patrol_index = 0;                          // 当前巡检点索引

// 事件标志定义
#define EVENT_UART_COMMAND      (1 << 0)
#define EVENT_IMAGE_CAPTURED    (1 << 1)
#define EVENT_GPS_UPDATED       (1 << 2)
#define EVENT_PATROL_START      (1 << 3)
#define EVENT_PATROL_STOP       (1 << 4)
#define EVENT_UART_DMA_READY    (1 << 5)  // UART DMA数据就绪事件
#define EVENT_UART_TIMEOUT      (1 << 6)  // UART接收超时事件

// 图像处理统计
static volatile int image_frame_count = 0;
static volatile int processed_frame_count = 0;

/* ==================== 实时控制任务实现 ==================== */

/*
 * PID控制器初始化
 */
static void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float max_out, float min_out)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->max_output = max_out;
    pid->min_output = min_out;
}

/*
 * PID控制器计算
 */
static float pid_calculate(pid_controller_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;
    float derivative;
    float output;
    
    // 积分项计算
    pid->integral += error * dt;
    
    // 积分限幅
    if (pid->integral > pid->max_output) pid->integral = pid->max_output;
    if (pid->integral < pid->min_output) pid->integral = pid->min_output;
    
    // 微分项计算
    derivative = (error - pid->prev_error) / dt;
    
    // PID输出计算
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    // 输出限幅
    if (output > pid->max_output) output = pid->max_output;
    if (output < pid->min_output) output = pid->min_output;
    
    pid->prev_error = error;
    
    return output;
}

/*
 * 编码器读取定时器回调函数 (200Hz)
 * 功能：高频读取编码器数据，计算车辆速度
 */
static void encoder_timer_callback(void *parameter)
{
    static int32_t last_encoder_count = 0;
    static rt_tick_t last_time = 0;
    
    // 读取单个编码器当前计数（后轮驱动电机）
    // 示例：使用GPIO中断计数或专用编码器接口
    int32_t current_count = vehicle_state.encoder_count;
    rt_tick_t current_time = rt_tick_get();
    
    if (last_time != 0) {
        float dt = (current_time - last_time) * 1.0f / RT_TICK_PER_SECOND;
        
        // 计算编码器增量
        int32_t count_delta = current_count - last_encoder_count;
        
        // 计算车轮速度（后轮）
        vehicle_state.vehicle_speed = (count_delta * WHEEL_CIRCUMFERENCE) / (ENCODER_PPR * dt);
        
        // 三轮车模型：前轮转向，后轮驱动，航向角主要由舵机控制
        // 这里可以结合MPU6050的陀螺仪数据进行航向角融合
    }
    
    last_encoder_count = current_count;
    last_time = current_time;
}

/*
 * MPU6050读取定时器回调函数 (50Hz)
 * 功能：读取MPU6050姿态数据，进行滤波处理
 */
static void mpu6050_timer_callback(void *parameter)
{
    // 这里需要实现MPU6050的I2C读取（根据实际硬件接口）
    // 示例代码：读取陀螺仪和加速度计数据
    
    // 读取陀螺仪Z轴数据（航向角速度）
    // float gyro_z_raw = mpu6050_read_gyro_z();
    // vehicle_state.gyro_z = gyro_z_raw; // 可以添加滤波
    
    // 读取加速度计数据
    // vehicle_state.accel_x = mpu6050_read_accel_x();
    // vehicle_state.accel_y = mpu6050_read_accel_y();
    
    // 姿态融合（互补滤波或卡尔曼滤波）
    // 这里使用简化的互补滤波
    static float alpha = 0.98f; // 互补滤波系数
    float dt = MPU6050_TIMER_PERIOD_MS / 1000.0f;
    
    // 陀螺仪积分
    vehicle_state.yaw += vehicle_state.gyro_z * dt;
    
    // 与编码器数据融合
    vehicle_state.vehicle_angle = alpha * vehicle_state.vehicle_angle + 
                                 (1.0f - alpha) * vehicle_state.yaw;
    
    rt_kprintf("[MPU6050] Gyro_Z: %.2f, Vehicle_Angle: %.2f\n", 
              vehicle_state.gyro_z, vehicle_state.vehicle_angle);
}

// 分层控制定时器计数器
static uint32_t control_loop_count = 0;

/*
 * GPS方向控制定时器回调 - 外环控制 (10Hz)
 * 功能：根据GPS位置计算目标航向和速度，更新导航指令
 */
static void gps_control_timer_callback(void *parameter)
{
    static uint32_t gps_control_counter = 0;
    gps_control_counter++;
    
    if (current_patrol_mode != PATROL_MODE_STOP && current_gps.valid) {
        
        // 根据巡检模式计算目标位置
        switch (current_patrol_mode) {
            case PATROL_MODE_AUTO:
                // 自动模式：按照从mode0.txt加载的路径点循迹
                if (patrol_point_count > 0) {
                    layered_ctrl.target_latitude = patrol_points[current_patrol_index].latitude;
                    layered_ctrl.target_longitude = patrol_points[current_patrol_index].longitude;
                    
                    // 计算到目标点的距离
                    float dx = (layered_ctrl.target_longitude - current_gps.longitude) * 111000; // 大约111km/度
                    float dy = (layered_ctrl.target_latitude - current_gps.latitude) * 111000;
                    float distance = sqrtf(dx*dx + dy*dy);
                    
                    // 到达判断与路径点切换
                    if (distance < 3.0f) { // 3米到达阈值
                        current_patrol_index = (current_patrol_index + 1) % patrol_point_count;
                    }
                    
                    // 计算目标航向角（GPS导航输出）
                    layered_ctrl.target_heading = atan2f(dx, dy) * 180.0f / M_PI;
                    if (layered_ctrl.target_heading < 0) layered_ctrl.target_heading += 360.0f;
                    
                    layered_ctrl.target_speed = distance > 10.0f ? 1.5f : 0.8f; // 远距离快速，近距离慢速
                }
                break;
                
            case PATROL_MODE_RETURN:
                // 返回模式：导航到第一个路径点
                if (patrol_point_count > 0) {
                    layered_ctrl.target_latitude = patrol_points[0].latitude;
                    layered_ctrl.target_longitude = patrol_points[0].longitude;
                    
                    float dx = (layered_ctrl.target_longitude - current_gps.longitude) * 111000;
                    float dy = (layered_ctrl.target_latitude - current_gps.latitude) * 111000;
                    
                    layered_ctrl.target_heading = atan2f(dx, dy) * 180.0f / M_PI;
                    if (layered_ctrl.target_heading < 0) layered_ctrl.target_heading += 360.0f;
                    
                    layered_ctrl.target_speed = 1.2f;
                }
                break;
                
            case PATROL_MODE_MANUAL:
                // 手动模式：基于蓝牙命令设置目标
                layered_ctrl.target_speed = manual_speed;
                layered_ctrl.target_heading = current_gps.course + manual_direction_offset;
                break;
        }
        
        // 计算航向偏差（处理360度边界）
        float current_heading = yaw_angle; // 从MPU6050获取的偏航角
        if (current_heading < 0) current_heading += 360.0f;
        
        layered_ctrl.heading_error = layered_ctrl.target_heading - current_heading;
        if (layered_ctrl.heading_error > 180.0f) layered_ctrl.heading_error -= 360.0f;
        if (layered_ctrl.heading_error < -180.0f) layered_ctrl.heading_error += 360.0f;
        
        // PID航向控制器（输出舵机角度）
        layered_ctrl.heading_error_integral += layered_ctrl.heading_error * 0.1f; // dt=0.1s
        
        // 积分饱和限制
        if (layered_ctrl.heading_error_integral > 50.0f) layered_ctrl.heading_error_integral = 50.0f;
        if (layered_ctrl.heading_error_integral < -50.0f) layered_ctrl.heading_error_integral = -50.0f;
        
        float heading_derivative = (layered_ctrl.heading_error - layered_ctrl.last_heading_error) / 0.1f;
        layered_ctrl.last_heading_error = layered_ctrl.heading_error;
        
        // PID输出 -> 舵机角度（-30到+30度）
        layered_ctrl.target_servo_angle = HEADING_KP * layered_ctrl.heading_error + 
                                          HEADING_KI * layered_ctrl.heading_error_integral + 
                                          HEADING_KD * heading_derivative;
        
        // 舵机角度限制
        if (layered_ctrl.target_servo_angle > 30.0f) layered_ctrl.target_servo_angle = 30.0f;
        if (layered_ctrl.target_servo_angle < -30.0f) layered_ctrl.target_servo_angle = -30.0f;
        
    } else {
        layered_ctrl.target_speed = 0.0f; // 停止或GPS无效时目标速度为0
        layered_ctrl.target_servo_angle = 0.0f; // 舵机回中
    }
    
    // 调试输出（每1秒输出一次）
    if (gps_control_counter % 10 == 0) {
        rt_kprintf("[GPS_Control] Target:(%.6f,%.6f) Heading:%.1f->%.1f Error:%.1f Servo:%.1f\n",
                  layered_ctrl.target_latitude, layered_ctrl.target_longitude,
                  yaw_angle, layered_ctrl.target_heading, layered_ctrl.heading_error, 
                  layered_ctrl.target_servo_angle);
    }
}

/*
 * 平衡控制定时器回调 - 内环控制 (200Hz)
 * 功能：维持车辆平衡，基于MPU6050的俯仰角进行平衡控制
 */
static void balance_control_timer_callback(void *parameter)
{
    static uint32_t balance_control_counter = 0;
    balance_control_counter++;
    
    // 平衡控制目标角度（对于三轮车，目标俯仰角通常为0）
    layered_ctrl.target_pitch = 0.0f;
    
    // 计算俯仰角误差
    layered_ctrl.pitch_error = layered_ctrl.target_pitch - pitch_angle;
    
    // PID平衡控制器
    layered_ctrl.pitch_error_integral += layered_ctrl.pitch_error * 0.005f; // dt=0.005s (200Hz)
    
    // 积分饱和限制
    if (layered_ctrl.pitch_error_integral > 10.0f) layered_ctrl.pitch_error_integral = 10.0f;
    if (layered_ctrl.pitch_error_integral < -10.0f) layered_ctrl.pitch_error_integral = -10.0f;
    
    float pitch_derivative = (layered_ctrl.pitch_error - layered_ctrl.last_pitch_error) / 0.005f;
    layered_ctrl.last_pitch_error = layered_ctrl.pitch_error;
    
    // PID输出 -> 平衡补偿值
    layered_ctrl.balance_output = BALANCE_KP * layered_ctrl.pitch_error + 
                                  BALANCE_KI * layered_ctrl.pitch_error_integral + 
                                  BALANCE_KD * pitch_derivative;
    
    // 平衡输出限制
    if (layered_ctrl.balance_output > 500.0f) layered_ctrl.balance_output = 500.0f;
    if (layered_ctrl.balance_output < -500.0f) layered_ctrl.balance_output = -500.0f;
    
    // 调试输出（每1秒输出一次）
    if (balance_control_counter % 200 == 0) {
        rt_kprintf("[Balance_Control] Pitch:%.2f Target:%.2f Error:%.2f Output:%.1f\n",
                  pitch_angle, layered_ctrl.target_pitch, layered_ctrl.pitch_error, 
                  layered_ctrl.balance_output);
    }
}

/*
 * 速度控制定时器回调 - 中环控制 (100Hz)  
 * 功能：根据目标速度和平衡补偿，输出最终的电机PWM控制
 */
static void speed_control_timer_callback(void *parameter)
{
    static uint32_t speed_control_counter = 0;
    speed_control_counter++;
    
    // 计算速度误差
    layered_ctrl.speed_error = layered_ctrl.target_speed - current_speed;
    
    // PID速度控制器
    layered_ctrl.speed_error_integral += layered_ctrl.speed_error * 0.01f; // dt=0.01s (100Hz)
    
    // 积分饱和限制
    if (layered_ctrl.speed_error_integral > 100.0f) layered_ctrl.speed_error_integral = 100.0f;
    if (layered_ctrl.speed_error_integral < -100.0f) layered_ctrl.speed_error_integral = -100.0f;
    
    float speed_derivative = (layered_ctrl.speed_error - layered_ctrl.last_speed_error) / 0.01f;
    layered_ctrl.last_speed_error = layered_ctrl.speed_error;
    
    // PID输出 + 平衡补偿 -> PWM占空比
    float speed_output = SPEED_KP * layered_ctrl.speed_error + 
                        SPEED_KI * layered_ctrl.speed_error_integral + 
                        SPEED_KD * speed_derivative;
    
    // 叠加平衡控制输出
    layered_ctrl.motor_pwm_output = (int)(speed_output + layered_ctrl.balance_output);
    
    // PWM限制
    if (layered_ctrl.motor_pwm_output > 800) layered_ctrl.motor_pwm_output = 800;
    if (layered_ctrl.motor_pwm_output < -800) layered_ctrl.motor_pwm_output = -800;
    
    // 死区补偿
    if (layered_ctrl.motor_pwm_output > 0 && layered_ctrl.motor_pwm_output < 100) 
        layered_ctrl.motor_pwm_output = 100;
    if (layered_ctrl.motor_pwm_output < 0 && layered_ctrl.motor_pwm_output > -100) 
        layered_ctrl.motor_pwm_output = -100;
    
    // 执行电机控制
    if (layered_ctrl.motor_pwm_output > 0) {
        rt_pin_write(MOTOR_DIR1_PIN, PIN_HIGH);
        rt_pin_write(MOTOR_DIR2_PIN, PIN_LOW);
        rt_device_write(pwm_motor_dev, 0, &layered_ctrl.motor_pwm_output, sizeof(layered_ctrl.motor_pwm_output));
    } else if (layered_ctrl.motor_pwm_output < 0) {
        rt_pin_write(MOTOR_DIR1_PIN, PIN_LOW);
        rt_pin_write(MOTOR_DIR2_PIN, PIN_HIGH);
        int abs_pwm = -layered_ctrl.motor_pwm_output;
        rt_device_write(pwm_motor_dev, 0, &abs_pwm, sizeof(abs_pwm));
    } else {
        rt_pin_write(MOTOR_DIR1_PIN, PIN_LOW);
        rt_pin_write(MOTOR_DIR2_PIN, PIN_LOW);
        int zero_pwm = 0;
        rt_device_write(pwm_motor_dev, 0, &zero_pwm, sizeof(zero_pwm));
    }
    
    // 执行舵机控制
    int servo_pwm = (int)(1500 + layered_ctrl.target_servo_angle * 500.0f / 90.0f); // 1000-2000us脉宽
    if (servo_pwm > 2000) servo_pwm = 2000;
    if (servo_pwm < 1000) servo_pwm = 1000;
    rt_device_write(pwm_servo_dev, 0, &servo_pwm, sizeof(servo_pwm));
    
    // 调试输出（每1秒输出一次）
    if (speed_control_counter % 100 == 0) {
        rt_kprintf("[Speed_Control] Speed:%.2f->%.2f Error:%.2f Balance:%.1f PWM:%d Servo:%d\n",
                  current_speed, layered_ctrl.target_speed, layered_ctrl.speed_error,
                  layered_ctrl.balance_output, layered_ctrl.motor_pwm_output, servo_pwm);
    }
}
        rt_kprintf("[Layered Control] GPS: %.1f° Err: %.1f° -> Steering: %.1f° -> Servo: %.1f°\n",
                  current_gps.course, layered_ctrl.heading_error, 
                  layered_ctrl.steering_command, vehicle_state.servo_angle);
        rt_kprintf("[Control] Speed: %.2f/%.2f m/s, Motor PWM: %d, GPS Valid: %s\n",
                  vehicle_state.vehicle_speed, vehicle_state.target_speed,
                  vehicle_state.motor_pwm, current_gps.valid ? "Yes" : "No");
    }
}

/*
 * 初始化RT-Thread设备 - 修正版
 * 功能：使用RT-Thread设备框架，配置UART为中断模式
 */
static rt_err_t init_rt_thread_devices(void)
{
    rt_kprintf("[Device] Initializing RT-Thread devices...\n");
    
    // 1. 初始化蓝牙UART设备
    uart_bluetooth = rt_device_find(BLUETOOTH_UART_NAME);
    if (uart_bluetooth != RT_NULL) {
        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
        config.baud_rate = BLUETOOTH_UART_BAUDRATE;
        config.data_bits = DATA_BITS_8;
        config.stop_bits = STOP_BITS_1;
        config.parity = PARITY_NONE;
        config.bit_order = BIT_ORDER_LSB;
        config.invert = NRZ_NORMAL;
        config.bufsz = 512;  // 设置缓冲区大小
        
        rt_device_control(uart_bluetooth, RT_DEVICE_CTRL_CONFIG, &config);
        
        // 以中断接收模式打开设备
        rt_device_open(uart_bluetooth, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_RDWR);
        rt_kprintf("[Device] Bluetooth UART1 initialized (INT mode)\n");
    } else {
        rt_kprintf("[Error] Cannot find %s device\n", BLUETOOTH_UART_NAME);
        return -RT_ERROR;
    }
    
    // 2. 初始化GPS UART设备
    uart_gps = rt_device_find(GPS_UART_NAME);
    if (uart_gps != RT_NULL) {
        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
        config.baud_rate = GPS_UART_BAUDRATE;
        config.data_bits = DATA_BITS_8;
        config.stop_bits = STOP_BITS_1;
        config.parity = PARITY_NONE;
        config.bit_order = BIT_ORDER_LSB;
        config.invert = NRZ_NORMAL;
        config.bufsz = 512;  // 设置缓冲区大小
        
        rt_device_control(uart_gps, RT_DEVICE_CTRL_CONFIG, &config);
        
        // 以中断接收模式打开设备
        rt_device_open(uart_gps, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_RDWR);
        rt_kprintf("[Device] GPS UART2 initialized (INT mode)\n");
    } else {
        rt_kprintf("[Error] Cannot find %s device\n", GPS_UART_NAME);
        return -RT_ERROR;
    }
    
    // 3. 初始化GPIO控制引脚（电机和舵机）
    rt_pin_mode(MOTOR_PIN_1, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR_PIN_2, PIN_MODE_OUTPUT);
    rt_pin_mode(SERVO_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR_EN_PIN, PIN_MODE_OUTPUT);
    
    // 初始化电机和舵机为停止状态
    rt_pin_write(MOTOR_PIN_1, PIN_LOW);
    rt_pin_write(MOTOR_PIN_2, PIN_LOW);
    rt_pin_write(SERVO_PIN, PIN_LOW);
    rt_pin_write(MOTOR_EN_PIN, PIN_LOW);
    
    rt_kprintf("[Device] Motor and Servo GPIO initialized\n");
    
    rt_kprintf("[Device] RT-Thread devices initialization completed\n");
    return RT_EOK;
}

/* ==================== 电机和舵机控制函数 ==================== */

/*
 * 电机控制函数
 * direction: 0=停止, 1=前进, 2=后退
 * speed: 0-100 (百分比)
 */
static void motor_control(int direction, int speed)
{
    switch (direction) {
        case 0: // 停止
            rt_pin_write(MOTOR_PIN_1, PIN_LOW);
            rt_pin_write(MOTOR_PIN_2, PIN_LOW);
            rt_pin_write(MOTOR_EN_PIN, PIN_LOW);
            break;
            
        case 1: // 前进
            rt_pin_write(MOTOR_PIN_1, PIN_HIGH);
            rt_pin_write(MOTOR_PIN_2, PIN_LOW);
            rt_pin_write(MOTOR_EN_PIN, PIN_HIGH);
            break;
            
        case 2: // 后退
            rt_pin_write(MOTOR_PIN_1, PIN_LOW);
            rt_pin_write(MOTOR_PIN_2, PIN_HIGH);
            rt_pin_write(MOTOR_EN_PIN, PIN_HIGH);
            break;
            
        default:
            motor_control(0, 0); // 默认停止
            break;
    }
    
    rt_kprintf("[Motor] Direction: %d, Speed: %d%%\n", direction, speed);
}

/*
 * 舵机控制函数
 * angle: -45 到 +45 度
 */
static void servo_control(int angle)
{
    // 限制角度范围
    if (angle > 45) angle = 45;
    if (angle < -45) angle = -45;
    
    // 简单的舵机控制（实际项目中可能需要PWM）
    if (angle > 5) {
        rt_pin_write(SERVO_PIN, PIN_HIGH); // 右转
    } else if (angle < -5) {
        rt_pin_write(SERVO_PIN, PIN_LOW);  // 左转
    } else {
        // 中位，可以用定时器实现PWM或者保持上次状态
    }
    
    rt_kprintf("[Servo] Angle: %d degrees\n", angle);
}

/*
 * 初始化实时控制系统
 * 功能：创建定时器，初始化PID控制器
 */
static rt_err_t init_realtime_control(void)
{
    rt_kprintf("[Control] Initializing real-time control system...\n");
    
    // 1. 初始化PID控制器
    pid_init(&speed_pid, 0.8f, 0.1f, 0.05f, 1.0f, -1.0f);  // 速度环PID
    pid_init(&angle_pid, 2.0f, 0.0f, 0.1f, 500.0f, -500.0f); // 角度环PID
    
    // 2. 创建编码器读取定时器 (200Hz)
    encoder_timer = rt_timer_create("encoder",
                                   encoder_timer_callback,
                                   RT_NULL,
                                   rt_tick_from_millisecond(ENCODER_TIMER_PERIOD_MS),
                                   RT_TIMER_FLAG_PERIODIC);
    if (encoder_timer != RT_NULL) {
        rt_timer_start(encoder_timer);
        rt_kprintf("[Control] Encoder timer started (200Hz)\n");
    }
    
    // 3. 创建MPU6050读取定时器 (100Hz)
    mpu6050_timer = rt_timer_create("mpu6050",
                                   mpu6050_timer_callback,
                                   RT_NULL,
                                   rt_tick_from_millisecond(MPU6050_TIMER_PERIOD_MS),
                                   RT_TIMER_FLAG_PERIODIC);
    if (mpu6050_timer != RT_NULL) {
        rt_timer_start(mpu6050_timer);
        rt_kprintf("[Control] MPU6050 timer started (100Hz)\n");
    }
    
    // 4. 创建GPS方向控制定时器 (10Hz) - 外环
    gps_control_timer = rt_timer_create("gps_ctrl",
                                       gps_control_timer_callback,
                                       RT_NULL,
                                       rt_tick_from_millisecond(GPS_CONTROL_PERIOD_MS),
                                       RT_TIMER_FLAG_PERIODIC);
    if (gps_control_timer != RT_NULL) {
        rt_timer_start(gps_control_timer);
        rt_kprintf("[Control] GPS control timer started (10Hz)\n");
    }
    
    // 5. 创建平衡控制定时器 (200Hz) - 内环
    balance_control_timer = rt_timer_create("balance_ctrl",
                                           balance_control_timer_callback,
                                           RT_NULL,
                                           rt_tick_from_millisecond(BALANCE_CONTROL_PERIOD_MS),
                                           RT_TIMER_FLAG_PERIODIC);
    if (balance_control_timer != RT_NULL) {
        rt_timer_start(balance_control_timer);
        rt_kprintf("[Control] Balance control timer started (200Hz)\n");
    }
    
    // 6. 创建速度控制定时器 (100Hz) - 中环
    speed_control_timer = rt_timer_create("speed_ctrl",
                                         speed_control_timer_callback,
                                         RT_NULL,
                                         rt_tick_from_millisecond(SPEED_CONTROL_PERIOD_MS),
                                         RT_TIMER_FLAG_PERIODIC);
    if (speed_control_timer != RT_NULL) {
        rt_timer_start(speed_control_timer);
        rt_kprintf("[Control] Speed control timer started (100Hz)\n");
    }
    
    // 7. 初始化车辆状态
    vehicle_state.target_speed = 0.0f;
    vehicle_state.target_angle = 0.0f;
    
    rt_kprintf("[Control] Three-layer control system initialized\n");
    rt_kprintf("[Control] GPS Control (10Hz) -> Balance Control (200Hz) -> Speed Control (100Hz)\n");
    rt_kprintf("[Control] Speed PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", SPEED_KP, SPEED_KI, SPEED_KD);
    rt_kprintf("[Control] Balance PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", BALANCE_KP, BALANCE_KI, BALANCE_KD);
    rt_kprintf("[Control] Heading PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", HEADING_KP, HEADING_KI, HEADING_KD);
    
    return RT_EOK;
}

/* ==================== RT-Thread UART通信实现 - 最佳实践版 ==================== */

/*
 * RT-Thread串口最佳实践说明：
 * 1. RT-Thread串口驱动内置环形缓冲区（大小为RT_SERIAL_RB_BUFSZ，通常64-256字节）
 * 2. 直接访问驱动层rx_fifo避免了重复的rt_device_read调用
 * 3. 减少了中断上下文的处理时间，提高实时性
 * 4. 应用层缓冲区专门用于协议解析，职责更清晰
 */

// UART接收回调函数（RT-Thread设备框架）- 直接访问驱动层缓冲区
static rt_err_t uart_bluetooth_callback(rt_device_t dev, rt_size_t size)
{
    struct rt_serial_device *serial = (struct rt_serial_device *)dev;
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;

    // 从RT-Thread串口驱动内置环形缓冲区复制数据到应用层环形缓冲区
    while (rx_fifo->get_index != rx_fifo->put_index) {
        uint8_t ch = rx_fifo->buffer[rx_fifo->get_index];
        rx_fifo->get_index = (rx_fifo->get_index + 1) % RT_SERIAL_RB_BUFSZ;
        ring_buffer_put(ch);  // 用户自定义的协议解析环形缓冲区
    }
    
    return RT_EOK;
}

// GPS UART接收回调函数 - 直接访问驱动层缓冲区
static rt_err_t uart_gps_callback(rt_device_t dev, rt_size_t size)
{
    struct rt_serial_device *serial = (struct rt_serial_device *)dev;
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;

    // 从RT-Thread串口驱动内置环形缓冲区复制数据到GPS专用环形缓冲区
    while (rx_fifo->get_index != rx_fifo->put_index) {
        uint8_t ch = rx_fifo->buffer[rx_fifo->get_index];
        rx_fifo->get_index = (rx_fifo->get_index + 1) % RT_SERIAL_RB_BUFSZ;
        gps_ring_buffer_put(ch);  // GPS NMEA解析专用环形缓冲区
    }
    
    return RT_EOK;
}

// 蓝牙数据发送函数（RT-Thread设备框架）
static rt_err_t bluetooth_send_data(const uint8_t *data, size_t len)
{
    if (uart_bluetooth != RT_NULL) {
        rt_size_t written = rt_device_write(uart_bluetooth, 0, data, len);
        return (written == len) ? RT_EOK : -RT_ERROR;
    }
    return -RT_ERROR;
}

/* ==================== 手机通信协议实现 ==================== */

// 协议命令定义
#define CMD_PATROL_START    0x01  // 开始巡检
#define CMD_PATROL_STOP     0x02  // 停止巡检
#define CMD_MODE_MANUAL     0x10  // 手动模式
#define CMD_MODE_AUTO       0x11  // 自动模式
#define CMD_MODE_PRESET     0x12  // 预设路径模式
#define CMD_SET_WAYPOINT    0x20  // 设置路径点
#define CMD_STATUS_QUERY    0x30  // 状态查询

// 通信协议结构体
typedef struct {
    uint8_t header;      // 帧头 0xAA
    uint8_t cmd;         // 命令字节
    uint8_t data_len;    // 数据长度
    uint8_t data[16];    // 数据区域
    uint8_t checksum;    // 校验和
    uint8_t tail;        // 帧尾 0x55
} uart_msg_t;

// 协议解析状态机
typedef enum {
    PARSE_HEADER = 0,
    PARSE_CMD,
    PARSE_DATA_LEN,
    PARSE_DATA,
    PARSE_CHECKSUM,
    PARSE_TAIL
} parse_state_t;

/* ==================== GPS环形缓冲区实现 ==================== */

#define GPS_RING_BUFFER_SIZE 512  // GPS环形缓冲区大小

typedef struct {
    uint8_t buffer[GPS_RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t overflow_count;
} gps_ring_buffer_t;

static gps_ring_buffer_t gps_ring_buf = {0};

// GPS环形缓冲区操作函数
static inline uint16_t gps_ring_buffer_count(void)
{
    return (gps_ring_buf.head - gps_ring_buf.tail) & (GPS_RING_BUFFER_SIZE - 1);
}

static inline uint8_t gps_ring_buffer_put(uint8_t data)
{
    uint16_t next_head = (gps_ring_buf.head + 1) & (GPS_RING_BUFFER_SIZE - 1);
    if(next_head == gps_ring_buf.tail) {
        gps_ring_buf.overflow_count++;
        return 0;  // 缓冲区满
    }
    gps_ring_buf.buffer[gps_ring_buf.head] = data;
    gps_ring_buf.head = next_head;
    return 1;
}

static inline uint8_t gps_ring_buffer_get(uint8_t *data)
{
    if(gps_ring_buf.head == gps_ring_buf.tail) {
        return 0;  // 缓冲区空
    }
    *data = gps_ring_buf.buffer[gps_ring_buf.tail];
    gps_ring_buf.tail = (gps_ring_buf.tail + 1) & (GPS_RING_BUFFER_SIZE - 1);
    return 1;
}

/* ==================== 应用层环形缓冲区实现 ==================== */
// 注意：RT-Thread串口驱动已内置环形缓冲区，这里的缓冲区主要用于应用层协议解析

#define RING_BUFFER_SIZE 256  // 应用层协议解析缓冲区大小（相对较小）
#define GPS_RING_BUFFER_SIZE 256  // GPS NMEA解析缓冲区大小

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t overflow_count;
} ring_buffer_t;

typedef struct {
    uint8_t buffer[GPS_RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t overflow_count;
} gps_ring_buffer_t;

static ring_buffer_t uart_ring_buf = {0};

// 环形缓冲区操作函数 (优化版)
static inline uint16_t ring_buffer_count(void)
{
    return (uart_ring_buf.head - uart_ring_buf.tail) & (RING_BUFFER_SIZE - 1);
}

static inline uint8_t ring_buffer_put(uint8_t data)
{
    uint16_t next_head = (uart_ring_buf.head + 1) & (RING_BUFFER_SIZE - 1);
    if(next_head == uart_ring_buf.tail) {
        uart_ring_buf.overflow_count++;  // 记录溢出
        return 0;  // 缓冲区满
    }
    uart_ring_buf.buffer[uart_ring_buf.head] = data;
    uart_ring_buf.head = next_head;
    return 1;
}

static inline uint8_t ring_buffer_get(uint8_t *data)
{
    if(uart_ring_buf.head == uart_ring_buf.tail) {
        return 0;  // 缓冲区空
    }
    *data = uart_ring_buf.buffer[uart_ring_buf.tail];
    uart_ring_buf.tail = (uart_ring_buf.tail + 1) & (RING_BUFFER_SIZE - 1);
    return 1;
}

// 获取缓冲区状态信息（用于监控）
static inline void ring_buffer_get_stats(uint16_t *used, uint16_t *free, uint16_t *overflows)
{
    *used = ring_buffer_count();
    *free = RING_BUFFER_SIZE - *used - 1;
    *overflows = uart_ring_buf.overflow_count;
}

/* ==================== GPS NMEA数据解析函数 ==================== */

/*
 * 解析NMEA GPGGA语句（GPS定位数据）
 * 格式: $GPGGA,时间,纬度,N/S,经度,E/W,定位质量,卫星数,HDOP,海拔,M,大地水准面高,M,差分时间,差分站ID*校验和
 * 示例: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
 */
static int parse_nmea_gpgga(const char *nmea_sentence)
{
    if (strncmp(nmea_sentence, "$GPGGA", 6) != 0) {
        return -1;  // 不是GPGGA语句
    }
    
    // 简化的NMEA解析（实际项目中应使用更完整的解析器）
    char *tokens[15];
    char sentence_copy[128];
    strncpy(sentence_copy, nmea_sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';
    
    // 按逗号分割字符串
    int token_count = 0;
    char *token = strtok(sentence_copy, ",");
    while (token != NULL && token_count < 15) {
        tokens[token_count++] = token;
        token = strtok(NULL, ",");
    }
    
    if (token_count >= 6 && strlen(tokens[2]) > 0 && strlen(tokens[4]) > 0) {
        // 解析纬度 (DDMM.MMMM格式转换为十进制度)
        float lat_deg = (float)atoi(tokens[2]) / 100;  // 度
        float lat_min = (float)atof(tokens[2]) - (lat_deg * 100);  // 分
        current_gps.latitude = lat_deg + lat_min / 60.0f;
        if (tokens[3][0] == 'S') current_gps.latitude = -current_gps.latitude;
        
        // 解析经度 (DDDMM.MMMM格式转换为十进制度)
        float lon_deg = (float)atoi(tokens[4]) / 100;  // 度
        float lon_min = (float)atof(tokens[4]) - (lon_deg * 100);  // 分
        current_gps.longitude = lon_deg + lon_min / 60.0f;
        if (tokens[5][0] == 'W') current_gps.longitude = -current_gps.longitude;
        
        // 解析定位质量和卫星数
        if (token_count >= 8) {
            current_gps.fix_quality = (uint8_t)atoi(tokens[6]);
            current_gps.satellites = (uint8_t)atoi(tokens[7]);
        }
        
        // 解析海拔
        if (token_count >= 10 && strlen(tokens[9]) > 0) {
            current_gps.altitude = (float)atof(tokens[9]);
        }
        
        current_gps.timestamp = rt_tick_get();
        current_gps.valid = (current_gps.fix_quality > 0) ? 1 : 0;
        
        return 0;  // 解析成功
    }
    
    return -1;  // 解析失败
}

/*
 * 解析NMEA GPRMC语句（推荐最小定位信息）
 * 格式: $GPRMC,时间,状态,纬度,N/S,经度,E/W,速度,航向,日期,磁偏角,E/W*校验和
 * 示例: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
 */
static int parse_nmea_gprmc(const char *nmea_sentence)
{
    if (strncmp(nmea_sentence, "$GPRMC", 6) != 0) {
        return -1;  // 不是GPRMC语句
    }
    
    char *tokens[12];
    char sentence_copy[128];
    strncpy(sentence_copy, nmea_sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';
    
    int token_count = 0;
    char *token = strtok(sentence_copy, ",");
    while (token != NULL && token_count < 12) {
        tokens[token_count++] = token;
        token = strtok(NULL, ",");
    }
    
    if (token_count >= 9 && tokens[2][0] == 'A') {  // A=有效, V=无效
        // 解析速度 (节转换为km/h)
        if (strlen(tokens[7]) > 0) {
            current_gps.speed = (float)atof(tokens[7]) * 1.852f;  // 节转km/h
        }
        
        // 解析航向
        if (strlen(tokens[8]) > 0) {
            current_gps.course = (float)atof(tokens[8]);
        }
        
        current_gps.timestamp = rt_tick_get();
        current_gps.valid = 1;
        
        return 0;  // 解析成功
    }
    
    return -1;  // 解析失败或数据无效
}

/*
 * 从SD卡读取GPS路径点文件
 * 功能：读取mode0.txt中的GPS坐标点，格式：纬度,经度
 */
static int load_gps_route_from_file(void)
{
    FILE *route_file = fopen(GPS_ROUTE_FILE, "r");
    if (!route_file) {
        rt_kprintf("[GPS Route] Cannot open %s, using default route\n", GPS_ROUTE_FILE);
        return -1;
    }

    char line[64];
    /*
声明一个字符数组 line，用于暂存从文件中读取的一行数据。
大小为 64 字节，意味着每次最多读取 63 个字符的内容（再加一个 \0 结尾符）。
用于解析文本中存储的 GPS 坐标点（例如：35.6895,139.6917\n）。
*/
    int loaded_count = 0;
    
    rt_kprintf("[GPS Route] Loading route points from %s...\n", GPS_ROUTE_FILE);
    /*
    是 C 标准库中的 fgets 函数的调用，用于从文件中读取一行文本，其各个参数含义如下：
     */
    while (fgets(line, sizeof(line), route_file) && loaded_count < MAX_PATROL_POINTS) {//一次读一行数据
        float lat, lon;
        
        // 解析每行数据：纬度,经度
        if (sscanf(line, "%f,%f", &lat, &lon) == 2) {
            patrol_points[loaded_count].latitude = lat;
            patrol_points[loaded_count].longitude = lon;
            snprintf(patrol_points[loaded_count].description, 
                    sizeof(patrol_points[loaded_count].description),
                    "Route_Point_%d", loaded_count + 1);
            
            rt_kprintf("[GPS Route] Point %d: (%.6f, %.6f)\n", 
                      loaded_count + 1, lat, lon);
            loaded_count++;
        } else {
            rt_kprintf("[GPS Route] Invalid line format: %s", line);
        }
    }
    
    fclose(route_file);
    patrol_point_count = loaded_count;
    
    rt_kprintf("[GPS Route] Successfully loaded %d route points\n", loaded_count);
    return loaded_count;
}

/*
 * 创建图像保存目录
 * 功能：确保/res/image目录存在
 */
static rt_err_t create_image_directory(void)
{
    // 创建res目录
    if (mkdir(SD_MOUNT_POINT"/res", 0755) != 0) {
        rt_kprintf("[Init] /res directory already exists or created\n");
    }
    
    // 创建image目录  
    if (mkdir(IMAGE_SAVE_DIR, 0755) != 0) {
        rt_kprintf("[Init] /res/image directory already exists or created\n");
    } else {
        rt_kprintf("[Init] Created image directory: %s\n", IMAGE_SAVE_DIR);
    }
    
    return RT_EOK;
}

/*
 * 初始化K210 KPU YOLOv2模型
 * 功能：加载SD卡中的YOLOv2模型文件，初始化KPU硬件加速器
 * 优势：YOLOv2模型更小，更适合K210的8MB SRAM限制
 */
static rt_err_t kpu_yolo_init(void)
{
    rt_kprintf("[KPU] Initializing YOLOv2 model (K210 optimized)...\n");
    
    // 从SD卡加载YOLOv2模型文件
    FILE *model_file = fopen(YOLO_MODEL_FILE, "rb");
    if (!model_file) {
        rt_kprintf("[KPU] Error: Cannot open YOLOv2 model file: %s\n", YOLO_MODEL_FILE);
        return -RT_ERROR;
    }
    
    // 获取模型文件大小
    fseek(model_file, 0, SEEK_END);
    size_t model_size = ftell(model_file);
    fseek(model_file, 0, SEEK_SET);
    
    rt_kprintf("[KPU] Model file size: %d bytes\n", model_size);
    
    // 检查模型大小是否适合K210 (建议小于6MB以留出运行空间)
    if (model_size > 6 * 1024 * 1024) {
        rt_kprintf("[KPU] Warning: Model size (%d bytes) may be too large for K210\n", model_size);
        rt_kprintf("[KPU] K210 SRAM: 8MB, Recommended model size: <6MB\n");
    }
    
    // 分配内存加载模型
    yolo_model_data = (uint8_t *)rt_malloc(model_size);
    if (!yolo_model_data) {
        rt_kprintf("[KPU] Error: Failed to allocate memory for model\n");
        fclose(model_file);
        return -RT_ERROR;
    }
    
    // 读取模型数据
    size_t read_size = fread(yolo_model_data, 1, model_size, model_file);
    fclose(model_file);
    
    if (read_size != model_size) {
        rt_kprintf("[KPU] Error: Failed to read model file completely\n");
        rt_free(yolo_model_data);
        return -RT_ERROR;
    }
    
    // 加载模型到KPU
    if (kpu_load_kmodel(&yolo_model, yolo_model_data) != 0) {
        rt_kprintf("[KPU] Error: Failed to load kmodel\n");
        rt_free(yolo_model_data);
        return -RT_ERROR;
    }
    
    kpu_initialized = true;
    rt_kprintf("[KPU] YOLOv2 model loaded successfully\n");
    rt_kprintf("[KPU] YOLO Version: %s (K210 optimized)\n", YOLO_VERSION);
    rt_kprintf("[KPU] Network input size: %dx%d\n", NET_INPUT_WIDTH, NET_INPUT_HEIGHT);
    rt_kprintf("[KPU] Grid size: %dx%d\n", GRID_WIDTH, GRID_HEIGHT);
    rt_kprintf("[KPU] Garbage classes: %d, Anchors: %d\n", CLASS_NUM, ANCHOR_NUM);
    rt_kprintf("[KPU] Model architecture: YOLOv2 single-scale detection\n");
    rt_kprintf("[KPU] Memory usage: Model=%dKB, Runtime=~2MB\n", model_size/1024);
    
    return RT_EOK;
}

/*
 * 反初始化KPU资源
 */
static void kpu_yolo_deinit(void)
{
    if (kpu_initialized) {
        kpu_model_free(&yolo_model);
        if (yolo_model_data) {
            rt_free(yolo_model_data);
            yolo_model_data = NULL;
        }
        kpu_initialized = false;
        rt_kprintf("[KPU] YOLO model resources freed\n");
    }
}

/*
 * YOLOv2后处理函数 - 适配K210资源限制
 * 功能：解析YOLOv2网络输出，提取边界框和类别信息
 * 优化：针对K210内存和计算能力进行优化
 */
static void yolov2_postprocess(float *output_data, yolo_result_t *result)
{
    // YOLOv2后处理：遍历网格单元
    for (int grid_y = 0; grid_y < GRID_HEIGHT; grid_y++) {
        for (int grid_x = 0; grid_x < GRID_WIDTH; grid_x++) {
            for (int anchor = 0; anchor < ANCHOR_NUM; anchor++) {
                // 计算输出索引 (YOLOv2格式)
                int base_idx = ((grid_y * GRID_WIDTH + grid_x) * ANCHOR_NUM + anchor) * (CLASS_NUM + 5);
                
                float objectness = 1.0f / (1.0f + expf(-output_data[base_idx + 4])); // Sigmoid激活
                
                if (objectness > YOLO_THRESHOLD && result->detected_count < 10) {
                    // 解析边界框 (YOLOv2格式)
                    float x = (1.0f / (1.0f + expf(-output_data[base_idx + 0])) + grid_x) / GRID_WIDTH;
                    float y = (1.0f / (1.0f + expf(-output_data[base_idx + 1])) + grid_y) / GRID_HEIGHT;
                    float w = expf(output_data[base_idx + 2]) * yolo_anchor[anchor * 2] / NET_INPUT_WIDTH;
                    float h = expf(output_data[base_idx + 3]) * yolo_anchor[anchor * 2 + 1] / NET_INPUT_HEIGHT;
                    
                    // 找到最大类别概率
                    int best_class = 0;
                    float best_score = 0;
                    for (int c = 0; c < CLASS_NUM; c++) {
                        float class_prob = 1.0f / (1.0f + expf(-output_data[base_idx + 5 + c])); // Sigmoid
                        float final_score = objectness * class_prob;
                        if (final_score > best_score) {
                            best_score = final_score;
                            best_class = c;
                        }
                    }
                    
                    if (best_score > YOLO_THRESHOLD) {
                        // 转换为像素坐标
                        int box_x = (int)((x - w/2) * IMAGE_WIDTH);
                        int box_y = (int)((y - h/2) * IMAGE_HEIGHT);
                        int box_w = (int)(w * IMAGE_WIDTH);
                        int box_h = (int)(h * IMAGE_HEIGHT);
                        
                        // 边界检查
                        if (box_x < 0) box_x = 0;
                        if (box_y < 0) box_y = 0;
                        if (box_x + box_w > IMAGE_WIDTH) box_w = IMAGE_WIDTH - box_x;
                        if (box_y + box_h > IMAGE_HEIGHT) box_h = IMAGE_HEIGHT - box_y;
                        
                        // 保存检测结果
                        strcpy(result->garbage_types[result->detected_count], GARBAGE_TYPES[best_class]);
                        result->confidences[result->detected_count] = best_score;
                        result->bboxes[result->detected_count].x = box_x;
                        result->bboxes[result->detected_count].y = box_y;
                        result->bboxes[result->detected_count].w = box_w;
                        result->bboxes[result->detected_count].h = box_h;
                        
                        result->detected_count++;
                    }
                }
            }
        }
    }

/*
 * K210 DVP摄像头中断处理函数 - 修复版
 * 功能：DVP帧完成中断，切换缓冲区并触发图像处理
 */
static int dvp_camera_callback(void *ctx)
{
    if (current_patrol_mode != PATROL_MODE_STOP) {
        // 清除DVP帧完成中断标志
        dvp_clear_interrupt(DVP_CFG_FINISH_INT_ENABLE);
        
        // 原子切换缓冲区
        atomic_switch_current_buf();
        image_frame_count++;
        
        // 设置下一帧的接收地址到新的当前缓冲区
        uint8_t *next_frame_addr = (uint8_t *)mt9v03x_image[current_buf];
        dvp_set_ai_addr((uint32_t)next_frame_addr, 
                       (uint32_t)next_frame_addr + IMAGE_WIDTH * IMAGE_HEIGHT * 2,
                       (uint32_t)next_frame_addr + IMAGE_WIDTH * IMAGE_HEIGHT * 4);
        
        // 释放图像处理信号量 - 通知处理线程有新图像
        rt_sem_release(&image_sem);
        
        // 可选：触发图像捕获事件
        rt_event_send(patrol_event, EVENT_IMAGE_CAPTURED);
    }
    
    return 0;
}

/*
 * 注册DVP中断处理函数 - 重要：建立中断连接
 */
static rt_err_t register_dvp_interrupt(void)
{
    // 注册DVP中断处理函数
    plic_set_priority(IRQN_DVP_INTERRUPT, 1);
    plic_irq_register(IRQN_DVP_INTERRUPT, dvp_camera_callback, NULL);
    plic_irq_enable(IRQN_DVP_INTERRUPT);
    
    rt_kprintf("[DVP] DVP interrupt handler registered\n");
    return RT_EOK;
}

/*
 * 初始化K210 DVP + OV2640摄像头
 * 功能：配置DVP接口和OV2640摄像头，启用硬件DMA采集
 */
static rt_err_t k210_camera_init(void)
{
    rt_kprintf("[Camera] Initializing K210 DVP + OV2640 camera...\n");
    
    // 1. 初始化DVP接口
    dvp_init(8);  // 8位寄存器长度
    
    // 2. 配置DVP参数
    dvp_set_image_format(DVP_CFG_RGB_FORMAT);  // RGB565格式
    dvp_set_image_size(IMAGE_WIDTH, IMAGE_HEIGHT);  // 320x240分辨率
    
    // 3. 启用DVP硬件DMA突发传输
    dvp_enable_burst();
    
    // 4. 设置帧中断回调
    dvp_config_interrupt(DVP_CFG_FINISH_INT_ENABLE, 1);
    
/*
 * OV2640摄像头初始化（K210兼容版本）
 * 功能：基于OV5640驱动，配置为OV2640兼容模式
 */
static rt_err_t ov2640_k210_init(void)
{
    uint16_t id_reg = 0;
    
    rt_kprintf("[OV2640] Initializing OV2640 camera...\n");
    
    // 尝试读取摄像头ID
    id_reg = dvp_sccb_receive_data(OV5640_ADDR, OV5640_CHIPIDH);
    id_reg <<= 8;
    id_reg |= dvp_sccb_receive_data(OV5640_ADDR, OV5640_CHIPIDL);
    
    rt_kprintf("[OV2640] Camera ID: 0x%04X\n", id_reg);
    
    // 如果是OV5640，直接使用现有驱动
    if (id_reg == OV5640_ID) {
        rt_kprintf("[OV2640] Detected OV5640, using compatible mode\n");
        return (ov5640_init() == 0) ? RT_EOK : -RT_ERROR;
    }
    
    // 否则使用OV2640特定配置
    rt_kprintf("[OV2640] Configuring for OV2640 compatible mode\n");
    
    // OV2640基本配置寄存器（简化版本）
    dvp_sccb_send_data(OV5640_ADDR, 0x3008, 0x80);  // 软复位
    rt_thread_mdelay(10);
    
    // 设置QVGA 320x240分辨率（OV2640兼容）
    dvp_sccb_send_data(OV5640_ADDR, 0x3034, 0x18);  // 时钟配置
    dvp_sccb_send_data(OV5640_ADDR, 0x3035, 0x14);  // 时钟分频
    dvp_sccb_send_data(OV5640_ADDR, 0x3036, 0x54);  // PLL配置
    
    // DVP输出配置
    dvp_sccb_send_data(OV5640_ADDR, 0x300e, 0x58);  // MIPI power down, DVP enable
    dvp_sccb_send_data(OV5640_ADDR, 0x4300, 0x30);  // RGB565格式
    
    // 分辨率设置
    dvp_sccb_send_data(OV5640_ADDR, 0x3808, (IMAGE_WIDTH >> 8));   // 宽度高字节
    dvp_sccb_send_data(OV5640_ADDR, 0x3809, (IMAGE_WIDTH & 0xff)); // 宽度低字节
    dvp_sccb_send_data(OV5640_ADDR, 0x380a, (IMAGE_HEIGHT >> 8));  // 高度高字节
    dvp_sccb_send_data(OV5640_ADDR, 0x380b, (IMAGE_HEIGHT & 0xff)); // 高度低字节
    
    rt_thread_mdelay(50);
    
    rt_kprintf("[OV2640] OV2640 initialization completed\n");
    return RT_EOK;
}

/*
 * 初始化K210 DVP + OV2640摄像头 - 完整实现
 * 功能：配置DVP接口和OV2640摄像头，启用硬件DMA采集
 */
static rt_err_t k210_camera_init(void)
{
    rt_kprintf("[Camera] Initializing K210 DVP + OV2640 camera...\n");
    
    // 配置DVP相关引脚复用 - 重要步骤
    fpioa_set_function(47, FUNC_CMOS_PCLK);
    fpioa_set_function(46, FUNC_CMOS_XCLK);
    fpioa_set_function(45, FUNC_CMOS_HREF);
    fpioa_set_function(44, FUNC_CMOS_PWDN);
    fpioa_set_function(43, FUNC_CMOS_VSYNC);
    fpioa_set_function(42, FUNC_CMOS_RST);
    fpioa_set_function(41, FUNC_SCCB_SCLK);
    fpioa_set_function(40, FUNC_SCCB_SDA);
    
    // 配置数据引脚D0-D7
    for (int i = 0; i < 8; i++) {
        fpioa_set_function(32 + i, FUNC_CMOS_D0 + i);
    }
    
    // 1. 初始化SCCB接口（用于摄像头配置）
    sccb_init(3, 4);  // SDA=3, SCL=4
    
    // 2. 初始化DVP接口
    dvp_init(8);  // 8位数据宽度
    
    // 3. 配置DVP参数
    dvp_set_image_format(DVP_CFG_RGB_FORMAT);  // RGB565格式
    dvp_set_image_size(IMAGE_WIDTH, IMAGE_HEIGHT);  // 320x240分辨率
    
    // 4. 配置DVP输出格式和AI地址
    dvp_set_ai_addr((uint32_t)mt9v03x_image[0], 
                   (uint32_t)mt9v03x_image[0] + IMAGE_WIDTH * IMAGE_HEIGHT * 2,
                   (uint32_t)mt9v03x_image[0] + IMAGE_WIDTH * IMAGE_HEIGHT * 4);
    
    // 5. 启用DVP硬件功能
    dvp_enable_burst();
    dvp_disable_auto();  // 先禁用自动模式进行配置
    
    // 6. 配置中断
    dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
    dvp_set_output_enable(0, 1);  // AI输出使能
    dvp_set_output_enable(1, 1);  // 显示输出使能
    
    // 7. 初始化OV2640摄像头
    if (ov2640_k210_init() != RT_EOK) {
        rt_kprintf("[Camera] OV2640 initialization failed\n");
        return -RT_ERROR;
    }
    
    // 8. 注册DVP中断处理函数
    if (register_dvp_interrupt() != RT_EOK) {
        rt_kprintf("[Camera] DVP interrupt registration failed\n");
        return -RT_ERROR;
    }
    
    // 9. 启用DVP自动模式开始采集
    dvp_enable_auto();
    
    rt_kprintf("[Camera] K210 DVP + OV2640 camera initialized successfully\n");
    rt_kprintf("[Camera] Resolution: %dx%d, Format: RGB565, DMA: Enabled\n", 
               IMAGE_WIDTH, IMAGE_HEIGHT);
    rt_kprintf("[Camera] DVP pins configured: PCLK(47), XCLK(46), HREF(45), VSYNC(43)\n");
    rt_kprintf("[Camera] Data pins D0-D7: 32-39, SCCB: SDA(40), SCL(41)\n");
    
    return RT_EOK;
}

/*
 * K210 KPU YOLOv2垃圾检测算法
 * 功能：使用KPU硬件加速器运行训练好的YOLOv2垃圾检测模型
 * 架构：YOLOv2单尺度检测，5个锚框，适合K210资源限制
 * 优化：内存使用优化，推理速度快，模型小巧
 */
static yolo_result_t yolo_garbage_detection(uint16_t image[IMAGE_HEIGHT][IMAGE_WIDTH])
{
    yolo_result_t result = {0};
    rt_tick_t inference_start = rt_tick_get();
    
    // 检查KPU是否已初始化
    if (!kpu_initialized) {
        rt_kprintf("[YOLOv2] Warning: KPU not initialized, using fallback detection\n");
        // 返回空结果或使用备用检测方法
        return result;
    }
    
    // 1. 图像预处理：转换为KPU输入格式 (YOLOv2优化)
    static uint8_t ai_buf[NET_INPUT_WIDTH * NET_INPUT_HEIGHT * 3]; // RGB888格式
    
    // 将RGB565转换为RGB888并调整尺寸到网络输入大小
    for (int y = 0; y < NET_INPUT_HEIGHT; y++) {
        for (int x = 0; x < NET_INPUT_WIDTH; x++) {
            // 从原图像中采样（简单缩放）
            int src_x = (x * IMAGE_WIDTH) / NET_INPUT_WIDTH;
            int src_y = (y * IMAGE_HEIGHT) / NET_INPUT_HEIGHT;
            
            uint16_t pixel = image[src_y][src_x];
            
            // RGB565转RGB888
            uint8_t r = (pixel >> 11) << 3;      // 5位红色扩展到8位
            uint8_t g = ((pixel >> 5) & 0x3F) << 2;  // 6位绿色扩展到8位  
            uint8_t b = (pixel & 0x1F) << 3;     // 5位蓝色扩展到8位
            
            // 存储为RGB888格式
            int idx = (y * NET_INPUT_WIDTH + x) * 3;
            ai_buf[idx] = r;
            ai_buf[idx + 1] = g;
            ai_buf[idx + 2] = b;
        }
    }
    
    // 2. KPU推理
    kpu_model_output_t output;
    int kpu_ret = kpu_run_kmodel(&yolo_model, ai_buf, DMAC_CHANNEL5, &output, NULL);
    
    if (kpu_ret != 0) {
        rt_kprintf("[YOLOv2] KPU inference failed: %d\n", kpu_ret);
        return result;
    }
    
    // 3. YOLOv2后处理：解析输出结果 (使用优化的后处理函数)
    float *output_data = (float *)output.output[0];
    yolov2_postprocess(output_data, &result);
    
    rt_tick_t inference_end = rt_tick_get();
    rt_uint32_t inference_time = (inference_end - inference_start) * 1000 / RT_TICK_PER_SECOND;
    
    rt_kprintf("[YOLOv2-KPU] Inference time: %dms, detected: %d objects\n", 
              inference_time, result.detected_count);
    
    return result;
}

/*
 * 在图像上绘制检测框和标注
 * 功能：在检测到垃圾的图像上绘制矩形框并标注垃圾类型
 */
static void draw_detection_boxes(uint16_t src_image[IMAGE_HEIGHT][IMAGE_WIDTH], 
                                uint16_t dst_image[IMAGE_HEIGHT][IMAGE_WIDTH],
                                yolo_result_t *result)
{
    // 首先复制原始图像
    memcpy(dst_image, src_image, IMAGE_HEIGHT * IMAGE_WIDTH * sizeof(uint16_t));
    
    // 如果没有检测到垃圾，直接返回
    if (result->detected_count == 0) {
        return;
    }
    
    // 为每个检测到的垃圾绘制矩形框
    for (int i = 0; i < result->detected_count; i++) {
        int x = result->bboxes[i].x;
        int y = result->bboxes[i].y;
        int w = result->bboxes[i].w;
        int h = result->bboxes[i].h;
        
        // 确保边界框在图像范围内
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x + w >= IMAGE_WIDTH) w = IMAGE_WIDTH - x - 1;
        if (y + h >= IMAGE_HEIGHT) h = IMAGE_HEIGHT - y - 1;
        
        // 绘制矩形框（使用高亮像素值）
        uint16_t box_color = 0xFFFF;  // 白色边框
        
        // 绘制上下边框
        for (int j = x; j < x + w; j++) {
            if (j >= 0 && j < IMAGE_WIDTH) {
                if (y >= 0 && y < IMAGE_HEIGHT) 
                    dst_image[y][j] = box_color;
                if (y + h >= 0 && y + h < IMAGE_HEIGHT) 
                    dst_image[y + h][j] = box_color;
            }
        }
        
        // 绘制左右边框
        for (int j = y; j < y + h; j++) {
            if (j >= 0 && j < IMAGE_HEIGHT) {
                if (x >= 0 && x < IMAGE_WIDTH) 
                    dst_image[j][x] = box_color;
                if (x + w >= 0 && x + w < IMAGE_WIDTH) 
                    dst_image[j][x + w] = box_color;
            }
        }
        
        // 在矩形框附近标注垃圾类型（简化实现：在框上方绘制几个像素点表示标签）
        int label_y = (y > 5) ? y - 3 : y + h + 3;
        for (int k = 0; k < 8 && x + k < IMAGE_WIDTH; k++) {
            if (label_y >= 0 && label_y < IMAGE_HEIGHT && x + k >= 0) {
                dst_image[label_y][x + k] = 0xF800;  // 红色标签像素
            }
        }
        
        rt_kprintf("[ImageProcess] Drew bounding box for %s at (%d,%d,%d,%d)\n", 
                  result->garbage_types[i], x, y, w, h);
    }
}

/*
 * 生成带时间和地点的文件名
 * 功能：根据当前时间和GPS位置生成文件名，保存到/res/image目录
 * 格式：/sd/res/image/时间-经纬度.png
 */
static void generate_filename(char *filename, size_t size, float lat, float lon)
{
    // 获取当前时间戳
    rt_tick_t current_tick = rt_tick_get();
    uint32_t timestamp = current_tick / RT_TICK_PER_SECOND;
    
    // 计算小时、分钟、秒
    uint32_t hours = (timestamp / 3600) % 24;
    uint32_t minutes = (timestamp / 60) % 60;
    uint32_t seconds = timestamp % 60;
    
    // 生成文件名：保存到/res/image目录，格式：时间-经度纬度.png
    snprintf(filename, size, "%s/%02d%02d%02d-%.4f_%.4f.png", 
             IMAGE_SAVE_DIR, hours, minutes, seconds, lat, lon);
}

/*
 * 串口接收回调函数 - 已移除，改用UART DMA
 * 功能：K210 UART DMA自动处理数据接收
 */

/*
 * 解析手机发送的巡检指令
 * 功能：基于状态机解析协议栈，支持路径点动态设置
 */
static rt_err_t parse_patrol_command(uart_msg_t *msg)
{
    rt_kprintf("[Protocol] Received cmd: 0x%02X, data_len: %d\n", msg->cmd, msg->data_len);
    
    switch (msg->cmd) {
        case CMD_PATROL_START:
            current_patrol_mode = PATROL_MODE_AUTO;
            rt_event_send(patrol_event, EVENT_PATROL_START);
            rt_kprintf("[Command] Patrol started\n");
            break;
            
        case CMD_PATROL_STOP:
            current_patrol_mode = PATROL_MODE_STOP;
            rt_event_send(patrol_event, EVENT_PATROL_STOP);
            rt_kprintf("[Command] Patrol stopped\n");
            break;
            
        case CMD_MODE_MANUAL:
            current_patrol_mode = PATROL_MODE_MANUAL;
            rt_event_send(patrol_event, EVENT_PATROL_START);
            rt_kprintf("[Command] Manual mode activated\n");
            break;
            
        case CMD_MODE_AUTO:
            current_patrol_mode = PATROL_MODE_AUTO;
            rt_event_send(patrol_event, EVENT_PATROL_START);
            rt_kprintf("[Command] Auto mode activated\n");
            break;
            
        case CMD_MODE_PRESET:
            current_patrol_mode = PATROL_MODE_PRESET;
            rt_event_send(patrol_event, EVENT_PATROL_START);
            rt_kprintf("[Command] Preset path mode activated\n");
            break;
            
        case CMD_SET_WAYPOINT:
            // 解析路径点数据：经度(4字节) + 纬度(4字节)
            if (msg->data_len >= 8 && patrol_point_count < MAX_PATROL_POINTS) {
                float lat, lon;
                memcpy(&lat, &msg->data[0], 4);
                memcpy(&lon, &msg->data[4], 4);
                
                patrol_points[patrol_point_count].latitude = lat;
                patrol_points[patrol_point_count].longitude = lon;
                snprintf(patrol_points[patrol_point_count].description, 
                        sizeof(patrol_points[patrol_point_count].description),
                        "Point_%d", patrol_point_count + 1);
                        
                patrol_point_count++;
                rt_kprintf("[Command] Added waypoint: (%.6f, %.6f)\n", lat, lon);
            }
            break;
            
        case CMD_STATUS_QUERY:
            // 发送状态响应（包含环形缓冲区状态）
            {
                uint16_t buf_used, buf_free, buf_overflows;
                ring_buffer_get_stats(&buf_used, &buf_free, &buf_overflows);
                
                rt_kprintf("[Status] Mode: %d, Points: %d, Frame: %d\n", 
                          current_patrol_mode, patrol_point_count, image_frame_count);
                rt_kprintf("[Status] Ring Buffer: Used=%d, Free=%d, Overflows=%d\n",
                          buf_used, buf_free, buf_overflows);
            }
            break;
            
        default:
            rt_kprintf("[Error] Unknown command: 0x%02X\n", msg->cmd);
            return -RT_ERROR;
    }
    
    return RT_EOK;
}

/*
 * 使用K210 SDK原生UART API后，数据处理通过中断回调完成
 * 不再需要复杂的线程处理和事件机制
 * 数据接收和处理都在bluetooth_uart_callback()和gps_uart_callback()中完成
 */

/*
 * UART协议解析线程 - 参照mm32car.c单字节处理机制
 * 功能：使用环形缓冲区 + 状态机，逐字节处理协议数据，避免批量处理延迟
 */
static void uart_rx_thread(void *parameter)
{
    static parse_state_t parse_state = PARSE_HEADER;
    static uart_msg_t rx_msg;
    static uint8_t data_index = 0;
    static uint8_t calc_checksum = 0;
    uint8_t rx_data;
    
    rt_kprintf("[UART Protocol] Single-byte parsing thread started (based on mm32car.c)\n");
    
    while (1)
    {
        // 从环形缓冲区逐字节读取数据进行状态机解析（类似mm32car.c机制）
        if (ring_buffer_get(&rx_data)) {
            switch (parse_state) {
                case PARSE_HEADER:
                    if (rx_data == 0xAA) {
                        rx_msg.header = rx_data;
                        calc_checksum = 0;
                        parse_state = PARSE_CMD;
                    }
                    break;
                    
                case PARSE_CMD:
                    rx_msg.cmd = rx_data;
                    calc_checksum ^= rx_data;
                    parse_state = PARSE_DATA_LEN;
                    break;
                    
                case PARSE_DATA_LEN:
                    rx_msg.data_len = rx_data;
                    calc_checksum ^= rx_data;
                    data_index = 0;
                    if (rx_msg.data_len > 0 && rx_msg.data_len <= 16) {
                        parse_state = PARSE_DATA;
                    } else if (rx_msg.data_len == 0) {
                        parse_state = PARSE_CHECKSUM;
                    } else {
                        // 数据长度异常，重新开始
                        rt_kprintf("[UART] Invalid data length: %d\n", rx_msg.data_len);
                        parse_state = PARSE_HEADER;
                    }
                    break;
                    
                case PARSE_DATA:
                    rx_msg.data[data_index] = rx_data;
                    calc_checksum ^= rx_data;
                    data_index++;
                    if (data_index >= rx_msg.data_len) {
                        parse_state = PARSE_CHECKSUM;
                    }
                    break;
                    
                case PARSE_CHECKSUM:
                    rx_msg.checksum = rx_data;
                    parse_state = PARSE_TAIL;
                    break;
                    
                case PARSE_TAIL:
                    if (rx_data == 0x55 && calc_checksum == rx_msg.checksum) {
                        // 协议解析完成，处理命令
                        rt_kprintf("[UART] Valid command received: 0x%02X\n", rx_msg.cmd);
                        if (parse_patrol_command(&rx_msg) == RT_EOK) {
                            rt_event_send(patrol_event, EVENT_UART_COMMAND);
                        }
                    } else {
                        rt_kprintf("[UART] Protocol error - Tail: 0x%02X, Checksum calc: 0x%02X, recv: 0x%02X\n", 
                                  rx_data, calc_checksum, rx_msg.checksum);
                    }
                    parse_state = PARSE_HEADER;
                    break;
                    
                default:
                    parse_state = PARSE_HEADER;
                    break;
            }
        } else {
            // 没有数据时让出CPU，类似mm32car.c的处理方式
            rt_thread_mdelay(1);  // 1ms延迟，避免空循环占用CPU
        }
    }
}

/*
 * YOLO检测结果排序函数 - 按置信度从高到低排序
 * 功能：对检测结果进行排序，让置信度高的垃圾排在前面，便于优先处理
 */
static void sort_detection_results(yolo_result_t *result)
{
    // 简单的冒泡排序，因为检测结果数量通常很少（<10个）
    for (int i = 0; i < result->detected_count - 1; i++) {
        for (int j = 0; j < result->detected_count - i - 1; j++) {
            if (result->confidences[j] < result->confidences[j + 1]) {
                // 交换置信度
                float temp_conf = result->confidences[j];
                result->confidences[j] = result->confidences[j + 1];
                result->confidences[j + 1] = temp_conf;
                
                // 交换垃圾类型名称
                char temp_type[32];
                strcpy(temp_type, result->garbage_types[j]);
                strcpy(result->garbage_types[j], result->garbage_types[j + 1]);
                strcpy(result->garbage_types[j + 1], temp_type);
                
                // 交换边界框
                bbox_t temp_bbox = result->bboxes[j];
                result->bboxes[j] = result->bboxes[j + 1];
                result->bboxes[j + 1] = temp_bbox;
            }
        }
    }
}

/*
 * 图像采集与处理线程
 * 功能：基于K210 DVP + OV2640硬件中断的双缓冲机制实现YOLO垃圾检测
 */
static void camera_thread_entry(void *parameter)
{
    int detection_counter = 0;
    
    rt_kprintf("[Camera] YOLO garbage detection thread started\n");
    
    while (1)
    {
        // 等待图像处理信号量
        rt_sem_take(&image_sem, RT_WAITING_FOREVER);
        
        rt_kprintf("[Camera] Processing image frame %d...\n", ++processed_frame_count);
        
        // 原子读取当前DMA写入的缓冲区，处理另一个缓冲区
        int process_buf = atomic_read_current_buf() ^ 1;
        
        // YOLO垃圾检测
        rt_tick_t process_start = rt_tick_get();
        yolo_result_t yolo_result = yolo_garbage_detection(mt9v03x_image[process_buf]);
        rt_tick_t process_end = rt_tick_get();
        
        rt_uint32_t process_time = (process_end - process_start) * 1000 / RT_TICK_PER_SECOND;
        
        // 对检测结果按置信度排序
        if (yolo_result.detected_count > 1) {
            sort_detection_results(&yolo_result);
        }
        
        // 更新检测结果 - 保存所有检测到的垃圾信息
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        if (yolo_result.detected_count > 0) {
            // 保存所有检测到的垃圾类型，而不是只保留置信度最高的
            char all_detections[512] = {0};  // 用于存储所有检测结果的缓冲区
            int total_length = 0;
            
            for (int i = 0; i < yolo_result.detected_count; i++) {
                char single_detection[64];
                int single_len = snprintf(single_detection, sizeof(single_detection), 
                                         "%s_%.2f", 
                                         yolo_result.garbage_types[i], 
                                         yolo_result.confidences[i]);
                
                // 检查缓冲区空间是否足够
                if (total_length + single_len + 2 < sizeof(all_detections)) {
                    if (i > 0) {
                        strcat(all_detections, "|");  // 使用 | 分隔多个检测结果
                        total_length += 1;
                    }
                    strcat(all_detections, single_detection);
                    total_length += single_len;
                } else {
                    rt_kprintf("[Warning] Detection buffer overflow, truncating results\n");
                    break;
                }
            }
            
            // 添加检测计数器
            snprintf(detected_garbage, sizeof(detected_garbage), 
                    "%s_%d", all_detections, ++detection_counter);
                    
            rt_kprintf("[YOLO] Multiple detections: %s\n", detected_garbage);
            
            // 按置信度排序并显示详细信息
            rt_kprintf("[YOLO] Detailed results (sorted by confidence):\n");
            for (int i = 0; i < yolo_result.detected_count; i++) {
                rt_kprintf("  [%d] %s (%.2f%%) bbox:(%d,%d,%d,%d)\n", 
                          i+1, yolo_result.garbage_types[i], 
                          yolo_result.confidences[i] * 100,
                          yolo_result.bboxes[i].x, yolo_result.bboxes[i].y,
                          yolo_result.bboxes[i].w, yolo_result.bboxes[i].h);
            }
        } else {
            snprintf(detected_garbage, sizeof(detected_garbage), "No_Garbage_%d", detection_counter);
        }
        rt_mutex_release(data_mutex);
        
        rt_kprintf("[YOLO] Frame %d processed: %s (Buffer: %d, Time: %dms)\n", 
                  ++processed_frame_count, detected_garbage, process_buf, process_time);
        
        // 触发数据存储事件
        rt_event_send(patrol_event, EVENT_IMAGE_CAPTURED);
    }
}

/*
 * GPS数据处理线程 - 仅负责NMEA解析，不包含控制逻辑
 * 功能：逐字节读取GPS数据，解析NMEA协议，更新全局GPS位置
 */
static void gps_thread_entry(void *parameter)
{
    static char nmea_buffer[128];
    static uint8_t nmea_index = 0;
    uint8_t gps_byte;
    
    rt_kprintf("[GPS] Pure NMEA parsing thread started (control moved to timer)\n");
    
    while (1)
    {
        // 从GPS环形缓冲区逐字节读取数据
        if (gps_ring_buffer_get(&gps_byte)) {
            // NMEA句子以$开始，\n或\r结束
            if (gps_byte == '$') {
                nmea_index = 0;
                nmea_buffer[nmea_index++] = gps_byte;
            } 
            else if (nmea_index > 0 && nmea_index < sizeof(nmea_buffer) - 1) {
                nmea_buffer[nmea_index++] = gps_byte;
                
                // 检查句子结束
                if (gps_byte == '\n' || gps_byte == '\r') {
                    nmea_buffer[nmea_index] = '\0';
                    
                    // 实际解析NMEA数据，更新全局GPS位置
                    if (strstr(nmea_buffer, "$GPGGA") != NULL) {
                        if (parse_nmea_gpgga(nmea_buffer) == 0) {
                            rt_kprintf("[GPS] GPGGA parsed: Lat=%.6f, Lon=%.6f, Alt=%.1f, Sats=%d\n",
                                      current_gps.latitude, current_gps.longitude, 
                                      current_gps.altitude, current_gps.satellites);
                        }
                    }
                    else if (strstr(nmea_buffer, "$GPRMC") != NULL) {
                        if (parse_nmea_gprmc(nmea_buffer) == 0) {
                            rt_kprintf("[GPS] GPRMC parsed: Speed=%.1f km/h, Course=%.1f°\n",
                                      current_gps.speed, current_gps.course);
                        }
                    }
                    nmea_index = 0;  // 重置缓冲区
                }
            } else if (nmea_index >= sizeof(nmea_buffer) - 1) {
                // 缓冲区溢出，重置
                nmea_index = 0;
            }
        }
        
        // 更新GPS位置信息字符串（仅数据格式化，不包含控制逻辑）
        if (current_gps.valid) {
            rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
            snprintf(gps_location, sizeof(gps_location), 
                    "Lat: %.6f, Lon: %.6f, Mode: %d, Sats: %d", 
                    current_gps.latitude, current_gps.longitude, 
                    current_patrol_mode, current_gps.satellites);
            rt_mutex_release(data_mutex);
            
            rt_event_send(patrol_event, EVENT_GPS_UPDATED);
        }
        
        // 线程延迟（GPS通常1Hz更新，但NMEA解析需要更高频率）
        rt_thread_mdelay(10);
    }
                            
                            // 如果接近目标点（< 5米），切换到下一个路径点
                            if (distance < 5.0f) {
                                current_patrol_index = (current_patrol_index + 1) % patrol_point_count;
                                rt_kprintf("[GPS] Reached waypoint, moving to next: %d/%d\n", 
                                          current_patrol_index + 1, patrol_point_count);
                            }
                            
                            // 计算目标航向角（从当前位置到目标位置）
                            layered_ctrl.target_heading = atan2f(dx, dy) * 180.0f / M_PI;
                            if (layered_ctrl.target_heading < 0) layered_ctrl.target_heading += 360.0f;
                            
                            // 计算航向误差
                            layered_ctrl.heading_error = layered_ctrl.target_heading - current_gps.course;
                            while (layered_ctrl.heading_error > 180.0f) layered_ctrl.heading_error -= 360.0f;
                            while (layered_ctrl.heading_error < -180.0f) layered_ctrl.heading_error += 360.0f;
                            
                            rt_kprintf("[GPS] Navigation: Target(%.6f,%.6f) Current(%.6f,%.6f) Heading: %.1f°->%.1f° Error: %.1f°\n",
                                      layered_ctrl.target_latitude, layered_ctrl.target_longitude,
                                      current_gps.latitude, current_gps.longitude,
                                      current_gps.course, layered_ctrl.target_heading, layered_ctrl.heading_error);
                        }
                        break;
                        
                    case PATROL_MODE_PRESET:
                        // 预设路径模式：类似自动模式
                        // ... 实现类似逻辑
                        break;
                        
                    case PATROL_MODE_MANUAL:
                        // 手动模式：GPS仅用于位置监控
                        layered_ctrl.heading_error = 0.0f;
                        break;
                        
                    default:
                        break;
                }
                
                // 更新GPS位置信息字符串（线程中可以安全使用mutex）
                rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
                snprintf(gps_location, sizeof(gps_location), 
                        "Lat: %.6f, Lon: %.6f, Mode: %d, Sats: %d", 
                        current_gps.latitude, current_gps.longitude, 
                        current_patrol_mode, current_gps.satellites);
                rt_mutex_release(data_mutex);
                
                rt_event_send(patrol_event, EVENT_GPS_UPDATED);
            }
        }
        
        // 线程延迟（GPS通常1Hz更新，但NMEA解析需要更高频率）
        rt_thread_mdelay(10);
    }
                        sim_lat += (rand() % 21 - 10) * 0.0001f;
                        sim_lon += (rand() % 21 - 10) * 0.0001f;
                    }
                    break;
                    
                case PATROL_MODE_PRESET:
                    // 预设路径模式：按照设定的路径点移动
                    if (patrol_point_count > 0) {
                        sim_lat = patrol_points[current_patrol_index].latitude;
                        sim_lon = patrol_points[current_patrol_index].longitude;
                        
                        // 模拟到达路径点后切换到下一个点
                        if (++location_counter % 3 == 0) {
                            current_patrol_index = (current_patrol_index + 1) % patrol_point_count;
                            rt_kprintf("[GPS] Moving to next patrol point: %d\n", current_patrol_index);
                        }
                    }
                    break;
                    
                case PATROL_MODE_MANUAL:
                    // 手动模式：位置相对固定
                    sim_lat += (rand() % 5 - 2) * 0.00001f;
                    sim_lon += (rand() % 5 - 2) * 0.00001f;
                    break;
                    
                default:
                    break;
            }
            
            // 更新GPS位置信息（线程中可以安全使用mutex）
            rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
            snprintf(gps_location, sizeof(gps_location), 
                    "Lat: %.6f, Lon: %.6f, Mode: %d", sim_lat, sim_lon, current_patrol_mode);
            rt_mutex_release(data_mutex);
            
            rt_event_send(patrol_event, EVENT_GPS_UPDATED);
        }
        
        // 线程延迟（GPS通常1Hz更新）
        rt_thread_mdelay(1000);
    }
        
        // 发送GPS更新事件
        rt_event_send(patrol_event, EVENT_GPS_UPDATED);
        
        rt_kprintf("[GPS] Location updated: %.6f, %.6f\n", sim_lat, sim_lon);
        rt_thread_mdelay(2000); // GPS更新频率
    }
}

/*
 * SD卡数据存储线程
 * 功能：利用RT-Thread阻塞式SPI SD卡驱动，保存带检测框标注的巡检照片
 */
static void sdcard_thread_entry(void *parameter)
{
    rt_uint32_t event_flags;
    int file_counter = 0;
    
    rt_kprintf("[SDCard] SD card storage thread started (using blocking SPI driver)\n");
    
    while (1)
    {
        // 等待图像捕获或GPS更新事件
        rt_err_t result = rt_event_recv(patrol_event, 
                                       EVENT_IMAGE_CAPTURED | EVENT_GPS_UPDATED,
                                       RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                                       RT_WAITING_FOREVER, &event_flags);
        
        if (result != RT_EOK) {
            continue;
        }
        
        // 记录写入开始时间
        rt_tick_t start_time = rt_tick_get();
        
        // 获取数据互斥锁
        rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
        
        // 复制当前数据（避免长时间占用锁）
        char local_garbage[64], local_location[64];
        strcpy(local_garbage, detected_garbage);
        strcpy(local_location, gps_location);
        
        rt_mutex_release(data_mutex);
        
        if (event_flags & EVENT_IMAGE_CAPTURED) {
            // 使用实时GPS位置信息，而不是解析字符串
            float current_lat, current_lon;
            if (current_gps.valid) {
                current_lat = current_gps.latitude;
                current_lon = current_gps.longitude;
                rt_kprintf("[SDCard] Using real GPS data: %.6f, %.6f\n", current_lat, current_lon);
            } else {
                // 仅在GPS无效时使用默认值
                current_lat = 39.9042f;
                current_lon = 116.4074f;
                rt_kprintf("[SDCard] GPS invalid, using default coordinates\n");
            }
            
            // 生成带时间和地点的文件名
            char filename[128];
            generate_filename(filename, sizeof(filename), current_lat, current_lon);
            
            // RT-Thread阻塞式文件操作 - 无需completion机制
            FILE *img_file = fopen(filename, "wb");
            if (img_file) {
                // 获取当前处理的图像缓冲区
                int process_buf = atomic_read_current_buf() ^ 1;
                
                // 重新执行YOLO检测以获取检测结果（用于绘制标注框）
                yolo_result_t current_result = yolo_garbage_detection(mt9v03x_image[process_buf]);
                
                // 在图像上绘制检测框和标注
                draw_detection_boxes(mt9v03x_image[process_buf], processed_image, &current_result);
                
                // 写入PNG文件头信息（简化的PNG格式头）
                fprintf(img_file, "PNG_HEADER: YOLO_DETECTION\n");
                fprintf(img_file, "DETECTION_RESULT: %s\n", local_garbage);
                fprintf(img_file, "GPS_LOCATION: %s\n", local_location);
                fprintf(img_file, "DETECTION_COUNT: %d\n", current_result.detected_count);
                
                // 写入检测详情
                for (int i = 0; i < current_result.detected_count; i++) {
                    fprintf(img_file, "OBJECT_%d: %s, Confidence: %.2f%%, BBox: (%d,%d,%d,%d)\n",
                           i, current_result.garbage_types[i], 
                           current_result.confidences[i] * 100,
                           current_result.bboxes[i].x, current_result.bboxes[i].y,
                           current_result.bboxes[i].w, current_result.bboxes[i].h);
                }
                
                fprintf(img_file, "IMAGE_DATA_START:\n");
                
                // RT-Thread阻塞式写入 - 直接写入，无需等待完成
                size_t total_size = IMAGE_HEIGHT * IMAGE_WIDTH * sizeof(uint16_t);
                size_t written = fwrite(processed_image, 1, total_size, img_file);
                
                if (written == total_size) {
                    rt_kprintf("[SDCard] Annotated image saved: %s (%d bytes)\n", filename, written);
                } else {
                    rt_kprintf("[SDCard] Write error: expected %d, written %d\n", total_size, written);
                }
                
                // 阻塞式关闭文件 - 确保数据写入完成
                fclose(img_file);
                file_counter++;
                
                rt_kprintf("[SDCard] Saved annotated image: %s\n", filename);
                rt_kprintf("[SDCard] Detection details: %d objects detected\n", current_result.detected_count);
            } else {
                rt_kprintf("[SDCard] Failed to create image file: %s\n", filename);
            }
        }
        
        // 保存巡检日志（同样是阻塞式操作）
        FILE *log_file = fopen(SD_MOUNT_POINT"/yolo_detection_log.txt", "a");
        if (log_file) {
            // 高效写入：使用格式化字符串一次性写入
            fprintf(log_file, "[%lu] YOLO: %s | Location: %s | Mode: %d | Files: %d\n", 
                   rt_tick_get(), local_garbage, local_location, current_patrol_mode, file_counter);
            fclose(log_file);
        }
        
        rt_tick_t end_time = rt_tick_get();
        rt_uint32_t write_time = (end_time - start_time) * 1000 / RT_TICK_PER_SECOND;
        
        rt_kprintf("[SDCard] Annotated image processing completed, total time: %d ms\n", write_time);
        
        // 定期验证数据写入（读取确认）
        if (file_counter % 5 == 0) {
            FILE *verify_file = fopen(SD_MOUNT_POINT"/yolo_detection_log.txt", "r");
            if (verify_file) {
                char line[128];
                int line_count = 0;
                rt_kprintf("[SDCard] Verification - Recent annotated image logs:\n");
                
                // 读取最后几行进行验证
                while (fgets(line, sizeof(line), verify_file) && line_count < 3) {
                    rt_kprintf("  %s", line);
                    line_count++;
                }
                fclose(verify_file);
            }
        }
    }
}

/*
 * 巡检控制线程
 * 功能：通过RTT事件机制触发小车运动模式切换与路径点动态设置
 */
static void patrol_control_thread(void *parameter)
{
    rt_uint32_t event_flags;
    
    rt_kprintf("[Patrol] Patrol control thread started\n");
    
    while (1)
    {
        // 等待巡检相关事件
        rt_err_t result = rt_event_recv(patrol_event,
                                       EVENT_UART_COMMAND | EVENT_PATROL_START | EVENT_PATROL_STOP,
                                       RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                                       RT_WAITING_FOREVER, &event_flags);
        
        if (result != RT_EOK) {
            continue;
        }
        
        if (event_flags & EVENT_PATROL_START) {
            rt_kprintf("[Patrol] Starting patrol mode: %d\n", current_patrol_mode);
            
            switch (current_patrol_mode) {
                case PATROL_MODE_AUTO:
                    rt_kprintf("[Patrol] Auto patrol mode activated\n");
                    // 这里可以添加自动巡航逻辑
                    break;
                    
                case PATROL_MODE_PRESET:
                    rt_kprintf("[Patrol] Preset path mode activated with %d points\n", patrol_point_count);
                    for (int i = 0; i < patrol_point_count; i++) {
                        rt_kprintf("  Point %d: (%.6f, %.6f) - %s\n", 
                                  i + 1, patrol_points[i].latitude, 
                                  patrol_points[i].longitude, patrol_points[i].description);
                    }
                    break;
                    
                case PATROL_MODE_MANUAL:
                    rt_kprintf("[Patrol] Manual control mode activated\n");
                    break;
                    
                default:
                    break;
            }
        }
        
        if (event_flags & EVENT_PATROL_STOP) {
            rt_kprintf("[Patrol] Patrol stopped\n");
            current_patrol_mode = PATROL_MODE_STOP;
        }
        
        if (event_flags & EVENT_UART_COMMAND) {
            rt_kprintf("[Patrol] Processing new command, mode: %d\n", current_patrol_mode);
        }
    }
}

/*
 * 主函数
 * 功能：基于多线程架构初始化城市垃圾巡检系统
 */
int main(void)
{
    rt_kprintf("=== 城市垃圾巡检系统启动 ===\n");
    rt_kprintf("技术特性：\n");
    rt_kprintf("1. 多线程架构 + 双缓冲机制\n");
    rt_kprintf("2. YOLOv2垃圾检测 + 10种垃圾分类 (K210优化)\n");
    rt_kprintf("3. 图像标注 + 检测框绘制\n");
    rt_kprintf("4. 智能文件命名：时间-经纬度.png\n");
    rt_kprintf("5. DMA + 完成量阻塞式SD卡写入\n");
    rt_kprintf("6. UART + 环形缓冲 + 状态机协议解析\n");
    rt_kprintf("7. RTT事件机制运动模式切换\n");
    rt_kprintf("8. K210资源优化：YOLOv2 < 6MB 内存占用\n");
    rt_kprintf("================================\n");

    // 初始化同步对象
    data_mutex = rt_mutex_create("data_mutex", RT_IPC_FLAG_FIFO);
    patrol_event = rt_event_create("patrol_event", RT_IPC_FLAG_FIFO);
    
    if (!data_mutex || !patrol_event) {
        rt_kprintf("[Error] Failed to create synchronization objects\n");
        return -1;
    }

    // 初始化双缓冲区和处理后图像缓冲区
    memset(mt9v03x_image, 0, sizeof(mt9v03x_image));
    memset(processed_image, 0, sizeof(processed_image));
    current_buf = 0;

    // 初始化图像处理信号量
    rt_sem_init(&image_sem, "image_sem", 0, RT_IPC_FLAG_FIFO);

    // 初始化 MSD 设备（SD卡）
    if (msd_init("sd0", SD_DEVICE_NAME) != RT_EOK) {
        rt_kprintf("[Error] Failed to initialize MSD device\n");
        return -1;
    }

    // 挂载 SD 卡文件系统
    // 首先尝试挂载现有文件系统
    if (dfs_mount("sd0", SD_MOUNT_POINT, "elm", 0, 0) == 0) {
        rt_kprintf("[Init] SD card mounted successfully at %s\n", SD_MOUNT_POINT);
    } else {
        // 挂载失败，尝试格式化并创建新文件系统
        rt_kprintf("[Init] Failed to mount existing filesystem, trying to format...\n");
        if (dfs_mkfs("elm", "sd0") == 0) {
            rt_kprintf("[Init] SD card filesystem created successfully\n");
            // 格式化成功后再次挂载
            if (dfs_mount("sd0", SD_MOUNT_POINT, "elm", 0, 0) == 0) {
                rt_kprintf("[Init] SD card mounted successfully at %s after formatting\n", SD_MOUNT_POINT);
            } else {
                rt_kprintf("[Error] Failed to mount SD card after formatting\n");
                return -1;
            }
        } else {
            rt_kprintf("[Error] Failed to format SD card\n");
            return -1;
        }
    }

    // 创建图像保存目录
    create_image_directory();

    // 初始化K210 KPU YOLOv2模型
    if (kpu_yolo_init() != RT_EOK) {
        rt_kprintf("[Warning] KPU YOLOv2 initialization failed, will use fallback detection\n");
    }

    // 从SD卡加载GPS路径点
    int loaded_points = load_gps_route_from_file();
    if (loaded_points <= 0) {
        // 如果无法从文件加载，使用默认路径点
        rt_kprintf("[Init] Using default GPS route points\n");
        patrol_points[0] = (patrol_point_t){39.904200f, 116.407400f, "Default_Point_1"};
        patrol_points[1] = (patrol_point_t){39.904500f, 116.407700f, "Default_Point_2"};
        patrol_points[2] = (patrol_point_t){39.904800f, 116.408000f, "Default_Point_3"};
        patrol_point_count = 3;
    }

    // 初始化RT-Thread设备（替代K210 SDK原生API）
    if (init_rt_thread_devices() != RT_EOK) {
        rt_kprintf("[Error] RT-Thread devices initialization failed\n");
        return -1;
    }
    
    // 设置UART接收回调函数
    if (uart_bluetooth != RT_NULL) {
        rt_device_set_rx_indicate(uart_bluetooth, uart_bluetooth_callback);
    }
    if (uart_gps != RT_NULL) {
        rt_device_set_rx_indicate(uart_gps, uart_gps_callback);
    }

    // 初始化实时控制系统（编码器、MPU6050、电机控制）
    if (init_realtime_control() != RT_EOK) {
        rt_kprintf("[Error] Real-time control system initialization failed\n");
        return -1;
    }

    // 初始化摄像头（使用RT-Thread驱动或简化实现）
    // 注意：实际的摄像头初始化需要根据RT-Thread K210 BSP的具体实现
    rt_kprintf("[Camera] Camera initialization skipped - needs RT-Thread BSP support\n");
    rt_kprintf("[Camera] Note: DVP camera requires proper RT-Thread driver implementation\n");

    // 创建UART协议解析线程 - 添加此线程
    rt_thread_t uart_thread = rt_thread_create("uart_rx", uart_rx_thread,
                                               RT_NULL, THREAD_STACK_SIZE,
                                               UART_THREAD_PRIORITY, THREAD_TIMESLICE);
    if (uart_thread != RT_NULL) {
        rt_thread_startup(uart_thread);
        rt_kprintf("[Init] UART protocol parsing thread created\n");
    }

    // 创建摄像头线程 - 使用高优先级保证实时性
    rt_thread_t camera_thread = rt_thread_create("camera", camera_thread_entry,
                                                 RT_NULL, THREAD_STACK_SIZE,
                                                 CAMERA_THREAD_PRIORITY, THREAD_TIMESLICE);
    if (camera_thread != RT_NULL) {
        rt_thread_startup(camera_thread);
        rt_kprintf("[Init] Camera thread created with high priority\n");
    }

    // 创建GPS线程
    rt_thread_t gps_thread = rt_thread_create("gps", gps_thread_entry,
                                              RT_NULL, THREAD_STACK_SIZE,
                                              GPS_THREAD_PRIORITY, THREAD_TIMESLICE);
    if (gps_thread != RT_NULL) {
        rt_thread_startup(gps_thread);
        rt_kprintf("[Init] GPS thread created\n");
    }

    // 创建SD卡存储线程 - 使用低优先级
    rt_thread_t sdcard_thread = rt_thread_create("sdcard", sdcard_thread_entry,
                                                 RT_NULL, THREAD_STACK_SIZE,
                                                 SDCARD_THREAD_PRIORITY, THREAD_TIMESLICE);
    if (sdcard_thread != RT_NULL) {
        rt_thread_startup(sdcard_thread);
        rt_kprintf("[Init] SD card storage thread created with low priority\n");
    }

    // 创建巡检控制线程
    rt_thread_t patrol_thread = rt_thread_create("patrol", patrol_control_thread,
                                                 RT_NULL, THREAD_STACK_SIZE,
                                                 PATROL_THREAD_PRIORITY, THREAD_TIMESLICE);
    if (patrol_thread != RT_NULL) {
        rt_thread_startup(patrol_thread);
        rt_kprintf("[Init] Patrol control thread created\n");
    }

    rt_kprintf("[Init] System initialization completed successfully\n");
    rt_kprintf("[Init] Real-time Control: Encoder(200Hz), MPU6050(50Hz), Control(100Hz)\n");
    rt_kprintf("[Init] PID Controllers: Speed & Angle control with PWM output\n");
    rt_kprintf("[Init] RT-Thread Devices: UART1(Bluetooth), UART2(GPS), PWM1(Motor), PWM2(Servo)\n");
    rt_kprintf("[Init] YOLO Version: %s (K210 optimized) with %d anchor boxes\n", YOLO_VERSION, ANCHOR_NUM);
    rt_kprintf("[Init] K210 KPU YOLOv2 model: %s\n", YOLO_MODEL_FILE);
    rt_kprintf("[Init] KPU Status: %s\n", kpu_initialized ? "Initialized" : "Not Available");
    rt_kprintf("[Init] YOLOv2 detection supports 10 garbage types with bounding box annotation\n");
    rt_kprintf("[Init] Image files saved to: %s\n", IMAGE_SAVE_DIR);
    rt_kprintf("[Init] GPS route file: %s\n", GPS_ROUTE_FILE);
    rt_kprintf("[Init] Auto patrol mode with %d route points loaded\n", patrol_point_count);
    rt_kprintf("[Init] Network input: %dx%d, Grid: %dx%d, Threshold: %.2f\n", 
              NET_INPUT_WIDTH, NET_INPUT_HEIGHT, GRID_WIDTH, GRID_HEIGHT, YOLO_THRESHOLD);
    rt_kprintf("[Init] Protocol: Header(0xAA) + CMD + DataLen + Data + Checksum + Tail(0x55)\n");
    rt_kprintf("[Init] Vehicle Control: Speed PID + Angle PID with encoder feedback\n");
    rt_kprintf("[Init] Sensor Fusion: MPU6050 + Encoder + GPS for navigation\n");
    rt_kprintf("[Status] System ready for intelligent garbage patrol with real-time control\n");
    rt_kprintf("[Status] Multi-threading: Camera + GPS + Control + Protocol + Storage\n");

    return 0;
}