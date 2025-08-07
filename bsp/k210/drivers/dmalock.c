
/* Copyright Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 /*
 这是一个为 DMA 控制器设计的“通道分配锁”，用于线程安全地动态分配和释放 DMA 通道，防止多个任务抢占同一个通道。
 */
#include <rtthread.h>
#include "dmalock.h"
#include "../packages/K210-SDK-latest/lib/drivers/include/dmac.h"  // K210 DMA 控制器驱动头文件，定义了 DMA 通道数等常量。
//结构体定义：DMA 通道状态管理器
struct dmac_host
{
    struct rt_semaphore sem;   //控制总共可用的DMA通道数
    struct rt_mutex mutex;     //控制通道分配过程的互斥
    uint8_t channel_used[DMAC_CHANNEL_COUNT]; //标志每个通道是否被占用（1=占用，0=空闲）
    char *channel_name[DMAC_CHANNEL_COUNT]; //  为什么是  占用该通道的线程或函数名（用于调试）
};

static struct dmac_host _dmac_host;
//定义了一个全局静态变量 _dmac_host，作为唯一的 DMA 通道分配器。

void dmalock_init(void)
{
    rt_sem_init(&_dmac_host.sem, "dma_sem", DMAC_CHANNEL_COUNT, RT_IPC_FLAG_FIFO);//初始化一个信号量，初始值为 DMAC_CHANNEL_COUNT，表示有多少个 DMA 通道。RT_IPC_FLAG_FIFO 表示按先进先出的顺序唤醒等待线程。
    rt_mutex_init(&_dmac_host.mutex, "dma_mutex", RT_IPC_FLAG_PRIO); //初始化一个互斥锁 mutex，用于控制对 channel_used 的访问。RT_IPC_FLAG_PRIO 表示优先级高的线程优先获得互斥锁。
    for (int i = 0; i < DMAC_CHANNEL_COUNT; i++)
    {
        _dmac_host.channel_used[i] = 0;
        _dmac_host.channel_name[i] = NULL;
    }
}

//分配通道 获取锁
/*
分配一个可用 DMA 通道。参数含义：
chn: 输出参数，返回获得的通道号。
timeout_ms: 获取通道的最大等待时间（ms）。
name: 当前使用者的名字（调试信息用）。
*/
int _dmalock_sync_take(dmac_channel_number_t *chn, int timeout_ms, const char *name)
{
    rt_err_t result;

    *chn = DMAC_CHANNEL_MAX; //初始化返回值为最大值（表示无效通道）。
    result = rt_sem_take(&_dmac_host.sem, timeout_ms);
    //等待信号量，表示申请使用一个 DMA 通道。如果没有资源，会阻塞直到超时或资源释放。
    if (result == RT_EOK)
    {
        //信号量获取成功后，进入互斥区，防止多个线程同时修改通道状态。
        //遍历通道，找到第一个空闲的通道，标记为已使用，并记录使用者的名字。然后释放互斥锁。
        rt_mutex_take(&_dmac_host.mutex, RT_WAITING_FOREVER);
        for (int i = 0; i < DMAC_CHANNEL_COUNT; i++)
        {
            if (_dmac_host.channel_used[i] == 0)
            {
                _dmac_host.channel_used[i] = 1;
                _dmac_host.channel_name[i] = name;
                *chn = i;
                break;
            }
        }
        rt_mutex_release(&_dmac_host.mutex);
    }
    return result;  //并没有在这里释放信号量
}

//释放DMA通道  非法通道号直接返回。
//将通道标记为空闲，清空名字，并释放信号量（表示有一个通道被释放了）。
void dmalock_release(dmac_channel_number_t chn)
{
    if (chn >= DMAC_CHANNEL_MAX)
        return;
    _dmac_host.channel_name[chn] = NULL;
    _dmac_host.channel_used[chn] = 0;
    rt_sem_release(&_dmac_host.sem); //
}

//— 调试用命令，打印当前通道占用情况
static void dma_ch_info(int argc, char **argv)
{
    uint32_t cnt = 0;

    for (int i = 0; i < DMAC_CHANNEL_COUNT; i++)
    {
        if (_dmac_host.channel_used[i] != 0)
        {
            rt_kprintf("dma_ch%d is using by func [%s]\n", i, _dmac_host.channel_name[i]);
            cnt++;
        }
    }

    if(cnt == 0)
        rt_kprintf(" no dma_ch is using.\n");
}
MSH_CMD_EXPORT(dma_ch_info, list dma channel informationn.);
