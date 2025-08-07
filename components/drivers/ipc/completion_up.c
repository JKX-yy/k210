/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-09-30     Bernard      first version.
 * 2021-08-18     chenyingchun add comments
 * 2023-09-15     xqyjlj       perf rt_hw_interrupt_disable/enable
 * 2024-01-25     Shell        reduce resource usage in completion for better synchronization
 *                             and smaller footprint.
 */

#define DBG_TAG           "drivers.ipc"
#define DBG_LVL           DBG_INFO
#include <rtdbg.h>

#include <rthw.h>
#include <rtdevice.h>

/**
 * This is an implementation of completion core on UP system.
 * Noted that spinlock is (preempt_lock + irq_mask) on UP scheduler.
 */

#define RT_COMPLETED    1
#define RT_UNCOMPLETED  0
#define RT_COMPLETION_FLAG(comp) ((comp)->susp_thread_n_flag & 1)
#define RT_COMPLETION_THREAD(comp) ((rt_thread_t)((comp)->susp_thread_n_flag & ~1))
#define RT_COMPLETION_NEW_STAT(thread, flag) (((flag) & 1) | (((rt_base_t)thread) & ~1))

static struct rt_spinlock _completion_lock = RT_SPINLOCK_INIT;

/**
 * @brief This function will initialize a completion object.
 *
 * @param completion is a pointer to a completion object.
 */
void rt_completion_init(struct rt_completion *completion)
{
    RT_ASSERT(completion != RT_NULL);

    completion->susp_thread_n_flag = RT_COMPLETION_NEW_STAT(RT_NULL, RT_UNCOMPLETED);
}
RTM_EXPORT(rt_completion_init);

/**
 * @brief This function will wait for a completion, if the completion is unavailable, the thread shall wait for
 *        the completion up to a specified time.
 *
 * @param completion is a pointer to a completion object.
 *
 * @param timeout is a timeout period (unit: OS ticks). If the completion is unavailable, the thread will wait for
 *                the completion done up to the amount of time specified by the argument.
 *                NOTE: Generally, we use the macro RT_WAITING_FOREVER to set this parameter, which means that when the
 *                completion is unavailable, the thread will be waitting forever.
 * @param suspend_flag suspend flags. See rt_thread_suspend_with_flag()
 *
 * @return Return the operation status. ONLY when the return value is RT_EOK, the operation is successful.
 *         If the return value is any other values, it means that the completion wait failed.
 *
 * @warning This function can ONLY be called in the thread context. It MUST NOT be called in interrupt context.
 */
rt_err_t rt_completion_wait_flags(struct rt_completion *completion,
                                  rt_int32_t timeout, int suspend_flag)
{
    rt_err_t result;
    rt_base_t level;
    rt_thread_t thread;
    RT_ASSERT(completion != RT_NULL);

    /* current context checking */
    RT_DEBUG_SCHEDULER_AVAILABLE(timeout != 0);

    result = RT_EOK;
    thread = rt_thread_self();

    level = rt_spin_lock_irqsave(&_completion_lock);

__try_again:
    if (RT_COMPLETION_FLAG(completion) != RT_COMPLETED)
    {
        /* only one thread can suspend on complete */
        RT_ASSERT(RT_COMPLETION_THREAD(completion) == RT_NULL);

        if (timeout == 0)
        {
            result = -RT_ETIMEOUT;
            goto __exit;
        }
        else
        {
            /* reset thread error number */
            thread->error = RT_EOK;

            /* suspend thread */
            result = rt_thread_suspend_with_flag(thread, suspend_flag);
            if (result == RT_EOK)
            {
                /* add to suspended thread */
                rt_base_t waiting_stat = RT_COMPLETION_NEW_STAT(thread, RT_UNCOMPLETED);
                completion->susp_thread_n_flag = waiting_stat;

                /* current context checking */
                RT_DEBUG_NOT_IN_INTERRUPT;

                /* start timer */
                if (timeout > 0)
                {
                    /* reset the timeout of thread timer and start it */
                    rt_timer_control(&(thread->thread_timer),
                                     RT_TIMER_CTRL_SET_TIME,
                                     &timeout);
                    rt_timer_start(&(thread->thread_timer));
                }
                /* enable interrupt */
                rt_spin_unlock_irqrestore(&_completion_lock, level);

                /* do schedule */
                rt_schedule();

                level = rt_spin_lock_irqsave(&_completion_lock);

                if (completion->susp_thread_n_flag != waiting_stat)
                {
                    /* completion may be completed after we suspend */
                    timeout = 0;
                    goto __try_again;
                }
                else
                {
                    /* no changes, waiting failed */
                    result = thread->error;
                    result = result > 0 ? -result : result;
                    RT_ASSERT(result != RT_EOK);
                }
            }
        }
    }

    /* clean completed flag & remove susp_thread on the case of waking by timeout */
    completion->susp_thread_n_flag = RT_COMPLETION_NEW_STAT(RT_NULL, RT_UNCOMPLETED);

__exit:
    rt_spin_unlock_irqrestore(&_completion_lock, level);

    return result;
}

/**
 * @brief This is same as rt_completion_wait_flags(), except that this API is NOT
 *        ISR-safe (you can NOT call completion_done() on isr routine).
 *
 * @param completion is a pointer to a completion object.
 * @param timeout is a timeout period (unit: OS ticks). If the completion is unavailable, the thread will wait for
 *                the completion done up to the amount of time specified by the argument.
 *                NOTE: Generally, we use the macro RT_WAITING_FOREVER to set this parameter, which means that when the
 *                completion is unavailable, the thread will be waitting forever.
 * @param suspend_flag suspend flags. See rt_thread_suspend_with_flag()
 *
 * @return Return the operation status. ONLY when the return value is RT_EOK, the operation is successful.
 *         If the return value is any other values, it means that the completion wait failed.
 *
 * @warning This function can ONLY be called in the thread context. It MUST NOT be called in interrupt context.
 */
rt_err_t rt_completion_wait_flags_noisr(struct rt_completion *completion,
                                        rt_int32_t timeout, int suspend_flag)
{
    return rt_completion_wait_flags(completion, timeout, suspend_flag);
}

/**
 * @brief   This function indicates a completion has done and wakeup the thread
 *          and update its errno. No update is applied if it's a negative value.
 *
 * @param   completion is a pointer to a completion object.
 * @param   thread_errno is the errno set to waking thread.
 * @return  RT_EOK if wakeup succeed.
 *          RT_EEMPTY if wakeup failure and the completion is set to completed.
 *          RT_EBUSY if the completion is still in completed state
 */
/*
从中断或其他上下文中唤醒一个因 rt_completion_wait() 而阻塞的线程，并设置唤醒状态。
*/
rt_err_t rt_completion_wakeup_by_errno(struct rt_completion *completion,
                                       rt_err_t thread_errno)
{
    rt_base_t level;
    rt_err_t error;
    rt_thread_t suspend_thread;
    RT_ASSERT(completion != RT_NULL);  /*定义中间变量。
RT_ASSERT(...) 确保传进来的 completion 不为空，防止异常。*/
/*因为自旋锁避免了操作系统进程调度和线程切换，所以自旋锁通常适用在时间比较短的情况下。由于这个原因，操作系统的内核经常使用自旋锁。但是，如果长时间上锁的话，自旋锁会非常耗费性能，它阻止了其他线程的运行和调度。线程持有锁的时间越长，则持有该锁的线程将被 OS(Operating System) 调度程序中断的风险越大。如果发生中断情况，那么其他线程将保持旋转状态(反复尝试获取锁)，而持有该锁的线程并不打算释放锁，这样导致的是结果是无限期推迟，直到持有锁的线程可以完成并释放它为止。*/
/*上锁：进入临界区，禁止中断并获得 _completion_lock 自旋锁；

保证后面访问 susp_thread_n_flag 是安全的，避免并发冲突。*/
    level = rt_spin_lock_irqsave(&_completion_lock);
    if (RT_COMPLETION_FLAG(completion) == RT_COMPLETED)
    {
        rt_spin_unlock_irqrestore(&_completion_lock, level);
        return -RT_EBUSY;
        /*如果这个 completion 已经被标记为完成了（即别人唤醒过），就直接返回错误 -RT_EBUSY；
不能重复唤醒同一个完成量（这是为了防止重复唤醒同一个线程）。*/
    }

    suspend_thread = RT_COMPLETION_THREAD(completion); //从 completion->susp_thread_n_flag 中解析出等待这个完成事件的线程指针（rt_thread_t）；如果没有线程在等，返回 NULL。
    if (suspend_thread)
    {
        /* there is one thread in suspended list */

        if (thread_errno >= 0)
        {
            suspend_thread->error = thread_errno;
        }

        error = rt_thread_resume(suspend_thread);
        if (error)
        {
            LOG_D("%s: failed to resume thread with %d", __func__, error);
            error = -RT_EEMPTY;
        }
    }
    else
    {
        /* no thread waiting */
        error = -RT_EEMPTY;
    }
        //RT_COMPLETION_NEW_STAT() 是一个宏，把线程指针和完成标志合成一个 rt_atomic_t 值；
    completion->susp_thread_n_flag = RT_COMPLETION_NEW_STAT(RT_NULL, RT_COMPLETED);

    rt_spin_unlock_irqrestore(&_completion_lock, level);

    return error;
}
