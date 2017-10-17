/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_stm32/thread.hpp>
#include <uavcan_stm32/clock.hpp>
#include <uavcan_stm32/can.hpp>
#include "internal.hpp"


namespace uavcan_stm32
{
/*
 * BusEvent
 */
bool BusEvent::wait(uavcan::MonotonicDuration duration)
{
    static const uavcan::int64_t MaxDelayMSec = 0x000FFFFF;

    const uavcan::int64_t msec = duration.toMSec();
    msg_t ret = msg_t();

    if (msec <= 0)
    {
# if (CH_KERNEL_MAJOR == 2)
        ret = sem_.waitTimeout(TIME_IMMEDIATE);
# else // ChibiOS 3+
        ret = sem_.wait(TIME_IMMEDIATE);
# endif
    }
    else
    {
# if (CH_KERNEL_MAJOR == 2)
        ret = sem_.waitTimeout((msec > MaxDelayMSec) ? MS2ST(MaxDelayMSec) : MS2ST(msec));
# else // ChibiOS 3+
        ret = sem_.wait((msec > MaxDelayMSec) ? MS2ST(MaxDelayMSec) : MS2ST(msec));
# endif
    }
# if (CH_KERNEL_MAJOR == 2)
    return ret == RDY_OK;
# else // ChibiOS 3+
    return ret == MSG_OK;
# endif
}

void BusEvent::signal()
{
    sem_.signal();
}

void BusEvent::signalFromInterrupt()
{
# if (CH_KERNEL_MAJOR == 2)
    chSysLockFromIsr();
    sem_.signalI();
    chSysUnlockFromIsr();
# else // ChibiOS 3+
    chSysLockFromISR();
    sem_.signalI();
    chSysUnlockFromISR();
# endif
}

/*
 * Mutex
 */
void Mutex::lock()
{
    mtx_.lock();
}

void Mutex::unlock()
{
# if (CH_KERNEL_MAJOR == 2)
    chibios_rt::BaseThread::unlockMutex();
# else // ChibiOS 3+
    mtx_.unlock();
# endif
}

}
