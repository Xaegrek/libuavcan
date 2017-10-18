/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstring>
#include <uavcan_stm32/can.hpp>
#include <uavcan_stm32/clock.hpp>
#include "internal.hpp"

#if UAVCAN_STM32_CHIBIOS
# include <hal.h>
#else
# error "Unknown OS"
#endif

namespace uavcan_stm32
{

/*
 * CanIface
 */
const uavcan::uint32_t CanIface::TSR_ABRQx[CanIface::NumTxMailboxes] =
{
    bxcan::TSR_ABRQ0,
    bxcan::TSR_ABRQ1,
    bxcan::TSR_ABRQ2
};

uavcan::int16_t CanIface::send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                               uavcan::CanIOFlags flags)
{
    if (frame.isErrorFrame() || frame.dlc > 8)
    {
        return -ErrUnsupportedFrame;
    }

    /*
     * Normally we should perform the same check as in @ref canAcceptNewTxFrame(), because
     * it is possible that the highest-priority frame between select() and send() could have been
     * replaced with a lower priority one due to TX timeout. But we don't do this check because:
     *
     *  - It is a highly unlikely scenario.
     *
     *  - Frames do not timeout on a properly functioning bus. Since frames do not timeout, the new
     *    frame can only have higher priority, which doesn't break the logic.
     *
     *  - If high-priority frames are timing out in the TX queue, there's probably a lot of other
     *    issues to take care of before this one becomes relevant.
     *
     *  - It takes CPU time. Not just CPU time, but critical section time, which is expensive.
     */
    CriticalSectionLocker lock;

    /*
     * Seeking for an empty slot
     */
    uavcan::uint8_t txmailbox = 0xFF;
    if ((can_->TSR & bxcan::TSR_TME0) == bxcan::TSR_TME0)
    {
        txmailbox = 0;
    }
    else if ((can_->TSR & bxcan::TSR_TME1) == bxcan::TSR_TME1)
    {
        txmailbox = 1;
    }
    else if ((can_->TSR & bxcan::TSR_TME2) == bxcan::TSR_TME2)
    {
        txmailbox = 2;
    }
    else
    {
        return 0;       // No transmission for you.
    }

    peak_tx_mailbox_index_ = uavcan::max(peak_tx_mailbox_index_, txmailbox);    // Statistics

    /*
     * Setting up the mailbox
     */
    bxcan::TxMailboxType& mb = can_->TxMailbox[txmailbox];
    if (frame.isExtended())
    {
        mb.TIR = ((frame.id & uavcan::CanFrame::MaskExtID) << 3) | bxcan::TIR_IDE;
    }
    else
    {
        mb.TIR = ((frame.id & uavcan::CanFrame::MaskStdID) << 21);
    }

    if (frame.isRemoteTransmissionRequest())
    {
        mb.TIR |= bxcan::TIR_RTR;
    }

    mb.TDTR = frame.dlc;

    mb.TDHR = (uavcan::uint32_t(frame.data[7]) << 24) |
              (uavcan::uint32_t(frame.data[6]) << 16) |
              (uavcan::uint32_t(frame.data[5]) << 8)  |
              (uavcan::uint32_t(frame.data[4]) << 0);
    mb.TDLR = (uavcan::uint32_t(frame.data[3]) << 24) |
              (uavcan::uint32_t(frame.data[2]) << 16) |
              (uavcan::uint32_t(frame.data[1]) << 8)  |
              (uavcan::uint32_t(frame.data[0]) << 0);

    mb.TIR |= bxcan::TIR_TXRQ;  // Go.

    /*
     * Registering the pending transmission so we can track its deadline and loopback it as needed
     */
    TxItem& txi = pending_tx_[txmailbox];
    txi.deadline       = tx_deadline;
    txi.frame          = frame;
    txi.loopback       = (flags & uavcan::CanIOFlagLoopback) != 0;
    txi.abort_on_error = (flags & uavcan::CanIOFlagAbortOnError) != 0;
    txi.pending        = true;
    return 1;
}

uavcan::int16_t CanIface::receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                  uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
{
    out_ts_monotonic = clock::getMonotonic();  // High precision is not required for monotonic timestamps
    uavcan::uint64_t utc_usec = 0;
    {
        CriticalSectionLocker lock;
        if (rx_queue_.getLength() == 0)
        {
            return 0;
        }
        rx_queue_.pop(out_frame, utc_usec, out_flags);
    }
    out_ts_utc = uavcan::UtcTime::fromUSec(utc_usec);
    return 1;
}

void CanIface::handleTxMailboxInterrupt(uavcan::uint8_t mailbox_index, bool txok, const uavcan::uint64_t utc_usec)
{
    UAVCAN_ASSERT(mailbox_index < NumTxMailboxes);

    had_activity_ = had_activity_ || txok;

    TxItem& txi = pending_tx_[mailbox_index];

    if (txi.loopback && txok && txi.pending)
    {
        rx_queue_.push(txi.frame, utc_usec, uavcan::CanIOFlagLoopback);
    }

    txi.pending = false;
}

void CanIface::handleTxInterrupt(const uavcan::uint64_t utc_usec)
{
    // TXOK == false means that there was a hardware failure
    if (can_->TSR & bxcan::TSR_RQCP0)
    {
        const bool txok = can_->TSR & bxcan::TSR_TXOK0;
        can_->TSR = bxcan::TSR_RQCP0;
        handleTxMailboxInterrupt(0, txok, utc_usec);
    }
    if (can_->TSR & bxcan::TSR_RQCP1)
    {
        const bool txok = can_->TSR & bxcan::TSR_TXOK1;
        can_->TSR = bxcan::TSR_RQCP1;
        handleTxMailboxInterrupt(1, txok, utc_usec);
    }
    if (can_->TSR & bxcan::TSR_RQCP2)
    {
        const bool txok = can_->TSR & bxcan::TSR_TXOK2;
        can_->TSR = bxcan::TSR_RQCP2;
        handleTxMailboxInterrupt(2, txok, utc_usec);
    }
    update_event_.signalFromInterrupt();

    pollErrorFlagsFromISR();
}

void CanIface::handleRxInterrupt(uavcan::uint8_t fifo_index, uavcan::uint64_t utc_usec)
{
    UAVCAN_ASSERT(fifo_index < 2);

    volatile uavcan::uint32_t* const rfr_reg = (fifo_index == 0) ? &can_->RF0R : &can_->RF1R;
    if ((*rfr_reg & bxcan::RFR_FMP_MASK) == 0)
    {
        UAVCAN_ASSERT(0);  // Weird, IRQ is here but no data to read
        return;
    }

    /*
     * Register overflow as a hardware error
     */
    if ((*rfr_reg & bxcan::RFR_FOVR) != 0)
    {
        error_cnt_++;
    }

    /*
     * Read the frame contents
     */
    uavcan::CanFrame frame;
    const bxcan::RxMailboxType& rf = can_->RxMailbox[fifo_index];

    if ((rf.RIR & bxcan::RIR_IDE) == 0)
    {
        frame.id = uavcan::CanFrame::MaskStdID & (rf.RIR >> 21);
    }
    else
    {
        frame.id = uavcan::CanFrame::MaskExtID & (rf.RIR >> 3);
        frame.id |= uavcan::CanFrame::FlagEFF;
    }

    if ((rf.RIR & bxcan::RIR_RTR) != 0)
    {
        frame.id |= uavcan::CanFrame::FlagRTR;
    }

    frame.dlc = rf.RDTR & 15;

    frame.data[0] = uavcan::uint8_t(0xFF & (rf.RDLR >> 0));
    frame.data[1] = uavcan::uint8_t(0xFF & (rf.RDLR >> 8));
    frame.data[2] = uavcan::uint8_t(0xFF & (rf.RDLR >> 16));
    frame.data[3] = uavcan::uint8_t(0xFF & (rf.RDLR >> 24));
    frame.data[4] = uavcan::uint8_t(0xFF & (rf.RDHR >> 0));
    frame.data[5] = uavcan::uint8_t(0xFF & (rf.RDHR >> 8));
    frame.data[6] = uavcan::uint8_t(0xFF & (rf.RDHR >> 16));
    frame.data[7] = uavcan::uint8_t(0xFF & (rf.RDHR >> 24));

    *rfr_reg = bxcan::RFR_RFOM | bxcan::RFR_FOVR | bxcan::RFR_FULL;  // Release FIFO entry we just read

    /*
     * Store with timeout into the FIFO buffer and signal update event
     */
    rx_queue_.push(frame, utc_usec, 0);
    had_activity_ = true;
    update_event_.signalFromInterrupt();

    pollErrorFlagsFromISR();
}

void CanIface::pollErrorFlagsFromISR()
{
    const uavcan::uint8_t lec = uavcan::uint8_t((can_->ESR & bxcan::ESR_LEC_MASK) >> bxcan::ESR_LEC_SHIFT);
    if (lec != 0)
    {
        can_->ESR = 0;
        error_cnt_++;

        // Serving abort requests
        for (int i = 0; i < NumTxMailboxes; i++)    // Dear compiler, may I suggest you to unroll this loop please.
        {
            TxItem& txi = pending_tx_[i];
            if (txi.pending && txi.abort_on_error)
            {
                can_->TSR = TSR_ABRQx[i];
                txi.pending = false;
                served_aborts_cnt_++;
            }
        }
    }
}

void CanIface::discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time)
{
    CriticalSectionLocker lock;
    for (int i = 0; i < NumTxMailboxes; i++)
    {
        TxItem& txi = pending_tx_[i];
        if (txi.pending && txi.deadline < current_time)
        {
            can_->TSR = TSR_ABRQx[i];  // Goodnight sweet transmission
            txi.pending = false;
            error_cnt_++;
        }
    }
}

bool CanIface::canAcceptNewTxFrame(const uavcan::CanFrame& frame) const
{
    /*
     * We can accept more frames only if the following conditions are satisfied:
     *  - There is at least one TX mailbox free (obvious enough);
     *  - The priority of the new frame is higher than priority of all TX mailboxes.
     */
    {
        static const uavcan::uint32_t TME = bxcan::TSR_TME0 | bxcan::TSR_TME1 | bxcan::TSR_TME2;
        const uavcan::uint32_t tme = can_->TSR & TME;

        if (tme == TME)     // All TX mailboxes are free (as in freedom).
        {
            return true;
        }

        if (tme == 0)       // All TX mailboxes are busy transmitting.
        {
            return false;
        }
    }

    /*
     * The second condition requires a critical section.
     */
    CriticalSectionLocker lock;

    for (int mbx = 0; mbx < NumTxMailboxes; mbx++)
    {
        if (pending_tx_[mbx].pending && !frame.priorityHigherThan(pending_tx_[mbx].frame))
        {
            return false;       // There's a mailbox whose priority is higher or equal the priority of the new frame.
        }
    }

    return true;                // This new frame will be added to a free TX mailbox in the next @ref send().
}

bool CanIface::isRxBufferEmpty() const
{
    CriticalSectionLocker lock;
    return rx_queue_.getLength() == 0;
}

uavcan::uint64_t CanIface::getErrorCount() const
{
    CriticalSectionLocker lock;
    return error_cnt_ + rx_queue_.getOverflowCount();
}

unsigned CanIface::getRxQueueLength() const
{
    CriticalSectionLocker lock;
    return rx_queue_.getLength();
}

bool CanIface::hadActivity()
{
    CriticalSectionLocker lock;
    const bool ret = had_activity_;
    had_activity_ = false;
    return ret;
}

/*
 * CanDriver
 */
uavcan::CanSelectMasks CanDriver::makeSelectMasks(const uavcan::CanFrame* (& pending_tx)[uavcan::MaxCanIfaces]) const
{
    uavcan::CanSelectMasks msk;

    // Iface 0
    msk.read  = if0_.isRxBufferEmpty() ? 0 : 1;

    if (pending_tx[0] != UAVCAN_NULLPTR)
    {
        msk.write = if0_.canAcceptNewTxFrame(*pending_tx[0]) ? 1 : 0;
    }

    return msk;
}

bool CanDriver::hasReadableInterfaces() const
{
	return !if0_.isRxBufferEmpty();
}

uavcan::int16_t CanDriver::select(uavcan::CanSelectMasks& inout_masks,
                                  const uavcan::CanFrame* (& pending_tx)[uavcan::MaxCanIfaces],
                                  const uavcan::MonotonicTime blocking_deadline)
{
    const uavcan::CanSelectMasks in_masks = inout_masks;
    const uavcan::MonotonicTime time = clock::getMonotonic();

    if0_.discardTimedOutTxMailboxes(time);              // Check TX timeouts - this may release some TX slots
    {
        CriticalSectionLocker cs_locker;
        if0_.pollErrorFlagsFromISR();
    }

    inout_masks = makeSelectMasks(pending_tx);          // Check if we already have some of the requested events
    if ((inout_masks.read  & in_masks.read)  != 0 ||
        (inout_masks.write & in_masks.write) != 0)
    {
        return 1;
    }

    (void)update_event_.wait(blocking_deadline - time); // Block until timeout expires or any iface updates
    inout_masks = makeSelectMasks(pending_tx);  // Return what we got even if none of the requested events are set
    return 1;                                   // Return value doesn't matter as long as it is non-negative
}

void CanDriver::initOnce()
{
    /*
     * CAN1, CAN2
     */
    {
        CriticalSectionLocker lock;
        RCC->APB1ENR  |=  RCC_APB1ENR_CAN1EN;
        RCC->APB1RSTR |=  RCC_APB1RSTR_CAN1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
    }

    /*
     * IRQ
     */
#ifdef UAVCAN_STM32_CHIBIOS || UAVCAN_STM32_BAREMETAL || UAVCAN_STM32_FREERTOS
    {
        CriticalSectionLocker lock;
        nvicEnableVector(CAN1_TX_IRQn,  UAVCAN_STM32_IRQ_PRIORITY_MASK);
        nvicEnableVector(CAN1_RX0_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
        nvicEnableVector(CAN1_RX1_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
    }
#endif
}

int CanDriver::init(const uavcan::uint32_t bitrate, const CanIface::OperatingMode mode)
{
    int res = 0;

    UAVCAN_STM32_LOG("Bitrate %lu mode %d", static_cast<unsigned long>(bitrate), static_cast<int>(mode));

    static bool initialized_once = false;
    if (!initialized_once)
    {
        initialized_once = true;
        UAVCAN_STM32_LOG("First initialization");
        initOnce();
    }

    /*
     * CAN1
     */
    UAVCAN_STM32_LOG("Initing iface 0...");
    ifaces[0] = &if0_;                          // This link must be initialized first,
    res = if0_.init(bitrate, mode);             // otherwise an IRQ may fire while the interface is not linked yet;
    if (res < 0)                                // a typical race condition.
    {
        UAVCAN_STM32_LOG("Iface 0 init failed %i", res);
        ifaces[0] = UAVCAN_NULLPTR;
        goto fail;
    }

    UAVCAN_STM32_LOG("CAN drv init OK");
    UAVCAN_ASSERT(res >= 0);
    return res;

fail:
    UAVCAN_STM32_LOG("CAN drv init failed %i", res);
    UAVCAN_ASSERT(res < 0);
    return res;
}

CanIface* CanDriver::getIface(uavcan::uint8_t iface_index)
{
    if (iface_index < UAVCAN_STM32_NUM_IFACES)
    {
        return ifaces[iface_index];
    }
    return UAVCAN_NULLPTR;
}

bool CanDriver::hadActivity()
{
    bool ret = if0_.hadActivity();
    return ret;
}

} // namespace uavcan_stm32

/*
 * Interrupt handlers
 */
extern "C"
{
UAVCAN_STM32_IRQ_HANDLER(CAN1_TX_IRQHandler);
UAVCAN_STM32_IRQ_HANDLER(CAN1_TX_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    uavcan_stm32::handleTxInterrupt(0);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

UAVCAN_STM32_IRQ_HANDLER(CAN1_RX0_IRQHandler);
UAVCAN_STM32_IRQ_HANDLER(CAN1_RX0_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    uavcan_stm32::handleRxInterrupt(0, 0);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

UAVCAN_STM32_IRQ_HANDLER(CAN1_RX1_IRQHandler);
UAVCAN_STM32_IRQ_HANDLER(CAN1_RX1_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    uavcan_stm32::handleRxInterrupt(0, 1);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

} // extern "C"
