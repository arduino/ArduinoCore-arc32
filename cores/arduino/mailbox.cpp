#include <string.h>
#include "interrupt.h"
#include "scss_registers.h"
#include "mailbox.h"

#define CHANNEL_STS_MASK            0x1
#define CHANNEL_INT_MASK            0x2
#define CTRL_WORD_MASK              0x7FFFFFFF
#define CHALL_STATUS_MASK           0xFFFF
#define CHANNEL_STS_BITS            (CHANNEL_STS_MASK | CHANNEL_INT_MASK)

/* Mailbox channel status register */
#define IO_REG_MAILBOX_CHALL_STS    (SCSS_REGISTER_BASE + 0xAC0)

typedef struct mbox_channel mbox_channel_t;
typedef struct mbox_intmask mbox_intmask_t;

/* Represents the registers for a single mailbox channel */
struct mbox_channel {
    uint32_t ctrl;
    uint32_t data[CHANNEL_DATA_WORDS];
    uint32_t sts;
};

/* Mailbox interrupt mask; right now we only care about the
 * second byte, for SS mailbox interrupts (one bit per mailbox channel) */
struct mbox_intmask {
    uint8_t lmt_intmask;
    uint8_t ss_intmask;
    uint8_t lmt_halt;
    uint8_t ss_halt;
};

static volatile mbox_intmask_t *intmask;
static volatile mbox_channel_t *mbox;
static void (*callbacks[NUM_MAILBOX_CHANNELS])(CurieMailboxMsg);

void mailbox_register (int channel, void(*callback)(CurieMailboxMsg))
{
    if (channel >= 0 && channel < NUM_MAILBOX_CHANNELS) {
        callbacks[channel] = callback;
    }
}

void mailbox_unregister (int channel)
{
    if (channel >= 0 && channel < NUM_MAILBOX_CHANNELS) {
        callbacks[channel] = 0;
    }
}

void mailbox_disable_receive (int channel)
{
    intmask->ss_intmask |= 1 << channel;
}

void mailbox_enable_receive (int channel)
{
    intmask->ss_intmask &= ~(1 << channel);
}

static void do_callback (int channel, CurieMailboxMsg& msg)
{
    void (*cb)(CurieMailboxMsg) = callbacks[channel];
    if (cb) {
        cb(msg);
    }
}

static void mailbox_read (int channel, CurieMailboxMsg& msg)
{
    unsigned int i;

    /* Copy channel data into CurieMailboxMsg object */
    msg.id = mbox[channel].ctrl & CTRL_WORD_MASK;
    msg.channel = channel;

    for (i = 0; i < CHANNEL_DATA_WORDS; ++i) {
        msg.data[i] = mbox[channel].data[i];
    }

    /* Clear channel status & interrupt flags */
    mbox[channel].sts |= CHANNEL_STS_BITS;
}

void mailbox_write (CurieMailboxMsg& msg)
{
    int i;
    uint32_t key;

    /* Can't write if channel status flag is set */
    while ((mbox[msg.channel].sts & CHANNEL_STS_MASK));
    key = interrupt_lock();

    /* Poplate channel payload */
    mbox[msg.channel].ctrl |= (msg.id & CTRL_WORD_MASK);
    for (i = 0; i < CHANNEL_DATA_WORDS; ++i) {
        mbox[msg.channel].data[i] = msg.data[i];
    }

    /* Trigger interupt to host */
    mbox[msg.channel].ctrl |= ~(CTRL_WORD_MASK);

    /* Wait for HW to set the channel status bit */
    while (!(mbox[msg.channel].sts & CHANNEL_STS_MASK));

    /* Wait for destination processor to clear channel status bit */
    while ((mbox[msg.channel].sts & CHANNEL_STS_MASK));
    interrupt_unlock(key);
}

static uint16_t get_chall_sts (void)
{
    return MMIO_REG_VAL(IO_REG_MAILBOX_CHALL_STS) & CHALL_STATUS_MASK;
}

static void mailbox_isr (void)
{
    int i;
    uint32_t sts;
    CurieMailboxMsg msg;

    sts = get_chall_sts();
    /* Get channel number */
    for (i = 0; i < NUM_MAILBOX_CHANNELS; ++i) {
        if (sts & (1 << (i * 2 + 1))) {
            break;
        }
    }

    mailbox_read(i, msg);
    do_callback(i, msg);
}

static void mailbox_hardware_init (void)
{
    int i;

    for (i = 0; i < NUM_MAILBOX_CHANNELS; ++i) {
        mbox[i].sts &= ~(CHANNEL_STS_BITS);
    }
}

static void mailbox_interrupts_init (bool master)
{
    interrupt_disable(SOC_MBOX_INTERRUPT);

   /* Mask SS mailbox interrupts for all channels;
    * Unmasking is done by enableReceive */
    intmask->ss_intmask = 0xFF;

    if (master) mailbox_hardware_init();
    interrupt_connect(SOC_MBOX_INTERRUPT, mailbox_isr);
    interrupt_enable(SOC_MBOX_INTERRUPT);
}

void mailbox_init (bool master)
{
    intmask = (mbox_intmask_t *)IO_REG_MAILBOX_INT_MASK;
    mbox = (mbox_channel_t *)IO_REG_MAILBOX_BASE;
    memset(callbacks, 0, sizeof(callbacks));
    mailbox_interrupts_init(master);
}
