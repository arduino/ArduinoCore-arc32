#include "interrupt.h"
#include "scss_registers.h"
#include "CurieMailboxMsg.h"
#include "CurieMailbox.h"

#define BUFSIZE                     33
#define NUM_CHANNELS                8
#define CHANNEL_STS_MASK            0x1
#define CHANNEL_INT_MASK            0x2
#define CTRL_WORD_MASK              0x7FFFFFFF
#define CHALL_STATUS_MASK           0xFFFF
#define CHANNEL_STS_BITS            (CHANNEL_STS_MASK | CHANNEL_INT_MASK)

#define CAP_CHAN(chan)              chan = (chan >= NUM_CHANNELS) ? \
                                    NUM_CHANNELS - 1 : chan

/* Mailbox channel status register */
#define IO_REG_MAILBOX_CHALL_STS    (SCSS_REGISTER_BASE + 0xAC0)

static CurieMailboxMsg buf[BUFSIZE];
static volatile unsigned int head;
static unsigned int tail;

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

CurieMailboxClass::CurieMailboxClass (void)
{
    intmask = (mbox_intmask_t *)IO_REG_MAILBOX_INT_MASK;
    mbox = (mbox_channel_t *)IO_REG_MAILBOX_BASE;
    head = 0;
    tail = 0;
}

CurieMailboxClass CurieMailbox = CurieMailboxClass();

static void buf_put (CurieMailboxMsg msg)
{
    if (CurieMailbox.available() == (BUFSIZE - 1)) {
        /* Full- drop the new message */
        return;
    }

    buf[head] = msg;
    head = (head + 1) % BUFSIZE;
}

static uint16_t get_chall_sts (void)
{
    return MMIO_REG_VAL(IO_REG_MAILBOX_CHALL_STS) & CHALL_STATUS_MASK;
}

static void read_channel (int channel)
{
    int i;
    CurieMailboxMsg msg;

    /* Copy channel data into CurieMailboxMsg object */
    msg.id = mbox[channel].ctrl & CTRL_WORD_MASK;
    msg.channel = channel;

    for (i = 0; i < CHANNEL_DATA_WORDS; ++i) {
        msg.data[i] = mbox[channel].data[i];
    }

    /* Add CurieMailboxMsg object to buffer */
    buf_put(msg);

    /* Clear channel status & interrupt flags */
    mbox[channel].sts |= CHANNEL_STS_BITS;
}

static void write_channel (CurieMailboxMsg msg)
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

static void mbox_isr (void)
{
    int i;
    uint32_t sts;

    sts = get_chall_sts();
    /* Get channel number */
    for (i = 0; i < NUM_CHANNELS; ++i) {
        if (sts & (1 << (i * 2 + 1))) {
            break;
        }
    }

    read_channel(i);
}

static void mbox_hardware_init (void)
{
    int i;

    for (i = 0; i < NUM_CHANNELS; ++i) {
        mbox[i].sts &= ~(CHANNEL_STS_BITS);
    }
}

static void mbox_interrupts_init (bool master)
{
    interrupt_disable(SOC_MBOX_INTERRUPT);

   /* Mask SS mailbox interrupts for all channels;
    * Unmasking is done by enableReceive */
    intmask->ss_intmask = 0xFF;

    if (master) mbox_hardware_init();
    interrupt_connect(SOC_MBOX_INTERRUPT, mbox_isr);
    interrupt_enable(SOC_MBOX_INTERRUPT);
}

int CurieMailboxClass::available (void)
{
    return ((head + BUFSIZE) - tail) % BUFSIZE;
}

void CurieMailboxClass::enableReceive (unsigned int channel)
{
    CAP_CHAN(channel);
    intmask->ss_intmask &= ~(1 << channel);
}

void CurieMailboxClass::disableReceive (unsigned int channel)
{
    CAP_CHAN(channel);
    intmask->ss_intmask |= 1 << channel;
}

void CurieMailboxClass::begin (void)
{
    mbox_interrupts_init(false);
}

void CurieMailboxClass::begin (bool master)
{
    mbox_interrupts_init(master);
}

void CurieMailboxClass::end (void)
{
    /* Wait for all channels to be inactive */
    while (get_chall_sts());

    interrupt_disable(SOC_MBOX_INTERRUPT);
    interrupt_disconnect(SOC_MBOX_INTERRUPT);
}

void CurieMailboxClass::put (CurieMailboxMsg msg)
{
    if (msg.channel > (NUM_CHANNELS - 1)) {
        msg.channel = NUM_CHANNELS - 1;
    }

    write_channel(msg);
}

CurieMailboxMsg CurieMailboxClass::get (void)
{
    CurieMailboxMsg msg;

    if (head != tail) {
        msg = buf[tail];
        tail = (tail + 1) % BUFSIZE;
    }

    return msg;
}
