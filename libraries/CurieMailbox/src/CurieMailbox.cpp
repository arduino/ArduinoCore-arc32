#include "interrupt.h"
#include "scss_registers.h"
#include "CurieMailbox.h"

#define BUFSIZE                     33
#define CAP_CHAN(chan)              chan = (chan >= CurieMailbox.numChannels) ?\
                                    CurieMailbox.numChannels - 1 : ((chan < 0) \
                                    ? 0 : chan)

static CurieMailboxMsg buf[BUFSIZE];
static volatile unsigned int head;
static unsigned int tail;

CurieMailboxClass::CurieMailboxClass (void)
{
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

int CurieMailboxClass::available (void)
{
    return ((head + BUFSIZE) - tail) % BUFSIZE;
}

void CurieMailboxClass::enableReceive (int channel)
{
    CAP_CHAN(channel);
    mailbox_enable_receive(channel);
}

void CurieMailboxClass::disableReceive (int channel)
{
    CAP_CHAN(channel);
    mailbox_disable_receive(channel);
}

static void mbox_isr (CurieMailboxMsg msg)
{
    buf_put(msg);
}

void CurieMailboxClass::begin (void)
{
    /* Channel 7 is reserved for Serial */
    for (int i = 0; i < NUM_MAILBOX_CHANNELS - 1; ++i) {
        mailbox_register(i, mbox_isr);
    }
}

void CurieMailboxClass::end (void)
{
    /* Channel 7 is reserved for Serial */
    for (int i = 0; i < NUM_MAILBOX_CHANNELS - 1; ++i) {
        mailbox_register(i, 0);
    }
}

void CurieMailboxClass::put (CurieMailboxMsg msg)
{
    CAP_CHAN(msg.channel);
    mailbox_write(msg);
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
