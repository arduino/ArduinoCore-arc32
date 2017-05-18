#ifndef _MAILBOX_H_
#define _MAILBOX_H_

#define NUM_MAILBOX_CHANNELS 8
#define CHANNEL_DATA_WORDS   4
#define MBOX_BYTES           16

class CurieMailboxMsg {
public:
    uint32_t data[CHANNEL_DATA_WORDS];
    uint32_t id;
    int channel = 0;
};

void mailbox_init (bool master);
void mailbox_register (int channel, void(*callback)(CurieMailboxMsg));
void mailbox_unregister (int channel);
void mailbox_enable_receive (int channel);
void mailbox_disable_receive (int channel);
void mailbox_write(CurieMailboxMsg& msg);

#endif
