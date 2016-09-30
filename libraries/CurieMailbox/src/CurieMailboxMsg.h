#ifndef _CURIEMAILBOXMSG_H_
#define _CURIEMAILBOXMSG_H_

#define CHANNEL_DATA_WORDS   4

class CurieMailboxMsg {
public:
    uint32_t data[CHANNEL_DATA_WORDS];
    uint32_t id;
    int channel = 0;
};

#endif
