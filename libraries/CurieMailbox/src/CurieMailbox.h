#ifndef _CURIEMAILBOX_H_
#define _CURIEMAILBOX_H_

#include "mailbox.h"

class CurieMailboxClass {
public:
    const int numChannels = NUM_MAILBOX_CHANNELS - 1;

    CurieMailboxClass (void);
    void begin (void);
    void end (void);

    void enableReceive (int channel);
    void disableReceive (int channel);

    int available (void);
    void put (CurieMailboxMsg msg);
    CurieMailboxMsg get (void);
};

extern CurieMailboxClass CurieMailbox;

#endif
