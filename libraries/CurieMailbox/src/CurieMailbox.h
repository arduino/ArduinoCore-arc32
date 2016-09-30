#ifndef _CURIEMAILBOX_H_
#define _CURIEMAILBOX_H_

#include "CurieMailboxMsg.h"

class CurieMailboxClass {
public:
    CurieMailboxClass (void);
    void begin (void);
    void begin (bool master);
    void end (void);

    void enableReceive (int channel);
    void disableReceive (int channel);

    int available (void);
    void put (CurieMailboxMsg msg);
    CurieMailboxMsg get (void);
};

extern CurieMailboxClass CurieMailbox;

#endif
