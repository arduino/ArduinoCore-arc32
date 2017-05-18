CurieMailbox library for Arduino/Genuino 101

Overview:

This library provides access to the inter-processor mailbox that allows
interrupt-based communication between the x86 and ARC cores contained in the
Curie module (in such a transaction, the message sender is referred to as the
"source" processor, and the recipient as the destination processor). Note that
using this library to send a message to the mailbox, when there is no
application running at the destination to receive it, will cause the source
processor busy-wait indefinitely.

Message format:

The CurieMailbox library uses the CurieMailboxMsg class to describe a message
for the mailbox. In total, a mailbox message contains 17 bytes of data that you
can fill. Here's how you might populate a CurieMailboxMsg;

    #include "CurieMailbox.h"
    CurieMailboxMsg msg;

    /* ID can be any 31-bit value */
    msg.id = 0x7FFFFF;

    /* Data payload is an array of 4 32-bit values */
    msg.data[0] = 0xDEADBEEF;
    msg.data[1] = 0xCAFEBABE;
    msg.data[2] = 0xFFFFFFFF;
    msg.data[3] = 0x00000000;

    /* And the channel on which the message will be sent */
    msg.channel = 6;

API reference:


void CurieMailbox::begin ()

  Enable interrupts for the mailbox, and set up the mailbox hardware for use

Params:

  None


void CurieMailbox::enableReceive (unsigned int channel)

  Unmask interrupts for mailbox channel 'channel', allowing it to receive
  messages (channels can always send, only reception has to be enabled).

Params:

  int channel:  integer value (0-7) representing the channel to be enabled.
  If the passed value does not represent a valid channel, then the highest
  channel will be used instead.



void CurieMailbox::disableReceive (unsigned int channel)

  Mask interrupts for mailbox channel 'channel', so that messages can no longer
  be received (but they can still be sent).

Params:

  int channel:  integer value between 0 and CurieMailbox.numChannels,
  representing the channel to be enabled. If the passed value does not
  represent a valid channel, then the highest channel will be used instead.


int CurieMailbox::available (void)

  Returns the number of received mailbox messages available for reading.

Params:

  None



void CurieMailbox::put (CurieMailboxMsg msg)

  Writes the message 'msg' to the mailbox.

Params:

  CurieMailboxMsg msg: the message to send



CurieMailboxMsg CurieMailbox::get (void)

  Gets the most recently received unread message from the mailbox.

Params:

  None.
