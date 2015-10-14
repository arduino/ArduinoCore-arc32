/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _IPC_UART_H_
#define _IPC_UART_H_

/**
 * @defgroup IPC_UART IPC UART channel definitions
 * Defines the interface used for LMT/Nordic UART IPC
 * @ingroup ipc
 * @{
 */

#define SYNC_CHANNEL 0

/**
 * IPC UART Header length (non-common part with mailbox/shared memory version).
 */
#define IPC_HEADER_LEN 4

/**
 * Definitions valid for NONE sync IPC UART headers.
 */

/** Data payload pointer where high layer message is being stored. */
#define IPC_FRAME_DATA(_frame_) ((unsigned char *)_frame_ + IPC_HEADER_LEN + sizeof(uint32_t))

#define IPC_FRAME_GET_LEN(_frame_)  (((unsigned char *)_frame_)[1] << 8 | ((unsigned char*)_frame_)[0])
#define IPC_FRAME_GET_CHANNEL(_frame_) ((unsigned char *) _frame_)[2]
#define IPC_FRAME_GET_SRC(_frame_) ((unsigned char *) _frame_)[3]
#define IPC_FRAME_GET_REQUEST(_frame_) *(uint32_t *) &(_frame_)[4]

#define IPC_FRAME_SET_LEN(_frame_, _len_)  do { (_frame_)[1] = (_len_ & 0xFF00) >> 8; (_frame_)[0] = (_len_) & 0xff; } while(0)
#define IPC_FRAME_SET_CHANNEL(_frame_, _channel_) (_frame_)[2] = (_channel_)
#define IPC_FRAME_SET_SRC(_frame_, _src_) (_frame_)[3] = (_src_)
#define IPC_FRAME_SET_REQUEST(_frame_, _request_) do { (_frame_)[7] = (_request_ & 0xFF000000) >> 24;(_frame_)[6] = (_request_ & 0xFF0000) >> 16; (_frame_)[5] = (_request_ & 0xFF00) >> 8; (_frame_)[4] = (_request_) & 0xff; } while(0)

/**
 * Additional definitions for IPC Sync frames.
 *
 * This additional parameters are only present for sync frame types.
 */
#define SYNC_FRAME_GET_PARAM1(_frame_) (((unsigned char *)_frame_)[11] << 24 | ((unsigned char *)_frame_)[10] << 16 | ((unsigned char *)_frame_)[9] << 8 | ((unsigned char *)_frame_)[8])
#define SYNC_FRAME_GET_PARAM2(_frame_) (((unsigned char *)_frame_)[15] << 24 | ((unsigned char *)_frame_)[14] << 16 | ((unsigned char *)_frame_)[13] << 8 | ((unsigned char *)_frame_)[12])
#define SYNC_FRAME_GET_PTR(_frame_) (((unsigned char *)_frame_)[19] << 24 | ((unsigned char *)_frame_)[18] << 16 | ((unsigned char *)_frame_)[17] << 8 | ((unsigned char*)_frame_)[16])

#define SYNC_FRAME_SET_PARAM1(_frame_, _param1_) do { (_frame_)[11] = (_param1_ & 0xFF000000) >> 24;(_frame_)[10] = (_param1_ & 0xFF0000) >> 16; (_frame_)[9] = (_param1_ & 0xFF00) >> 8; (_frame_)[8] = (_param1_) & 0xff; } while(0)
#define SYNC_FRAME_SET_PARAM2(_frame_, _param2_) do { (_frame_)[15] = (_param2_ & 0xFF000000) >> 24;(_frame_)[14] = (_param2_ & 0xFF0000) >> 16; (_frame_)[13] = (_param2_ & 0xFF00) >> 8; (_frame_)[12] = (_param2_) & 0xff; } while(0)
#define SYNC_FRAME_SET_PTR(_frame_, _ptr_) do { (_frame_)[19] = ((uint32_t)(_ptr_) & 0xFF000000) >> 24; (_frame_)[18] = ((uint32_t)(_ptr_) & 0xFF0000) >> 16; (_frame_)[17] = ((uint32_t)(_ptr_) & 0xFF00) >> 8; (_frame_)[16] = (uint32_t)(_ptr_) & 0xff; } while(0)

/* optional sync frame payload */
#define SYNC_FRAME_DATA(_frame_) ((unsigned char *)&(_frame_)[20])

#define IPC_CHANNEL_STATE_CLOSED 0
#define IPC_CHANNEL_STATE_OPEN 1

#define IPC_UART_MAX_CHANNEL 4

struct ipc_uart_channels {
	uint16_t index;
	uint16_t state;
	void (*cb) (uint8_t cpu_id, int chan, int len, void * data);
};

void * uart_ipc_channel_open(int channel, void(*cb)(uint8_t cpu_id, int chan, int len, void * data));
int uart_ipc_send_message(void * handle, int len, void *p_data);
void uart_ipc_set_channel(void * ipc_channel);
void * uart_ipc_get_channel(void);
int uart_ipc_send_sync_resp(int channel, int request_id, int param1, int param2, void * ptr);
void uart_ipc_init(int num);
void uart_ipc_disable(int num);

/** @} */

#endif /* _IPC_UART_H_ */
