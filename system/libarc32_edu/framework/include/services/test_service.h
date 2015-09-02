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

#ifndef __TEST_SERVICE_H__
#define __TEST_SERVICE_H__
#include "cfw/cfw.h"
#include "cfw/cfw_client.h"


#define MSG_ID_TEST_1 1
#define MSG_ID_TEST_2 2
#define MSG_ID_TEST_3 3
#define MSG_ID_TEST_4 4

#define MSG_ID_TEST_1_RSP 0x81
#define MSG_ID_TEST_2_RSP 0x82
#define MSG_ID_TEST_3_RSP 0x83
#define MSG_ID_TEST_4_RSP 0x84

#define MSG_ID_TEST_1_EVT 0x1001


void test_service_init(void * queue, int service_id);

int test_service_test_1(svc_client_handle_t * svc_handle, void * priv);
int test_service_test_2(svc_client_handle_t * svc_handle, void * priv);


typedef struct test_1_rsp_msg {
	struct cfw_rsp_message rsp_header;
	int data;
} test_1_rsp_msg_t;

typedef struct test_2_rsp_msg {
	struct cfw_rsp_message rsp_header;
	int data2;
} test_2_rsp_msg_t;

#endif
