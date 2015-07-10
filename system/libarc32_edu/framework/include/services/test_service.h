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
