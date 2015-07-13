#include "services/test_service.h"
#include "cfw/cfw_client.h"

/****************************************************************************************
 *********************** SERVICE API IMPLEMENATION **************************************
 ****************************************************************************************/
int test_service_test_1(svc_client_handle_t * h, void * priv) {
	struct cfw_message * msg = cfw_alloc_message_for_service(h, MSG_ID_TEST_1, sizeof(*msg), priv);
	cfw_send_message(msg);
	return 0;
}

int test_service_test_2(svc_client_handle_t * h, void *priv) {
	struct cfw_message * msg = cfw_alloc_message_for_service(h, MSG_ID_TEST_2, sizeof(*msg), priv);
	cfw_send_message(msg);
	return 0;
}
