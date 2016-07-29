
#include <os/os.h>
#include "uart.h"
#include "ipc_uart_ns16550.h"

void on_nble_gap_dtm_init_rsp(void *user_data)
{
#ifdef CONFIG_UART_NS16550
	uart_ipc_disable();
#endif
}

