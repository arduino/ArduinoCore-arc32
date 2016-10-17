/* ns16550.c - NS16550D serial driver */

/*
 * Copyright (c) 2010, 2012-2015 Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors
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

/*
DESCRIPTION
This is the driver for the Intel NS16550 UART Chip used on the PC 386.
It uses the SCCs in asynchronous mode only.


USAGE
An ns16550 structure is used to describe the chip.
The BSP's _InitHardware() routine initializes all the
values in the uart_init_info structure before calling uart_init().

A board support package's board.h header must provide definitions for:

- the following register access routines:

  unsigned int board_inByte(unsigned int address);
  void board_outByte(unsigned char data, unsigned int address);

- and the following macro for the number of bytes between register addresses:

  UART_REG_ADDR_INTERVAL


INCLUDE FILES: drivers/uart.h
*/

/* includes */

#include <stdint.h>

#include "board.h"
#include "interrupt.h"
#include "uart.h"
#include "scss_registers.h"

/* defines */

#define UART_REG_ADDR_INTERVAL 4

/* register definitions */

#define REG_THR 0x00  /* Transmitter holding reg. */
#define REG_RDR 0x00  /* Receiver data reg.       */
#define REG_BRDL 0x00 /* Baud rate divisor (LSB)  */
#define REG_BRDH 0x01 /* Baud rate divisor (MSB)  */
#define REG_IER 0x01  /* Interrupt enable reg.    */
#define REG_IIR 0x02  /* Interrupt ID reg.        */
#define REG_FCR 0x02  /* FIFO control reg.        */
#define REG_LCR 0x03  /* Line control reg.        */
#define REG_MDC 0x04  /* Modem control reg.       */
#define REG_LSR 0x05  /* Line status reg.         */
#define REG_MSR 0x06  /* Modem status reg.        */
#define REG_DLF 0x30  /* Divisor Latch Fraction.  */

/* equates for interrupt enable register */

#define IER_RXRDY 0x01 /* receiver data ready */
#define IER_TBE 0x02   /* transmit bit enable */
#define IER_LSR 0x04   /* line status interrupts */
#define IER_MSI 0x08   /* modem status interrupts */

/* equates for interrupt identification register */

#define IIR_IP 0x01    /* interrupt pending bit */
#define IIR_MASK 0x07  /* interrupt id bits mask */
#define IIR_MSTAT 0x00 /* modem status interrupt */
#define IIR_THRE 0X02  /* transmit holding register empty */
#define IIR_RBRF 0x04  /* receiver buffer register full */
#define IIR_RLS 0x06   /* receiver line status */
#define IIR_ID 0x06    /* interupt ID mask without IP */
#define IIR_SEOB 0x06  /* serialization error or break */

/* equates for FIFO control register */

#define FCR_FIFO 0x01    /* enable XMIT and RCVR FIFO */
#define FCR_RCVRCLR 0x02 /* clear RCVR FIFO */
#define FCR_XMITCLR 0x04 /* clear XMIT FIFO */

/*
 * Per PC16550D (Literature Number: SNLS378B):
 *
 * RXRDY, Mode 0: When in the 16450 Mode (FCR0 = 0) or in
 * the FIFO Mode (FCR0 = 1, FCR3 = 0) and there is at least 1
 * character in the RCVR FIFO or RCVR holding register, the
 * RXRDY pin (29) will be low active. Once it is activated the
 * RXRDY pin will go inactive when there are no more charac-
 * ters in the FIFO or holding register.
 *
 * RXRDY, Mode 1: In the FIFO Mode (FCR0 = 1) when the
 * FCR3 = 1 and the trigger level or the timeout has been
 * reached, the RXRDY pin will go low active. Once it is acti-
 * vated it will go inactive when there are no more characters
 * in the FIFO or holding register.
 *
 * TXRDY, Mode 0: In the 16450 Mode (FCR0 = 0) or in the
 * FIFO Mode (FCR0 = 1, FCR3 = 0) and there are no charac-
 * ters in the XMIT FIFO or XMIT holding register, the TXRDY
 * pin (24) will be low active. Once it is activated the TXRDY
 * pin will go inactive after the first character is loaded into the
 * XMIT FIFO or holding register.
 *
 * TXRDY, Mode 1: In the FIFO Mode (FCR0 = 1) when
 * FCR3 = 1 and there are no characters in the XMIT FIFO, the
 * TXRDY pin will go low active. This pin will become inactive
 * when the XMIT FIFO is completely full.
 */
#define FCR_MODE0 0x00 /* set receiver in mode 0 */
#define FCR_MODE1 0x08 /* set receiver in mode 1 */

/* RCVR FIFO interrupt levels: trigger interrupt with this bytes in FIFO */
#define FCR_FIFO_1 0x00  /* 1 byte in RCVR FIFO */
#define FCR_FIFO_4 0x40  /* 4 bytes in RCVR FIFO */
#define FCR_FIFO_8 0x80  /* 8 bytes in RCVR FIFO */
#define FCR_FIFO_14 0xC0 /* 14 bytes in RCVR FIFO */

/* constants for line control register */

#define LCR_CS5 0x00   /* 5 bits data size */
#define LCR_CS6 0x01   /* 6 bits data size */
#define LCR_CS7 0x02   /* 7 bits data size */
#define LCR_CS8 0x03   /* 8 bits data size */
#define LCR_2_STB 0x04 /* 2 stop bits */
#define LCR_1_STB 0x00 /* 1 stop bit */
#define LCR_PEN 0x08   /* parity enable */
#define LCR_PDIS 0x00  /* parity disable */
#define LCR_EPS 0x10   /* even parity select */
#define LCR_SP 0x20    /* stick parity select */
#define LCR_SBRK 0x40  /* break control bit */
#define LCR_DLAB 0x80  /* divisor latch access enable */

/* constants for the modem control register */

#define MCR_DTR 0x01  /* dtr output */
#define MCR_RTS 0x02  /* rts output */
#define MCR_OUT1 0x04 /* output #1 */
#define MCR_OUT2 0x08 /* output #2 */
#define MCR_LOOP 0x10 /* loop back */
#define MCR_AFCE 0x20 /* auto flow control enable */

/* constants for line status register */

#define LSR_RXRDY 0x01 /* receiver data available */
#define LSR_OE 0x02    /* overrun error */
#define LSR_PE 0x04    /* parity error */
#define LSR_FE 0x08    /* framing error */
#define LSR_BI 0x10    /* break interrupt */
#define LSR_THRE 0x20  /* transmit holding register empty */
#define LSR_TEMT 0x40  /* transmitter empty */
#define LSR_BOTH_EMPTY (LSR_THRE|LSR_TEMT)

/* constants for modem status register */

#define MSR_DCTS 0x01 /* cts change */
#define MSR_DDSR 0x02 /* dsr change */
#define MSR_DRI 0x04  /* ring change */
#define MSR_DDCD 0x08 /* data carrier change */
#define MSR_CTS 0x10  /* complement of cts */
#define MSR_DSR 0x20  /* complement of dsr */
#define MSR_RI 0x40   /* complement of ring signal */
#define MSR_DCD 0x80  /* complement of dcd */

/* convenience defines */

#define THR(n) (uart[n].port + REG_THR * UART_REG_ADDR_INTERVAL)
#define RDR(n) (uart[n].port + REG_RDR * UART_REG_ADDR_INTERVAL)
#define BRDL(n) (uart[n].port + REG_BRDL * UART_REG_ADDR_INTERVAL)
#define BRDH(n) (uart[n].port + REG_BRDH * UART_REG_ADDR_INTERVAL)
#define IER(n) (uart[n].port + REG_IER * UART_REG_ADDR_INTERVAL)
#define IIR(n) (uart[n].port + REG_IIR * UART_REG_ADDR_INTERVAL)
#define FCR(n) (uart[n].port + REG_FCR * UART_REG_ADDR_INTERVAL)
#define LCR(n) (uart[n].port + REG_LCR * UART_REG_ADDR_INTERVAL)
#define MDC(n) (uart[n].port + REG_MDC * UART_REG_ADDR_INTERVAL)
#define LSR(n) (uart[n].port + REG_LSR * UART_REG_ADDR_INTERVAL)
#define MSR(n) (uart[n].port + REG_MSR * UART_REG_ADDR_INTERVAL)
#define DLF(n) (uart[n].port + REG_DLF * UART_REG_ADDR_INTERVAL)

#define IIRC(n) uart[n].iirCache

static inline void board_outByte(uint8_t data, uint32_t addr)
{
    *(volatile uint8_t *)addr = data;
}

static inline uint8_t board_inByte(uint32_t addr)
{
    return *((volatile uint8_t *)addr);
}

#define INBYTE(x) board_inByte(x)
#define OUTBYTE(x, d) board_outByte(d, x)

/* typedefs */

struct ns16550 {
	uint32_t port;    /* base port number or MM base address */
	uint8_t irq;      /* interrupt request level */
	uint8_t intPri;   /* interrupt priority */
	uint8_t iirCache; /* cache of IIR since it clears when read */
};

/* locals */

//static struct ns16550 __noinit uart[CONFIG_UART_NUM_SYSTEM_PORTS];
static struct ns16550 uart[CONFIG_UART_NUM_SYSTEM_PORTS];

/*******************************************************************************
*
* uart_init - initialize the chip
*
* This routine is called to reset the chip in a quiescent state.
*
* RETURNS: N/A
*/

void uart_init(int which, /* UART channel to initialize */
	       const struct uart_init_info * const init_info
	       )
{
	unsigned int oldLevel;     /* old interrupt lock level */
	uint32_t divisor, fdivisor, tmp; /* baud rate divisors */
	uint8_t mdc = 0;

	uart[which].port = init_info->regs;
	uart[which].irq = init_info->irq;
	uart[which].intPri = init_info->int_pri;
	uart[which].iirCache = 0;

	oldLevel = interrupt_lock();

	/* The formula for calculating these baud rate divisors is:
	 *   baud rate = (Fbase) / (16 * (divisor+(fdivisor/16))
	 */

	/* calculate baud rate divisor */
	divisor = (init_info->sys_clk_freq / init_info->baud_rate) >> 4;
	/* Calculate baud rate fractional divisor from the remainder.
	 * The result is rounded to the nearest whole number */
	tmp = init_info->baud_rate >> 4;
	fdivisor = (((init_info->sys_clk_freq >> 4) % init_info->baud_rate) + (tmp >> 1)) / tmp;

	/* set the DLAB to access the baud rate divisor registers */
	OUTBYTE(LCR(which), LCR_DLAB);
	OUTBYTE(BRDL(which), (unsigned char)(divisor & 0xff));
	OUTBYTE(BRDH(which), (unsigned char)((divisor >> 8) & 0xff));
	OUTBYTE(DLF(which), (unsigned char)(fdivisor & 0xf));

	/* specify the format of the asynchronous data communication exchange */
	OUTBYTE(LCR(which), init_info->async_format);

	mdc = MCR_OUT2 | MCR_RTS | MCR_DTR;
	if ((init_info->options & UART_OPTION_AFCE) == UART_OPTION_AFCE)
		mdc |= MCR_AFCE;

	OUTBYTE(MDC(which), mdc);

	/*
	 * Program FIFO: enabled, mode 0 (set for compatibility with quark),
	 * generate the interrupt at 8th byte
	 * Clear TX and RX FIFO
	 */
	OUTBYTE(FCR(which),
		FCR_FIFO | FCR_MODE0 | FCR_FIFO_8 | FCR_RCVRCLR | FCR_XMITCLR);

	/* clear the port */
	INBYTE(RDR(which));

	/* disable interrupts  */
	OUTBYTE(IER(which), 0x00);

	interrupt_unlock(oldLevel);
}

/*******************************************************************************
*
* uart_poll_in - poll the device for input.
*
* RETURNS: 0 if a character arrived, -1 if the input buffer if empty.
*/

int uart_poll_in(int which,		/* UART channel to select for input */
		 unsigned char *pChar /* pointer to char */
		 )
{
	if ((INBYTE(LSR(which)) & LSR_RXRDY) == 0x00)
		return (-1);

	/* got a character */
	*pChar = INBYTE(RDR(which));

	return 0;
}

/*******************************************************************************
*
* uart_poll_out - output a character in polled mode.
*
* Checks if the transmitter is empty. If empty, a character is written to
* the data register.
*
* If the hardware flow control is enabled then the handshake signal CTS has to
* be asserted in order to send a character.
*
* RETURNS: sent character
*/
unsigned char uart_poll_out(
	int which,	    /* UART channel to select for output */
	unsigned char outChar /* char to send */
	)
{
	/* wait for transmitter to ready to accept a character */
	while ((INBYTE(LSR(which)) & LSR_TEMT) == 0)
		;

	OUTBYTE(THR(which), outChar);

	return outChar;
}

/*******************************************************************************
*
* uart_fifo_fill - fill FIFO with data
*
* RETURNS: number of bytes sent
*/

int uart_fifo_fill(int which, /* UART on which to send */
			    const uint8_t *txData, /* data to transmit */
			    int size /* number of bytes to send */
			    )
{
	int i;

	for (i = 0; i < size && (INBYTE(LSR(which)) &
			LSR_BOTH_EMPTY) != 0; i++) {
		OUTBYTE(THR(which), txData[i]);
	}
	return i;
}

/*******************************************************************************
*
* uart_fifo_read - read data from FIFO
*
* RETURNS: number of bytes read
*/

int uart_fifo_read(int which, /* UART to receive from */
			    uint8_t *rxData, /* data container */
			    const int size   /* container size */
			    )
{
	int i;

	for (i = 0; i < size && (INBYTE(LSR(which)) & LSR_RXRDY) != 0; i++) {
		rxData[i] = INBYTE(RDR(which));
	}

	return i;
}

/*******************************************************************************
*
* uart_irq_tx_enable - enable TX interrupt in IER
*
* RETURNS: N/A
*/

void uart_irq_tx_enable(int which /* UART to enable Tx
					      interrupt */
				 )
{
	OUTBYTE(IER(which), INBYTE(IER(which)) | IER_TBE);
}

/*******************************************************************************
*
* uart_irq_tx_disable - disable TX interrupt in IER
*
* RETURNS: N/A
*/

void uart_irq_tx_disable(int which /* UART to disable Tx interrupt */
				  )
{
	OUTBYTE(IER(which), INBYTE(IER(which)) & (~IER_TBE));
}

/*******************************************************************************
*
* uart_irq_tx_ready - check if Tx IRQ has been raised
*
* RETURNS: N/A
*/

int uart_irq_tx_ready(int which /* UART to check */
			       )
{
	return ((IIRC(which) & IIR_ID) == IIR_THRE);
}

/*******************************************************************************
*
* _uart_irq_rx_enable - enable RX interrupt in IER
*
* RETURNS: N/A
*/

void uart_irq_rx_enable(int which /* UART to enable Rx
					      interrupt */
				 )
{
	OUTBYTE(IER(which), INBYTE(IER(which)) | IER_RXRDY);
}

/*******************************************************************************
*
* uart_irq_rx_disable - disable RX interrupt in IER
*
* RETURNS: N/A
*/

void uart_irq_rx_disable(int which /* UART to disable Rx interrupt */
				  )
{
	OUTBYTE(IER(which), INBYTE(IER(which)) & (~IER_RXRDY));
}

/*******************************************************************************
*
* uart_irq_rx_ready - check if Rx IRQ has been raised
*
* RETURNS: 1 if an IRQ is ready, 0 otherwise
*/

int uart_irq_rx_ready(int which /* UART to check */
			       )
{
	return ((IIRC(which) & IIR_ID) == IIR_RBRF);
}

/*******************************************************************************
*
* uart_irq_err_enable - enable error interrupt in IER
*
* RETURNS: N/A
*/

void uart_irq_err_enable(int which /* UART to enable Rx interrupt */
			 )
{
	OUTBYTE(IER(which), INBYTE(IER(which)) | IER_LSR);
}

/*******************************************************************************
*
* uart_irq_err_disable - disable error interrupt in IER
*
* RETURNS: N/A
*/

void uart_irq_err_disable(int which /* UART to disable Rx interrupt */
			  )
{
	OUTBYTE(IER(which), INBYTE(IER(which)) & (~IER_LSR));
}

/*******************************************************************************
*
* uart_irq_err_detected - check if line status IRQ has been raised
*
* RETURNS: 1 if an IRQ is detected, 0 otherwise
*/

int uart_irq_err_detected(int which /* UART to check */
			  )
{
	return (IIRC(which) & IIR_ID) == IIR_RLS;
}

/*******************************************************************************
*
* uart_line_status - returns line status
*
* RETURNS: line status register content
*/

int uart_line_status(int which /* UART to check */
			  )
{
	return INBYTE(LSR(which));
}

/*******************************************************************************
*
* uart_break_check - check if line status IRQ is due to break
*
* RETURNS: non null if a break has been detected, 0 otherwise
*/

int uart_break_check(int which /* UART to check */
			  )
{
	return INBYTE(LSR(which)) & LSR_BI;
}

/*******************************************************************************
*
* uart_break_send - send a break
*
* The delay implementation must be working in panic context where IRQs
* or other features are disabled, so a basic for loop is used.
* A value of (1<<12) is more or less 1 ms on Arduino 101/Lakemont (32 MHz).
*
* RETURNS: N/A
*/

void uart_break_send(int which, /* UART to check */
                    int delay  /* Delay (empty loop iteration count) */
			  )
{
	int ier = INBYTE(LCR(which));
	OUTBYTE(LCR(which), ier | LCR_SBRK);
	for (volatile int i = 0 ; i < delay; i++);
	OUTBYTE(LCR(which), ier);
}

/*******************************************************************************
*
* uart_disable - disables UART by resetting RTS and DTR
*
* Before going to sleep we might want to set the UART in such configuration
* that RTS and DTR are set, so that the connected device stops sending.
* You will need to call uart_init when you'll want to setup the port again.
*
* RETURNS: N/A
*/

void uart_disable(int which /* UART to check */
			   )
{
	uint8_t mdc = INBYTE(MDC(which));
	mdc |= MCR_OUT2;
	mdc &= ~(MCR_RTS | MCR_DTR | MCR_AFCE);
	OUTBYTE(MDC(which), mdc);
}

/*******************************************************************************
*
* uart_irq_is_pending - check if any IRQ is pending
*
* RETURNS: 1 if an IRQ is pending, 0 otherwise
*/

int uart_irq_is_pending(int which /* UART to check */
				 )
{
	return (!(IIRC(which) & IIR_IP));
}

/*******************************************************************************
*
* uart_irq_update - update cached contents of IIR
*
* RETURNS: always 1
*/

int uart_irq_update(int which /* UART to update */
			     )
{
	IIRC(which) = INBYTE(IIR(which));

	return 1;
}

/*******************************************************************************
*
* uart_int_connect - connect an ISR to an interrupt line
*
* The kernel configuration allows to setup an interrupt line for a particular
* DUART. This routine installs the ISR of a UART user to the interrupt line
* chosen for the hardware at configuration time.
*
* RETURNS: N/A
*/

void uart_int_connect(int which,	   /* UART to which to connect */
		      void (*isr)(void), /* interrupt handler */
		      void *arg,	   /* argument to pass to handler */
		      void *stub	   /* ptr to interrupt stub code */
		      )
{
	interrupt_connect((unsigned int)uart[which].irq, isr);
    interrupt_priority_set ((int)uart[which].irq, uart[which].intPri);
	interrupt_enable((unsigned int)uart[which].irq);
	/* set the Host Processor Interrupt Routing Mask */
	SOC_UNMASK_INTERRUPTS(INT_UART_0_MASK + (which * UART_REG_ADDR_INTERVAL));
}

/*******************************************************************************
*
* uart_tx_complete - check if tx holding and shift register are empty
*
* RETURNS: zero if registers are non-empty (transmission not complete), 
*          non-zero if registers are empty (transmission complete)
*/

uint8_t uart_tx_complete(int which)
{
	return INBYTE(LSR(which)) & LSR_TEMT;
}

/*******************************************************************************
*
* uart_loop_enable - enable loopback
*
* This function enable the local loopback.
* When enabled, the output of the Transmitter Shift
* Register is looped back into the Receiver Shift Register input,
* and any data that is transmitted is immediately received.
*
* RETURNS: N/A
*/

void uart_loop_enable(int which)
{
	uint8_t mdc = INBYTE(MDC(which));
	mdc |= MCR_LOOP;
	OUTBYTE(MDC(which), mdc);
}

/*******************************************************************************
*
* uart_loop_disable - disable loopback
*
* This function disable the local loopback.
*
* RETURNS: N/A
*/

void uart_loop_disable(int which)
{
	uint8_t mdc = INBYTE(MDC(which));
	mdc &= ~MCR_LOOP;
	OUTBYTE(MDC(which), mdc);
}

