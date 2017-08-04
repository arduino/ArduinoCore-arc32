#ifndef WSRC_H_
#define WSRC_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "board.h"
#include "pins_arduino.h"

#define GPIO_START                     0
#define GPIO_END                       (NUM_DIGITAL_PINS - 6)

#define GPIO_MODE_MASK                 0x38
#define SRC_STAT(src)                  (wsrc_table[src].status)

#define MAX_ATTACHABLE                 10
#define wsrc_cansleep(stat)            (stat & 1)
#define wsrc_working(stat)             (stat & 2)
#define wsrc_wakeup(stat)              (stat & 4)
#define wsrc_set_cansleep(stat)        (stat |= 1)
#define wsrc_clear_cansleep(stat)      (stat &= ~1)
#define wsrc_set_working(stat)         (stat |= 2)
#define wsrc_clear_working(stat)       (stat &= ~2)
#define wsrc_set_wakeup(stat)          (stat |= 4)
#define wsrc_clear_wakeup(stat)        (stat &= ~4)
#define wsrc_gpio_mode(stat)           ((stat & GPIO_MODE_MASK) >> 3)
#define wsrc_set_gpio_mode(stat, mode)({\
                        stat &= ~GPIO_MODE_MASK;\ 
                        stat |= (mode << 3);\
                        });

enum {
    /* TODO: not all of these can actually be used as wakeup sources.
     * find out which ones they are. */
    AON_GPIO0 = GPIO_END ,
    AON_GPIO1 = GPIO_END + 1,
    AON_GPIO2 = GPIO_END + 2,
    AON_GPIO3 = GPIO_END + 3 ,
    IMU_WAKEUP = GPIO_END + 4,
    BLE_WAKEUP = GPIO_END + 5,
    I2S_WAKEUP = GPIO_END + 6,
    WIRE_RX_WAKEUP = GPIO_END + 7,
    SERIAL_WAKEUP = GPIO_END + 8,
    SERIAL1_WAKEUP = GPIO_END + 9,
    ADC_WAKEUP = GPIO_END + 10,
    AON_TIMER_WAKEUP = GPIO_END + 11,
    TIMER1_WAKEUP = GPIO_END + 12,
    PWM_TIMER_WAKEUP = GPIO_END + 13,
    SPI_RX_WAKEUP = GPIO_END + 14,
    SPI1_RX_WAKEUP = GPIO_END + 15,
    RTC_WAKEUP = GPIO_END + 16,
    WATCHDOG_WAKEUP = GPIO_END + 17,
    MAILBOX_WAKEUP = GPIO_END + 18,
    COMPARATORS_WAKEUP = GPIO_END + 19,
    NUM_WAKEUP = GPIO_END + 20
};

struct wsrc_entry {
    /* IRQ associated with this wakeup source */
    uint8_t irq;
    /* Status flags;
     *
     * Bit 0:  sleep:         indicates if this wakeup source can be used in
     *                        deeper sleep modes (if unset, can only be used in
     *                        doze mode)
     * Bit 1:  working:       indicates this entry is waiting for something
     *                        to complete in interrupt context (e.g.
     *                        interrupt-based UART transfer in progress)
     * Bit 2:  wakeup:        indicates this entry is to be used as a wakeup
     *                        source
     * Bits 3-5: GPIO mode:   0 - LOW
     *                        1 - HIGH
     *                        2 - CHANGE
     *                        3 - FALLING
     *                        4 - RISING
     *
     * Bits 6-7: Reserved.    */
    volatile uint8_t status;
} __attribute__((packed));

struct wsrc_active {
    uint32_t cb;
    uint8_t index;
} __attribute__((packed));

struct wsrc {
    uint32_t callback;
    uint8_t status;
    uint8_t irq;
    uint8_t id;
};

typedef struct wsrc wsrc_t;
typedef struct wsrc_entry wsrc_entry_t;
typedef struct wsrc_active wsrc_active_t;

extern wsrc_entry_t wsrc_table[NUM_WAKEUP];

void wsrc_table_init (void);
void wsrc_register_gpio (uint32_t pin, void (*callback)(void), uint32_t mode);
void wsrc_register_id (int id, void (*callback)(void));
void wsrc_unregister (int id);
int wsrc_getIndex(int id);

int wsrc_get_newest_attached (wsrc_t *wsrc);
int wsrc_get_oldest_attached (wsrc_t *wsrc);

#ifdef __cplusplus
}
#endif

#endif
