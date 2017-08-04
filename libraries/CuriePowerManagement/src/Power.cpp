#include "Power.h"

Power PM;
uint32_t arc_restore_addr;
uint32_t cpu_context[33];

typedef void (*user_cb)(void);

static void sleepInterruptHandler(void)
{
    unsigned int flags = interrupt_lock();
    PM.wakeFromDoze();
    PM.wakeFromSleepCallback();
    interrupt_unlock(flags);
}

static void dozeInterruptHandler(void)
{
    unsigned int flags = interrupt_lock();
    PM.wakeFromDoze();
    interrupt_unlock(flags);
}

static void soc_gpio_sleep_isr()
{
    if(soc_sleeping || soc_dozing)
    {
        unsigned int flags = interrupt_lock();
        PM.wakeFromDoze();
        wsrc_t wsrc;
        while (wsrc_get_newest_attached(&wsrc)){
            PinDescription *p = &g_APinDescription[wsrc.id];
            uint32_t mask = 0x1 << p->ulGPIOId;
            if(p->ulGPIOPort == SOC_GPIO_32){
                uint32_t status = shared_data->pm_int_status;
                if((status>>p->ulGPIOId)&0x1){
                    //call user callback
                    user_cb cb = (user_cb)wsrc.callback;
                    cb();
                }
            }
        }
        interrupt_unlock(flags);
    }
    soc_sleeping = false;
}

static void soc_aongpio_sleep_isr()
{
    if(soc_sleeping || soc_dozing){
        unsigned int flags = interrupt_lock();
        PM.wakeFromDoze();
        wsrc_t wsrc;
        while (wsrc_get_newest_attached(&wsrc)){
            PinDescription *p = &g_APinDescription[wsrc.id];
            uint32_t mask = 0x1 << p->ulGPIOId;
            if(p->ulGPIOPort == SOC_GPIO_32){
                // Save interrupt status
                uint32_t status = MMIO_REG_VAL_FROM_BASE(SOC_GPIO_AON_BASE_ADDR, SOC_GPIO_INTSTATUS);
                // Mask the pending interrupts
                MMIO_REG_VAL_FROM_BASE(SOC_GPIO_AON_BASE_ADDR, SOC_GPIO_INTMASK) |= status;
                shared_data->pm_int_status = status;
                // Clear interrupt flag (write 1 to clear)
                MMIO_REG_VAL_FROM_BASE(SOC_GPIO_AON_BASE_ADDR, SOC_GPIO_PORTA_EOI) = status;
                if((status>>p->ulGPIOId)&0x1){
                    //call user callback
                    user_cb cb = (user_cb)wsrc.callback;
                    cb();
                }
            }
        }
        interrupt_unlock(flags);
    }
    soc_sleeping = false;
}

static void ss_gpio0_sleep_isr()
{
    if(soc_sleeping || soc_dozing){
        unsigned int flags = interrupt_lock();
        PM.wakeFromDoze();
        wsrc_t wsrc;
        while (wsrc_get_newest_attached(&wsrc)) {
            PinDescription *p = &g_APinDescription[wsrc.id];
            uint32_t mask = 0x1 << p->ulGPIOId;
            if(p->ulGPIOPort == SS_GPIO_8B0){
                uint32_t status = shared_data->pm_int_status;
                if((status>>p->ulGPIOId)&0x1){
                    //call user callback
                    user_cb cb = (user_cb)wsrc.callback;
                    cb();
                }
            }
        }
        interrupt_unlock(flags);
    }
    soc_sleeping = false;
}

static void ss_gpio1_sleep_isr()
{
    if(soc_sleeping || soc_dozing){
        unsigned int flags = interrupt_lock();
        PM.wakeFromDoze();
        wsrc_t wsrc;
        while (wsrc_get_newest_attached(&wsrc)) {
            PinDescription *p = &g_APinDescription[wsrc.id];
            uint32_t mask = 0x1 << p->ulGPIOId;
            if(p->ulGPIOPort == SS_GPIO_8B1){
                uint32_t status = shared_data->pm_int_status;
                if((status>>p->ulGPIOId)&0x1){
                    //call user callback
                    user_cb cb = (user_cb)wsrc.callback;
                    cb();
                }
            }
        }
        interrupt_unlock(flags);
    }
    soc_sleeping = false;
}

static void aontimer_isr()
{
    if(soc_sleeping || soc_dozing){
        unsigned int flags = interrupt_lock();
        interrupt_disable(IRQ_ALWAYS_ON_TMR);
        *AONPT_CFG = 0;
        PM.wakeFromDoze();
        interrupt_unlock(flags);
        //wait until quark core is running
        volatile uint32_t psts = (MMIO_REG_VAL(P_STS))&0x7;
        while(psts){
          psts = (MMIO_REG_VAL(P_STS))&0x7;
        }
        PM.wakeFromSleepCallback();
    }
    soc_sleeping = false;
}

Power::Power()
{
    
}

void Power::doze()
{
    //actually attach the interrupts
    enableWakeInterrupts();
    turnOffUSB();
    soc_dozing = true;
    //switch from external crystal oscillator to internal hybrid oscillator
    switchToHybridOscillator();

    //Set system clock to the RTC Crystal Oscillator
    uint32_t current_val = MMIO_REG_VAL(CCU_SYS_CLK_CTL);
    MMIO_REG_VAL(CCU_SYS_CLK_CTL) = current_val & 0xFFFFFFFE;

    //Powerdown hybrid oscillator
    current_val = MMIO_REG_VAL(OSC0_CFG1);
    MMIO_REG_VAL(OSC0_CFG1) = current_val | 0x00000004; 
}

void Power::doze(int duration)
{
    doze();
    delayTicks(millisToRTCTicks(duration));
    wakeFromDoze();
}

void Power::idle()
{
    doze();
    while(soc_dozing);
}

void Power::idle(int duration)
{
    enableAONPTimerInterrrupt(duration);
    idle();
}

void Power::wakeFromDoze()
{
    //Powerup hybrid oscillator
    uint32_t current_val = MMIO_REG_VAL(OSC0_CFG1);
    MMIO_REG_VAL(OSC0_CFG1) = current_val & 0xFFFFFFFB;
   
    //Set system clock to the Hybrid Oscillator
    current_val = MMIO_REG_VAL(CCU_SYS_CLK_CTL);
    MMIO_REG_VAL(CCU_SYS_CLK_CTL) = current_val | 0x00000001;

    //switch back to the external crystal oscillator
    void switchToCrystalOscillator();
  
    turnOnUSB();
    soc_dozing = false;
}

void Power::sleep()
{
    soc_sleeping = true;
    //disable low power mode on OPM_2P6 regulator
    MMIO_REG_VAL(SLP_CFG) &= ~(1 << LPMODE_EN);
    
    uint32_t creg_mst0_ctrl = 0;
    creg_mst0_ctrl = READ_ARC_REG(QM_SS_CREG_BASE);
    
    /*
	  * Clock gate the sensor peripherals at CREG level.
	  * This clock gating is independent of the peripheral-specific clock
	  * gating provided in ss_clk.h .
    */
    creg_mst0_ctrl |= (QM_SS_IO_CREG_MST0_CTRL_ADC_CLK_GATE |
        QM_SS_IO_CREG_MST0_CTRL_I2C1_CLK_GATE |
	    QM_SS_IO_CREG_MST0_CTRL_I2C0_CLK_GATE |
	    QM_SS_IO_CREG_MST0_CTRL_SPI1_CLK_GATE |
	    QM_SS_IO_CREG_MST0_CTRL_SPI0_CLK_GATE);
    
    WRITE_ARC_REG(creg_mst0_ctrl, QM_SS_CREG_BASE);
    
    //Send request to put quark core to sleep
    //x86_C2LPRequest(); //can only wake Quark core using AON_GPIO, RTC, AON_Timer, and AON_Comparators
    x86_C2Request();
    doze();
   
    __asm__ __volatile__(
		"sleep %0"
		:
		: "i"(QM_SS_SLEEP_MODE_CORE_TIMERS_RTC_OFF));
    
    creg_mst0_ctrl &= ~(QM_SS_IO_CREG_MST0_CTRL_ADC_CLK_GATE |
		QM_SS_IO_CREG_MST0_CTRL_I2C1_CLK_GATE |
		QM_SS_IO_CREG_MST0_CTRL_I2C0_CLK_GATE |
		QM_SS_IO_CREG_MST0_CTRL_SPI1_CLK_GATE |
		QM_SS_IO_CREG_MST0_CTRL_SPI0_CLK_GATE);
                
    WRITE_ARC_REG(creg_mst0_ctrl, QM_SS_CREG_BASE);
    soc_sleeping = false;
}

void Power::sleep(int duration)
{
    enableAONPTimerInterrrupt(duration);
    sleep();
}

void Power::deepSleep()
{
   sleep();
}

void Power::deepSleep(int duration)
{
    sleep(duration);
}

inline void Power::wakeFromSleepCallback(void)
{
    //ToDo: check table and call apprpriate CBs
    if(pmCB != NULL)
        pmCB();
    //ToDo: unregister all sleep IRQs
}

inline void Power::wakeFromDozeCallback(void)
{
    //ToDo: check table and call apprpriate CBs
    if(pmCB != NULL)
        pmCB();
}

void Power::attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode)
{
    if (pin > NUM_WAKEUP) {
        return;
    }

    if (pin <= GPIO_END) {
        wsrc_register_gpio(pin, callback, mode);
    }
    else{
        wsrc_register_id(pin, callback);
    }
}

void Power::detachInterruptWakeup(uint32_t pin)
{
    wsrc_unregister(pin);
    if (pin <= GPIO_END) {
        detachInterrupt(pin);
    }
}

//Privates

void Power::turnOffUSB()
{
    MMIO_REG_VAL(USB_PHY_CFG0) |= 0x00000001; 
}

void Power::turnOnUSB()
{
    MMIO_REG_VAL(USB_PHY_CFG0) &= 0xFFFFFFFE;
}

void Power::switchToHybridOscillator()
{
    //read trim value from OTP
    uint32_t trimMask = *(uint16_t*)OSCTRIM_ADDR << 20;
    MMIO_REG_VAL(OSC0_CFG1) = 0x00000002 | trimMask;  //switch to internal hybrid oscillator using trim value from OTP
    //ToDo: wait for hybrid oscillator to stabilize
}

void Power::switchToCrystalOscillator()
{
    MMIO_REG_VAL(OSC0_CFG1) = 0x00070009;
    while(!(MMIO_REG_VAL(OSC0_STAT) & 0x00000002));   //wait till crystal oscillator is stable
}

void Power::setRTCCMR(int seconds)
{
    MMIO_REG_VAL(RTC_CMR) = readRTC_CCVR() + seconds;
}

uint32_t Power::readRTC_CCVR()
{
    return *RTC_CCVR;
}

uint32_t Power::millisToRTCTicks(int milliseconds)
{
    return (uint32_t)((double)milliseconds*32.768);
}

void Power::enableRTCInterrupt(int seconds)
{
    setRTCCMR(seconds);
    MMIO_REG_VAL(RTC_MASK_INT) &= 0xFFFFFEFE;
    MMIO_REG_VAL(RTC_CCR) |= 0x00000001;
    MMIO_REG_VAL(RTC_CCR) &= 0xFFFFFFFD;
    volatile uint32_t read = MMIO_REG_VAL(RTC_EOI);
    
    pmCB = &wakeFromRTC;
    interrupt_disable(IRQ_RTC_INTR);
    interrupt_connect(IRQ_RTC_INTR , &sleepInterruptHandler);
    delayTicks(6400);   //2ms
    interrupt_enable(IRQ_RTC_INTR);
}

void Power::enableAONGPIOInterrupt(int aon_gpio, int mode)
{
    switch(mode){
        case CHANGE:    //not supported just do the same as FALLING
            MMIO_REG_VAL(AON_GPIO_INTTYPE_LEVEL) |= 1 << aon_gpio;
            MMIO_REG_VAL(AON_GPIO_INT_POL) &= ~(1 << aon_gpio);
            break;
        case RISING:
            MMIO_REG_VAL(AON_GPIO_INTTYPE_LEVEL) |= 1 << aon_gpio;
            MMIO_REG_VAL(AON_GPIO_INT_POL) |= 1 << aon_gpio;
            break;
        case FALLING:
            MMIO_REG_VAL(AON_GPIO_INTTYPE_LEVEL) |= 1 << aon_gpio;
            MMIO_REG_VAL(AON_GPIO_INT_POL) &= ~(1 << aon_gpio);
            break;
        case HIGH:
            MMIO_REG_VAL(AON_GPIO_INTTYPE_LEVEL) &= ~(1 << aon_gpio);
            MMIO_REG_VAL(AON_GPIO_INT_POL) |= 1 << aon_gpio;
            break;
        case LOW:
            MMIO_REG_VAL(AON_GPIO_INTTYPE_LEVEL) &= ~(1 << aon_gpio);
            MMIO_REG_VAL(AON_GPIO_INT_POL) &= ~(1 << aon_gpio);
            break;
        default:
            MMIO_REG_VAL(AON_GPIO_INTTYPE_LEVEL) &= ~(1 << aon_gpio);
            MMIO_REG_VAL(AON_GPIO_INT_POL) &= ~(1 << aon_gpio);
            break;
    };
    
    MMIO_REG_VAL(AON_GPIO_SWPORTA_DDR) &= ~(1 << aon_gpio);
    MMIO_REG_VAL(AON_GPIO_INTMASK) &= ~(1 << aon_gpio);
    MMIO_REG_VAL(AON_GPIO_INTEN) |= 1 << aon_gpio;
    
    *AON_GPIO_MASK_INT &= 0xFFFFFEFE;
    interrupt_disable(IRQ_ALWAYS_ON_GPIO);
    interrupt_connect(IRQ_ALWAYS_ON_GPIO , &soc_aongpio_sleep_isr);
    interrupt_enable(IRQ_ALWAYS_ON_GPIO);
}

void Power::enableAONPTimerInterrrupt(int millis)
{
    WRITE_ARC_REG(QM_IRQ_AONPT_0_INT_VECTOR, QM_SS_AUX_IRQ_SELECT);
	WRITE_ARC_REG(QM_SS_IRQ_LEVEL_SENSITIVE, QM_SS_AUX_IRQ_TRIGGER);
    
    interrupt_disable(IRQ_ALWAYS_ON_TMR);
    pmCB = resetAONPTimer;
    *AONPT_CFG = millisToRTCTicks(millis);
    interrupt_connect(IRQ_ALWAYS_ON_TMR , &aontimer_isr);
    *AON_TIMER_MASK_INT &= 0xFFFFFEFE;
    *AONPT_CTRL |= 0x00000002;
    *AONPT_CTRL |= 0x00000001;  
    volatile uint32_t aonpt_stat = *AONPT_STAT;
    while(aonpt_stat){
       *AONPT_CTRL &= 0xFFFFFFFE;
       *AONPT_CTRL |= 0x00000001;
       aonpt_stat = *AONPT_STAT;
       //delayTicks(1000); 
    }
    interrupt_enable(IRQ_ALWAYS_ON_TMR);
}

void Power::enableWakeInterrupts()
{
    void (*fptr)(void);
    wsrc_t wsrc;
 
    while (wsrc_get_newest_attached(&wsrc)) {
        switch(wsrc.irq){
        case IRQ_ALWAYS_ON_GPIO:
            enableAONGPIOInterrupt((wsrc.id-GPIO_END), wsrc_gpio_mode(wsrc.status));
            break;
        case IRQ_ALWAYS_ON_TMR:
            break;
        case IRQ_RTC_INTR:
            break;
        case IRQ_GPIO0_INTR:
            attachInterrupt(wsrc.id, &ss_gpio0_sleep_isr, wsrc_gpio_mode(wsrc.status));
            interrupt_enable(wsrc.irq);
            break;
        case IRQ_GPIO1_INTR:
            attachInterrupt(wsrc.id, &ss_gpio1_sleep_isr, wsrc_gpio_mode(wsrc.status));
            interrupt_enable(wsrc.irq);
            break;
        case IRQ_GPIO_INTR:
            attachInterrupt(wsrc.id, &soc_gpio_sleep_isr, wsrc_gpio_mode(wsrc.status));
            interrupt_enable(wsrc.irq);
            break;
        case IRQ_TIMER1:
            break;
        default:
            break;
        }
    }   
}

void Power::resetAONPTimer()
{
    interrupt_disable(IRQ_ALWAYS_ON_TMR);
    *AON_TIMER_MASK_INT |= 0xFFFFF1F1;
    *AONPT_CFG = 0;
    *AONPT_CTRL |= 0x00000001;
    WRITE_ARC_REG(QM_IRQ_AONPT_0_INT_VECTOR, QM_SS_AUX_IRQ_SELECT);
	WRITE_ARC_REG(QM_SS_IRQ_EDGE_SENSITIVE, QM_SS_AUX_IRQ_TRIGGER);
}

void Power::wakeFromRTC()
{
    MMIO_REG_VAL(RTC_MASK_INT) |= 0x00000101;
    interrupt_disable(IRQ_RTC_INTR);
    volatile uint32_t read = MMIO_REG_VAL(RTC_EOI);
}

void Power::x86_C2Request()
{
    switchToHybridOscillator();
    //request for the x86 core go into C2 sleep
    MMIO_REG_VAL(CCU_LP_CLK_CTL) &= 0xFFFFFFFC;
    volatile uint32_t c2 = MMIO_REG_VAL(P_LVL2);
}

void Power::x86_C2LPRequest()
{
    switchToHybridOscillator();
    //request for the x86 core go into C2LP sleep
    MMIO_REG_VAL(CCU_LP_CLK_CTL) &= 0xFFFFFFFE;
    MMIO_REG_VAL(CCU_LP_CLK_CTL) |= 0x00000002;
    volatile uint32_t c2lp = MMIO_REG_VAL(P_LVL2);
}
