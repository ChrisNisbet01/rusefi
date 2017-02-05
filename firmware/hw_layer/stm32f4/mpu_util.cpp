/**
 * @file	mpu_util.cpp
 *
 * @date Jul 27, 2014
 * @author Andrey Belomutskiy, (c) 2012-2017
 */

#include "main.h"
#include "mpu_util.h"
#include "error_handling.h"
#include "engine.h"
#include "pin_repository.h"

EXTERN_ENGINE;

extern "C" {
int getRemainingStack(thread_t *otp);
void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress);
}

extern uint32_t __main_stack_base__;

#define GET_CFSR() (*((volatile uint32_t *) (0xE000ED28)))

#if defined __GNUC__
// GCC version

int getRemainingStack(thread_t *otp) {

#if CH_DBG_ENABLE_STACK_CHECK
	// this would dismiss coverity warning - see http://rusefi.com/forum/viewtopic.php?f=5&t=655
	// coverity[uninit_use]
	register struct intctx *r13 asm ("r13");
	otp->activeStack = r13;

	int remainingStack;
	if (dbg_isr_cnt > 0) {
		// ISR context
		remainingStack = (int)(r13 - 1) - (int)&__main_stack_base__;
	} else {
		remainingStack = (int)(r13 - 1) - (int)otp->p_stklimit;
	}
	otp->remainingStack = remainingStack;
	return remainingStack;
#else
	return 99999;
#endif /* CH_DBG_ENABLE_STACK_CHECK */
}

#else /* __GNUC__ */

extern uint32_t CSTACK$$Base; /* symbol created by the IAR linker */
extern uint32_t IRQSTACK$$Base; /* symbol created by the IAR linker */

int getRemainingStack(Thread *otp) {
#if CH_DBG_ENABLE_STACK_CHECK || defined(__DOXYGEN__)
	int remainingStack;
	if (dbg_isr_cnt > 0) {
		remainingStack = (__get_SP() - sizeof(struct intctx)) - (int)&IRQSTACK$$Base;
	} else {
		remainingStack = (__get_SP() - sizeof(struct intctx)) - (int)otp->p_stklimit;
	}
	otp->remainingStack = remainingStack;
	return remainingStack;
#else
	return 999999;
#endif  
}

// IAR version

#endif

void baseHardwareInit(void) {
	// looks like this holds a random value on start? Let's set a nice clean zero
	DWT_CYCCNT = 0;
}

void DebugMonitorVector(void) {

	chDbgPanic3("DebugMonitorVector", __FILE__, __LINE__);

	while (TRUE)
		;
}

void UsageFaultVector(void) {

	chDbgPanic3("UsageFaultVector", __FILE__, __LINE__);

	while (TRUE)
		;
}

void BusFaultVector(void) {

	chDbgPanic3("BusFaultVector", __FILE__, __LINE__);

	while (TRUE) {
	}
}

/**
 + * @brief   Register values for postmortem debugging.
 + */
volatile uint32_t postmortem_r0;
volatile uint32_t postmortem_r1;
volatile uint32_t postmortem_r2;
volatile uint32_t postmortem_r3;
volatile uint32_t postmortem_r12;
volatile uint32_t postmortem_lr; /* Link register. */
volatile uint32_t postmortem_pc; /* Program counter. */
volatile uint32_t postmortem_psr;/* Program status register. */
volatile uint32_t postmortem_CFSR;
volatile uint32_t postmortem_HFSR;
volatile uint32_t postmortem_DFSR;
volatile uint32_t postmortem_AFSR;
volatile uint32_t postmortem_BFAR;
volatile uint32_t postmortem_MMAR;
volatile uint32_t postmortem_SCB_SHCSR;

/**
 * @brief   Evaluates to TRUE if system runs under debugger control.
 * @note    This bit resets only by power reset.
 */
#define is_under_debugger() (((CoreDebug)->DHCSR) & \
                            CoreDebug_DHCSR_C_DEBUGEN_Msk)

/**
 *
 */
void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress) {

	postmortem_r0 = pulFaultStackAddress[0];
	postmortem_r1 = pulFaultStackAddress[1];
	postmortem_r2 = pulFaultStackAddress[2];
	postmortem_r3 = pulFaultStackAddress[3];
	postmortem_r12 = pulFaultStackAddress[4];
	postmortem_lr = pulFaultStackAddress[5];
	postmortem_pc = pulFaultStackAddress[6];
	postmortem_psr = pulFaultStackAddress[7];

	/* Configurable Fault Status Register. Consists of MMSR, BFSR and UFSR */
	postmortem_CFSR = GET_CFSR();

	/* Hard Fault Status Register */
	postmortem_HFSR = (*((volatile uint32_t *) (0xE000ED2C)));

	/* Debug Fault Status Register */
	postmortem_DFSR = (*((volatile uint32_t *) (0xE000ED30)));

	/* Auxiliary Fault Status Register */
	postmortem_AFSR = (*((volatile uint32_t *) (0xE000ED3C)));

	/* Read the Fault Address Registers. These may not contain valid values.
	 Check BFARVALID/MMARVALID to see if they are valid values
	 MemManage Fault Address Register */
	postmortem_MMAR = (*((volatile uint32_t *) (0xE000ED34)));
	/* Bus Fault Address Register */
	postmortem_BFAR = (*((volatile uint32_t *) (0xE000ED38)));

	postmortem_SCB_SHCSR = SCB->SHCSR;

	if (is_under_debugger()) {
		__asm("BKPT #0\n");
		// Break into the debugger
	}

	/* harmless infinite loop */
	while (1) {
		;
	}
}

void HardFaultVector(void) {
#if 0 && defined __GNUC__
	__asm volatile (
			" tst lr, #4                                                \n"
			" ite eq                                                    \n"
			" mrseq r0, msp                                             \n"
			" mrsne r0, psp                                             \n"
			" ldr r1, [r0, #24]                                         \n"
			" ldr r2, handler2_address_const                            \n"
			" bx r2                                                     \n"
			" handler2_address_const: .word prvGetRegistersFromStack    \n"
	);

#else
#endif        

	int cfsr = GET_CFSR();
	if (cfsr & 0x1) {
		chDbgPanic3("H IACCVIOL", __FILE__, __LINE__);
	} else if (cfsr & 0x100) {
		chDbgPanic3("H IBUSERR", __FILE__, __LINE__);
	} else if (cfsr & 0x20000) {
		chDbgPanic3("H INVSTATE", __FILE__, __LINE__);
	} else {
		chDbgPanic3("HardFaultVector", __FILE__, __LINE__);
	}

	while (TRUE) {
	}
}

#if HAL_USE_SPI || defined(__DOXYGEN__)

bool isSpiInitialized[5] = { false, false, false, false, false };

typedef struct spi_config_st
{
    SPIDriver * driver;
    int alternate_function;

    brain_pin_e(* get_SPI_sck_pin)(board_configuration_s * board_configuration);
    int(* get_SPI_sck_mode)(engine_configuration_s * engine_configuration);

    brain_pin_e(* get_SPI_miso_pin)(board_configuration_s * board_configuration);
    int (* get_SPI_miso_mode)(engine_configuration_s * engine_configuration);

    brain_pin_e(* get_SPI_mosi_pin)(board_configuration_s * board_configuration);
    int (* get_SPI_mosi_mode)(engine_configuration_s * engine_configuration);

} spi_config_st;

static brain_pin_e get_unassigned_miso_pin(board_configuration_s * board_configuration)
{
    UNUSED(board_configuration);

    return GPIO_UNASSIGNED;
}

static brain_pin_e get_SPI1_miso_pin(board_configuration_s * board_configuration)
{
    return board_configuration->spi1misoPin;
}

static brain_pin_e get_SPI2_miso_pin(board_configuration_s * board_configuration)
{
    return board_configuration->spi2misoPin;
}

static brain_pin_e get_SPI3_miso_pin(board_configuration_s * board_configuration)
{
    return board_configuration->spi3misoPin;
}

static brain_pin_e get_unassigned_mosi_pin(board_configuration_s * board_configuration)
{
    UNUSED(board_configuration);

    return GPIO_UNASSIGNED;
}

static brain_pin_e get_SPI1_mosi_pin(board_configuration_s * board_configuration)
{
    return board_configuration->spi1mosiPin;
}

static brain_pin_e get_SPI2_mosi_pin(board_configuration_s * board_configuration)
{
    return board_configuration->spi2mosiPin;
}

static brain_pin_e get_SPI3_mosi_pin(board_configuration_s * board_configuration)
{
    return board_configuration->spi3mosiPin;
}

static brain_pin_e get_unassigned_sck_pin(board_configuration_s * board_configuration)
{
    UNUSED(board_configuration);

    return GPIO_UNASSIGNED;
}

static brain_pin_e get_SPI1_sck_pin(board_configuration_s * board_configuration)
{
    return board_configuration->spi1sckPin;
}

static brain_pin_e get_SPI2_sck_pin(board_configuration_s * board_configuration)
{
    return board_configuration->spi2sckPin;
}

static brain_pin_e get_SPI3_sck_pin(board_configuration_s * board_configuration)
{
    return board_configuration->spi3sckPin;
}

static int get_SPI_default_sck_mode(engine_configuration_s * engine_configuration)
{
    UNUSED(engine_configuration);

    return 0;
}

static int get_SPI_default_miso_mode(engine_configuration_s * engine_configuration)
{
    UNUSED(engine_configuration);

    return 0;
}

static int get_SPI_default_mosi_mode(engine_configuration_s * engine_configuration)
{
    UNUSED(engine_configuration);

    return 0;
}

static int get_SPI2_sck_mode(engine_configuration_s * engine_configuration)
{
    return engine_configuration->spi2SckMode;
}

static int get_SPI2_miso_mode(engine_configuration_s * engine_configuration)
{
    return engine_configuration->spi2MisoMode;
}

static int get_SPI2_mosi_mode(engine_configuration_s * engine_configuration)
{
    return engine_configuration->spi2MosiMode;
}

static spi_config_st const spi_configs[] =
{
    [SPI_NONE] = 
    {
        .driver = NULL,
        .alternate_function = -1,
        .get_SPI_sck_pin = get_unassigned_sck_pin,
        .get_SPI_sck_mode = get_SPI_default_sck_mode,
        .get_SPI_miso_pin = get_unassigned_miso_pin,
        .get_SPI_miso_mode = get_SPI_default_miso_mode,
        .get_SPI_mosi_pin = get_unassigned_mosi_pin,
        .get_SPI_mosi_mode = get_SPI_default_mosi_mode
    }
#if STM32_SPI_USE_SPI1
    , [SPI_DEVICE_1] =
    {
        .driver = &SPID1,
        .alternate_function = EFI_SPI1_AF,
        .get_SPI_sck_pin = get_SPI1_sck_pin,
        .get_SPI_sck_mode = get_SPI_default_sck_mode,
        .get_SPI_miso_pin = get_SPI1_miso_pin,
        .get_SPI_miso_mode = get_SPI_default_miso_mode,
        .get_SPI_mosi_pin = get_SPI1_mosi_pin,
        .get_SPI_mosi_mode = get_SPI_default_mosi_mode
    }
#endif
#if STM32_SPI_USE_SPI2
    , [SPI_DEVICE_2] =
    {
        .driver = &SPID2,
        .alternate_function = EFI_SPI2_AF,
        .get_SPI_sck_pin = get_SPI2_sck_pin,
        .get_SPI_sck_mode = get_SPI2_sck_mode,
        .get_SPI_miso_pin = get_SPI2_miso_pin,
        .get_SPI_miso_mode = get_SPI2_miso_mode,
        .get_SPI_mosi_pin = get_SPI2_mosi_pin,
        .get_SPI_mosi_mode = get_SPI2_mosi_mode
    }
#endif
#if STM32_SPI_USE_SPI3
    , [SPI_DEVICE_3] =
    {
        .driver = &SPID3,
        .alternate_function = EFI_SPI3_AF,
        .get_SPI_sck_pin = get_SPI3_sck_pin,
        .get_SPI_sck_mode = get_SPI_default_sck_mode,
        .get_SPI_miso_pin = get_SPI3_miso_pin,
        .get_SPI_miso_mode = get_SPI_default_miso_mode,
        .get_SPI_mosi_pin = get_SPI3_mosi_pin,
        .get_SPI_mosi_mode = get_SPI_default_mosi_mode
    }
#endif
};
#define SPI_CONFIG_SIZE ((size_t)sizeof(spi_configs) / (size_t)sizeof(spi_configs[0]))

spi_config_st const * get_SPI_config(spi_device_e const device)
{
    spi_config_st const * spi_config;

    if (device >= 0 && device < SPI_CONFIG_SIZE && spi_configs[device].driver != NULL)
    {
        spi_config = &spi_configs[device];
    }
    else
    {
        spi_config = NULL;
    }

    return spi_config;
}

brain_pin_e getMisoPin(spi_device_e device) {
    spi_config_st const * const spi_config = get_SPI_config(device);
    brain_pin_e miso_pin;

    if (spi_config == NULL)
    {
        miso_pin = GPIO_UNASSIGNED;
    }
    else
    {
        miso_pin = spi_config->get_SPI_miso_pin(boardConfiguration);
    }

    return miso_pin;
}

brain_pin_e getMosiPin(spi_device_e device) {
    spi_config_st const * const spi_config = get_SPI_config(device);
    brain_pin_e mosi_pin;

    if (spi_config == NULL)
    {
        mosi_pin = GPIO_UNASSIGNED;
    }
    else
    {
        mosi_pin = spi_config->get_SPI_mosi_pin(boardConfiguration);
    }

    return mosi_pin;
}

brain_pin_e getSckPin(spi_device_e device) {
    spi_config_st const * const spi_config = get_SPI_config(device);
    brain_pin_e sck_pin;

    if (spi_config == NULL)
    {
        sck_pin = GPIO_UNASSIGNED;
    }
    else
    {
        sck_pin = spi_config->get_SPI_sck_pin(boardConfiguration);
    }

    return sck_pin;
}

static void initSpiModule(spi_config_st const * const spi_config) {

    mySetPadMode2("SPI clock",
                  spi_config->get_SPI_sck_pin(boardConfiguration),
                  PAL_MODE_ALTERNATE(spi_config->alternate_function) + spi_config->get_SPI_sck_mode(engineConfiguration));

    mySetPadMode2("SPI master out",
                  spi_config->get_SPI_mosi_pin(boardConfiguration),
                  PAL_MODE_ALTERNATE(spi_config->alternate_function) + spi_config->get_SPI_mosi_mode(engineConfiguration));

    mySetPadMode2("SPI master in",
                  spi_config->get_SPI_miso_pin(boardConfiguration),
                  PAL_MODE_ALTERNATE(spi_config->alternate_function) + spi_config->get_SPI_miso_mode(engineConfiguration));
}

void turnOnSpi(spi_device_e device) {
    spi_config_st const * const spi_config = get_SPI_config(device);

    if (spi_config == NULL) {
        /* XXX - Log a message? */
        return;
    }
    if (isSpiInitialized[device]) {
		return; // already initialized
    }

    initSpiModule(spi_config);

    isSpiInitialized[device] = true;
}

void initSpiCs(SPIConfig *spiConfig, brain_pin_e csPin) {
	spiConfig->end_cb = NULL;
	ioportid_t port = getHwPort(csPin);
	ioportmask_t pin = getHwPin(csPin);
	spiConfig->ssport = port;
	spiConfig->sspad = pin;
	mySetPadMode2("chip select", csPin, PAL_STM32_MODE_OUTPUT);
}

#endif
