#include <stdio.h>
#include <stddef.h>

#include "cpu.h"
#include "sched.h"
#include "vtimer.h"

#include "cpu.h"

/* drivers  */
#include "cc110x_ng.h"

#define	CSn		GPIO_Pin_4
#define GDO0_line	EXTI_Line0
#define GDO2_line	EXTI_Line1

#define CC1100_GET_GDO0()         (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0))	// read serial I/O (GDO0)
#define CC1100_GET_MISO()         (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6))	// read serial I/O (SO_GDO1)
#define CC1100_GET_GDO2()         (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1))	// read serial I/O (GDO2)

#define CC1100_MISO_LOW_RETRY		 (100)		// max. retries for MISO to go low
#define CC1100_MISO_LOW_COUNT		(2700)		// loop count (timeout ~ 500 us) to wait

#ifdef DEBUG

#include "stdio.h"

static unsigned long time_value;

static void set_time(void)
{
    time_value = 0;
}

static int test_time(int code)
{
    time_value++;

    if (time_value > 10000000) {
        printf("CC1100 SPI alarm: %u!\n", code);
        time_value = 0;
        return 1;
    }

    return 0;
}
#endif

int cc110x_get_gdo0(void)
{
    return CC1100_GET_GDO0();
}

int cc110x_get_gdo1(void)
{
    return CC1100_GET_MISO();
}

int cc110x_get_gdo2(void)
{
    return CC1100_GET_GDO2();
}

static
void Enable_EXTI_interrupt(EXTITrigger_TypeDef EXTI_trigger, uint32_t EXTI_line)
{
    EXTI_InitTypeDef   EXTI_InitStructure;

    EXTI_InitStructure.EXTI_Line = EXTI_line;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_trigger;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

static
void Disable_EXTI_interrupt(uint32_t EXTI_line)
{
    EXTI_InitTypeDef   EXTI_InitStructure;

    EXTI_InitStructure.EXTI_Line = EXTI_line;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
}

void cc110x_spi_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	/* RCC */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* GPIO */
	/* Configure SPI MASTER pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*
	 * SPI
	 * NOTE: APB2 is 72MHz, prescaler 16 => SPI @ 4.5 MHz, radio spi max is 7.5MHz
	 * Clock idle low, rising edge
	 */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	/* Enable SPI */
	SPI_Cmd(SPI1, ENABLE);
}

uint8_t cc110x_txrx(uint8_t value)
{
    uint8_t retval;

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
#ifdef DEBUG
        test_time(0);
#endif
    }

    SPI_I2S_SendData(SPI1, value);
#ifdef DEBUG
    set_time();
#endif

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
#ifdef DEBUG
        test_time(2);
#endif
    }

    retval = SPI_I2S_ReceiveData(SPI1);
    return retval;
}

void cc110x_spi_cs(void)
{
    GPIO_ResetBits(GPIOA, CSn);
}

void cc110x_spi_unselect(void)
{
    GPIO_SetBits(GPIOA, CSn);
}

void cc110x_spi_select(void)
{
	volatile int retry_count = 0;
	volatile int abort_count;

    cc110x_spi_cs();
    while(CC1100_GET_MISO()) {
    	cc110x_spi_cs();
		abort_count++;
		if (abort_count > CC1100_MISO_LOW_COUNT) {
			retry_count++;
			if (retry_count > CC1100_MISO_LOW_RETRY) {
				puts("[CC1100 SPI] fatal error\n");
				return;
			}
			cc110x_spi_unselect();		// CS to high
		}
    }
}

void cc110x_before_send(void)
{
    //Disable GDO2 interrupt before sending packet
    cc110x_gdo2_disable();
}

void cc110x_after_send(void)
{
    //Enable GDO2 interrupt after sending packet
    cc110x_gdo2_enable();
}

void cc110x_gdo0_enable(void)
{
	Enable_EXTI_interrupt(EXTI_Trigger_Rising, GDO0_line);
}

void cc110x_gdo2_enable(void)
{
	Enable_EXTI_interrupt(EXTI_Trigger_Falling, GDO2_line);
}

void cc110x_gdo0_disable(void)
{
	Disable_EXTI_interrupt(GDO0_line);
}

void cc110x_gdo2_disable(void)
{
	Disable_EXTI_interrupt(GDO2_line);
}

void Init_interrupt(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTI0 and EXTI1 Line to PB0 and PB1 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

	/* Configure EXTI0 and EXTI1 line */
	Enable_EXTI_interrupt(EXTI_Trigger_Rising, GDO0_line);
	Enable_EXTI_interrupt(EXTI_Trigger_Falling, GDO2_line);

	/* Enable and set EXTI4 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

void cc110x_init_interrupts(void)
{
	Init_interrupt();
}

extern void cc110x_gdo0_irq(void);
__attribute__((naked))
void EXTI0_IRQHandler(void)
{
    save_context();

    if (EXTI_GetITStatus(GDO0_line) != RESET) {
        EXTI_ClearITPendingBit(GDO0_line);

        cc110x_gdo0_irq();

        if (sched_context_switch_request) {
            /* scheduler */
            thread_yield();
        }
    }

    restore_context();
}

extern void cc110x_gdo2_irq(void);
__attribute__((naked))
void EXTI1_IRQHandler(void)
{
    save_context();

    if (EXTI_GetITStatus(GDO2_line) != RESET) {
        EXTI_ClearITPendingBit(GDO2_line);

        cc110x_gdo2_irq();

        if (sched_context_switch_request) {
            /* scheduler */
            thread_yield();
        }
    }

    restore_context();
}
