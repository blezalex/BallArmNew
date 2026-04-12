#include <stdint.h>

#include "cmsis_boot/stm32f10x.h"
#include "stm_lib/inc/stm32f10x_tim.h"
#include "stm_lib/inc/stm32f10x_rcc.h"


#define MICROS_TIMER TIM3
#define MICROS_TIMER_PERIPH RCC_APB1Periph_TIM3

void initMicrosTimer() {
	/***************** TIM3 ****************/

	// Conflicts with led!!

	RCC_APB1PeriphClockCmd(MICROS_TIMER_PERIPH, ENABLE);

	TIM_TimeBaseInitTypeDef TimerBaseInit;
	TIM_TimeBaseStructInit(&TimerBaseInit);

	TimerBaseInit.TIM_Prescaler =  SystemCoreClock / uint32_t {1000000} - 1; // 1us tick ;
	TimerBaseInit.TIM_Period = 0xFFFF;
	TimerBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
	TimerBaseInit.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(MICROS_TIMER,&TimerBaseInit);
	TIM_Cmd(MICROS_TIMER, ENABLE);
}

void initArduino() {
	SysTick_Config(SystemCoreClock / uint32_t {1000} - 1);
	initMicrosTimer();
}

static volatile uint32_t msTicks = 0;

extern "C" void SysTick_Handler() {
	msTicks++;
}

uint32_t millis() {
	return msTicks;
}

uint16_t micros() {
	return MICROS_TIMER->CNT;
}

void delay(uint32_t time) {
	uint32_t start_time = millis();

	while ((uint32_t)(millis() - start_time) < time);
}
