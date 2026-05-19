#include "footpadGuard.hpp"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

namespace {
	constexpr uint16_t kFootpadInactiveLevel = 0;
	constexpr uint16_t kFootpadActiveLevel = 4095;
	constexpr uint16_t kFootpadSwitchPin = GPIO_Pin_0;
}

void initFootpadInput() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin   = kFootpadSwitchPin;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void FootpadGuard::Update() {
	const uint16_t footpad_level = GPIO_ReadInputDataBit(GPIOA, kFootpadSwitchPin)
		? kFootpadInactiveLevel
		: kFootpadActiveLevel;

	for (int i = 0; i < 2; i++) {
		padLevelFilter[0].compute(footpad_level);
		padLevelFilter[1].compute(footpad_level);
	}

	if (!seen_booth_off) {
		// check if both off now
		if (padLevelFilter[0].getVal() < settings_->min_level_to_continue && padLevelFilter[1].getVal() < settings_->min_level_to_continue) {
			seen_booth_off = true;
		}
	}
}

FootpadGuard::FootpadGuard(const Config_FootPadSettings* settings)
 : padLevelFilter {&settings->filter_rc, &settings->filter_rc}, settings_(settings) {
	initFootpadInput();
}

bool FootpadGuard::CanStart() {
	if (!seen_booth_off) {
		// sensor failure or still pressed since start.
		return false;
	}
	return padLevelFilter[0].getVal() > settings_->min_level_to_start && padLevelFilter[1].getVal() > settings_->min_level_to_start;
}

// Stop if one of the footpads below the threshold for at least shutoff_delay_ms
bool FootpadGuard::MustStop() {
	bool stop_condition = padLevelFilter[0].getVal() < settings_->min_level_to_continue || padLevelFilter[1].getVal() < settings_->min_level_to_continue;

	if (!stop_condition) {
		stop_requested_ = false;
		return false;
	}

	if (!stop_requested_) {
		stop_requested_ = true;
		stop_request_timestamp_ = millis();
	}

	return (uint16_t)(millis() - stop_request_timestamp_) > settings_->shutoff_delay_ms;
}
