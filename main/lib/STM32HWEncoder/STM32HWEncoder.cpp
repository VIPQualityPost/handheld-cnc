#include "STM32HWEncoder.h"

#ifdef STM32G4xx

/**
 * Code taken and modified from the SimpleFOC Drivers repo, need to cite correctly.
 * https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/encoders/stm32hwencoder
 */

/*
  HardwareEncoder(int cpr)
*/
EncoderButton::EncoderButton(uint32_t ppr, int8_t pinA, int8_t pinB, int8_t switchPin) {
    cpr = ppr * 4; // 4x for quadrature
    _pinA = digitalPinToPinName(pinA);
    _pinB = digitalPinToPinName(pinB);
    _pinSwitch = digitalPinToPinName(switchPin);
}

// encoder initialisation of the hardware pins
void EncoderButton::init() {
    // GPIO configuration
    TIM_TypeDef *InstanceA = (TIM_TypeDef *)pinmap_peripheral(_pinA, PinMap_TIM);
    if (!IS_TIM_ENCODER_INTERFACE_INSTANCE(InstanceA)) {
        initialized = false;
        return;
    }
    TIM_TypeDef *InstanceB = (TIM_TypeDef *)pinmap_peripheral(_pinB, PinMap_TIM);
    if (InstanceA != InstanceB) {
        initialized = false;
        return;
    }
    pinmap_pinout(_pinA, PinMap_TIM);
    pinmap_pinout(_pinB, PinMap_TIM);

    // TODO check width:
    //IS_TIM_32B_COUNTER_INSTANCE(InstanceA);

    // set up timer for encoder
    encoder_handle.Init.Period = cpr - 1;
    encoder_handle.Init.Prescaler = 0;
    encoder_handle.Init.ClockDivision = 0;
    encoder_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    encoder_handle.Init.RepetitionCounter = 0;
    encoder_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef encoder_config;

    encoder_config.EncoderMode = TIM_ENCODERMODE_TI12;

    encoder_config.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC1Filter = 0;

    encoder_config.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC2Filter = 0;

    encoder_handle.Instance = InstanceA; // e.g. TIM4;
    enableTimerClock(&encoder_handle);

    if (HAL_TIM_Encoder_Init(&encoder_handle, &encoder_config) != HAL_OK) {
        initialized = false;
        return;
    }

    if (HAL_TIM_Encoder_Start(&encoder_handle, TIM_CHANNEL_1) != HAL_OK) {
        initialized = false;
        return;
    }

    initialized = true;
}

void EncoderButton::update(void)
{
    if(encoder_handle.Instance->CNT != last_position)
        ((void (*)())changed_cb)();
}

void EncoderButton::setChangedHandler(CallbackFunction f)
{

}

void EncoderButton::setPressedHandler(CallbackFunction f)
{
    EXTI_HandleTypeDef hexti;
    HAL_EXTI_RegisterCallback(&hexti, HAL_EXTI_COMMON_CB_ID, (void (*)())f);
}

void EncoderButton::setReleasedHandler(CallbackFunction f)
{

}

void EncoderButton::setClickHandler(CallbackFunction f)
{

}

void EncoderButton::setDoubleClickHandler(CallbackFunction f)
{

}

void EncoderButton::setTripleClickHandler(CallbackFunction f)
{

}

void EncoderButton::setEncoderHandler(CallbackFunction f)
{

}

void EncoderButton::resetPosition(long pos)
{
    
}

void EncoderButton::resetPressedPosition(long pos)
{

}

uint8_t EncoderButton::longPressCount(void)
{
    return 0;
}

bool EncoderButton::isPressed(void)
{
    return digitalRead(_pinSwitch);
}

int16_t EncoderButton::increment(void)
{
    uint16_t new_pos = encoder_handle.Instance->CNT;
    uint16_t delta = new_pos - last_position;
    last_position = new_pos;
    return delta;
}

long EncoderButton::position(void)
{
    return encoder_handle.Instance->CNT;
}

bool EncoderButton::enabled()
{
    return (encoder_handle.Instance->CR1 & TIM_CR1_CEN) == TIM_CR1_CEN;
}

void EncoderButton::enable(bool e)
{
    if(e)
        __HAL_TIM_ENABLE(&encoder_handle);
    else
        __HAL_TIM_DISABLE(&encoder_handle);
}

#endif