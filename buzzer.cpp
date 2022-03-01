/**
 * @file buzzer.cpp
 * @author Evandro Teixeira
 * @brief 
 * @version 0.1
 * @date 21-08-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "buzzer.hpp"
#include "driver/gpio.h"

#define BUZZER_RESOLUTION   12

/**
 * @brief Construct a new buzzer::buzzer object
 * 
 * @param pin 
 */
buzzer::buzzer(BuzzerPin_t pin) : pin_buzzer(pin)
{
    /* */
}

/**
 * @brief Destroy the buzzer::buzzer object
 * 
 */
buzzer::~buzzer()
{
    /* */
}

/**
 * @brief 
 * 
 * @param freq 
 * @param dutyCycle 
 */
void buzzer::begin(uint16_t freq, uint8_t dutyCycle)
{
    frequency = freq;
    duty_cycle = dutyCycle;

    pinMode(pin_buzzer.pin,OUTPUT);
    ledcSetup(pin_buzzer.channel,frequency,BUZZER_RESOLUTION);
    ledcAttachPin(pin_buzzer.pin,pin_buzzer.channel);
    ledcWrite(pin_buzzer.channel, 0x00 );
    digitalWrite(pin_buzzer.pin, LOW );
}

/**
 * @brief Set the duty cycle object
 * 
 * @param dutyCycle 
 */
void buzzer::set_duty_cycle(uint8_t dutyCycle)
{
    duty_cycle = dutyCycle;
}

/**
 * @brief Set the frequency object
 * 
 * @param freq 
 */
void buzzer::set_frequency(uint16_t freq)
{
    frequency = freq;
}

/**
 * @brief 
 * 
 */
void buzzer::stop(void)
{
    ledcWrite(pin_buzzer.channel, 0x00 );
    digitalWrite(pin_buzzer.pin, LOW );
}

/**
 * @brief 
 * 
 */
void buzzer::start(void)
{
    ledcWrite(pin_buzzer.channel, duty_cycle );
}

/**
 * @brief 
 * 
 */
void buzzer::update(void)
{
    start();
}

/**
 * @brief 
 * 
 * @param note 
 */
void buzzer::note(BuzzerNote_t note, uint8_t octave)
{
    ledcWriteNote(pin_buzzer.channel, (note_t) note, octave);
}