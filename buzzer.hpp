/**
 * @file buzzer.hpp
 * @author Evandro Teixeira
 * @brief 
 * @version 0.1
 * @date 21-08-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <Arduino.h>

/**
 * @brief 
 * 
 */
typedef struct
{
    uint8_t pin;
    uint8_t channel; 
}BuzzerPin_t;

typedef enum 
{
    Buzzer_Note_C = 0,
    Buzzer_Note_Cs,
    Buzzer_Note_D,
    Buzzer_Note_Eb,
    Buzzer_Note_E,
    Buzzer_Note_F,
    Buzzer_Note_Fs,
    Buzzer_Note_G,
    Buzzer_Note_Gs,
    Buzzer_Note_A,
    Buzzer_Note_Bb,
    Buzzer_Note_B,
    Buzzer_Note_MAX
}BuzzerNote_t;

/**
 * @brief 
 * 
 */
class buzzer
{
private:
    const BuzzerPin_t pin_buzzer;
    uint16_t frequency;
    uint8_t duty_cycle;
public:
    buzzer(BuzzerPin_t pin);
    ~buzzer();
    void begin(uint16_t freq, uint8_t dutyCycle);
    void set_duty_cycle(uint8_t dutyCycle);
    void set_frequency(uint16_t freq);
    void stop(void);
    void start(void);
    void update(void);
    void note(BuzzerNote_t note, uint8_t octave);
};


