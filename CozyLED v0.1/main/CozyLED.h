#ifndef COZYLED_H
#define COZYLED_H

#include <inttypes.h>


// GPIO Defines
#define LED_R   GPIO_NUM_26
#define LED_G   GPIO_NUM_25
#define LED_B   GPIO_NUM_33
#define LED_PWR GPIO_NUM_27
#define L_BTN   GPIO_NUM_35
#define R_BTN   GPIO_NUM_34

// LED Bit Mask
#define LED_R_BIT_MASK (1ULL << LED_R)
#define LED_G_BIT_MASK (1ULL << LED_G)
#define LED_B_BIT_MASK (1ULL << LED_B)

// LED Controller Defines
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_13_BIT   // Set duty resolution to 13-bits

#define LEDC_DUTY       (2048)              // Set duty to 25% (2 ** 13) * 25% = 2048
#define LEDC_FREQUENCY  (5000)              // Frequency in Hz
#define LEDC_FADE_TIME  (3000)
#define LEDC_MAX_RES    (7373)             // Max duty 90% (2 ** 13) * 90% ~ 7373

enum STATE {
    INIT = 0,
    RGB,
    YTP
};

enum NEXT_COLOR {
    RED = 0,
    YELLOW,
    GREEN,
    TEAL,
    BLUE,
    PURPLE
};

enum PIN {
    NONE = 0,
    LEFT,
    RIGHT
};

struct btn_status {
    const uint8_t PIN;
    volatile bool pressed;
};



#endif