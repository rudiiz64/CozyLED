#ifndef COZYLED_H
#define COZYLED_H

#include <inttypes.h>

// Variable Defines
#define HOLD_TIMER      250
#define INIT_CHECK      9

// I2C Defines
#define I2C_ADDR        0x3C
#define I2C_SLAVE_NUM   I2C_NUM_0

// LCD Defines
#define DISPL_X         0x80
#define DISPL_Y         0x40
#define LEFT_TRI_X      0x05

#define RIGHT_TRI_X     0x7B        // 112 x
#define TRI_Y           0x20        // 32 y
#define TRI_0           0xFF
#define TRI_1           0x7E
#define TRI_2           0x3C
#define TRI_3           0x18

// GPIO Defines
#define LED_R           GPIO_NUM_26
#define LED_G           GPIO_NUM_25
#define LED_B           GPIO_NUM_33
#define LED_PWR         GPIO_NUM_27
#define L_BTN           GPIO_NUM_35
#define R_BTN           GPIO_NUM_34
#define SDA             GPIO_NUM_14
#define SCL             GPIO_NUM_12

// LED Bit Mask
#define LED_R_BIT_MASK (1ULL << LED_R)
#define LED_G_BIT_MASK (1ULL << LED_G)
#define LED_B_BIT_MASK (1ULL << LED_B)

// LED Controller Defines
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_13_BIT   // Set duty resolution to 13-bits

#define LEDC_DUTY        (2048)              // Set duty to 25% (2 ** 13) * 25% = 2048
#define LEDC_FREQUENCY   (5000)              // Frequency in Hz
#define LEDC_FREQ_CHANGE (500)               // Incremental change to frequency value
#define LEDC_MAX_RES     (8192)             // Max duty 90% (2 ** 13) * 90% ~ 7373
#define LEDC_ON          (819)              // output inverted, using 10% as 90%

enum STATE {
    INIT = 0,
    RGB,
    YTP,
    BREATHE,
    WAVE
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
    volatile bool locked;
};



#endif

/*// Breathing Function
        
        for (;;){
            // LED 100% Duty
            //printf("LEDC Set duty = %d\n\n", LED_Duty);
            if (LED_Duty - dutyChange < 0){                     // If next duty change is zero or negative, enter
                LED_Duty -= LED_Duty;                           // Instead of entering negative, we hit zero
                dutyChange *= -1;
            }
            else if (LED_Duty - dutyChange > LEDC_MAX_RES){     // If we are going to exceed the max resolution value, enter
                LED_Duty += (LEDC_MAX_RES - LED_Duty);          // We add the remainder of LED_Duty to hit max res 
                dutyChange *= -1;
            }
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LED_Duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            printf("Duty: %ld\n",ledc_get_duty(LEDC_MODE, LEDC_CHANNEL));

            vTaskDelay(100 / portTICK_PERIOD_MS);
            LED_Duty -= dutyChange;
        }*/