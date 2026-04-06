/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>

// Default ESP Libraries
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_intr_types.h"
#include "esp_timer.h"

// Driver Libraries
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

// Private Libraries
#include "CozyLED.h"
#include "ssd1306.h"

// Globals
volatile unsigned long leftBtnTime = 0;
volatile unsigned long leftLastBtnTime = 0;
volatile unsigned long rightBtnTime = 0;
volatile unsigned long rightLastBtnTime = 0;
int last_button = NONE;
bool colorShift = false;

// State Declaration
int CURR_STATE = INIT;

static void ledc_init(){
    /* RED LED SETUP */
    ledc_timer_config_t ledc_timer0 = {
        .speed_mode      = LEDC_MODE,               // High or low speed
        .duty_resolution = LEDC_DUTY_RES,           // Max Range of cap, [0, 2 ** duty_resolution]
        .timer_num       = LEDC_TIMER_0,            // Controller Timer
        .freq_hz         = LEDC_FREQUENCY,          // Frequency (higher freq -> lower res, lower freq -> higher res)
        .clk_cfg         = LEDC_AUTO_CLK            // Clock config, pre-defined
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer0));

    ledc_channel_config_t ledc_channel0 = {
        .speed_mode      = LEDC_MODE,               // High or low speed
        .channel         = LEDC_CHANNEL_0,          // Channel per controller
        .timer_sel       = LEDC_TIMER_0,            // Controller timer
        .intr_type       = LEDC_INTR_DISABLE,       // Interrupt type
        .gpio_num        = LED_R,                   // GPIO assignment
        .duty            = LEDC_MAX_RES,            // Channel duty
        .hpoint          = 0                        // Horizontal point / phase control; [0, 2 ** duty_resolution - 1]
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel0));

    /* GREEN LED SETUP */
    ledc_timer_config_t ledc_timer1 = {
        .speed_mode      = LEDC_MODE,               // High or low speed
        .duty_resolution = LEDC_DUTY_RES,           // Max Range of cap, [0, 2 ** duty_resolution]
        .timer_num       = LEDC_TIMER_1,            // Controller Timer
        .freq_hz         = LEDC_FREQUENCY,          // Frequency (higher freq -> lower res, lower freq -> higher res)
        .clk_cfg         = LEDC_AUTO_CLK            // Clock config, pre-defined
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer1));

    ledc_channel_config_t ledc_channel1 = {
        .speed_mode      = LEDC_MODE,               // High or low speed
        .channel         = LEDC_CHANNEL_1,          // Channel per controller
        .timer_sel       = LEDC_TIMER_1,            // Controller timer
        .intr_type       = LEDC_INTR_DISABLE,       // Interrupt type
        .gpio_num        = LED_G,                   // GPIO assignment
        .duty            = LEDC_MAX_RES,            // Channel duty
        .hpoint          = 0                        // Horizontal point / phase control; [0, 2 ** duty_resolution - 1]
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

    /* BLUE LED SETUP */
    ledc_timer_config_t ledc_timer2 = {
        .speed_mode      = LEDC_MODE,               // High or low speed
        .duty_resolution = LEDC_DUTY_RES,           // Max Range of cap, [0, 2 ** duty_resolution]
        .timer_num       = LEDC_TIMER_2,            // Controller Timer
        .freq_hz         = LEDC_FREQUENCY,          // Frequency (higher freq -> lower res, lower freq -> higher res)
        .clk_cfg         = LEDC_AUTO_CLK            // Clock config, pre-defined
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer2));

    ledc_channel_config_t ledc_channel2 = {
        .speed_mode      = LEDC_MODE,               // High or low speed
        .channel         = LEDC_CHANNEL_2,          // Channel per controller
        .timer_sel       = LEDC_TIMER_2,            // Controller timer
        .intr_type       = LEDC_INTR_DISABLE,       // Interrupt type
        .gpio_num        = LED_B,                   // GPIO assignment
        .duty            = LEDC_MAX_RES,            // Channel duty
        .hpoint          = 0                        // Horizontal point / phase control; [0, 2 ** duty_resolution - 1]
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));
}

struct led_setting {
    ledc_channel_t current_ch;
    ledc_channel_t prev_ch;
    ledc_timer_t current_tim;
    ledc_timer_t prev_tim;
    uint8_t color;
    uint8_t last_button;
    uint32_t LED_Duty;
    uint32_t dutyChange;
    uint32_t freq;
};

struct led_setting led_color;
struct btn_status LBtn;
struct btn_status RBtn;
gpio_config_t myGPIO;

// Millis timer
static uint32_t lmillis(){
    return esp_timer_get_time() / 1000;
}

// ISRs
void left_btn_isr(){
    leftBtnTime = lmillis();
    if (leftBtnTime - leftLastBtnTime > 250){
        LBtn.pressed = true;
        leftLastBtnTime = leftBtnTime;

        /* STATIC substate check */
        if (CURR_STATE == YTP && LBtn.locked == false){
           CURR_STATE = RGB;
        }
        else if (CURR_STATE == RGB && LBtn.locked == false) {
           CURR_STATE = YTP;
           last_button = LEFT;
        }
        /* BREATHE substate check */
    }
}

void right_btn_isr(){
    rightBtnTime = lmillis();
    if (rightBtnTime - rightLastBtnTime > 250){
        RBtn.pressed = true;
        rightLastBtnTime = rightBtnTime;

        /* STATIC substate check */
        if (CURR_STATE == YTP && (RBtn.locked == false)){
            CURR_STATE = RGB;
        }
        else if (CURR_STATE == RGB && (RBtn.locked == false)) {
            CURR_STATE = YTP;
            last_button = RIGHT;
        }
        /* BREATHE substate check */
    }
}

static void gpio_init(void){
    // Initialization of ISRs
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(L_BTN, left_btn_isr, NULL);
    gpio_set_intr_type(L_BTN, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(R_BTN, right_btn_isr, NULL);
    gpio_set_intr_type(R_BTN, GPIO_INTR_NEGEDGE);

    // Initialization of the peripherals
    gpio_set_direction(LED_PWR, GPIO_MODE_OUTPUT);
    gpio_set_direction(L_BTN, GPIO_MODE_INPUT);    
    gpio_input_enable(L_BTN);
    gpio_set_direction(R_BTN, GPIO_MODE_INPUT);
    gpio_input_enable(R_BTN);
    
    vTaskDelay(10);
}

static void i2c_init(void){
    i2c_config_t i2c_slave = {
        .mode           = I2C_MODE_SLAVE,
        .sda_io_num     = SDA,
        .scl_io_num     = SCL,
        .sda_pullup_en  = true,
        .scl_pullup_en  = true,
    };
    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_slave.slave.slave_addr = I2C_ADDR;
    i2c_slave.slave.addr_10bit_en = 0;
    i2c_param_config(i2c_slave_port, &i2c_slave);
    i2c_driver_install(i2c_slave_port, i2c_slave.mode,
                        1024, 1024, 0);
}

static void disp_init(void){
    ssd1306_128x64_i2c_initEx(SCL, SDA, I2C_ADDR);
    ssd1306_128x64_init();
    ssd1306_fillScreen(0x00);
    ssd1306_setFixedFont(ssd1306xled_font6x8);
    for (int i = 0; i < INIT_CHECK; i++){
        if (i == 0 || i == 3 || i == 6){
            ssd1306_clearScreen();
            ssd1306_printFixed(0, 16, "Initializing system, please wait.", STYLE_NORMAL);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, LEDC_ON);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
        }
        else if (i == 1 || i == 4 || i == 7){
            ssd1306_printFixed(0, 16, "Initializing system, please wait..", STYLE_NORMAL);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, LEDC_ON);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
        }
        else if (i == 2 || i == 5 || i == 8){
            ssd1306_printFixed(0, 16, "Initializing system, please wait...", STYLE_NORMAL);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, LEDC_ON);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);
        }
        vTaskDelay(100);
    }
    /* Clear screen of loading drawing */
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, LEDC_MAX_RES);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, LEDC_MAX_RES);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, LEDC_MAX_RES);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ssd1306_clearScreen();

    /* Draw Text for mode and title */
    ssd1306_printFixed((DISPL_X >> 2), (DISPL_Y >> 1), "Mode: STATIC", STYLE_NORMAL);

    /* Draw left triangle */
    ssd1306_putPixels(LEFT_TRI_X - 3, TRI_Y, TRI_3);
    ssd1306_putPixels(LEFT_TRI_X - 2, TRI_Y, TRI_2);
    ssd1306_putPixels(LEFT_TRI_X - 1, TRI_Y, TRI_1);
    ssd1306_putPixels(LEFT_TRI_X,     TRI_Y, TRI_0);

    /* Draw right triangle */
    ssd1306_putPixels(RIGHT_TRI_X,     TRI_Y, TRI_0);
    ssd1306_putPixels(RIGHT_TRI_X + 1, TRI_Y, TRI_1);
    ssd1306_putPixels(RIGHT_TRI_X + 2, TRI_Y, TRI_2);
    ssd1306_putPixels(RIGHT_TRI_X + 3, TRI_Y, TRI_3);
}

void app_main(void){
    int holdTimer = false;
    led_color.LED_Duty = LEDC_ON;
    led_color.dutyChange = (LEDC_MAX_RES / (2 * LEDC_DUTY_RES));
    led_color.freq = LEDC_FREQUENCY;
    
    // LED FSM
    while(true){
        /* If user is holding left button, disable right button */
        while(gpio_get_level(L_BTN) && (CURR_STATE != INIT)){
            gpio_set_direction(R_BTN, GPIO_MODE_DISABLE);
            holdTimer++;
            if (holdTimer >= HOLD_TIMER){                                           // If holdTimer exceeds 2 sec, change state and break
                LBtn.locked = true;                                      
                holdTimer = 0;
                if (CURR_STATE == RGB || CURR_STATE == YTP){                         // If we are in RGB/YTP, then we just loop to last state WAVE (4)
                    CURR_STATE = WAVE;
                }
                else {                                                               // If we are not in RGB/YTP, subtract 1
                    CURR_STATE = CURR_STATE - 1;
                }
                break;
            }
            vTaskDelay(1);
        }

        /* If user is holding right button, disable left button */
        while(gpio_get_level(R_BTN) && (CURR_STATE != INIT)){
            gpio_set_direction(L_BTN, GPIO_MODE_DISABLE);
            holdTimer++;
            if (holdTimer >= HOLD_TIMER){                       // If holdTimer exceeds 2 sec, change state and break
                RBtn.locked = true;
                holdTimer = 0;
                if (CURR_STATE == WAVE){                         // If we are in WAVE (4), then we just loop to first state RGB (1)
                    CURR_STATE = RGB;
                }
                else if (CURR_STATE == RGB || CURR_STATE == YTP){                                          // If we are not in WAVE, just add 1 from enum
                    CURR_STATE = BREATHE;
                }
                else {
                    CURR_STATE = CURR_STATE + 1;
                }
                break;
            }
            vTaskDelay(1);
        }
        
        /* Begin FSM */
        switch (CURR_STATE){
        case INIT:
            gpio_init();
            gpio_set_level(LED_PWR, true);
            ledc_init();
            i2c_init();
            disp_init();

            /* Initial values for the LED struct */
            led_color.current_ch = LEDC_CHANNEL_0;
            led_color.current_tim = LEDC_TIMER_0;
            led_color.prev_ch = LEDC_CHANNEL_2;
            led_color.prev_tim = LEDC_TIMER_2;
            led_color.color = RED;

            /* Unlock buttons */
            LBtn.locked = false;
            RBtn.locked = false;
            
            CURR_STATE = YTP;

            /* Enable RED */
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, LEDC_ON);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);

            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        
            /* Static state */
        case RGB:
            /* Left Button */
            ssd1306_printFixed((DISPL_X >> 2), (DISPL_Y >> 1), "Mode: STATIC", STYLE_NORMAL);
            // Yellow -> Red
            if (LBtn.pressed && led_color.color == YELLOW){
                if (last_button == LEFT){
                    ssd1306_printFixed(42, 2, "Color: RED", STYLE_BOLD);
                    ledc_set_duty(LEDC_MODE, led_color.prev_ch, LEDC_MAX_RES);
                    ledc_update_duty(LEDC_MODE, led_color.prev_ch);
                }
                else {
                    ledc_set_duty(LEDC_MODE, led_color.current_ch, LEDC_MAX_RES);
                    ledc_update_duty(LEDC_MODE, led_color.current_ch);
                }
                led_color.prev_ch = led_color.current_ch;
                led_color.current_ch = LEDC_CHANNEL_0;
                led_color.color = RED;
                printf("Yellow -> Red\n");
            }
            // Teal -> Green
            else if (LBtn.pressed && led_color.color == TEAL){
                if (last_button == LEFT){
                    ledc_set_duty(LEDC_MODE, led_color.prev_ch, LEDC_MAX_RES);
                    ledc_update_duty(LEDC_MODE, led_color.prev_ch);
                }
                else {
                    ledc_set_duty(LEDC_MODE, led_color.current_ch, LEDC_MAX_RES);
                    ledc_update_duty(LEDC_MODE, led_color.current_ch);
                }
                led_color.prev_ch = led_color.current_ch;
                led_color.current_ch = LEDC_CHANNEL_1;
                led_color.color = GREEN;
                printf("Teal -> Green\n");
            }
            // Purple -> Blue
            else if (LBtn.pressed && led_color.color == PURPLE){
                if (last_button == LEFT){
                    ledc_set_duty(LEDC_MODE, led_color.prev_ch, LEDC_MAX_RES);
                    ledc_update_duty(LEDC_MODE, led_color.prev_ch);
                }
                else {
                    ledc_set_duty(LEDC_MODE, led_color.current_ch, LEDC_MAX_RES);
                    ledc_update_duty(LEDC_MODE, led_color.current_ch);
                }
                led_color.prev_ch = led_color.current_ch;
                led_color.current_ch = LEDC_CHANNEL_2;
                led_color.color = BLUE;
                printf("Purple -> Blue\n");
            }

            /* Right Button */
            // Yellow -> Green
            else if (RBtn.pressed && led_color.color == YELLOW){
                if (last_button == RIGHT){
                    ledc_set_duty(LEDC_MODE, led_color.prev_ch, LEDC_MAX_RES);
                    ledc_update_duty(LEDC_MODE, led_color.prev_ch);
                }
                else {
                    ledc_set_duty(LEDC_MODE, led_color.current_ch, LEDC_MAX_RES);
                    ledc_update_duty(LEDC_MODE, led_color.current_ch);
                }
                led_color.prev_ch = led_color.current_ch;
                led_color.current_ch = LEDC_CHANNEL_1;
                led_color.color = GREEN;
                printf("Yellow -> Green\n");
            }
            // Teal -> Blue
            else if (RBtn.pressed && led_color.color == TEAL){
                if (last_button == RIGHT){
                    ledc_set_duty(LEDC_MODE, led_color.prev_ch, LEDC_MAX_RES);
                    ledc_update_duty(LEDC_MODE, led_color.prev_ch);
                }
                else {
                    ledc_set_duty(LEDC_MODE, led_color.current_ch, LEDC_MAX_RES);
                    ledc_update_duty(LEDC_MODE, led_color.current_ch);
                }
                led_color.prev_ch = led_color.current_ch;
                led_color.current_ch = LEDC_CHANNEL_2;
                led_color.color = BLUE;
                printf("Teal -> Blue\n");
            }
            // Purple -> Red
            else if (RBtn.pressed && led_color.color == PURPLE){
                if (last_button == RIGHT){
                    ledc_set_duty(LEDC_MODE, led_color.prev_ch, LEDC_MAX_RES);
                    ledc_update_duty(LEDC_MODE, led_color.prev_ch);
                }
                else {
                    ledc_set_duty(LEDC_MODE, led_color.current_ch, LEDC_MAX_RES);
                    ledc_update_duty(LEDC_MODE, led_color.current_ch);
                }
                led_color.prev_ch = led_color.current_ch;
                led_color.current_ch = LEDC_CHANNEL_0;
                led_color.color = RED;
                printf("Purple -> Red\n");
            }
            break;

        case YTP:            
            ssd1306_printFixed((DISPL_X >> 2), (DISPL_Y >> 1), "Mode: STATIC   ", STYLE_NORMAL);
            /* Left Button */
            // Red -> Purple
            if (LBtn.pressed && led_color.color == RED){
                
                led_color.prev_ch = led_color.current_ch;
                led_color.current_ch = LEDC_CHANNEL_2;
                led_color.color = PURPLE;
                ledc_set_duty(LEDC_MODE, led_color.current_ch, LEDC_ON);
                ledc_update_duty(LEDC_MODE, led_color.current_ch);
                printf("Red -> Purple\n");
            }

            // Green -> Yellow
            else if (LBtn.pressed && led_color.color == GREEN){
                
                led_color.prev_ch = led_color.current_ch;
                led_color.current_ch = LEDC_CHANNEL_0;
                led_color.color = YELLOW;
                ledc_set_duty(LEDC_MODE, led_color.current_ch, LEDC_ON);
                ledc_update_duty(LEDC_MODE, led_color.current_ch);
                printf("Green -> Yellow\n");
            }

            // Blue -> Teal
            else if (LBtn.pressed && led_color.color == BLUE){
                
                led_color.prev_ch = led_color.current_ch;
                led_color.current_ch = LEDC_CHANNEL_1;
                led_color.color = TEAL;
                ledc_set_duty(LEDC_MODE, led_color.current_ch, LEDC_ON);
                ledc_update_duty(LEDC_MODE, led_color.current_ch);
                printf("Blue -> Teal\n");
            }

            /* Right Button */
            // Red -> Yellow
            else if (RBtn.pressed && led_color.color == RED){
                led_color.prev_ch = led_color.current_ch;
                led_color.current_ch = LEDC_CHANNEL_1;
                led_color.color = YELLOW;
                ledc_set_duty(LEDC_MODE, led_color.current_ch, LEDC_ON);
                ledc_update_duty(LEDC_MODE, led_color.current_ch);
                printf("Red -> Yellow\n");
            }
            // Green -> Teal
            else if (RBtn.pressed && led_color.color == GREEN){
                led_color.prev_ch = led_color.current_ch;
                led_color.current_ch = LEDC_CHANNEL_2;
                led_color.color = TEAL;
                ledc_set_duty(LEDC_MODE, led_color.current_ch, LEDC_ON);
                ledc_update_duty(LEDC_MODE, led_color.current_ch);
                printf("Green -> Teal\n");
            }
            // Blue -> Purple
            else if (RBtn.pressed && led_color.color == BLUE){
                led_color.prev_ch = led_color.current_ch;
                led_color.current_ch = LEDC_CHANNEL_0;
                led_color.color = PURPLE;
                ledc_set_duty(LEDC_MODE, led_color.current_ch, LEDC_ON);
                ledc_update_duty(LEDC_MODE, led_color.current_ch);
                printf("Blue -> Purple\n");
            }
            break;
        /* END STATIC */

        /* Breathe */
        case BREATHE:
            ssd1306_printFixed(DISPL_X >> 2, DISPL_Y, "Speed: ", STYLE_ITALIC);
            ssd1306_printFixed((DISPL_X >> 2), (DISPL_Y >> 1), "Mode: BREATHE", STYLE_NORMAL);
            printf("In breathe\n");

            break;
        /* END BREATHE */

        /* Wave */
        case WAVE:
            ssd1306_printFixed((DISPL_X >> 2), (DISPL_Y >> 1), "Mode: WAVE   ", STYLE_NORMAL);

            /* Section for initial breath -> color carries over from previous mode 
               Starting color will start at max, next channel will increment to max, then previous channel will decrement to 0
               Button press will change the frequency of transition
               Button hold will freeze 
            */
            if (LBtn.pressed && led_color.freq > 0){
                /* Decrement frequency function including failsafe (if 0, discard action) */
                led_color.freq -= LEDC_FREQ_CHANGE;
                ledc_set_freq(LEDC_MODE, led_color.prev_tim, led_color.freq);     // Frequency shift needs testing to find a good shift up and down that is noticeable
                
                // If the value of the frequency is at 0, then ignore the button press
                LBtn.pressed = false;
            }
            
            else if (RBtn.pressed && led_color.freq < LEDC_MAX_FREQUENCY){
                /* Increment frequency function including failsafe (if overflow, discard action) */
                if (led_color.LED_Duty < led_color.freq){
                    led_color.freq += LEDC_FREQ_CHANGE;
                    ledc_set_freq(LEDC_MODE, led_color.prev_tim, led_color.freq);
                }
                RBtn.pressed = false;

           }

           /* Take stored color and channel to begin
           One channel must always be on, at max duty cycle; if both, then begin turning off previous channel */
           if ((ledc_get_duty(LEDC_MODE, led_color.current_ch) == LEDC_ON) && (ledc_get_duty(LEDC_MODE, led_color.prev_ch) == LEDC_MAX_RES)){     // Current channel is at max, i.e. red/green/blue
                printf("Max RGB\n");
                //led_color.LED_Duty -= led_color.dutyChange;
                led_color.prev_ch = led_color.current_ch;
                led_color.prev_tim = led_color.current_tim;
                
                // If current channel is blue, loop to red
                if (led_color.current_ch == LEDC_CHANNEL_2){
                    led_color.current_ch = LEDC_CHANNEL_0;
                    led_color.current_tim = LEDC_TIMER_0;
                }
                else {
                    led_color.current_ch += 1;
                    led_color.current_tim += 1;
                }
                
                /* Begin lighting next channel */
                led_color.LED_Duty = LEDC_MAX_RES;
                //ledc_set_duty(LEDC_MODE, led_color.current_ch, led_color.LED_Duty);
                //ledc_update_duty(LEDC_MODE, led_color.current_ch);
                //led_color.LED_Duty -= led_color.dutyChange;
                colorShift = true;
           }
           else if ((ledc_get_duty(LEDC_MODE, led_color.current_ch) == LEDC_ON) && (ledc_get_duty(LEDC_MODE, led_color.prev_ch) == LEDC_ON)){ // Both colors are on at max, i.e. yellow/teal/purple
                printf("Max YTP\n");
                //led_color.LED_Duty += led_color.dutyChange;

                /* Begin turning off previous channel */
                //ledc_set_duty(LEDC_MODE, led_color.prev_ch, led_color.LED_Duty);
                //ledc_update_duty(LEDC_MODE, led_color.prev_ch);
                colorShift = false;
           }

           /* Perform wave action */
           // RGB color -> increase duty of current channel
           if (colorShift){
                if (led_color.LED_Duty - led_color.dutyChange < LEDC_ON){
                    printf("Overflow: %ld\n", led_color.LED_Duty);
                    led_color.LED_Duty = LEDC_ON;
                    led_color.dutyChange *= -1;
                }
                else {
                    printf("Increment current: %ld\n", led_color.LED_Duty);
                    led_color.LED_Duty -= led_color.dutyChange;
                }
                ledc_set_duty(LEDC_MODE, led_color.current_ch, led_color.LED_Duty);
                ledc_update_duty(LEDC_MODE, led_color.current_ch);
                
           }
           // YTP color -> decrease duty of previous channel
           else {
                if (led_color.LED_Duty - led_color.dutyChange > LEDC_MAX_RES){
                    printf("underflow: %ld\n", led_color.LED_Duty);
                    led_color.LED_Duty += (LEDC_MAX_RES - led_color.LED_Duty);
                    led_color.dutyChange *= -1;
                }
                else {
                    printf("Decrement prev: %ld\n", led_color.LED_Duty);
                    led_color.LED_Duty -= led_color.dutyChange;
                }
                ledc_set_duty(LEDC_MODE, led_color.prev_ch, led_color.LED_Duty);
                ledc_update_duty(LEDC_MODE, led_color.prev_ch);
           }
            break;
        /* END WAVE */

        /* No no zone */
        default:
            printf("Oopsies, state: %d\n", CURR_STATE);
            vTaskDelay(100);
            break;
        }
        /* END FSM */

        /* After buttons are no longer held, re-enable and reset timer */
        holdTimer = 0;
        if (LBtn.pressed){
            LBtn.locked = false;
            LBtn.pressed = false;
            gpio_set_direction(R_BTN, GPIO_MODE_INPUT);
        }
        else if (RBtn.pressed){
            RBtn.locked = false;
            RBtn.pressed = false;
            gpio_set_direction(L_BTN, GPIO_MODE_INPUT);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);

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
    }
}