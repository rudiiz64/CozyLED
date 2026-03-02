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

// State Declaration
int CURR_STATE = INIT;
int last_button = NONE;

static void ledc_init(){
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_MODE,               // High or low speed
        .duty_resolution = LEDC_DUTY_RES,           // Max Range of cap, [0, 2 ** duty_resolution]
        .timer_num       = LEDC_TIMER,              // Controller Timer
        .freq_hz         = LEDC_FREQUENCY,          // Frequency (higher freq -> lower res, lower freq -> higher res)
        .clk_cfg         = LEDC_AUTO_CLK            // Clock config, pre-defined
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode      = LEDC_MODE,               // High or low speed
        .channel         = LEDC_CHANNEL,            // Channel per controller
        .timer_sel       = LEDC_TIMER,              // Controller timer
        .intr_type       = LEDC_INTR_DISABLE,       // Interrupt type
        .gpio_num        = LED_G,                   // GPIO assignment
        .duty            = 0,                       // Channel duty
        .hpoint          = 0                        // Horizontal point / phase control; [0, 2 ** duty_resolution - 1]
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

struct led_setting {
    gpio_num_t current_pin;
    gpio_num_t prev_pin;
    int color;
    int last_button;
    int mode;
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
    int timer = 0;
    leftBtnTime = lmillis();
    if (leftBtnTime - leftLastBtnTime > 250){
        LBtn.pressed = true;
        leftLastBtnTime = leftBtnTime;
        while (gpio_get_level(L_BTN)  == true){
            timer++;
            if (timer >= HOLD_TIMER){
                break;
            }
        }
        /* Static color check transition */
        if (CURR_STATE == YTP && timer < HOLD_TIMER){
           CURR_STATE = RGB;
        }
        /* Static color check transition */
        else if (CURR_STATE == RGB && timer < HOLD_TIMER) {
           CURR_STATE = YTP;
           last_button = LEFT;
        }
        /* Mode change check */
        else if ((CURR_STATE == YTP || CURR_STATE == RGB)  && timer >= HOLD_TIMER){
            CURR_STATE = WAVE;
        }
        
    }
}

void right_btn_isr(){
    rightBtnTime = lmillis();
    if (rightBtnTime - rightLastBtnTime > 250){
        RBtn.pressed = true;
        rightLastBtnTime = rightBtnTime;
        if (CURR_STATE == YTP){
            CURR_STATE = RGB;
        }
        else {
            CURR_STATE = YTP;
            last_button = RIGHT;

        }
    }
}

static void gpio_init(void){
    // Initialization of the GPIO pins
    myGPIO.pin_bit_mask = (LED_R_BIT_MASK | LED_G_BIT_MASK | LED_B_BIT_MASK);
    myGPIO.pull_down_en = GPIO_PULLDOWN_DISABLE;
    myGPIO.pull_up_en   = GPIO_PULLUP_DISABLE;
    myGPIO.intr_type    = GPIO_INTR_DISABLE;
    myGPIO.mode         = GPIO_MODE_OUTPUT;
    gpio_config(&myGPIO);

    // Initialization of ISRs
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(L_BTN, left_btn_isr, NULL);
    gpio_set_intr_type(L_BTN, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(R_BTN, right_btn_isr, NULL);
    gpio_set_intr_type(R_BTN, GPIO_INTR_POSEDGE);

    // Initialization of the peripherals
    gpio_set_direction(LED_PWR, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_R, false);
    gpio_set_level(LED_G, true);
    gpio_set_level(LED_B, true);
    gpio_set_direction(L_BTN, GPIO_MODE_INPUT);    
    gpio_input_enable(L_BTN);
    gpio_set_direction(R_BTN, GPIO_MODE_INPUT);
    gpio_input_enable(R_BTN);
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
    for (int i = 0; i < 9; i++){
        if (i == 0 || i == 3 || i == 6){
            ssd1306_clearScreen();
            ssd1306_printFixed(0, 16, "Initializing system, please wait.", STYLE_NORMAL);
        }
        else if (i == 1 || i == 4 || i == 7){
            ssd1306_printFixed(0, 16, "Initializing system, please wait..", STYLE_NORMAL);
        }
        else if (i == 2 || i == 5 || i == 8){
            ssd1306_printFixed(0, 16, "Initializing system, please wait...", STYLE_NORMAL);
        }
        vTaskDelay(50);
    }
    /* Clear screen of loading drawing */
    ssd1306_clearScreen();

    /* Draw Text for mode and title */
    ssd1306_printFixed(42, 2, "RGB LED", STYLE_BOLD);
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

    // LED FSM
    while(1){
        //printf("Current state: %d, Button: L %d / R %d\n", CURR_STATE, LBtn.pressed, RBtn.pressed);
        switch (CURR_STATE){
        case INIT:
            gpio_init();
            //ledc_init();
            i2c_init();
            disp_init();
            led_color.current_pin = LED_R;
            led_color.color = RED;
            gpio_set_level(LED_PWR, true);
            CURR_STATE = BREATHE;
            led_color.mode = LCD_STATIC;

            vTaskDelay(100);
            break;
        
        case RGB:
            ssd1306_printFixed((DISPL_X >> 2), (DISPL_Y >> 1), "Mode: WAVE   ", STYLE_NORMAL);

            /* Left Button */
            // Yellow -> Red
            // if (LBtn.pressed && led_color.color == YELLOW){
            //     if (last_button == LEFT){
            //         gpio_set_level(led_color.prev_pin, true);
            //     }
            //     else {
            //         gpio_set_level(led_color.current_pin, true);
            //     }
            //     led_color.prev_pin = led_color.current_pin;
            //     led_color.current_pin = LED_R;
            //     led_color.color = RED;
            //     LBtn.pressed = false;
            // }
            // // Teal -> Green
            // else if (LBtn.pressed && led_color.color == TEAL){
            //     if (last_button == LEFT){
            //         gpio_set_level(led_color.prev_pin, true);
            //     }
            //     else {
            //         gpio_set_level(led_color.current_pin, true);
            //     }
            //     led_color.prev_pin = led_color.current_pin;
            //     led_color.current_pin = LED_G;
            //     led_color.color = GREEN;
            //     LBtn.pressed = false;
            // }
            // // Purple -> Blue
            // else if (LBtn.pressed && led_color.color == PURPLE){
            //     if (last_button == LEFT){
            //         gpio_set_level(led_color.prev_pin, true);
            //     }
            //     else {
            //         gpio_set_level(led_color.current_pin, true);
            //     }
            //     led_color.prev_pin = led_color.current_pin;
            //     led_color.current_pin = LED_B;
            //     led_color.color = BLUE;
            //     LBtn.pressed = false;
            // }

            // /* Right Button */
            // // Yellow -> Green
            // else if (RBtn.pressed && led_color.color == YELLOW){
            //     if (last_button == RIGHT){
            //         gpio_set_level(led_color.prev_pin, true);
            //     }
            //     else {
            //         gpio_set_level(led_color.current_pin, true);
            //     }
            //     led_color.prev_pin = led_color.current_pin;
            //     led_color.current_pin = LED_G;
            //     led_color.color = GREEN;
            //     RBtn.pressed = false;
            // }
            // // Teal -> Blue
            // else if (RBtn.pressed && led_color.color == TEAL){
            //     if (last_button == RIGHT){
            //         gpio_set_level(led_color.prev_pin, true);
            //     }
            //     else {
            //         gpio_set_level(led_color.current_pin, true);
            //     }
            //     led_color.prev_pin = led_color.current_pin;
            //     led_color.current_pin = LED_B;
            //     led_color.color = BLUE;
            //     RBtn.pressed = false;
            // }
            // // Purple -> Red
            // else if (RBtn.pressed && led_color.color == PURPLE){
            //     if (last_button == RIGHT){
            //         gpio_set_level(led_color.prev_pin, true);
            //     }
            //     else {
            //         gpio_set_level(led_color.current_pin, true);
            //     }
            //     led_color.prev_pin = led_color.current_pin;
            //     led_color.current_pin = LED_R;
            //     led_color.color = RED;
            //     RBtn.pressed = false;
            // }
            break;

/****************************************************************************************************************************/

        case YTP:            
            /* Left Button */
            //Red -> Purple
            // if (LBtn.pressed && led_color.color == RED){
                
            //     led_color.prev_pin = led_color.current_pin;
            //     led_color.current_pin = LED_B;
            //     led_color.color = PURPLE;
            //     LBtn.pressed = false;
            //     gpio_set_level(led_color.current_pin, false);
            // }

            // // Green -> Yellow
            // else if (LBtn.pressed && led_color.color == GREEN){
                
            //     led_color.prev_pin = led_color.current_pin;
            //     led_color.current_pin = LED_R;
            //     led_color.color = YELLOW;
            //     LBtn.pressed = false;
            //     gpio_set_level(led_color.current_pin, false);
            // }

            // // Blue -> Teal
            // else if (LBtn.pressed && led_color.color == BLUE){
                
            //     led_color.prev_pin = led_color.current_pin;
            //     led_color.current_pin = LED_G;
            //     led_color.color = TEAL;
            //     LBtn.pressed = false;
            //     gpio_set_level(led_color.current_pin, false);
            // }

            // /* Right Button */
            // // Red -> Yellow
            // else if (RBtn.pressed && led_color.color == RED){
                
            //     led_color.prev_pin = led_color.current_pin;
            //     led_color.current_pin = LED_G;
            //     led_color.color = YELLOW;
            //     RBtn.pressed = false;
            //     gpio_set_level(led_color.current_pin, false);
            // }
            // // Green -> Teal
            // else if (RBtn.pressed && led_color.color == GREEN){
            //     led_color.prev_pin = led_color.current_pin;
            //     led_color.current_pin = LED_B;
            //     led_color.color = TEAL;
            //     RBtn.pressed = false;
            //     gpio_set_level(led_color.current_pin, false);
            // }
            // // Blue -> Purple
            // else if (RBtn.pressed && led_color.color == BLUE){
            //     led_color.prev_pin = led_color.current_pin;
            //     led_color.current_pin = LED_R;
            //     led_color.color = PURPLE;
            //     RBtn.pressed = false;
            //     gpio_set_level(led_color.current_pin, false);
            // }
            break;

        case BREATHE:
        /* 
            while (button press == true AND button signal is HIGH){
                btnTimer++;
            }
            if (btnTimer >= holdTrigger (2 sec) && LBtn.pressed == TRUE){
                mode changes to previous
                state changes to previous state
                btnTimer == 0;
            }
            else if (btnTimer >= holdTrigger (2 sec) && RBtn.pressed == TRUE){
                mode changes to next
                state changes to next state
                btnTimer == 0;
            }
            else if (btnTimer < holdTrigger){
                btnTimer == 0;
            }
            break;
        */
            int timer = 0;
            while (gpio_get_level(L_BTN) == true || gpio_get_level(R_BTN == true)){
                timer++;
                printf("Timer: %d\n", timer);
            }

            
            

            //CURR_STATE = RGB;
            vTaskDelay(10);
            break;


        case WAVE:
            break;

        default:
            printf("Oopsies, state: %d\n", CURR_STATE);
            vTaskDelay(100);
            break;
        }
        vTaskDelay(10);
        
        /*// Breathing Function
        int LED_Duty = LEDC_MAX_RES;
        int dutyChange = (LEDC_MAX_RES / (2 * LEDC_DUTY_RES));
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