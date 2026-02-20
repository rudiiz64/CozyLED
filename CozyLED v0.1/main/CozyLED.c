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

// Private Libraries
#include "CozyLED.h"

// Globals
volatile unsigned long leftBtnTime = 0;
volatile unsigned long leftLastBtnTime = 0;
volatile unsigned long rightBtnTime = 0;
volatile unsigned long rightLastBtnTime = 0;

// State Declaration
int CURR_STATE = INIT;

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
    gpio_num_t pin;
};

struct btn_status LBtn;
struct btn_status RBtn;
gpio_config_t myGPIO;

// Millis timer
static uint32_t millis(){
    return esp_timer_get_time() / 1000;
}

// ISRs
void left_btn_isr(){
    leftBtnTime= millis();
    if (leftBtnTime - leftLastBtnTime > 250){
        LBtn.numberPress++;
        LBtn.pressed = true;
        leftLastBtnTime = leftBtnTime;
        CURR_STATE = L_COLOR;
    }
}

void right_btn_isr(){
    rightBtnTime = millis();
    if (rightBtnTime - rightLastBtnTime > 250){
        RBtn.numberPress++;
        RBtn.pressed = true;
        rightLastBtnTime = rightBtnTime;
        CURR_STATE = R_COLOR;
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
    gpio_set_level(LED_R, true);
    gpio_set_level(LED_G, true);
    gpio_set_level(LED_B, true);
    gpio_set_direction(L_BTN, GPIO_MODE_INPUT);    
    gpio_input_enable(L_BTN);
    gpio_set_direction(R_BTN, GPIO_MODE_INPUT);
    gpio_input_enable(R_BTN);
}

void app_main(void){
    struct led_setting led_color;
    // LED FSM
    while(1){
        switch (CURR_STATE){
        case INIT:
            printf("Initializing, state: %d\n", CURR_STATE);
            //led_init();
            gpio_init();
            led_color.pin = LED_R;
            gpio_set_level(LED_PWR, true);
            CURR_STATE = DEFAULT;
            vTaskDelay(10);
            break;
        
        case DEFAULT:
            gpio_set_level(led_color.pin, false);
            vTaskDelay(10);
            break;

        case L_COLOR:
            printf("Left button pressed, state: %d\n", CURR_STATE);
            // If L button has been pressed, change color to previous color
            gpio_set_level(led_color.pin, true);
            if (led_color.pin == LED_R){
                led_color.pin = LED_B;
            }
            else if (led_color.pin == LED_G){
                led_color.pin = LED_R;
            }
            else if (led_color.pin == LED_B){
                led_color.pin = LED_G;
            }
            // After actions, return to default state
            CURR_STATE = DEFAULT;
            vTaskDelay(10);
            break;

        case R_COLOR:
            printf("Right button pressed, state: %d\n", CURR_STATE);
            // If right button is pressed, change color to next color
            gpio_set_level(led_color.pin, true);
            if (led_color.pin == LED_R){
                led_color.pin = LED_G;
            }
            else if(led_color.pin == LED_G){
                led_color.pin = LED_B;
            }
            else if (led_color.pin == LED_B){
                led_color.pin = LED_R;
            }
            // After actions, return to default state
            CURR_STATE = DEFAULT;
            vTaskDelay(10);
            break;

        default:
            printf("Oopsies, state: %d\n", CURR_STATE);
            vTaskDelay(100);
            break;
        }
        
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