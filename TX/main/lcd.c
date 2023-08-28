#include "lcd.h"
//#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
//#include "esp_system.h"
#include <rom/ets_sys.h>

#include "GPIO_PINS.h"

static char lcd_buffer[LCD_COL_COUNT + 1];


static const char *TAG = "Debug:";


void lcd_init_gpio() {
    // Initialize GPIO pins as outputs
    //gpio_pad_select_gpio(LCD_RS_PIN);
    gpio_set_direction(LCD_RS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_RS_PIN, 0);
    //gpio_pad_select_gpio(LCD_RW_PIN);
    //gpio_set_direction(LCD_RW_PIN, GPIO_MODE_OUTPUT);
    
    //gpio_pad_select_gpio(LCD_EN_PIN);
    gpio_set_direction(LCD_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_EN_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(4.1));
    //gpio_pad_select_gpio(LCD_D0_PIN);

    gpio_config_t io_config;
    io_config.pin_bit_mask = (1ULL << LCD_D5_PIN);
    io_config.mode = GPIO_MODE_OUTPUT; // Change this to your required mode
    io_config.pull_up_en = GPIO_PULLUP_DISABLE; // Disable internal pull-up
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable internal pull-down
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);
    
    gpio_set_direction(LCD_D4_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_D4_PIN, 0);
    //gpio_set_direction(LCD_D5_PIN, GPIO_MODE_OUTPUT);
    //gpio_set_level(LCD_D5_PIN, 0);
    gpio_reset_pin(LCD_D6_PIN);
    gpio_set_direction(LCD_D6_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_D6_PIN, 0);
    gpio_reset_pin(LCD_D7_PIN);
    gpio_set_direction(LCD_D7_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_D7_PIN, 0);
    
}

void lcd_pulse_enable() {
    
    gpio_set_level(LCD_EN_PIN, 0);
    ets_delay_us(1);

    gpio_set_level(LCD_EN_PIN, 1);
    ets_delay_us(1); // Delay for a short time

    gpio_set_level(LCD_EN_PIN, 0);
    ets_delay_us(100); // Delay for a short time
}

void lcd_send_nibble(uint8_t nibble) {
    gpio_set_level(LCD_D4_PIN, 0);  // Set D4 to low
    gpio_set_level(LCD_D5_PIN, 0);  // Set D5 to low
    gpio_set_level(LCD_D6_PIN, 0);  // Set D6 to low
    gpio_set_level(LCD_D7_PIN, 0);  // Set D7 to low

    gpio_set_level(LCD_D4_PIN, (nibble >> 0) & 0x01);
    gpio_set_level(LCD_D5_PIN, (nibble >> 1) & 0x01);
    gpio_set_level(LCD_D6_PIN, (nibble >> 2) & 0x01);
    gpio_set_level(LCD_D7_PIN, (nibble >> 3) & 0x01);
    //ESP_LOGI(TAG, "D7%u: ", (nibble >> 3) & 0x01);
    //ESP_LOGI(TAG, "D6%u: ", (nibble >> 2) & 0x01);
    //ESP_LOGI(TAG, "D5%u: ", (nibble >> 1) & 0x01);
    //ESP_LOGI(TAG, "D4%u: ", (nibble >> 0) & 0x01);
    //vTaskDelay(pdMS_TO_TICKS(1));//TEST

    lcd_pulse_enable();
}

void lcd_send_command(uint8_t command) {
    gpio_set_level(LCD_RS_PIN, 0); // RS = 0 (command mode)
    //vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_nibble(command >> 4);
    lcd_send_nibble(command);
    
    vTaskDelay(pdMS_TO_TICKS(2)); // Delay after sending command
}

void lcd_send_data(uint8_t data) {
    gpio_set_level(LCD_RS_PIN, 1); // RS = 1 (data mode)
    //vTaskDelay(pdMS_TO_TICKS(1));//10
    lcd_send_nibble(data >> 4);
    lcd_send_nibble(data);
    
    vTaskDelay(pdMS_TO_TICKS(0.3)); // Delay after sending data
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
  static uint8_t offsets[] = { 0x00, 0x40, 0x14, 0x54 };

  lcd_send_command(LCD_SETDDRAMADDR | (col + offsets[row]));
}

void lcd_clear(void) {
  lcd_send_command(LCD_CLEARDISPLAY);
  vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_puts(char *string) {
    for (char *it = string; *it; it++) {
        lcd_send_data(*it);
    }
}

void lcd_printf(char *format, ...) {
  va_list args;

  va_start(args, format);
  vsnprintf(lcd_buffer, LCD_COL_COUNT + 1, format, args);
  va_end(args);

  lcd_puts(lcd_buffer);
}

void lcd_init() {
    
    lcd_init_gpio();

    vTaskDelay(pdMS_TO_TICKS(50)); // Wait after power on

    gpio_set_level(LCD_EN_PIN, 0);
    gpio_set_level(LCD_RS_PIN, 0);
    //gpio_set_level(LCD_D4_PIN, 1);
    //gpio_set_level(LCD_D5_PIN, 1);
    //gpio_set_level(LCD_D6_PIN, 1);
    gpio_set_level(LCD_D7_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(4.1));
    //ESP_LOGI(TAG, "ALL ZERO");
    //vTaskDelay(pdMS_TO_TICKS(1000000));
    //PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[13], PIN_FUNC_GPIO);
    lcd_send_nibble(0x03);
    vTaskDelay(pdMS_TO_TICKS(4.1));
    ESP_LOGI(TAG, "nibble1");
    //vTaskDelay(pdMS_TO_TICKS(1000000));

    lcd_send_nibble(0x03);
    vTaskDelay(pdMS_TO_TICKS(4.1));

    lcd_send_nibble(0x03);
    vTaskDelay(pdMS_TO_TICKS(4.1));

    lcd_send_nibble(0x02); // 4-bit mode
    vTaskDelay(pdMS_TO_TICKS(4.1));

    ESP_LOGI(TAG, "COMMANDS:");
    lcd_send_command(40);
    vTaskDelay(pdMS_TO_TICKS(4.1));
    
    lcd_send_command(40);
    vTaskDelay(pdMS_TO_TICKS(4.1));
    
    lcd_send_command(40);
    vTaskDelay(pdMS_TO_TICKS(4.1));

    ESP_LOGI(TAG, "3x 40 done: ");
    
    //lcd_send_command(32); //1line
    //vTaskDelay(pdMS_TO_TICKS(4.1));
    ESP_LOGI(TAG, "dispay off: ");
    lcd_send_command(8); //    Display off
    vTaskDelay(pdMS_TO_TICKS(4.1));
    ESP_LOGI(TAG, "clear: ");
    lcd_send_command(1); //clear
    vTaskDelay(pdMS_TO_TICKS(4.1));
    ESP_LOGI(TAG, "entry: ");
    lcd_send_command(2);  //Entry mode
    vTaskDelay(pdMS_TO_TICKS(4.1));
    
    //init sequence done

    ESP_LOGI(TAG, "display on: ");
    lcd_send_command(12);
    vTaskDelay(pdMS_TO_TICKS(2));
    //lcd_send_command(0x01);


}