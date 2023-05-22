
// // Nicholas Hardy - U97871602

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "driver/adc.h"
#include <math.h>
#include "esp_adc_cal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/i2c.h"
#include "esp_sleep.h"
#include "esp_timer.h"

static const char *TAG = "example";

////////////// 14-Segment Display //////////
////////////////////////////////////////////
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value
#define Kp 0.75
#define Ki 0.5
#define Kd 0.0

static const uint16_t alphafonttable[] = {
    0b0000000000000000, //  space
    0b0000000000000110, // !
    0b0000001000100000, // "
    0b0001001011001110, // #
    0b0001001011101101, // $
    0b0000110000100100, // %
    0b0010001101011101, // &
    0b0000010000000000, // '
    0b0010010000000000, // (
    0b0000100100000000, // )
    0b0011111111000000, // *
    0b0001001011000000, // +
    0b0000100000000000, // ,
    0b0000000011000000, // -
    0b0000000000000000, // .
    0b0000110000000000, // /
    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111, // 9
    0b0001001000000000, // :
    0b0000101000000000, // ;
    0b0010010000000000, // <
    0b0000000011001000, // =
    0b0000100100000000, // >
    0b0001000010000011, // ?
    0b0000001010111011, // @
    0b0000000011110111, // A
    0b0001001010001111, // B
    0b0000000000111001, // C
    0b0001001000001111, // D
    0b0000000011111001, // E
    0b0000000001110001, // F
    0b0000000010111101, // G
    0b0000000011110110, // H
    0b0001001000000000, // I
    0b0000000000011110, // J
    0b0010010001110000, // K
    0b0000000000111000, // L
    0b0000010100110110, // M
    0b0010000100110110, // N
    0b0000000000111111, // O
    0b0000000011110011, // P
    0b0010000000111111, // Q
    0b0010000011110011, // R
    0b0000000011101101, // S
    0b0001001000000001, // T
    0b0000000000111110, // U
    0b0000110000110000, // V
    0b0010100000110110, // W
    0b0010110100000000, // X
    0b0001010100000000, // Y
    0b0000110000001001, // Z
    0b0000000000111001, // [
    0b0010000100000000, //
    0b0000000000001111, // ]
    0b0000110000000011, // ^
    0b0000000000001000, // _
    0b0000000100000000, // `
    0b0001000001011000, // a
    0b0010000001111000, // b
    0b0000000011011000, // c
    0b0000100010001110, // d
    0b0000100001011000, // e
    0b0000000001110001, // f
    0b0000010010001110, // g
    0b0001000001110000, // h
    0b0001000000000000, // i
    0b0000000000001110, // j
    0b0011011000000000, // k
    0b0000000000110000, // l
    0b0001000011010100, // m
    0b0001000001010000, // n
    0b0000000011011100, // o
    0b0000000101110000, // p
    0b0000010010000110, // q
    0b0000000001010000, // r
    0b0010000010001000, // s
    0b0000000001111000, // t
    0b0000000000011100, // u
    0b0010000000000100, // v
    0b0010100000010100, // w
    0b0010100011000000, // x
    0b0010000000001100, // y
    0b0000100001001000, // z
    0b0000100101001001, // {
    0b0001001000000000, // |
    0b0010010010001001, // }
    0b0000010100100000, // ~
    0b0011111111111111,
    
};

/////////// I2C //////////////

//register definitions
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

// Please consult the datasheet of your servo before changing the following parameters
#define ESC_MIN_PULSEWIDTH_US 900      // Minimum pulse width in microsecond
#define ESC_MAX_PULSEWIDTH_US 2100     // Maximum pulse width in microsecond
#define ESC_NEUTRAL_PULSEWIDTH_US 1500 // neutral value for esc to calibrate

/// STEERING SERVO
#define S_SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define S_SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond

#define SERVO_MIN_DEGREE -90 // Minimum angle
#define SERVO_MAX_DEGREE 90  // Maximum angle

#define STEERING_SERVO_PULSE_GPIO 26 // GPIO connects to the PWM signal for servo 1
#define ESC_SERVO_PULSE_GPIO 25      // GPIO connects to the PWM signal for servo 2

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms

#define MAX_DUTY_LENGTH 8191 // Maximum duty cycle of 8191

//////////// ULTRASONIC SENSOR /////////
////////////////////////////////////////
#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64  // Multisampling
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_3; // GPIO39 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;
//////////// ULTRASONIC SENSOR /////////
////////////////////////////////////////

////////////// SPEED SENSOR /////////////
////////////////////////////////////////
#define EXAMPLE_PCNT_HIGH_LIMIT 100
#define EXAMPLE_PCNT_LOW_LIMIT -100
#define WHEEL_SPEED_GPIO_A 34
#define WHEEL_SPEED_GPIO_B 2

#define setpoint 5


static void i2c_example_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;
    
    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    
    //Config
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    conf.clk_flags = 0;                                       // UNCOMMENT IF YOU GET ERRORS (see readme.md)
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}
    
    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK) {printf("- initialized: yes\n\n");}
    
    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}


// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
        printf("- No I2C devices found!" "\n");
    printf("\n");
}


// Turn on oscillator for alpha display
int alpha_oscillator() {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1/ portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    return ret;
}

// Set blink rate to off
int no_blink() {
    int ret;
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
    i2c_master_stop(cmd2);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd2);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
    int ret;
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
    i2c_master_stop(cmd3);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd3);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    return ret;
}

// Read and write to register Functions ///////////////////////////////////////////////////////////

// Write one byte to register (single byte write)
void writeRegister(uint8_t reg, uint8_t data) {
  // create i2c communication init
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);    // 1. Start (Master write start)
  i2c_master_write_byte(cmd, ( LIDARLite_ADDRESS << 1 ) | WRITE_BIT, I2C_MASTER_ACK); // (Master write slave add + write bit)
  // wait for salve to ack
  i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK); // (Master write register address)
  // wait for slave to ack
  i2c_master_write_byte(cmd, data, I2C_MASTER_ACK);// master write data
  // wait for slave to ack
  i2c_master_stop(cmd); // 11. Stop
  // i2c communication done and delete
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  // no return here since data is written by master onto slave
}

// Read register (single byte read)
uint16_t readRegister(uint8_t reg) {
  uint8_t data1; //first byte MSB
  uint8_t data2; //second byte LSB

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_cmd_handle_t cmd1 = i2c_cmd_link_create();

  // Start
  i2c_master_start(cmd);
  // Master write slave address + write bit
  i2c_master_write_byte(cmd, ( LIDARLite_ADDRESS << 1 ) | WRITE_BIT, I2C_MASTER_ACK);
  // Master write register address + send ack
  i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
  //master stops
  i2c_master_stop(cmd);
  // This starts the I2C communication
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  //master starts
  i2c_master_start(cmd1);
  // Master write slave address + read bit
  i2c_master_write_byte(cmd1, ( LIDARLite_ADDRESS << 1 ) | READ_BIT, I2C_MASTER_ACK);
  // Master reads in slave ack and data
  i2c_master_read_byte(cmd1, &data1 , I2C_MASTER_ACK);
  i2c_master_read_byte(cmd1, &data2 , I2C_MASTER_NACK);
  // Master nacks and stops.
  i2c_master_stop(cmd1);
  // This starts the I2C communication
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd1, 1 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd1);
  

  uint16_t two_byte_data = (data1 << 8 | data2);
  return two_byte_data;
}

//////////////////////////////////////


static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}
////////////// SPEED SENSOR /////////////
////////////////////////////////////////

static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (S_SERVO_MAX_PULSEWIDTH_US - S_SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + S_SERVO_MIN_PULSEWIDTH_US;
}

static inline uint32_t ESC_SPEED(float speed)
{
    uint32_t ESC_value;

    if (speed >= 0)
    {
        ESC_value = ((ESC_MAX_PULSEWIDTH_US - ESC_NEUTRAL_PULSEWIDTH_US) * speed) + ESC_NEUTRAL_PULSEWIDTH_US;
    }
    else
    {
        ESC_value = ((ESC_NEUTRAL_PULSEWIDTH_US - ESC_MIN_PULSEWIDTH_US) * speed) + ESC_NEUTRAL_PULSEWIDTH_US;
    }

    ESP_LOGI(TAG, "ESC Value: %ld", ESC_value);

    return ESC_value;
}

static void check_efuse(void)
{
    // Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }
    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
}
static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("Characterized using eFuse Vref\n");
    }
    else
    {
        printf("Characterized using Default Vref\n");
    }
}

static inline uint32_t Forward_Collision_Senseor(void)
{

    uint32_t adc_reading = 0;
    // Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        if (unit == ADC_UNIT_1)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }
        else
        {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    // Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    // float distance = (voltage/1024)/5
    float distance = (voltage * 5) / 10;
    distance = distance - 18;
    // printf("Raw: %ld\tVoltage: %ldmV\tDistance: %.2f cm\n", adc_reading, voltage, distance);

    return distance;
}

// static float get_LidarDist(void) {
//     //write to register 0x00 the value 0x04
//         writeRegister(0x00, 0x04);
//         //READ REGISTER 0X01 UNTIL LSB GOES LOW
//         //if LSB goes low then set flag to true
//         int flag = 1;
//         while(flag){
//             uint16_t data = readRegister(0x01);
//             // printf("DATA: %d\n", data);
//             flag = data & (1<<15);
//             vTaskDelay(5);
//         }

//         uint16_t distance = readRegister(RegisterHighLowB);
//         float distance_m = (float)distance/100.0;
//         vTaskDelay(100 / portTICK_PERIOD_MS);
//         return distance_m;
// }

void app_main(void)
{
    i2c_example_master_init();
    i2c_scanner();

    printf("\n>> Polling Lidar\n");

    float Lidar_Dist;

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    // Debug
    int ret;
    printf(">> Test Alphanumeric Display: \n");
    
    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}

    //////////////////////////////////////////////
    /////////////// Steering Servo ///////////////

    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer1 = NULL;
    mcpwm_timer_config_t timer_config1 = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config1, &timer1));

    mcpwm_oper_handle_t oper1 = NULL;
    mcpwm_operator_config_t operator_config1 = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config1, &oper1));

    ESP_LOGI(TAG, "Connect timer and operator for steering servo");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper1, timer1));

    ESP_LOGI(TAG, "Create comparator and generator from the operator for steering servo");
    mcpwm_cmpr_handle_t comparator1 = NULL;
    mcpwm_comparator_config_t comparator_config1 = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper1, &comparator_config1, &comparator1));

    mcpwm_gen_handle_t generator1 = NULL;
    mcpwm_generator_config_t generator_config1 = {
        .gen_gpio_num = STEERING_SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper1, &generator_config1, &generator1));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event for steering servo");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator1,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator1,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator1, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer for steering servo");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer1));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer1, MCPWM_TIMER_START_NO_STOP));

    //////////////////////////////////////////////
    /////////////// Steering Servo ///////////////

    /////////////////////////////////////////
    /////////////// ESC Servo ///////////////

    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer2 = NULL;
    mcpwm_timer_config_t timer_config2 = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config2, &timer2));

    mcpwm_oper_handle_t oper2 = NULL;
    mcpwm_operator_config_t operator_config2 = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config2, &oper2));

    ESP_LOGI(TAG, "Connect timer and operator for ESC servo");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper2, timer2));

    ESP_LOGI(TAG, "Create comparator and generator from the operator for ESC servo");
    mcpwm_cmpr_handle_t comparator2 = NULL;
    mcpwm_comparator_config_t comparator_config2 = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper2, &comparator_config2, &comparator2));

    mcpwm_gen_handle_t generator2 = NULL;
    mcpwm_generator_config_t generator_config2 = {
        .gen_gpio_num = ESC_SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper2, &generator_config2, &generator2));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event for ESC servo");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator2,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator2,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator2, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer for ESC servo");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer2));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer2, MCPWM_TIMER_START_NO_STOP));

    /////////////////////////////////////////
    /////////////// ESC Servo ///////////////

    //////////// ULTRASONIC SENSOR /////////
    ////////////////////////////////////////

    // Check if Two Point or Vref are burned into eFuse
    check_efuse();
    // Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    float current_distance;
    //////////// ULTRASONIC SENSOR /////////
    ////////////////////////////////////////

    ////////////// WHEEL SPEED SENSOR ///////
    ////////////////////////////////////////
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = WHEEL_SPEED_GPIO_A,
        .level_gpio_num = WHEEL_SPEED_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = WHEEL_SPEED_GPIO_B,
        .level_gpio_num = WHEEL_SPEED_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_LOGI(TAG, "add watch points and register callbacks");
    int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++)
    {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));
    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(WHEEL_SPEED_GPIO_A, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif

    ////////////// WHEEL SPEED SENSOR ///////
    ////////////////////////////////////////

    int angle = 0;
    int step = 2;
    int neutral = 0;

    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(angle)));

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(neutral)));
    vTaskDelay(3000 / portTICK_PERIOD_MS);

//ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(0.15)));

    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SERVO_ANGLE(25)));
    // vTaskDelay(3000 / portTICK_PERIOD_MS);
    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SERVO_ANGLE(0)));
    // vTaskDelay(3000 / portTICK_PERIOD_MS);
    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, 1580));
    // 1580 = 300RPM
    angle = -90;
    int flag = 1;
    int pulse_count = 0;
    int event_count = 0;
    int counter = 0;
    char str_number[10] = {'0'};
    float circumfrence = 23.9;
    float speed = 0;
    int prev_count = 0;
    float time_since_last_rev = 0;
    
    float error;
    float integral = 0;
    float derivative;
    float output;
    float previous_error = 0;
    float dt = 0.1;
    float default_speed = 0.15;
    int flag_left=0;
    int flag_right=0;
    bool front = 0;
    //vTaskDelay(1000);
    current_distance = Forward_Collision_Senseor();
    while (1)
    {
       
        

        
       // if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(20))) {
            //ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
        //}
  //  else {
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
           // ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
     //   }
        
        current_distance = Forward_Collision_Senseor();
        
        if(counter == 0)
        {
            time_since_last_rev = esp_timer_get_time();
        }
        
        
        if(pulse_count != prev_count)
        {
            counter = counter + 1;
            printf("Counter %d\n",counter);
        }
        if (counter == 12)
        {
            float finaltime = esp_timer_get_time();
            float time_rev = finaltime - time_since_last_rev;
            time_rev = time_rev/1000000;
           // ESP_LOGI(TAG, "timetimetimetimetimetiem: %lld\n", time_rev);
            counter = 0;
            speed = circumfrence/time_rev;
            printf("speed %f\n",speed);
        }
        prev_count = pulse_count;
        sprintf(str_number, "%.5f", speed);

        
        
                error = setpoint - speed;
                integral = integral + error * dt;
                derivative = (error - previous_error) / dt;
                output = Kp * error + Ki * integral + Kd * derivative;
                previous_error = error;
     

        
        //write to register 0x00 the value 0x04
        writeRegister(0x00, 0x04);
        //READ REGISTER 0X01 UNTIL LSB GOES LOW
        //if LSB goes low then set flag to true
        flag = 1;
        
        while(flag){
            uint16_t data = readRegister(0x01);
            // printf("DATA: %d\n", data);
            flag = data & (1<<15);
            vTaskDelay(5);
            flag = 0;
        }

        uint16_t distance = readRegister(RegisterHighLowB);
        float distance_m = (float)distance;
        ESP_LOGI(TAG, "Lidar Distance %fcm away", distance_m);

        uint16_t displaybuffer[8];
        if(speed >= 10)
        {
            displaybuffer[0] = alphafonttable[str_number[0]-32];
            displaybuffer[1] = alphafonttable[str_number[1]-32];
            displaybuffer[3] = alphafonttable[str_number[2]-32];
            displaybuffer[2] = 0b0100000000000000;
        }
        
        if(speed < 10)
        {
            displaybuffer[0] = alphafonttable[16];
            displaybuffer[1] = alphafonttable[str_number[0]-32];
            displaybuffer[3] = alphafonttable[str_number[1]-32];
            displaybuffer[2] = 0b0100000000000000;
        }
       

        //ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(-5)));
        //vTaskDelay(10);

    
       // else
       // {
          //  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(-0.14)));
          //  ESP_LOGI(TAG, "Obstacle Detected %fcm away\n\n\n\n\n", current_distance);
       // }
        
        if(current_distance > 120)
        {
            
            if(distance_m >= 30 && distance_m<=50)
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(-5)));

            }
            
            if(distance_m < 30)
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(15)));
                flag_left = 1;
                flag_right = 0;
            }
            
            if(distance_m >50)
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(-20)));
                flag_left = 0;
                flag_right = 1;
            }
            
            if(error > 0)
            {
                default_speed = default_speed + 0.01;
                if(default_speed>0.16)
                {
                    default_speed = 0.15;
                }
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(default_speed)));
            }
            if(error < 0)
            {
                
                default_speed = default_speed - 0.01;
                
                if(default_speed<0.12)
                {
                    default_speed = 0.14;
                }
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(default_speed)));
            }
            
            if (default_speed >= 0.16)
            {
                default_speed = 0.15;
            }
            
        }
        
        else
         {
             ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(-0.2)));
             ESP_LOGI(TAG, "Obstacle Detected %fcm away\n\n\n\n\n", current_distance);
             ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(90)));
             vTaskDelay(100);
             ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(0.15)));
             vTaskDelay(500);

         }
        

        
        
        
        int ret;
      //  printf(">> Test Alphanumeric Display: \n");
    
        // Set up routines
        // Turn on alpha oscillator
        ret = alpha_oscillator();
       // if(ret == ESP_OK) {printf("- oscillator: ok \n");}
        // Set display blink off
        ret = no_blink();
       // if(ret == ESP_OK) {printf("- blink: off \n");}
        ret = set_brightness_max(0xF);
       // if(ret == ESP_OK) {printf("- brightness: max \n");}


        i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
        for (uint8_t i = 0; i < 8; i++) {
            i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
        }
        i2c_master_stop(cmd4);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd4);
    }
}
