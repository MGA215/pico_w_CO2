#include "gfx_pack.h"

#define GFX_PACK_SPI spi0
#define GFX_PACK_RESET_PIN 21
#define GFX_PACK_CHIP_SELECT_PIN 17
#define GFX_PACK_SERIAL_CLOCK_PIN 18
#define GFX_PACK_MASTER_OUT_SLAVE_IN_PIN 19
#define GFX_PACK_MASTER_IN_SLAVE_OUT_PIN 2147483647
#define GFX_PACK_DATA_COMMAND_PIN 20
#define GFX_PACK_BACKLIGHT_PIN 9
#define GFX_SPI_BAUDRATE 10000000

#define REG_DISPOFF     0xAE      //
#define REG_DISPON      0xAF      //
#define REG_SETSTARTLINE 0x40     //
#define REG_STARTLINE_MASK 0x3f   //
#define REG_RATIO       0x20      //
#define REG_SETPAGESTART 0xb0     //
#define REG_PAGESTART_MASK 0x07   //
#define REG_SETCOLL 0x00          //# 0x00-0x0f: Set lower column address */
#define REG_COLL_MASK 0x0f        //
#define REG_SETCOLH 0x10          //# 0x10-0x1f: Set higher column address */
#define REG_COLH_MASK 0x0F        //
#define REG_SEG_DIR_NORMAL 0xa0   //# 0xa0: Column address 0 is mapped to SEG0 */
#define REG_SEG_DIR_REV 0xa1      //# 0xa1: Column address 128 is mapped to S
#define REG_DISPNORMAL 0xa6       //# 0xa6: Normal display */
#define REG_DISPINVERSE 0xa7      //# 0xa7: Inverse disp
#define REG_DISPRAM 0xa4          //# 0xa4: Resume to RAM content display */
#define REG_DISPENTIRE 0xa5       //# 0xa5: Entire display
#define REG_BIAS_1_9 0xa2         //# 0xa2: Select BIAS setting 1/9 */
#define REG_BIAS_1_7 0xa3         //# 0xa3: Select BIAS setting 
#define REG_ENTER_RMWMODE 0xe0    //# 0xe0: Enter the Read Modify Write mode */
#define REG_EXIT_RMWMODE 0xee     //# 0xee: Leave the Read Modify Write mode */
#define REG_EXIT_SOFTRST 0xe2     //# 0xe2: Software RESET
#define REG_SETCOMNORMAL 0xc0     //# 0xc0: Set COM output direction, normal mode */
#define REG_SETCOMREVERSE 0xc8    //# 0xc8: Set COM output direction, reverse mode
#define REG_POWERCTRL_VF 0x29     //# 0x29: Control built-in power circuit */
#define REG_POWERCTRL_VR 0x2a     //# 0x2a: Control built-in power circuit */
#define REG_POWERCTRL_VB 0x2c     //# 0x2c: Control built-in power circuit */
#define REG_POWERCTRL 0x2f        //# 0x2f: Control built-in power circuit
#define REG_REG_RES_RR0 0x21      //# 0x21: Regulation Resistior ratio */
#define REG_REG_RES_RR1 0x22      //# 0x22: Regulation Resistior ratio */
#define REG_REG_RES_RR2 0x24      //# 0x24: Regulation Resistior ratio
#define REG_SETCONTRAST 0x81      //# 0x81: Set contrast level
#define REG_SETBOOSTER 0xf8       //# Set booster level */
#define REG_SETBOOSTER4X 0x00     //# Set booster level */
#define REG_SETBOOSTER5X 0x01     //# Set booster level

/**
 * @brief Function to perform the initialization sequence
 * 
 */
void gfx_pack_init_sequence(void);

void gfx_pack_init(void)
{
    spi_init(GFX_PACK_SPI, GFX_SPI_BAUDRATE);

    gpio_init(GFX_PACK_BUTTON_A); // Initialize GPIO pin for button A
    gpio_set_function(GFX_PACK_BUTTON_A, GPIO_FUNC_SIO);
    gpio_set_dir(GFX_PACK_BUTTON_A, GPIO_IN); // Set in direction for the button A GPIO pin
    gpio_pull_up(GFX_PACK_BUTTON_A); // Pull button A GPIO pin up

    gpio_init(GFX_PACK_BUTTON_B); // Initialize GPIO pin for button B
    gpio_set_function(GFX_PACK_BUTTON_B, GPIO_FUNC_SIO);
    gpio_set_dir(GFX_PACK_BUTTON_B, GPIO_IN); // Set in direction for the button B GPIO pin
    gpio_pull_up(GFX_PACK_BUTTON_B); // Pull button B GPIO pin up

    gpio_init(GFX_PACK_BUTTON_C); // Initialize GPIO pin for button C
    gpio_set_function(GFX_PACK_BUTTON_C, GPIO_FUNC_SIO);
    gpio_set_dir(GFX_PACK_BUTTON_C, GPIO_IN); // Set in direction for the button C GPIO pin
    gpio_pull_up(GFX_PACK_BUTTON_C); // Pull button C GPIO pin up

    gpio_init(GFX_PACK_BUTTON_D); // Initialize GPIO pin for button D
    gpio_set_function(GFX_PACK_BUTTON_D, GPIO_FUNC_SIO);
    gpio_set_dir(GFX_PACK_BUTTON_D, GPIO_IN); // Set in direction for the button D GPIO pin
    gpio_pull_up(GFX_PACK_BUTTON_D); // Pull button D GPIO pin up

    gpio_init(GFX_PACK_BUTTON_E); // Initialize GPIO pin for button E
    gpio_set_function(GFX_PACK_BUTTON_E, GPIO_FUNC_SIO);
    gpio_set_dir(GFX_PACK_BUTTON_E, GPIO_IN); // Set in direction for the button E GPIO pin
    gpio_pull_up(GFX_PACK_BUTTON_E); // Pull button E GPIO pin up


    gpio_init(GFX_PACK_RESET_PIN);
    gpio_set_function(GFX_PACK_RESET_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(GFX_PACK_RESET_PIN, GPIO_OUT);
    gpio_put(GFX_PACK_RESET_PIN, 1);
    
    gpio_init(GFX_PACK_CHIP_SELECT_PIN);
    gpio_set_function(GFX_PACK_CHIP_SELECT_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(GFX_PACK_CHIP_SELECT_PIN, GPIO_OUT);
    gpio_put(GFX_PACK_CHIP_SELECT_PIN, 1);

    gpio_init(GFX_PACK_DATA_COMMAND_PIN);
    gpio_set_function(GFX_PACK_DATA_COMMAND_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(GFX_PACK_DATA_COMMAND_PIN, GPIO_OUT);
    gpio_put(GFX_PACK_DATA_COMMAND_PIN, 0);

    gpio_init(GFX_PACK_MASTER_OUT_SLAVE_IN_PIN);
    gpio_set_function(GFX_PACK_MASTER_OUT_SLAVE_IN_PIN, GPIO_FUNC_SPI);
    gpio_set_function(GFX_PACK_SERIAL_CLOCK_PIN, GPIO_FUNC_SPI);


    gpio_init(GFX_PACK_BACKLIGHT_PIN); // Initialize GPIO pin for the backlight
    pwm_config cfg = pwm_get_default_config();
    pwm_set_wrap(pwm_gpio_to_slice_num(GFX_PACK_BACKLIGHT_PIN), 65535);
    pwm_init(pwm_gpio_to_slice_num(GFX_PACK_BACKLIGHT_PIN), &cfg, true);
    gpio_set_function(GFX_PACK_BACKLIGHT_PIN, GPIO_FUNC_PWM);
    gpio_set_dir(GFX_PACK_BACKLIGHT_PIN, GPIO_OUT); // Set out direction for the GPIO pin for the backlight
    gfx_pack_set_backlight(0);

    gfx_pack_reset();

    gfx_pack_init_sequence(); // Perform initialization sequence

    gfx_pack_set_backlight(255);
}

void gfx_pack_init_sequence(void)
{
    command(REG_BIAS_1_7, 0, NULL);
    command(REG_SEG_DIR_NORMAL, 0, NULL);
    command(REG_SETCOMREVERSE, 0, NULL);
    command(REG_DISPNORMAL, 0, NULL);
    command(REG_SETSTARTLINE | 0x00, 0, NULL);
    command(REG_POWERCTRL, 0, NULL);
    command(REG_RATIO | 4, 0, NULL);
    command(REG_DISPON, 0, NULL);
    command(REG_SETCONTRAST, 0, NULL);
    command(30, 0, NULL); // defalut contrast level
}

void command(uint8_t command, size_t len, const char* data)
{
    gpio_put(GFX_PACK_CHIP_SELECT_PIN, 0);
    gpio_put(GFX_PACK_DATA_COMMAND_PIN, 0); // Command mode
    spi_write_blocking(GFX_PACK_SPI, &command, 1);
    gpio_put(GFX_PACK_CHIP_SELECT_PIN, 1);
    sleep_us(100);
    if (data) {
        gpio_put(GFX_PACK_CHIP_SELECT_PIN, 0);
        gpio_put(GFX_PACK_DATA_COMMAND_PIN, 1); // Data mode
        spi_write_blocking(GFX_PACK_SPI, (const uint8_t*)data, len);
        gpio_put(GFX_PACK_CHIP_SELECT_PIN, 1);
    }    
}

void gfx_pack_set_backlight(uint8_t brightness)
{
    float gamma = 2.8; // mapping 0..255 onto range 0..65535 with gamma correction for output PWM
    uint16_t value = (uint16_t)(pow((float)(brightness) / 255.0f, gamma) * 65535.0f + 0.5f);
    pwm_set_gpio_level(GFX_PACK_BACKLIGHT_PIN, value);
}

bool gfx_pack_read_button(uint8_t GFX_PACK_BUTTON)
{
    // if (GFX_PACK_BUTTON == GFX_PACK_BUTTON_A || // Check if valid button pin
    //     GFX_PACK_BUTTON == GFX_PACK_BUTTON_B || 
    //     GFX_PACK_BUTTON == GFX_PACK_BUTTON_C || 
    //     GFX_PACK_BUTTON == GFX_PACK_BUTTON_D || 
    //     GFX_PACK_BUTTON == GFX_PACK_BUTTON_E)
        return !gpio_get(GFX_PACK_BUTTON); // Return button status
    //else return false;
}

void gfx_pack_reset(void)
{
    gpio_put(GFX_PACK_RESET_PIN, 1);
    sleep_ms(10);
    gpio_put(GFX_PACK_RESET_PIN, 0);
    sleep_ms(10);
}





