#include "gfx_pack.h"
#include "monospace_font.c"
#include "string.h"
#include <stdio.h>

#define GFX_PACK_SPI spi0
#define GFX_PACK_RESET_PIN 21
#define GFX_PACK_CHIP_SELECT_PIN 17
#define GFX_PACK_SERIAL_CLOCK_PIN 18
#define GFX_PACK_MASTER_OUT_SLAVE_IN_PIN 19
#define GFX_PACK_MASTER_IN_SLAVE_OUT_PIN 2147483647
#define GFX_PACK_DATA_COMMAND_PIN 20
#define GFX_PACK_BACKLIGHT_PIN 9
#define GFX_PACK_SPI_BAUDRATE 10000000

#define GFX_PACK_DISPLAY_WIDTH 128
#define GFX_PACK_DISPLAY_HEIGHT 64
#define GFX_PACK_PAGE_SIZE 128

#define GFX_PACK_CHAR_WIDTH 6
#define GFX_PACK_CHAR_HEIGHT 10

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
#define REG_SEG_DIR_REV 0xa1      //# 0xa1: Column address 128 is mapped to SEG0
#define REG_DISPNORMAL 0xa6       //# 0xa6: Normal display */
#define REG_DISPINVERSE 0xa7      //# 0xa7: Inverse display
#define REG_DISPRAM 0xa4          //# 0xa4: Resume to RAM content display */
#define REG_DISPENTIRE 0xa5       //# 0xa5: Entire display
#define REG_BIAS_1_9 0xa2         //# 0xa2: Select BIAS setting 1/9 */
#define REG_BIAS_1_7 0xa3         //# 0xa3: Select BIAS setting 1/7
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

// Frame buffer for the display
uint8_t framebuffer[GFX_PACK_DISPLAY_WIDTH * GFX_PACK_DISPLAY_HEIGHT / 8] = {0};

/**
 * @brief Function that performs the initialization sequence
 * 
 */
void gfx_pack_init_sequence(void);

/**
 * @brief Sends a command to the GFX Pack
 * 
 * @param command Code of the command
 * @param len Length of the data send
 * @param data Data to be sent to the GFX Pack
 */
void command(uint8_t command, size_t len, const char* data);

uint8_t* get_char_map(char c);

void gfx_pack_init(void)
{
    spi_init(GFX_PACK_SPI, GFX_PACK_SPI_BAUDRATE);

    gpio_init(GFX_PACK_BUTTON_A); // Initialize GPIO pin for button A
    gpio_set_function(GFX_PACK_BUTTON_A, GPIO_FUNC_SIO); // Set function of the pin to IO pin
    gpio_set_dir(GFX_PACK_BUTTON_A, GPIO_IN); // Set in direction for the button A GPIO pin
    gpio_pull_up(GFX_PACK_BUTTON_A); // Pull button A GPIO pin up

    gpio_init(GFX_PACK_BUTTON_B); // Initialize GPIO pin for button B
    gpio_set_function(GFX_PACK_BUTTON_B, GPIO_FUNC_SIO); // Set function of the pin to IO pin
    gpio_set_dir(GFX_PACK_BUTTON_B, GPIO_IN); // Set in direction for the button B GPIO pin
    gpio_pull_up(GFX_PACK_BUTTON_B); // Pull button B GPIO pin up

    gpio_init(GFX_PACK_BUTTON_C); // Initialize GPIO pin for button C
    gpio_set_function(GFX_PACK_BUTTON_C, GPIO_FUNC_SIO); // Set function of the pin to IO pin
    gpio_set_dir(GFX_PACK_BUTTON_C, GPIO_IN); // Set in direction for the button C GPIO pin
    gpio_pull_up(GFX_PACK_BUTTON_C); // Pull button C GPIO pin up

    gpio_init(GFX_PACK_BUTTON_D); // Initialize GPIO pin for button D
    gpio_set_function(GFX_PACK_BUTTON_D, GPIO_FUNC_SIO); // Set function of the pin to IO pin
    gpio_set_dir(GFX_PACK_BUTTON_D, GPIO_IN); // Set in direction for the button D GPIO pin
    gpio_pull_up(GFX_PACK_BUTTON_D); // Pull button D GPIO pin up

    gpio_init(GFX_PACK_BUTTON_E); // Initialize GPIO pin for button E
    gpio_set_function(GFX_PACK_BUTTON_E, GPIO_FUNC_SIO); // Set function of the pin to IO pin
    gpio_set_dir(GFX_PACK_BUTTON_E, GPIO_IN); // Set in direction for the button E GPIO pin
    gpio_pull_up(GFX_PACK_BUTTON_E); // Pull button E GPIO pin up

    gpio_init(GFX_PACK_RESET_PIN); // Initialize GPIO pin for display reset
    gpio_set_function(GFX_PACK_RESET_PIN, GPIO_FUNC_SIO); // Set function of the reset pin to IO pin
    gpio_set_dir(GFX_PACK_RESET_PIN, GPIO_OUT); // Set direction of the reset pin to out
    gpio_put(GFX_PACK_RESET_PIN, 1); // Put reset pin high
    
    gpio_init(GFX_PACK_CHIP_SELECT_PIN); // Initialize GPIO pin for display chip select
    gpio_set_function(GFX_PACK_CHIP_SELECT_PIN, GPIO_FUNC_SIO); // Set function of the chip select pin to IO pin
    gpio_set_dir(GFX_PACK_CHIP_SELECT_PIN, GPIO_OUT); // Set direction of the CS pin to out
    gpio_put(GFX_PACK_CHIP_SELECT_PIN, 1); // Put CS pin high

    gpio_init(GFX_PACK_DATA_COMMAND_PIN); // Initialize GPIO pin for data/command
    gpio_set_function(GFX_PACK_DATA_COMMAND_PIN, GPIO_FUNC_SIO); // Set function of the data/command pin to IO pin
    gpio_set_dir(GFX_PACK_DATA_COMMAND_PIN, GPIO_OUT); // Set direction of the DC pin to out
    gpio_put(GFX_PACK_DATA_COMMAND_PIN, 0); // Put DC pin low

    gpio_init(GFX_PACK_MASTER_OUT_SLAVE_IN_PIN); // Initialize GPIO pin for MOSI
    gpio_set_function(GFX_PACK_MASTER_OUT_SLAVE_IN_PIN, GPIO_FUNC_SPI); // Set function of the MOSI pin to SPI pin
    gpio_init(GFX_PACK_SERIAL_CLOCK_PIN); // Initialize GPIO pin for SCLK
    gpio_set_function(GFX_PACK_SERIAL_CLOCK_PIN, GPIO_FUNC_SPI); // Set function of the SCLK pin to SPI pin


    gpio_init(GFX_PACK_BACKLIGHT_PIN); // Initialize GPIO pin for the backlight
    pwm_config cfg = pwm_get_default_config(); // Default PWM config
    pwm_set_wrap(pwm_gpio_to_slice_num(GFX_PACK_BACKLIGHT_PIN), 65535); // Enable fine PWM
    pwm_init(pwm_gpio_to_slice_num(GFX_PACK_BACKLIGHT_PIN), &cfg, true); // Initialize PWM for the display backlight
    gpio_set_function(GFX_PACK_BACKLIGHT_PIN, GPIO_FUNC_PWM); // Set function of the backlight pin to PWM
    gpio_set_dir(GFX_PACK_BACKLIGHT_PIN, GPIO_OUT); // Set out direction for the GPIO pin for the backlight
    gfx_pack_set_backlight(0); // Set backlight to 0

    gfx_pack_reset(); // Reset display

    gfx_pack_init_sequence(); // Perform initialization sequence

    gfx_pack_clear_display(); // Clear display
    gfx_pack_update(); // Refresh display

    gfx_pack_set_backlight(255); // Turn backlight to max
}

void gfx_pack_init_sequence(void)
{
    command(REG_BIAS_1_7, 0, NULL);
    command(REG_SEG_DIR_NORMAL, 0, NULL); // normal row direction
    command(REG_SETCOMREVERSE, 0, NULL); // reverse column direction
    command(REG_DISPNORMAL, 0, NULL);
    command(REG_SETSTARTLINE | 0x00, 0, NULL); // Set start line position
    command(REG_POWERCTRL, 0, NULL);
    command(REG_RATIO | 4, 0, NULL);
    command(REG_DISPON, 0, NULL); // Turn display on
    command(REG_SETCONTRAST, 0, NULL); // Set contrast
    command(30, 0, NULL); // defalut contrast level
    command(REG_DISPRAM, 0, NULL); // Display RAM contents
}

void command(uint8_t command, size_t len, const char* data)
{
    gpio_put(GFX_PACK_CHIP_SELECT_PIN, 0); // Enable GFX Communication
    gpio_put(GFX_PACK_DATA_COMMAND_PIN, 0); // Command mode
    spi_write_blocking(GFX_PACK_SPI, &command, 1); // Send command
    gpio_put(GFX_PACK_CHIP_SELECT_PIN, 1); // Disable GFX Communication
    busy_wait_us(100);

    if (data) { // Send data
        gpio_put(GFX_PACK_CHIP_SELECT_PIN, 0); // Enable GFX Communication
        gpio_put(GFX_PACK_DATA_COMMAND_PIN, 1); // Data mode
        spi_write_blocking(GFX_PACK_SPI, (const uint8_t*)data, len); // Send data
        gpio_put(GFX_PACK_CHIP_SELECT_PIN, 1); // Disable GFX Communication
    }    
}

void gfx_pack_update(void)
{
    uint8_t pagebuffer[GFX_PACK_PAGE_SIZE]; // Initialize page buffer
    uint8_t page_byte_selector, page_bit_selector;

    uint8_t max_page = GFX_PACK_DISPLAY_WIDTH * GFX_PACK_DISPLAY_HEIGHT / 
                        (8 * GFX_PACK_PAGE_SIZE);

    for (uint8_t page = 0; page < max_page; page++) // For each page
    {
        for (uint16_t pixel_index = 0; pixel_index < (8 * GFX_PACK_PAGE_SIZE); pixel_index++) // For each pixel
        {
            page_byte_selector = pixel_index % GFX_PACK_PAGE_SIZE;
            page_bit_selector = pixel_index / GFX_PACK_PAGE_SIZE;

            if (framebuffer[page * GFX_PACK_PAGE_SIZE + pixel_index / 8] & (0b10000000 >> (pixel_index % 8))) // row-column array transformation
            {
                pagebuffer[page_byte_selector] |= (0b1 << page_bit_selector);
            }
            else
            {
                pagebuffer[page_byte_selector] &= ~(0b1 << page_bit_selector);
            }
        }

        command(REG_ENTER_RMWMODE, 0, NULL); // Read Modify Write mode
        command(REG_SETPAGESTART | page, 0, NULL); // Page
        command(REG_SETCOLL, 0, NULL); // Set current column
        command(REG_SETCOLH, 0, NULL);

        gpio_put(GFX_PACK_CHIP_SELECT_PIN, 0); // Enable communication
        gpio_put(GFX_PACK_DATA_COMMAND_PIN, 1); // Data mode
        spi_write_blocking(GFX_PACK_SPI, (const uint8_t*)pagebuffer, sizeof(pagebuffer)); // Send page
        gpio_put(GFX_PACK_DATA_COMMAND_PIN, 0); // Return to command mode
        gpio_put(GFX_PACK_CHIP_SELECT_PIN, 1); // Disable communication
        command(REG_EXIT_RMWMODE, 0, NULL); // Exit Read Modify Write mode
    }
}

bool gfx_pack_write_char(point_t* position, char c)
{
    
    uint8_t* char_map = get_char_map(c); // Pixel map of the character
    uint16_t start_row = position->y * GFX_PACK_CHAR_HEIGHT; // Start row in px
    uint16_t start_byte_row = GFX_PACK_DISPLAY_WIDTH / 8 * start_row; // Starting byte of the row
    uint16_t start_col = position->x * GFX_PACK_CHAR_WIDTH; // Start column in px
    if ((position->x >= (GFX_PACK_DISPLAY_WIDTH / GFX_PACK_CHAR_WIDTH)) ||
        (position->y >= (GFX_PACK_DISPLAY_HEIGHT / GFX_PACK_CHAR_HEIGHT))) return false; // if character outside of drawable area return false
    uint16_t start_byte = start_byte_row + start_col / 8; // Start byte of the character
    uint8_t start_bit = start_col % 8; // Start bit of the character
    for (int i = 0; i < GFX_PACK_CHAR_HEIGHT - 1; i++) // For each character pixel column
    {
        for (int j = 0; j < GFX_PACK_CHAR_WIDTH - 1; j++) // For each character pixel row
        {
            uint8_t curr_bit = (start_bit + j) % 8; // Current bit 
            uint16_t curr_byte = start_byte + i * GFX_PACK_DISPLAY_WIDTH / 8 + (start_bit + j) / 8; // Current byte
            if (curr_byte >= GFX_PACK_DISPLAY_HEIGHT * GFX_PACK_DISPLAY_WIDTH / 8) return false; // If outside of buffer return false
            if (char_map[i] & (0b10000000 >> j)) // Set bit value in buffer
            {
                framebuffer[curr_byte] |= (0b10000000 >> curr_bit);
            }
            else
            {
                framebuffer[curr_byte] &= ~(0b10000000 >> curr_bit);
            }
        }
    }
    return true; // Character successfully written to the buffer
}

bool gfx_pack_write_text(point_t* position, char* text)
{
    int text_len = strlen(text); // Length of the text to be written
    uint8_t text_pos = 0; // Current character to be written
    while (text_pos < text_len)
    {
        while (gfx_pack_write_char(position, text[text_pos])) // Write character until unable to
        {
            position->x++; // Increment column
            text_pos++; // Increment text position
            if (text_pos == text_len) break; // If end of the text break
        }
        position->y++; // Increment row
        position->x = 0; // Set column to 0
        if (position->y >= GFX_PACK_DISPLAY_HEIGHT / GFX_PACK_CHAR_HEIGHT) return false; // If row not writable return false
    }
    return true; // Text written successfully
}

void gfx_pack_set_backlight(uint8_t brightness)
{
    float gamma = 2.8; // mapping 0..255 onto range 0..65535 with gamma correction for output PWM
    uint16_t value = (uint16_t)(pow((float)(brightness) / 255.0f, gamma) * 65535.0f + 0.5f);
    pwm_set_gpio_level(GFX_PACK_BACKLIGHT_PIN, value); // Set PWM level to backlight pin
}

bool gfx_pack_read_button(uint8_t GFX_PACK_BUTTON)
{
    if (GFX_PACK_BUTTON == GFX_PACK_BUTTON_A || // Check if valid button pin
        GFX_PACK_BUTTON == GFX_PACK_BUTTON_B || 
        GFX_PACK_BUTTON == GFX_PACK_BUTTON_C || 
        GFX_PACK_BUTTON == GFX_PACK_BUTTON_D || 
        GFX_PACK_BUTTON == GFX_PACK_BUTTON_E)
        return !gpio_get(GFX_PACK_BUTTON); // Return button status - buttons are pressed low
    else return false;
}

void gfx_pack_reset(void)
{
    gpio_put(GFX_PACK_RESET_PIN, 0); // Reset pin low
    sleep_ms(10);
    gpio_put(GFX_PACK_RESET_PIN, 1); // Reset pin high
    sleep_ms(10);
}

uint8_t* get_char_map(char c)
{
    switch (c)
    {
        case 'A': return (uint8_t*)font_A;
        case 'B': return (uint8_t*)font_B;
        case 'C': return (uint8_t*)font_C;
        case 'D': return (uint8_t*)font_D;
        case 'E': return (uint8_t*)font_E;
        case 'F': return (uint8_t*)font_F;
        case 'G': return (uint8_t*)font_G;
        case 'H': return (uint8_t*)font_H;
        case 'I': return (uint8_t*)font_I;
        case 'J': return (uint8_t*)font_J;
        case 'K': return (uint8_t*)font_K;
        case 'L': return (uint8_t*)font_L;
        case 'M': return (uint8_t*)font_M;
        case 'N': return (uint8_t*)font_N;
        case 'O': return (uint8_t*)font_O;
        case 'P': return (uint8_t*)font_P;
        case 'Q': return (uint8_t*)font_Q;
        case 'R': return (uint8_t*)font_R;
        case 'S': return (uint8_t*)font_S;
        case 'T': return (uint8_t*)font_T;
        case 'U': return (uint8_t*)font_U;
        case 'V': return (uint8_t*)font_V;
        case 'W': return (uint8_t*)font_W;
        case 'X': return (uint8_t*)font_X;
        case 'Y': return (uint8_t*)font_Y;
        case 'Z': return (uint8_t*)font_Z;

        case 'a': return (uint8_t*)font_a;
        case 'b': return (uint8_t*)font_b;
        case 'c': return (uint8_t*)font_c;
        case 'd': return (uint8_t*)font_d;
        case 'e': return (uint8_t*)font_e;
        case 'f': return (uint8_t*)font_f;
        case 'g': return (uint8_t*)font_g;
        case 'h': return (uint8_t*)font_h;
        case 'i': return (uint8_t*)font_i;
        case 'j': return (uint8_t*)font_j;
        case 'k': return (uint8_t*)font_k;
        case 'l': return (uint8_t*)font_l;
        case 'm': return (uint8_t*)font_m;
        case 'n': return (uint8_t*)font_n;
        case 'o': return (uint8_t*)font_o;
        case 'p': return (uint8_t*)font_p;
        case 'q': return (uint8_t*)font_q;
        case 'r': return (uint8_t*)font_r;
        case 's': return (uint8_t*)font_s;
        case 't': return (uint8_t*)font_t;
        case 'u': return (uint8_t*)font_u;
        case 'v': return (uint8_t*)font_v;
        case 'w': return (uint8_t*)font_w;
        case 'x': return (uint8_t*)font_x;
        case 'y': return (uint8_t*)font_y;
        case 'z': return (uint8_t*)font_z;

        case '1': return (uint8_t*)font_1;
        case '2': return (uint8_t*)font_2;
        case '3': return (uint8_t*)font_3;
        case '4': return (uint8_t*)font_4;
        case '5': return (uint8_t*)font_5;
        case '6': return (uint8_t*)font_6;
        case '7': return (uint8_t*)font_7;
        case '8': return (uint8_t*)font_8;
        case '9': return (uint8_t*)font_9;
        case '0': return (uint8_t*)font_0;

        case '+': return (uint8_t*)font_plus;
        case '-': return (uint8_t*)font_dash;
        case '=': return (uint8_t*)font_equal;
        case '_': return (uint8_t*)font_underscore;
        case '/': return (uint8_t*)font_slash;
        case '\\': return (uint8_t*)font_bslash;
        case '*': return (uint8_t*)font_astrix;
        case ':': return (uint8_t*)font_colon;
        case ';': return (uint8_t*)font_semicol;
        case '(': return (uint8_t*)font_lpar;
        case ')': return (uint8_t*)font_rpar;
        case '[': return (uint8_t*)font_lbrack;
        case ']': return (uint8_t*)font_rbrack;
        case '{': return (uint8_t*)font_lbrace;
        case '}': return (uint8_t*)font_rbrace;
        case '<': return (uint8_t*)font_lt;
        case '>': return (uint8_t*)font_gt;
        case '!': return (uint8_t*)font_excl;
        case '?': return (uint8_t*)font_ques;
        case '.': return (uint8_t*)font_dot;
        case ',': return (uint8_t*)font_comma;
        case '\'': return (uint8_t*)font_apos;
        case '"': return (uint8_t*)font_quote;
        case '&': return (uint8_t*)font_ampr;
        case '#': return (uint8_t*)font_hashtag;
        case '%': return (uint8_t*)font_percent;
        case '^': return (uint8_t*)font_carrot;
        case '~': return (uint8_t*)font_tilde;
        case '`': return (uint8_t*)font_tick;
        //case '|': return (uint8_t*)font_pipe;
        case '|': return (uint8_t*)font_degree; // C char does not support codes higher than 127, ° is 248
        case '$': return (uint8_t*)font_dollar;
        case '@': return (uint8_t*)font_at;
        case ' ': return (uint8_t*)font_space;
        default: return (uint8_t*)font_undefined;
    }
}

void gfx_pack_clear_display(void)
{
    for (int i = 0; i < GFX_PACK_DISPLAY_WIDTH * GFX_PACK_DISPLAY_HEIGHT / 8; i++) // For each byte in framebuffer
    {
        framebuffer[i] = 0; // Sets byte to 0
    }
}

