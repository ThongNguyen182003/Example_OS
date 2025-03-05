#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kernel.h>

#define SSD1306_MAX_SEG 128
#define SSD1306_MAX_LINE 7
#define SSD1306_DEF_FONT_SIZE 5

struct ssd1306_i2c_module {
	struct i2c_client *client;
	uint8_t line_num;
	uint8_t cursor_position;
	uint8_t font_size;
};

static const unsigned char ssd1306_font[][SSD1306_DEF_FONT_SIZE] = {
		{0x00, 0x00, 0x00, 0x00, 0x00}, // space
		{0x00, 0x00, 0x2f, 0x00, 0x00}, // !
		{0x00, 0x07, 0x00, 0x07, 0x00}, // "
		{0x14, 0x7f, 0x14, 0x7f, 0x14}, // #
		{0x24, 0x2a, 0x7f, 0x2a, 0x12}, // $
		{0x23, 0x13, 0x08, 0x64, 0x62}, // %
		{0x36, 0x49, 0x55, 0x22, 0x50}, // &
		{0x00, 0x05, 0x03, 0x00, 0x00}, // '
		{0x00, 0x1c, 0x22, 0x41, 0x00}, // (
		{0x00, 0x41, 0x22, 0x1c, 0x00}, // )
		{0x14, 0x08, 0x3E, 0x08, 0x14}, // *
		{0x08, 0x08, 0x3E, 0x08, 0x08}, // +
		{0x00, 0x00, 0xA0, 0x60, 0x00}, // ,
		{0x08, 0x08, 0x08, 0x08, 0x08}, // -
		{0x00, 0x60, 0x60, 0x00, 0x00}, // .
		{0x20, 0x10, 0x08, 0x04, 0x02}, // /
		{0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
		{0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
		{0x42, 0x61, 0x51, 0x49, 0x46}, // 2
		{0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
		{0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
		{0x27, 0x45, 0x45, 0x45, 0x39}, // 5
		{0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
		{0x01, 0x71, 0x09, 0x05, 0x03}, // 7
		{0x36, 0x49, 0x49, 0x49, 0x36}, // 8
		{0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
		{0x00, 0x36, 0x36, 0x00, 0x00}, // :
		{0x00, 0x56, 0x36, 0x00, 0x00}, // ;
		{0x08, 0x14, 0x22, 0x41, 0x00}, // <
		{0x14, 0x14, 0x14, 0x14, 0x14}, // =
		{0x00, 0x41, 0x22, 0x14, 0x08}, // >
		{0x02, 0x01, 0x51, 0x09, 0x06}, // ?
		{0x32, 0x49, 0x59, 0x51, 0x3E}, // @
		{0x7C, 0x12, 0x11, 0x12, 0x7C}, // A
		{0x7F, 0x49, 0x49, 0x49, 0x36}, // B
		{0x3E, 0x41, 0x41, 0x41, 0x22}, // C
		{0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
		{0x7F, 0x49, 0x49, 0x49, 0x41}, // E
		{0x7F, 0x09, 0x09, 0x09, 0x01}, // F
		{0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
		{0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
		{0x00, 0x41, 0x7F, 0x41, 0x00}, // I
		{0x20, 0x40, 0x41, 0x3F, 0x01}, // J
		{0x7F, 0x08, 0x14, 0x22, 0x41}, // K
		{0x7F, 0x40, 0x40, 0x40, 0x40}, // L
		{0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
		{0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
		{0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
		{0x7F, 0x09, 0x09, 0x09, 0x06}, // P
		{0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
		{0x7F, 0x09, 0x19, 0x29, 0x46}, // R
		{0x46, 0x49, 0x49, 0x49, 0x31}, // S
		{0x01, 0x01, 0x7F, 0x01, 0x01}, // T
		{0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
		{0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
		{0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
		{0x63, 0x14, 0x08, 0x14, 0x63}, // X
		{0x07, 0x08, 0x70, 0x08, 0x07}, // Y
		{0x61, 0x51, 0x49, 0x45, 0x43}, // Z
		{0x00, 0x7F, 0x41, 0x41, 0x00}, // [
		{0x55, 0xAA, 0x55, 0xAA, 0x55}, // Backslash (Checker pattern)
		{0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
		{0x04, 0x02, 0x01, 0x02, 0x04}, // ^
		{0x40, 0x40, 0x40, 0x40, 0x40}, // _
		{0x00, 0x03, 0x05, 0x00, 0x00}, // `
		{0x20, 0x54, 0x54, 0x54, 0x78}, // a
		{0x7F, 0x48, 0x44, 0x44, 0x38}, // b
		{0x38, 0x44, 0x44, 0x44, 0x20}, // c
		{0x38, 0x44, 0x44, 0x48, 0x7F}, // d
		{0x38, 0x54, 0x54, 0x54, 0x18}, // e
		{0x08, 0x7E, 0x09, 0x01, 0x02}, // f
		{0x18, 0xA4, 0xA4, 0xA4, 0x7C}, // g
		{0x7F, 0x08, 0x04, 0x04, 0x78}, // h
		{0x00, 0x44, 0x7D, 0x40, 0x00}, // i
		{0x40, 0x80, 0x84, 0x7D, 0x00}, // j
		{0x7F, 0x10, 0x28, 0x44, 0x00}, // k
		{0x00, 0x41, 0x7F, 0x40, 0x00}, // l
		{0x7C, 0x04, 0x18, 0x04, 0x78}, // m
		{0x7C, 0x08, 0x04, 0x04, 0x78}, // n
		{0x38, 0x44, 0x44, 0x44, 0x38}, // o
		{0xFC, 0x24, 0x24, 0x24, 0x18}, // p
		{0x18, 0x24, 0x24, 0x18, 0xFC}, // q
		{0x7C, 0x08, 0x04, 0x04, 0x08}, // r
		{0x48, 0x54, 0x54, 0x54, 0x20}, // s
		{0x04, 0x3F, 0x44, 0x40, 0x20}, // t
		{0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
		{0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
		{0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
		{0x44, 0x28, 0x10, 0x28, 0x44}, // x
		{0x1C, 0xA0, 0xA0, 0xA0, 0x7C}, // y
		{0x44, 0x64, 0x54, 0x4C, 0x44}, // z
		{0x00, 0x10, 0x7C, 0x82, 0x00}, // {
		{0x00, 0x00, 0xFF, 0x00, 0x00}, // |
		{0x00, 0x82, 0x7C, 0x10, 0x00}, // }
		{0x00, 0x06, 0x09, 0x09, 0x06}	// ~ (Degrees)
};

static int ssd1306_i2c_write(struct ssd1306_i2c_module *module, unsigned char *buf, unsigned int len)
{
	return i2c_master_send(module->client, buf, len);
}

static int ssd1306_i2c_read(struct ssd1306_i2c_module *module, unsigned char *out_buf, unsigned int len)
{
	return i2c_master_recv(module->client, out_buf, len);
}

static void ssd1306_write(struct ssd1306_i2c_module *module, bool is_cmd, unsigned char data)
{
	unsigned char buf[2] = {0};

	if (is_cmd == true) {
		buf[0] = 0x00;
	} else {
		buf[0] = 0x40;
	}

	buf[1] = data;
	ssd1306_i2c_write(module, buf, 2);
}

static void ssd1306_set_cursor(struct ssd1306_i2c_module *module, uint8_t line_num, uint8_t cursor_position)
{
	if ((line_num <= SSD1306_MAX_LINE) && (cursor_position < SSD1306_MAX_SEG)) {
		module->line_num = line_num;			   // Save the specified line number
		module->cursor_position = cursor_position; // Save the specified cursor position
		ssd1306_write(module, true, 0x21);				   // cmd for the column start and end address
		ssd1306_write(module, true, cursor_position);	   // column start addr
		ssd1306_write(module, true, SSD1306_MAX_SEG - 1);  // column end addr
		ssd1306_write(module, true, 0x22);				   // cmd for the page start and end address
		ssd1306_write(module, true, line_num);			   // page start addr
		ssd1306_write(module, true, SSD1306_MAX_LINE);	   // page end addr
	}
}

static void ssd1306_goto_next_line(struct ssd1306_i2c_module *module)
{
	module->line_num++;
	module->line_num = (module->line_num & SSD1306_MAX_LINE);
	ssd1306_set_cursor(module, module->line_num, 0);
}

static void ssd1306_print_char(struct ssd1306_i2c_module *module, unsigned char c)
{
	uint8_t data_byte;
	uint8_t temp = 0;

	if (((module->cursor_position + module->font_size) >= SSD1306_MAX_SEG) || (c == '\n'))
		ssd1306_goto_next_line(module);

	if (c != '\n') {
		c -= 0x20;
		do {
			data_byte = ssd1306_font[c][temp]; 
			ssd1306_write(module, false, data_byte);   
			module->cursor_position++;

			temp++;

		} while (temp < module->font_size);

		ssd1306_write(module, false, 0x00); 
		module->cursor_position++;
	}
}

static void ssd1306_print_string(struct ssd1306_i2c_module *module, unsigned char *str)
{
	while (*str) {
		ssd1306_print_char(module, *str++);
	}
}

static void ssd1306_set_brightness(struct ssd1306_i2c_module *module, uint8_t brightness)
{
	ssd1306_write(module, true, 0x81);
	ssd1306_write(module, true, brightness);
}

static void ssd1306_clear(struct ssd1306_i2c_module *module)
{
	unsigned int total = 128 * 8; 
	int i;

	for (i = 0; i < total; i++) {
		ssd1306_write(module, false, 0);
	}
}

static int ssd1306_display_init(struct ssd1306_i2c_module *module)
{
	msleep(100);
	ssd1306_write(module, true, 0xAE); // Entire Display OFF
	ssd1306_write(module, true, 0xD5); // Set Display Clock Divide Ratio and Oscillator Frequency
	ssd1306_write(module, true, 0x80); // Default Setting for Display Clock Divide Ratio and Oscillator Frequency that is recommended
	ssd1306_write(module, true, 0xA8); // Set Multiplex Ratio
	ssd1306_write(module, true, 0x3F); // 64 COM lines
	ssd1306_write(module, true, 0xD3); // Set display offset
	ssd1306_write(module, true, 0x00); // 0 offset
	ssd1306_write(module, true, 0x40); // Set first line as the start line of the display
	ssd1306_write(module, true, 0x8D); // Charge pump
	ssd1306_write(module, true, 0x14); // Enable charge dump during display on
	ssd1306_write(module, true, 0x20); // Set memory addressing mode
	ssd1306_write(module, true, 0x00); // Horizontal addressing mode
	ssd1306_write(module, true, 0xA1); // Set segment remap with column address 127 mapped to segment 0
	ssd1306_write(module, true, 0xC8); // Set com output scan direction, scan from com63 to com 0
	ssd1306_write(module, true, 0xDA); // Set com pins hardware configuration
	ssd1306_write(module, true, 0x12); // Alternative com pin configuration, disable com left/right remap
	ssd1306_write(module, true, 0x81); // Set contrast control
	ssd1306_write(module, true, 0x80); // Set Contrast to 128
	ssd1306_write(module, true, 0xD9); // Set pre-charge period
	ssd1306_write(module, true, 0xF1); // Phase 1 period of 15 DCLK, Phase 2 period of 1 DCLK
	ssd1306_write(module, true, 0xDB); // Set Vcomh deselect level
	ssd1306_write(module, true, 0x20); // Vcomh deselect level ~ 0.77 Vcc
	ssd1306_write(module, true, 0xA4); // Entire display ON, resume to RAM content display
	ssd1306_write(module, true, 0xA6); // Set Display in Normal Mode, 1 = ON, 0 = OFF
	ssd1306_write(module, true, 0x2E); // Deactivate scroll
	ssd1306_write(module, true, 0xAF); // Display ON in normal mode
	ssd1306_clear(module);

	return 0;
}


static int ssd1306_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{
	struct ssd1306_i2c_module *module;

	module = kmalloc(sizeof(*module), GFP_KERNEL);
	if (!module) {
		pr_err("kmalloc failed\n");
		return -1;
	}

	module->client = client;
	module->line_num = 0;
	module->cursor_position = 0;
	module->font_size = SSD1306_DEF_FONT_SIZE;
	i2c_set_clientdata(client, module);

	ssd1306_display_init(module);
	ssd1306_set_cursor(module, 0, 0);
	ssd1306_print_string(module, "Hello\nWorld\n");
	pr_info("Jay: %s, %d\n", __func__, __LINE__);
	return 0;
}

static int ssd1306_i2c_remove(struct i2c_client *client)
{
	struct ssd1306_i2c_module *module = i2c_get_clientdata(client);

	ssd1306_print_string(module, "End!!!");
	msleep(1000);
	ssd1306_clear(module);
	ssd1306_write(module, true, 0xAE); // Entire Display OFF

	kfree(module);
	pr_info("Jay: %s, %d\n", __func__, __LINE__);
	return 0;
}

static const struct i2c_device_id ssd1306_i2c_match_id[] = {
	{ "ssd1306", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ssd1306_i2c_match_id);

static struct i2c_driver ssd1306_i2c_driver = {
	.driver = {
		.name = "ssd1306",
		.owner = THIS_MODULE,
	},
	.probe = ssd1306_i2c_probe,
	.remove = ssd1306_i2c_remove,
	.id_table = ssd1306_i2c_match_id,
};

module_i2c_driver(ssd1306_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jay <thong.nguyen1820031956@hcmut.edu.vn>");
MODULE_DESCRIPTION("SSD1306");
