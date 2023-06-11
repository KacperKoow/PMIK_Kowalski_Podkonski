/*
 * lcd.c
 *
 *  Created on: May 11, 2023
 *      Author: Kacper
 */
#include "lcd.h"
#include "i2c.h"
#include "bme280_add.h"


/* Funkcja inicjalizujaca wyświetlacz lcd*/
void lcd_init(struct lcd_disp * lcd)
{
	uint8_t xpin = 0;
	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	/* init sequence */
	user_delay_ms(40);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);
	user_delay_ms(5);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);
	user_delay_ms(1);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);

	/* set 4-bit mode */
	lcd_write(lcd->addr, INIT_4_BIT_MODE, xpin);

	/* set cursor mode */
	lcd_write(lcd->addr, UNDERLINE_OFF_BLINK_OFF, xpin);

	/* clear */
	lcd_clear(lcd);

}

/* Funkcja wysyłajaca dane do wyświetlacza lcd*/
void lcd_write(uint8_t addr, uint8_t data, uint8_t xpin)
{
	uint8_t tx_data[4];

	/* split data */
	tx_data[0] = (data & 0xF0) | EN_PIN | xpin;
	tx_data[1] = (data & 0xF0) | xpin;
	tx_data[2] = (data << 4) | EN_PIN | xpin;
	tx_data[3] = (data << 4) | xpin;

	/* send data via i2c */
	HAL_I2C_Master_Transmit(&HI2C_DEF, addr, tx_data, 4, 100);

	user_delay_ms(5);
}


/* Funkcja wyświetlajaca dane na wyświetlaczu lcd*/
void lcd_display(struct lcd_disp * lcd)
{
	uint8_t xpin = 0, i = 0;

	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	lcd_clear(lcd);

	/* send first line data */
	lcd_write(lcd->addr, FIRST_CHAR_LINE_1, xpin);
	while(lcd->f_line[i])
	{
		lcd_write(lcd->addr, lcd->f_line[i], (xpin | RS_PIN));
		i++;
	}

	/* send second line data */
	i = 0;
	lcd_write(lcd->addr, FIRST_CHAR_LINE_2, xpin);
	while(lcd->s_line[i])
	{
		lcd_write(lcd->addr, lcd->s_line[i], (xpin | RS_PIN));
		i++;
	}
}

/* Funkcja czyszcząca wyświatlacz*/
void lcd_clear(struct lcd_disp * lcd)
{
	uint8_t xpin = 0;

	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	/* clear display */
	lcd_write(lcd->addr, CLEAR_LCD, xpin);
}

