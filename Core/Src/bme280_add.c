/*
 * bme280_add.c
 *
 *  Created on: Apr 11, 2023
 *      Author: Kacper
 */
#include "bme280.h"
#include "bme280_add.h"
#include "i2c.h"
#include "lcd.h"
#include "tim.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdio.h>

#define BUF_SIZE 10

static struct lcd_disp disp;
static struct bme280_dev bme;
static struct bme280_data comp_data;
struct
{
	float temp;/*zmiena przechowujaca ostatnią zmierzoną temperatura */
	float press;/*zmiena przechowujaca ostatnie zmierzone ciśnienie */
	float hum;/*zmiena przechowujaca ostatnią zmierzoną wilgotność */
	float list_temp[BUF_SIZE];/*zmiena przechowujaca 10 ostatnich pomiarów temperatury */
	float list_press[BUF_SIZE];/*zmiena przechowujaca 10 ostatnich pomiarów ciśnienia */
	float list_hum[BUF_SIZE];/*zmiena przechowujaca 10 ostatnich pomiarów wilgotności */
	int list_position_t;/*zmiena przechowujaca pozycje w buforze do której bedziemy zapisywać kolajną wartość temperatury*/
	int list_position_p;/*zmiena przechowujaca pozycje w buforze do której bedziemy zapisywać kolajną wartość ciśnienia*/
	int list_position_h;/*zmiena przechowujaca pozycje w buforze do której bedziemy zapisywać kolajną wartość wilgotnośći*/

}date;
static int8_t init_done;

/* Funkcja inicjalizujaca czujnik */
int8_t BME280_init(void) {
	int8_t rslt = BME280_OK;
	uint8_t settings_sel;
	init_done = BME280_E_DEV_NOT_FOUND;

	bme.intf_ptr = (BME280_I2C_ADDR_PRIM<<1);
	bme.intf = BME280_I2C_INTF;
	bme.read = user_i2c_read;
	bme.write = user_i2c_write;
	bme.delay_ms = user_delay_ms;

	rslt = bme280_init(&bme);

	/* Recommended mode of operation: Indoor navigation */
	bme.settings.osr_h = BME280_OVERSAMPLING_1X;
	bme.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme.settings.osr_t = BME280_OVERSAMPLING_2X;
	bme.settings.filter = BME280_FILTER_COEFF_16;
	bme.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;


		settings_sel = BME280_OSR_PRESS_SEL;
		settings_sel |= BME280_OSR_TEMP_SEL;
		settings_sel |= BME280_OSR_HUM_SEL;
		settings_sel |= BME280_STANDBY_SEL;
		settings_sel |= BME280_FILTER_SEL;

		rslt = bme280_set_sensor_settings(settings_sel, &bme);
		rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme);
		init_done = rslt;

	return rslt;
}


/* Funkcja pobierajaca temperature z czujnika i zapisująca ją do buffora z ostatnimi 10 pomiarami*/
int8_t get_temperature(void) {
	int8_t rslt = BME280_E_NULL_PTR;

	if(init_done == BME280_OK)
	{
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme);

	}

		    date.temp = comp_data.temperature;
		    date.list_temp[date.list_position_t]=date.temp;
		    date.list_position_t=date.list_position_t+1;
		    if(date.list_position_t>=BUF_SIZE){
		    	date.list_position_t=0;
		    }

}

/* Funkcja liczaca średnią temperature z ostatnich 10 pomiarów i wyświetlajaca ją na wyświetlaczu lcd */
/* oraz wysyłajaca przez uarta na komputer*/
int8_t print_temperature(void){

	float avg_temp=0;
	for (int i = 0; i <BUF_SIZE; i++) {
	    avg_temp=avg_temp+date.list_temp[i];
	}
	avg_temp=avg_temp/BUF_SIZE;

    disp.addr = (0x27 << 1);
    disp.bl = true;
    lcd_init(&disp);

	printf("Temperature\r\n");
    printf("%0.2f C\r\n", avg_temp);
    sprintf((char *)disp.f_line, "Temperatura");
    sprintf((char *)disp.s_line, "%0.2f C", avg_temp);
    lcd_display(&disp);

}

/* Funkcja pobierajaca ciśnienia z czujnika i zapisująca ją do buffora z ostatnimi 10 pomiarami*/
int8_t  get_pressure(void) {
	int8_t rslt = BME280_E_NULL_PTR;

	if(init_done == BME280_OK)
	{
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme);
	}

    date.press = 0.01 * comp_data.pressure;
    date.list_press[date.list_position_p]=date.press;
    date.list_position_p=date.list_position_p+1;
    if(date.list_position_p>=BUF_SIZE){
    	date.list_position_p=0;
    }
}

/* Funkcja liczaca średnie ciśnienie z ostatnich 10 pomiarów i wyświetlajaca ją na wyświetlaczu lcd */
/* oraz wysyłajaca przez uarta na komputer*/
int8_t print_pressure(void){

	float avg_press=0;
	for (int i = 0; i <BUF_SIZE; i++) {
	    avg_press=avg_press+date.list_press[i];
	}
	avg_press=avg_press/BUF_SIZE;

    disp.addr = (0x27 << 1);
    disp.bl = true;
    lcd_init(&disp);

    printf("Pressure\r\n");
    printf("%0.2f hPa\r\n", avg_press);
    sprintf((char *)disp.f_line, "Pressure");
    sprintf((char *)disp.s_line, "%0.2f hPa", avg_press);
    lcd_display(&disp);

}


/* Funkcja pobierajaca wilgotność z czujnika i zapisująca ją do buffora z ostatnimi 10 pomiarami*/
int8_t  get_humidity(void) {
	int8_t rslt = BME280_E_NULL_PTR;

	if(init_done == BME280_OK)
	{
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme);
	}

    date.hum = comp_data.humidity;
    date.list_hum[date.list_position_h]=date.hum;
    date.list_position_h=date.list_position_h+1;
    if(date.list_position_h>=BUF_SIZE){
    	date.list_position_h=0;
    }


}

/* Funkcja liczaca średnią wilgotność z ostatnich 10 pomiarów i wyświetlajaca ją na wyświetlaczu lcd */
/* oraz wysyłajaca przez uarta na komputer*/
int8_t print_humidity(void){

	float avg_hum=0;
	for (int i = 0; i <BUF_SIZE; i++) {
	    avg_hum=avg_hum+date.list_hum[i];
	}
	avg_hum=avg_hum/BUF_SIZE;

    disp.addr = (0x27 << 1);
    disp.bl = true;
    lcd_init(&disp);

    printf("Humidity\r\n");
    printf("%0.2f \r\n", avg_hum);
    sprintf((char *)disp.f_line, "Humidity");
    sprintf((char *)disp.s_line, "%0.2f ", avg_hum);
    lcd_display(&disp);


}



/* Funkcja dodaje opoźnienia wymagane do prawidłowago działania sprzętu*/
void user_delay_ms(uint32_t period)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < period);

}

/* Funkcja umożliwiajaca odzczyt danych z czujnika*/
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    HAL_I2C_Mem_Read(&hi2c1, intf_ptr, reg_addr, 1, reg_data, len, 100);


    return rslt;
}

/* Funkcja umożliwiajaca zapis danych do czujnika*/
int8_t user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    HAL_I2C_Mem_Write(&hi2c1, intf_ptr, reg_addr, 1, reg_data, len, 100);

    return rslt;
}
