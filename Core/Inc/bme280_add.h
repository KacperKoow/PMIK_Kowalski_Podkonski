/*
 * bme280_add.h
 *
 *  Created on: Apr 11, 2023
 *      Author: Kacper
 */

#ifndef INC_BME280_ADD_H_
#define INC_BME280_ADD_H_

int8_t BME280_init(void);
int8_t get_temperature(void);
int8_t print_temperature(void);
int8_t  get_pressure(void);
int8_t print_pressure(void);
int8_t  get_humidity(void);
int8_t print_humidity(void);
void print_sensor_data(struct bme280_data *comp_data);
void user_delay_ms(uint32_t period);
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);


#endif /* INC_BME280_ADD_H_ */
