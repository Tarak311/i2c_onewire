/*
 * project_cfg.h
 *
 *  Created on: Jul 17, 2018
 *      Author: tarak
 */

#ifndef PROJECT_CFG_H_
#define PROJECT_CFG_H_


#define I2C_MODE
#define FREERTOS_VAR
#define SPEED_100KHZ         100000
#define SPEED_400KHZ         400000

#define DEFAULT_I2C          I2C0

#define I2C_EEPROM_BUS       DEFAULT_I2C
#define I2C_CLIENT_BUS       DEFAULT_I2C
#define I2C_SLAVE_CLIENT_ADDR  0x5D
#define ADDADXL 0x53
#define I2C_SLAVE_DATA_SIZE 4


#endif /* PROJECT_CFG_H_ */
