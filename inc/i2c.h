/*
 * i2c.h
 *
 *  Created on: Jul 14, 2018
 *      Author: tarak
 */

/*
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "list.h"
 */

/*======================================================================================================================*/
/*This codes are for device init*/
#define DEVICE_RESET 0xF0
#define SET_READ_POINTER 0xE1
#define WRITE_CONFIG 0xD2
/*-----------------------------------------------------------------------------------*/
#define ONEWIRE_RESET 0xB4		//In command function
#define ONEWIRE_READ_BYTE 0x96 // In command function
#define ONEWIRE_WRITE_BYTE 0xA5
/*-----------------------------------------------------------------------------------*/
#define STATUS_REGISTER 0xF0
#define DATA_REG 0xE1
#define CONFIG_REG 0xC3
/*-----------------------------------------------------------------------------------*/
#define CONF_VALUE 0xF0
#define STATUS_MASK_BUSY 0x1A

/*======================================================================================================================*/
/*This codes are for sensor init*/
#define SKIP_ROM_COMMAND 0xCC
#define READ_ROM 0x33
/*-----------------------------------------------------------------------------------*/
#define START_CONVESION 0x44
#define WRITE_SCRATCHPAD 0x4E
#define READ_SCRATCH_PAD 0XBE
#define DSADD 0x18;
/*======================================================================================================================*/

/*-----------------------------------------VARIABLE DECLARATIONS------------------------------------------------------*/

/*=======================================================================================================================*/
static uint8_t write_init_tx[] = {DEVICE_RESET, WRITE_CONFIG, CONF_VALUE};
static uint8_t read_status_tx[] = {SET_READ_POINTER,STATUS_REGISTER};
static uint8_t read_status_rx[2];
/*----------------------------------------------------------------------------------------------------------------------*/
static uint8_t read_data_reg_tx[] = {SET_READ_POINTER,DATA_REG};
static uint8_t read_data_reg_rx[2];
/*=======================================================================================================================*/
static uint8_t one_wire_reset_rx[2];
static uint8_t one_wire_select[]={ONEWIRE_WRITE_BYTE,SKIP_ROM_COMMAND/*,START_CONVERSION,READ_SCRATCH_PAD*/};
static uint8_t start_conversion[]={ONEWIRE_WRITE_BYTE,START_CONVESION};
static uint8_t read_scratch_pad[]={ONEWIRE_WRITE_BYTE,READ_SCRATCH_PAD};
// And then we need to change pointer to data register
static uint8_t read_temp_data_tx[]={SET_READ_POINTER,DATA_REG};
static uint8_t read_temp_data_rx[1];

/*----------------------------------------------------------------------------------------------------------------------*/

/*=======================================================================================================================*/
/*Transfer structure for slave operations*/
static int mode_poll;
static I2C_XFER_T slave_xfr;

//====================ALL THIS MAKES OUR PROGRAM WORKS==================================================================//

/* Data area for slave operations */
static uint8_t slave_data_tx[]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08}; // THIS IS RX OF MASTER SO ADXL/DS2=>MASTER=>RX=>TX OF SLAVE=>HOST
static uint8_t slave_data_rx[]={0x01,0xF0,0x00,0x00,0xB0,0xA0,0x00,0xB0,0xA0};
static uint8_t slave_data_tx1[]={0x32,0x45,0x22,0x41};
static uint8_t tx_buff1[]={0x2d,0x08};// CONTAINS SETUP DATA

static uint8_t rx_buff1[8];
/*--------------------------------------------------------------------------------------------------------------------*/


/*=======================================================================================================================*/

struct i2c_data {
	I2C_XFER_T& xfer;
};
void i2c_block_init_cpp(I2C_ID_T id, int speed);
void i2c_set_mode_cpp(I2C_ID_T id, int polling);
void i2c_client_init_cpp(I2C_ID_T id);
void i2c_client_events_cpp(I2C_ID_T id, I2C_EVENT_T event);
void i2c_state_handling_cpp(I2C_ID_T id);
void i2c_rw_input_cpp(I2C_XFER_T *xfer, int ops);
void Chip_I2C_MasterCmdRead_cpp(I2C_ID_T id, uint8_t slaveAddr, uint8_t cmd, uint8_t *buff, int len);
void Chip_I2C_MasterSend_cpp(I2C_ID_T id, uint8_t slaveAddr, const uint8_t *buff, uint8_t len);
void Chip_I2C_MasterTransfer_cpp(I2C_ID_T id, I2C_XFER_T *xfer);
void one_wire_dev_init(I2C_XFER_T& xf,uint8_t dsad); // first run this in start code

uint8_t one_wire_dev_status(I2C_XFER_T& xf); //run this when seems necessary
void one_wire_reset(I2C_XFER_T& xf); //run this in loop external
void select_one_wire(I2C_XFER_T& xf); //run this in loop external
void start_cov(I2C_XFER_T& xf);//run this in loop internal and first
void read_scratch(I2C_XFER_T& xf);
void read_data(I2C_XFER_T& xf);
void read_temp(I2C_XFER_T& xf);
bool exec_temp(I2C_XFER_T& xf,int& r,uint8_t dsad); // use this third
void one_wire_read(I2C_XFER_T& xf);
void exec_scratch(I2C_XFER_T& xf,uint8_t dsad);//run this second probably
bool checkrx(void);
void send_data(I2C_XFER_T& xf,const uint8_t data );
void write_scratchblock(I2C_XFER_T& xf);
