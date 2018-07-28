/*
 * i2c.cpp
 *
 *  Created on: Jul 14, 2018
 *      Author: tarak
 */

#include "board.h"
/*
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "list.h"
 */
#include "i2c.h"
#include "gpiopin.hh"
#include "project_cfg.h"

/*ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc*/
extern "C" {


static void i2c_block_init(I2C_ID_T id, int speed);
static void i2c_set_mode(I2C_ID_T id, int polling);
static void i2c_client_init(I2C_ID_T id);
static void i2c_client_events(I2C_ID_T id, I2C_EVENT_T event);
/*-----------------------------------------INT-------------------------------------------------------------------------*/
static void i2c_state_handling(I2C_ID_T id);
static void i2c_rw_input(I2C_XFER_T *xfer, int ops);

//--------------------------------------------Definitions--------------------------------------------------------------//
static void i2c_rw_input(I2C_XFER_T *xfer, int ops)
{
	int tmp, i;



	xfer->slaveAddr = ADDADXL;
	xfer->rxBuff = 0;
	xfer->txBuff = 0;
	xfer->txSz = 0;
	xfer->rxSz = 0;
	xfer->rxSz = sizeof(slave_data_tx);
	xfer->rxBuff = slave_data_tx;// THIS LINE IS IMPORTANT AS IT ASSIGN TX TO RX AND WE ONLY NEED TX
	xfer->txSz = sizeof(tx_buff1);
	xfer->txBuff = tx_buff1;// ASSIGN SOMTHING TO THIS BUFFER FOR EXAMPLE 0XAD AND 0X08
}

/*Initialize the I2C bus*/
static void i2c_block_init(I2C_ID_T id, int speed)
{
	Board_I2C_Init(id);

	/* Initialize I2C*/
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, speed);

	/* Set default mode to interrupt*/
	i2c_set_mode(id, 0);
}

static void i2c_set_mode(I2C_ID_T id, int polling)
{
	if(!polling) {
		mode_poll &= ~(1 << id);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
		NVIC_EnableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
	} else {
		mode_poll |= 1 << id;
		NVIC_DisableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);
	}
}


/* Simulate a client slave device*/
static  void i2c_client_init(I2C_ID_T id)
{

	slave_xfr.slaveAddr = (I2C_SLAVE_CLIENT_ADDR << 1);
	i2c_client_events(id, I2C_EVENT_DONE);
	Chip_I2C_SlaveSetup(id, I2C_SLAVE_0, &slave_xfr, i2c_client_events, 0);
	/* Setup SysTick timer to get the button status updated at regular intervals*/
	SysTick_Config(Chip_Clock_GetSystemClockRate() / 50);
}
static void i2c_client_events(I2C_ID_T id, I2C_EVENT_T event)
{
	switch(event) {
	case I2C_EVENT_DONE:
		slave_xfr.rxBuff = slave_data_rx;

		slave_xfr.rxSz = sizeof(slave_data_rx);
		slave_xfr.txBuff = slave_data_tx;
		slave_xfr.txSz = sizeof(slave_data_tx)+1;
		break;

	case I2C_EVENT_SLAVE_RX:
		slave_xfr.rxBuff = slave_data_rx;
		slave_xfr.rxSz = sizeof(slave_data_rx);
		break;

	case I2C_EVENT_SLAVE_TX:
		if(slave_xfr.txSz == 1) {
			slave_xfr.txBuff = slave_data_tx;
			slave_xfr.txSz = sizeof(slave_data_tx) + 1;
		}
		break;
	}
}

/*
===============================================INTRUPT HANDLER==========================================================*/
static void i2c_state_handling(I2C_ID_T id)
{
	if (Chip_I2C_IsMasterActive(id)) {
		Chip_I2C_MasterStateHandler(id);
	} else {
		Chip_I2C_SlaveStateHandler(id);
	}
}
}
/*ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc*/

void i2c_block_init_cpp(I2C_ID_T id, int speed){i2c_block_init(id,speed);}
void i2c_set_mode_cpp(I2C_ID_T id, int polling){i2c_set_mode(id,polling);}
void i2c_client_init_cpp(I2C_ID_T id){i2c_client_init(id);}
void i2c_client_events_cpp(I2C_ID_T id, I2C_EVENT_T event){i2c_client_events(id,event);}
void i2c_state_handling_cpp(I2C_ID_T id){i2c_state_handling(id);}
void i2c_rw_input_cpp(I2C_XFER_T *xfer, int ops){ i2c_rw_input(xfer,ops);}
void Chip_I2C_MasterCmdRead_cpp(I2C_ID_T id, uint8_t slaveAddr, uint8_t cmd, uint8_t *buff, int len){Chip_I2C_MasterCmdRead( id, slaveAddr, cmd, buff, len);}
void Chip_I2C_MasterSend_cpp(I2C_ID_T id, uint8_t slaveAddr, const uint8_t *buff, uint8_t len){ Chip_I2C_MasterSend(id, slaveAddr,buff, len);}
void Chip_I2C_MasterTransfer_cpp(I2C_ID_T id, I2C_XFER_T *xfer){ Chip_I2C_MasterTransfer(id, xfer);}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
void one_wire_dev_init(I2C_XFER_T& xf)
{
	xf.slaveAddr=DSADD;
	xf.txBuff=write_init_tx;
	xf.txSz=sizeof(write_init_tx);
	Chip_I2C_MasterTransfer_cpp(I2C0, &xf);
}
uint8_t one_wire_dev_status(I2C_XFER_T& xf)
{
	uint8_t i;
	//gpio_pin_port pin1(LPC_GPIO ,0,2,output,&mu);
	//gpio_pin_port pin2(LPC_GPIO ,0,3,output,&mu);
	xf.slaveAddr=DSADD;
	xf.txBuff=read_status_tx;
	xf.rxBuff=&i;
	xf.txSz=sizeof(read_status_tx);
	xf.rxSz=1;
	//Chip_I2C_MasterTransfer_cpp(I2C0, &xf);
	Chip_I2C_MasterSend_cpp(I2C0, xf.slaveAddr,xf.txBuff, xf.txSz);
	bool m=true;
	while(m)
	{
		Chip_I2C_MasterRead(I2C0,xf.slaveAddr,&i, 1);
		m=0x01&i;
		//if(m){pin1=true;}
	}
	return i;
}
void one_wire_reset(I2C_XFER_T& xf)
{
	xf.slaveAddr=DSADD;
	/*xf.rxBuff=one_wire_reset_rx;
	xf.rxSz=sizeof(one_wire_reset_rx);*/
	Chip_I2C_MasterCmdRead_cpp( I2C0, xf.slaveAddr, ONEWIRE_RESET, NULL, 0);
}
void send_data(I2C_XFER_T& xf,const uint8_t data )
{
	one_wire_dev_status(xf);
	uint8_t tb[]={ONEWIRE_WRITE_BYTE,data};
	xf.txBuff=tb;
	xf.txSz=sizeof(tb);
	Chip_I2C_MasterSend_cpp(I2C0, xf.slaveAddr,xf.txBuff, xf.txSz);
	one_wire_dev_status(xf);
}
void select_one_wire(I2C_XFER_T& xf)
{
	xf.slaveAddr=DSADD;
	/*xf.txBuff=one_wire_select;
	xf.txSz=sizeof(one_wire_select);
	Chip_I2C_MasterSend_cpp(I2C0, xf.slaveAddr,xf.txBuff, xf.txSz);*/
	send_data(xf,SKIP_ROM_COMMAND);
}
void start_cov(I2C_XFER_T& xf)
{
	xf.slaveAddr=DSADD;
	send_data(xf,START_CONVESION);
	/*xf.txBuff=start_conversion;
	xf.txSz=sizeof(start_conversion);
	Chip_I2C_MasterSend_cpp(I2C0, xf.slaveAddr,xf.txBuff, xf.txSz);*/
}
void read_data(I2C_XFER_T& xf)
{
	xf.slaveAddr=DSADD;
	xf.txBuff=read_data_reg_tx;
	xf.txSz=sizeof(read_data_reg_tx);
	Chip_I2C_MasterSend_cpp(I2C0, xf.slaveAddr,xf.txBuff, xf.txSz);
}
void one_wire_read(I2C_XFER_T& xf)
{
	xf.slaveAddr=DSADD;
	xf.rxBuff=one_wire_reset_rx;
	xf.rxSz=sizeof(one_wire_reset_rx);
	Chip_I2C_MasterCmdRead_cpp( I2C0, xf.slaveAddr, ONEWIRE_READ_BYTE,  &(xf.rxBuff[1]), 1);
	//send_data(xf,ONEWIRE_READ_BYTE);
}

void write_scratch(I2C_XFER_T& xf)
{
	uint8_t th=43, tl=11,config=0x5F;
	uint8_t buf[]={th,tl,config};
	send_data(xf,0x4E);//write scratchpad
	send_data(xf,th);
	send_data(xf,tl);
	send_data(xf,config); // all data transmissions are verified
}


void read_scratch(I2C_XFER_T& xf)
{
	xf.slaveAddr=DSADD;
	send_data(xf,READ_SCRATCH_PAD);
	/*xf.txBuff=read_scratch_pad;
	xf.txSz=sizeof(read_scratch_pad);
	Chip_I2C_MasterSend_cpp(I2C0, xf.slaveAddr,xf.txBuff, xf.txSz);*/
}

/*void read_temp(I2C_XFER_T& xf)
{
	int i;
	uint8_t n,u[9];
	one_wire_reset(xf);
	xf.slaveAddr=DSADD;
	select_one_wire(xf);
	for (i=0;i<9;i++)
	{
		one_wire_read(xf);
		Chip_I2C_MasterRead(I2C0,xf.slaveAddr,&n, 1);
		slave_data_tx[i]=n;
	}
	xf.slaveAddr=DSADD;
	xf.txBuff=read_temp_data_tx;
	xf.txSz=sizeof(read_temp_data_tx);
	xf.rxBuff=slave_data_tx;
	xf.rxSz=sizeof(slave_data_tx);
	Chip_I2C_MasterTransfer_cpp(I2C0, &xf);
}*/
void exec_scratch(I2C_XFER_T& xf)
{
	one_wire_reset(xf);
	xf.slaveAddr=DSADD;
	select_one_wire(xf);
	start_cov(xf);
	read_data(xf);
	uint8_t i=0xFF;
/*while(!(i==0x00))
	{
		one_wire_read(xf);
		Chip_I2C_MasterRead(I2C0,xf.slaveAddr,&i, 1);
	}*/
}

void write_scratchblock(I2C_XFER_T& xf)
{
	one_wire_reset(xf);
	xf.slaveAddr=DSADD;
	select_one_wire(xf);
	write_scratch(xf);
}
bool exec_temp(I2C_XFER_T& xf,int& r)
{
	volatile int i;bool output=true;xSemaphoreHandle   mu=0;
	bool pinx;
	uint8_t n,u[9];
	one_wire_reset(xf);
	xf.slaveAddr=DSADD;
	pinx=false;
	select_one_wire(xf);
	read_scratch(xf);
	//one_wire_read(xf);
	//read_data(xf);
	for (i=0;i<9;i++)
	{
		one_wire_read(xf);
		read_data(xf);
		Chip_I2C_MasterRead(I2C0,xf.slaveAddr,&n, 1);
		slave_data_tx[i]=n;

	}
	uint16_t slave_data16=slave_data_tx[1]<<8;
	slave_data16=slave_data16|slave_data_tx[0];
if (slave_data_tx[0]>0xD8){
	pinx=true;
	int i;
}
return pinx;
}

bool checkrx(void)
{
	return ((slave_data_rx[1]==0x55)|(slave_data_rx[9]==0x55)|(slave_data_rx[10]==0x55)|(slave_data_rx[11]==0x55)|(slave_data_rx[12]==0x55)|(slave_data_rx[4]==0x55)|(slave_data_rx[13]==0x55)|(slave_data_rx[6]==0x55)|(slave_data_rx[7]==0x55)|(slave_data_rx[8]==0x55));
}
