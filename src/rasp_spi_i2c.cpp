/*
===============================================================================
 Name        : Tarak0.cpp
 Author      : Tarak Patel
 Version     :0.0.1
 Copyright   : GPL
 Description : RTOS program which comunicate with raspberry  pi
  	  	  	   via spi bus and reads temprature value by i2c sensor
===============================================================================
 */

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif



#include "portmacro.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "list.h"

#include "project_cfg.h"
#include "gpiopin.hh"

#include "spi.hh"
#include "spiclass.h"

#include "i2c.h"
#include "i2cclass.h"


//#define SPI_MODE

extern "C"{
void I2C1_IRQHandler(void)
{
	i2c_state_handling_cpp(I2C1);
}

/**
 * @brief	I2C0 Interrupt handler
 * @return	None
 */
void I2C0_IRQHandler(void)
{
	i2c_state_handling_cpp(I2C0);
}
}

//static xLIST list;




bool input=false;
bool output=true;
int j;
int a =0;
static I2C_XFER_T xfer,xf;
//static uint8_t tx_buff1[]={0x00,0x00,0x00,0x00};
#if defined(SPI_MODE)
static SPI_DATA_SETUP_T spi_main_xf;
static int spi_xfer_completed=0;
#endif

#if defined(FREERTOS_VAR)
xSemaphoreHandle   mu=0;
xSemaphoreHandle   mu1=0;
xSemaphoreHandle   muf=0;
xSemaphoreHandle   the_signal=0;
xSemaphoreHandle   the_signal2=0;
xQueueHandle       queue_handle = 0;
#endif

///////////*******************All the functions**********************///////////


/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();
	Board_LED_Set(0, false);
}

#if defined(I2C_MODE)
static void i2c_setup(I2C_XFER_T& xfer)
{
	i2c_block_init_cpp(I2C0, SPEED_100KHZ);
	i2c_block_init_cpp(I2C1, SPEED_100KHZ);
	i2c_client_init_cpp(I2C0);
	i2c_rw_input_cpp(&xfer, 1);
}
#endif

#if defined(SPI_MODE)
static void spi_setup(void)
{

	Board_SPI_Init(false);
	Chip_SPI_Init(LPC_SPI);
	NVIC_EnableIRQ(SPI_IRQn);

}
#endif


/* LED1 toggle thread */
static void blink(void *pvParameters) {


	gpio_pin_port pin1(LPC_GPIO ,0,2,output,&mu);
	gpio_pin_port pin2(LPC_GPIO ,0,3,output,&mu);
	int b;
	bool pin1_status=true;

	while (1) {
		//if (xSemaphoreTake(mu1,1000))
		{
			//if(pin_op1) {pin_op1=false;*pin_output1=true;*pin_output2=false;} else {pin_op1=true;*pin_output1=false;*pin_output2=true;};
			if(pin1_status){pin1_status=false;pin1=true;pin2=false;}else{pin1_status=true;pin1=false;pin2=true;};
			//xSemaphoreGive(mu1);
		}
		/* About a 3Hz on/off toggle rate */
		xQueueReceive(queue_handle,&b,10);
		vTaskDelay(configTICK_RATE_HZ /b);
	}
}
static void blink2(void *pvParameters)
{
	gpio_pin_port ledr( LPC_GPIO ,0,22,output,&muf);
	gpio_pin_port ledg( LPC_GPIO ,3,25,output,&muf);
	gpio_pin_port ledb( LPC_GPIO ,3,26,output,&muf);

	while (1)
	{
		if (xSemaphoreTake(mu,1000))
		{
			/* blinks led in cycle*/
			if (checkrx()){a=1;}
			if ((a==1)){
				if (ledg && ledb){ledg=false;ledb=true;ledr=true;}else{if(ledr && ledb){ledb=false;ledr=true;ledg=true;}else{ledr=false;ledg=true;ledb=true;}}}
			xSemaphoreGive(mu);
		}
		vTaskDelay(configTICK_RATE_HZ/2);
	}
}
/* LED2 toggle thread */

static void bottonreadandrelay(void *pvParameters) {


	gpio_pin_port pin_input1( LPC_GPIO ,1,26,input,&muf);
	gpio_pin_port pin_input2( LPC_GPIO ,1,25,input,&muf);

	while (1) {

		//if (xSemaphoreTake(mu1,1000))
		{
			if(pin_input1)
			{
				int o=4;
				xQueueSend(queue_handle,&o,4000);
			}
			if(pin_input2)
			{
				int v=6;
				xQueueSend(queue_handle,&v,4000);
			}
			else
			{
				int h=1;
				xQueueSend(queue_handle,&h,4000);
			}
		}
		// xSemaphoreGive(mu);
		/* About a 7Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ/2);
	}
}
static void onewire_data(void *pvParameters)
{
	int i;
	uint8_t addr1= 0x1B;
	one_wire_dev_init(xf,addr1);

//	write_scratchblock(xf);
	gpio_pin_port pinx(LPC_GPIO ,2,12,output,&mu);
	gpio_pin_port piny(LPC_GPIO ,2,11,output,&mu);
	pinx=false;
	while (1)
	{
		if (checkrx()&&!(checkrx44())){a=1;}
		if (a==1){piny=false;}
		if (checkrx44()){a=0;}
		if (a==0){piny=true;}
		exec_scratch(xf,addr1);
		pinx=exec_temp(xf,i,addr1);
		vTaskDelay(configTICK_RATE_HZ/2);
	}
}
static void i2c_data(void *pvParameters)
{

	int o,p;

	while (1)
	{
		Chip_I2C_MasterCmdRead_cpp( I2C0, xfer.slaveAddr, 0x32,  &(xfer.rxBuff[3]), 1);
		Chip_I2C_MasterCmdRead_cpp( I2C0, xfer.slaveAddr, 0x33,  &(xfer.rxBuff[4]), 1);
		Chip_I2C_MasterCmdRead_cpp( I2C0, xfer.slaveAddr, 0x34,  &(xfer.rxBuff[5]), 1);
		Chip_I2C_MasterCmdRead_cpp( I2C0, xfer.slaveAddr, 0x35,  &(xfer.rxBuff[6]), 1);
		Chip_I2C_MasterCmdRead_cpp( I2C0, xfer.slaveAddr, 0x36,  &(xfer.rxBuff[7]), 1);
		Chip_I2C_MasterCmdRead_cpp( I2C0, xfer.slaveAddr, 0x37,  &(xfer.rxBuff[8]), 1);
		/*Chip_I2C_MasterCmdRead_cpp( I2C0, xfer.slaveAddr, 0x35,  xfer.rxBuff, xfer.rxSz);
		Chip_I2C_MasterCmdRead_cpp( I2C0, xfer.slaveAddr, 0x36,  xfer.rxBuff, xfer.rxSz);*/
		//i2c_client_events_cpp(I2C0,I2C_EVENT_SLAVE_TX );
		vTaskDelay(configTICK_RATE_HZ / 4);
	}
}


/* UART (or output) thread */
static void vUARTTask(void *pvParameters) {
	int tickCnt = 0;

	while (1) {
		DEBUGOUT("Tick: %d\r\n", tickCnt);
		tickCnt++;

		/* About a 1s delay here */
		vTaskDelay(configTICK_RATE_HZ);
	}
}
void setup()
{

	Chip_GPIO_WriteDirBit(LPC_GPIO, 3, 26, true);
	xTaskCreate(blink2, (signed char *) "blink2",
			configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
			(xTaskHandle *) NULL);
		xTaskCreate(bottonreadandrelay, (signed char *) "bottonreadandrelay",
			configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
			(xTaskHandle *) NULL);
	xTaskCreate(onewire_data,(signed char *) "TX/RX function",
			configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
			(xTaskHandle *) NULL);
	/*xTaskCreate(i2c_data, (signed char *) "TX/RX function",
			configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
			(xTaskHandle *) NULL);*/
	xTaskCreate(blink, (signed char *) "blink",
			configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
			(xTaskHandle *) NULL);
	/* UART output thread, simply counts seconds */
	xTaskCreate(vUARTTask, (signed char *) "vTaskUart",
			configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
			(xTaskHandle *) NULL);

}
/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
#if defined FREERTOS_VAR
static void var_init(void)
{
	mu = xSemaphoreCreateMutex();
	mu1 = xSemaphoreCreateMutex();
	vSemaphoreCreateBinary(the_signal);
	vSemaphoreCreateBinary(the_signal2);
	queue_handle = xQueueCreate(3,sizeof(int));
}
#endif

int main()
{

	prvSetupHardware();
	i2c_setup(xfer);
	var_init();
	//Chip_I2C_MasterCmdRead_cpp( I2C0, xfer.slaveAddr, 0x00,  xfer.rxBuff, xfer.rxSz); //TODO:move this line to somewhere in task of FREERTOS
	setup();
	//uint8_t buff[]={0x2d,0x08};

	Chip_I2C_MasterSend_cpp(I2C0, xfer.slaveAddr,xfer.txBuff, xfer.txSz);
	Chip_I2C_MasterCmdRead_cpp( I2C0, xfer.slaveAddr, 0x00,  &(xfer.rxBuff[1]), 1);
	Chip_I2C_MasterCmdRead_cpp( I2C0, xfer.slaveAddr, 0x2D,  &(xfer.rxBuff[2]), 1);

	vTaskStartScheduler();
	//while (1) {if (spi_xfer_completed) { spi_xfer_completed=0;appSPIRun();}}

	return 1;

}



/**
 * @}
 */
