/*
 * spiclass.cpp
 *
 *  Created on: Jul 10, 2018
 *      Author: tarak
 */

#include "spiclass.h"
#include "spi.hh"
#include "list.h"
#include "portmacro.h"

spi_class::spi_class(LPC_SPI_T* pSPI,SPI_BITS_T BITLN,SPI_CLOCK_MODE_T CLKM,SPI_DATA_ORDER_T DA)
{


	SPI_CONFIG_FORMAT_T spiformat;
	spiformat.bits = SPI_BITS_8;
	spiformat.clockMode = SPI_CLOCK_MODE0;
	spiformat.dataOrder = SPI_DATA_MSB_FIRST;
	Chip_SPI_SetFormat(LPC_SPI, &spiformat);
	this->spidev.spi_format=spiformat;


	SPI_DATA_SETUP_T transfer;
	transfer.fnBefFrame =  NULL;
	transfer.fnAftFrame =  NULL;
	transfer.fnBefTransfer = NULL;
	transfer.fnAftTransfer = NULL;
	this->spidev.spi_xf=transfer;






	this->spidev.pSPI0 =pSPI;
}

spi_class::~spi_class() {
	// TODO Auto-generated destructor stub
}
void spi_class::spiRW(SPI_DATA_SETUP_T& spi_txr,int& status)
{
	spi_txr.cnt = 0;
	spi_txr.length = BUFFER_SIZE;
	spi_txr.pTxData = this->spi_tx_buf;
	spi_txr.pRxData = this->spi_rx_buf;
	spi_txr.fnBefFrame =  NULL;
	spi_txr.fnAftFrame =  NULL;
	spi_txr.fnBefTransfer = NULL;
	spi_txr.fnAftTransfer = NULL;
	//bufferInit(this->spi_tx_buf,this->spi_rx_buf);
	status=0;
	Chip_SPI_Int_FlushData(LPC_SPI);

	Chip_SPI_Int_RWFrames8Bits(this->spidev.pSPI0, &(this->spidev.spi_xf));
	Chip_SPI_Int_Enable(LPC_SPI);

}
