/*
 * spi.hh
 *
 *  Created on: Jul 10, 2018
 *      Author: tarak
 */


#include "board.h"
#include "stdio.h"
#include "list.h"
#include "portmacro.h"


#define BUFFER_SIZE                         (0x40)

#define SPI_MASTER_MODE_SEL                 (0x31)
#define SPI_SLAVE_MODE_SEL                  (0x32)
#define SPI_MODE_SEL                        (SPI_SLAVE_MODE_SEL)

#define SPI_POLLING_SEL                     (0x31)
#define SPI_INTERRUPT_SEL                   (0x32)
#if (SPI_MODE_SEL == SPI_SLAVE_MODE_SEL)
#define SPI_TRANSFER_MODE_SEL               (SSPI_INTERRUPT_SEL)
#else
#define SPI_TRANSFER_MODE_SEL               (SPI_POLLING_SEL)
#endif

#define LPC_SSP                             LPC_SSP1
#define LPC_GPDMA_SSP_TX                    GPDMA_CONN_SSP1_Tx
#define LPC_GPDMA_SSP_RX                    GPDMA_CONN_SSP1_Rx






//void bufferInit(uint8_t*, uint8_t*);
void SPI_IRQHandler(void);

