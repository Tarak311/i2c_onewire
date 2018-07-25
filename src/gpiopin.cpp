/*
 * gpiopin.cpp
 *
 *  Created on: Mar 19, 2018
 *      Author: tarak
 */
#include "FreeRTOS.h"
#include "gpiopin.hh"
#include "board.h"
#include "semphr.h"

gpio_pin_port::gpio_pin_port(LPC_GPIO_T* base,int port,int pin,bool direction,xSemaphoreHandle *mu)
{
mu3=mu;
gpio_pin=pin;
gpio_base=base;
gpio_port=port;
dir = direction;

Chip_GPIO_WriteDirBit(LPC_GPIO, gpio_port, gpio_pin, dir);
this->gpio_pin_set(this);

}
gpio_pin_port::~gpio_pin_port()
{

}
 bool set=1;
 bool clear=0;
void gpio_pin_port::gpio_pin_set(gpio_pin_port* d){Chip_GPIO_WritePortBit(d->gpio_base,d->gpio_port,d->gpio_pin,set);}
void gpio_pin_port::gpio_pin_reset(gpio_pin_port* d){Chip_GPIO_WritePortBit(d->gpio_base,d->gpio_port,d->gpio_pin,clear);}
bool gpio_pin_port::gpio_pin_read(gpio_pin_port* d){return Chip_GPIO_ReadPortBit(d->gpio_base,d->gpio_port,d->gpio_pin);}

