#include "FreeRTOS.h"
#include "board.h"
#include "semphr.h"

#define LPC_GPIO                  ((LPC_GPIO_T             *) LPC_GPIO0_BASE)
#define LPC_GPIO1                 ((LPC_GPIO_T             *) LPC_GPIO1_BASE)
#define LPC_GPIO2                 ((LPC_GPIO_T             *) LPC_GPIO2_BASE)
#define LPC_GPIO3                 ((LPC_GPIO_T             *) LPC_GPIO3_BASE)
#define LPC_GPIO4                 ((LPC_GPIO_T             *) LPC_GPIO4_BASE)



class lock{
private:
	xSemaphoreHandle*   mu2;

public:
	 lock(xSemaphoreHandle  *muf)
{

		 if(xSemaphoreTake(muf,1000)){}mu2=muf;
}
	 ~lock()
	 {
		 xSemaphoreGive(mu2);
	 }
};


class gpio_pin_port {
public:
	 gpio_pin_port(LPC_GPIO_T* base,int port,int pin, bool direction,xSemaphoreHandle *mu);
	 gpio_pin_port();
	 ~gpio_pin_port();

	void gpio_pin_set(gpio_pin_port* d );
	void gpio_pin_reset(gpio_pin_port* d);
	bool gpio_pin_read(gpio_pin_port* d);
	gpio_pin_port& operator =( const bool &b){if(b==true){this->gpio_pin_set(this);}else{this->gpio_pin_reset(this);}return *this;}
    bool operator = ( gpio_pin_port& st ){return st.gpio_pin_read(this);}
    operator bool ()  {return this->gpio_pin_read(this);}

    //gpio_pin_port& operator = ( bool b){if(b){this->gpio_pin_set(gpio_base, gpio_pin, gpio_port,this);}else{this->gpio_pin_reset(gpio_base, gpio_pin, gpio_port,this);}return *this;}
    private: uint8_t  gpio_pin;uint32_t gpio_port;LPC_GPIO_T* gpio_base; bool dir; xSemaphoreHandle *mu3;
	};

