#ifndef __GPIOMACROS__
#define __GPIOMACROS__

//#include "stm32f30x.h"
#include "stm32f0xx.h"

#define GPIO_SET_OUTPUT(x,y) x->MODER &= ~(3<<((y)*2)); x->MODER |= (1<<((y)*2)); x->OSPEEDR|= ((3)<<((y)*2))
#define GPIO_SET_INPUT(x,y) x->MODER &= ~(3<<((y)*2)); x->OSPEEDR|= ((3)<<((y)*2))
#define GPIO_SET_SPEED(x,y,z) x->OSPEEDR|= ((z)<<((y)*2)) // 0 low, 1 medium, 3 high speed
#define GPIO_SET_AF(x,y,z) x->MODER &= ~(3<<((y)*2)); x->MODER |= (2<<((y)*2)); x->AFR[(y>>3)&1]|= ((z)<<((y&7)*4))
#define GPIO_SET_ANALOG(x,y) x->MODER |= (3<<((y)*2))
#define GPIO_SET_OTYPE(x, y, z) x->OTYPER &= ~(1<<(y)); x->OTYPER |= (z<<(y))
#define GPIO_SET_PUPDR(x, y, z) x->PUPDR &= ~(3<<(y*2)); x->PUPDR |= (z<<(y*2))
#define GPIO_SET_BIT(x, y) x->BSRR = (1<<(y))
#define GPIO_RESET_BIT(x, y) x->BRR = (1<<(y))
#define GPIO_READ_BIT(x, y) (x->IDR & (1<<(y)))

#define GPIO_SET_PUPUP(x,y) GPIO_SET_PUPDR(x,y, 1)
#define GPIO_SET_PUPDOWN(x,y) GPIO_SET_PUPDR(x,y, 2)
#define GPIO_SET_PUPDOWN2(x,y,z) GPIO_SET_PUPDR(x,y, z*0 + 1)
#define GPIO_SET_PUPUP2(x,y,z) GPIO_SET_PUPDR(x,y, z*0 + 2)



#define GPIO_SET_OUTPUT_(t) GPIO_SET_OUTPUT(t)
#define GPIO_SET_INPUT_(t) GPIO_SET_INPUT(t)
#define GPIO_SET_PUPUP_(t) GPIO_SET_PUPUP(t)
#define GPIO_SET_PUPDOWN_(t) GPIO_SET_PUPDOWN(t)

#define GPIO_SET_BIT_(t)  GPIO_SET_BIT(t) 
#define GPIO_RESET_BIT_(t) GPIO_RESET_BIT(t)
#define GPIO_READ_BIT_(t) GPIO_READ_BIT(t)
#define GPIO_SET_AF_(t,z) GPIO_SET_AF(t,z)
#define GPIO_SET_AF__(t) GPIO_SET_AF(t)
#define GPIO_SET_PUPDOWN__(t) GPIO_SET_PUPDOWN2(t)
#define GPIO_SET_PUPUP__(t) GPIO_SET_PUPUP2(t)

#endif