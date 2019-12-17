#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include <string.h>

#include <irq.h>



#define RED_LED_GPIO GPIOA
#define GREEN_LED_GPIO GPIOA
#define BLUE_LED_GPIO GPIOB
#define GREEN2_LED_GPIO GPIOA

#define RED_LED_PIN 6
#define GREEN_LED_PIN 7
#define BLUE_LED_PIN 0
#define GREEN2_LED_PIN 5

#define RedLEDon() \
RED_LED_GPIO->BSRR = 1 << (RED_LED_PIN + 16)
#define RedLEDoff() \
RED_LED_GPIO->BSRR = 1 << RED_LED_PIN


#define GreenLEDon() \
GREEN_LED_GPIO->BSRR = 1 << (16 + GREEN_LED_PIN)
#define GreenLEDoff() \
GREEN_LED_GPIO->BSRR = 1 << GREEN_LED_PIN


#define BlueLEDon() \
BLUE_LED_GPIO->BSRR = 1 << (16 + BLUE_LED_PIN)
#define BlueLEDoff() \
BLUE_LED_GPIO->BSRR = 1 << BLUE_LED_PIN


// wlaczana jedynka
#define Green2LEDon() \
GREEN2_LED_GPIO->BSRR = 1 << GREEN2_LED_PIN
#define Green2LEDoff() \
GREEN2_LED_GPIO->BSRR = 1 << (GREEN2_LED_PIN + 16)



void LedsConfig() {

	RedLEDoff();
	GreenLEDoff();
	BlueLEDoff();
	Green2LEDoff();

	GPIOoutConfigure(RED_LED_GPIO,
	RED_LED_PIN,
	GPIO_OType_PP,
	GPIO_Low_Speed,	
	GPIO_PuPd_NOPULL);

	GPIOoutConfigure(GREEN_LED_GPIO,
	GREEN_LED_PIN,
	GPIO_OType_PP,
	GPIO_Low_Speed,
	GPIO_PuPd_NOPULL);

	GPIOoutConfigure(BLUE_LED_GPIO,
	BLUE_LED_PIN,
	GPIO_OType_PP,
	GPIO_Low_Speed,
	GPIO_PuPd_NOPULL);

	GPIOoutConfigure(GREEN2_LED_GPIO,
	GREEN2_LED_PIN,
	GPIO_OType_PP,
	GPIO_Low_Speed,
	GPIO_PuPd_NOPULL);

}


