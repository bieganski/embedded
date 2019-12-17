#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include <string.h>

#include <irq.h>


#include "leds.h"
#include "debug.h"


// PB8, PB9
void i2c_config() {
    
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	GPIOafConfigure(GPIOB, 8, GPIO_OType_OD,
	GPIO_Low_Speed, GPIO_PuPd_NOPULL,
	GPIO_AF_I2C1);
	GPIOafConfigure(GPIOB, 9, GPIO_OType_OD,
	GPIO_Low_Speed, GPIO_PuPd_NOPULL,
	GPIO_AF_I2C1);
    
    
    //  Konfiguruj szynę w wersji podstawowej
    I2C1->CR1 = 0;
    
    // Konfiguruj częstotliwość taktowania szyny
    #define I2C_SPEED_HZ 100000
    #define PCLK1_MHZ 16
    I2C1->CR2 = PCLK1_MHZ;
    I2C1->CCR = (PCLK1_MHZ * 1000000) /
    (I2C_SPEED_HZ << 1);
    I2C1->TRISE = PCLK1_MHZ + 1;
    
    // Włącz szynę
    I2C1->CR1 |= I2C_CR1_PE;
}


#define LIS35DE_ADDR 0x1C
#define OUT_X 0x29
#define OUT_Y 0x2B
#define OUT_Z 0x2D

#define LIS35DE_CTRL_REG1 0x20
#define LIS35DE_CTRL_REG3 0x22
#define LIS35DE_PD_BIT 7


#define wait(cond) while(!(cond));


uint8_t MR_receive(char reg);

// #define LIS35DE_INT1_GPIO GPIOA
// #define LIS35DE_INT1_PIN 1

#define LIS35DE_INT2_GPIO GPIOA
#define LIS35DE_INT2_PIN 8


void EXTI9_5_IRQHandler(void) {
	EXTI->PR = EXTI_PR_PR8; // int2 

	uint8_t x,z,y;

 	do {
		x = MR_receive(OUT_X);
		y = MR_receive(OUT_Y);
		z = MR_receive(OUT_Z);
	}
	while( (LIS35DE_INT2_GPIO->IDR & (1 << LIS35DE_INT2_PIN)) != 0);

	if (x == 0)
		x = 0b00000001;

	if (y == 0)
		y = 0b00000001;

		TIM3->CCR1 = x;
		TIM3->CCR2 = y;
}


void LIS35DEwrite(int reg, int val) {
	I2C1->CR1 |= I2C_CR1_START;
	wait(I2C1->SR1 & I2C_SR1_SB);
	I2C1->DR = LIS35DE_ADDR << 1;

	wait(I2C1->SR1 & I2C_SR1_ADDR);

	I2C1->SR2;

	I2C1->DR = reg;

	wait(I2C1->SR1 & I2C_SR1_TXE);

	I2C1->DR = val;

	wait(I2C1->SR1 & I2C_SR1_BTF);

	I2C1->CR1 |= I2C_CR1_STOP;
}

void LIS35DEreset() {
	LIS35DEwrite(LIS35DE_CTRL_REG1, 0b00000000);
	LIS35DEwrite(LIS35DE_CTRL_REG3, 0b00000000);
}

void LIS35DEpowerOn() {
	LIS35DEwrite(LIS35DE_CTRL_REG1, 0b01000111); // power on, enable x,y,z reading
}

void LIS35DEenableIntAfterMeasured() {
	LIS35DEwrite(LIS35DE_CTRL_REG3, 0b10100000); // active low, push-pull, data ready, int2 line
}



void LIS35DE_GPIO_IntEnable() {
	GPIOinConfigure(LIS35DE_INT2_GPIO, 
	LIS35DE_INT2_PIN,
	GPIO_PuPd_NOPULL,
	EXTI_Mode_Interrupt,
	EXTI_Trigger_Falling);
}


// (master receive)
uint8_t MR_receive(char reg) {
    // Zainicjuj transmisję sygnału START
    I2C1->CR1 |= I2C_CR1_START;
    
    wait(I2C1->SR1 & I2C_SR1_SB);
    // START wysłano
    
    I2C1->DR = LIS35DE_ADDR << 1;
    
    wait(I2C1->SR1 & I2C_SR1_ADDR);
    // ADRES wysłano
    
    // skasuj bit ADDR
    I2C1->SR2;
    
    // Zainicjuj wysyłanie 8-bitowego numeru rejestru slave’a
    I2C1->DR = reg;
    
    wait(I2C1->SR1 & I2C_SR1_BTF);
    // dane wysłano (numer rejestru slave'a)
    
    // Zainicjuj transmisję sygnału REPEATED START
    I2C1->CR1 |= I2C_CR1_START;
    
    wait(I2C1->SR1 & I2C_SR1_SB);
    // START wysłano
    
    // tryb MR
    I2C1->DR = (LIS35DE_ADDR << 1) | 1U;

    
    I2C1->CR1 &= ~I2C_CR1_ACK;
    
    wait(I2C1->SR1 & I2C_SR1_ADDR);
    // zakończona transmisja adresu
    
    
    char nic = I2C1->SR2;
    
    // wyślij STOP
    I2C1->CR1 |= I2C_CR1_STOP;
    
    
    wait(I2C1->SR1 & I2C_SR1_RXNE);
    // ODEBRANO WYNIK
    uint8_t value = I2C1->DR;
    return value;
}

void CounterConfig() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->CR1 = 0; //  w górę

	TIM3->EGR = TIM_EGR_UG; // zainicjuj prescaler i auto reload

	TIM3->SR = ~TIM_SR_CC1IF; // TODO ~(TIM_SR_UIF | TIM_SR_CC1IF)

	TIM3->DIER = TIM_DIER_CC1IE; // TODO TIM_DIER_UIE | TIM_DIER_CC1IE;

	// przerwania
	NVIC_EnableIRQ(TIM3_IRQn);
}


void TIM3_IRQHandler(void) {
	uint32_t it_status = TIM3->SR & TIM3->DIER;
	if (it_status & TIM_SR_UIF) {
		TIM3->SR = ~TIM_SR_UIF;
	}

	if (it_status & TIM_SR_CC1IF) {
		TIM3->SR = ~TIM_SR_CC1IF;
	}
}

void counterToLed() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	GPIOafConfigure(GPIOA, 6, GPIO_OType_PP,
	GPIO_Low_Speed,
	GPIO_PuPd_NOPULL, GPIO_AF_TIM3); // czerwona

	GPIOafConfigure(GPIOA, 7, GPIO_OType_PP,
	GPIO_Low_Speed,
	GPIO_PuPd_NOPULL, GPIO_AF_TIM3); // zielona

/*
	GPIOafConfigure(GPIOB, 0, GPIO_OType_PP,
	GPIO_Low_Speed,
	GPIO_PuPd_NOPULL, GPIO_AF_TIM3); // niebieska
*/

	// 1,2
	TIM3->CCMR1 =
	TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 |  
	TIM_CCMR1_OC1PE |
	TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 |
	TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2PE;

/*
	// 3
	TIM3->CCMR2 =
	TIM_CCMR2_OC3M_2 | TIM_CCMR1_OC3M_1 |
	TIM_CCMR1_OC3PE
*/

	// 1,2
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P |
	TIM_CCER_CC2E | TIM_CCER_CC2P;

	// Włączamy licznik w trybie zliczania w górę z buforowaniem rejestru TIM3->ARR
	TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

	TIM3->PSC = 999;
	TIM3->ARR = 0xff;
	TIM3->EGR = TIM_EGR_UG;
	TIM3->CCR1 = 40;
	TIM3->CCR2 = 40;
}


void CounterEnable() {
	TIM3->CR1 |= TIM_CR1_CEN;
}	


int main() {
    	// Trzeba pamiętać o uprzednim włączeniu taktowania układu SYSCFG
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
	// włączenie GPIO, DMA, USART
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOBEN |  
	RCC_AHB1ENR_DMA1EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

 
	i2c_config();
	__NOP();


	LIS35DE_GPIO_IntEnable();

	LIS35DEreset();
	
	EXTI->PR = 1 << 8;
	NVIC_EnableIRQ(EXTI9_5_IRQn);

	LIS35DEpowerOn();
	LIS35DEenableIntAfterMeasured();

	DMAconfig();
	LedsConfig();
	DMA_USART_config();

	CounterConfig();
	CounterEnable();
	counterToLed();

	for(;;) {
	}

} // main




