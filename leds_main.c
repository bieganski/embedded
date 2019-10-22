#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include <string.h>



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


#define MODE_BUT_PIN 0
#define MODE_BUT_GPIO GPIOA
int mode_but_enabled() {
	return ( MODE_BUT_GPIO->IDR & (1 << MODE_BUT_PIN) ) == 1;
}


#define USER_BUT_PIN 13
#define USER_BUT_GPIO GPIOC
int user_but_enabled() {
	return ( USER_BUT_GPIO->IDR & (1 << USER_BUT_PIN) ) == 0;
}


// Tryb pracy
#define USART_Mode_Rx_Tx (USART_CR1_RE | \
	USART_CR1_TE)
#define USART_Enable USART_CR1_UE
// Przesyłane słowo to dane łącznie z ewentualnym bitem parzystości
#define USART_WordLength_8b 0x0000
#define USART_WordLength_9b USART_CR1_M
// Bit parzystości
#define USART_Parity_No 0x0000
#define USART_Parity_Even USART_CR1_PCE
#define USART_Parity_Odd (USART_CR1_PCE | \
	USART_CR1_PS)


#define USART_StopBits_1 0x0000
#define USART_StopBits_0_5 0x1000
#define USART_StopBits_2 0x2000
#define USART_StopBits_1_5 0x3000


#define USART_FlowControl_None 0x0000
#define USART_FlowControl_RTS USART_CR3_RTSE
#define USART_FlowControl_CTS USART_CR3_CTSE

#define HSI_HZ 16000000U
#define PCLK1_HZ HSI_HZ



int mozna_wyslac() {
	return USART2->SR & USART_SR_TXE;
}

void wyslij(char c) {
	USART2->DR = c;
}

int mozna_odebrac() {
	return USART2->SR & USART_SR_RXNE;
}

char odbierz() {
	return USART2->DR;
}



int mode_but_changed_state(int was_enabled);
int user_but_changed_state(int was_enabled);

int main() {

// Włączenie taktowania portów PA i PB (układy GPIOA i GPIOB)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
    RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;



    __NOP();



//  Konfigurujemy linię TXD
GPIOafConfigure(GPIOA,
2,
GPIO_OType_PP,
GPIO_Fast_Speed,
GPIO_PuPd_NOPULL,
GPIO_AF_USART2);
// Konfigurujemy linię RXD
GPIOafConfigure(GPIOA,
3,
GPIO_OType_PP,
GPIO_Fast_Speed,
GPIO_PuPd_UP,
GPIO_AF_USART2);



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


USART2->CR1 = USART_Mode_Rx_Tx |
	USART_WordLength_8b |
	USART_Parity_No;
USART2->CR2 = USART_StopBits_1;
USART2->CR3 = USART_FlowControl_None;

uint32_t const baudrate = 9600U;
USART2->BRR = (PCLK1_HZ + (baudrate / 2U)) /
baudrate;




// ENABLE
USART2->CR1 |= USART_Enable;


    RedLEDoff();
    GreenLEDoff();
    BlueLEDoff();
    Green2LEDoff();

char txbuf[8000];
char rxbuf[8000];

char* MODE_PRESSED = "MODE_PRESSED\r\n";
char* MODE_RELEASED = "MODE_RELEASED\r\n";

char* LED1ON = "LED1ON";
char* LED1OFF = "LED1OFF";
char* LED1TOG = "LED1TOG";

char* USER_PRESSED = "USER_PRESSED\r\n";
char* USER_RELEASED = "USER_RELEASED\r\n";

char* LED2ON = "LED2ON";
char* LED2OFF = "LED2OFF";


int rxi = 0;

int txmin = 0, txmax = 0;


/*
for(;;)
	if (niepusty tx buf ^ txne == 1)
		send byte from tx buf;
	if (rxe == 1)
		receive byte and save it in rx buffer
	if (key pressed of released)
		put msg to tx buf
	if (rx buf contains command)
		turn led on/off
*/

int mode_was_enabled = 0; // poporzednio zaobserwowany stan
int user_was_enabled = 0;

int printed = 0; // DEBUG

int blue_enabled = 0;

for(;;) {
	if (txmin == txmax) {
		txmin = 0;
		txmax = 0;
	}
	if(txmax > 0 && mozna_wyslac()) {
		wyslij(txbuf[txmin]);
		txmin++;
	}
	if(mozna_odebrac()) {
		rxbuf[rxi] = odbierz();
		rxi++;
		printed = 0;
	}
	if(mode_but_changed_state(mode_was_enabled)) {
		char* dest = txbuf + txmax;
		if (mode_was_enabled) {
			strcpy(dest, MODE_RELEASED);
			txmax += strlen(MODE_RELEASED);
		} else {
			strcpy(dest, MODE_PRESSED);
			txmax += strlen(MODE_PRESSED);
		}
		mode_was_enabled = 1 - mode_was_enabled; // change recently observed state
	}
	if(user_but_changed_state(user_was_enabled)) {
		char* dest = txbuf + txmax;
		if (user_was_enabled) {
			strcpy(dest, USER_RELEASED);
			txmax += strlen(USER_RELEASED);
		} else {
			strcpy(dest, USER_PRESSED);
			txmax += strlen(USER_PRESSED);
		}
		user_was_enabled = 1 - user_was_enabled; // change recently observed state
	}


	rxbuf[rxi] = '\0';
	if (0 == strcmp(LED1ON, rxbuf)) {
		BlueLEDon();
		blue_enabled = 1;
		rxi = 0;
	} else if (0 == strcmp(LED1OFF, rxbuf)) {
		BlueLEDoff();
		blue_enabled = 0;
		rxi = 0;
	} else if (0 == strcmp(LED2ON, rxbuf)) {
		Green2LEDon();
		rxi = 0;
	} else if (0 == strcmp(LED2OFF, rxbuf)) {
		Green2LEDoff();
		rxi = 0;
	} else if (0 == strcmp(LED1TOG, rxbuf)) {
		if (blue_enabled)
			BlueLEDoff();
		else
			BlueLEDon();
		blue_enabled = 1 - blue_enabled;
		rxi = 0;
	}
} // for(;;)

} // main

int mode_but_changed_state(int was_enabled) {
	int one = (!was_enabled) && mode_but_enabled();
	int two = was_enabled && (!mode_but_enabled());
	return one || two;
}

int user_but_changed_state(int was_enabled) {
	int one = (!was_enabled) && user_but_enabled();
	int two = was_enabled && (!user_but_enabled());
	return one || two;
}
