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



// HANDLER PRZERWANIA PO WYSŁANIU
void DMA1_Stream6_IRQHandler() {
	/* Odczytaj zgłoszone przerwania DMA1. */
	uint32_t isr = DMA1->HISR;
	if (isr & DMA_HISR_TCIF6) {
		/* Obsłuż zakończenie transferu
		w strumieniu 6. */
		DMA1->HIFCR = DMA_HIFCR_CTCIF6;

		RedLEDon();
	}
// Uwaga: zakończenie transferu DMA nie oznacza, że UART
// zakończył wysyłanie
}


void DMA1_Stream5_IRQHandler() {
	/* Odczytaj zgłoszone przerwania DMA1. */
	uint32_t isr = DMA1->HISR;

	if (isr & DMA_HISR_TCIF5) {
		/* Obsłuż zakończenie transferu
		w strumieniu 5. */
		DMA1->HIFCR = DMA_HIFCR_CTCIF5;
		RedLEDoff();
		BlueLEDon();
		Green2LEDon();
	/* Ponownie uaktywnij odbieranie. */
	}

}


void DMAconfig() {
	GPIOafConfigure(GPIOA,
	2,
	GPIO_OType_PP,
	GPIO_Fast_Speed,
	GPIO_PuPd_NOPULL,
	GPIO_AF_USART2);

	GPIOafConfigure(GPIOA,
	3,
	GPIO_OType_PP,
	GPIO_Fast_Speed,
	GPIO_PuPd_UP,
	GPIO_AF_USART2);
}


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


#define USER_INT_PRIO LOW_IRQ_PRIO
#define USER_INT_SUBPRIO LOW_IRQ_SUBPRIO
void UserButInterruptPrioritiesSetup() {
	IRQsetPriority(EXTI15_10_IRQn, USER_INT_PRIO, USER_INT_SUBPRIO);
}

void UserButInterruptEXTIConfig() {

	GPIOinConfigure(GPIOC, 
	USER_BUT_PIN, // 13
	GPIO_PuPd_NOPULL,
	EXTI_Mode_Interrupt,
	EXTI_Trigger_Falling);


	EXTI->PR = 1 << USER_BUT_PIN; // zerowanie rejestru
}

void EXTI15_10_IRQHandler(void) {
	EXTI->PR = EXTI_PR_PR13;
/* Tu wstaw kod obsługujący przerwanie. */
	BlueLEDon();
}


int main() {

	// NOWE: włączenie PA, DMA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOBEN |  
	RCC_AHB1ENR_DMA1EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	// Trzeba pamiętać o uprzednim włączeniu taktowania układu SYSCFG
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;


	    __NOP();



	//DMAconfig();

	LedsConfig();

	UserButInterruptEXTIConfig();

	// UserButInterruptPrioritiesSetup();

	NVIC_EnableIRQ(EXTI15_10_IRQn);

	for(;;);
	uint32_t const baudrate = 9600U;
	// USART
	USART2->CR1 = USART_CR1_RE | USART_CR1_TE;
	USART2->CR2 = 0;
	USART2->BRR = (PCLK1_HZ + (baudrate / 2U)) / baudrate;

	// DMA
	USART2->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;


/*
USART2 TX korzysta ze strumienia 6 i kanału 4, tryb
bezpośredni, transfery 8-bitowe, wysoki priorytet, zwiększanie
adresu pamięci po każdym przesłaniu, przerwanie po
zakończeniu transmisji
*/
DMA1_Stream6->CR = 4U << 25 |
DMA_SxCR_PL_1 |
DMA_SxCR_MINC |
DMA_SxCR_DIR_0 |
DMA_SxCR_TCIE;

// Adres układu peryferyjnego nie zmienia się
DMA1_Stream6->PAR = (uint32_t)&USART2->DR;

/*
USART2 RX korzysta ze strumienia 5 i kanału 4, tryb
bezpośredni, transfery 8-bitowe, wysoki priorytet, zwiększanie
adresu pamięci po każdym przesłaniu, przerwanie po
zakończeniu transmisji
*/
DMA1_Stream5->CR = 4U << 25 |
DMA_SxCR_PL_1 |
DMA_SxCR_MINC |
DMA_SxCR_TCIE;

// Adres ukladu per. sie nie zmienia
DMA1_Stream5->PAR = (uint32_t)&USART2->DR;


// Wyczyść znaczniki przerwań i włącz przerwania
DMA1->HIFCR = DMA_HIFCR_CTCIF6 |
DMA_HIFCR_CTCIF5;
NVIC_EnableIRQ(DMA1_Stream6_IRQn);
NVIC_EnableIRQ(DMA1_Stream5_IRQn);


// ENABLE
USART2->CR1 |= USART_CR1_UE;


/*
Inicjowanie wysyłania:
DMA1_Stream6->M0AR = (uint32_t)buff;
DMA1_Stream6->NDTR = len;
DMA1_Stream6->CR |= DMA_SxCR_EN;


Inicjowanie odbierania:
DMA1_Stream5->M0AR = (uint32_t)buff;
DMA1_Stream5->NDTR = 1;
DMA1_Stream5->CR |= DMA_SxCR_EN;

*/




unsigned int BUFSIZE = 8000;

char txbuf[BUFSIZE];
char rxbuf[BUFSIZE];

char* MODE_PRESSED = "MODE_PRESSED\r\n";
char* MODE_RELEASED = "MODE_RELEASED\r\n";

char* USER_PRESSED = "USER_PRESSED\r\n";
char* USER_RELEASED = "USER_RELEASED\r\n";


unsigned int MODE_RELEASED_LEN = strlen(MODE_RELEASED);
unsigned int MODE_PRESSED_LEN = strlen(MODE_PRESSED);

unsigned int USER_RELEASED_LEN = strlen(USER_RELEASED);
unsigned int USER_PRESSED_LEN = strlen(USER_PRESSED);

char* LED1ON = "LED1ON";
char* LED1OFF = "LED1OFF";
char* LED1TOG = "LED1TOG";

char* LED2ON = "LED2ON";
char* LED2OFF = "LED2OFF";


int rxi = 0;

int txmin = 0, txmax = 0;


int mode_was_enabled = 0; // poporzednio zaobserwowany stan
int user_was_enabled = 0;

int blue_enabled = 0;

for(;;);



//
// DALEJ NA RAZIE NIC NIE MA
//



/*
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
	}
	if(mode_but_changed_state(mode_was_enabled)) {
		char* dest = txbuf + txmax;
		if (mode_was_enabled) {
			strcpy(dest, MODE_RELEASED);
			txmax += MODE_RELEASED_LEN;
		} else {
			strcpy(dest, MODE_PRESSED);
			txmax += MODE_PRESSED_LEN;
		}
		mode_was_enabled = 1 - mode_was_enabled; // change recently observed state
	}
	if(user_but_changed_state(user_was_enabled)) {
		char* dest = txbuf + txmax;
		if (user_was_enabled) {
			strcpy(dest, USER_RELEASED);
			txmax += USER_RELEASED_LEN;
		} else {
			strcpy(dest, USER_PRESSED);
			txmax += USER_PRESSED_LEN;
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
*/

/*
DMA1_Stream6->M0AR = (uint32_t)LED2ON;
DMA1_Stream6->NDTR = 4;
DMA1_Stream6->CR |= DMA_SxCR_EN;

char RES[5];
DMA1_Stream5->M0AR = (uint32_t)RES;
DMA1_Stream5->NDTR = 1;
DMA1_Stream5->CR |= DMA_SxCR_EN;
*/

for(;;);

} // main




