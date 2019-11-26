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

	GPIOinConfigure(USER_BUT_GPIO, 
	USER_BUT_PIN,
	GPIO_PuPd_NOPULL,
	EXTI_Mode_Interrupt,
	EXTI_Trigger_Rising_Falling);


	EXTI->PR = 1 << USER_BUT_PIN;
}

void ModeButInterruptEXTIConfig() {
	GPIOinConfigure(MODE_BUT_GPIO, 
	MODE_BUT_PIN,
	GPIO_PuPd_NOPULL,
	EXTI_Mode_Interrupt,
	EXTI_Trigger_Rising_Falling);

	EXTI->PR = 1 << MODE_BUT_PIN;
}

uint32_t QUE_IDX_MIN = 0;
uint32_t QUE_IDX_MAX = 0;
#define QUE_SIZE 127
char* QUEUE[QUE_SIZE];


void send() {
    if (QUE_IDX_MAX == QUE_IDX_MIN)
        return; // nie ma nic do wysłania
    uint32_t idx = QUE_IDX_MIN % QUE_SIZE;
    if (QUEUE[idx] != NULL) {
        DMA1_Stream6->M0AR = (uint32_t)QUEUE[idx];
        DMA1_Stream6->NDTR = strlen(QUEUE[idx]);
        DMA1_Stream6->CR |= DMA_SxCR_EN;
        QUE_IDX_MIN++;
    }
}

void send_or_enqueue(char* text) {
    uint32_t idx = QUE_IDX_MAX % QUE_SIZE;
    QUEUE[idx] = text;
    QUE_IDX_MAX++;
    
    if ((DMA1_Stream6->CR & DMA_SxCR_EN) == 0 &&
        (DMA1->HISR & DMA_HISR_TCIF6) == 0) {
        send();
    } else {
        // nie można wysłać bo DMA zajęte
        return;
    }
}

// HANDLER PRZERWANIA PO WYSŁANIU
void DMA1_Stream6_IRQHandler() {
	/* Odczytaj zgłoszone przerwania DMA1. */
	uint32_t isr = DMA1->HISR;
	if (isr & DMA_HISR_TCIF6) {
		/* Obsłuż zakończenie transferu
		w strumieniu 6. */
		DMA1->HIFCR = DMA_HIFCR_CTCIF6;

        /* Jeśli jest coś do wysłania,
            wystartuj kolejną transmisję. */
        send();
	}
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





void EXTI15_10_IRQHandler(void) {
	EXTI->PR = EXTI_PR_PR13;
    /* Tu wstaw kod obsługujący przerwanie. */

    char USER_FIRED[] = "USER FIRED\r\n";
    char USER_UNFIRED[] = "USER UNFIRED\r\n";
    

    /*
    int user_but_enabled() {
        return ( USER_BUT_GPIO->IDR & (1 << USER_BUT_PIN) ) == 0;
    }
    */
    send_or_enqueue(user_but_enabled() ? USER_FIRED : USER_UNFIRED);
}


void DMA_USART_config() {
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
    
}

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
#define LIS35DE_PD_BIT 7


#define wait(cond) while(!(cond));

void LIS35DEpowerOn() {
	I2C1->CR1 |= I2C_CR1_START;
	wait(I2C1->SR1 & I2C_SR1_SB);
	I2C1->DR = LIS35DE_ADDR << 1;

	wait(I2C1->SR1 & I2C_SR1_ADDR);

	I2C1->SR2;

	// piszemy do rejestru kontrolnego
	I2C1->DR = LIS35DE_CTRL_REG1;

	wait(I2C1->SR1 & I2C_SR1_TXE);

	I2C1->DR = 0b01000111;

	wait(I2C1->SR1 & I2C_SR1_BTF);

	I2C1->CR1 |= I2C_CR1_STOP;
}


// (master receive)
char MR_receive(char reg) {
    
    // Zainicjuj transmisję sygnału START
    I2C1->CR1 |= I2C_CR1_START;
    
    // send_or_enqueue("0");
    
    wait(I2C1->SR1 & I2C_SR1_SB);
    // START wysłano
    
    // send_or_enqueue("1");
    I2C1->DR = LIS35DE_ADDR << 1;
    
    wait(I2C1->SR1 & I2C_SR1_ADDR);
    // ADRES wysłano
    
    // send_or_enqueue("2");
    // skasuj bit ADDR
    I2C1->SR2;
    
    // Zainicjuj wysyłanie 8-bitowego numeru rejestru slave’a
    I2C1->DR = reg;
    
    wait(I2C1->SR1 & I2C_SR1_BTF);
    // dane wysłano (numer rejestru slave'a)
    
    // send_or_enqueue("3");
    
    // Zainicjuj transmisję sygnału REPEATED START
    I2C1->CR1 |= I2C_CR1_START;
    
    wait(I2C1->SR1 & I2C_SR1_SB);
    // START wysłano
    
    
    // send_or_enqueue("4");
    
    // tryb MR
    I2C1->DR = (LIS35DE_ADDR << 1) | 1U;

    
    I2C1->CR1 &= ~I2C_CR1_ACK;
    
    wait(I2C1->SR1 & I2C_SR1_ADDR);
    // zakończona transmisja adresu
    
    // send_or_enqueue("5");
    
    char nic = I2C1->SR2;
    
    // wyślij STOP
    I2C1->CR1 |= I2C_CR1_STOP;
    
    
    wait(I2C1->SR1 & I2C_SR1_RXNE);
    // ODEBRANO WYNIK
    
    // send_or_enqueue("6");
    
    char value = I2C1->DR;
    return value;
}

void EXTI0_IRQHandler(void) {
    char MODE_FIRED[] = "MODE FIRED\r\n";
    char MODE_UNFIRED[] = "MODE UNFIRED\r\n";
    
    char res = MR_receive(OUT_X);
    char resstr[5];
    resstr[0] = res == NULL ? 'X' : res;
    resstr[1] = '\n';
    resstr[2] = '\0';
    if (strlen(resstr) == 0)
	send_or_enqueue("zle");
    // sprintf(resstr, "%d\n", (int) res); TODO
    // send_or_enqueue(mode_but_enabled() ? MODE_FIRED : MODE_UNFIRED);
    EXTI->PR = EXTI_PR_PR0;
    send_or_enqueue(resstr);

    EXTI->PR = EXTI_PR_PR0;
}

void CounterConfig() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->CR1 = 0; //  w górę
	TIM3->PSC = 1000;
	TIM3->ARR = 0xffff;
	TIM3->EGR = TIM_EGR_UG; // zainicjuj prescaler i auto reload

	// przerwania
	NVIC_EnableIRQ(TIM3_IRQn);

	TIM3->SR = ~(TIM_SR_UIF | TIM_SR_CC1IF); // TODO UIF nie trzeba
	TIM3->CCR1 = 0xfff0; // TODO
	TIM3->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
}

void CounterEnable() {
	TIM3->CR1 |= TIM_CR1_CEN;
}


void TIM3_IRQHandler(void) {
	uint32_t it_status = TIM3->SR & TIM3->DIER;
	if (it_status & TIM_SR_UIF) {
		TIM3->SR = ~TIM_SR_UIF;
		send_or_enqueue("tego nie powinno byc\r\n");
	}

	if (it_status & TIM_SR_CC1IF) {
		TIM3->SR = ~TIM_SR_CC1IF;
		send_or_enqueue("przerwanie\r\n");
		BlueLEDon();
	}
}

int main() {
    // Trzeba pamiętać o uprzednim włączeniu taktowania układu SYSCFG
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
	// włączenie GPIO, DMA, USART
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOBEN |  
	RCC_AHB1ENR_DMA1EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // NOWE
	i2c_config();
	__NOP();


	LIS35DEpowerOn();


	DMAconfig();
	LedsConfig();
	UserButInterruptEXTIConfig();
	ModeButInterruptEXTIConfig();
	DMA_USART_config();
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(EXTI0_IRQn);

	CounterConfig();
	CounterEnable();

	for(;;) {
		// char x = MR_receive(OUT_X);
		// char y = MR_receive(OUT_Y);
		// char z = MR_receive(OUT_Z);
		
	}

} // main




