#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include <string.h>

#include <irq.h>

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






