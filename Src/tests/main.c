#include "serial_telemetry.h"
#include "targets.h"
#include "stm32h5xx_ll_usart.h"

#include <stdint.h>

int main()
{
    telem_UART_Init();
    // USART3->TDR = 'U';
    // while (!(USART3->ISR & USART_ISR_TXE));
    while(1) {
        send_telem_DMA();
    }
}