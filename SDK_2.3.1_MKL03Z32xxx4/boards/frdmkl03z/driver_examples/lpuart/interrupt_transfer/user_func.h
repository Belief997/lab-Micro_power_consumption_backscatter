#ifndef _USER__FUNC__H_
#define _USER__FUNC__H_

#include "fsl_common.h"
#include "fsl_lpuart.h"

// common

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;





// uart
#define DEMO_LPUART LPUART0
#define DEMO_LPUART_CLKSRC SYS_CLK
#define DEMO_LPUART_CLK_FREQ CLOCK_GetFreq(SYS_CLK)
#define ECHO_BUFFER_LENGTH 8


void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);






// dac




#endif /* _USER__FUNC__H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
