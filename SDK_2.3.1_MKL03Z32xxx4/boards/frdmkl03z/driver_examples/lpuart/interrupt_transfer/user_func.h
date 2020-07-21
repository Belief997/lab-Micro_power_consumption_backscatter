#ifndef _USER__FUNC__H_
#define _USER__FUNC__H_

#include "fsl_common.h"
#include "fsl_lpuart.h"




// common

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;





// uart
//#define EN_UART
#define DEMO_LPUART LPUART0
#define DEMO_LPUART_CLKSRC SYS_CLK
#define DEMO_LPUART_CLK_FREQ CLOCK_GetFreq(SYS_CLK)
#define ECHO_BUFFER_LENGTH 8


void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);






// dac
// PIN
#define PIN_DOUT  (5)
#define PIN_CS    (6)
#define PIN_CLK   (7)
#define PIN_DIN   (0)
#define PIN_MOD   (3)


typedef enum{
	GPIO_L = 0,
	GPIO_H = 1,

};


void dac_init(void);
void dac_test(void);
void dac_send(u16 data);
void dac_setVol(float Vol_mV, float shift);

void timer_init(void);

#endif /* _USER__FUNC__H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
