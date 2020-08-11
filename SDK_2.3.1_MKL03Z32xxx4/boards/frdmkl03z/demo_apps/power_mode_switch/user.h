
#ifndef ___USER__H
#define ___USER__H

#include "fsl_common.h"

#define MAX_ADC_COUNT  (25)
#define ADC_PACK_LEN   (20) // header(7) + payload(12) + check_bit(1) = 20
#define ADC_HEADER_LEN (7)
#define ADC_DATA_LEN   (12)

#define ADC_HEADER  (0x5E >> 1)


typedef uint8_t  u8 ;
typedef uint16_t u16;
typedef uint32_t u32;

typedef enum
{
	FALSE = 0,
	TRUE,
}USER_BOOL;

typedef struct
{
	u16 adcValue;
}ADC_DATA;

typedef struct
{
    int front;
    int rear;
    ADC_DATA adc[MAX_ADC_COUNT];
}ADC_QUEUE;


typedef struct
{
	ADC_QUEUE adc_queue;

}DATA;

typedef struct{
	u32 reserve : 12;
    u32 header  : 7;
    u32 data    : 12;
    u32 check   : 1;
}ADC_PACK;


DATA *data_getData(void);
USER_BOOL data_isadcQueueFull(void);
USER_BOOL data_isadcQueueEmpty(void);
USER_BOOL data_enqueueadc(ADC_DATA* adc);
USER_BOOL data_dequeueadc(ADC_DATA* adc);


#endif
