#ifndef NUC_TEST_USBD_CDC_H
#define NUC_TEST_USBD_CDC_H
#include "usbd_core.h"
#define CDC_SET_LINE_CODING 0x20u
#define CDC_GET_LINE_CODING 0x21u
typedef struct { volatile uint32_t TxState; uint8_t buffer[512]; } USBD_CDC_HandleTypeDef;
typedef struct {
    int8_t (*Init)(void);
    int8_t (*DeInit)(void);
    int8_t (*Control)(uint8_t, uint8_t *, uint16_t);
    int8_t (*Receive)(uint8_t *, uint32_t *);
    int8_t (*TransmitCplt)(uint8_t *, uint32_t *, uint8_t);
} USBD_CDC_ItfTypeDef;
extern USBD_ClassTypeDef USBD_CDC;
#define USBD_CDC_CLASS (&USBD_CDC)
uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *, USBD_CDC_ItfTypeDef *);
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *, uint8_t *, uint32_t);
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *, uint8_t *);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *);
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *);
#endif
