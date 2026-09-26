#ifndef NUC_TEST_USBD_CORE_H
#define NUC_TEST_USBD_CORE_H
#include <stdint.h>
#include <stddef.h>
#define USBD_OK 0u
#define USBD_FAIL 2u
#define USBD_STATE_CONFIGURED 3u
typedef struct { uint8_t dev_state; void *pClassData; } USBD_HandleTypeDef;
typedef struct { int unused; } USBD_DescriptorsTypeDef;
typedef struct { int unused; } USBD_ClassTypeDef;
uint8_t USBD_Init(USBD_HandleTypeDef *, USBD_DescriptorsTypeDef *, uint8_t);
uint8_t USBD_DeInit(USBD_HandleTypeDef *);
uint8_t USBD_RegisterClass(USBD_HandleTypeDef *, USBD_ClassTypeDef *);
uint8_t USBD_Start(USBD_HandleTypeDef *);
void *bsp_nuc_usb_alloc(size_t);
void bsp_nuc_usb_free(void *);
#endif
