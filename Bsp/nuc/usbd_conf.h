#ifndef APP_USBD_CONF_H
#define APP_USBD_CONF_H

#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <string.h>

#define USBD_MAX_NUM_INTERFACES 2u
#define USBD_MAX_NUM_CONFIGURATION 1u
#define USBD_MAX_STR_DESC_SIZ 128u
#define USBD_SELF_POWERED 1u
#define USBD_DEBUG_LEVEL 0u
#define USBD_LPM_ENABLED 0u
#define USBD_SUPPORT_USER_STRING_DESC 0u

void *bsp_nuc_usb_alloc(size_t size);
void bsp_nuc_usb_free(void *memory);
/* CDC 只需要一个长期存在的类对象，静态分配避免占用很小的 C 库 heap。 */
#define USBD_malloc bsp_nuc_usb_alloc
#define USBD_free bsp_nuc_usb_free
#define USBD_memset memset
#define USBD_memcpy memcpy
#define USBD_Delay HAL_Delay
#define USBD_UsrLog(...) ((void)0)
#define USBD_ErrLog(...) ((void)0)
#define USBD_DbgLog(...) ((void)0)

#endif
