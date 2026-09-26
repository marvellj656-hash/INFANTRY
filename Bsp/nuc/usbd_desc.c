#include "usbd_core.h"
#include "usbd_ctlreq.h"

/* 沿用旧 rtconfig.h 的 VID=0x0FFE/PID=0x0001，避免 NUC 按 VID/PID 查找时失效。
 * 不借用 ST 示例的 VID/PID。字符串改变后按设备名绑定的脚本仍需核对。 */
static uint8_t device_descriptor[USB_LEN_DEV_DESC] __attribute__((aligned(4))) = {
    18, USB_DESC_TYPE_DEVICE, 0x00, 0x02, 0x02, 0x02, 0x00, USB_MAX_EP0_SIZE,
    0xfe, 0x0f, 0x01, 0x00, 0x00, 0x02, 1, 2, 3, 1};
static uint8_t language_descriptor[4] __attribute__((aligned(4))) = {4, USB_DESC_TYPE_STRING, 0x09, 0x04};
static uint8_t string_descriptor[USBD_MAX_STR_DESC_SIZ] __attribute__((aligned(4)));
static uint8_t serial_descriptor[50] __attribute__((aligned(4)));

static uint8_t *device(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(device_descriptor);
    return device_descriptor;
}
static uint8_t *language(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(language_descriptor);
    return language_descriptor;
}
static uint8_t *manufacturer(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"HNU", string_descriptor, length);
    return string_descriptor;
}
static uint8_t *product(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"OMNI NUC Virtual Serial", string_descriptor, length);
    return string_descriptor;
}
static uint8_t *serial(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    const uint32_t uid[3] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
    const char digits[] = "0123456789ABCDEF";
    serial_descriptor[0] = sizeof(serial_descriptor);
    serial_descriptor[1] = USB_DESC_TYPE_STRING;
    for (unsigned i = 0; i < 24u; ++i)
    {
        serial_descriptor[2u + i * 2u] = (uint8_t)digits[(uid[i / 8u] >> (28u - (i % 8u) * 4u)) & 15u];
        serial_descriptor[3u + i * 2u] = 0u;
    }
    *length = sizeof(serial_descriptor);
    return serial_descriptor;
}
static uint8_t *configuration(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"CDC", string_descriptor, length);
    return string_descriptor;
}
static uint8_t *interface(USBD_SpeedTypeDef speed, uint16_t *length)
{
    return configuration(speed, length);
}
USBD_DescriptorsTypeDef nuc_usb_descriptors = {
    device, language, manufacturer, product, serial, configuration, interface};
