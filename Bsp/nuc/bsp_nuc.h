#ifndef BSP_NUC_H
#define BSP_NUC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BSP_NUC_PACKET_BYTES 64u
typedef struct
{
    uint8_t data[BSP_NUC_PACKET_BYTES];
    uint16_t length;
    uint32_t received_at_ms;
} BspNucPacket;

typedef struct
{
    uint32_t rx_packets, rx_bytes, dropped_packets, tx_packets, tx_busy;
    uint32_t resets, generation, init_failures;
    /* tx_packets只表示交给USB；以下计数表示USB完成回调，不代表NUC应用已解析。 */
    uint32_t tx_completed, tx_completed_bytes;
    uint8_t initialized, configured;
} BspNucDebug;
extern volatile BspNucDebug debug_nuc_usb;

/* 只能由任务启动；USB 初始化有 HAL_Delay，不能放在临界区或中断中。 */
bool bsp_nuc_start(void);
bool bsp_nuc_ready(void);
bool bsp_nuc_receive(BspNucPacket *out);
bool bsp_nuc_send(const uint8_t *data, size_t length);
uint32_t bsp_nuc_generation(void);
/* USB LL 回调通知链路变化。只改标记，不在中断重建 USB/队列。 */
void bsp_nuc_link_changed(void);

#endif
