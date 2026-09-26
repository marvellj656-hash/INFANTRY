#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "remote/dbus_protocol.h"

#include "bsp_uart.h"
#include "queue.h"
#include "usart.h"

UART_HandleTypeDef huart3;
static BspUartPacket queued;
static bool occupied;
static uint8_t *dma;
static unsigned dma_size, position;
static TickType_t tick;
static int event;
static bool fail_start;
QueueHandle_t xQueueCreate(unsigned n, unsigned bytes)
{ assert(n == 1 && bytes == sizeof queued); return &queued; }
BaseType_t xQueueReceive(QueueHandle_t q, void *out, TickType_t wait)
{ (void)q; assert(wait == 0); if (!occupied) return 0; memcpy(out,&queued,sizeof queued); occupied=false; return pdPASS; }
BaseType_t xQueueOverwriteFromISR(QueueHandle_t q, const void *p, BaseType_t *wake)
{ (void)q; memcpy(&queued,p,sizeof queued); occupied=true; *wake=0; return pdPASS; }
BaseType_t xQueueReset(QueueHandle_t q) { (void)q; occupied=false; return pdPASS; }
uint32_t bsp_time_ms_from_isr(void) { return tick; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h)
{ assert(h == &huart3); return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *b,uint16_t n)
{ assert(h == &huart3); dma=b; dma_size=n; position=0; return fail_start ? HAL_ERROR : HAL_OK; }
int HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *h) { (void)h; return event; }

static void feed(const uint8_t *bytes, unsigned n, bool idle)
{
    for(unsigned i=0;i<n;i++) {
        dma[position++] = bytes[i];
        if(position == dma_size/2 || position == dma_size) {
            event = position == dma_size ? HAL_UART_RXEVENT_TC : HAL_UART_RXEVENT_HT;
            HAL_UARTEx_RxEventCallback(&huart3,(uint16_t)position);
            if(position == dma_size) position=0;
        }
    }
    if(idle) {
        event=HAL_UART_RXEVENT_IDLE;
        HAL_UARTEx_RxEventCallback(&huart3,(uint16_t)(position ? position : dma_size));
    }
}
static void pack(uint8_t *b, int a, int c, int d, int e, unsigned s1, unsigned s2)
{
    memset(b,0,18);
    uint64_t bits=(uint64_t)(a+1024) | (uint64_t)(c+1024)<<11 |
                  (uint64_t)(d+1024)<<22 | (uint64_t)(e+1024)<<33 |
                  (uint64_t)s2<<44 | (uint64_t)s1<<46;
    for(unsigned i=0;i<6;i++) b[i]=(uint8_t)(bits>>(i*8));
    b[17]=4; /* wheel centered */
}
int main(void)
{
    uint8_t b[18];
    DbusState state, before;
    pack(b,-660,660,-123,456,1,3);
    assert(dbus_decode(b,18,&state));
    assert(state.ch[0]==-660 && state.ch[1]==660 && state.ch[2]==-123 && state.ch[3]==456);
    assert(state.sw1==1 && state.sw2==3 && state.wheel==0);
    b[6]=0xff; b[7]=0xff; b[12]=1; b[14]=0x55; b[15]=0xaa;
    assert(dbus_decode(b,18,&state) && state.mouse_x==-1 && state.keys==0xaa55);
    before=state;
    assert(!dbus_decode(b,17,&state) && memcmp(&before,&state,sizeof state)==0);
    assert(!dbus_decode(NULL,18,&state));
    pack(b,661,0,0,0,1,3); assert(!dbus_decode(b,18,&state));
    pack(b,0,0,0,0,0,3); assert(!dbus_decode(b,18,&state));
    pack(b,0,0,0,0,1,3); b[13]=2; assert(!dbus_decode(b,18,&state));
    assert(bsp_uart_init() && bsp_uart_recover_rx());
    BspUartPacket packet;
    for(unsigned i=0;i<24;i++) {
        pack(b,(int)i,0,0,0,1,3); tick=i*7;
        feed(b,18,true);
        assert(bsp_uart_receive(&packet) && packet.length == DBUS_FRAME_BYTES);
        assert(packet.received_at_ms==tick && memcmp(packet.data,b,18)==0);
        /* Duplicate IDLE must not deliver twice. */
        feed(b,0,true); assert(!bsp_uart_receive(&packet));
    }
    feed(b,9,true);
    assert(bsp_uart_receive(&packet) && packet.length != DBUS_FRAME_BYTES);
    /* Good packet crossing a DMA boundary after malformed burst. */
    feed(b,18,true);
    assert(bsp_uart_receive(&packet) && packet.length == DBUS_FRAME_BYTES && !memcmp(b,packet.data,18));
    feed(b,18,false); feed(b,18,true);
    assert(bsp_uart_receive(&packet) && packet.length != DBUS_FRAME_BYTES);
    feed(b,18,true); assert(bsp_uart_receive(&packet) && packet.length == DBUS_FRAME_BYTES);

    /* 超出通用包容量也不能伪装成一个正常短包。 */
    for(unsigned i=0;i<5;i++) feed(b,18,i==4);
    assert(bsp_uart_receive(&packet) && packet.length > BSP_UART_PACKET_BYTES);
    feed(b,18,true); assert(bsp_uart_receive(&packet) && packet.length == 18);
    UART_HandleTypeDef other={0};
    HAL_UART_ErrorCallback(&other); assert(debug_remote_rx.uart_errors==0);
    feed(b,18,true);
    huart3.ErrorCode=8; HAL_UART_ErrorCallback(&huart3);
    fail_start=true;
    assert(!bsp_uart_recover_rx() && debug_remote_rx.start_failures==1);
    assert(!bsp_uart_receive(&packet)); /* Old packet discarded on restart. */
    fail_start=false; assert(bsp_uart_recover_rx());
    feed(b,18,true); assert(bsp_uart_receive(&packet) && packet.length == DBUS_FRAME_BYTES);
    assert(debug_remote_rx.uart_errors==1 && debug_remote_rx.last_uart_error==8);
    puts("PASS: DBUS decode, input validation, DMA wrap/IDLE, UART restart");
    return 0;
}
