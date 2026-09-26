#include "usbd_core.h"
#include "bsp_nuc.h"
#include "usb_otg.h"

/* USB 库只懂“端点”，HAL PCD 才懂 STM32 寄存器；本文件是二者之间的翻译。 */
/* 唯一 PCD 对象由 CubeMX 创建；这里不再创建第二个硬件句柄。 */
static USBD_StatusTypeDef status(HAL_StatusTypeDef result)
{
    return result == HAL_OK ? USBD_OK : (result == HAL_BUSY ? USBD_BUSY : USBD_FAIL);
}
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *p) { USBD_LL_SetupStage(p->pData, (uint8_t *)p->Setup); }
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *p, uint8_t ep) { USBD_LL_DataOutStage(p->pData, ep, p->OUT_ep[ep].xfer_buff); }
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *p, uint8_t ep) { USBD_LL_DataInStage(p->pData, ep, p->IN_ep[ep].xfer_buff); }
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *p) { USBD_LL_SOF(p->pData); }
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *p)
{
    bsp_nuc_link_changed();
    USBD_LL_SetSpeed(p->pData, USBD_SPEED_FULL);
    USBD_LL_Reset(p->pData);
}
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *p)
{
    bsp_nuc_link_changed();
    USBD_LL_Suspend(p->pData);
}
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *p) { USBD_LL_Resume(p->pData); }
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *p, uint8_t ep) { USBD_LL_IsoOUTIncomplete(p->pData, ep); }
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *p, uint8_t ep) { USBD_LL_IsoINIncomplete(p->pData, ep); }
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *p) { USBD_LL_DevConnected(p->pData); }
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *p)
{
    bsp_nuc_link_changed();
    USBD_LL_DevDisconnected(p->pData);
}
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *device)
{
    PCD_HandleTypeDef *pcd = &hpcd_USB_OTG_FS;
    /* main 已调用 CubeMX 的 MX_USB_OTG_FS_PCD_Init：引脚、速率、VBUS 都由它配置。
     * 先把两边对象连起来，USB 中断才知道该把数据交给哪个协议栈。
     * 首次无需重复初始化；失败清理后 PCD 为 RESET，重试时复用原来的硬件配置。 */
    if (pcd->Instance != USB_OTG_FS)
        return USBD_FAIL;
    pcd->pData = device;
    device->pData = pcd;
    if (HAL_PCD_GetState(pcd) != HAL_PCD_STATE_READY && HAL_PCD_Init(pcd) != HAL_OK)
        return USBD_FAIL;
    /* FIFO 单位是 32 位字。128+64+96+16=304，低于 FS 的 320 字容量。 */
    if (HAL_PCDEx_SetRxFiFo(pcd, 128u) != HAL_OK ||
        HAL_PCDEx_SetTxFiFo(pcd, 0u, 64u) != HAL_OK ||
        HAL_PCDEx_SetTxFiFo(pcd, 1u, 96u) != HAL_OK ||
        HAL_PCDEx_SetTxFiFo(pcd, 2u, 16u) != HAL_OK)
        return USBD_FAIL;
    return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *d) { return status(HAL_PCD_DeInit(d->pData)); }
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *d) { return status(HAL_PCD_Start(d->pData)); }
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *d) { return status(HAL_PCD_Stop(d->pData)); }
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *d, uint8_t ep, uint8_t type, uint16_t size) { return status(HAL_PCD_EP_Open(d->pData, ep, size, type)); }
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *d, uint8_t ep) { return status(HAL_PCD_EP_Close(d->pData, ep)); }
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *d, uint8_t ep) { return status(HAL_PCD_EP_Flush(d->pData, ep)); }
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *d, uint8_t ep) { return status(HAL_PCD_EP_SetStall(d->pData, ep)); }
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *d, uint8_t ep) { return status(HAL_PCD_EP_ClrStall(d->pData, ep)); }
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *d, uint8_t ep)
{
    PCD_HandleTypeDef *p = d->pData;
    return (ep & 0x80u) ? p->IN_ep[ep & 0x7fu].is_stall : p->OUT_ep[ep & 0x7fu].is_stall;
}
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *d, uint8_t address) { return status(HAL_PCD_SetAddress(d->pData, address)); }
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *d, uint8_t ep, uint8_t *buffer, uint32_t size) { return status(HAL_PCD_EP_Transmit(d->pData, ep, buffer, size)); }
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *d, uint8_t ep, uint8_t *buffer, uint32_t size) { return status(HAL_PCD_EP_Receive(d->pData, ep, buffer, size)); }
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *d, uint8_t ep) { return HAL_PCD_EP_GetRxCount(d->pData, ep); }
void USBD_LL_Delay(uint32_t milliseconds) { HAL_Delay(milliseconds); }
