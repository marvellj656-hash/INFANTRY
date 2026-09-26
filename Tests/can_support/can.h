#include <stdint.h>
typedef struct { uint32_t ESR; } CAN_Registers;
typedef struct { CAN_Registers *Instance; } CAN_HandleTypeDef;
extern CAN_HandleTypeDef hcan1, hcan2;
typedef struct {
    uint32_t FilterBank, SlaveStartFilterBank, FilterMode, FilterScale;
    uint32_t FilterMaskIdLow, FilterFIFOAssignment, FilterActivation;
} CAN_FilterTypeDef;
typedef struct { uint32_t IDE, RTR, DLC, StdId; } CAN_RxHeaderTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define CAN_FILTERMODE_IDMASK 0
#define CAN_FILTERSCALE_32BIT 1
#define CAN_RX_FIFO0 0
#define ENABLE 1
#define DISABLE 0
#define CAN_ESR_BOFF 4u
#define HAL_CAN_STATE_LISTENING 1u
#define CAN_ID_STD 0
#define CAN_RTR_DATA 0
#define CAN_IT_RX_FIFO0_MSG_PENDING 1u
#define CAN_IT_RX_FIFO0_OVERRUN 2u
#define CAN_IT_ERROR 4u
#define CAN_IT_BUSOFF 8u
#define CAN_IT_ERROR_PASSIVE 16u
#define CAN_IT_ERROR_WARNING 32u
#define CAN_IT_LAST_ERROR_CODE 64u
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *, CAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t, CAN_RxHeaderTypeDef *, uint8_t *);
uint32_t HAL_CAN_GetState(CAN_HandleTypeDef *);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *, uint32_t);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *, CAN_TxHeaderTypeDef *, uint8_t *, uint32_t *);
