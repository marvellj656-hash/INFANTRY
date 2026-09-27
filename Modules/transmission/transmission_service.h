#ifndef TRANSMISSION_SERVICE_H
#define TRANSMISSION_SERVICE_H

#include "transmission.h"
#include "transmission_tx.h"

/* 只有 transmission_task 写入。cmd 只包含 transmission.h 读取快照。 */
void transmission_init(void);
void transmission_service_update(uint32_t now_ms, uint32_t link_generation, bool ready);
void transmission_service_receive(const uint8_t *data, size_t length,
                                  uint32_t received_at_ms, uint32_t now_ms);
/* 参考角变了，旧目标就不再属于新坐标系；清掉目标及半包，但保留诊断计数。 */
void transmission_invalidate_target(void);
/* 心跳不依赖云台归中。motion为NULL表示本圈姿态无效，只维持链路心跳。 */
void transmission_service_send(uint32_t now_ms, bool ready, const TransmissionMotion *motion,
                               TransmissionSend send, void *context);

#endif
