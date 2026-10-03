#include "dji_motor_service.h"
#include "bsp_can.h"
#include "bsp_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
struct DjiMotor
{
    DjiMotorConfig config;
    DjiMotorFeedback feedback;
    uint16_t rx_id;
    uint8_t group, slot;
    bool registered, command_valid;
    int16_t requested;
    uint32_t commanded_at_ms;
    int64_t encoder_total;
};
typedef struct
{
    uint8_t bus;
    uint16_t id;
    bool active, tx_ok;
    BspCanTx tx;
    CanTxDebug debug;
} MotorGroup;
static struct DjiMotor objects[MOTOR_COUNT];
/* 每条总线五组：C6x0两组/6020电压两组（共享0x1FF）/6020电流两组。
 * 电流帧必须单独存放，不能覆盖同总线摩擦轮的0x200帧。 */
static MotorGroup groups[10];
volatile MotorFeedbackDebug debug_motor[MOTOR_COUNT];
volatile CanTxDebug debug_chassis_tx; /* 保留现有 LinkScope 名称。 */
static bool valid_handle(DjiMotorHandle motor)
{
    /* 先比较地址再解引用，NULL 或外来的指针不能访问对象内部。 */
    for (unsigned i = 0; i < MOTOR_COUNT; ++i)
        if (motor == &objects[i])
            return objects[i].registered;
    return false;
}
static bool fresh(uint32_t now, uint32_t then, uint32_t timeout)
{
    /* 无符号减法能跨过计数回绕；超时值限制为小于半个计数范围。 */
    return (uint32_t)(now - then) < timeout;
}
DjiMotorHandle dji_motor_register(const DjiMotorConfig *c)
{
    if (!c || (unsigned)c->id >= MOTOR_COUNT || (unsigned)c->type > DJI_GM6020_CURRENT || c->bus < 1 ||
        c->bus > 2 || c->esc_id < 1 || c->esc_id > (c->type >= DJI_GM6020 ? 7 : 8) ||
        c->output_limit_raw < 0 || !c->feedback_timeout_ms || !c->command_timeout_ms ||
        c->feedback_timeout_ms >= 0x80000000u || c->command_timeout_ms >= 0x80000000u)
        return NULL;
    int limit = c->type == DJI_M3508 ? 16384 : c->type == DJI_M2006 ? 10000 : c->type == DJI_GM6020_CURRENT ? 16384 : 30000;
    if (c->output_limit_raw > limit || objects[c->id].registered)
        return NULL;
    bool gm6020 = c->type >= DJI_GM6020;
    uint16_t rx = (gm6020 ? 0x204 : 0x200) + c->esc_id;
    unsigned local = c->type == DJI_GM6020_CURRENT ? (c->esc_id <= 4 ? 3 : 4)
                     : gm6020 ? (c->esc_id <= 4 ? 1 : 2) : (c->esc_id <= 4 ? 0 : 1);
    unsigned group = (c->bus - 1u) * 5u + local, slot = (c->esc_id - 1u) % 4u;
    for (unsigned i = 0; i < MOTOR_COUNT; ++i)
    {
        DjiMotor *m = &objects[i];
        if (m->registered && m->config.bus == c->bus &&
            (m->rx_id == rx || (m->group == group && m->slot == slot)))
            return NULL;
    }
    /* 全部检查通过才写对象，失败不会留下半个对象。 */
    DjiMotor *m = &objects[c->id];
    *m = (DjiMotor){.config = *c,
                    .rx_id = rx,
                    .group = (uint8_t)group,
                    .slot = (uint8_t)slot,
                    .registered = true};
    groups[group].bus = c->bus;
    static const uint16_t ids[5] = {0x200, 0x1ff, 0x2ff, 0x1fe, 0x2fe};
    groups[group].id = ids[local];
    groups[group].active |= c->output_limit_raw > 0;
    groups[group].tx_ok = true;
    debug_motor[c->id] = (MotorFeedbackDebug){.bus = c->bus, .id = rx, .tx_id = groups[group].id, .age_ms = UINT32_MAX};
    return m;
}
DjiMotorHandle dji_motor_find(DjiMotorId id)
{
    return (unsigned)id < MOTOR_COUNT && objects[id].registered ? &objects[id] : NULL;
}
bool dji_motor_read_many(const DjiMotorHandle *motors, size_t count, DjiMotorFeedback *out)
{
    if (!motors || !out || !count || count > MOTOR_COUNT)
        return false;
    for (size_t i = 0; i < count; ++i)
        if (!valid_handle(motors[i]))
            return false;
    /* 临界区像暂时关门：只搬小块数据，不能在这里等 CAN 或打印。 */
    taskENTER_CRITICAL();
    for (size_t i = 0; i < count; ++i)
    {
        out[i] = motors[i]->feedback;
        out[i].transport_ready = groups[motors[i]->group].tx_ok;
    }
    taskEXIT_CRITICAL();
    uint32_t now = bsp_time_ms();
    for (size_t i = 0; i < count; ++i)
    {
        out[i].online = out[i].received &&
                        fresh(now, out[i].received_at_ms, motors[i]->config.feedback_timeout_ms);
        out[i].transport_ready &= bsp_can_is_ready(motors[i]->config.bus);
    }
    return true;
}
bool dji_motor_read(DjiMotorHandle motor, DjiMotorFeedback *out)
{
    if (out)
        *out = (DjiMotorFeedback){0};
    return dji_motor_read_many(&motor, 1, out);
}
bool dji_motor_set_outputs(const DjiMotorHandle *motors, const int16_t *raw, size_t count)
{
    if (!motors || !raw || !count || count > MOTOR_COUNT)
        return false;
    for (size_t i = 0; i < count; ++i)
    {
        if (!valid_handle(motors[i]) || (raw[i] && !motors[i]->config.output_limit_raw))
            return false;
        for (size_t j = 0; j < i; ++j)
            if (motors[i] == motors[j])
                return false;
    }
    uint32_t now = bsp_time_ms();
    taskENTER_CRITICAL();
    for (size_t i = 0; i < count; ++i)
    {
        motors[i]->requested = raw[i];
        motors[i]->commanded_at_ms = now;
        motors[i]->command_valid = true;
    }
    taskEXIT_CRITICAL();
    return true;
}
bool dji_motor_set_output(DjiMotorHandle motor, int16_t raw)
{
    return dji_motor_set_outputs(&motor, &raw, 1);
}
bool dji_motor_relax(DjiMotorHandle motor)
{
    return dji_motor_set_output(motor, 0);
}
bool dji_motor_receive(uint8_t bus, uint16_t id, const uint8_t *data, size_t length, uint32_t time)
{
    for (unsigned i = 0; i < MOTOR_COUNT; ++i)
    {
        DjiMotor *m = &objects[i];
        if (!m->registered || m->config.bus != bus || m->rx_id != id)
            continue;
        DjiFeedback value;
        if (!dji_motor_decode_feedback(data, length, &value))
        {
            debug_motor[i].invalid_frames++;
            return true;
        }
        taskENTER_CRITICAL();
        uint32_t generation = m->feedback.generation;
        if (!m->feedback.received ||
            !fresh(time, m->feedback.received_at_ms, m->config.feedback_timeout_ms))
        {
            m->encoder_total = 0;
            generation++;
        }
        else
        {
            /* 相邻帧转角必须小于半圈；断线期间转了多少圈无法靠单圈编码器猜出。 */
            int delta = (int)value.ecd - (int)m->feedback.measure.ecd;
            if (delta > 4096)
                delta -= 8192;
            if (delta < -4096)
                delta += 8192;
            m->encoder_total += delta;
        }
        m->feedback =
            (DjiMotorFeedback){.measure = value,
                               .received_at_ms = time,
                               .received = true,
                               .total_angle_deg = (float)m->encoder_total * (360.0f / 8192.0f),
                               .generation = generation};
        taskEXIT_CRITICAL();
        debug_motor[i].ecd = value.ecd;
        debug_motor[i].speed_rpm = value.speed_rpm;
        debug_motor[i].current_raw = value.current_raw;
        debug_motor[i].temperature = value.temperature;
        debug_motor[i].frames++;
        return true;
    }
    return false;
}
void dji_motor_service(void)
{
    int16_t raw[10][4] = {{0}}, limits[10][4] = {{0}};
    bool ready[2] = {bsp_can_is_ready(1), bsp_can_is_ready(2)};
    /* 时间和指令一起取，避免高优先级任务恰好提交了“未来”的时间戳。 */
    taskENTER_CRITICAL();
    uint32_t now = bsp_time_ms();
    for (unsigned i = 0; i < MOTOR_COUNT; ++i)
    {
        DjiMotor *m = &objects[i];
        if (!m->registered)
            continue;
        bool online = m->feedback.received &&
                      fresh(now, m->feedback.received_at_ms, m->config.feedback_timeout_ms);
        debug_motor[i].online = online;
        debug_motor[i].age_ms =
            m->feedback.received ? now - m->feedback.received_at_ms : UINT32_MAX;
        if (!online || !ready[m->config.bus - 1] || !groups[m->group].tx_ok ||
            !fresh(now, m->commanded_at_ms, m->config.command_timeout_ms))
            m->command_valid = false;
        limits[m->group][m->slot] = m->config.output_limit_raw;
        raw[m->group][m->slot] = m->command_valid ? m->requested : 0;
    }
    taskEXIT_CRITICAL();
    for (unsigned g = 0; g < 10; ++g)
    {
        MotorGroup *group = &groups[g];
        if (!group->active)
            continue; /* 只读云台不产生控制帧。 */
        BspCanFrame frame = {.bus = group->bus, .id = group->id, .length = 8};
        dji_motor_encode_currents(raw[g], limits[g], frame.data);
        /* 唯一发送者，不同任务控制同帧中的不同电机，也不会互相覆盖。 */
        bool ok = bsp_can_send_latest(&group->tx, &frame, &group->debug);
        if (ok)
        {
            for (unsigned i = 0; i < MOTOR_COUNT; ++i)
            {
                const DjiMotor *m = &objects[i];
                if (!m->registered || m->group != g)
                    continue;
                unsigned offset = 2u * m->slot;
                unsigned value = ((unsigned)frame.data[offset] << 8) | frame.data[offset + 1u];
                debug_motor[i].queued_output_raw =
                    value >= 32768u ? (int32_t)value - 65536 : (int32_t)value;
                debug_motor[i].output_queue_count++;
            }
        }
        taskENTER_CRITICAL();
        group->tx_ok = ok;
        taskEXIT_CRITICAL();
        if (g == 0)
            debug_chassis_tx = group->debug;
    }
}
