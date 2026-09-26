#ifndef MODULE_REFEREE_H
#define MODULE_REFEREE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define REFEREE_LINK_TIMEOUT_MS 1000u
#define REFEREE_STATUS_TIMEOUT_MS 1500u
#define REFEREE_POWER_TIMEOUT_MS 500u
#define REFEREE_STREAM_GAP_MS 100u

typedef struct {
    uint32_t received_at_ms;
    uint32_t age_ms;
    bool received; /* 区分“真的收到 0”与“开机还没收到”。 */
    bool fresh;
} RefereeFreshness;

/* 持有真实状态的是本模块。外部只拿副本，不能通过指针改内部数据。 */
typedef struct {
    bool online;
    uint32_t generation; /* UART 重启或 robot_id 改变，UI 等消费者需重新建立身份。 */
    uint32_t received_at_ms;
    uint32_t age_ms;
    uint32_t valid_frames;
    uint16_t last_command;
    uint8_t last_sequence;

    RefereeFreshness status;
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_cooling_per_s;
    uint16_t shooter_heat_limit;
    uint16_t chassis_power_limit_w;
    bool gimbal_power_enabled;
    bool chassis_power_enabled;
    bool shooter_power_enabled;

    RefereeFreshness power_heat;
    uint16_t buffer_energy_j;
    uint16_t shooter_heat_17mm_1;
    uint16_t shooter_heat_17mm_2;
    uint16_t shooter_heat_42mm;
    /* 旧头文件已把前 8 字节命名 reserved_1/2/3。
     * 这里只保存原始值供排查，不能拿它们作为有效电流/功率限幅反馈。 */
    uint16_t legacy_voltage_raw;
    uint16_t legacy_current_raw;
    uint32_t legacy_power_bits;

    RefereeFreshness shooting;
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency_hz;
    float bullet_speed_m_s;

    RefereeFreshness allowance;
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;

    RefereeFreshness game;
    uint8_t game_type;
    uint8_t game_progress;
    uint16_t stage_remaining_s;
    uint64_t sync_timestamp;
    RefereeFreshness robot_hp;
    uint16_t team_robot_hp[16]; /* 依次红 1/2/3/4/保留/7/前哨/基地，蓝方相同。 */
    RefereeFreshness event;
    uint32_t event_bits;
} RefereeState;

typedef struct {
    uint32_t task_cycles;
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t header_errors;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t unsupported_commands;
    uint32_t invalid_payloads;
    uint32_t stream_resets;
    uint32_t tx_accepted;
    uint32_t tx_rejected;
    uint32_t age_ms;
    uint16_t last_command;
    uint16_t chassis_power_limit_w;
    uint16_t buffer_energy_j;
    uint16_t heat_17mm_1;
    uint16_t heat_limit;
    float bullet_speed_m_s;
    uint8_t robot_id;
    uint8_t online;
    uint8_t status_fresh;
    uint8_t power_heat_fresh;
} RefereeDebug;
extern volatile RefereeDebug debug_referee;

/* 下列写入函数只由 referee_task 调用，不可从中断和多个任务同时调用。 */
void referee_init(void);
void referee_receive(const uint8_t *bytes, size_t length, uint32_t received_at_ms);
void referee_reset_stream(void);
void referee_update(void);
/* 任意任务可读；返回链路在线，具体数据还要查看对应的 .fresh。 */
bool referee_get_state(RefereeState *out);

#endif
