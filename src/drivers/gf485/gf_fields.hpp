// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

/**
 * @file gf_fields.hpp
 * @brief GF485 寄存器映射表与定点解码（RS485通讯协议规范 V1.3，表驱动，同 SCUD 模式）。
 * @details 四块 Modbus 0x03 读取覆盖全表：0~3 版本、100~142 基本信息、
 *          143~192 电芯明细、193~208 充电交互。解码公式：实数 = raw × scale +
 *          value_off（多字节大端）。温度寄存器单位 K，value_off=-273.15 统一转 ℃；
 *          电压/电流/容量按规范精度折算为 V/A/Ah。118/121/123/125 状态字按位展开
 *          为具名布尔字段（0.0/1.0），原值保留；119~120 均衡字展开为
 *          bal_cell1..bal_cell32。类型/解码/线协议复用 scud_fields.hpp，
 *          两驱动共用 .fields socket 记录格式。
 */
#pragma once

#include <string>

#include "scud_fields.hpp"

namespace gf_bms {

using scud_bms::expand_bits;
using scud_bms::field_or;
using scud_bms::FieldMap;
using scud_bms::FieldSpec;
using scud_bms::NamedBit;
using scud_bms::parse_fields;
using scud_bms::RawKind;

// ---- Modbus 块范围（寄存器地址，见规范 §4；载荷偏移 = (寄存器-起始)×2） ----
constexpr uint16_t kVerStart = 0, kVerCount = 4;      // 0~3   版本/设备类型
constexpr uint16_t kIdStart = 100, kIdCount = 43;     // 100~142 基本信息
constexpr uint16_t kCellStart = 143, kCellCount = 50; // 143~192 电芯明细
constexpr uint16_t kChgStart = 193, kChgCount = 16;   // 193~208 充电交互

// ---- 电池编码（SN）：100 起 12 个寄存器 = 24 字节 ASCII ----
constexpr uint16_t kSnStart = 100, kSnCount = 12;
constexpr size_t kSnLen = 24;

// ---- 0~3 版本区：版本字高字节大版本、低字节小版本（0x0001 → 0.1） ----
inline constexpr FieldSpec kVersionFields[] = {
    {"proto_ver", 0, RawKind::U16, 1.0, 0.0}, // 0 协议版本号
    {"sw_ver",    2, RawKind::U16, 1.0, 0.0}, // 1 软件版本号
    {"hw_ver",    4, RawKind::U16, 1.0, 0.0}, // 2 硬件版本号
    {"dev_type",  6, RawKind::U16, 1.0, 0.0}, // 3 设备类型：固定 1 = BMS
};

// ---- 100~142 基本信息 ----
inline constexpr FieldSpec kIdFields[] = {
    // 100~111 电池编码（24 字节 ASCII）由 parse_sn() 单独交付，不进 double 表
    {"pack_vendor",    24, RawKind::U8,  1.0, 0.0},     // 112 高字节 PACK 厂商
    {"bms_vendor",     25, RawKind::U8,  1.0, 0.0},     // 112 低字节 BMS 厂商
    {"cell_type",      26, RawKind::U8,  1.0, 0.0},     // 113 高字节 电芯类型 0 LFP/1 NCM/2 LMO/3 LTO/4 LMFP/5 LCO/6 NMC/7 NCA
    {"cell_series",    27, RawKind::U8,  1.0, 0.0},     // 113 低字节 电芯串数（16=16S）
    {"total_v",        28, RawKind::U16, 0.01, 0.0},    // 114 总电压 10mV → V
    {"total_a",        30, RawKind::I16, 0.01, 0.0},    // 115 总电流 10mA → A（充电为正放电为负）
    {"soc_pct",        32, RawKind::U16, 1.0, 0.0},     // 116 SOC 1%
    {"soh_pct",        34, RawKind::U16, 1.0, 0.0},     // 117 SOH 1%
    {"sys_state",      36, RawKind::U16, 1.0, 0.0},     // 118 BMS 系统状态
    {"balance",        38, RawKind::U32, 1.0, 0.0},     // 119~120 均衡状态 bit0~31 → 电芯 1~32
    {"alarm",          42, RawKind::U32, 1.0, 0.0},     // 121~122 告警状态
    {"protect",        46, RawKind::U32, 1.0, 0.0},     // 123~124 保护状态
    {"fault",          50, RawKind::U16, 1.0, 0.0},     // 125 故障状态
    {"remain_ah",      52, RawKind::U16, 0.01, 0.0},    // 126 剩余容量 10mAH → Ah
    {"design_ah",      54, RawKind::U16, 0.01, 0.0},    // 127 设计容量 10mAH → Ah
    {"full_ah",        56, RawKind::U16, 0.01, 0.0},    // 128 满充容量 10mAH → Ah
    {"cycles",         58, RawKind::U16, 1.0, 0.0},     // 129 循环次数
    {"cell_mv_max",    60, RawKind::U16, 1.0, 0.0},     // 130 最高电芯电压 mV
    {"cell_mv_min",    62, RawKind::U16, 1.0, 0.0},     // 131 最低电芯电压 mV
    {"cell_pos_max",   64, RawKind::U8,  1.0, 0.0},     // 132 高字节 最高电压位置
    {"cell_pos_min",   65, RawKind::U8,  1.0, 0.0},     // 132 低字节 最低电压位置
    {"cell_t_max_c",   66, RawKind::U16, 1.0, -273.15}, // 133 最高电芯温度 K → ℃
    {"cell_t_min_c",   68, RawKind::U16, 1.0, -273.15}, // 134 最低电芯温度 K → ℃
    {"cell_t_pos_max", 70, RawKind::U8,  1.0, 0.0},     // 135 高字节 最高温度位置
    {"cell_t_pos_min", 71, RawKind::U8,  1.0, 0.0},     // 135 低字节 最低温度位置
    {"env_t_max_c",    72, RawKind::U16, 1.0, -273.15}, // 136 最高环境温度 K → ℃
    {"env_t_min_c",    74, RawKind::U16, 1.0, -273.15}, // 137 最低环境温度 K → ℃
    {"env_pos_max",    76, RawKind::U8,  1.0, 0.0},     // 138 高字节 环境温度最值位置
    {"env_pos_min",    77, RawKind::U8,  1.0, 0.0},     // 138 低字节
    {"mos_t_max_c",    78, RawKind::U16, 1.0, -273.15}, // 139 最高 MOS 温度 K → ℃
    {"mos_t_min_c",    80, RawKind::U16, 1.0, -273.15}, // 140 最低 MOS 温度 K → ℃
    {"mos_pos_max",    82, RawKind::U8,  1.0, 0.0},     // 141 高字节 MOS 温度最值位置
    {"mos_pos_min",    83, RawKind::U8,  1.0, 0.0},     // 141 低字节
    {"cell_count",     84, RawKind::U16, 1.0, 0.0},     // 142 电芯串数（最大 32）
};

// ---- 143~192 电芯明细 ----
inline constexpr FieldSpec kCellFields[] = {
    {"cell1_mv",  0, RawKind::U16, 1.0, 0.0}, // 143 第 1 节电芯电压 mV
    {"cell2_mv",  2, RawKind::U16, 1.0, 0.0},
    {"cell3_mv",  4, RawKind::U16, 1.0, 0.0},
    {"cell4_mv",  6, RawKind::U16, 1.0, 0.0},
    {"cell5_mv",  8, RawKind::U16, 1.0, 0.0},
    {"cell6_mv", 10, RawKind::U16, 1.0, 0.0},
    {"cell7_mv", 12, RawKind::U16, 1.0, 0.0},
    {"cell8_mv", 14, RawKind::U16, 1.0, 0.0},
    {"cell9_mv", 16, RawKind::U16, 1.0, 0.0},
    {"cell10_mv", 18, RawKind::U16, 1.0, 0.0},
    {"cell11_mv", 20, RawKind::U16, 1.0, 0.0},
    {"cell12_mv", 22, RawKind::U16, 1.0, 0.0},
    {"cell13_mv", 24, RawKind::U16, 1.0, 0.0},
    {"cell14_mv", 26, RawKind::U16, 1.0, 0.0},
    {"cell15_mv", 28, RawKind::U16, 1.0, 0.0},
    {"cell16_mv", 30, RawKind::U16, 1.0, 0.0},
    {"cell17_mv", 32, RawKind::U16, 1.0, 0.0},
    {"cell18_mv", 34, RawKind::U16, 1.0, 0.0},
    {"cell19_mv", 36, RawKind::U16, 1.0, 0.0},
    {"cell20_mv", 38, RawKind::U16, 1.0, 0.0},
    {"cell21_mv", 40, RawKind::U16, 1.0, 0.0},
    {"cell22_mv", 42, RawKind::U16, 1.0, 0.0},
    {"cell23_mv", 44, RawKind::U16, 1.0, 0.0},
    {"cell24_mv", 46, RawKind::U16, 1.0, 0.0},
    {"cell25_mv", 48, RawKind::U16, 1.0, 0.0},
    {"cell26_mv", 50, RawKind::U16, 1.0, 0.0},
    {"cell27_mv", 52, RawKind::U16, 1.0, 0.0},
    {"cell28_mv", 54, RawKind::U16, 1.0, 0.0},
    {"cell29_mv", 56, RawKind::U16, 1.0, 0.0},
    {"cell30_mv", 58, RawKind::U16, 1.0, 0.0},
    {"cell31_mv", 60, RawKind::U16, 1.0, 0.0},
    {"cell32_mv", 62, RawKind::U16, 1.0, 0.0}, // 174 第 32 节电芯电压 mV
    {"cell_temp_cnt", 64, RawKind::U16, 1.0, 0.0}, // 175 电芯温度个数
    {"cell_t1_c", 66, RawKind::U16, 1.0, -273.15}, // 176 电芯温度 1 K → ℃
    {"cell_t2_c", 68, RawKind::U16, 1.0, -273.15},
    {"cell_t3_c", 70, RawKind::U16, 1.0, -273.15},
    {"cell_t4_c", 72, RawKind::U16, 1.0, -273.15},
    {"cell_t5_c", 74, RawKind::U16, 1.0, -273.15},
    {"cell_t6_c", 76, RawKind::U16, 1.0, -273.15},
    {"cell_t7_c", 78, RawKind::U16, 1.0, -273.15},
    {"cell_t8_c", 80, RawKind::U16, 1.0, -273.15}, // 183 电芯温度 8 K → ℃
    {"env_cnt",   82, RawKind::U8,  1.0, 0.0},     // 184 高字节 环境温度个数（≤4）
    {"mos_cnt",   83, RawKind::U8,  1.0, 0.0},     // 184 低字节 MOS 温度个数（≤4）
    {"env_t1_c",  84, RawKind::U16, 1.0, -273.15}, // 185 环境温度 1 K → ℃
    {"env_t2_c",  86, RawKind::U16, 1.0, -273.15},
    {"env_t3_c",  88, RawKind::U16, 1.0, -273.15},
    {"env_t4_c",  90, RawKind::U16, 1.0, -273.15}, // 188 环境温度 4 K → ℃
    {"mos_t1_c",  92, RawKind::U16, 1.0, -273.15}, // 189 MOS 温度 1 K → ℃
    {"mos_t2_c",  94, RawKind::U16, 1.0, -273.15},
    {"mos_t3_c",  96, RawKind::U16, 1.0, -273.15},
    {"mos_t4_c",  98, RawKind::U16, 1.0, -273.15}, // 192 MOS 温度 4 K → ℃
};

// ---- 193~208 充电交互 ----
inline constexpr FieldSpec kChgFields[] = {
    {"dsg_cont_a",  0, RawKind::I16, 0.01, 0.0}, // 193 可持续放电电流 10mA → A
    {"fbk_a",       2, RawKind::I16, 0.01, 0.0}, // 194 回馈电流 10mA → A（SOC>95% 时为 0）
    // 195~199 预留
    {"chg_req_v",  14, RawKind::U16, 0.01, 0.0}, // 200 BMS 请求充电电压 10mV → V（禁止充电时为 0）
    {"chg_req_a",  16, RawKind::I16, 0.01, 0.0}, // 201 BMS 请求充电电流 10mA → A
    {"chgr_out_a", 18, RawKind::I16, 0.01, 0.0}, // 202 充电器输出电流（可写）
    {"chgr_state", 20, RawKind::U16, 1.0, 0.0},  // 203 充电器接入状态 0x55AA=接入（可写）
    {"acc_state",  22, RawKind::U16, 1.0, 0.0},  // 204 中控 ACC 0 None/1 ON/2 OFF（可写）
    {"bat_shape",  24, RawKind::U8,  1.0, 0.0},  // 205 高字节 电池类型 0 圆柱/1 方形/2 软包
    {"soc_main",   26, RawKind::U16, 1.0, 0.0},  // 206 SOC 主 1%
    {"soc_sub",    28, RawKind::U16, 1.0, 0.0},  // 207 SOC 从 1%
    {"work_mode",  30, RawKind::U16, 1.0, 0.0},  // 208 工作模式 0 单包/1 双包
};

constexpr size_t kVersionFieldCount =
    sizeof(kVersionFields) / sizeof(kVersionFields[0]);
constexpr size_t kIdFieldCount =
    sizeof(kIdFields) / sizeof(kIdFields[0]);
constexpr size_t kCellFieldCount =
    sizeof(kCellFields) / sizeof(kCellFields[0]);
constexpr size_t kChgFieldCount =
    sizeof(kChgFields) / sizeof(kChgFields[0]);

// ---- 位定义表（原值保留，展开为具名布尔字段 0.0/1.0） ----

// 118 BMS 系统状态
inline constexpr NamedBit kSysStateBits[] = {
    {0,  "chg_mos"},   // 充电 MOS 状态（1 闭合）
    {1,  "dsg_mos"},   // 放电 MOS 状态（1 闭合）
    {2,  "pre_mos"},   // 预充 MOS 状态
    {3,  "heat_mos"},  // 加热 MOS 状态
    {4,  "cur_lim"},   // 限流模块（1 使能）
    {5,  "fuse"},      // 保险丝（1 断开）
    {8,  "full_chg"},  // 满充状态
    {9,  "full_dsg"},  // 满放状态
    {10, "chg_state"}, // 充电状态
    {11, "dsg_state"}, // 放电状态
    {12, "chgr_conn"}, // 充电器接入
    {13, "buzzer"},    // 蜂鸣器（1 鸣响）
};

// 121~122 BMS 告警状态（0 未发生 1 发生；告警不触发 MOS 动作）
inline constexpr NamedBit kAlarmBits[] = {
    {0,  "alarm_pack_ov"}, // 总压高压告警
    {1,  "alarm_pack_uv"}, // 总压低压告警
    {2,  "alarm_cell_ov"}, // 单体电芯过压告警
    {3,  "alarm_cell_uv"}, // 单体电芯欠压告警
    {4,  "alarm_vdif"},    // 电芯压差大告警
    {5,  "alarm_chg_oc"},  // 充电过流告警
    {6,  "alarm_dsg_oc"},  // 放电过流告警
    {7,  "alarm_chg_th"},  // 电芯高温禁充告警
    {8,  "alarm_chg_tl"},  // 电芯低温禁充告警
    {9,  "alarm_dsg_th"},  // 电芯高温禁放告警
    {10, "alarm_dsg_tl"},  // 电芯低温禁放告警
    {11, "alarm_env_h"},   // 环境高温告警
    {12, "alarm_env_l"},   // 环境低温告警
    {13, "alarm_mos_h"},   // MOS 高温告警
    {14, "alarm_fbk_oc"},  // 回馈过流告警
};

// 123~124 BMS 保护状态（0 未发生 1 发生；保护断开对应充/放 MOS）
inline constexpr NamedBit kProtectBits[] = {
    {0,  "prot_pack_ov"},   // 总压高压保护
    {1,  "prot_pack_uv"},   // 总压低压保护
    {2,  "prot_cell_ov"},   // 单体电芯过压保护
    {3,  "prot_cell_uv"},   // 单体电芯欠压保护
    {4,  "prot_vdif"},      // 电芯压差大保护
    {5,  "prot_chg_oc1"},   // 充电 1 级过流保护
    {6,  "prot_chg_oc2"},   // 充电 2 级过流保护
    {7,  "prot_dsg_oc1"},   // 放电 1 级过流保护
    {8,  "prot_dsg_oc2"},   // 放电 2 级过流保护
    {9,  "prot_short"},     // 短路保护
    {10, "prot_rev"},       // 反接保护
    {11, "prot_chg_th"},    // 电芯高温禁充
    {12, "prot_chg_tl"},    // 电芯低温禁充
    {13, "prot_dsg_th"},    // 电芯高温禁放
    {14, "prot_dsg_tl"},    // 电芯低温禁放
    {15, "prot_env_h"},     // 环境高温保护
    {16, "prot_env_l"},     // 环境低温保护
    {17, "prot_mos_h"},     // MOS 高温保护
    {18, "prot_chg_time"},  // 充电超时保护
    {19, "prot_chg_oc3"},   // 充电 3 级过流保护
    {20, "prot_dsg_oc3"},   // 放电 3 级过流保护
    {21, "prot_fbk_oc1"},   // 回馈 1 级过流保护
    {22, "prot_fbk_oc2"},   // 回馈 2 级过流保护
    {23, "prot_sub_comm"},  // 从 BMS 通讯丢失保护
    {24, "prot_main_comm"}, // 主 BMS 通讯丢失保护
};

// 125 BMS 故障状态（0 未发生 1 发生；故障断开充放电 MOS）
inline constexpr NamedBit kFaultBits[] = {
    {0, "flt_chg_mos"},  // 充电 MOS 失效故障
    {1, "flt_dsg_mos"},  // 放电 MOS 失效故障
    {2, "flt_cell_ov"},  // 单体电芯电压过高失效
    {3, "flt_cell_uv"},  // 单体电芯电压过低失效
    {4, "flt_cell_th"},  // 电芯温度过高失效
    {5, "flt_ntc"},      // 温度探头失效
    {6, "flt_afe"},      // 模拟前端失效
    {7, "flt_other"},    // BMS 自身其他故障
    {8, "flt_thermal"},  // 热失控告警故障
};

constexpr size_t kSysStateBitCount =
    sizeof(kSysStateBits) / sizeof(kSysStateBits[0]);
constexpr size_t kAlarmBitCount = sizeof(kAlarmBits) / sizeof(kAlarmBits[0]);
constexpr size_t kProtectBitCount =
    sizeof(kProtectBits) / sizeof(kProtectBits[0]);
constexpr size_t kFaultBitCount = sizeof(kFaultBits) / sizeof(kFaultBits[0]);

// ---- 电池编码（100~111，24 字节 ASCII） ----
// 未烧录的包为空格，返回空串；由 read_serial_number() 单独交付（字符串
// 进不了纯 double 的 FieldMap）。
inline std::string parse_sn(const uint8_t* payload, size_t len) {
    if (len < kSnLen) return {};
    std::string sn(reinterpret_cast<const char*>(payload), kSnLen);
    while (!sn.empty() && (sn.back() == ' ' || sn.back() == '\0')) sn.pop_back();
    return sn;
}

// 119~120 均衡字按 bit0~31 展开为 bal_cell1..bal_cell32（0 未均衡 1 均衡中）
inline void expand_balance(FieldMap& map) {
    auto it = map.find("balance");
    if (it == map.end()) return;
    const uint32_t value = static_cast<uint32_t>(it->second);
    for (int i = 0; i < 32; ++i) {
        map["bal_cell" + std::to_string(i + 1)] =
            ((value >> i) & 1u) ? 1.0 : 0.0;
    }
}

// 字段表 → 既有 BatteryStatus 线记录（单位同历史广播：V/A/Ah/0..1）。
// serial_number 是字符串，由 daemon 经 read_serial_number() 填入。
inline void fill_battery_status(const FieldMap& map, bms::BatteryStatus& status) {
    status.voltage = field_or(map, "total_v", 0.0);
    status.current = field_or(map, "total_a", 0.0); // 充电为正放电为负
    status.temperature = field_or(map, "cell_t_max_c", 0.0); // 最高电芯温度 ℃
    status.percentage = field_or(map, "soc_pct", 0.0) / 100.0; // % → 0..1
    status.charge = field_or(map, "remain_ah", 0.0);
    status.capacity = field_or(map, "full_ah", 0.0);
    status.design_capacity = field_or(map, "design_ah", 0.0);
    status.protect_status =
        static_cast<uint32_t>(field_or(map, "protect", 0.0));
    status.work_state = static_cast<uint16_t>(field_or(map, "sys_state", 0.0));
    status.max_cell_voltage = field_or(map, "cell_mv_max", 0.0) / 1000.0; // mV → V
    status.min_cell_voltage = field_or(map, "cell_mv_min", 0.0) / 1000.0;
    status.sw_version = static_cast<uint16_t>(field_or(map, "sw_ver", 0.0));
    status.hw_version = static_cast<uint16_t>(field_or(map, "hw_ver", 0.0));
    status.soh = static_cast<uint16_t>(field_or(map, "soh_pct", 0.0));
    status.cycles = static_cast<uint32_t>(field_or(map, "cycles", 0.0));
    status.io_state = 0;
    status.power_on = (field_or(map, "dsg_mos", 0.0) != 0.0) ? 1 : 0; // 放电 MOS 闭合
}

} // namespace gf_bms
