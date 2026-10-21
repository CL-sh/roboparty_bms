#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SCUD485 全量字段转储：0x61 + 0x31 两帧所有信息一次打印。

实现方式（拼凑，不依赖 bms 代码改动）：
  - daemon 每轮把 0x61/0x31 两帧完整解码成"字段名 -> 数值"映射，
    经 /tmp/bms.sock.fields 广播，客户端 SDK 内缓存这份全量映射；
  - SDK 只暴露单字段 get_field(name)，本脚本按 src/drivers/scud485/
    scud_fields.hpp 的键名表逐个取值拼出全量快照——帧上每个字节都在
    表里，因此拼出来的就是两帧的全部信息（含告警字/状态字位展开）。

用法（bms_py 编译产物在仓库 build/ 目录下）：
    cd <仓库>/build && python3 ../examples/dump_all_fields.py [/tmp/bms.sock]
"""
import math
import os
import sys
import time

# python3 脚本.py 时 sys.path[0] 是脚本所在目录而非当前目录，
# 手动把 cwd 与脚本目录加入查找路径，cd 到 build/ 后直接可用
sys.path.insert(0, os.getcwd())
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import bms_py
except ImportError:
    print('找不到 bms_py：请先 cd 到仓库的 build/ 目录再运行本脚本', file=sys.stderr)
    raise

# ---- 0x61 用户信息帧（BYTE0..BYTE29）----
USER_INFO = [
    'bat_flag', 'chg_ma', 'dis_ma', 'chg_mv', 'bplus_mv', 'pplus_mv',
    'cell_mv_max', 'cell_idx_max', 'cell_mv_min', 'cell_idx_min',
    'cap_mah', 'remain_mah', 'soc_pct', 'cycles', 'vdif_mv',
]

# ---- 0x31 电池摘要帧（BYTE0..BYTE38）----
SUMMARY = (['cell%d_mv' % i for i in range(1, 14)] +
           ['mos_temp', 'cell_temp1', 'cell_temp2',
            'heater_temp1', 'heater_temp2', 'tdiff',
            'sw_version', 'hw_version', 'alarm', 'protect2'])

# ---- alarm 告警字按位展开（0.0/1.0）----
ALARM_BITS = [
    'uv_alarm', 'ov_alarm', 'chg_h_alarm_flag', 'chg_l_alarm_flag',
    'dsg_h_alarm_flag', 'dsg_l_alarm_flag', 'mos_h_alarm_flag',
    'cap_low_alarm', 'cell_volt_dif_alarm', 'cell_temp_dif_alarm',
    'cell_temp_h_alarm', 'cell_temp_l_alarm', 'afe_com_err',
]

# ---- bat_flag 状态字按位展开（0.0/1.0）----
BAT_FLAG_BITS = [
    'sys_dch_on', 'sys_chg_on', 'sys_dch_state', 'sys_chg_state',
    'afe_occ_flag', 'afe_ocd_flag', 'afe_scd_flag', 'afe_uv_flag',
    'afe_ov_flag', 'chg_temp_h_protect', 'chg_temp_l_protect',
    'dch_temp_h_protect', 'dch_temp_l_protect', 'mos_temp_h_protect',
    'dch_pre_state', 'dch_pre_err',
]

ALL_NAMES = USER_INFO + SUMMARY + ALARM_BITS + BAT_FLAG_BITS


def fmt(name, v):
    """打印格式：NaN 标注、状态/告警字带十六进制，其余原值（mA/mV/℃/%）。"""
    if math.isnan(v):
        return 'NaN(未知字段/暂无数据)'
    if name in ('bat_flag', 'alarm', 'protect2'):
        return '%d (0x%04X)' % (int(v), int(v))
    return v


def dump(bms, title, names):
    print('== %s ==' % title)
    for n in names:
        print('  %-20s = %s' % (n, fmt(n, bms.get_field(n))))


def main():
    socket_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/bms.sock'
    bms = bms_py.BmsDriver.create_bms('SCUD485', socket_path)

    # is_connected = socket 已连 且 3 秒内收到过完整快照
    for _ in range(10):
        if bms.is_connected():
            break
        time.sleep(0.5)
    if not bms.is_connected():
        print('daemon 未连接或总线无数据（先启动 bms_daemon）')
        return 1

    print('共 %d 个字段（0x61 %d + 0x31 %d + 位展开 %d）' %
          (len(ALL_NAMES), len(USER_INFO), len(SUMMARY),
           len(ALARM_BITS) + len(BAT_FLAG_BITS)))
    dump(bms, '0x61 用户信息帧', USER_INFO)
    dump(bms, '0x31 电池摘要帧', SUMMARY)
    dump(bms, 'alarm 告警位展开 (0=无 1=有)', ALARM_BITS)
    dump(bms, 'bat_flag 状态位展开 (0=关 1=开)', BAT_FLAG_BITS)
    return 0


if __name__ == '__main__':
    sys.exit(main())
