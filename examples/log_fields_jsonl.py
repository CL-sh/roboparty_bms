#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SCUD485 全字段稳定性记录器：每 1 秒一条 JSON Lines，长时间落盘。

用途：稳定性测试——连续记录 0x61+0x31 两帧全部 67 个字段（含位展开），
供 examples/analyze_fields_jsonl.py 事后分析掉线/缺口/冻结/异常值。

⚠️ SIGPIPE 铁律（daemon 已知缺陷：客户端断开会杀死 daemon）：
  1. 本记录器一次连接长跑到底，不重建对象、不中途退出；
  2. 运行窗口内严禁任何其他 bms_py 客户端连接后断开
     （dump_all_fields.py、第二个记录器、任何试探脚本）；
  3. 记录器正常退出后 daemon 会因 SIGPIPE 死亡——属预期现象。

用法（bms_py 编译产物在仓库 build/ 目录下）：
    cd <仓库>/build
    python3 -u ../examples/log_fields_jsonl.py --hours 2 \\
        --out ~/bms_logs/scud485_$(date +%Y%m%d_%H%M%S).jsonl
短测：--records 120 --out /tmp/scud485_smoke.jsonl

每行 schema（紧凑 JSON）：
  {"ts_ms":…,"seq":i,"conn":bool,"changed":int|null,"nan":n,"lag_ms":n,"f":{…67 键…}}
  seq      记录序号（连续性唯一真相，防时钟跳变误判缺口）
  conn     is_connected()（socket 连通 AND 数据 ≤3s 新鲜）
  changed  与上一条不同的字段数（冻结启发式；首条 null）
  nan      本条 f 中 null 个数（67=尚无字段快照）
  lag_ms   实际写盘时刻相对目标节拍的延迟（调度卡顿审计）
  f        字段名 -> 数值（NaN 落 null，整数存 int）
"""
import argparse
import json
import math
import os
import signal
import sys
import time

# python3 脚本.py 时 sys.path[0] 是脚本目录而非当前目录，
# 手动把 cwd 和脚本目录加入查找路径，cd 到 build/ 后直接可用
sys.path.insert(0, os.getcwd())
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import bms_py
except ImportError:
    print('找不到 bms_py：请先 cd 到仓库的 build/ 目录再运行本脚本', file=sys.stderr)
    raise

# ---- 字段名表（与 src/drivers/scud485/scud_fields.hpp 同步；此处复制而非
#      import dump_all_fields，因后者顶层强制 import bms_py，有导入路径坑）----

# 0x61 用户信息帧（BYTE0..BYTE29）
USER_INFO = [
    'bat_flag', 'chg_ma', 'dis_ma', 'chg_mv', 'bplus_mv', 'pplus_mv',
    'cell_mv_max', 'cell_idx_max', 'cell_mv_min', 'cell_idx_min',
    'cap_mah', 'remain_mah', 'soc_pct', 'cycles', 'vdif_mv',
]

# 0x31 电池摘要帧（BYTE0..BYTE38）
SUMMARY = (['cell%d_mv' % i for i in range(1, 14)] +
           ['mos_temp', 'cell_temp1', 'cell_temp2',
            'heater_temp1', 'heater_temp2', 'tdiff',
            'sw_version', 'hw_version', 'alarm', 'protect2'])

# alarm 告警字按位展开（0.0/1.0）
ALARM_BITS = [
    'uv_alarm', 'ov_alarm', 'chg_h_alarm_flag', 'chg_l_alarm_flag',
    'dsg_h_alarm_flag', 'dsg_l_alarm_flag', 'mos_h_alarm_flag',
    'cap_low_alarm', 'cell_volt_dif_alarm', 'cell_temp_dif_alarm',
    'cell_temp_h_alarm', 'cell_temp_l_alarm', 'afe_com_err',
]

# bat_flag 状态字按位展开（0.0/1.0）
BAT_FLAG_BITS = [
    'sys_dch_on', 'sys_chg_on', 'sys_dch_state', 'sys_chg_state',
    'afe_occ_flag', 'afe_ocd_flag', 'afe_scd_flag', 'afe_uv_flag',
    'afe_ov_flag', 'chg_temp_h_protect', 'chg_temp_l_protect',
    'dch_temp_h_protect', 'dch_temp_l_protect', 'mos_temp_h_protect',
    'dch_pre_state', 'dch_pre_err',
]

ALL_NAMES = USER_INFO + SUMMARY + ALARM_BITS + BAT_FLAG_BITS
# 单一事实来源：src/drivers/scud485/scud_fields.hpp
assert len(ALL_NAMES) == 67 and len(set(ALL_NAMES)) == 67


class StopLogging(Exception):
    """SIGINT/SIGTERM 触发的优雅收尾（sleep 被 PEP 475 续睡，置 flag 无效）。"""


def encode_value(v):
    """float -> JSON 安全值：NaN->None，整数值存 int，其余保留 3 位小数。"""
    if v is None or math.isnan(v):
        return None
    if v == int(v) and abs(v) < 2 ** 31:
        return int(v)
    return round(v, 3)


def snapshot(bms, names):
    """取一轮全量快照：(conn, 字段 dict, nan 计数)。只读缓存，微秒级。"""
    conn = bool(bms.is_connected())
    f = {}
    nan_count = 0
    for n in names:
        ev = encode_value(bms.get_field(n))
        if ev is None:
            nan_count += 1
        f[n] = ev
    return conn, f, nan_count


def count_changed(prev, cur):
    """与上一条不同的字段个数（None==None 视为相同）；首条返回 None。"""
    if prev is None:
        return None
    n = 0
    for k, v in cur.items():
        if prev.get(k, object()) != v:
            n += 1
    return n


def parse_args():
    p = argparse.ArgumentParser(description='SCUD485 全字段 JSONL 稳定性记录器')
    p.add_argument('--out', default=None,
                   help='输出 JSONL 路径（默认 ./scud485_fields_<UTC>.jsonl）')
    p.add_argument('--hours', type=float, default=2.0, help='记录时长小时数（默认 2.0）')
    p.add_argument('--interval', type=float, default=1.0, help='采样节拍秒数（默认 1.0）')
    p.add_argument('--records', type=int, default=None,
                   help='直接指定条数（短测用；给定则覆盖 --hours）')
    p.add_argument('--socket', default='/tmp/bms.sock', help='daemon socket 路径')
    p.add_argument('--gate-timeout', type=float, default=15.0,
                   help='启动门禁：等待 is_connected 的秒数（默认 15）')
    p.add_argument('--pidfile', default=None,
                   help='把本进程真实 PID 写入该文件（setsid 启动时 $! 拿不到，供监控用）')
    args = p.parse_args()
    if args.out is None:
        args.out = './scud485_fields_%s.jsonl' % time.strftime('%Y%m%d_%H%M%S', time.gmtime())
    if args.records is None:
        args.records = max(1, round(args.hours * 3600.0 / args.interval))
    return args


def init_stats(t0, args):
    return {
        't0': t0, 't_end': None, 'expected': args.records,
        'written': 0, 'sample_errors': 0,
        'conn_false': 0, 'conn_runs': [], 'cur_conn_run': None,
        'freeze_runs': [], 'cur_freeze': None,
        'nan_gt0': 0, 'nan_eq67': 0, 'nan_max': 0,
        'lag_max': 0, 'lag_gt500': 0,
        'ts_first': None, 'ts_last': None,
    }


def track_runs(stats, rec):
    """增量维护 conn=false 段与 changed==0 冻结段。"""
    # conn=false 段
    if not rec['conn']:
        stats['conn_false'] += 1
        if stats['cur_conn_run'] is None:
            stats['cur_conn_run'] = [rec['seq'], rec['seq']]
        else:
            stats['cur_conn_run'][1] = rec['seq']
    elif stats['cur_conn_run'] is not None:
        stats['conn_runs'].append(stats['cur_conn_run'])
        stats['cur_conn_run'] = None
    # changed==0 冻结段（含 changed 为 0 的连续段）
    if rec['changed'] == 0:
        if stats['cur_freeze'] is None:
            stats['cur_freeze'] = [rec['seq'], rec['seq']]
        else:
            stats['cur_freeze'][1] = rec['seq']
    elif stats['cur_freeze'] is not None:
        stats['freeze_runs'].append(stats['cur_freeze'])
        stats['cur_freeze'] = None


def update_stats(stats, rec):
    stats['written'] += 1
    if stats['ts_first'] is None:
        stats['ts_first'] = rec['ts_ms']
    stats['ts_last'] = rec['ts_ms']
    if rec['nan'] > 0:
        stats['nan_gt0'] += 1
    if rec['nan'] == 67:
        stats['nan_eq67'] += 1
    if rec['nan'] > stats['nan_max']:
        stats['nan_max'] = rec['nan']
    if rec['lag_ms'] > stats['lag_max']:
        stats['lag_max'] = rec['lag_ms']
    if rec['lag_ms'] > 500:
        stats['lag_gt500'] += 1
    track_runs(stats, rec)


def wait_for_connect(bms, timeout):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if bms.is_connected():
            return True
        time.sleep(0.5)
    return bms.is_connected()


def run_logger(bms, args, fh, stats):
    names = ALL_NAMES
    t0 = time.monotonic()
    prev_f = None
    for i in range(args.records):
        target = t0 + i * args.interval
        now = time.monotonic()
        if now < target:
            time.sleep(target - now)  # 信号处理器抛 StopLogging 打断
        ts_ms = int(time.time() * 1000)
        lag_ms = int(max(0.0, (time.monotonic() - target) * 1000))
        try:
            conn, f, nan_count = snapshot(bms, names)
        except Exception:
            # 铁律：主循环永不因采样异常退出
            conn, f, nan_count = False, {k: None for k in names}, len(names)
            stats['sample_errors'] += 1
            if stats['sample_errors'] <= 3:
                print('[log_fields] 采样异常（第 %d 次）' % stats['sample_errors'],
                      file=sys.stderr)
        changed = count_changed(prev_f, f)
        rec = {'ts_ms': ts_ms, 'seq': i, 'conn': conn,
               'changed': changed, 'nan': nan_count, 'lag_ms': lag_ms, 'f': f}
        # allow_nan=False 是安全闸：编码路径漏了 NaN 会在短测立刻抛错
        fh.write(json.dumps(rec, separators=(',', ':'),
                            ensure_ascii=True, allow_nan=False) + '\n')
        fh.flush()  # 1 Hz 下成本可忽略，崩溃最多损失半行
        prev_f = f
        update_stats(stats, rec)


def _fmt_ts(ms):
    return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(ms / 1000.0))


def print_summary(stats, args, out_path):
    def p(*a):
        print(*a, file=sys.stderr)

    runs = stats['conn_runs'] + ([stats['cur_conn_run']]
                                 if stats['cur_conn_run'] else [])
    freezes = stats['freeze_runs'] + ([stats['cur_freeze']]
                                      if stats['cur_freeze'] else [])
    long_freezes = sorted((r for r in freezes if r[1] - r[0] + 1 >= 5),
                          key=lambda r: r[0] - r[1])
    dur = (stats['t_end'] - stats['t0']) if stats['t_end'] else 0.0
    p('======== 记录摘要 ========')
    p('输出文件   : %s' % out_path)
    if os.path.exists(out_path):
        p('文件大小   : %.2f MB / %d 行' %
          (os.path.getsize(out_path) / 1e6, stats['written']))
    p('记录条数   : %d / 期望 %d（节拍 %.3fs）' %
      (stats['written'], stats['expected'], args.interval))
    p('实际时长   : %.1f s（墙钟 %s ~ %s）' %
      (dur, _fmt_ts(stats['ts_first']) if stats['ts_first'] else '?',
       _fmt_ts(stats['ts_last']) if stats['ts_last'] else '?'))
    p('conn=false : %d 条，%d 段，最长 %s' %
      (stats['conn_false'], len(runs),
       ('%d 条 (seq %d..%d)' % (r[1] - r[0] + 1, r[0], r[1])) if runs else '无'))
    p('冻结段     : changed==0 连续≥5 条的段 %d 个，最长 %s' %
      (len(long_freezes),
       ('%d 条 (seq %d..%d)' % (long_freezes[0][1] - long_freezes[0][0] + 1,
                                long_freezes[0][0], long_freezes[0][1]))
       if long_freezes else '无'))
    for r in long_freezes[:5]:
        p('             - seq %d..%d（%d 条）' % (r[0], r[1], r[1] - r[0] + 1))
    p('NaN 统计   : nan>0 的记录 %d 条，nan==67（无快照）%d 条，最大 nan=%d' %
      (stats['nan_gt0'], stats['nan_eq67'], stats['nan_max']))
    p('调度延迟   : lag_ms 最大 %d，>500ms 的 %d 次' %
      (stats['lag_max'], stats['lag_gt500']))
    p('采样异常   : %d 次' % stats['sample_errors'])
    p('提示：记录器退出后 daemon 将因 SIGPIPE 死亡，属预期现象（已知 bug）。')
    p('==========================')


def main():
    args = parse_args()
    if args.pidfile:
        with open(args.pidfile, 'w') as pf:
            pf.write(str(os.getpid()))
    print('[log_fields] out=%s records=%d interval=%.3fs 预计 %.1f 分钟' %
          (args.out, args.records, args.interval,
           args.records * args.interval / 60.0), file=sys.stderr)

    try:
        bms = bms_py.BmsDriver.create_bms('SCUD485', args.socket)
    except Exception as e:
        print('[log_fields] 创建客户端失败：%s' % e, file=sys.stderr)
        return 2

    if not wait_for_connect(bms, args.gate_timeout):
        print('[log_fields] 门禁失败：%ds 内未连上 daemon（is_connected 为假），'
              '请先确认 bms_daemon 在跑，退出。' % args.gate_timeout,
              file=sys.stderr)
        return 2
    print('[log_fields] 已连接，开始记录（Ctrl+C 优雅收尾）', file=sys.stderr)

    def _sig(_signum, _frame):
        raise StopLogging()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    stats = init_stats(time.monotonic(), args)
    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    with open(args.out, 'w', buffering=1) as fh:
        try:
            run_logger(bms, args, fh, stats)
        except StopLogging:
            print('[log_fields] 收到停止信号，收尾…', file=sys.stderr)
        finally:
            stats['t_end'] = time.monotonic()
    print_summary(stats, args, args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
