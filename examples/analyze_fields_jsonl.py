#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SCUD485 稳定性日志分析器：分析 log_fields_jsonl.py 产出的 JSONL。

检查项：
  - 缺口：seq 必连续（唯一真相）；相邻 ts_ms 差 > 1.5×interval 为 logger 缺口，
    dt<=0 为时钟跳变（NTP 校时，单列不混入缺口）
  - 掉线：conn=false 连续段（次数/最长/总时长）
  - 冻结：changed==0 连续 ≥ --freeze-secs 为一段（1Hz 相位差会有单条 0，不判单条）；
    另列"单字段静止 >10 分钟"嫌疑清单（含恒真常量，供人裁决）
  - NaN：per-field null 统计；nan==67 段（无字段快照）
  - 越界：字段物理范围校验（电压/电流/温度/位字段等）

用法：
    python3 examples/analyze_fields_jsonl.py LOG.jsonl [--interval 1.0]
        [--freeze-secs 5] [--static-secs 600] [--json]
纯标准库；坏行/崩溃截断尾行跳过并计数。
"""
import argparse
import json
import sys

BIT_FIELDS = [
    'uv_alarm', 'ov_alarm', 'chg_h_alarm_flag', 'chg_l_alarm_flag',
    'dsg_h_alarm_flag', 'dsg_l_alarm_flag', 'mos_h_alarm_flag',
    'cap_low_alarm', 'cell_volt_dif_alarm', 'cell_temp_dif_alarm',
    'cell_temp_h_alarm', 'cell_temp_l_alarm', 'afe_com_err',
    'sys_dch_on', 'sys_chg_on', 'sys_dch_state', 'sys_chg_state',
    'afe_occ_flag', 'afe_ocd_flag', 'afe_scd_flag', 'afe_uv_flag',
    'afe_ov_flag', 'chg_temp_h_protect', 'chg_temp_l_protect',
    'dch_temp_h_protect', 'dch_temp_l_protect', 'mos_temp_h_protect',
    'dch_pre_state', 'dch_pre_err',
]


def range_rule(name):
    """字段名 -> (min, max) 或 None（不校验）。"""
    if name.startswith('cell') and name.endswith('_mv') or name in (
            'cell_mv_max', 'cell_mv_min'):
        return (2000, 4500)
    if name in ('bplus_mv', 'pplus_mv', 'chg_mv'):
        return (0, 60000)
    if name in ('chg_ma', 'dis_ma'):
        return (-100000, 100000)
    if name == 'soc_pct':
        return (0, 100)
    if 'temp' in name or name == 'tdiff':
        return (-40, 120)
    if name in ('cap_mah', 'remain_mah'):
        return (0, 1000000)
    if name in ('cycles', 'cell_idx_max', 'cell_idx_min', 'vdif_mv'):
        return (0, 65535)
    if name in ('bat_flag', 'alarm'):
        return (0, 65535)
    if name == 'protect2':
        return (0, 255)
    if name in BIT_FIELDS:
        return (0, 1)
    return None


def iter_records(path):
    """逐行解析；坏行/截断尾行跳过并计数。"""
    bad = 0
    with open(path, 'r', encoding='utf-8', errors='replace') as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except ValueError:
                bad += 1
    iter_records.bad_lines = bad


def analyze(path, interval, freeze_secs, static_secs):
    rep = {
        'file': path, 'interval': interval,
        'records': 0, 'bad_lines': 0,
        'gaps': [], 'clock_jumps': 0, 'seq_breaks': [],
        'conn_false_total': 0, 'conn_runs': [],
        'freeze_runs': [], 'nan67_runs': [],
        'nan_by_field': {}, 'out_of_range': {},
        'static_fields': [],
        'seq_first': None, 'seq_last': None,
        'ts_first': None, 'ts_last': None,
    }
    prev = None
    cur_conn_run = None
    cur_freeze = None
    cur_nan67 = None
    last_change = {}          # 字段 -> (ts_ms, seq)
    oor_samples = {}          # 字段 -> 样例列表

    def close_runs(rec):
        nonlocal cur_conn_run, cur_freeze, cur_nan67
        for flag, run in (('conn', cur_conn_run), ('freeze', cur_freeze),
                          ('nan67', cur_nan67)):
            if run is not None:
                key = {'conn': 'conn_runs', 'freeze': 'freeze_runs',
                       'nan67': 'nan67_runs'}[flag]
                rep[key].append(run)
        cur_conn_run = cur_freeze = cur_nan67 = None

    for rec in iter_records(path):
        rep['records'] += 1
        seq, ts = rec.get('seq'), rec.get('ts_ms')
        if rep['seq_first'] is None:
            rep['seq_first'], rep['ts_first'] = seq, ts
        rep['seq_last'], rep['ts_last'] = seq, ts

        # --- 缺口 / seq 连续性 ---
        if prev is not None:
            if seq != prev['seq'] + 1:
                rep['seq_breaks'].append((prev['seq'], seq))
            dt = ts - prev['ts_ms']
            if dt <= 0:
                rep['clock_jumps'] += 1
            elif dt > 1.5 * interval * 1000:
                rep['gaps'].append((prev['seq'], seq, dt))

        # --- conn=false 段 ---
        if not rec.get('conn'):
            rep['conn_false_total'] += 1
            if cur_conn_run is None:
                cur_conn_run = {'seq': [seq, seq], 'ts_ms': ts}
            else:
                cur_conn_run['seq'][1] = seq
        elif cur_conn_run is not None:
            rep['conn_runs'].append(cur_conn_run)
            cur_conn_run = None

        # --- 冻结段（changed==0 连续）---
        if rec.get('changed') == 0:
            if cur_freeze is None:
                cur_freeze = {'seq': [seq, seq], 'ts_ms': ts}
            else:
                cur_freeze['seq'][1] = seq
        elif cur_freeze is not None:
            rep['freeze_runs'].append(cur_freeze)
            cur_freeze = None

        # --- nan==67 段（无字段快照）---
        if rec.get('nan') == 67:
            if cur_nan67 is None:
                cur_nan67 = {'seq': [seq, seq], 'ts_ms': ts}
            else:
                cur_nan67['seq'][1] = seq
        elif cur_nan67 is not None:
            rep['nan67_runs'].append(cur_nan67)
            cur_nan67 = None

        # --- per-field：null 计数、变化追踪、越界 ---
        for name, val in rec.get('f', {}).items():
            if val is None:
                rep['nan_by_field'][name] = rep['nan_by_field'].get(name, 0) + 1
            if last_change.get(name, (None,))[0] is None or \
                    (prev is not None and prev.get('f', {}).get(name) != val):
                last_change[name] = (ts, seq)
            rule = range_rule(name)
            if val is not None and rule is not None and \
                    not (rule[0] <= val <= rule[1]):
                rep['out_of_range'][name] = rep['out_of_range'].get(name, 0) + 1
                s = oor_samples.setdefault(name, [])
                if len(s) < 5:
                    s.append({'seq': seq, 'val': val})

        prev = rec

    close_runs(prev)
    rep['bad_lines'] = getattr(iter_records, 'bad_lines', 0)

    # 冻结段只保留 ≥ freeze_secs 条的
    min_freeze = max(2, int(round(freeze_secs / interval)))
    rep['freeze_runs'] = [r for r in rep['freeze_runs']
                          if r['seq'][1] - r['seq'][0] + 1 >= min_freeze]

    # 单字段静止 > static_secs 的嫌疑清单
    if rep['ts_last'] is not None:
        for name, (ts, seq) in sorted(last_change.items()):
            static_s = (rep['ts_last'] - ts) / 1000.0
            if static_s >= static_secs:
                rep['static_fields'].append(
                    {'field': name, 'static_secs': round(static_s, 1),
                     'since_seq': seq})

    rep['oor_samples'] = oor_samples
    return rep


def _run_len(r):
    return r['seq'][1] - r['seq'][0] + 1


def report_text(rep, interval):
    p = print
    p('======== 稳定性分析报告 ========')
    p('文件       : %s' % rep['file'])
    p('记录       : %d 条（seq %s..%s），坏行 %d' %
      (rep['records'], rep['seq_first'], rep['seq_last'], rep['bad_lines']))
    if rep['ts_first'] is not None:
        dur = (rep['ts_last'] - rep['ts_first']) / 1000.0
        p('时间跨度   : %.1f s（%.2f 小时）' % (dur, dur / 3600.0))
    p('')
    p('-- 缺口 --')
    p('logger 缺口: %d 个，总缺失 %.1f s，最长 %s' %
      (len(rep['gaps']),
       sum(g[2] for g in rep['gaps']) / 1000.0,
       ('%.1f s (seq %d→%d)' % (max((g[2] for g in rep['gaps']), default=0) / 1000.0,
                                *rep['gaps'][0][:2])) if rep['gaps'] else '无'))
    for g in sorted(rep['gaps'], key=lambda g: -g[2])[:5]:
        p('  - seq %d→%d  %.1f s' % (g[0], g[1], g[2] / 1000.0))
    p('seq 断裂   : %d 处 %s' %
      (len(rep['seq_breaks']), rep['seq_breaks'][:5] if rep['seq_breaks'] else ''))
    p('时钟跳变   : %d 次（dt<=0，NTP 校时）' % rep['clock_jumps'])
    p('')
    p('-- 掉线（conn=false）--')
    runs = sorted(rep['conn_runs'], key=_run_len, reverse=True)
    p('总条数     : %d，段数 %d，最长 %s' %
      (rep['conn_false_total'], len(runs),
       ('%d 条 (seq %d..%d)' % (_run_len(runs[0]), *runs[0]['seq'])) if runs else '无'))
    for r in runs[:5]:
        p('  - seq %d..%d（%d 条 ≈ %.0f s）' %
          (r['seq'][0], r['seq'][1], _run_len(r), _run_len(r) * interval))
    p('')
    p('-- 冻结（changed==0 连续段）--')
    fr = sorted(rep['freeze_runs'], key=_run_len, reverse=True)
    p('≥阈值的段  : %d，最长 %s' %
      (len(fr), ('%d 条' % _run_len(fr[0])) if fr else '无'))
    for r in fr[:5]:
        p('  - seq %d..%d（%d 条 ≈ %.0f s）' %
          (r['seq'][0], r['seq'][1], _run_len(r), _run_len(r) * interval))
    if rep['nan67_runs']:
        p('无快照段   : %d 段（nan==67）' % len(rep['nan67_runs']))
        for r in rep['nan67_runs'][:5]:
            p('  - seq %d..%d（%d 条）' % (r['seq'][0], r['seq'][1], _run_len(r)))
    p('')
    p('-- NaN --')
    if rep['nan_by_field']:
        top = sorted(rep['nan_by_field'].items(), key=lambda kv: -kv[1])[:10]
        p('per-field null 计数 top10: %s' % top)
    else:
        p('无 null 字段')
    p('')
    p('-- 越界 --')
    if rep['out_of_range']:
        for name, cnt in sorted(rep['out_of_range'].items(), key=lambda kv: -kv[1]):
            rule = range_rule(name)
            p('  - %-20s %d 次，合法范围 %s，样例 %s' %
              (name, cnt, rule, rep['oor_samples'].get(name, [])[:3]))
    else:
        p('无越界值')
    p('')
    p('-- 单字段静止嫌疑（供人裁决，恒真常量也会上榜）--')
    if rep['static_fields']:
        for s in sorted(rep['static_fields'],
                        key=lambda s: -s['static_secs'])[:15]:
            p('  - %-20s 静止 %.0f s（自 seq %d）' %
              (s['field'], s['static_secs'], s['since_seq']))
    else:
        p('无')
    p('================================')


def main():
    ap = argparse.ArgumentParser(description='SCUD485 稳定性 JSONL 分析器')
    ap.add_argument('log', help='log_fields_jsonl.py 产出的 .jsonl')
    ap.add_argument('--interval', type=float, default=1.0, help='记录节拍秒数')
    ap.add_argument('--freeze-secs', type=float, default=5.0,
                    help='冻结段最短秒数（默认 5）')
    ap.add_argument('--static-secs', type=float, default=600.0,
                    help='单字段静止嫌疑阈值秒数（默认 600）')
    ap.add_argument('--json', action='store_true', help='输出机器可读 JSON 报告')
    args = ap.parse_args()

    rep = analyze(args.log, args.interval, args.freeze_secs, args.static_secs)
    if args.json:
        print(json.dumps(rep, ensure_ascii=False, indent=2))
    else:
        report_text(rep, args.interval)
    return 0


if __name__ == '__main__':
    sys.exit(main())
