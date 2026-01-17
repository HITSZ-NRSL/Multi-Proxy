#!/usr/bin/env python3
"""
analyze_docker_cpu_mem.py

按 container_id 统计：
  - CPU 平均值 (百分比) 与 最大值
  - 内存 used 平均值 (bytes) 与 最大值

输出漂亮对齐的 ASCII 表格，支持 --human 单位显示与 --out 保存 summary CSV。

用法:
  python3 analyze_docker_cpu_mem.py stats.csv [--human] [--out summary.csv]
"""
import csv, sys, math, argparse
from collections import defaultdict
from statistics import mean

# 单位映射（用于 parse）
UNIT_MAP = {
    'B': 1,
    'KB': 1000, 'KIB': 1024,
    'MB': 1000**2, 'MIB': 1024**2,
    'GB': 1000**3, 'GIB': 1024**3,
    'TB': 1000**4, 'TIB': 1024**4,
    'kB': 1000, 'KiB': 1024, 'MiB': 1024**2, 'GiB': 1024**3
}

def parse_percent(s):
    if s is None: return None
    s = str(s).strip()
    if not s: return None
    if s.endswith('%'): s = s[:-1]
    try:
        return float(s)
    except:
        return None

def parse_size_token(tok):
    if tok is None: return None
    t = str(tok).strip()
    if not t: return None
    t = t.replace(' ', '')
    num = ''
    i = 0
    while i < len(t) and (t[i].isdigit() or t[i] in '.-'):
        num += t[i]; i += 1
    unit = t[i:].upper() or 'B'
    try:
        val = float(num)
    except:
        return None
    factor = UNIT_MAP.get(unit, None)
    if factor is None:
        # fallback: return numeric
        return val
    return val * factor

def parse_mem_used_field(field):
    """'207.9MiB / 31.15GiB' -> used_bytes (float)"""
    if field is None: return None
    parts = field.split('/')
    if len(parts) == 0: return None
    return parse_size_token(parts[0])

def bytes_to_human(n):
    if n is None or (isinstance(n, float) and math.isnan(n)): return "N/A"
    n = float(n)
    for unit in ['B','KiB','MiB','GiB','TiB']:
        if abs(n) < 1024.0:
            if unit == 'B':
                return f"{n:.0f}{unit}"
            else:
                return f"{n:.2f}{unit}"
        n /= 1024.0
    return f"{n:.2f}PiB"

def analyze(csv_file):
    data = defaultdict(lambda: {'name':'', 'cpu': [], 'mem_used': []})
    with open(csv_file, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            cid = row.get('container_id') or row.get('container') or row.get('Container')
            if not cid:
                continue
            name = row.get('name','')
            data[cid]['name'] = name
            cpu = parse_percent(row.get('cpu_perc',''))
            if cpu is not None:
                data[cid]['cpu'].append(cpu)
            mem_used = parse_mem_used_field(row.get('mem_usage',''))
            if mem_used is not None:
                data[cid]['mem_used'].append(mem_used)
    return data

def make_rows(data, human=False):
    header = ("container_id","name","N",
              "cpu_mean(%)","cpu_max(%)",
              "mem_mean","mem_max")
    rows = []
    for cid, s in data.items():
        cpu_list = s['cpu']
        mem_list = s['mem_used']
        N = max(1, len(cpu_list) or len(mem_list))
        cpu_mean = mean(cpu_list) if cpu_list else float('nan')
        cpu_max = max(cpu_list) if cpu_list else float('nan')
        mem_mean = mean(mem_list) if mem_list else float('nan')
        mem_max = max(mem_list) if mem_list else float('nan')

        if human:
            mem_mean_str = bytes_to_human(mem_mean)
            mem_max_str = bytes_to_human(mem_max)
        else:
            mem_mean_str = f"{mem_mean:.1f}" if not (mem_mean is None or (isinstance(mem_mean,float) and math.isnan(mem_mean))) else "N/A"
            mem_max_str = f"{mem_max:.1f}" if not (mem_max is None or (isinstance(mem_max,float) and math.isnan(mem_max))) else "N/A"

        row = [
            cid,
            s['name'],
            str(N),
            f"{cpu_mean:.3f}" if not (math.isnan(cpu_mean)) else "N/A",
            f"{cpu_max:.3f}" if not (math.isnan(cpu_max)) else "N/A",
            mem_mean_str,
            mem_max_str
        ]
        rows.append(row)
    rows.sort(key=lambda r: r[1] or r[0])
    return header, rows

def print_table(header, rows):
    cols = [header] + rows
    widths = [0]*len(header)
    for row in cols:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(str(cell)))
    # border
    sep = "+" + "+".join(["-"*(w+2) for w in widths]) + "+"
    print(sep)
    # header
    header_cells = [" " + str(h).ljust(widths[i]) + " " for i,h in enumerate(header)]
    print("|" + "|".join(header_cells) + "|")
    print(sep)
    # rows
    for row in rows:
        cells = []
        for i, cell in enumerate(row):
            s = str(cell)
            # numeric columns: cpu_mean/cpu_max, N, mem_* are considered numeric except name
            if i in (2,3,4):  # N, cpu_mean, cpu_max -> right align
                cells.append(" " + s.rjust(widths[i]) + " ")
            elif i in (5,6):  # mem columns -> right align
                cells.append(" " + s.rjust(widths[i]) + " ")
            else:
                cells.append(" " + s.ljust(widths[i]) + " ")
        print("|" + "|".join(cells) + "|")
    print(sep)

def write_csv(out_csv, header, rows):
    with open(out_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for r in rows:
            writer.writerow(r)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("csvfile", help="CSV file from docker_stats_recorder.sh")
    p.add_argument("--human", action='store_true', help="show memory in human readable units")
    p.add_argument("--out", help="save summary CSV")
    args = p.parse_args()

    data = analyze(args.csvfile)
    header, rows = make_rows(data, human=args.human)
    print_table(header, rows)
    if args.out:
        write_csv(args.out, header, rows)
        print(f"\nWrote summary CSV to {args.out}")

if __name__ == "__main__":
    main()
