#!/usr/bin/env python3
"""
jitter_imu_lidar_seq.py — Jitter IMU e LiDAR Mid-360 via DDS (Unitree SDK2)
=============================================================================
Coleta sequencialmente:
  1. IMU do robô      → tópico: rt/lf/lowstate
  2. LiDAR Mid-360    → tópico: rt/utlidar/cloud_livox_mid360

Cada fase roda isolada (sem subscriber concorrente) para não inflar o jitter.

Uso:
  export LD_LIBRARY_PATH=/home/unitree/cyclonedds_ws/install/cyclonedds/lib:$LD_LIBRARY_PATH
  python3 jitter_imu_lidar_seq.py --duration 120 --output ~/data/jitter
"""
#source /opt/ros/foxy/setup.bash
#export LD_LIBRARY_PATH=/home/unitree/cyclonedds_ws/install/cyclonedds/lib:$LD_LIBRARY_PATH
#top -b -n 1 | head -30

import argparse
import csv
import math
import os
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

try:
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    LIDAR_MSG  = PointCloud2_
    STAMP_MODE = 'header'
except ImportError:
    try:
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LidarState_
        LIDAR_MSG  = LidarState_
        STAMP_MODE = 'none'
    except ImportError:
        LIDAR_MSG  = None
        STAMP_MODE = 'none'
    print('[AVISO] PointCloud2_ não encontrado — timestamp interno do LiDAR indisponível.')

LIDAR_TOPIC = 'utlidar/cloud_livox_mid360'


# ──────────────────────────────────────────────────────────────────────────────
# Utilitários
# ──────────────────────────────────────────────────────────────────────────────

def mono() -> float:
    return time.monotonic()


def stats(values):
    v = [x for x in values if not math.isnan(x)]
    if not v:
        return None
    v.sort()
    n   = len(v)
    avg = sum(v) / n
    std = math.sqrt(sum((x - avg) ** 2 for x in v) / n)
    return {
        'n':    n,
        'min':  v[0],
        'max':  v[-1],
        'mean': avg,
        'std':  std,
        'p50':  v[int(n * 0.50)],
        'p95':  v[int(n * 0.95)],
        'p99':  v[int(n * 0.99)],
    }


def print_stats(label, s):
    if not s:
        print(f'  {label}: sem dados')
        return
    print(f'  {label}')
    print(f'    n={s["n"]}')
    print(f'    min={s["min"]*1e3:.3f}ms   max={s["max"]*1e3:.3f}ms')
    print(f'    mean={s["mean"]*1e3:.3f}ms  std={s["std"]*1e3:.3f}ms')
    print(f'    p50={s["p50"]*1e3:.3f}ms   p95={s["p95"]*1e3:.3f}ms   p99={s["p99"]*1e3:.3f}ms')


def save_csv(path, fields, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    print(f'  CSV salvo: {path} ({len(rows)} amostras)')


def progress_loop(buf, lock, duration, stop_event, label='amostras'):
    """Imprime progresso até duration expirar ou stop_event ser setado."""
    t0 = mono()
    try:
        while not stop_event.is_set():
            elapsed = mono() - t0
            if duration > 0 and elapsed >= duration:
                break
            with lock:
                n = len(buf)
            print(f'  [{elapsed:6.1f}s] {label}: {n:6d}', end='\r', flush=True)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print('\n  Interrompido pelo usuário.')
        stop_event.set()
    print()


# ──────────────────────────────────────────────────────────────────────────────
# Fase 1 — IMU (rt/lf/lowstate)
# ──────────────────────────────────────────────────────────────────────────────

def run_imu(args):
    print('╔══════════════════════════════════════════════════════════════╗')
    print('║  FASE 1 — IMU  (rt/lf/lowstate)                            ║')
    print('╚══════════════════════════════════════════════════════════════╝')

    buf  = []
    lock = threading.Lock()
    stop = threading.Event()
    last = {'recv': None, 'tick': None}

    def handler(msg):
        if stop.is_set():
            return
        t_recv = mono()
        tick   = msg.tick
        with lock:
            inter_recv = (t_recv - last['recv']) if last['recv'] is not None else float('nan')
            inter_tick = ((tick  - last['tick']) / 1000.0) if last['tick'] is not None else float('nan')
            last['recv'] = t_recv
            last['tick'] = tick
            buf.append({
                't_recv':     t_recv,
                'tick_hw':    tick,
                'inter_recv': inter_recv,
                'inter_tick': inter_tick,
                'acc_x': msg.imu_state.accelerometer[0],
                'acc_y': msg.imu_state.accelerometer[1],
                'acc_z': msg.imu_state.accelerometer[2],
                'gyr_x': msg.imu_state.gyroscope[0],
                'gyr_y': msg.imu_state.gyroscope[1],
                'gyr_z': msg.imu_state.gyroscope[2],
            })

    sub = ChannelSubscriber('rt/lf/lowstate', LowState_)
    sub.Init(handler, 10)

    progress_loop(buf, lock, args.duration, stop, label='amostras IMU')
    stop.set()
    time.sleep(0.2)   # drena callbacks pendentes
    del sub           # encerra subscriber antes do próximo

    with lock:
        snapshot = list(buf)

    if not snapshot:
        print('  Nenhum dado coletado no IMU.')
        return

    s_recv = stats([r['inter_recv'] for r in snapshot])
    s_tick = stats([r['inter_tick'] for r in snapshot])

    print('  ─────────────────────────────────────────────────────────────')
    print_stats('inter_recv  (jitter DDS)', s_recv)
    print()
    #print_stats('inter_tick  (jitter hardware — delta tick/1000)', s_tick)
    #print()

    fields = ['t_recv', 'tick_hw', 'inter_recv', 'inter_tick',
              'acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z']
    save_csv(os.path.join(args.output, 'imu_jitter.csv'), fields, snapshot)


# ──────────────────────────────────────────────────────────────────────────────
# Fase 2 — LiDAR Mid-360 (rt/utlidar/cloud_livox_mid360)
# ──────────────────────────────────────────────────────────────────────────────

def extract_stamp_ns(msg):
    if STAMP_MODE == 'header':
        try:
            h = msg.header.stamp
            return int(h.sec) * 1_000_000_000 + int(h.nanosec)
        except Exception:
            pass
    return None


def run_lidar(args):
    print('╔══════════════════════════════════════════════════════════════╗')
    print('║  FASE 2 — LiDAR Mid-360  (rt/utlidar/cloud_livox_mid360)   ║')
    print('╚══════════════════════════════════════════════════════════════╝')

    if LIDAR_MSG is None:
        print('  Tipo de mensagem do LiDAR não disponível — abortando fase 2.')
        return

    buf  = []
    lock = threading.Lock()
    stop = threading.Event()
    last = {'recv': None, 'stamp': None}

    def handler(msg):
        if stop.is_set():
            return
        t_recv   = mono()
        stamp_ns = extract_stamp_ns(msg)
        with lock:
            inter_recv  = (t_recv - last['recv']) if last['recv'] is not None else float('nan')
            inter_stamp = (
                (stamp_ns - last['stamp']) / 1e9
                if (stamp_ns is not None and last['stamp'] is not None)
                else float('nan')
            )
            last['recv']  = t_recv
            last['stamp'] = stamp_ns
            try:
                n_points = (msg.row_step * msg.height) // msg.point_step
            except Exception:
                n_points = -1
            buf.append({
                't_recv':      t_recv,
                'stamp_ns':    stamp_ns if stamp_ns is not None else -1,
                'inter_recv':  inter_recv,
                'inter_stamp': inter_stamp,
                'n_points':    n_points,
            })

    sub = ChannelSubscriber(f'rt/{LIDAR_TOPIC}', LIDAR_MSG)
    sub.Init(handler, 10)

    progress_loop(buf, lock, args.duration, stop, label='scans LiDAR')
    stop.set()
    time.sleep(0.2)
    del sub

    with lock:
        snapshot = list(buf)

    if not snapshot:
        print('  Nenhum dado coletado no LiDAR.')
        print(f'  Tópico usado: rt/{LIDAR_TOPIC}')
        return

    s_recv  = stats([r['inter_recv']  for r in snapshot])
    s_stamp = stats([r['inter_stamp'] for r in snapshot])
    pts     = [r['n_points'] for r in snapshot if r['n_points'] >= 0]
    avg_pts = sum(pts) / len(pts) if pts else float('nan')

    print('  ─────────────────────────────────────────────────────────────')
    print_stats('inter_recv  (jitter DDS)', s_recv)
    print()
    #print_stats('inter_stamp (jitter hardware — delta timestamp interno)', s_stamp)
    if not math.isnan(avg_pts):
        print(f'\n  Pontos por scan: média={avg_pts:.0f}')
    print()

    fields = ['t_recv', 'stamp_ns', 'inter_recv', 'inter_stamp', 'n_points']
    save_csv(os.path.join(args.output, 'lidar_jitter.csv'), fields, snapshot)


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration',   type=float, default=120,
                    help='Segundos de coleta por sensor (0 = infinito)')
    ap.add_argument('--output',     type=str,   default='.',
                    help='Diretório de saída')
    ap.add_argument('--iface',      type=str,   default='eth0',
                    help='Interface de rede')
    ap.add_argument('--skip-imu',   action='store_true', help='Pula fase IMU')
    ap.add_argument('--skip-lidar', action='store_true', help='Pula fase LiDAR')
    args = ap.parse_args()

    os.makedirs(args.output, exist_ok=True)

    print(f'Inicializando DDS (iface={args.iface})...')
    ChannelFactoryInitialize(0, args.iface)
    print()

    if not args.skip_imu:
        run_imu(args)
        print()
        if not args.skip_lidar:
            print('  Aguardando 2s antes de iniciar o LiDAR...')
            time.sleep(2)
            print()

    if not args.skip_lidar:
        run_lidar(args)

    print('═══════════════════════════════════════════════════════════════')
    print('  Concluído.')
    print('═══════════════════════════════════════════════════════════════')


if __name__ == '__main__':
    main()
