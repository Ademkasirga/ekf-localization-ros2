#!/usr/bin/env python3
"""Plot GPS vs EKF position error from error_logger CSV."""

import argparse
import csv
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description='Plot ekf_error_log.csv')
    parser.add_argument('csv_path', nargs='?', default='ekf_error_log.csv')
    parser.add_argument('--save', default='', help='Save figure to path instead of showing')
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib required: sudo apt install python3-matplotlib', file=sys.stderr)
        return 1

    t, gps_err, ekf_err = [], [], []
    with open(args.csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row['t_sec']))
            gps_err.append(float(row['gps_err']))
            ekf_err.append(float(row['ekf_err']))

    plt.figure(figsize=(10, 4))
    plt.plot(t, gps_err, label='GPS error [m]', alpha=0.7)
    plt.plot(t, ekf_err, label='EKF error [m]', alpha=0.9)
    plt.xlabel('time [s]')
    plt.ylabel('position error [m]')
    plt.title('Ground truth comparison')
    plt.legend()
    plt.grid(True)
    if args.save:
        plt.savefig(args.save, dpi=120)
    else:
        plt.show()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
