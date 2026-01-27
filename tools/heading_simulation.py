#!/usr/bin/env python3
"""
Heading Stability Simulation

Tests different heading computation algorithms on recorded telemetry data
to evaluate heading stability during hexapod walking.

Problem: Gait-induced tilt oscillation (~1.27 Hz) causes heading wobble of ±7°
Solution: Use slowly-filtered reference gravity for tilt compensation

Algorithms tested:
A. Current (baseline): Instantaneous accel for tilt compensation
B. Low-pass filtered roll/pitch
C. Reference gravity vector (slowly filtered accel direction)
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from scipy import signal
from scipy.fft import fft, fftfreq


def load_telemetry(csv_path: str) -> pd.DataFrame:
    """Load telemetry CSV file."""
    df = pd.read_csv(csv_path)
    # Convert timestamp to relative seconds
    df['time_s'] = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    return df


def compute_heading_direct(ax, ay, az, mx, my, mz):
    """
    Current algorithm: Tilt-compensated heading using instantaneous accel.
    This is what mahony_get_euler_direct() does in firmware.
    """
    # Normalize accelerometer
    anorm = np.sqrt(ax**2 + ay**2 + az**2)
    anorm = np.where(anorm < 0.001, 1.0, anorm)
    ax_n = ax / anorm
    ay_n = ay / anorm
    az_n = az / anorm

    # Roll and pitch from accelerometer
    roll = np.arctan2(ay_n, az_n)
    pitch = np.arctan2(-ax_n, np.sqrt(ay_n**2 + az_n**2))

    # Tilt compensation
    cos_roll = np.cos(roll)
    sin_roll = np.sin(roll)
    cos_pitch = np.cos(pitch)
    sin_pitch = np.sin(pitch)

    mx_h = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch
    my_h = my * cos_roll - mz * sin_roll

    heading = np.arctan2(-my_h, mx_h) * (180.0 / np.pi)
    heading = np.where(heading < 0, heading + 360, heading)

    return heading, np.degrees(roll), np.degrees(pitch)


def compute_heading_lpf_tilt(ax, ay, az, mx, my, mz, alpha=0.1):
    """
    Algorithm B: Low-pass filter on roll/pitch before tilt compensation.
    """
    # First compute instantaneous roll/pitch
    anorm = np.sqrt(ax**2 + ay**2 + az**2)
    anorm = np.where(anorm < 0.001, 1.0, anorm)
    ax_n = ax / anorm
    ay_n = ay / anorm
    az_n = az / anorm

    roll_inst = np.arctan2(ay_n, az_n)
    pitch_inst = np.arctan2(-ax_n, np.sqrt(ay_n**2 + az_n**2))

    # Low-pass filter roll and pitch
    roll_filt = np.zeros_like(roll_inst)
    pitch_filt = np.zeros_like(pitch_inst)
    roll_filt[0] = roll_inst[0]
    pitch_filt[0] = pitch_inst[0]

    for i in range(1, len(roll_inst)):
        roll_filt[i] = alpha * roll_inst[i] + (1 - alpha) * roll_filt[i-1]
        pitch_filt[i] = alpha * pitch_inst[i] + (1 - alpha) * pitch_filt[i-1]

    # Tilt compensation with filtered angles
    cos_roll = np.cos(roll_filt)
    sin_roll = np.sin(roll_filt)
    cos_pitch = np.cos(pitch_filt)
    sin_pitch = np.sin(pitch_filt)

    mx_h = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch
    my_h = my * cos_roll - mz * sin_roll

    heading = np.arctan2(-my_h, mx_h) * (180.0 / np.pi)
    heading = np.where(heading < 0, heading + 360, heading)

    return heading


def compute_heading_ref_gravity(ax, ay, az, mx, my, mz, alpha=0.05):
    """
    Algorithm C: Reference gravity vector (recommended).

    Uses slowly-filtered normalized gravity direction to define the
    stable walking plane, instead of instantaneous accelerometer.
    """
    n = len(ax)
    heading = np.zeros(n)

    # Initialize reference gravity
    anorm = np.sqrt(ax[0]**2 + ay[0]**2 + az[0]**2)
    if anorm > 0.5:
        ref_gx = ax[0] / anorm
        ref_gy = ay[0] / anorm
        ref_gz = az[0] / anorm
    else:
        ref_gx, ref_gy, ref_gz = 0.0, 0.0, 1.0

    for i in range(n):
        # Normalize current accelerometer
        anorm = np.sqrt(ax[i]**2 + ay[i]**2 + az[i]**2)
        if anorm > 0.5:  # Valid reading (>0.5g)
            gx = ax[i] / anorm
            gy = ay[i] / anorm
            gz = az[i] / anorm

            # Update reference gravity with EMA
            ref_gx = alpha * gx + (1 - alpha) * ref_gx
            ref_gy = alpha * gy + (1 - alpha) * ref_gy
            ref_gz = alpha * gz + (1 - alpha) * ref_gz

            # Renormalize
            rn = np.sqrt(ref_gx**2 + ref_gy**2 + ref_gz**2)
            ref_gx /= rn
            ref_gy /= rn
            ref_gz /= rn

        # Compute roll/pitch from reference gravity (stable plane)
        roll = np.arctan2(ref_gy, ref_gz)
        pitch = np.arctan2(-ref_gx, np.sqrt(ref_gy**2 + ref_gz**2))

        # Tilt compensation
        cos_roll = np.cos(roll)
        sin_roll = np.sin(roll)
        cos_pitch = np.cos(pitch)
        sin_pitch = np.sin(pitch)

        mx_h = mx[i] * cos_pitch + my[i] * sin_roll * sin_pitch + mz[i] * cos_roll * sin_pitch
        my_h = my[i] * cos_roll - mz[i] * sin_roll

        heading[i] = np.arctan2(-my_h, mx_h) * (180.0 / np.pi)
        if heading[i] < 0:
            heading[i] += 360

    return heading


def detrend_heading(heading):
    """Remove linear trend from heading for std calculation."""
    # Handle wrap-around by unwrapping
    heading_unwrap = np.unwrap(np.radians(heading))
    heading_deg = np.degrees(heading_unwrap)

    # Linear detrend
    t = np.arange(len(heading))
    coeffs = np.polyfit(t, heading_deg, 1)
    trend = np.polyval(coeffs, t)

    return heading_deg - trend


def compute_fft_power(heading, sample_rate, target_freq=1.27):
    """Compute FFT power at target frequency."""
    # Detrend
    heading_detrend = detrend_heading(heading)

    # FFT
    n = len(heading_detrend)
    yf = fft(heading_detrend)
    xf = fftfreq(n, 1/sample_rate)

    # Find power at target frequency
    idx = np.argmin(np.abs(xf - target_freq))
    power = np.abs(yf[idx])**2

    return power, xf[:n//2], np.abs(yf[:n//2])**2


def find_walking_segment(df, motor_threshold=50):
    """Find the segment where robot is walking (motor_linear > threshold)."""
    walking = df['motor_linear'].abs() > motor_threshold
    if not walking.any():
        return 0, len(df)

    # Find first and last walking sample
    start_idx = walking.idxmax()
    end_idx = len(df) - 1 - walking[::-1].idxmax()

    return start_idx, end_idx


def main():
    parser = argparse.ArgumentParser(description='Heading stability simulation')
    parser.add_argument('csv_file', nargs='?',
                        default='../2026-01-27_18-49-24_telemetrie.csv',
                        help='Telemetry CSV file')
    parser.add_argument('--alpha', type=float, default=0.05,
                        help='Reference gravity filter alpha (default: 0.05)')
    parser.add_argument('--plot', action='store_true', default=True,
                        help='Show plots')
    parser.add_argument('--save', type=str,
                        help='Save plot to file')
    args = parser.parse_args()

    # Resolve path relative to script location
    csv_path = Path(__file__).parent / args.csv_file
    if not csv_path.exists():
        csv_path = Path(args.csv_file)

    print(f"Loading telemetry from: {csv_path}")
    df = load_telemetry(csv_path)
    print(f"Loaded {len(df)} samples")

    # Find walking segment
    start_idx, end_idx = find_walking_segment(df)
    print(f"Walking segment: samples {start_idx} to {end_idx}")

    # Use walking segment only
    df_walk = df.iloc[start_idx:end_idx].reset_index(drop=True)
    print(f"Analyzing {len(df_walk)} walking samples")

    # Extract sensor data (convert milli-g to g, milli-Gauss to Gauss)
    ax = df_walk['raw_ax'].values / 1000.0
    ay = df_walk['raw_ay'].values / 1000.0
    az = df_walk['raw_az'].values / 1000.0
    mx = df_walk['raw_mx'].values / 1000.0
    my = df_walk['raw_my'].values / 1000.0
    mz = df_walk['raw_mz'].values / 1000.0

    # Recorded (Kalman-filtered) heading
    heading_kalman = df_walk['heading'].values

    # Estimate sample rate
    dt_mean = df_walk['time_s'].diff().mean()
    sample_rate = 1.0 / dt_mean if dt_mean > 0 else 20.0
    print(f"Sample rate: {sample_rate:.1f} Hz")

    # Compute headings with different algorithms
    print("\nComputing headings...")

    # A. Current (instantaneous tilt)
    heading_direct, roll_inst, pitch_inst = compute_heading_direct(ax, ay, az, mx, my, mz)

    # B. Low-pass filtered tilt (test different alphas)
    heading_lpf_01 = compute_heading_lpf_tilt(ax, ay, az, mx, my, mz, alpha=0.1)
    heading_lpf_02 = compute_heading_lpf_tilt(ax, ay, az, mx, my, mz, alpha=0.2)
    heading_lpf_05 = compute_heading_lpf_tilt(ax, ay, az, mx, my, mz, alpha=0.05)

    # C. Reference gravity (test different alphas)
    heading_refg_005 = compute_heading_ref_gravity(ax, ay, az, mx, my, mz, alpha=0.05)
    heading_refg_01 = compute_heading_ref_gravity(ax, ay, az, mx, my, mz, alpha=0.1)
    heading_refg_02 = compute_heading_ref_gravity(ax, ay, az, mx, my, mz, alpha=0.02)

    # Compute metrics
    print("\n" + "="*70)
    print("HEADING STABILITY METRICS")
    print("="*70)
    print(f"{'Algorithm':<35} {'Std (deg)':<12} {'FFT @1.27Hz':<12}")
    print("-"*70)

    algorithms = [
        ("Kalman (recorded)", heading_kalman),
        ("Direct (instantaneous tilt)", heading_direct),
        ("LPF tilt (alpha=0.05)", heading_lpf_05),
        ("LPF tilt (alpha=0.1)", heading_lpf_01),
        ("LPF tilt (alpha=0.2)", heading_lpf_02),
        ("Ref gravity (alpha=0.02)", heading_refg_02),
        ("Ref gravity (alpha=0.05)", heading_refg_005),
        ("Ref gravity (alpha=0.1)", heading_refg_01),
    ]

    fft_results = {}
    for name, heading in algorithms:
        heading_detrend = detrend_heading(heading)
        std = np.std(heading_detrend)
        power, xf, yf = compute_fft_power(heading, sample_rate)
        fft_results[name] = (xf, yf)
        print(f"{name:<35} {std:>8.2f}    {power:>10.1f}")

    print("="*70)
    print("\nLower values are better.")
    print("Target: Std < 2°, FFT power @ 1.27Hz < 50")

    # Plot results
    if args.plot or args.save:
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))

        time = df_walk['time_s'].values

        # Plot 1: Roll and Pitch oscillation
        ax1 = axes[0, 0]
        ax1.plot(time, roll_inst, label='Roll', alpha=0.7)
        ax1.plot(time, pitch_inst, label='Pitch', alpha=0.7)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Angle (deg)')
        ax1.set_title('Gait-Induced Tilt Oscillation')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot 2: Heading comparison (detrended)
        ax2 = axes[0, 1]
        ax2.plot(time, detrend_heading(heading_kalman), label='Kalman (recorded)', alpha=0.8)
        ax2.plot(time, detrend_heading(heading_direct), label='Direct', alpha=0.6)
        ax2.plot(time, detrend_heading(heading_refg_005), label='Ref gravity (0.05)', alpha=0.8, linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Heading deviation (deg)')
        ax2.set_title('Heading Comparison (Detrended)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Plot 3: FFT comparison
        ax3 = axes[1, 0]
        for name in ["Kalman (recorded)", "Direct (instantaneous tilt)", "Ref gravity (alpha=0.05)"]:
            xf, yf = fft_results[name]
            ax3.semilogy(xf, yf, label=name, alpha=0.8)
        ax3.axvline(x=1.27, color='r', linestyle='--', label='Gait freq (1.27 Hz)')
        ax3.set_xlabel('Frequency (Hz)')
        ax3.set_ylabel('Power (log)')
        ax3.set_title('FFT of Heading')
        ax3.set_xlim(0, 5)
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Plot 4: Alpha comparison for ref gravity
        ax4 = axes[1, 1]
        ax4.plot(time, detrend_heading(heading_refg_02), label='alpha=0.02 (slowest)', alpha=0.7)
        ax4.plot(time, detrend_heading(heading_refg_005), label='alpha=0.05', alpha=0.8)
        ax4.plot(time, detrend_heading(heading_refg_01), label='alpha=0.1', alpha=0.7)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Heading deviation (deg)')
        ax4.set_title('Reference Gravity Alpha Comparison')
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        # Plot 5: Magnetometer raw
        ax5 = axes[2, 0]
        ax5.plot(time, mx * 1000, label='Mx', alpha=0.7)
        ax5.plot(time, my * 1000, label='My', alpha=0.7)
        ax5.plot(time, mz * 1000, label='Mz', alpha=0.7)
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Mag (mG)')
        ax5.set_title('Magnetometer (shows gait interference)')
        ax5.legend()
        ax5.grid(True, alpha=0.3)

        # Plot 6: Improvement summary
        ax6 = axes[2, 1]
        names = ['Kalman', 'Direct', 'RefGrav\n(0.05)']
        stds = [
            np.std(detrend_heading(heading_kalman)),
            np.std(detrend_heading(heading_direct)),
            np.std(detrend_heading(heading_refg_005)),
        ]
        colors = ['tab:blue', 'tab:orange', 'tab:green']
        bars = ax6.bar(names, stds, color=colors)
        ax6.axhline(y=2.0, color='r', linestyle='--', label='Target (<2°)')
        ax6.set_ylabel('Heading Std (deg)')
        ax6.set_title('Stability Comparison')
        ax6.legend()
        for bar, std in zip(bars, stds):
            ax6.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                     f'{std:.1f}°', ha='center', va='bottom')

        plt.tight_layout()

        if args.save:
            plt.savefig(args.save, dpi=150)
            print(f"\nPlot saved to: {args.save}")

        if args.plot:
            plt.show()


if __name__ == '__main__':
    main()
