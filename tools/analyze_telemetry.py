#!/usr/bin/env python3
"""
Telemetry Analysis & Visualization Tool

Analyzes CSV telemetry data from the Kosmos Proxi robot to identify
gait frequencies and heading drift patterns.

Usage:
    python analyze_telemetry.py <csv_file>
    python analyze_telemetry.py  # Uses most recent *_telemetrie.csv
"""

import sys
import glob
import numpy as np
import csv
from pathlib import Path

# Check for matplotlib
try:
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
except ImportError:
    print("Please install matplotlib: pip install matplotlib")
    sys.exit(1)


def load_telemetry(filename: str) -> dict:
    """Load telemetry CSV into numpy arrays."""
    with open(filename, 'r') as f:
        data = list(csv.DictReader(f))

    return {
        'timestamp_ms': np.array([int(r['timestamp_ms']) for r in data]),
        'roll': np.array([float(r['roll']) for r in data]),
        'pitch': np.array([float(r['pitch']) for r in data]),
        'heading': np.array([float(r['heading']) for r in data]),
        'yaw_rate': np.array([float(r['yaw_rate']) for r in data]),
        'ir_left': np.array([int(r['ir_left_mm']) for r in data]),
        'ir_right': np.array([int(r['ir_right_mm']) for r in data]),
        'motor_linear': np.array([int(r['motor_linear']) for r in data]),
        'motor_angular': np.array([int(r['motor_angular']) for r in data]),
        'raw_ax': np.array([int(r['raw_ax']) for r in data]),
        'raw_ay': np.array([int(r['raw_ay']) for r in data]),
        'raw_az': np.array([int(r['raw_az']) for r in data]),
        'raw_mx': np.array([int(r['raw_mx']) for r in data]),
        'raw_my': np.array([int(r['raw_my']) for r in data]),
        'raw_mz': np.array([int(r['raw_mz']) for r in data]),
    }


def compute_fft(signal: np.ndarray, fs: float, detrend: bool = True):
    """Compute FFT of signal, return frequencies and magnitudes."""
    if detrend:
        # Remove linear trend
        t = np.arange(len(signal))
        signal = signal - np.polyval(np.polyfit(t, signal, 1), t)

    N = len(signal)
    yf = np.abs(np.fft.fft(signal))
    xf = np.fft.fftfreq(N, 1/fs)

    # Return only positive frequencies
    pos = xf > 0
    return xf[pos], yf[pos]


def find_peaks(freqs: np.ndarray, mags: np.ndarray, min_freq: float = 0.3,
               max_freq: float = 5.0, n_peaks: int = 5):
    """Find top N peaks in frequency range."""
    mask = (freqs >= min_freq) & (freqs <= max_freq)
    f_range = freqs[mask]
    m_range = mags[mask]

    top_idx = np.argsort(m_range)[-n_peaks:][::-1]
    return [(f_range[i], m_range[i]) for i in top_idx]


def plot_analysis(data: dict, filename: str):
    """Create comprehensive analysis plot."""
    # Calculate time and sample rate
    t = (data['timestamp_ms'] - data['timestamp_ms'][0]) / 1000  # seconds
    dt = np.diff(data['timestamp_ms']) / 1000
    fs = 1 / np.mean(dt)

    # Find walking segments
    walking = np.abs(data['motor_linear']) > 20

    # Setup figure
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(f'Telemetry Analysis: {Path(filename).name}\n'
                 f'Duration: {t[-1]:.1f}s, Sample Rate: {fs:.1f} Hz, '
                 f'Walking: {100*walking.sum()/len(walking):.0f}%',
                 fontsize=12, fontweight='bold')

    gs = GridSpec(4, 3, figure=fig, hspace=0.3, wspace=0.3)

    # Color scheme
    colors = {
        'roll': '#e74c3c',
        'pitch': '#3498db',
        'heading': '#2ecc71',
        'accel': '#9b59b6',
        'motor': '#f39c12',
        'gait': '#e74c3c',
    }

    # --- Row 1: Time Series ---

    # Roll & Pitch
    ax1 = fig.add_subplot(gs[0, 0:2])
    ax1.plot(t, data['roll'], color=colors['roll'], label='Roll', alpha=0.8)
    ax1.plot(t, data['pitch'], color=colors['pitch'], label='Pitch', alpha=0.8)
    ax1.fill_between(t, -20, 20, where=walking, alpha=0.1, color='green', label='Walking')
    ax1.set_ylabel('Angle (°)')
    ax1.set_xlabel('Time (s)')
    ax1.legend(loc='upper right')
    ax1.set_title('Roll & Pitch')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(-20, 10)

    # Heading
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.plot(t, data['heading'], color=colors['heading'], linewidth=1)
    ax2.set_ylabel('Heading (°)')
    ax2.set_xlabel('Time (s)')
    ax2.set_title('Heading')
    ax2.grid(True, alpha=0.3)

    # --- Row 2: FFT Analysis ---

    # Roll FFT
    ax3 = fig.add_subplot(gs[1, 0])
    freqs, mags = compute_fft(data['roll'], fs)
    ax3.semilogy(freqs, mags, color=colors['roll'], alpha=0.8)
    ax3.axvspan(1.0, 1.5, alpha=0.2, color=colors['gait'], label='Gait range')
    ax3.set_xlim(0, 5)
    ax3.set_xlabel('Frequency (Hz)')
    ax3.set_ylabel('Magnitude')
    ax3.set_title('Roll FFT')
    ax3.grid(True, alpha=0.3)

    # Annotate peaks
    peaks = find_peaks(freqs, mags)
    for f, m in peaks[:3]:
        ax3.annotate(f'{f:.2f}Hz', (f, m), textcoords="offset points",
                    xytext=(5, 5), fontsize=8)

    # Pitch FFT
    ax4 = fig.add_subplot(gs[1, 1])
    freqs, mags = compute_fft(data['pitch'], fs)
    ax4.semilogy(freqs, mags, color=colors['pitch'], alpha=0.8)
    ax4.axvspan(1.0, 1.5, alpha=0.2, color=colors['gait'])
    ax4.set_xlim(0, 5)
    ax4.set_xlabel('Frequency (Hz)')
    ax4.set_ylabel('Magnitude')
    ax4.set_title('Pitch FFT')
    ax4.grid(True, alpha=0.3)

    peaks = find_peaks(freqs, mags)
    for f, m in peaks[:3]:
        ax4.annotate(f'{f:.2f}Hz', (f, m), textcoords="offset points",
                    xytext=(5, 5), fontsize=8)

    # Accelerometer FFT
    ax5 = fig.add_subplot(gs[1, 2])
    freqs, mags = compute_fft(data['raw_ax'], fs)
    ax5.semilogy(freqs, mags, color=colors['accel'], alpha=0.8, label='Accel-X')
    ax5.axvspan(1.0, 1.5, alpha=0.2, color=colors['gait'])
    ax5.set_xlim(0, 5)
    ax5.set_xlabel('Frequency (Hz)')
    ax5.set_ylabel('Magnitude')
    ax5.set_title('Accelerometer FFT')
    ax5.grid(True, alpha=0.3)

    peaks = find_peaks(freqs, mags)
    gait_freq = peaks[0][0] if peaks else 0
    for f, m in peaks[:3]:
        ax5.annotate(f'{f:.2f}Hz', (f, m), textcoords="offset points",
                    xytext=(5, 5), fontsize=8)

    # --- Row 3: Motor & Yaw Rate ---

    # Motor commands
    ax6 = fig.add_subplot(gs[2, 0:2])
    ax6.plot(t, data['motor_linear'], color=colors['motor'], label='Linear', alpha=0.8)
    ax6.plot(t, data['motor_angular'], color='#8e44ad', label='Angular', alpha=0.8)
    ax6.set_ylabel('Motor Command')
    ax6.set_xlabel('Time (s)')
    ax6.legend(loc='upper right')
    ax6.set_title('Motor Commands')
    ax6.grid(True, alpha=0.3)
    ax6.set_ylim(-110, 110)

    # Yaw Rate
    ax7 = fig.add_subplot(gs[2, 2])
    ax7.plot(t, data['yaw_rate'], color='#1abc9c', alpha=0.8)
    ax7.set_ylabel('Yaw Rate (°/s)')
    ax7.set_xlabel('Time (s)')
    ax7.set_title('Yaw Rate (Kalman)')
    ax7.grid(True, alpha=0.3)

    # --- Row 4: Summary Stats ---

    # Heading drift analysis
    ax8 = fig.add_subplot(gs[3, 0])
    # Unwrap heading for drift calculation
    heading_unwrap = np.unwrap(np.deg2rad(data['heading']))
    heading_unwrap = np.rad2deg(heading_unwrap)
    drift = heading_unwrap - heading_unwrap[0]
    ax8.plot(t, drift, color=colors['heading'])
    ax8.set_ylabel('Cumulative Drift (°)')
    ax8.set_xlabel('Time (s)')
    ax8.set_title(f'Heading Drift: {drift[-1]:.1f}° total')
    ax8.grid(True, alpha=0.3)

    # IR Sensors
    ax9 = fig.add_subplot(gs[3, 1])
    ax9.plot(t, data['ir_left'], label='Left', alpha=0.8)
    ax9.plot(t, data['ir_right'], label='Right', alpha=0.8)
    ax9.set_ylabel('Distance (mm)')
    ax9.set_xlabel('Time (s)')
    ax9.legend(loc='upper right')
    ax9.set_title('IR Sensors')
    ax9.grid(True, alpha=0.3)

    # Summary text
    ax10 = fig.add_subplot(gs[3, 2])
    ax10.axis('off')

    # Calculate stats
    roll_std = data['roll'].std()
    pitch_std = data['pitch'].std()
    drift_rate = drift[-1] / t[-1] if t[-1] > 0 else 0

    summary = f"""
    === ANALYSIS SUMMARY ===

    Gait Frequency: {gait_freq:.2f} Hz

    Roll StdDev:  {roll_std:.2f}°
    Pitch StdDev: {pitch_std:.2f}°

    Heading Drift: {drift[-1]:.1f}°
    Drift Rate:    {drift_rate:.2f}°/s

    Samples:  {len(t)}
    Duration: {t[-1]:.1f}s

    === FILTER SUGGESTION ===

    Notch filter at {gait_freq:.1f} Hz
    (±0.2 Hz bandwidth)
    """

    ax10.text(0.1, 0.9, summary, transform=ax10.transAxes,
              fontsize=10, verticalalignment='top', fontfamily='monospace',
              bbox=dict(boxstyle='round', facecolor='#2c3e50', alpha=0.8),
              color='white')

    plt.tight_layout()

    # Save figure
    output_file = filename.replace('.csv', '_analysis.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight',
                facecolor='#1e1e1e', edgecolor='none')
    print(f"Saved: {output_file}")

    plt.show()


def main():
    # Find CSV file
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        # Find most recent telemetrie.csv
        files = glob.glob('*_telemetrie.csv') + glob.glob('../*_telemetrie.csv')
        if not files:
            print("No telemetrie.csv files found!")
            print("Usage: python analyze_telemetry.py <csv_file>")
            sys.exit(1)
        filename = max(files, key=lambda f: Path(f).stat().st_mtime)
        print(f"Using most recent: {filename}")

    # Load and analyze
    print(f"Loading {filename}...")
    data = load_telemetry(filename)
    print(f"Loaded {len(data['timestamp_ms'])} samples")

    # Plot
    plot_analysis(data, filename)


if __name__ == '__main__':
    main()
