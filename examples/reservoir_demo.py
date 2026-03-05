#!/usr/bin/env python3
"""
reservoir_demo.py — Minimal reservoir computing demo with FPGA neuron bank
==========================================================================
Demonstrates waveform classification using the NS-RAM FPGA bridge as a
physical reservoir computer.  Injects 4-class waveforms (sine, square,
triangle, sawtooth) via MAC current and reads out spike patterns for
ridge-regression classification.

Requirements:
    pip install numpy scikit-learn
    FPGA programmed with nsram_eth_top.bit and connected via Ethernet.

Usage:
    python reservoir_demo.py
"""

import sys, time
import numpy as np
from sklearn.linear_model import RidgeClassifier
from sklearn.model_selection import cross_val_score

# Add parent scripts dir to path
sys.path.insert(0, "../scripts")
from fpga_host_eth import FPGAEthBridge

# ── Configuration ──
BASE_VG     = 0.58      # Gate voltage (V) — near BVpar cliff
SAMPLE_HZ   = 20        # Telemetry sampling rate
STEPS        = 50        # Steps per waveform trial
N_TRIALS     = 40        # Trials per class
N_CLASSES    = 4         # sine, square, triangle, sawtooth
RIDGE_ALPHA  = 10.0      # Regularisation strength


def generate_waveform(cls: int, steps: int) -> np.ndarray:
    """Generate a normalised waveform signal [0, 1]."""
    t = np.linspace(0, 2 * np.pi, steps)
    if cls == 0:
        w = np.sin(t)
    elif cls == 1:
        w = np.sign(np.sin(t))
    elif cls == 2:
        w = 2 * np.abs(2 * (t / (2 * np.pi) - np.floor(t / (2 * np.pi) + 0.5))) - 1
    else:
        w = 2 * (t / (2 * np.pi) - np.floor(t / (2 * np.pi))) - 1
    return (w - w.min()) / (w.max() - w.min() + 1e-10)


def run_trial(fpga: FPGAEthBridge, waveform: np.ndarray) -> np.ndarray:
    """Inject waveform and collect spike responses."""
    spikes = []
    for val in waveform:
        fpga.set_mac(val)
        time.sleep(1.0 / SAMPLE_HZ)
        telem = fpga.read_telemetry()
        if telem is not None:
            spikes.append(telem["spike_counts"])
        else:
            spikes.append(np.zeros(fpga.num_neurons))
    return np.array(spikes)  # (steps, neurons)


def main():
    # Connect to FPGA
    fpga = FPGAEthBridge()
    fpga.connect()
    print(f"Connected: {fpga.num_neurons} neurons")

    # Set operating point
    fpga.set_kill(0)  # Ensure kill switch off
    for n in range(fpga.num_neurons):
        fpga.set_vg(n, BASE_VG)
    time.sleep(0.5)

    # Collect data
    X, y = [], []
    for trial in range(N_TRIALS):
        for cls in range(N_CLASSES):
            waveform = generate_waveform(cls, STEPS)
            response = run_trial(fpga, waveform)
            # Feature: mean spike count per neuron across trial
            features = response.mean(axis=0)
            X.append(features)
            y.append(cls)
        print(f"  Trial {trial+1}/{N_TRIALS} complete")

    X = np.array(X)
    y = np.array(y)

    # Ridge regression readout with 5-fold CV
    clf = RidgeClassifier(alpha=RIDGE_ALPHA)
    scores = cross_val_score(clf, X, y, cv=5)
    print(f"\nClassification accuracy: {scores.mean():.1%} +/- {scores.std():.1%}")
    print(f"Per-fold: {[f'{s:.1%}' for s in scores]}")

    fpga.close()


if __name__ == "__main__":
    main()
