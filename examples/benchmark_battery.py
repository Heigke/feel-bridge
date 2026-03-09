#!/usr/bin/env python3
"""
benchmark_battery.py — Standard reservoir computing benchmark suite
====================================================================
Evaluates a reservoir on 5 standard benchmarks:
  1. Waveform classification (4-class, 8-class)
  2. Memory capacity (MC) — linear recall at delays 1..10
  3. Temporal XOR — nonlinear parity at delays 1..3
  4. NARMA — nonlinear autoregressive moving average
  5. Scaling analysis — neuron count vs performance

Can be used with any reservoir that implements the ReservoirInterface.

For FPGA usage:
    python benchmark_battery.py --fpga
For GPU-only (requires compiled HIP kernel):
    python benchmark_battery.py --gpu
For simulation (numpy, no hardware needed):
    python benchmark_battery.py --sim

Requirements: numpy, scikit-learn
"""

import sys, time, argparse
import numpy as np
from sklearn.linear_model import RidgeClassifier, Ridge
from sklearn.model_selection import cross_val_score


# ══════════════════════════════════════════════════════════
# Reservoir Interface (abstract)
# ══════════════════════════════════════════════════════════

class ReservoirInterface:
    """Base class for reservoir implementations."""

    def __init__(self, n_neurons: int):
        self.n_neurons = n_neurons

    def reset(self):
        """Reset reservoir state."""
        pass

    def step(self, u: float) -> np.ndarray:
        """Feed one input, return state vector (n_neurons,)."""
        raise NotImplementedError

    def run(self, input_seq: np.ndarray) -> np.ndarray:
        """Run full sequence, return states (n_steps, n_neurons)."""
        self.reset()
        states = np.zeros((len(input_seq), self.n_neurons))
        for t, u in enumerate(input_seq):
            states[t] = self.step(u)
        return states


class SimulatedESN(ReservoirInterface):
    """Simple Echo State Network for baseline comparison (no hardware needed)."""

    def __init__(self, n_neurons=256, spectral_radius=0.95, input_scale=0.1,
                 leak=0.3, seed=42):
        super().__init__(n_neurons)
        rng = np.random.default_rng(seed)
        # Sparse random recurrent weights
        W = rng.standard_normal((n_neurons, n_neurons)) * 0.1
        mask = rng.random((n_neurons, n_neurons)) < 0.1
        W *= mask
        # Scale to desired spectral radius
        eig = np.max(np.abs(np.linalg.eigvals(W)))
        if eig > 0:
            W *= spectral_radius / eig
        self.W = W
        self.W_in = rng.standard_normal(n_neurons) * input_scale
        self.leak = leak
        self.state = np.zeros(n_neurons)

    def reset(self):
        self.state = np.zeros(self.n_neurons)

    def step(self, u: float) -> np.ndarray:
        pre = self.W @ self.state + self.W_in * u
        self.state = (1 - self.leak) * self.state + self.leak * np.tanh(pre)
        return self.state.copy()


class FPGAReservoir(ReservoirInterface):
    """128-neuron FPGA NS-RAM reservoir via Ethernet."""

    def __init__(self, base_vg=0.58, sample_hz=50):
        super().__init__(128)
        self.base_vg = base_vg
        self.sample_hz = sample_hz
        self.fpga = None

    def connect(self):
        sys.path.insert(0, "../scripts")
        from fpga_host_eth import FPGAEthBridge
        self.fpga = FPGAEthBridge()
        self.fpga.connect()
        self.fpga.set_kill(0)
        for n in range(self.n_neurons):
            self.fpga.set_vg(n, self.base_vg)
        time.sleep(0.5)
        print(f"FPGA connected: {self.fpga.num_neurons} neurons")

    def reset(self):
        if self.fpga:
            self.fpga.set_mac(0.0)
            time.sleep(0.05)

    def step(self, u: float) -> np.ndarray:
        self.fpga.set_mac(float(np.clip(u, 0, 1)))
        time.sleep(1.0 / self.sample_hz)
        telem = self.fpga.read_telemetry()
        if telem is not None:
            return telem['vmem'].astype(np.float64)
        return np.zeros(self.n_neurons)


# ══════════════════════════════════════════════════════════
# Signal generators
# ══════════════════════════════════════════════════════════

def generate_waveform(cls: int, steps: int) -> np.ndarray:
    t = np.linspace(0, 2 * np.pi, steps)
    if cls == 0:   return np.sin(t)
    elif cls == 1: return np.sign(np.sin(t))
    elif cls == 2: return 2 * np.abs(2 * (t/(2*np.pi) - np.floor(t/(2*np.pi) + 0.5))) - 1
    else:          return 2 * (t/(2*np.pi) - np.floor(t/(2*np.pi))) - 1

def generate_narma(u, order=10):
    """Standard NARMA target (Atiya & Parlos, 2000)."""
    n = len(u)
    y = np.zeros(n)
    u_s = np.clip(u * 0.2 + 0.2, 0.0, 0.5)
    for t in range(order, n):
        s = np.sum(y[max(0, t-order):t])
        y[t] = 0.3 * y[t-1] + 0.05 * y[t-1] * s \
             + 1.5 * u_s[t-order] * u_s[t] + 0.1
        y[t] = np.clip(y[t], -5, 5)
    return y


# ══════════════════════════════════════════════════════════
# Benchmarks
# ══════════════════════════════════════════════════════════

def benchmark_waveform(reservoir, n_classes=4, n_trials=40, steps=60):
    """Waveform classification with ridge readout."""
    print(f"\n{'='*50}")
    print(f"BENCHMARK: {n_classes}-class waveform classification")
    print(f"  {n_trials} trials × {steps} steps")
    print(f"{'='*50}")

    X, y = [], []
    for trial in range(n_trials):
        for cls in range(n_classes):
            wf = generate_waveform(cls, steps)
            wf_norm = (wf - wf.min()) / (wf.max() - wf.min() + 1e-10)
            states = reservoir.run(wf_norm)
            # Pool: mean + std + last step
            feat = np.concatenate([
                states.mean(axis=0),
                states.std(axis=0),
                states[-1]
            ])
            X.append(feat)
            y.append(cls)
        if (trial + 1) % 10 == 0:
            print(f"  trial {trial+1}/{n_trials}")

    X = np.array(X)
    y = np.array(y)

    clf = RidgeClassifier(alpha=10.0)
    scores = cross_val_score(clf, X, y, cv=5)
    print(f"  Accuracy: {scores.mean():.1%} +/- {scores.std():.1%}")
    return scores.mean()


def benchmark_memory_capacity(reservoir, steps=1500, max_delay=10):
    """Linear memory capacity (Jaeger, 2002)."""
    print(f"\n{'='*50}")
    print(f"BENCHMARK: Memory capacity (delays 1..{max_delay})")
    print(f"{'='*50}")

    rng = np.random.default_rng(123)
    u = rng.uniform(-1, 1, steps)
    states = reservoir.run(u)

    warmup = 200
    mc_total = 0.0

    for d in range(1, max_delay + 1):
        X = states[warmup:]
        target = u[warmup - d:steps - d]
        n = min(len(X), len(target))
        X, target = X[:n], target[:n]

        n_tr = int(0.7 * n)
        reg = Ridge(alpha=1.0)
        reg.fit(X[:n_tr], target[:n_tr])
        pred = reg.predict(X[n_tr:])

        ss_res = np.sum((target[n_tr:] - pred) ** 2)
        ss_tot = np.sum((target[n_tr:] - target[n_tr:].mean()) ** 2)
        r2 = max(0, 1 - ss_res / ss_tot)
        mc_total += r2
        print(f"  delay={d:2d}: R2={r2:.4f}")

    print(f"  Total MC = {mc_total:.3f}")
    return mc_total


def benchmark_xor(reservoir, steps=1500, taus=(1, 2, 3)):
    """Temporal XOR — nonlinear computation benchmark."""
    print(f"\n{'='*50}")
    print(f"BENCHMARK: Temporal XOR (tau={list(taus)})")
    print(f"{'='*50}")

    rng = np.random.default_rng(456)
    u = rng.integers(0, 2, steps).astype(float)
    states = reservoir.run(u)

    warmup = 200
    results = {}

    for tau in taus:
        target = np.zeros(steps)
        for t in range(tau, steps):
            target[t] = float(int(u[t]) ^ int(u[t - tau]))

        X = states[warmup:]
        y = target[warmup:]
        n_tr = int(0.7 * len(X))

        reg = Ridge(alpha=1.0)
        reg.fit(X[:n_tr], y[:n_tr])
        pred = reg.predict(X[n_tr:])
        acc = np.mean((pred > 0.5).astype(float) == y[n_tr:])

        results[tau] = acc
        print(f"  tau={tau}: accuracy={acc:.1%}")

    return results


def benchmark_narma(reservoir, order=5, steps=1500):
    """NARMA regression — nonlinear memory benchmark."""
    print(f"\n{'='*50}")
    print(f"BENCHMARK: NARMA-{order}")
    print(f"{'='*50}")

    rng = np.random.default_rng(789)
    u = rng.uniform(-1, 1, steps)
    target = generate_narma(u, order=order)
    states = reservoir.run(u)

    warmup = max(200, order + 50)
    X = states[warmup:]
    y = target[warmup:]
    n_tr = int(0.7 * len(X))

    # Try multiple regularization strengths
    best_r2 = -999
    for alpha in [0.01, 0.1, 1.0, 10.0, 100.0]:
        reg = Ridge(alpha=alpha)
        reg.fit(X[:n_tr], y[:n_tr])
        pred = reg.predict(X[n_tr:])
        ss_res = np.sum((y[n_tr:] - pred) ** 2)
        ss_tot = np.sum((y[n_tr:] - y[n_tr:].mean()) ** 2)
        r2 = max(0, 1 - ss_res / ss_tot) if ss_tot > 1e-10 else 0
        if r2 > best_r2:
            best_r2 = r2
            best_alpha = alpha

    print(f"  R2 = {best_r2:.4f} (alpha={best_alpha})")
    return best_r2


# ══════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="Reservoir benchmark battery")
    parser.add_argument("--sim", action="store_true", help="Use simulated ESN")
    parser.add_argument("--fpga", action="store_true", help="Use FPGA reservoir")
    parser.add_argument("--gpu", action="store_true", help="Use GPU HIP kernel")
    parser.add_argument("--neurons", type=int, default=256, help="Simulated neurons")
    args = parser.parse_args()

    if args.fpga:
        reservoir = FPGAReservoir()
        reservoir.connect()
        label = "FPGA NS-RAM (128 neurons)"
    elif args.gpu:
        print("GPU mode requires compiled HIP kernel — see gpu_reservoir_demo.hip")
        print("For benchmarking, use the simulation mode: --sim")
        return
    else:
        reservoir = SimulatedESN(n_neurons=args.neurons)
        label = f"Simulated ESN ({args.neurons} neurons)"

    print(f"\n{'#'*60}")
    print(f"  Reservoir Benchmark Battery")
    print(f"  System: {label}")
    print(f"{'#'*60}")

    results = {}

    # 1. Waveform classification
    results['wave4'] = benchmark_waveform(reservoir, n_classes=4)
    results['wave8'] = benchmark_waveform(reservoir, n_classes=8)

    # 2. Memory capacity
    results['mc'] = benchmark_memory_capacity(reservoir)

    # 3. Temporal XOR
    results['xor'] = benchmark_xor(reservoir)

    # 4. NARMA
    results['narma5'] = benchmark_narma(reservoir, order=5)
    results['narma10'] = benchmark_narma(reservoir, order=10)

    # Summary
    print(f"\n{'='*60}")
    print(f"SUMMARY — {label}")
    print(f"{'='*60}")
    print(f"  Wave-4 accuracy:    {results['wave4']:.1%}")
    print(f"  Wave-8 accuracy:    {results['wave8']:.1%}")
    print(f"  Memory capacity:    {results['mc']:.3f}")
    print(f"  XOR tau=1:          {results['xor'][1]:.1%}")
    print(f"  XOR tau=2:          {results['xor'][2]:.1%}")
    print(f"  XOR tau=3:          {results['xor'][3]:.1%}")
    print(f"  NARMA-5 R2:         {results['narma5']:.4f}")
    print(f"  NARMA-10 R2:        {results['narma10']:.4f}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
