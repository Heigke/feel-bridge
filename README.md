# feel-bridge

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.18880759.svg)](https://doi.org/10.5281/zenodo.18880759)

**Hybrid analog-analog reservoir computing: bridging GPU firmware physics and memristive neuron dynamics on FPGA**

This repository accompanies the preprint:

> *Hybrid Analog–Analog Reservoir Computing: Bridging GPU Firmware Physics and Memristive Neuron Dynamics*
> Eric Bergvall, ENIMBLE Solutions AB, March 2026
> DOI: [10.5281/zenodo.18880759](https://doi.org/10.5281/zenodo.18880759)

## What is this?

An open-source platform for **neuromorphic reservoir computing** using a 128-neuron spiking neural network on FPGA, modelling [NS-RAM](https://en.wikipedia.org/wiki/Memristor) (Neuromorphic Spiking RAM) avalanche dynamics.

The system is designed as a **hardware-substitution platform**: the FPGA neuron model can be replaced with real memristive devices without modifying the analysis pipeline.

## Architecture

```
 GPU / Host PC                    FPGA (Arty A7-100T)
 +---------------------+  UDP   +------------------+
 | Host Python scripts  | 2kHz  | 128 LIF neurons  |
 | Noise injection      |<====>| NS-RAM avalanche |
 | Readout / analysis   |  ETH  | model (LFSR)     |
 |                      |       |                  |
 +---------------------+        +------------------+
       | Input signals               | Spike + Vmem
       v                             | telemetry
     +----------------------------------+
     |   Reservoir readout (ridge)      |
     |   Classification / regression    |
     +----------------------------------+
```

## Key results (from preprint + latest experiments)

| Metric | Value | Experiment |
|--------|-------|------------|
| **GPU HIP 4-class waveform** | **98.9%** | z2277 (real HIP kernel) |
| **GPU HIP 8-class waveform** | **77.6%** | z2277 |
| **GPU HIP Memory Capacity** | **2.860** | z2277 |
| **GPU HIP XOR tau=1** | **96.7%** | z2277 |
| **Bridge NARMA-3 R²** | **0.616** (> GPU 0.605) | z2277 |
| Bridge (GPU+FPGA) 4-class | 94.0% | z2277 |
| Bridge XOR tau=1 | 96.3% | z2277 |
| FPGA-only 128N classification | 81.0% | z2206 |
| Branching ratio (criticality) | 1.027 (2.7% from critical) | z2191 |
| Causal emergence ratio | 2.87x | z2188 |
| Transfer entropy (GPU->FPGA) | 0.122 bits | z2190 |
| Best fusion accuracy (7-level ladder) | 91.5% | z2210 |
| Energy per spike (FPGA) | ~fJ | z2166 |

### GPU-physics reservoir (4 populations)

The GPU reservoir exploits 4 distinct hardware mechanisms as computational primitives:

| Population | Hardware mechanism | Computational role |
|------------|-------------------|-------------------|
| Pop A | Branch divergence (warp path splitting) | Threshold neurons, classification |
| Pop B | LDS bank conflicts + product nonlinearity | XOR, nonlinear mixing |
| Pop C | Dense ESN with temperature-scaled tanh | Temporal memory (NARMA) |
| Pop D | Wavefront scheduling jitter (__clock64) | Stochastic diversity |

These are **not simulated** — each mechanism exploits real analog physics of the GPU silicon. See `examples/gpu_reservoir_demo.hip` for a runnable demo.

### Experiment scale

280+ tests across 70+ experiment groups, including:
- Waveform classification (4-class, 8-class)
- Memory capacity at 10 delays
- Temporal XOR at tau=1,2,3
- NARMA regression (orders 3, 5, 10)
- Transfer entropy and Granger causality
- Scaling laws (8N to 128N)
- Cross-substrate causal coupling analysis

## Repository structure

```
fpga/
  rtl/                  # Verilog source — 128 LIF neurons + NS-RAM model
    lif_membrane.v      # Leaky integrate-and-fire neuron
    avalanche_model.v   # LFSR-based stochastic avalanche current
    nsram_neuron_bank.v # 128-neuron array with parameter control
    nsram_eth_top.v     # Top-level with UDP Ethernet interface
    neuron_scheduler.v  # Time-multiplexed neuron update
    uart_rx.v / uart_tx.v  # Legacy serial interface
    cmd_decoder.v       # Command parser for parameter updates
  constraints/          # Xilinx pin constraints (Arty A7)
  scripts/              # Vivado build and programming TCL scripts

scripts/
  fpga_host_eth.py      # Python UDP bridge — connect, configure, read telemetry

examples/
  reservoir_demo.py         # Minimal FPGA waveform classification demo
  gpu_reservoir_demo.hip    # GPU-physics reservoir (4-population HIP kernel)
  benchmark_battery.py      # Standard RC benchmark suite (MC, XOR, NARMA, waveform)

preprint/
  main.tex              # LaTeX source
  main.pdf              # Compiled preprint
  figures/              # Figures from experiments
```

## Getting started

### Hardware requirements

- **FPGA**: Digilent Arty A7-100T (or compatible Artix-7)
- **Ethernet**: Direct connection or same subnet as FPGA (192.168.0.50)
- **Vivado**: 2024.2+ for synthesis (optional — prebuilt bitstream available on request)

### Software requirements

```bash
pip install numpy scikit-learn
```

### Quick start — GPU reservoir (no FPGA needed)

```bash
# Build the GPU reservoir demo (requires ROCm/HIP)
cd examples
hipcc -O2 -o gpu_reservoir_demo gpu_reservoir_demo.hip

# Run (AMD gfx11 needs HSA override)
HSA_OVERRIDE_GFX_VERSION=11.0.0 ./gpu_reservoir_demo

# Or run the benchmark battery in simulation mode (no hardware)
pip install numpy scikit-learn
python benchmark_battery.py --sim
```

### Quick start — FPGA reservoir

1. **Program the FPGA** (requires Vivado + Arty A7):
   ```bash
   source /path/to/Vivado/settings64.sh
   vivado -mode batch -source fpga/scripts/build_eth.tcl
   vivado -mode batch -source fpga/scripts/program.tcl
   ```

2. **Connect and test**:
   ```python
   from scripts.fpga_host_eth import FPGAEthBridge

   fpga = FPGAEthBridge()
   fpga.connect()
   print(f"Neurons: {fpga.num_neurons}")

   # Read telemetry
   telem = fpga.read_telemetry()
   print(f"Spike counts: {telem['spike_counts'][:8]}")
   print(f"Membrane voltages: {telem['vmem'][:8]}")

   # Set gate voltage for neuron 0
   fpga.set_vg(0, 0.58)

   # Inject MAC current
   fpga.set_mac(0.5)

   fpga.close()
   ```

3. **Run the demos**:
   ```bash
   cd examples
   python reservoir_demo.py            # FPGA classification demo
   python benchmark_battery.py --fpga  # Full benchmark suite on FPGA
   ```

## FPGA neuron model

Each neuron implements the discrete-time LIF equation:

```
v_m[t+1] = v_m[t] + (dt/C) * (i_aval + i_exc + g_bias * MAC - g_leak * v_m[t])
```

- **i_aval**: Stochastic avalanche current from LFSR model, gated by V_g
- **i_exc**: Constant excitatory drive
- **g_bias * MAC**: GPU-injected neuromodulatory current
- **g_leak**: Leak conductance (configurable, default tau ~ 210 ms)

All parameters are **runtime-configurable** via UDP commands — no FPGA rebuild needed.

### Configurable parameters

| Parameter | Command | Default | Description |
|-----------|---------|---------|-------------|
| Gate voltage (V_g) | `SET_VG` | 0.62 V | Controls avalanche threshold per neuron |
| Leak conductance | `SET_LEAK` | 0.0001 | Membrane time constant (tau ~ 210 ms) |
| Threshold | `SET_THRESH` | 0.50 V | Spike threshold |
| MAC current | `SET_MAC` | 0.0 | Neuromodulatory input from host |
| Bias gain | `SET_BIAS_GAIN` | 0.125 | MAC-to-current conversion |
| Refractory period | `SET_REFRACT` | 5 us | Post-spike dead time |
| dt/C | `SET_DT_C` | 0.0078 | Integration step size |
| Base excitation | `SET_BASE_EXC` | varies | Constant drive current |

## NS-RAM connection

The FPGA model is calibrated to match NS-RAM (Neuromorphic Spiking RAM) device characteristics:

- **BV_par cliff**: Sharp avalanche onset at V_g ~ 0.55 V
- **Stochastic spiking**: LFSR-driven avalanche current mimics filament formation noise
- **Energy scale**: Femtojoules per spike (matching memristive device estimates)

The platform is designed for **hardware substitution**: replacing the FPGA model with real NS-RAM devices requires adapting the physical interface (ADC/DAC for membrane readout and gate voltage control), not the analysis pipeline.

## Citation

If you use this platform, please cite:

```bibtex
@article{feel-bridge-2026,
  title={Hybrid Analog--Analog Reservoir Computing: Bridging GPU Firmware
         Physics and Memristive Neuron Dynamics},
  author={{FEEL Research Project}},
  year={2026},
  note={Preprint}
}
```

## License

MIT License. See [LICENSE](LICENSE).
