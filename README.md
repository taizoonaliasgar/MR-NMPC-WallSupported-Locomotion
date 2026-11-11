# Multi-Rate NMPC for Wall-Supported Bipedal Locomotion of Quadrupedal Robots

[![arXiv](https://img.shields.io/badge/arXiv-2607.01574-b31b1b.svg)](https://arxiv.org/abs/2607.01574)
[![IROS 2026](https://img.shields.io/badge/IROS-2026-blue.svg)](https://arxiv.org/abs/2607.01574)
[![Video](https://img.shields.io/badge/Video-YouTube-red.svg)](https://www.youtube.com/watch?v=N6XwNJ0E9s0)
[![Project page](https://img.shields.io/badge/Project-Page-791731.svg)](https://taizoonaliasgar.github.io/bipedal-mr-nmpc5.html)

Official code for the IROS 2026 paper:

> **Multi-Rate Nonlinear Model Predictive Control for Wall-Supported Bipedal Locomotion of Quadrupedal Robots**<br>
> Taizoon Chunawala, Jeeseop Kim, and Kaveh Akbari Hamed<br>
> *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2026*<br>
> [[arXiv]](https://arxiv.org/abs/2607.01574)

<p align="center"><img src="https://taizoonaliasgar.github.io/assets/bipedal/snapshots.png" width="720" alt="Wall-supported bipedal locomotion with MR-NMPC and the Raibert-heuristic baseline"></p>

This repository has a layered planning and control framework that lets a Unitree A1 quadruped walk upright on its hind legs while bracing its front legs against the walls of a narrow corridor.

- **High level: multi-rate NMPC (MR-NMPC), 100 Hz.** Uses a single rigid body (SRB) model. It jointly optimizes the CoM and orientation trajectories, the environment reaction forces (fast inputs), and the foothold locations (slow inputs). Footholds are added to the prediction model as states. An indicator function lets their step-length inputs change only at gait-domain switching instants.
- **Low level: nonlinear whole-body controller (WBC), 1 kHz.** A QP based on virtual constraints. It enforces the full-order dynamics, friction cones, and torque limits while tracking the MR-NMPC references.
- **Key idea.** Footholds are part of the SRB prediction model's state, so step locations and body motion are optimized together rather than set by a heuristic.
- **Baseline.** The same pipeline with a Raibert-heuristic foot placement, used for comparison.

All results are simulated in [RaiSim](https://raisim.com/) on a 45 cm wide corridor with random rough terrain and external pushes.

## Results

- **2.9x higher success** than the Raibert-heuristic baseline (68.8% vs 23.6%) over 250 randomized rough-terrain trials in simulation.
- MR-NMPC solves in **8.3 ms on average at 100 Hz**, tracked by the 1 kHz QP-based WBC.
- Preliminary hardware deployment on a Unitree Go2 (balanced for 60+ s), shown in the [video](https://www.youtube.com/watch?v=N6XwNJ0E9s0). Hardware code is not included in this repository.

## Repository layout

| Path | Contents |
|---|---|
| `src/SRBNMPC.cpp`, `include/SRBNMPC.hpp` | MR-NMPC formulation (CasADi + IPOPT) with footstep planning |
| `src/SRBNMPCR.cpp`, `include/SRBNMPCR.hpp` | Baseline NMPC with Raibert-heuristic footholds |
| `src/LowLevelCtrl.cpp`, `src/VirtualConstraints.cpp` | QP-based whole-body controller |
| `src/LocoWrapperwalk.cpp`, `src/MotionPlanner.cpp` | Gait scheduling and glue between the two layers |
| `src/upright_NMPC.cpp` | Generates and compiles the MR-NMPC solver to a shared library |
| `src/A1_up_walk.cpp` | Wall-supported walking simulation with **MR-NMPC** (`uprightwalk`) |
| `src/A1_up_walk_Raib.cpp` | Same simulation with the **Raibert baseline** (`uprightwalkR`) |
| `LL_integrator/dynamics/` | Full-order A1 dynamics (mass matrix, Jacobians, FK) |
| `params/` | Controller, gait, and walking parameter files |
| `rsc/` | URDFs, meshes, and materials |

## Dependencies

Tested on Ubuntu with:

- [RaiSim](https://raisim.com/) and raisimOgre (needs a RaiSim activation key)
- [CasADi](https://web.casadi.org/) (C++), built with [IPOPT](https://github.com/coin-or/Ipopt)
- [Eigen3](https://eigen.tuxfamily.org/)
- qpSWIFT, bundled in `optimization.zip`
- CMake ≥ 3.12, a C++20 compiler, and `gcc` (used at runtime to compile the generated NMPC code)

## Build

```bash
git clone https://github.com/taizoonaliasgar/MR-NMPC-WallSupported-Locomotion.git
cd MR-NMPC-WallSupported-Locomotion
unzip optimization.zip          # extracts the qpSWIFT sources into optimization/
```

`CMakeLists.txt` currently points to local install paths for RaiSim, OGRE, raisimOgre, and Eigen (the `find_package(... HINTS ...)` lines). Change them to your own install locations, then build:

```bash
mkdir -p build && cd build
cmake ..
make -j
```

Place your RaiSim activation key where the executables expect it (`activation.raisim`, loaded through `raisim::loadResource`).

## Run

Run all commands from the `build/` directory.

1. **Generate the MR-NMPC solver.** This builds the NLP in CasADi, generates C code, and compiles it to `upright_Jun29_symm.so` in the current directory:
   ```bash
   ./upright_NMPC ../params/LL_w_CLF.txt ../params/Walking_params.txt
   ```
2. **Run wall-supported walking with MR-NMPC:**
   ```bash
   ./uprightwalk ../params/LL_w_CLF.txt ../params/Walking_params.txt <terrain_seed>
   ```
3. **Run the Raibert baseline:**
   ```bash
   ./uprightwalkR ../params/LL_w_CLF.txt ../params/Walking_params.txt <terrain_seed>
   ```

`<terrain_seed>` is an integer that selects the random rough-terrain layout. Use the same seed for both controllers to compare them on the same terrain. Parameter files are matched by name: files containing `LL`, `MPC`, or `Walking` are loaded as low-level, MPC, or walking parameters.

## Citation

If you use this code, please cite:

```bibtex
@inproceedings{chunawala2026multirate,
  title     = {Multi-Rate Nonlinear Model Predictive Control for Wall-Supported Bipedal Locomotion of Quadrupedal Robots},
  author    = {Chunawala, Taizoon and Kim, Jeeseop and Akbari Hamed, Kaveh},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2026},
  note      = {arXiv:2607.01574}
}
```

Until the IROS proceedings are published, you can also cite the arXiv preprint:

```bibtex
@misc{chunawala2026multirate_arxiv,
  title         = {Multi-Rate Nonlinear Model Predictive Control for Wall-Supported Bipedal Locomotion of Quadrupedal Robots},
  author        = {Chunawala, Taizoon and Kim, Jeeseop and Akbari Hamed, Kaveh},
  year          = {2026},
  eprint        = {2607.01574},
  archivePrefix = {arXiv},
  url           = {https://arxiv.org/abs/2607.01574}
}
```

## Acknowledgments

The work of T. Chunawala is partially supported by the National Science Foundation (NSF) under Grant 2306984. The work of K. Akbari Hamed is supported by the NSF under Grants 2024772 and 2423725.

## Contact

Taizoon Chunawala, Department of Mechanical Engineering, Virginia Tech. Email: taizoon.chunawala@gmail.com. Webpage: <https://taizoonaliasgar.github.io/>
