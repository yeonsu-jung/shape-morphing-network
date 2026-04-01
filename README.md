# Shape-Morphing Network

Simulation code for shape morphing of elastic rod networks via differential growth.
A flat square lattice of elastic rods deforms into 3D curved surfaces (sphere-like or
saddle-like) by prescribing different growth rates to inner vs. outer edges.

This repository accompanies the publication:

**Rotational 3D printing of active-passive filaments and lattices with programmable shape morphing**.

Mustafa K Abdelrahman , Jackson Wilt , Yeonsu Jung , Rodrigo Telles , Gurminder Paink , Natalie M. Larson , Joanna Aizenberg, L. Mahadevan , Jennifer Lewis. *Proceedings of the National Academy of Sciences of the United States of America* (2026)

## Overview

The simulation is built on top of [**disMech**](https://github.com/StructuresComp/dismech-rods), a discrete elastic rod simulator.
The files in this repository are meant to be integrated into disMech as a custom example.

### How It Works

1. A square lattice of `num_side_vertices × num_side_vertices` nodes is constructed,
   with each edge represented by a sinusoidally-shaped elastic rod.
2. Edges are classified as **inner** (expand) or **outer** (shrink).
3. A controller applies differential growth by scaling the natural curvature (`kappa_bar`)
   of each rod:
   - **Expand edges**: `kappa_bar *= (1 - stretch)`
   - **Shrink edges**: `kappa_bar *= (1 + stretch)`
4. When `stretch > 0`, the lattice buckles into a **sphere-like** shape;
   when `stretch < 0`, it forms a **saddle-like** shape.

## File Descriptions

| File | Description |
|------|-------------|
| `robotDescription.cpp` | Defines the lattice geometry, rod material properties, boundary conditions, forces, and attaches the growth controller. This file is compiled with disMech as the robot description. |
| `snakeLatticeController.h` | Header for the custom controller that implements region-dependent differential growth. |
| `snakeLatticeController.cpp` | Implementation of the controller. At each timestep, it scales `kappa_bar` for expand and shrink edges by the prescribed `stretch` factor. |
| `snake_lattice_params.yml` | YAML configuration file containing all simulation parameters (material properties, lattice geometry, growth factor, etc.). |
| `run.py` | Helper script for launching parameter sweeps over `stretch` values on a SLURM cluster. |

## Integration with disMech

These files are designed to work with [disMech](https://github.com/StructuresComp/dismech-rods).
To run the simulation:

1. Clone and build disMech following its README.
2. Copy the files from this repository into the disMech project:
   - `robotDescription.cpp` → `<dismech-root>/robotDescription.cpp` (replaces the default)
   - `snakeLatticeController.h` → `<dismech-root>/src/controllers/snakeLatticeController.h`
   - `snakeLatticeController.cpp` → `<dismech-root>/src/controllers/snakeLatticeController.cpp`
   - `snake_lattice_params.yml` → `<dismech-root>/snake_lattice_params.yml`
3. Rebuild disMech (the controller files are picked up automatically by CMake).
4. Run the executable:
   ```bash
   ./disMech
   ```
   The `stretch` parameter can also be passed as a command-line argument.

## Parameters

Key parameters in `snake_lattice_params.yml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stretch` | 0.1 | Growth factor. Positive → sphere, negative → saddle. |
| `num_side_vertices` | 5 | Grid size (5×5 = 25 nodes, 40 edges). |
| `n` | 10 | Number of nodes per rod. |
| `step` | 1.0 | Lattice spacing. |
| `amplitude` | 0.25 | Sinusoidal perturbation amplitude for initial rod shapes. |
| `young_mod` | 1e6 | Young's modulus (Pa). |
| `radius` | 1e-3 | Rod cross-section radius (m). |
| `density` | 1000 | Material density (kg/m³). |
| `dt` | 1e-5 | Simulation timestep (s). |
| `sim_time` | 10 | Total simulation time (s). |

## License

This project is licensed under the [MIT License](LICENSE).
