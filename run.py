"""
Batch runner for shape-morphing network simulations.

Sweeps over a range of `stretch` values, creating a separate run folder for
each value.  Each folder gets a copy of the compiled disMech executable,
an updated snake_lattice_params.yml, and an SBATCH script for SLURM submission.

Usage (on a SLURM cluster):
    python run.py
"""
import os
import sys
from datetime import datetime
import subprocess
import yaml
import numpy as np

# ---- Configuration --------------------------------------------------------
RUNS_FOLDER = "/n/home01/yjung/Github/dismech-rods-main/runs"
EXE_DIR = "/n/home01/yjung/Github/dismech-rods-main/runs/20250405-0000_COMPILE_yml"
# ---------------------------------------------------------------------------


def create_result_folder(note):
    stamp = datetime.now().strftime("%Y%m%d-%H%M")
    result_folder = os.path.join(RUNS_FOLDER, f"{stamp}_COMPILE_{note}")
    if os.path.exists(result_folder):
        print(f"Folder {result_folder} already exists. Exiting.")
        sys.exit(1)
    os.makedirs(result_folder)
    return result_folder


def find_dismech_root():
    """Walk up from this script to find the dismech-rods-main root."""
    current = os.path.abspath(__file__)
    while current != os.path.dirname(current):
        if os.path.basename(current) == "dismech-rods-main":
            return current
        current = os.path.dirname(current)
    print("Root folder 'dismech-rods-main' not found.")
    sys.exit(1)


for stretch in np.arange(-0.75, +0.75, +0.01):
    run_id = f"snake_lattice_stretch{stretch:.2f}"
    print(f"Run ID: {run_id}")
    result_folder = create_result_folder(run_id)
    root_folder = find_dismech_root()

    # Copy source files and executable into the run folder
    for src, dst in [
        (f"{EXE_DIR}/robotDescription.cpp", "robotDescription.cpp"),
        (f"{EXE_DIR}/disMech", "disMech"),
        (f"{root_folder}/examples/snake_lattice/snake_lattice_params.yml", "snake_lattice_params.yml"),
    ]:
        subprocess.run(["cp", src, os.path.join(result_folder, dst)], check=True)

    # Update stretch value in the YAML config
    yml_path = os.path.join(result_folder, "snake_lattice_params.yml")
    with open(yml_path, "r") as f:
        yml_dict = yaml.safe_load(f)
    yml_dict["stretch"] = float(stretch)
    with open(yml_path, "w") as f:
        yaml.dump(yml_dict, f, default_flow_style=False)

    # Generate SLURM submission scripts
    run_sh = f"""#!/bin/bash
cd {result_folder}
sbatch Sbatch.sh
"""
    sbatch_sh = f"""#!/bin/bash
#SBATCH -n 1
#SBATCH -c 1
#SBATCH -N 1
#SBATCH -t 0-12:00
#SBATCH -p seas_compute
#SBATCH --mem=1500
#SBATCH -o output_%j.out
#SBATCH -e errors_%j.err

module load intel
module load intel-mkl

export LD_PRELOAD=/n/sw/helmod-rocky8/apps/Core/gcc/13.2.0-fasrc01/lib64/libstdc++.so.6
time ./disMech {stretch}
"""
    with open(os.path.join(result_folder, "run.sh"), "w") as f:
        f.write(run_sh)
    with open(os.path.join(result_folder, "Sbatch.sh"), "w") as f:
        f.write(sbatch_sh)

