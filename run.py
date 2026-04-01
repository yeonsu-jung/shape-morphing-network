import os
import sys
from datetime import datetime
import subprocess
import yaml


RUNS_FOLDER = "/n/home01/yjung/Github/dismech-rods-main/runs"
def create_result_folder(NOTE):
    # RUNS_FOLDER = "runs"
    
    STAMP = datetime.now().strftime("%Y%m%d-%H%M")
    RESULT_FOLDER = os.path.join(RUNS_FOLDER, f"{STAMP}_COMPILE_{NOTE}")

    # Exit if the folder exists
    if os.path.exists(RESULT_FOLDER):
        print(f"Folder {RESULT_FOLDER} already exists. Exiting.")
        sys.exit(1)

    os.makedirs(RESULT_FOLDER)
    return RESULT_FOLDER


# run_id = f"logjam_test_N{to_update["num_rods"]}_L{to_update["rod_length"]}_R{to_update["rod_radius"]}_AR{aspect_ratio}_mu{to_update["mu"]}_U{to_update["flow_speed"]}_T{to_update["sim_time"]}_G{to_update["gate_opening"]}"
# print(f"Run ID: {run_id}")

# RESULT_FOLDER = create_result_folder(run_id)
# EXE_DIR="/n/home01/yjung/Github/dismech-rods-main/runs/20250129-1238_COMPILE_"

EXE_DIR="/n/home01/yjung/Github/dismech-rods-main/runs/20250405-0000_COMPILE_yml"
EXE_PATH="{EXE_DIR}/disMech"

# /n/home01/yjung/Github/dismech-rods-main/runs/20250403-2339_COMPILE_snake_lattice/disMech

# stretch = -0.4
import numpy as np

# for stretch in [-0.1,-0 -0.4]:
# for stretch in np.arange(-0.01, -0.5, -0.01):
for stretch in np.arange(-0.75, +0.75, +0.01):
# for stretch in [0.4]:
    run_id = f'snake_lattice_stretch{stretch:.2f}'
    print(f"Run ID: {run_id}")
    RESULT_FOLDER = create_result_folder(run_id)

    # Find the root folder named "dismech-rods-main"
    current_path = os.path.abspath(__file__)
    root_folder = None

    while current_path != os.path.dirname(current_path):
        if os.path.basename(current_path) == "dismech-rods-main":
            root_folder = current_path
            break
        current_path = os.path.dirname(current_path)

    if root_folder is None:
        print("Root folder 'dismech-rods-main' not found.")
        sys.exit(1)

    print(f"Root folder found: {root_folder}")


    # Copy files
    src_file = f"{root_folder}/examples/logjam/logjam.cpp"
    subprocess.run(["cp", src_file, "robotDescription.cpp"], check=True)
    subprocess.run(["cp", "robotDescription.cpp", os.path.join(RESULT_FOLDER, "robotDescription.cpp")], check=True)
    subprocess.run(["cp", f"{root_folder}/examples/snake_lattice/run.py", os.path.join(RESULT_FOLDER, "run.py")], check=True)

    subprocess.run(["cp", f"{root_folder}/examples/snake_lattice/snake_lattice_params.yml", os.path.join(RESULT_FOLDER, "snake_lattice_params.yml")], check=True)

    this_file_path = os.path.abspath(__file__)
    subprocess.run(["cp", this_file_path, os.path.join(RESULT_FOLDER, os.path.basename(this_file_path))], check=True)
    subprocess.run(["cp", f"{EXE_DIR}/robotDescription.cpp", os.path.join(RESULT_FOLDER, "robotDescription.cpp")], check=True)
    subprocess.run(["cp", f"{EXE_DIR}/disMech", os.path.join(RESULT_FOLDER, "disMech")], check=True)

    # read snake_lattice_params.yml
    with open(os.path.join(RESULT_FOLDER, "snake_lattice_params.yml"), "r") as f:
        yml_content = f.read()
    
    try:
        yml_dict = yaml.safe_load(yml_content)
    except yaml.YAMLError as exc:
        print(f"Error reading YAML file: {exc}")
        sys.exit(1)

    yml_dict["stretch"] = float(stretch)
    
    # dict to yaml
    yml_content_updated = yaml.dump(yml_dict, default_flow_style=False)
    # Write the updated content back to the file
    with open(os.path.join(RESULT_FOLDER, "snake_lattice_params.yml"), "w") as f:
        if f.writable():
            f.write(yml_content_updated)
        else:
            print("Error: The file is not writable.")
            sys.exit(1)
    

    # os.chdir(f"{RESULT_FOLDER}")
    # subprocess.run(["python", "prepare_initial_condition.py", run_id])


    stupid_run_txt = f"""#!/bin/bash
    cd {RESULT_FOLDER}
    sbatch Sbatch.sh
    """

    sbatch_txt = f"""#!/bin/bash
#SBATCH -n 1                # Number of cores (-n)
#SBATCH -c 1                # Number of threads per core (-c)
#SBATCH -N 1                # Ensure that all cores are on one Node (-N)
#SBATCH -t 0-12:00          # Runtime in D-HH:MM, minimum of 10 minutes
#SBATCH -p seas_compute             # Partition to submit to
#SBATCH --mem=1500           # Memory pool for all cores (see also --mem-per-cpu)
#SBATCH -o output_%j.out  # File to which STDOUT will be written, %j inserts jobid
#SBATCH -e errors_%j.err  # File to which STDERR will be written, %j inserts jobid
#SBATCH --mail-type=END
#SBATCH --mail-user=jung@seas.harvard.edu

module load intel
module load intel-mkl

export LD_PRELOAD=/n/sw/helmod-rocky8/apps/Core/gcc/13.2.0-fasrc01/lib64/libstdc++.so.6
time ./disMech {stretch}
    """
    # /n/home01/yjung/.conda/envs/simdata-analysis/bin/python logjam_quick_check.py {run_id}
    # python logjam_quick_check.py {run_id}

    with open(f"{RESULT_FOLDER}run.sh", "w") as file:
        file.write(stupid_run_txt)
    with open(f"{RESULT_FOLDER}/Sbatch.sh", "w") as file:
        file.write(sbatch_txt)

