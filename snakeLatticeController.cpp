#include "snakeLatticeController.h"
#include "rod_mechanics/softRobots.h"

#include <iostream>

snakeLatticeController::snakeLatticeController(const shared_ptr<softRobots>& soft_robots,
                                               std::vector<int> _expand_edge_indices,
                                               std::vector<int> _shrink_edge_indices,
                                               double start_time, double end_time, double stretch) :
        baseController(soft_robots->limbs),
        start_time(start_time), end_time(end_time), stretch(stretch)
{
    initial_kappa_bar.resize(num_actuators);
    for (int limb_idx = 0; limb_idx < num_actuators; limb_idx++) {
        initial_kappa_bar[limb_idx] = limbs[limb_idx]->kappa_bar;
    }
    expand_edge_indices = _expand_edge_indices;
    shrink_edge_indices = _shrink_edge_indices;

    for (int limb_idx = 0; limb_idx < num_actuators; limb_idx++) {
        auto limb = limbs[limb_idx];
        std::vector<double> ref_len_vec;
        for (int i = 1; i < limb->ne; i++) {
            ref_len_vec.push_back(limb->edge_len(i));
        }
        initial_ref_len.push_back(ref_len_vec);
    }

    cout << "snakeLatticeController initialized with stretch: " << stretch << endl;
}

snakeLatticeController::~snakeLatticeController() = default;

void snakeLatticeController::updateTimeStep(double dt) {
    baseController::updateTimeStep(dt);

    // Scale kappa_bar for expand edges (reduce curvature)
    for (auto limb_idx : expand_edge_indices) {
        auto limb = limbs[limb_idx];
        for (int i = 1; i < limb->ne; i++) {
            limb->kappa_bar(i, 0) = initial_kappa_bar[limb_idx](i, 0) * (1 - stretch);
            limb->kappa_bar(i, 1) = initial_kappa_bar[limb_idx](i, 1) * (1 - stretch);
        }
    }

    // Scale kappa_bar for shrink edges (increase curvature)
    for (auto limb_idx : shrink_edge_indices) {
        auto limb = limbs[limb_idx];
        for (int i = 1; i < limb->ne; i++) {
            limb->kappa_bar(i, 0) = initial_kappa_bar[limb_idx](i, 0) * (1 + stretch);
            limb->kappa_bar(i, 1) = initial_kappa_bar[limb_idx](i, 1) * (1 + stretch);
        }
    }
}
