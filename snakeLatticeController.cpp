#include "snakeLatticeController.h"
#include "rod_mechanics/softRobots.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

snakeLatticeController::snakeLatticeController(const shared_ptr <softRobots> &soft_robots,
                                                           std::vector<int> _expand_edge_indices, std::vector<int> _shrink_edge_indices, 
                                                           double start_time, double end_time, double stretch) :
                                                           baseController(soft_robots->limbs),
                                                           start_time(start_time), end_time(end_time), stretch(stretch)
{    
    initial_kappa_bar.resize(num_actuators);
    for (int limb_idx = 0; limb_idx < num_actuators; limb_idx++) {
        auto limb = limbs[limb_idx];
        initial_kappa_bar[limb_idx] = limb->kappa_bar;
    }
    expand_edge_indices = _expand_edge_indices;
    shrink_edge_indices = _shrink_edge_indices;
    
    for (int limb_idx = 0; limb_idx < num_actuators; limb_idx++) {
        auto limb = limbs[limb_idx];
        std::vector<double> ref_len_vec;
        for (int i = 1; i < limb->ne; i++) {
            
            // limb->ref_len(i) = limb->edge_len(i) * 1.00005;
            ref_len_vec.push_back(limb->edge_len(i));
        }
        initial_ref_len.push_back(ref_len_vec);
    }

    cout << "snakeLatticeController initialized with stretch: " << stretch << endl;

}

snakeLatticeController::~snakeLatticeController() = default;

void snakeLatticeController::updateTimeStep(double dt) {
    baseController::updateTimeStep(dt);

    // if (current_time < start_time || current_time > end_time) return;

    double curr_ratio = (current_time - start_time) / (end_time - start_time);
    double shrink_ratio = 1 - curr_ratio;
    if (shrink_ratio < 0.5) shrink_ratio = 0.5;

    double expand_ratio = 1 + curr_ratio;
    double upper_bound = 1.5;
    if (expand_ratio > upper_bound) expand_ratio = upper_bound;

    
    for (auto limb_idx : expand_edge_indices) {
        auto limb = limbs[limb_idx];
        for (int i = 1; i < limb->ne; i++) {
            limb->kappa_bar(i, 0) = initial_kappa_bar[limb_idx](i, 0) * (1-stretch);
            limb->kappa_bar(i, 1) = initial_kappa_bar[limb_idx](i, 1) * (1-stretch);
            
            // limb->ref_len(i) = initial_ref_len[limb_idx][i-1] * 1.15;
        }
    }
    for (auto limb_idx : shrink_edge_indices) {
        auto limb = limbs[limb_idx];
        for (int i = 1; i < limb->ne; i++) {
            limb->kappa_bar(i, 0) = initial_kappa_bar[limb_idx](i, 0) * (1+stretch);
            limb->kappa_bar(i, 1) = initial_kappa_bar[limb_idx](i, 1) * (1+stretch);
            
        }
    }

    // for (int limb_idx = 0; limb_idx < num_actuators; limb_idx++) {
    //     auto limb = limbs[limb_idx];

    //     // if limb_idx in expand_edge_indices


    //     for (int i = 1; i < limb->ne; i++) {            

    //         limb->kappa_bar(i, 0) = initial_kappa_bar[limb_idx](i, 0) * expand_ratio;
    //         limb->kappa_bar(i, 1) = initial_kappa_bar[limb_idx](i, 1) * expand_ratio;
    //     }
    // }

}
