#ifndef SNAKE_LATTICE_CONTROLLER__H
#define SNAKE_LATTICE_CONTROLLER__H

#include "baseController.h"

class softRobots;

class snakeLatticeController : public baseController
{
public:
    explicit snakeLatticeController(const shared_ptr<softRobots>& soft_robots,
                                    std::vector<int> expand_edge_indices, std::vector<int> shrink_edge_indices,
                                    double start_time, double end_time, double stretch);
    ~snakeLatticeController();

    void updateTimeStep(double dt) override;

private:
    std::vector<Eigen::MatrixXd> initial_kappa_bar;
    std::vector<int> expand_edge_indices;
    std::vector<int> shrink_edge_indices;

    std::vector<std::vector<double>> initial_ref_len;

    double start_time;
    double end_time;

    double stretch;
};

#endif
