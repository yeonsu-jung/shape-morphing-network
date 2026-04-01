#include "robotDescription.h"
#include <yaml-cpp/yaml.h>

extern ofstream logging_output_file;  // defined in main.cpp

/*
 * Shape-Morphing Network
 *
 * Constructs a square lattice of elastic rods with sinusoidal initial shapes.
 * Differential growth is applied via the snakeLatticeController: inner edges
 * expand while outer edges shrink, driving the flat lattice into a curved
 * 3D shape (sphere-like for stretch > 0, saddle-like for stretch < 0).
 */
void get_robot_description(int argc, char** argv,
                           const shared_ptr<softRobots>& soft_robots,
                           const shared_ptr<forceContainer>& forces,
                           shared_ptr<worldLogger>& logger,
                           simParams& sim_params) {

    YAML::Node config = YAML::LoadFile("snake_lattice_params.yml");

    // Simulation parameters
    double dt = config["dt"].as<double>();
    double sim_time = config["sim_time"].as<double>();

    // Rod physical properties
    int n = config["n"].as<int>();                       // Number of nodes per rod
    double radius = config["radius"].as<double>();
    double young_mod = config["young_mod"].as<double>();
    double density = config["density"].as<double>();
    double poisson = config["poisson"].as<double>();
    double mu = config["mu"].as<double>();               // Friction coefficient
    double viscosity = config["viscosity"].as<double>();
    double velocity_amplitude = config["velocity_amplitude"].as<double>();
    double k_scaler = config["k_scaler"].as<double>();   // Contact stiffness
    double nu = config["nu"].as<double>();               // Floor friction coefficient

    // Lattice geometry
    int num_side_vertices = config["num_side_vertices"].as<int>();
    double step = config["step"].as<double>();
    double amplitude = config["amplitude"].as<double>(); // Sinusoidal perturbation amplitude

    // Output / logging
    int cmd_line_per = config["cmd_line_per"].as<int>();
    int num_frames = config["num_frames"].as<int>();

    // Growth and gravity
    double stretch = config["stretch"].as<double>();     // > 0 for sphere, < 0 for saddle
    double gravity_magnitude = config["gravity_magnitude"].as<double>();
    double delta = radius / 10;                          // Contact distance for floor
    double floor_z = -radius * 4;

    // Simulation settings
    sim_params.dt = dt;
    sim_params.sim_time = sim_time;
    sim_params.ftol = 1e-3;
    sim_params.cmd_line_per = cmd_line_per;
    sim_params.adaptive_time_stepping = 7;



    // ---------------------------------------------------------------
    // Build the square lattice of elastic rods
    // ---------------------------------------------------------------
    int num_nodes = num_side_vertices * num_side_vertices;
    int total_horizontal_limbs = num_side_vertices * (num_side_vertices - 1);
    int total_vertical_limbs = (num_side_vertices - 1) * num_side_vertices;
    int total_limbs = total_horizontal_limbs + total_vertical_limbs;

    vector<int> limb_start_node(total_limbs);
    vector<int> limb_end_node(total_limbs);

    // Map from node index to vector of (limb_idx, m_node_idx)
    vector<vector<pair<int, int>>> node_to_limbs(num_nodes);

    int limb_idx = 0;

    // Create horizontal limbs (along x direction)
    for (int i = 0; i < num_side_vertices; i++) {
        for (int j = 0; j < num_side_vertices - 1; j++) {
            Vector3d start_pos(step * j, step * i, 0.0);
            Vector3d end_pos(step * (j + 1), step * i, 0.0);

            vector<Eigen::Vector3d> nodes;
            Vector3d dir = end_pos - start_pos;
            Vector3d perp(0, -1, 0);

            for (int k = 0; k < n; k++) {
                double t = static_cast<double>(k) / (n - 1);
                Vector3d point = start_pos + dir * t;
                point += perp * amplitude * sin(2 * M_PI * t);
                nodes.push_back(point);
            }
            soft_robots->addLimb(nodes, density, radius, young_mod, poisson, mu);

            int start_node_idx = i * num_side_vertices + j;
            int end_node_idx = i * num_side_vertices + j + 1;
            limb_start_node[limb_idx] = start_node_idx;
            limb_end_node[limb_idx] = end_node_idx;
            node_to_limbs[start_node_idx].push_back(make_pair(limb_idx, 0));
            node_to_limbs[end_node_idx].push_back(make_pair(limb_idx, -1));
            limb_idx++;
        }
    }

    // Create vertical limbs (along y direction)
    for (int i = 0; i < num_side_vertices - 1; i++) {
        for (int j = 0; j < num_side_vertices; j++) {
            Vector3d start_pos(step * j, step * i, 0.0);
            Vector3d end_pos(step * j, step * (i + 1), 0.0);

            vector<Eigen::Vector3d> nodes;
            Vector3d dir = end_pos - start_pos;
            Vector3d perp(1, 0, 0);

            for (int k = 0; k < n; k++) {
                double t = static_cast<double>(k) / (n - 1);
                Vector3d point = start_pos + dir * t;
                point += perp * amplitude * sin(2 * M_PI * t);
                nodes.push_back(point);
            }
            soft_robots->addLimb(nodes, density, radius, young_mod, poisson, mu);

            int start_node_idx = i * num_side_vertices + j;
            int end_node_idx = (i + 1) * num_side_vertices + j;
            limb_start_node[limb_idx] = start_node_idx;
            limb_end_node[limb_idx] = end_node_idx;
            node_to_limbs[start_node_idx].push_back(make_pair(limb_idx, 0));
            node_to_limbs[end_node_idx].push_back(make_pair(limb_idx, -1));
            limb_idx++;
        }
    }

    // ---------------------------------------------------------------
    // Create joints at lattice vertices
    // ---------------------------------------------------------------
    for (int node_idx = 0; node_idx < num_nodes; node_idx++) {
        if (node_to_limbs[node_idx].empty()) continue;

        int first_limb_idx = -1;
        int first_m_node_idx = -1;
        for (const auto& limb_info : node_to_limbs[node_idx]) {
            first_limb_idx = limb_info.first;
            first_m_node_idx = limb_info.second;
            break;
        }
        if (first_limb_idx == -1) continue;

        soft_robots->createJoint(first_limb_idx, first_m_node_idx);

        for (const auto& limb_info : node_to_limbs[node_idx]) {
            if (limb_info.first == first_limb_idx) continue;
            soft_robots->addToJoint(node_idx, limb_info.first, limb_info.second);
        }
    }
    soft_robots->setup();

    // ---------------------------------------------------------------
    // Classify edges: inner (expand) vs outer (shrink)
    // ---------------------------------------------------------------
    std::vector<int> expand_edge_indices = {5, 6, 9, 10, 13, 14, 26, 27, 28, 31, 32, 33};
    std::vector<int> shrink_edge_indices = {0, 1, 2, 3, 4, 7, 8, 11, 12, 15, 16, 17, 18, 19,
                                            20, 21, 22, 23, 24, 25, 34, 35, 36, 37, 38, 39};

    // Boundary edge groups (for saddle initial velocities)
    std::vector<int> left_side = {25, 30};
    std::vector<int> right_side = {29, 34};
    std::vector<int> top_side = {17, 18};
    std::vector<int> bottom_side = {1, 2};

    // ---------------------------------------------------------------
    // Apply initial velocities to break symmetry
    // ---------------------------------------------------------------
    vector<Vector3d> positive_velocities(n, Vector3d(0.0, 0.0, velocity_amplitude));
    vector<Vector3d> negative_velocities(n, Vector3d(0.0, 0.0, -velocity_amplitude));

    if (stretch > 0.0) {
        // Sphere: expand edges go up, shrink edges go down
        for (int i = 0; i < limb_idx; ++i) {
            if (std::find(expand_edge_indices.begin(), expand_edge_indices.end(), i) != expand_edge_indices.end()) {
                soft_robots->applyInitialVelocities(i, positive_velocities);
            } else if (std::find(shrink_edge_indices.begin(), shrink_edge_indices.end(), i) != shrink_edge_indices.end()) {
                soft_robots->applyInitialVelocities(i, negative_velocities);
            }
        }
    } else {
        // Saddle: boundary edges get opposite velocities depending on side
        for (int i = 0; i < limb_idx; ++i) {
            if (std::find(shrink_edge_indices.begin(), shrink_edge_indices.end(), i) != shrink_edge_indices.end()) {
                if (std::find(left_side.begin(), left_side.end(), i) != left_side.end() ||
                    std::find(right_side.begin(), right_side.end(), i) != right_side.end()) {
                    soft_robots->applyInitialVelocities(i, negative_velocities);
                } else if (std::find(top_side.begin(), top_side.end(), i) != top_side.end() ||
                           std::find(bottom_side.begin(), bottom_side.end(), i) != bottom_side.end()) {
                    soft_robots->applyInitialVelocities(i, positive_velocities);
                }
            }
        }
    }
    
    // Ensure expand and shrink lists are disjoint and unique
    std::unordered_set<int> shrink_edge_set(shrink_edge_indices.begin(), shrink_edge_indices.end());
    expand_edge_indices.erase(
        std::remove_if(expand_edge_indices.begin(), expand_edge_indices.end(),
                       [&shrink_edge_set](int idx) { return shrink_edge_set.count(idx) > 0; }),
        expand_edge_indices.end());

    auto remove_duplicates = [](std::vector<int>& vec) {
        std::sort(vec.begin(), vec.end());
        vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
    };
    remove_duplicates(expand_edge_indices);
    remove_duplicates(shrink_edge_indices);

    // ---------------------------------------------------------------
    // Forces
    // ---------------------------------------------------------------
    forces->addForce(make_shared<dampingForce>(soft_robots, viscosity));
    forces->addForce(make_shared<floorContactForce>(soft_robots, delta, k_scaler, nu, floor_z, mu));
    Vector3d gravity_vector(0.0, 0.0, -gravity_magnitude);
    forces->addForce(make_shared<gravityForce>(soft_robots, gravity_vector));

    // ---------------------------------------------------------------
    // Controller: differential growth
    // ---------------------------------------------------------------
    double start_time = 0.0;
    double end_time = 500.0;
    soft_robots->addController(make_shared<snakeLatticeController>(
        soft_robots, expand_edge_indices, shrink_edge_indices, start_time, end_time, stretch));

    // ---------------------------------------------------------------
    // Logger
    // ---------------------------------------------------------------
    string logfile_base = "log_files";
    int logging_period = sim_time / dt / num_frames;
    logger = make_shared<rodNodeLogger>(logfile_base, logging_output_file, logging_period);
}
