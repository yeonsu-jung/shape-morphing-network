#include "robotDescription.h"
#include <random>
#include <yaml-cpp/yaml.h>

extern ofstream logging_output_file;  // defined in main.cpp
/*
 * Spider on Incline Example
 *
 * Define your soft robot structure(s), boundary conditions,
 * custom external forces, and loggers in the function below.
 */
void get_robot_description(int argc, char** argv,
                           const shared_ptr<softRobots>& soft_robots,
                           const shared_ptr<forceContainer>& forces,
                           shared_ptr<worldLogger>& logger,
                           simParams& sim_params) {

    

    YAML::Node config = YAML::LoadFile("snake_lattice_params.yml");
        
    double dt = config["dt"].as<double>(); // Time step for the simulation
    double sim_time = config["sim_time"].as<double>(); // Total simulation time

    int n = config["n"].as<int>(); // n here is number of ghost nodes per snake
    double radius = config["radius"].as<double>();
    double young_mod = config["young_mod"].as<double>();
    double density = config["density"].as<double>();
    double poisson = config["poisson"].as<double>();
    double mu = config["mu"].as<double>();
    double viscosity = config["viscosity"].as<double>();
    double velocity_amplitude = config["velocity_amplitude"].as<double>();
    double k_scaler = config["k_scaler"].as<double>(); // Contact stiffness for floor
    double nu = config["nu"].as<double>(); // Friction coefficient for floor

    int num_side_vertices = config["num_side_vertices"].as<int>();  // Number of vertices along one side of the square lattice
    double step = config["step"].as<double>(); // Step size for the lattice
    double amplitude = config["amplitude"].as<double>(); // Amplitude for the sinusoidal perturbation along the limbs
    
    int cmd_line_per = config["cmd_line_per"].as<int>(); // Command line sim info output period
    int num_frames = config["num_frames"].as<int>(); // Number of frames to log for visualization (if applicable)

    double stretch = config["stretch"].as<double>(); // Stretching factor for the snake lattice, > 0 for sphere, < 0 for saddle
    double gravity_magnitude = config["gravity_magnitude"].as<double>(); // Gravitational acceleration (m/s^2)
    double delta = radius / 10; // Contact distance for floor, this should be small compared to the radius of the nodes 


    double friction = true;


    cout << "Configuration Parameters:" << endl;
    cout << "dt: " << dt << endl;
    cout << "sim_time: " << sim_time << endl;
    cout << "n (ghost nodes per snake): " << n << endl;
    cout << "radius: " << radius << endl;
    cout << "young_mod: " << young_mod << endl;
    cout << "density: " << density << endl;
    cout << "poisson: " << poisson << endl;
    cout << "mu (friction coefficient): " << mu << endl;
    cout << "viscosity: " << viscosity << endl;
    cout << "velocity_amplitude: " << velocity_amplitude << endl;
    cout << "k_scaler (contact stiffness for floor): " << k_scaler << endl;
    cout << "nu (friction coefficient for floor): " << nu << endl;
    cout << "num_side_vertices: " << num_side_vertices << endl;
    cout << "step: " << step << endl;
    cout << "amplitude (for sinusoidal perturbation): " << amplitude << endl;
    cout << "stretch (for snake lattice): " << stretch << endl;
    cout << "gravity_magnitude: " << gravity_magnitude << endl;
    cout << "delta (contact distance for floor): " << delta << endl;
    

    

    
    
    // double nu = sim_params.nu;
    // double k_scaler = 1e7;
    // bool self_contact = false;
    // double mu = 0.4;
    // double stretch = -0.5;
    // int n = 10; // number of ghost nodes per snake
    // double radius = 1e-3;
    // double young_mod = 2e11;
    // double density = 1180;
    // double poisson = 0.5;
    // double gravity_magnitude = 0.; // Gravitational acceleration (m/s^2)
    // double viscosity = .1;
    // double k_scaler = 1e7; // Contact stiffness for floor
    // double velocity_amplitude = 10.;
    // double delta = radius / 10; // Contact distance for floor
    // double nu = 1e-3; // Friction coefficient for floor    
    // int num_side_vertices = 5;  // Adjust num_side_vertices as needed for the lattice
    // double step = 1; // Adjust step size based on desired lattice size
    // double amplitude = step / 4;
    // double sim_time = 10;
    // double dt = 1e-5;

    double floor_z = -radius*4;
    sim_params.dt = dt;
    sim_params.sim_time = sim_time;
    sim_params.ftol = 1e-3;
    sim_params.cmd_line_per = cmd_line_per;
    
    
    

    // sim_params.render_scale = 3.0;
    // sim_params.show_mat_frames = true;
    sim_params.adaptive_time_stepping = 7;    
    






    int num_nodes = num_side_vertices * num_side_vertices;
    int total_horizontal_limbs = num_side_vertices * (num_side_vertices - 1);
    int total_vertical_limbs = (num_side_vertices - 1) * num_side_vertices;
    int total_limbs = total_horizontal_limbs + total_vertical_limbs;

    vector<int> limb_start_node(total_limbs);
    vector<int> limb_end_node(total_limbs);

    // Map from node index to vector of (limb_idx, m_node_idx)
    vector< vector< pair<int, int> > > node_to_limbs(num_nodes);
    std::random_device rd;  // Non-deterministic random device
    std::default_random_engine generator(rd());

    
    
    // Define a normal distribution with mean 0.0 and standard deviation 1.0
    std::normal_distribution<double> distribution(0.0, 1.0);
    
    
    int limb_idx = 0;
    // Create horizontal limbs (along x direction)
    for (int i = 0; i < num_side_vertices; i++) { // i is y index
        for (int j = 0; j < num_side_vertices - 1; j++) { // j is x index
            Vector3d start_pos(step * j, step * i, 0.0);
            Vector3d end_pos(step * (j + 1), step * i, 0.0);

            // soft_robots->addLimb(start_pos, end_pos, n, density, radius, young_mod, poisson, mu);
            vector<Eigen::Vector3d> nodes;
            // Sinusoidal nodes along the limb, keeping nodes in z = 0 plane
            Vector3d dir = end_pos - start_pos;
            double length = dir.norm();
            Vector3d dir_norm = dir / length; // Normalized direction vector
            // Perpendicular vector in the z = 0 plane
            Vector3d perp(0,-1,0);

            for (int k = 0; k < n; k++) {
                double t = static_cast<double>(k) / (n - 1); // Parameter from 0 to 1
                Vector3d point = start_pos + dir * t;
                double displacement = amplitude * sin(2 * M_PI * t); // sinusoidal snake                
                point += perp * displacement; // Apply displacement perpendicular to the limb in z = 0 plane
                // point.z() += distribution(generator) * 0.1; // Add random perturbation in z direction
                nodes.push_back(point);
            }
            soft_robots->addLimb(nodes, density, radius, young_mod, poisson, mu);

            int start_node_idx = i * num_side_vertices + j;
            int end_node_idx = i * num_side_vertices + j + 1;

            limb_start_node[limb_idx] = start_node_idx;
            limb_end_node[limb_idx] = end_node_idx;

            // Add to node_to_limbs mapping
            node_to_limbs[start_node_idx].push_back(make_pair(limb_idx, 0)); // Start node
            node_to_limbs[end_node_idx].push_back(make_pair(limb_idx, -1)); // End node (-1 indicates last node)

            limb_idx++;
        }
    }

    // Create vertical limbs (along y direction)
    for (int i = 0; i < num_side_vertices - 1; i++) { // i is y index
        for (int j = 0; j < num_side_vertices; j++) { // j is x index
            Vector3d start_pos(step * j, step * i, 0.0);
            Vector3d end_pos(step * j, step * (i + 1), 0.0);

            // soft_robots->addLimb(start_pos, end_pos, n, density, radius, young_mod, poisson, mu);
            vector<Eigen::Vector3d> nodes;

            // Sinusoidal nodes along the limb, keeping nodes in z = 0 plane
            Vector3d dir = end_pos - start_pos;
            double length = dir.norm();
            Vector3d dir_norm = dir / length; // Normalized direction vector

            // Perpendicular vector in the z = 0 plane
            Vector3d perp(1,0,0);
            Vector3d out_of_plane(0,0,1);

            for (int k = 0; k < n; k++) {
                double t = static_cast<double>(k) / (n - 1); // Parameter from 0 to 1
                Vector3d point = start_pos + dir * t;
                double displacement = amplitude * sin(2 * M_PI * t);
                // double displacement =  sqrt(abs(1/4 - (t-1/2) * (t-1/2))) * amplitude; // semicircle snake
                point += perp * displacement; // Apply displacement perpendicular to the limb in z = 0 plane
                // point += out_of_plane * distribution(generator) * 0.1; // Add random perturbation in z direction
                nodes.push_back(point);
            }

            soft_robots->addLimb(nodes, density, radius, young_mod, poisson, mu);

            int start_node_idx = i * num_side_vertices + j;
            int end_node_idx = (i + 1) * num_side_vertices + j;

            limb_start_node[limb_idx] = start_node_idx;
            limb_end_node[limb_idx] = end_node_idx;

            // Add to node_to_limbs mapping
            node_to_limbs[start_node_idx].push_back(make_pair(limb_idx, 0)); // Start node
            node_to_limbs[end_node_idx].push_back(make_pair(limb_idx, -1)); // End node

            limb_idx++;
        }
    }

    // Assign limbs to joints - one stroke drawing
    for (int node_idx = 0; node_idx < num_nodes; node_idx++) {
        if (node_to_limbs[node_idx].empty()) {
            continue; // No limbs at this node
        }

        // Find the first limb that hasn't been added to a joint
        int first_limb_idx = -1;
        int first_m_node_idx = -1;
        for (const auto& limb_info : node_to_limbs[node_idx]) {
            int _limb_idx = limb_info.first;
            int _m_node_idx = limb_info.second;

            if (_limb_idx == first_limb_idx) {
                continue; // Limb already added to a joint
            }

            first_limb_idx = _limb_idx;
            first_m_node_idx = _m_node_idx;
            break;
        }

        if (first_limb_idx == -1) {
            continue; // No available limb to create joint
        }

        soft_robots->createJoint(first_limb_idx, first_m_node_idx);

        // Add other limbs to this joint if they haven't been added elsewhere
        for (const auto& limb_info : node_to_limbs[node_idx]) {
            int _limb_idx = limb_info.first;
            int _m_node_idx = limb_info.second;

            if (_limb_idx == first_limb_idx) {
                continue; // Limb already added to a joint
            }

            soft_robots->addToJoint(node_idx, _limb_idx, _m_node_idx);
        }
    }
    // This has to be called after all joints are declared
    soft_robots->setup();
    int num_joints = soft_robots->joints.size(); // Get the number of joints after setup
    
    // print out joints into a file
    string joint_file = "joints_info.txt"; // Set the file name for logging joint information   
    // Log joint information to file
    ofstream joint_log(joint_file);
    if (!joint_log.is_open()) {
        cerr << "Error opening joint log file: " << joint_file << endl;
        return;
    }
    joint_log << "Total number of joints: " << num_joints << endl;
    for (int i = 0; i < num_joints; ++i) {
        auto joint = soft_robots->joints[i];
        joint_log << "Joint " << i << ": Limb " << joint->joint_limb
                  << ", m_node_idx " << joint->joint_node
                  << ", connected limbs: ";
        joint_log << endl;
    }
    joint_log.close();

    std::vector<int> expand_edge_indices; // Expand edges: inside edges
    std::vector<int> shrink_edge_indices; // Shrink edges: outside edges
    
    // 5,6,9,10,13,14,26,27,28,31,32,33
    expand_edge_indices = {5,6,9,10,13,14,26,27,28,31,32,33};
    shrink_edge_indices = {0,1,2,3,4,7,8,11,12,15,16,17,18,19,20,21,22,23,24,25,34,35,36,37,38,39};
    std::vector<int> lower_left = {0,20};
    std::vector<int> upper_right = {39,19}; // 39 is the last node in the grid, 19 is the last node in the first row
    std::vector<int> lower_right = {3,24};
    std::vector<int> upper_left = {35,16}; // 36 is the last node in the grid, 15 is the last node in the first row

    // left side
    std::vector<int> left_side = {25,30};
    // right side
    std::vector<int> right_side = {29,34};
    // top side
    std::vector<int> top_side = {17,18};
    // bottom side
    std::vector<int> bottom_side = {1,2};
    

    // initial velocity vector; size: num_side_vertices    
    vector<Vector3d> positive_velocities;
    for (int i = 0; i < n; ++i) {
        Vector3d velocity(0.0, 0.0, velocity_amplitude);
        positive_velocities.push_back(velocity);
    }

    vector<Vector3d> negative_velocities;
    for (int i = 0; i < n; ++i) {
        Vector3d velocity(0.0, 0.0, -velocity_amplitude);
        negative_velocities.push_back(velocity);
    }

    // for sphere
    if (stretch > 0.0) {
        for (int i = 0; i < limb_idx; ++i) {
            // if i in expand_edge_indices
            if (std::find(expand_edge_indices.begin(), expand_edge_indices.end(), i) != expand_edge_indices.end()) {
                // Set velocity for the limb
                soft_robots->applyInitialVelocities(i, positive_velocities);
            } else if (std::find(shrink_edge_indices.begin(), shrink_edge_indices.end(), i) != shrink_edge_indices.end()) {            
                soft_robots->applyInitialVelocities(i, negative_velocities);
            }
        }
    } else {
        // for saddle

            for (int i = 0; i < limb_idx; ++i) {
            // if i in expand_edge_indices
            if (std::find(expand_edge_indices.begin(), expand_edge_indices.end(), i) != expand_edge_indices.end()) {
                // Set velocity for the limb
                // soft_robots->applyInitialVelocities(i, positive_velocities);
            } else if (std::find(shrink_edge_indices.begin(), shrink_edge_indices.end(), i) != shrink_edge_indices.end()) {
                // Set velocity for the limb
                // soft_robots->applyInitialVelocities(i, negative_velocities);

                // left side
                if (std::find(left_side.begin(), left_side.end(), i) != left_side.end()) {
                    soft_robots->applyInitialVelocities(i, negative_velocities);
                }
                // right side
                else if (std::find(right_side.begin(), right_side.end(), i) != right_side.end()) {
                    soft_robots->applyInitialVelocities(i, negative_velocities);
                }
                // top side
                else if (std::find(top_side.begin(), top_side.end(), i) != top_side.end()) {
                    soft_robots->applyInitialVelocities(i, positive_velocities);
                }
                // bottom side
                else if (std::find(bottom_side.begin(), bottom_side.end(), i) != bottom_side.end()) {
                    soft_robots->applyInitialVelocities(i, positive_velocities);
                }
            }
        }


        // for symmetric saddle
        // for (int i = 0; i < limb_idx; ++i) {
        //     // if i in expand_edge_indices
        //     if (std::find(expand_edge_indices.begin(), expand_edge_indices.end(), i) != expand_edge_indices.end()) {
        //         // Set velocity for the limb
        //         // soft_robots->applyInitialVelocities(i, positive_velocities);
        //     } else if (std::find(shrink_edge_indices.begin(), shrink_edge_indices.end(), i) != shrink_edge_indices.end()) {
        //         // Set velocity for the limb
        //         // soft_robots->applyInitialVelocities(i, negative_velocities);

        //         // lower left
        //         if (std::find(lower_left.begin(), lower_left.end(), i) != lower_left.end()) {
        //             soft_robots->applyInitialVelocities(i, negative_velocities);
        //         }
        //         // upper right
        //         else if (std::find(upper_right.begin(), upper_right.end(), i) != upper_right.end()) {
        //             soft_robots->applyInitialVelocities(i, negative_velocities);
        //         }
        //         // lower right
        //         else if (std::find(lower_right.begin(), lower_right.end(), i) != lower_right.end()) {
        //             soft_robots->applyInitialVelocities(i, positive_velocities);
        //         }
        //         // upper left
        //         else if (std::find(upper_left.begin(), upper_left.end(), i) != upper_left.end()) {
        //             soft_robots->applyInitialVelocities(i, positive_velocities);
        //         } // else zero
        //     }
        // }


    }
    
    // Use unordered_set to filter out shrink edges from expand_edge_indices
    std::unordered_set<int> shrink_edge_set(shrink_edge_indices.begin(), shrink_edge_indices.end());

    // Filter out shrink edges from expand_edge_indices
    expand_edge_indices.erase(
        std::remove_if(expand_edge_indices.begin(), expand_edge_indices.end(),
                    [&shrink_edge_set](int idx) { return shrink_edge_set.count(idx) > 0; }),
        expand_edge_indices.end()
    );

    // Remove duplicates from both vectors
    auto remove_duplicates = [](std::vector<int>& vec) {
        std::sort(vec.begin(), vec.end());
        vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
    };

    remove_duplicates(expand_edge_indices);
    remove_duplicates(shrink_edge_indices); // do we need this...?

    
    forces->addForce(make_shared<dampingForce>(soft_robots, viscosity));

    forces->addForce(make_shared<floorContactForce>(soft_robots, delta, k_scaler, nu, floor_z, mu));

    // gravity
    Vector3d gravity_vector(0.0, 0.0, -gravity_magnitude); // Gravity acts in the negative z-direction
    forces->addForce(make_shared<gravityForce>(soft_robots, gravity_vector));
    
    double start_time = 0.0;
    double end_time = 500;    
    soft_robots->addController(make_shared<snakeLatticeController>(soft_robots, expand_edge_indices, shrink_edge_indices, start_time, end_time, stretch));

    // TODO: floor and gravity - to think about sagging.

    string logfile_base = "log_files";
    int logging_period = sim_time/dt/num_frames;
    logger = make_shared<rodNodeLogger>(logfile_base, logging_output_file, logging_period);
}
